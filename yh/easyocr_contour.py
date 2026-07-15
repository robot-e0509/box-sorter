"""
EasyOCR 문자 인식 + 글자→라벨 외곽 승격 + 같은 높이(base Z) + base 좌표

처리 순서
  1) OCR로 글자 검출·검증 (deskew + 다각도)
  2) 글자 박스를 포함하는 흰 라벨/물체 외곽 사각형으로 승격
  3) 근접·겹친 최종 외곽 병합
  4) cam2base base-Z 같은 높이 클러스터만 최종 선택·표시

실행
  python3 easyocr_contour.py
  python3 easyocr_contour.py /path/to/image.jpg
"""

from __future__ import annotations

import base64
import json
import re
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
LANGS = ["ko", "en"]
MIN_CONF = 0.30          # EasyOCR 후보 하한 default 0.35
MIN_KEEP_CONF = 0.35     # 최종 채택 conf (이하면 글자 없음으로 버림) default 0.5
USE_GPU = True
WINDOW_NAME = "EasyOCR Contour"
SAVE_DIR = Path(__file__).resolve().parent / "ocr_out"
CALIB_NPZ = Path(__file__).resolve().parent / "hand_eye_data" / "cam2base.npz"
CALIB_JSON = Path(__file__).resolve().parent / "hand_eye_data" / "cam2base.json"

# 윤곽선 면적 / 전체 이미지 면적
MIN_AREA_RATIO = 0.0004 #default 0.0008
MAX_AREA_RATIO = 0.40
DRAW_MAX_AREA_RATIO = 0.35
# 그려질/승격 가능 외곽: 원본 기준 가로·세로 상한 (라벨 전체)
MAX_SIDE_PX = 220
# 글자 박스만 이보다 작으면 외곽 승격 시도
MIN_PROMOTE_AREA_RATIO = 1.25  # 외곽면적 >= 글자박스면적 * 이 값
MAX_CANDIDATES = 80 #default 50
DESKEW_PAD = 8
# 흰 라벨 flood 시 배경 대비 (밝기)
LABEL_BRIGHT_LO = 40   # seed보다 이만큼 어두워도 포함
LABEL_BRIGHT_UP = 80
LABEL_MIN_MEAN = 140   # 흰 라벨로 볼 최소 평균 밝기

# 글자 인식 후: cam2base base-Z 같은 높이대 (mm) — median ± 허용치
SAME_HEIGHT_TOL_MM = 40.0
# deskew 후 OCR에 추가 시험할 회전각 (도)
OCR_EXTRA_ANGLES_DEG = (0, 45, 90, 135, 180, 225, 270, 315)
DEPTH_SAMPLE_R = 3
# 같은 물체로 볼 중심 거리 (작을수록 더 많이 분리 유지)
DUP_CENTER_PX = 18
# IoU 이하면 서로 다른 객체로 유지
DUP_IOU_MAX = 0.35
# 최종 외곽: 겹치거나 거의 붙어 있으면 하나로 합침
MERGE_NEAR_GAP_PX = 22          # AABB 간격(px) 이하 → 합침
MERGE_NEAR_IOU = 0.02           # IoU 이상 → 합침
MAX_SIDE_PX_MERGED = 400        # 합친 외곽 허용 한도


def create_reader():
    import easyocr

    print(f"EasyOCR 로딩 중... langs={LANGS}, gpu={USE_GPU}")
    reader = easyocr.Reader(LANGS, gpu=USE_GPU)
    print("EasyOCR 준비 완료")
    return reader


_TEXT_CHAR_RE = re.compile(r"[0-9A-Za-z가-힣]")
_NOISE_ONLY_RE = re.compile(r"^[\W_]+$", re.UNICODE)


def is_plausible_ocr_text(text: str) -> bool:
    """실제 글자(숫자/영문/한글)가 포함된 인식만 허용."""
    t = (text or "").strip()
    if len(t) < 1:
        return False
    if _NOISE_ONLY_RE.match(t):
        return False
    if not _TEXT_CHAR_RE.search(t):
        return False
    # 흔한 1글자 오인
    if len(t) == 1 and t in "|Il!j;:`~'\",.":
        return False
    return True


def filter_ocr_hits(hits: list[tuple[str, float]]) -> list[tuple[str, float]]:
    """conf·텍스트 품질로 OCR 히트 필터."""
    out = []
    for text, conf in hits:
        conf = float(conf)
        text = str(text).strip()
        if conf < MIN_KEEP_CONF:
            continue
        if not is_plausible_ocr_text(text):
            continue
        out.append((text, conf))
    return out


def load_cam2base() -> tuple[np.ndarray, np.ndarray, dict]:
    """eye-to-hand: P_base = R @ P_cam + t  (단위 m)."""
    meta: dict = {}
    if CALIB_NPZ.is_file():
        data = np.load(CALIB_NPZ)
        R = np.array(data["R_cam2base"], dtype=np.float64)
        t = np.array(data["t_cam2base_m"], dtype=np.float64).reshape(3)
        print(f"캘리브 로드: {CALIB_NPZ}")
        if CALIB_JSON.is_file():
            meta = json.loads(CALIB_JSON.read_text(encoding="utf-8"))
        return R, t, meta

    if CALIB_JSON.is_file():
        meta = json.loads(CALIB_JSON.read_text(encoding="utf-8"))
        R = np.array(meta["R_cam2base"], dtype=np.float64)
        t = np.array(meta["t_cam2base_m"], dtype=np.float64).reshape(3)
        print(f"캘리브 로드: {CALIB_JSON} (npz 없으면 json 사용)")
        return R, t, meta

    raise FileNotFoundError(
        f"캘리브 없음:\n  {CALIB_NPZ}\n  {CALIB_JSON}\n"
        "먼저 hand_eye_calib.py 로 캘리브를 완료하세요."
    )


def cam_to_base(P_cam_m: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return R @ P_cam_m.reshape(3) + t.reshape(3)


def deproject_pixel(
    u: float, v: float, depth_m: float, fx: float, fy: float, ppx: float, ppy: float
) -> np.ndarray:
    """픽셀+깊이 → 카메라 좌표 [X,Y,Z] (m)."""
    x = (u - ppx) / fx * depth_m
    y = (v - ppy) / fy * depth_m
    return np.array([x, y, depth_m], dtype=np.float64)


def sample_depth_m(
    depth_m: np.ndarray, u: float, v: float, radius: int = DEPTH_SAMPLE_R
) -> float:
    """유효 depth 중앙값 (m). 없으면 0."""
    h, w = depth_m.shape[:2]
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return 0.0
    x0 = max(0, ui - radius)
    x1 = min(w, ui + radius + 1)
    y0 = max(0, vi - radius)
    y1 = min(h, vi + radius + 1)
    patch = depth_m[y0:y1, x0:x1].reshape(-1)
    valid = patch[patch > 0.05]
    if valid.size == 0:
        return 0.0
    return float(np.median(valid))


def sample_depth_in_quad(
    depth_m: np.ndarray, box_pts: np.ndarray, fallback_uv: tuple[float, float]
) -> float:
    """사각형 내부 유효 depth 중앙값. 실패 시 중심 근처 샘플."""
    h, w = depth_m.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    pts = np.round(box_pts.reshape(-1, 2)).astype(np.int32)
    cv2.fillConvexPoly(mask, pts, 1)
    vals = depth_m[mask > 0]
    valid = vals[vals > 0.05]
    if valid.size >= 5:
        return float(np.median(valid))
    return sample_depth_m(depth_m, fallback_uv[0], fallback_uv[1])


def pixel_to_base_mm(
    u: float,
    v: float,
    depth_m_img: np.ndarray,
    intr: dict,
    R: np.ndarray,
    t: np.ndarray,
    depth_override: float | None = None,
) -> tuple[np.ndarray | None, float]:
    """픽셀 → base [mm]. 반환: (base_mm[3] or None, depth_m)"""
    z = float(depth_override) if depth_override is not None else sample_depth_m(
        depth_m_img, u, v
    )
    if z <= 0:
        return None, 0.0
    P_cam = deproject_pixel(u, v, z, intr["fx"], intr["fy"], intr["ppx"], intr["ppy"])
    P_base_mm = cam_to_base(P_cam, R, t) * 1000.0
    return P_base_mm, z


def quad_size_mm_in_base(
    box_pts: np.ndarray,
    depth_m_img: np.ndarray,
    intr: dict,
    R: np.ndarray,
    t: np.ndarray,
) -> tuple[float, float] | None:
    """원본 4점을 base로 올려 인접 변 길이(mm). 카메라 기울기 반영."""
    pts = box_pts.reshape(-1, 2)
    if pts.shape[0] != 4:
        return None
    bases = []
    for u, v in pts:
        Pb, _ = pixel_to_base_mm(float(u), float(v), depth_m_img, intr, R, t)
        if Pb is None:
            return None
        bases.append(Pb)
    bases = np.asarray(bases)
    d01 = float(np.linalg.norm(bases[1] - bases[0]))
    d12 = float(np.linalg.norm(bases[2] - bases[1]))
    width_mm, height_mm = (d01, d12) if d01 >= d12 else (d12, d01)
    return width_mm, height_mm


def make_binary_masks(gray: np.ndarray) -> list[np.ndarray]:
    """물체 분리 유지: morph를 약하게 (합쳐진 '객체군' 방지)."""
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    masks = []
    _, th_inv = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    masks.extend([th_inv, th])
    adapt = cv2.adaptiveThreshold(
        blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 5
    )
    masks.append(adapt)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    return [cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel, iterations=1) for m in masks]


def normalize_min_area_rect(rect):
    (cx, cy), (rw, rh), angle = rect
    rw, rh = float(rw), float(rh)
    angle = float(angle)
    if rw < rh:
        rw, rh = rh, rw
        angle += 90.0
    while angle < 0:
        angle += 180.0
    while angle >= 180.0:
        angle -= 180.0
    return (float(cx), float(cy)), (rw, rh), angle


def quad_iou(box_a: np.ndarray, box_b: np.ndarray) -> float:
    """두 사각형(4점) IoU (축정렬 AABB 근사)."""
    a = box_a.reshape(-1, 2)
    b = box_b.reshape(-1, 2)
    ax0, ay0 = float(a[:, 0].min()), float(a[:, 1].min())
    ax1, ay1 = float(a[:, 0].max()), float(a[:, 1].max())
    bx0, by0 = float(b[:, 0].min()), float(b[:, 1].min())
    bx1, by1 = float(b[:, 0].max()), float(b[:, 1].max())
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter
    return float(inter / union) if union > 0 else 0.0


def point_in_quad(pt: tuple[float, float], box_pts: np.ndarray) -> bool:
    pts = box_pts.reshape(-1, 2).astype(np.float32)
    return cv2.pointPolygonTest(pts, (float(pt[0]), float(pt[1])), False) >= 0


def quad_contains_points(outer_pts: np.ndarray, points: np.ndarray, margin: float = 1.0) -> bool:
    """outer 사각형이 points의 모든 점을 포함하면 True (여유 margin px)."""
    outer = outer_pts.reshape(-1, 2).astype(np.float32)
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if pts.size == 0:
        return False
    if margin > 0 and outer.shape[0] >= 4:
        c = outer.mean(axis=0)
        outer_exp = c + (outer - c) * (
            1.0 + margin / max(float(np.linalg.norm(outer[0] - c)), 1.0)
        )
    else:
        outer_exp = outer
    for p in pts:
        if cv2.pointPolygonTest(outer_exp, (float(p[0]), float(p[1])), False) < 0:
            return False
    return True


def quad_contains_text_box(outer_pts: np.ndarray, text_box_pts: np.ndarray) -> bool:
    """외곽이 글자 박스(네 꼭짓점+중심)를 모두 포함해야 함."""
    tb = np.asarray(text_box_pts, dtype=np.float32).reshape(-1, 2)
    if tb.shape[0] < 1:
        return False
    center = tb.mean(axis=0, keepdims=True)
    pts = np.vstack([tb, center])
    return quad_contains_points(outer_pts, pts, margin=2.0)


def cand_from_rect(
    raw_rect, area: float, img_area: float, *, source: str = "contour"
) -> dict | None:
    (cx, cy), (rw, rh), angle = normalize_min_area_rect(raw_rect)
    if rw < 5 or rh < 5:
        return None
    ratio = area / img_area if img_area > 0 else 0.0
    quad = np.round(cv2.boxPoints(raw_rect)).astype(np.int32).reshape(-1, 1, 2)
    box_pts = quad.reshape(-1, 2).astype(np.float32)
    return {
        "contour": quad,
        "area": float(area),
        "area_ratio": float(ratio),
        "center": (float(cx), float(cy)),
        "width": float(rw),
        "height": float(rh),
        "angle": float(angle),
        "box_pts": box_pts,
        "rect": ((cx, cy), (rw, rh), angle),
        "source": source,
    }


def merge_or_append_candidate(seen: list[dict], cand: dict) -> None:
    """가까우면서 IoU 높은 것만 합침. 글자 시드가 있으면 시드 정보 보존하며 큰 박스 선호."""
    cx, cy = cand["center"]
    for prev in seen:
        pcx, pcy = prev["center"]
        dist = ((pcx - cx) ** 2 + (pcy - cy) ** 2) ** 0.5
        iou = quad_iou(prev["box_pts"], cand["box_pts"])
        if dist < DUP_CENTER_PX or iou > DUP_IOU_MAX:
            # 글자 시드 유지한 채 geometry는 더 큰 쪽
            seed_text = prev.get("seed_text") or cand.get("seed_text")
            seed_conf = prev.get("seed_conf") or cand.get("seed_conf")
            text_center = prev.get("text_center") or cand.get("text_center")
            if cand["area"] >= prev["area"]:
                prev.update(cand)
            if seed_text:
                prev["seed_text"] = seed_text
                prev["seed_conf"] = seed_conf
            if text_center:
                prev["text_center"] = text_center
            return
    seen.append(cand)


def suppress_huge_parents(cands: list[dict]) -> list[dict]:
    """
    MAX_SIDE 를 크게 넘는 포함 외곽(배경/가방 전체)만 제거하고,
    라벨 크기 외곽은 남긴다.
    """
    if len(cands) < 2:
        return cands
    drop = set()
    for i, big in enumerate(cands):
        too_big = (
            big["width"] > MAX_SIDE_PX * 1.2
            or big["height"] > MAX_SIDE_PX * 1.2
            or big["area_ratio"] > DRAW_MAX_AREA_RATIO
        )
        if not too_big:
            continue
        for j, small in enumerate(cands):
            if i == j:
                continue
            if small["area"] >= big["area"] * 0.5:
                continue
            if point_in_quad(small["center"], big["box_pts"]):
                drop.add(i)
                break
    kept = [c for i, c in enumerate(cands) if i not in drop]
    if drop:
        print(f"과대 포함 외곽 제거: {len(drop)}개 → {len(kept)}개")
    return kept


def prefer_outer_over_text_boxes(items: list[dict]) -> list[dict]:
    """
    최종 결과: 작은 글자 박스가 큰 라벨 외곽 안에 있으면 작은 쪽을 버리고 외곽을 남김.
    """
    if len(items) < 2:
        return items
    drop = set()
    for i, small in enumerate(items):
        for j, big in enumerate(items):
            if i == j or j in drop:
                continue
            if big["area"] < small["area"] * MIN_PROMOTE_AREA_RATIO:
                continue
            if point_in_quad(small["center"], big["box_pts"]):
                # 텍스트가 같거나 겹치면 작은 글자박스 제거
                drop.add(i)
                break
    kept = [c for i, c in enumerate(items) if i not in drop]
    if drop:
        print(f"글자박스→라벨외곽 통합: {len(drop)}개 글자박스 제거, {len(kept)}개 유지")
    return kept


def _aabb_of_box(box_pts: np.ndarray) -> tuple[float, float, float, float]:
    p = np.asarray(box_pts, dtype=np.float64).reshape(-1, 2)
    return (
        float(p[:, 0].min()),
        float(p[:, 1].min()),
        float(p[:, 0].max()),
        float(p[:, 1].max()),
    )


def _aabb_edge_gap(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> float:
    """두 AABB 사이 간격(겹치면 0)."""
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    dx = max(0.0, max(bx0 - ax1, ax0 - bx1))
    dy = max(0.0, max(by0 - ay1, ay0 - by1))
    if dx <= 0.0 and dy <= 0.0:
        return 0.0
    if dx <= 0.0:
        return float(dy)
    if dy <= 0.0:
        return float(dx)
    return float((dx * dx + dy * dy) ** 0.5)


def _outer_boxes_near(a: dict, b: dict, gap_px: float, iou_thr: float) -> bool:
    if quad_iou(a["box_pts"], b["box_pts"]) >= iou_thr:
        return True
    gap = _aabb_edge_gap(_aabb_of_box(a["box_pts"]), _aabb_of_box(b["box_pts"]))
    if gap <= gap_px:
        return True
    # 글자 간격이 좁은 경우: 중심 거리 ≈ (반폭합)+gap
    acx, acy = float(a["center"][0]), float(a["center"][1])
    bcx, bcy = float(b["center"][0]), float(b["center"][1])
    dist = ((acx - bcx) ** 2 + (acy - bcy) ** 2) ** 0.5
    span = 0.5 * (float(a["width"]) + float(a["height"]) + float(b["width"]) + float(b["height"]))
    return dist <= span * 0.55 + gap_px


def _merge_two_text_objects(a: dict, b: dict, img_area: float) -> dict:
    """두 최종 외곽을 minAreaRect 합집합 + 글자 텍스트 병합."""
    pts_list = [
        np.asarray(a["box_pts"], dtype=np.float32).reshape(-1, 2),
        np.asarray(b["box_pts"], dtype=np.float32).reshape(-1, 2),
    ]
    for src in (a, b):
        cnt = src.get("contour")
        if cnt is not None:
            try:
                pts_list.append(np.asarray(cnt, dtype=np.float32).reshape(-1, 2))
            except Exception:
                pass
    pts = np.vstack(pts_list).astype(np.float32)
    raw = cv2.minAreaRect(pts)
    (_c, (rw, rh), _ang) = normalize_min_area_rect(raw)
    area = float(max(rw, 1.0) * max(rh, 1.0))
    geom = cand_from_rect(raw, area, img_area, source="merged")

    texts: list[str] = []
    for src in (a, b):
        if src.get("texts"):
            texts.extend([str(t) for t in src["texts"] if str(t).strip()])
        elif src.get("text"):
            texts.append(str(src["text"]).strip())
    uniq: list[str] = []
    for t in texts:
        if t and t not in uniq:
            uniq.append(t)
    joined = (
        " ".join(uniq)
        if uniq
        else (str(a.get("text", "")) + " " + str(b.get("text", ""))).strip()
    )

    out = dict(a)
    out.update(
        {
            "text": joined,
            "texts": uniq if uniq else [joined],
            "conf": float(max(float(a.get("conf", 0)), float(b.get("conf", 0)))),
            "contour": pts.reshape(-1, 1, 2),
            "center": geom["center"],
            "width": geom["width"],
            "height": geom["height"],
            "angle": geom["angle"],
            "area": geom["area"],
            "area_ratio": geom["area_ratio"],
            "box_pts": geom["box_pts"],
            "source": "merged",
            "merged": True,
            "base_mm": None,
            "size_mm": None,
        }
    )
    # 합친 뒤에도 OCR 정자각·글자박스 유지 (conf 높은 쪽)
    prefer = a if float(a.get("conf", 0)) >= float(b.get("conf", 0)) else b
    for k in ("angle", "base_angle", "extra_rot", "text_box_pts"):
        if prefer.get(k) is not None:
            out[k] = prefer[k]
        elif a.get(k) is not None:
            out[k] = a[k]
        elif b.get(k) is not None:
            out[k] = b[k]
    return out


def merge_near_text_objects(
    items: list[dict],
    *,
    gap_px: float = MERGE_NEAR_GAP_PX,
    iou_thr: float = MERGE_NEAR_IOU,
    img_wh: tuple[int, int] | None = None,
) -> list[dict]:
    """
    글자 간격이 좁거나 최종 외곽이 겹치/거의 붙어 있으면 하나로 합침.
    """
    if len(items) < 2:
        return items
    if img_wh is not None:
        img_area = float(img_wh[0] * img_wh[1])
    else:
        img_area = 1.0
        for it in items:
            ar = float(it.get("area_ratio") or 0.0)
            area = float(it.get("area") or 0.0)
            if ar > 1e-12:
                img_area = max(img_area, area / ar)

    cluster = list(items)
    before = len(cluster)
    changed = True
    guard = 0
    while changed and guard < 64:
        guard += 1
        changed = False
        n = len(cluster)
        i = 0
        while i < n:
            j = i + 1
            merged_i = False
            while j < n:
                if not _outer_boxes_near(cluster[i], cluster[j], gap_px, iou_thr):
                    j += 1
                    continue
                cand = _merge_two_text_objects(cluster[i], cluster[j], img_area)
                if (
                    float(cand.get("width", 0)) > MAX_SIDE_PX_MERGED
                    or float(cand.get("height", 0)) > MAX_SIDE_PX_MERGED
                ):
                    j += 1
                    continue
                cluster[i] = cand
                del cluster[j]
                n = len(cluster)
                changed = True
                merged_i = True
                break
            if not merged_i:
                i += 1

    if len(cluster) < before:
        print(f"근접 외곽 병합: {before} → {len(cluster)}개")
        for r in cluster:
            if r.get("merged"):
                print(
                    f"  merged text='{r.get('text')}' "
                    f"size={r['width']:.0f}x{r['height']:.0f}px"
                )
    return cluster


def detect_object_candidates(bgr: np.ndarray) -> list[dict]:
    h, w = bgr.shape[:2]
    img_area = float(h * w)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    seen: list[dict] = []
    for mask in make_binary_masks(gray):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area <= 0:
                continue
            ratio = area / img_area
            if ratio < MIN_AREA_RATIO or ratio > MAX_AREA_RATIO:
                continue
            raw_rect = cv2.minAreaRect(cnt)
            cand = cand_from_rect(raw_rect, area, img_area, source="contour")
            if cand is None:
                continue
            merge_or_append_candidate(seen, cand)

    seen = suppress_huge_parents(seen)
    seen.sort(key=lambda d: d["area"], reverse=True)
    return seen[:MAX_CANDIDATES]


def candidates_from_full_ocr(reader, bgr: np.ndarray) -> list[dict]:
    """
    전체 프레임 EasyOCR로 글자 박스를 여러 개 후보로 추가.
    (윤곽이 하나로 합쳐져도 글자별 객체 분리)
    """
    h, w = bgr.shape[:2]
    img_area = float(h * w)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    try:
        raw = reader.readtext(rgb, detail=1, paragraph=False)
    except Exception as exc:
        print(f"전체 OCR 후보 실패: {exc}")
        return []

    out: list[dict] = []
    for bbox, text, conf in raw:
        conf = float(conf)
        text = str(text).strip()
        if conf < MIN_KEEP_CONF or not is_plausible_ocr_text(text):
            continue
        pts = np.array(bbox, dtype=np.float32).reshape(-1, 2)
        # 살짝 여유
        cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
        pts_exp = pts.copy()
        pts_exp[:, 0] = cx + (pts_exp[:, 0] - cx) * 1.15
        pts_exp[:, 1] = cy + (pts_exp[:, 1] - cy) * 1.15
        pts_exp[:, 0] = np.clip(pts_exp[:, 0], 0, w - 1)
        pts_exp[:, 1] = np.clip(pts_exp[:, 1], 0, h - 1)
        raw_rect = cv2.minAreaRect(pts_exp.reshape(-1, 1, 2))
        (rcx, rcy), (rw, rh), _ = raw_rect
        area = float(max(rw, 1.0) * max(rh, 1.0))
        if area / img_area < MIN_AREA_RATIO:
            continue
        cand = cand_from_rect(raw_rect, area, img_area, source="full_ocr")
        if cand is None:
            continue
        if cand["width"] > MAX_SIDE_PX * 1.5 or cand["height"] > MAX_SIDE_PX * 1.5:
            continue
        cand["seed_text"] = text
        cand["seed_conf"] = conf
        # 전체 OCR bbox 중심 (패치 OCR 검증·포함 여부용)
        cand["text_center"] = (cx, cy)
        out.append(cand)
    print(f"전체 OCR 글자 박스 후보: {len(out)}개")
    return out


def merge_candidate_lists(a: list[dict], b: list[dict]) -> list[dict]:
    seen: list[dict] = []
    for cand in list(a) + list(b):
        merge_or_append_candidate(seen, cand)
    seen = suppress_huge_parents(seen)
    seen.sort(key=lambda d: d["area"], reverse=True)
    return seen[:MAX_CANDIDATES]


def _side_ok(rw: float, rh: float) -> bool:
    return rw <= MAX_SIDE_PX and rh <= MAX_SIDE_PX and rw >= 5 and rh >= 5


def expand_bright_label_rect(
    bgr: np.ndarray,
    seed_uv: tuple[float, float],
    text_box_pts: np.ndarray,
) -> dict | None:
    """
    글자 주변의 밝은(흰 라벨) 연결성분을 찾아 minAreaRect 반환.
    검정 가방 위 흰 라벨에 특히 유효.
    """
    h, w = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    sx = int(np.clip(round(seed_uv[0]), 0, w - 1))
    sy = int(np.clip(round(seed_uv[1]), 0, h - 1))

    # 글자 박스 주변에서 밝은 시드(배경 흰부분) 찾기
    tb = text_box_pts.reshape(-1, 2)
    x0 = int(max(0, tb[:, 0].min() - 8))
    y0 = int(max(0, tb[:, 1].min() - 8))
    x1 = int(min(w - 1, tb[:, 0].max() + 8))
    y1 = int(min(h - 1, tb[:, 1].max() + 8))
    roi = gray[y0 : y1 + 1, x0 : x1 + 1]
    if roi.size == 0:
        return None
    # 상위 밝기 픽셀 평균을 시드 밝기로
    flat = roi.reshape(-1).astype(np.float32)
    thr_local = float(np.percentile(flat, 70))
    bright = flat[flat >= thr_local]
    if bright.size == 0:
        return None
    seed_val = float(np.median(bright))
    if seed_val < LABEL_MIN_MEAN:
        # 전체가 어두우면 Otsu 밝은쪽 시도
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        # seed 밝기 근처 + 절대 밝은 영역
        lo = max(0, int(seed_val - LABEL_BRIGHT_LO))
        th = cv2.inRange(gray, lo, 255)
        th = cv2.bitwise_or(th, cv2.inRange(gray, max(LABEL_MIN_MEAN, lo), 255))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, iterations=2)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=1)

    # 시드가 마스크 밖이면 근처 밝 점 탐색
    if th[sy, sx] == 0:
        found = False
        for r in range(1, 25):
            y_a, y_b = max(0, sy - r), min(h, sy + r + 1)
            x_a, x_b = max(0, sx - r), min(w, sx + r + 1)
            ys, xs = np.where(th[y_a:y_b, x_a:x_b] > 0)
            if len(xs) == 0:
                continue
            # 글자 중심에 가장 가까운 밝 점
            xs = xs + x_a
            ys = ys + y_a
            d2 = (xs - sx) ** 2 + (ys - sy) ** 2
            k = int(np.argmin(d2))
            sx, sy = int(xs[k]), int(ys[k])
            found = True
            break
        if not found:
            return None

    num, labels, stats, _ = cv2.connectedComponentsWithStats(th, connectivity=8)
    if num <= 1:
        return None
    lab = int(labels[sy, sx])
    if lab <= 0:
        return None
    area = int(stats[lab, cv2.CC_STAT_AREA])
    if area < 80:
        return None

    mask = (labels == lab).astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    raw_rect = cv2.minAreaRect(cnt)
    (cx, cy), (rw, rh), angle = normalize_min_area_rect(raw_rect)
    if not _side_ok(rw, rh):
        return None
    img_area = float(h * w)
    cand = cand_from_rect(raw_rect, float(rw * rh), img_area, source="bright_label")
    return cand


def pick_enclosing_contour(
    text_center: tuple[float, float],
    text_area: float,
    contour_cands: list[dict],
) -> dict | None:
    """글자 중심을 포함하는 윤곽 중, 글자보다 크고 MAX_SIDE 이하인 것(면적 최소=타이트한 라벨)."""
    best = None
    best_area = None
    for c in contour_cands:
        if not point_in_quad(text_center, c["box_pts"]):
            continue
        if not _side_ok(c["width"], c["height"]):
            continue
        if c["area"] < text_area * MIN_PROMOTE_AREA_RATIO:
            continue
        if c["area_ratio"] > DRAW_MAX_AREA_RATIO:
            continue
        if best is None or c["area"] < best_area:
            best = c
            best_area = c["area"]
    return best


def promote_text_to_outer_label(
    bgr: np.ndarray,
    obj: dict,
    contour_cands: list[dict],
) -> dict:
    """
    글자 확인된 객체를 흰 라벨/외곽 사각형으로 승격.
    실패 시 원본(글자 박스) 유지.
    """
    text_area = float(obj["width"] * obj["height"])
    seed = obj.get("text_center") or obj["center"]

    bright = expand_bright_label_rect(bgr, seed, obj["box_pts"])
    enclosing = pick_enclosing_contour(seed, text_area, contour_cands)

    chosen = None
    # 둘 다 있으면: 글자보다 확실히 크고, 면적이 더 작은(타이트) 쪽 우선하되
    # bright가 충분히 크면 bright 선호 (흰 라벨)
    candidates = []
    if bright is not None and bright["area"] >= text_area * MIN_PROMOTE_AREA_RATIO:
        candidates.append(bright)
    if enclosing is not None:
        candidates.append(enclosing)

    if candidates:
        # MAX_SIDE ok already. Prefer larger than text but not huge: smallest among valid
        # that cover text. If bright label exists prefer it when mean-ish larger.
        if bright is not None and bright in candidates:
            chosen = bright
        else:
            chosen = min(candidates, key=lambda d: d["area"])

    if chosen is None:
        return obj

    out = dict(obj)
    # 방향/정자각 계산용 — 승격 전 글자 박스 유지
    out["text_box_pts"] = np.asarray(obj["box_pts"], dtype=np.float32).copy()
    out["contour"] = chosen["contour"]
    out["box_pts"] = chosen["box_pts"]
    out["center"] = chosen["center"]
    out["width"] = chosen["width"]
    out["height"] = chosen["height"]
    out["area"] = chosen["area"]
    out["area_ratio"] = chosen["area_ratio"]
    out["rect"] = chosen["rect"]
    out["source"] = f"{obj.get('source', 'ocr')}+{chosen.get('source', 'outer')}"
    print(
        f"  승격: text_box={obj['width']:.0f}x{obj['height']:.0f} → "
        f"outer={chosen['width']:.0f}x{chosen['height']:.0f} "
        f"({chosen.get('source')})"
    )
    return out


def attach_base_coords(
    candidates: list[dict],
    depth_m_img: np.ndarray,
    intr: dict,
    R: np.ndarray,
    t: np.ndarray,
) -> list[dict]:
    out = []
    for cand in candidates:
        cx, cy = cand["center"]
        z = sample_depth_in_quad(depth_m_img, cand["box_pts"], (cx, cy))
        Pb, z = pixel_to_base_mm(
            cx, cy, depth_m_img, intr, R, t, depth_override=z if z > 0 else None
        )
        size_mm = None
        if Pb is not None:
            size_mm = quad_size_mm_in_base(cand["box_pts"], depth_m_img, intr, R, t)
        c = dict(cand)
        c["depth_m"] = z
        c["base_mm"] = Pb
        c["size_mm"] = size_mm
        out.append(c)
    return out


def filter_same_base_height(
    items: list[dict], tol_mm: float = SAME_HEIGHT_TOL_MM
) -> list[dict]:
    """
    OCR 결과 중 base-Z가 가장 많이 모인 높이대(클러스터)만 유지.
    → 같은 테이블 위 여러 객체를 함께 남김.
    """
    valid = [c for c in items if c.get("base_mm") is not None]
    if not valid:
        print("경고: 유효 base Z 없음 — 같은 높이 필터 생략")
        return items

    zs = np.array([float(c["base_mm"][2]) for c in valid], dtype=np.float64)
    best_idx: list[int] = []
    for i, z0 in enumerate(zs):
        idx = [j for j, zj in enumerate(zs) if abs(float(zj) - float(z0)) <= tol_mm]
        if len(idx) > len(best_idx):
            best_idx = idx
    z_ref = float(np.median(zs[best_idx]))
    kept = []
    for c in items:
        Pb = c.get("base_mm")
        if Pb is None:
            print(f"  skip(no base) text='{c.get('text', '')}' center={c['center']}")
            continue
        zb = float(Pb[2])
        if abs(zb - z_ref) > tol_mm:
            print(
                f"  skip(height) baseZ={zb:.1f}mm ref={z_ref:.1f}±{tol_mm:.0f} "
                f"depth={c.get('depth_m', 0):.3f}m text='{c.get('text', '')}'"
            )
            continue
        kept.append(c)
    print(
        f"같은 높이대(base Z≈{z_ref:.1f}mm ±{tol_mm:.0f}mm): "
        f"{len(kept)}/{len(items)} (클러스터 {len(best_idx)}개 기준)"
    )
    return kept


def deskew_patch(bgr: np.ndarray, rect) -> tuple[np.ndarray | None, float]:
    (cx, cy), (rw, rh), angle = normalize_min_area_rect(rect)

    h, w = bgr.shape[:2]
    M = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
    rotated = cv2.warpAffine(
        bgr, M, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE
    )

    out_w = int(np.ceil(rw)) + DESKEW_PAD * 2
    out_h = int(np.ceil(rh)) + DESKEW_PAD * 2
    if out_w < 8 or out_h < 8:
        return None, angle

    patch = cv2.getRectSubPix(rotated, (out_w, out_h), (cx, cy))
    if patch is None or patch.size == 0:
        return None, angle
    return patch, float(angle)


def _upscale_for_ocr(patch_bgr: np.ndarray) -> np.ndarray:
    h, w = patch_bgr.shape[:2]
    # default 64
    if max(h, w) < 50:
        scale = 64.0 / max(h, w)
        return cv2.resize(
            patch_bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC
        )
    return patch_bgr


def rotate_bgr(patch_bgr: np.ndarray, angle_deg: float) -> np.ndarray:
    """패치를 angle_deg(반시계)만큼 회전. 캔버스 확장."""
    a = float(angle_deg) % 360.0
    if abs(a) < 1e-6:
        return patch_bgr
    if abs(a - 90.0) < 1e-6:
        return cv2.rotate(patch_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if abs(a - 180.0) < 1e-6:
        return cv2.rotate(patch_bgr, cv2.ROTATE_180)
    if abs(a - 270.0) < 1e-6:
        return cv2.rotate(patch_bgr, cv2.ROTATE_90_CLOCKWISE)

    h, w = patch_bgr.shape[:2]
    center = (w * 0.5, h * 0.5)
    M = cv2.getRotationMatrix2D(center, a, 1.0)
    cos_a = abs(M[0, 0])
    sin_a = abs(M[0, 1])
    nw = int(h * sin_a + w * cos_a)
    nh = int(h * cos_a + w * sin_a)
    M[0, 2] += (nw - w) * 0.5
    M[1, 2] += (nh - h) * 0.5
    return cv2.warpAffine(
        patch_bgr, M, (nw, nh),
        flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE,
    )


def ocr_patch_once(reader, patch_bgr: np.ndarray) -> list[tuple[str, float]]:
    if patch_bgr is None or patch_bgr.size == 0:
        return []
    rgb = cv2.cvtColor(_upscale_for_ocr(patch_bgr), cv2.COLOR_BGR2RGB)
    raw = reader.readtext(rgb, detail=1, paragraph=False)
    out = []
    for _, text, conf in raw:
        conf = float(conf)
        text = str(text).strip()
        if conf < MIN_CONF or not is_plausible_ocr_text(text):
            continue
        out.append((text, conf))
    return out


def ocr_patch_multirot(
    reader, patch_bgr: np.ndarray
) -> tuple[list[tuple[str, float]], int]:
    """
    deskew 이후 여러 각도에서 OCR.
    - minAreaRect deskew가 이미 임의 기울기를 맞춤
    - 여기서는 글자 '위쪽' 방향을 45도 간격으로 시험
    """
    if patch_bgr is None or patch_bgr.size == 0:
        return [], 0

    best_hits: list[tuple[str, float]] = []
    best_score = -1.0
    best_extra = 0

    for extra in OCR_EXTRA_ANGLES_DEG:
        oriented = rotate_bgr(patch_bgr, float(extra))
        hits = ocr_patch_once(reader, oriented)
        if not hits:
            continue
        score = max(c for _, c in hits) + 0.02 * len(hits)
        if score > best_score:
            best_score = score
            best_hits = hits
            best_extra = int(extra)

    return best_hits, best_extra


def run_ocr_pipeline(
    reader,
    bgr: np.ndarray,
    *,
    depth_m_img: np.ndarray | None = None,
    intr: dict | None = None,
    R: np.ndarray | None = None,
    t: np.ndarray | None = None,
) -> list[dict]:
    """
    1) OCR로 글자 검출·검증
    2) 글자 박스 → 흰 라벨/외곽 사각형으로 승격
    3) cam2base base-Z로 같은 높이대만 최종 선택
    """
    # 승격용 윤곽 풀(글자 필터 전 전체 — 라벨 크기 후보 유지)
    contour_pool = detect_object_candidates(bgr)
    print(
        f"윤곽선 후보(풀): {len(contour_pool)}개 "
        f"(면적비율 {MIN_AREA_RATIO*100:.2f}% ~ {MAX_AREA_RATIO*100:.1f}%)"
    )
    ocr_cands = candidates_from_full_ocr(reader, bgr)
    text_centers = [
        c.get("text_center", c["center"]) for c in ocr_cands if c.get("seed_text")
    ]
    contour_cands = contour_pool
    if text_centers:
        filtered_contours = []
        for c in contour_pool:
            if any(point_in_quad(tc, c["box_pts"]) for tc in text_centers):
                filtered_contours.append(c)
            elif any(quad_iou(c["box_pts"], o["box_pts"]) > 0.15 for o in ocr_cands):
                filtered_contours.append(c)
        print(
            f"글자 연계 윤곽(OCR 후보용): {len(filtered_contours)}/{len(contour_pool)}"
        )
        contour_cands = filtered_contours
    else:
        print("전체 OCR 글자 없음 — 윤곽-only + 패치 OCR")

    # OCR 검증은 글자 박스 우선 (full_ocr) + 연계 윤곽
    candidates = merge_candidate_lists(ocr_cands, contour_cands)
    print(f"OCR 검증 후보: {len(candidates)}개")

    use_geo = (
        depth_m_img is not None
        and intr is not None
        and R is not None
        and t is not None
    )

    # --- 각도 보정 OCR → 글자 확인 ---
    text_objects: list[dict] = []
    for i, cand in enumerate(candidates):
        if cand["area_ratio"] > DRAW_MAX_AREA_RATIO:
            print(
                f"  candidate[{i}] skip(too large area) "
                f"ratio={cand['area_ratio']*100:.1f}%"
            )
            continue

        if cand["width"] > MAX_SIDE_PX or cand["height"] > MAX_SIDE_PX:
            print(
                f"  candidate[{i}] skip(size>{MAX_SIDE_PX}px) "
                f"size={cand['width']:.0f}x{cand['height']:.0f}px"
            )
            continue

        patch, base_angle = deskew_patch(bgr, cand["rect"])
        if patch is None:
            continue
        hits, extra_rot = ocr_patch_multirot(reader, patch)
        hits = filter_ocr_hits(hits)

        if not hits:
            if (
                cand.get("source") == "full_ocr"
                and cand.get("seed_text")
                and is_plausible_ocr_text(str(cand["seed_text"]))
                and float(cand.get("seed_conf", 0)) >= MIN_KEEP_CONF
            ):
                hits = [(str(cand["seed_text"]), float(cand["seed_conf"]))]
                extra_rot = 0
            else:
                print(
                    f"  candidate[{i}] skip(no verified text) "
                    f"src={cand.get('source', '?')} "
                    f"seed={cand.get('seed_text', '')!r}"
                )
                continue

        texts = [tx for tx, _ in hits]
        confs = [c for _, c in hits]
        best_conf = max(confs)
        joined = " ".join(texts)
        if best_conf < MIN_KEEP_CONF or not is_plausible_ocr_text(joined):
            print(f"  candidate[{i}] skip(weak text) '{joined}' conf={best_conf:.3f}")
            continue

        total_angle = (base_angle + extra_rot) % 360.0
        obj = {
            "text": joined,
            "texts": texts,
            "conf": best_conf,
            "contour": cand["contour"],
            "center": cand["center"],
            "width": cand["width"],
            "height": cand["height"],
            "angle": total_angle,
            "base_angle": base_angle,
            "extra_rot": extra_rot,
            "area": cand["area"],
            "area_ratio": cand["area_ratio"],
            "box_pts": cand["box_pts"],
            "depth_m": 0.0,
            "base_mm": None,
            "size_mm": None,
            "source": cand.get("source", "contour"),
            "text_center": cand.get("text_center", cand["center"]),
        }
        # 글자 박스 → 흰 라벨/외곽 승격
        obj = promote_text_to_outer_label(bgr, obj, contour_pool)
        text_objects.append(obj)
        print(
            f"  OCR[{i}] text='{joined}' conf={best_conf:.3f} "
            f"src={obj.get('source', '?')} "
            f"size={obj['width']:.0f}x{obj['height']:.0f}px "
            f"deskew={base_angle:.1f}° +{extra_rot}°"
        )

    # 작은 글자박스가 큰 라벨 안에 있으면 글자박스 제거
    text_objects = prefer_outer_over_text_boxes(text_objects)
    if len(text_objects) > 1:
        dedup: list[dict] = []
        for cand in sorted(text_objects, key=lambda d: d["conf"], reverse=True):
            clash = False
            for prev in dedup:
                dist = (
                    (prev["center"][0] - cand["center"][0]) ** 2
                    + (prev["center"][1] - cand["center"][1]) ** 2
                ) ** 0.5
                iou = quad_iou(prev["box_pts"], cand["box_pts"])
                # 거의 같은 라벨이면 conf 높은 쪽만
                if dist < DUP_CENTER_PX or iou > DUP_IOU_MAX:
                    clash = True
                    break
            if not clash:
                dedup.append(cand)
        text_objects = dedup

    # 글자 간격 좁음 / 외곽 겹침·밀착 → 하나로 합침
    h_img, w_img = bgr.shape[:2]
    text_objects = merge_near_text_objects(
        text_objects, img_wh=(w_img, h_img)
    )

    print(f"글자 포함 라벨 외곽: {len(text_objects)}개")

    # --- depth+cam2base → 같은 base-Z 높이대 ---
    if use_geo and text_objects:
        text_objects = attach_base_coords(text_objects, depth_m_img, intr, R, t)
        text_objects = filter_same_base_height(text_objects, SAME_HEIGHT_TOL_MM)
        for r in text_objects:
            Pb = r.get("base_mm")
            sm = r.get("size_mm")
            extra = ""
            if Pb is not None:
                extra += f"  base[mm]=({Pb[0]:.1f},{Pb[1]:.1f},{Pb[2]:.1f})"
                extra += f"  depth={r.get('depth_m', 0):.3f}m"
            if sm is not None:
                extra += f"  size_mm={sm[0]:.1f}x{sm[1]:.1f}"
            print(f"  keep text='{r['text']}'{extra}")
    elif not use_geo:
        print("geo 없음 — 같은 높이 필터/base 좌표 생략")

    text_objects.sort(key=lambda d: d["area"], reverse=True)
    return text_objects


def draw_results(bgr: np.ndarray, results: list[dict], hint: str = "") -> np.ndarray:
    out = bgr.copy()
    drawn = 0
    for r in results:
        width, height = r["width"], r["height"]
        max_side = MAX_SIDE_PX_MERGED if r.get("merged") else MAX_SIDE_PX
        if width > max_side or height > max_side:
            continue

        quad = r["contour"].astype(np.int32).reshape(-1, 1, 2)
        if quad.shape[0] != 4:
            quad = np.round(r["box_pts"]).astype(np.int32).reshape(-1, 1, 2)

        cx, cy = r["center"]
        cv2.drawContours(out, [quad], -1, (0, 220, 0), 2)
        cv2.drawMarker(
            out, (int(round(cx)), int(round(cy))), (0, 0, 255),
            cv2.MARKER_CROSS, 16, 2,
        )
        label1 = f"[{drawn}] {r['text']}"
        label2 = f"{width:.0f}x{height:.0f}px"
        Pb = r.get("base_mm")
        if Pb is not None:
            label2 = (
                f"{width:.0f}x{height:.0f}px  "
                f"B({Pb[0]:.0f},{Pb[1]:.0f},{Pb[2]:.0f})mm"
            )
            sm = r.get("size_mm")
            if sm is not None:
                label2 += f"  {sm[0]:.0f}x{sm[1]:.0f}mm"

        pts = quad.reshape(-1, 2)
        x0 = int(pts[:, 0].min())
        y0 = int(pts[:, 1].min())
        cv2.putText(
            out, label1, (x0, max(22, y0 - 44)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA,
        )
        cv2.putText(
            out, label2, (x0, max(42, y0 - 24)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 255, 200), 2, cv2.LINE_AA,
        )
        if Pb is not None:
            cv2.putText(
                out, f"pix({cx:.0f},{cy:.0f})", (x0, max(60, y0 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 255), 1, cv2.LINE_AA,
            )
        drawn += 1

    text = hint or f"objects={drawn} (≤{MAX_SIDE_PX}px)"
    cv2.putText(
        out, text, (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (50, 50, 255), 2,
    )
    return out


def print_results(results: list[dict]) -> None:
    print()
    print("=" * 70)
    print(f"글자 포함 물체: {len(results)}개")
    print("=" * 70)
    for i, r in enumerate(results):
        cx, cy = r["center"]
        line = (
            f"[{i:02d}] text='{r['text']}'  conf={r['conf']:.3f}  "
            f"pix=({cx:.1f},{cy:.1f})  "
            f"size_px={r['width']:.1f}x{r['height']:.1f}"
        )
        Pb = r.get("base_mm")
        if Pb is not None:
            line += (
                f"  base_mm=(X={Pb[0]:.1f}, Y={Pb[1]:.1f}, Z={Pb[2]:.1f})"
                f"  depth={r.get('depth_m', 0):.3f}m"
            )
        sm = r.get("size_mm")
        if sm is not None:
            line += f"  size_mm={sm[0]:.1f}x{sm[1]:.1f}"
        print(line)
    print("=" * 70)


def bgr_to_photo(bgr: np.ndarray) -> tk.PhotoImage:
    ok, buf = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError("이미지 인코딩 실패")
    return tk.PhotoImage(data=base64.b64encode(buf.tobytes()))


class ImageOcrApp:
    def __init__(self, root: tk.Tk, path: Path, reader):
        self.root = root
        self.root.title(WINDOW_NAME)
        self.root.protocol("WM_DELETE_WINDOW", self.root.destroy)

        bgr = cv2.imread(str(path))
        if bgr is None:
            raise FileNotFoundError(f"이미지를 열 수 없습니다: {path}")

        print(f"이미지 OCR: {path} (depth/base 좌표 없음)")
        t0 = time.time()
        results = run_ocr_pipeline(reader, bgr)
        print(f"소요 {time.time() - t0:.2f}s")
        print_results(results)

        vis = draw_results(bgr, results)
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        out_path = SAVE_DIR / f"{path.stem}_ocr.png"
        cv2.imwrite(str(out_path), vis)
        print(f"저장: {out_path}")

        top = ttk.Frame(root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(
            top,
            text=f"{path.name}  |  objects={len(results)}  |  저장: {out_path.name}",
        ).pack(side=tk.LEFT)
        ttk.Button(top, text="종료", command=root.destroy).pack(side=tk.RIGHT)

        self._photo = bgr_to_photo(vis)
        tk.Label(root, image=self._photo).pack(fill=tk.BOTH, expand=True)


class RealSenseOcrApp:
    def __init__(self, root: tk.Tk, reader, R: np.ndarray, t: np.ndarray, meta: dict):
        import pyrealsense2 as rs

        self.rs = rs
        self.root = root
        self.reader = reader
        self.R = R
        self.t = t
        self.meta = meta
        self._closed = False
        self._busy = False
        self._photo = None
        self.results: list[dict] = []
        self.last_frame: np.ndarray | None = None
        self.last_depth_m: np.ndarray | None = None
        self.last_intr: dict | None = None

        residual = (meta or {}).get("residual_mm") or {}
        mean_mm = residual.get("mean_mm")
        calib_note = (
            f"cam2base residual mean={mean_mm:.1f}mm" if mean_mm else "cam2base OK"
        )

        root.title(WINDOW_NAME)
        root.geometry("1100x750")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.status_var = tk.StringVar(
            value=(
                f"[OCR 실행]  |  {calib_note}  |  "
                f"같은높이(baseZ)±{SAME_HEIGHT_TOL_MM:.0f}mm"
            )
        )
        top = ttk.Frame(root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, textvariable=self.status_var, font=("Sans", 11)).pack(anchor=tk.W)

        btns = ttk.Frame(root, padding=8)
        btns.pack(fill=tk.X)
        self.btn_ocr = ttk.Button(btns, text="OCR 실행", command=self.on_ocr)
        self.btn_ocr.pack(side=tk.LEFT, padx=(0, 6))
        self.btn_save = ttk.Button(btns, text="저장", command=self.on_save)
        self.btn_save.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btns, text="종료", command=self.on_close).pack(side=tk.LEFT)

        self.video = tk.Label(root)
        self.video.pack(fill=tk.BOTH, expand=True)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)
        profile = self.pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())
        print(f"RealSense 시작 (depth_scale={self.depth_scale}) — depth+cam2base→base")
        print("R_cam2base =\n", R)
        print("t_cam2base [m] =", t)

        self.root.after(10, self.update_frame)

    def update_frame(self) -> None:
        if self._closed:
            return
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if color_frame and depth_frame:
                bgr = np.asanyarray(color_frame.get_data())
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth_m = depth_raw.astype(np.float32) * self.depth_scale

                intr = color_frame.profile.as_video_stream_profile().intrinsics
                self.last_frame = bgr
                self.last_depth_m = depth_m
                self.last_intr = {
                    "fx": float(intr.fx),
                    "fy": float(intr.fy),
                    "ppx": float(intr.ppx),
                    "ppy": float(intr.ppy),
                    "width": int(intr.width),
                    "height": int(intr.height),
                }

                if self.results:
                    display = draw_results(bgr, self.results)
                else:
                    display = draw_results(
                        bgr, [],
                        hint="OCR: deskew→다각도OCR→외곽→같은높이(baseZ)",
                    )
                self._photo = bgr_to_photo(display)
                self.video.configure(image=self._photo)
        except Exception as exc:
            if not self._closed:
                print(f"[카메라] {exc}")

        if not self._closed:
            self.root.after(30, self.update_frame)

    def on_ocr(self) -> None:
        if self._busy or self.last_frame is None or self.last_depth_m is None:
            return
        if self.last_intr is None:
            return
        self._busy = True
        self.btn_ocr.configure(state=tk.DISABLED)
        self.status_var.set("각도보정 OCR → 외곽사각형 → 같은높이(baseZ) ...")
        frame = self.last_frame.copy()
        depth_m = self.last_depth_m.copy()
        intr = dict(self.last_intr)
        R = self.R.copy()
        t = self.t.copy()

        def worker():
            err = None
            results: list[dict] = []
            elapsed = 0.0
            try:
                print("처리: deskew+다각도 OCR → 외곽사각형 → 같은 base-Z")
                t0 = time.time()
                results = run_ocr_pipeline(
                    self.reader, frame,
                    depth_m_img=depth_m, intr=intr, R=R, t=t,
                )
                elapsed = time.time() - t0
                print(f"소요 {elapsed:.2f}s")
                print_results(results)
            except Exception as exc:
                err = exc
            self.root.after(0, lambda: self._on_ocr_done(results, elapsed, err))

        threading.Thread(target=worker, daemon=True).start()

    def _on_ocr_done(self, results: list[dict], elapsed: float, err) -> None:
        self._busy = False
        self.btn_ocr.configure(state=tk.NORMAL)
        if err is not None:
            self.status_var.set(f"실패: {err}")
            messagebox.showerror("OCR 오류", str(err), parent=self.root)
            return
        self.results = results
        n_base = sum(1 for r in results if r.get("base_mm") is not None)
        self.status_var.set(
            f"글자 물체 {len(results)}개 (base좌표 {n_base})  |  {elapsed:.2f}s"
        )

    def on_save(self) -> None:
        if self.last_frame is None:
            messagebox.showwarning("저장", "프레임이 없습니다.", parent=self.root)
            return
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        vis = draw_results(self.last_frame, self.results)
        out_path = SAVE_DIR / f"ocr_{int(time.time())}.png"
        cv2.imwrite(str(out_path), vis)
        print(f"저장: {out_path}")
        self.status_var.set(f"저장됨: {out_path.name}")

    def on_close(self) -> None:
        self._closed = True
        try:
            self.pipeline.stop()
        except Exception:
            pass
        self.root.destroy()


def main() -> None:
    reader = create_reader()
    root = tk.Tk()
    if len(sys.argv) > 1:
        ImageOcrApp(root, Path(sys.argv[1]), reader)
    else:
        R, t, meta = load_cam2base()
        residual = (meta or {}).get("residual_mm") or {}
        if residual.get("mean_mm") is not None:
            print(
                f"캘리브 잔차 mean={residual['mean_mm']:.1f} mm, "
                f"max={residual.get('max_mm', float('nan')):.1f} mm"
            )
        RealSenseOcrApp(root, reader, R, t, meta)
    root.mainloop()


if __name__ == "__main__":
    main()
