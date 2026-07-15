"""
OCR 외곽 컨투어 + base XYZ 표시 → 클릭한 컨투어 중심으로 로봇 이동

easyocr_contour.py 의 OCR/윤곽·base 좌표와
click_and_move.py 의 RealSense + Doosan movel 을 결합합니다.

이동 기준
  - hand_eye_data/cam2base.npz → 로봇 base XYZ
  - doosan_ws TCP(그리퍼 tip) + 하향 정자세 후
    자세 조정 → 목표 XY(현재 Z 유지) → 목표 Z 순서로 movel

조작
  [OCR 실행] : 현재 프레임에서 글자 라벨 검출·base XYZ 계산
  영상 클릭  : 해당 외곽 컨투어 선택 (중심 XYZ 표시)
  모드       : [수동조작모드] 펜던트교시 / [자동모드] 앱 이동
  [기본자세] : J1..J6 = 0,0,90,0,90,0 로 movej 교정
  [자동물류] : 열기→픽업→닫기→홈→수원/서울 구역 놓기(저장Z)→열기→홈
               수원=(350,500)±150 / 서울=(370,0)±150 (base mm)
               (열기/닫기 완료·정착 후 팔 이동)
  [실행]     : 현 위치에서 그리퍼 회전 → tip 목표로 이동
  조인트     : J1~J6 표시·수정 후 [조인트 이동]
  그리퍼 TCP : tip 기준 base X,Y,Z (mm) 표시 (읽기/폴링)
  그리퍼     : 열기 / 닫기 / 커스텀닫기(기본 반만) / 설정
               → ros2 run dsr_gripper gripper_service 필요
  [종료]     : 종료

실행
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /home/newuser/ocr/doosan_ws/install/setup.bash
  # bringup은 다른 터미널
  python3 ocr_click_and_move.py
"""

from __future__ import annotations

import base64
import json
import math
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import cv2
import numpy as np
import pyrealsense2 as rs

from doosan_client import (
    DEFAULT_ROBOT_ID,
    DEFAULT_ROBOT_MODEL,
    DOOSAN_WS,
    DoosanClient,
    GRIPPER_CURRENT_CUSTOM,
    GRIPPER_DOWN_ORI_DEG,
    GRIPPER_POS_HALF,
    GRIPPER_TCP_NAME,
    GRIPPER_TCP_POS,
    HOME_POSJ_DEG,
    TARGET_Z_UP_MM,
    gripper_ori_with_yaw,
)

import easyocr_contour as ocr
from click_and_move import (
    ACC_ORI,
    ACC_XY,
    MAX_STEP_MM,
    VEL_ORI,
    VEL_XY,
    Z_OFFSET_MM,
    load_cam2base,
    warn_if_bad_calib,
    workspace_ok,
)

WINDOW_NAME = "OCR Contour → Click & Move"
SAVE_DIR = Path(__file__).resolve().parent / "ocr_out"
CALIB_NPZ = Path(__file__).resolve().parent / "hand_eye_data" / "cam2base.npz"
GRIPPER_SETTINGS_PATH = (
    Path(__file__).resolve().parent / "hand_eye_data" / "gripper_settings.json"
)

ROBOT_ID = DEFAULT_ROBOT_ID
ROBOT_MODEL = DEFAULT_ROBOT_MODEL
# 자동물류: 기본자세에서 J1만 이 값으로 이동 후 저장 Z로 하강(놓기, 미매칭 시)
AUTO_LOGISTICS_J1_DEG = -50.0
AUTO_LOGISTICS_VEL_J = 25.0
AUTO_LOGISTICS_ACC_J = 25.0
# 인식 글자 키워드 → 놓기 구역 (base mm). center + half of size
# 수원: (350,500) 중심 300×300 / 서울: (370,0) 중심 300×300
PLACE_ZONES: list[dict] = [
    {
        "keyword": "수원",
        "cx": 350.0,
        "cy": -500.0,
        "size": 300.0,
    },
    {
        "keyword": "서울",
        "cx": 370.0,
        "cy": 0.0,
        "size": 300.0,
    },
]


def resolve_place_xy(text: str) -> tuple[float, float, str] | None:
    """
    글자에 키워드가 포함되면 구역 중심 (x,y, keyword) 반환.
    여러 키워드면 PLACE_ZONES 앞쪽 우선.
    """
    t = str(text or "")
    for z in PLACE_ZONES:
        if z["keyword"] in t:
            return float(z["cx"]), float(z["cy"]), str(z["keyword"])
    return None


def place_zone_bounds(keyword: str) -> tuple[float, float, float, float] | None:
    """(xmin,xmax,ymin,ymax) — 중심±size/2."""
    for z in PLACE_ZONES:
        if z["keyword"] == keyword:
            half = float(z["size"]) * 0.5
            cx, cy = float(z["cx"]), float(z["cy"])
            return cx - half, cx + half, cy - half, cy + half
    return None


def load_gripper_settings() -> dict:
    """커스텀닫기 설정. 기본=반만 닫힘(375)."""
    defaults = {
        "custom_close_pos": int(GRIPPER_POS_HALF),
        "custom_close_current": int(GRIPPER_CURRENT_CUSTOM),
    }
    if not GRIPPER_SETTINGS_PATH.is_file():
        return defaults
    try:
        data = json.loads(GRIPPER_SETTINGS_PATH.read_text(encoding="utf-8"))
        pos = int(data.get("custom_close_pos", defaults["custom_close_pos"]))
        cur = int(data.get("custom_close_current", defaults["custom_close_current"]))
        return {
            "custom_close_pos": max(0, min(750, pos)),
            "custom_close_current": max(0, cur),
        }
    except Exception as exc:
        print(f"[그리퍼설정] 로드 실패, 기본값 사용: {exc}")
        return defaults


def save_gripper_settings(pos: int, current: int) -> None:
    GRIPPER_SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "custom_close_pos": int(max(0, min(750, pos))),
        "custom_close_current": int(max(0, current)),
        "note": "0=완전닫힘, 750=완전열림, 기본 커스텀닫기=375(반만 닫힘)",
    }
    GRIPPER_SETTINGS_PATH.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"[그리퍼설정] 저장: {GRIPPER_SETTINGS_PATH} → {payload}")


def bgr_to_photo(bgr: np.ndarray) -> tk.PhotoImage:
    ok, buf = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError("이미지 인코딩 실패")
    return tk.PhotoImage(data=base64.b64encode(buf.tobytes()))


def fit_display(
    bgr: np.ndarray, max_w: int, max_h: int
) -> tuple[np.ndarray, float, int, int]:
    """
    라벨에 맞게 축소한 표시 이미지 + scale(표시/원본) + 여백 off.
    반환: (disp_bgr, scale, off_x, off_y)  — off는 라벨 안에서 이미지 좌상단.
    """
    h, w = bgr.shape[:2]
    max_w = max(int(max_w), 1)
    max_h = max(int(max_h), 1)
    scale = min(max_w / w, max_h / h, 1.0)
    dw, dh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
    if scale < 0.999:
        disp = cv2.resize(bgr, (dw, dh), interpolation=cv2.INTER_AREA)
    else:
        disp = bgr
        dw, dh = w, h
        scale = 1.0
    off_x = max(0, (max_w - dw) // 2)
    off_y = max(0, (max_h - dh) // 2)
    return disp, float(scale), int(off_x), int(off_y)


def draw_ocr_with_selection(
    bgr: np.ndarray,
    results: list[dict],
    selected_idx: int | None = None,
    hint: str = "",
) -> np.ndarray:
    """OCR 결과 그리기 + 선택된 컨투어 강조."""
    out = ocr.draw_results(bgr, results, hint=hint)
    if selected_idx is None or not (0 <= selected_idx < len(results)):
        return out

    r = results[selected_idx]
    tb = r.get("text_box_pts")
    if tb is not None and not ocr.quad_contains_text_box(r["box_pts"], tb):
        return out
    if r["width"] > ocr.MAX_SIDE_PX or r["height"] > ocr.MAX_SIDE_PX:
        return out

    quad = r["contour"].astype(np.int32).reshape(-1, 1, 2)
    if quad.shape[0] != 4:
        quad = np.round(r["box_pts"]).astype(np.int32).reshape(-1, 1, 2)
    cv2.drawContours(out, [quad], -1, (0, 165, 255), 3)
    cx, cy = r["center"]
    cv2.drawMarker(
        out, (int(round(cx)), int(round(cy))), (0, 165, 255),
        cv2.MARKER_CROSS, 28, 3,
    )
    Pb = r.get("base_mm")
    if Pb is not None:
        msg = f"SEL base ({Pb[0]:.0f},{Pb[1]:.0f},{Pb[2]:.0f}) mm"
        cv2.putText(
            out, msg, (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2, cv2.LINE_AA,
        )
    return out


def _aabb_contains(box_pts: np.ndarray, px: float, py: float, pad: float = 4.0) -> bool:
    pts = np.asarray(box_pts, dtype=np.float32).reshape(-1, 2)
    x0, y0 = float(pts[:, 0].min()) - pad, float(pts[:, 1].min()) - pad
    x1, y1 = float(pts[:, 0].max()) + pad, float(pts[:, 1].max()) + pad
    return x0 <= px <= x1 and y0 <= py <= y1


def hit_test_result(results: list[dict], px: int, py: int) -> int | None:
    """
    클릭 픽셀이 들어있는 컨투어 인덱스.
    base_mm 없어도 선택 가능. 여러 개면 면적 작은 것.
    """
    hits: list[tuple[float, int]] = []
    pt = (float(px), float(py))
    for i, r in enumerate(results):
        tb = r.get("text_box_pts")
        if tb is not None and not ocr.quad_contains_text_box(r["box_pts"], tb):
            continue
        if r["width"] > ocr.MAX_SIDE_PX or r["height"] > ocr.MAX_SIDE_PX:
            if not r.get("merged"):
                continue
            if r["width"] > ocr.MAX_SIDE_PX_MERGED or r["height"] > ocr.MAX_SIDE_PX_MERGED:
                continue
        inside = ocr.point_in_quad(pt, r["box_pts"])
        if not inside:
            cnt = np.asarray(r["contour"], dtype=np.float32).reshape(-1, 2)
            if cnt.shape[0] >= 3:
                inside = cv2.pointPolygonTest(cnt, pt, False) >= 0
        if not inside:
            # 회전 사각형 꼭짓점 오차 대비 AABB 폴백
            inside = _aabb_contains(r["box_pts"], float(px), float(py), pad=8.0)
        if inside:
            hits.append((float(r["area"]), i))
    if not hits:
        return None
    hits.sort(key=lambda t: t[0])
    return hits[0][1]


def contour_skew_image_deg(box_pts: np.ndarray, *, along_short_side: bool = True) -> float:
    """
    원본(축정렬·이미지 +x) 대비 컨투어 방향각 [deg], 범위 (−90, 90].

    along_short_side=True (기본):
      단축(짧은 변) 방향 — 그리퍼가 짧은 쪽을 집도록 yaw에 사용.
      정각(축정렬) 긴 직사각형이면 장축이 아니라 짧은 변 쪽으로 90° 보정.
    along_short_side=False:
      장축(긴 변) 방향.
    """
    pts = np.asarray(box_pts, dtype=np.float32).reshape(-1, 2)
    if pts.shape[0] < 3:
        return 0.0
    raw = cv2.minAreaRect(pts)
    (_c, (rw, rh), angle) = ocr.normalize_min_area_rect(raw)
    a = float(angle)  # 장축 [0, 180)
    # 거의 정사각이면 장/단 구분이 무의미 — 장축 유지
    if along_short_side and float(rw) > float(rh) * 1.05:
        a += 90.0  # 단축 = 장축 ⊥
    # (−90, 90] 로 정규화
    while a > 90.0:
        a -= 180.0
    while a <= -90.0:
        a += 180.0
    return float(a)


def contour_yaw_base_deg(
    box_pts: np.ndarray, R_cam2base: np.ndarray, *, along_short_side: bool = True
) -> tuple[float, float]:
    """
    컨투어 방향 → (이미지 스큐각, 로봇 base XY yaw).
    기본은 짧은 변(단축) 기준 — 그리퍼가 짧은 쪽을 집도록.
    RealSense/OpenCV: 이미지 u→cam X, v→cam Y, R로 base에 투영.
    """
    skew = contour_skew_image_deg(box_pts, along_short_side=along_short_side)
    rad = math.radians(skew)
    d_cam = np.array([math.cos(rad), math.sin(rad), 0.0], dtype=np.float64)
    d_base = np.asarray(R_cam2base, dtype=np.float64).reshape(3, 3) @ d_cam
    yaw = math.degrees(math.atan2(float(d_base[1]), float(d_base[0])))
    return float(skew), float(yaw)


class OcrClickMoveApp:
    def __init__(
        self,
        root: tk.Tk,
        reader,
        R: np.ndarray,
        t: np.ndarray,
        meta: dict,
        robot: DoosanClient | None,
    ):
        self.root = root
        self.reader = reader
        self.R = R
        self.t = t
        self.meta = meta
        self.robot = robot

        self._closed = False
        self._busy = False
        self._moving = False
        self._grip_busy = False
        self._photo = None
        # 표시 변환: 원본 픽셀 ↔ Label 클릭 좌표
        self._view: dict | None = None  # scale, disp_w, disp_h, src_w, src_h

        self.results: list[dict] = []
        self.selected_idx: int | None = None
        self.last_frame: np.ndarray | None = None
        self.last_depth_m: np.ndarray | None = None
        self.last_intr: dict | None = None
        self.frozen_frame: np.ndarray | None = None  # OCR 당시 컬러 (오버레이용)
        self.frozen_depth_m: np.ndarray | None = None
        self.frozen_intr: dict | None = None

        gset = load_gripper_settings()
        self.custom_close_pos = int(gset["custom_close_pos"])
        self.custom_close_current = int(gset["custom_close_current"])
        if robot is not None:
            robot.set_custom_close(self.custom_close_pos, self.custom_close_current)

        residual = (meta or {}).get("residual_mm") or {}
        mean_mm = residual.get("mean_mm")
        calib_note = (
            f"cam2base residual mean={mean_mm:.1f}mm" if mean_mm is not None else "cam2base OK"
        )
        print(f"로봇 base 좌표용 캘리브: {CALIB_NPZ}")
        print(
            f"커스텀닫기 기본: pos={self.custom_close_pos} "
            f"(0닫힘~750열림, 반닫힘={GRIPPER_POS_HALF})"
        )

        root.title(WINDOW_NAME)
        root.geometry("1280x900")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.status_var = tk.StringVar(
            value=(
                f"[OCR 실행] → 컨투어 클릭 → [실행]  |  "
                f"그리퍼TCP={GRIPPER_TCP_NAME}  |  {calib_note}"
            )
        )
        self.info_var = tk.StringVar(value="선택: 없음")
        self.pose_var = tk.StringVar(value="Robot pose: -")
        self.grip_var = tk.StringVar(
            value=(
                f"그리퍼: 커스텀닫기={self.custom_close_pos} "
                f"(반닫힘 기본 {GRIPPER_POS_HALF})"
            )
        )

        top = ttk.Frame(root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, textvariable=self.status_var, font=("Sans", 11)).pack(anchor=tk.W)
        ttk.Label(top, textvariable=self.info_var).pack(anchor=tk.W)
        ttk.Label(top, textvariable=self.pose_var).pack(anchor=tk.W)
        ttk.Label(top, textvariable=self.grip_var).pack(anchor=tk.W)

        btns = ttk.Frame(root, padding=8)
        btns.pack(fill=tk.X)
        self.btn_ocr = ttk.Button(btns, text="OCR 실행", command=self.on_ocr)
        self.btn_ocr.pack(side=tk.LEFT, padx=(0, 6))
        self.btn_exec = ttk.Button(
            btns, text="실행 (이동)", command=self.on_execute, state=tk.DISABLED
        )
        self.btn_exec.pack(side=tk.LEFT, padx=(0, 6))
        self.btn_save = ttk.Button(btns, text="저장", command=self.on_save)
        self.btn_save.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btns, text="종료", command=self.on_close).pack(side=tk.LEFT)

        mode_state = tk.NORMAL if (robot and robot.connected) else tk.DISABLED
        self.btn_manual = ttk.Button(
            btns,
            text="수동조작모드",
            command=self.on_manual_mode,
            state=mode_state,
        )
        self.btn_manual.pack(side=tk.LEFT, padx=(16, 6))
        self.btn_auto = ttk.Button(
            btns,
            text="자동모드",
            command=self.on_auto_mode,
            state=mode_state,
        )
        self.btn_auto.pack(side=tk.LEFT, padx=(0, 6))
        self.btn_home = ttk.Button(
            btns,
            text="기본자세로교정",
            command=self.on_go_home,
            state=mode_state,
        )
        self.btn_home.pack(side=tk.LEFT, padx=(0, 6))
        self.btn_auto_logistics = ttk.Button(
            btns,
            text="자동물류",
            command=self.on_auto_logistics,
            state=mode_state,
        )
        self.btn_auto_logistics.pack(side=tk.LEFT, padx=(0, 6))

        self.robot_mode_var = tk.StringVar(
            value=(
                "로봇: 연결됨 (AUTONOMOUS)"
                if robot and robot.connected
                else "로봇: 미연결 (좌표만)"
            )
        )
        ttk.Label(btns, textvariable=self.robot_mode_var).pack(side=tk.RIGHT)

        # 그리퍼 손가락 제어 (dsr_gripper /gripper/cmd)
        grip = ttk.LabelFrame(root, text="그리퍼 (열기/닫기)", padding=8)
        grip.pack(fill=tk.X, padx=8, pady=(0, 4))
        grip_state = tk.NORMAL if (robot and robot.connected) else tk.DISABLED
        self.btn_grip_open = ttk.Button(
            grip, text="열기", command=self.on_grip_open, state=grip_state
        )
        self.btn_grip_open.pack(side=tk.LEFT, padx=(0, 6))
        self.btn_grip_close = ttk.Button(
            grip, text="닫기", command=self.on_grip_close, state=grip_state
        )
        self.btn_grip_close.pack(side=tk.LEFT, padx=(0, 6))
        self.btn_grip_custom = ttk.Button(
            grip,
            text=f"커스텀닫기 ({self.custom_close_pos})",
            command=self.on_grip_custom_close,
            state=grip_state,
        )
        self.btn_grip_custom.pack(side=tk.LEFT, padx=(0, 6))
        self.btn_grip_settings = ttk.Button(
            grip, text="그리퍼 설정", command=self.on_grip_settings, state=grip_state
        )
        self.btn_grip_settings.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(
            grip,
            text="0=완전닫힘 · 750=완전열림 · 커스텀 기본=375(반만)",
        ).pack(side=tk.LEFT, padx=(12, 0))

        # 조인트 상태 (deg) + 그리퍼 TCP XYZ (base mm)
        pose_box = ttk.LabelFrame(root, text="조인트 / 그리퍼 TCP", padding=8)
        pose_box.pack(fill=tk.X, padx=8, pady=(0, 4))

        joint = ttk.Frame(pose_box)
        joint.pack(fill=tk.X)
        self.joint_vars: list[tk.StringVar] = []
        self.joint_entries: list[ttk.Entry] = []
        for i in range(6):
            cell = ttk.Frame(joint)
            cell.pack(side=tk.LEFT, padx=(0, 8))
            ttk.Label(cell, text=f"J{i+1}").pack(side=tk.LEFT)
            var = tk.StringVar(value="-")
            ent = ttk.Entry(cell, textvariable=var, width=8)
            ent.pack(side=tk.LEFT, padx=(4, 0))
            self.joint_vars.append(var)
            self.joint_entries.append(ent)
        self.btn_joint_refresh = ttk.Button(
            joint, text="읽기", command=self.on_joint_refresh, state=grip_state
        )
        self.btn_joint_refresh.pack(side=tk.LEFT, padx=(8, 4))
        self.btn_joint_move = ttk.Button(
            joint, text="조인트 이동", command=self.on_joint_move, state=grip_state
        )
        self.btn_joint_move.pack(side=tk.LEFT, padx=(0, 4))
        self.joint_status = tk.StringVar(value="조인트: -")
        ttk.Label(joint, textvariable=self.joint_status).pack(side=tk.LEFT, padx=(8, 0))

        tcp = ttk.Frame(pose_box)
        tcp.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(tcp, text="그리퍼 tip (base mm)").pack(side=tk.LEFT, padx=(0, 8))
        self.tcp_xyz_vars: dict[str, tk.StringVar] = {}
        for axis in ("X", "Y", "Z"):
            cell = ttk.Frame(tcp)
            cell.pack(side=tk.LEFT, padx=(0, 10))
            ttk.Label(cell, text=axis).pack(side=tk.LEFT)
            var = tk.StringVar(value="-")
            ttk.Entry(cell, textvariable=var, width=10, state="readonly").pack(
                side=tk.LEFT, padx=(4, 0)
            )
            self.tcp_xyz_vars[axis] = var
        self.tcp_ori_var = tk.StringVar(value="ori: -")
        ttk.Label(tcp, textvariable=self.tcp_ori_var).pack(side=tk.LEFT, padx=(8, 0))
        self.tcp_status = tk.StringVar(value="")
        ttk.Label(tcp, textvariable=self.tcp_status).pack(side=tk.LEFT, padx=(8, 0))

        self.video = tk.Label(root)
        self.video.pack(fill=tk.BOTH, expand=True)
        self.video.bind("<Button-1>", self.on_image_click)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)
        profile = self.pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())
        print(f"RealSense 시작 (depth_scale={self.depth_scale})")
        print("R_cam2base =\n", R)
        print("t_cam2base [m] =", t)

        self.root.after(10, self.update_frame)
        if robot and robot.connected:
            self.root.after(500, self._poll_joints)

    def _show_bgr(self, bgr: np.ndarray) -> None:
        """원본 BGR을 라벨에 맞게 축소해 표시하고 클릭 매핑을 저장."""
        self.root.update_idletasks()
        lbl_w = max(self.video.winfo_width(), 1)
        lbl_h = max(self.video.winfo_height(), 1)
        # 아직 pack 전이면 대략 창 크기 사용
        if lbl_w < 40 or lbl_h < 40:
            lbl_w = max(self.root.winfo_width() - 24, 640)
            lbl_h = max(self.root.winfo_height() - 160, 360)
        src_h, src_w = bgr.shape[:2]
        disp, scale, _ox, _oy = fit_display(bgr, lbl_w, lbl_h)
        dh, dw = disp.shape[:2]
        self._view = {
            "scale": float(scale),
            "disp_w": int(dw),
            "disp_h": int(dh),
            "src_w": int(src_w),
            "src_h": int(src_h),
        }
        self._photo = bgr_to_photo(disp)
        self.video.configure(image=self._photo)

    def _label_to_image_xy(self, event) -> tuple[int, int] | None:
        """Label 클릭 → 원본 이미지 픽셀. PhotoImage는 축소본 + Label center 배치."""
        if self._view is None:
            return None
        vw = self._view
        lbl_w = max(self.video.winfo_width(), 1)
        lbl_h = max(self.video.winfo_height(), 1)
        dw, dh = int(vw["disp_w"]), int(vw["disp_h"])
        scale = float(vw["scale"])
        if scale <= 0:
            return None
        # PhotoImage는 dw×dh, Label은 기본적으로 center 앵커로 배치
        off_x = (lbl_w - dw) // 2
        off_y = (lbl_h - dh) // 2
        if not (off_x <= event.x < off_x + dw and off_y <= event.y < off_y + dh):
            return None
        x = int((event.x - off_x) / scale)
        y = int((event.y - off_y) / scale)
        if not (0 <= x < int(vw["src_w"]) and 0 <= y < int(vw["src_h"])):
            return None
        return x, y

    def _ensure_base_mm(self, r: dict) -> np.ndarray | None:
        """컨투어 중심을 cam2base로 base mm 계산 (없으면 동결 depth로 재계산)."""
        Pb = r.get("base_mm")
        if Pb is not None:
            return np.asarray(Pb, dtype=np.float64)

        depth = self.frozen_depth_m
        intr = self.frozen_intr
        if depth is None or intr is None:
            return None
        cx, cy = float(r["center"][0]), float(r["center"][1])
        Pb_new, _z = ocr.pixel_to_base_mm(cx, cy, depth, intr, self.R, self.t)
        if Pb_new is None:
            return None
        Pb_new = np.asarray(Pb_new, dtype=np.float64).copy()
        Pb_new[2] += Z_OFFSET_MM
        r["base_mm"] = Pb_new
        return Pb_new

    def on_image_click(self, event) -> None:
        if self._busy or self._moving:
            return
        if not self.results:
            self.status_var.set("먼저 [OCR 실행]을 하세요")
            return
        xy = self._label_to_image_xy(event)
        if xy is None:
            self.status_var.set("영상 영역 안을 클릭하세요")
            return
        px, py = xy
        idx = hit_test_result(self.results, px, py)
        if idx is None:
            self.selected_idx = None
            self.btn_exec.configure(state=tk.DISABLED)
            self.info_var.set(f"선택: 없음 (클릭 pix=({px},{py}) — 컨투어 밖)")
            self.status_var.set("컨투어 안을 클릭하세요")
            print(f"[클릭] pix=({px},{py}) 미스 (결과 {len(self.results)}개)")
            return

        self.selected_idx = idx
        r = self.results[idx]
        Pb = self._ensure_base_mm(r)
        cx, cy = r["center"]
        if Pb is None:
            self.btn_exec.configure(state=tk.DISABLED)
            self.info_var.set(
                f"[{idx}] '{r['text']}'  pix=({cx:.0f},{cy:.0f}) — base 좌표 없음(depth)"
            )
            self.status_var.set("depth/cam2base 변환 실패")
            return

        z_move = float(Pb[2]) + float(TARGET_Z_UP_MM)
        skew_img, yaw_base = contour_yaw_base_deg(r["box_pts"], self.R)
        ori = gripper_ori_with_yaw(yaw_base)
        self.info_var.set(
            f"[{idx}] '{r['text']}'  conf={r['conf']:.2f}  "
            f"pix=({cx:.0f},{cy:.0f})  "
            f"surface_Z={Pb[2]:.1f}  move_Z={z_move:.1f}  "
            f"skew(단축)={skew_img:.1f}°  yaw={yaw_base:.1f}°  "
            f"ori=({ori[0]:.1f},{ori[1]:.1f},{ori[2]:.1f})"
        )
        print(self.info_var.get())

        ok_ws, ws_msg = workspace_ok(np.array([Pb[0], Pb[1], z_move], dtype=np.float64))
        if not ok_ws:
            self.status_var.set(f"이동 불가 — {ws_msg}")
            self.btn_exec.configure(state=tk.DISABLED)
            return

        if self.robot is not None and self.robot.connected:
            self.btn_exec.configure(state=tk.NORMAL)
            self.status_var.set("컨투어 선택됨 — [실행]으로 이동")
        else:
            self.btn_exec.configure(state=tk.DISABLED)
            self.status_var.set("컨투어 선택됨 (로봇 미연결 — 좌표만)")

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

                if self.results and self.frozen_frame is not None:
                    display = draw_ocr_with_selection(
                        self.frozen_frame, self.results, self.selected_idx
                    )
                else:
                    display = draw_ocr_with_selection(
                        bgr, [],
                        hint="Press [OCR 실행] then click a contour",
                    )
                self._show_bgr(display)
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
        self.btn_exec.configure(state=tk.DISABLED)
        self._set_grip_buttons(False)
        self.selected_idx = None
        self.status_var.set("OCR 처리 중 (다각도·승격·cam2base base XYZ)...")
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
                t0 = time.time()
                results = ocr.run_ocr_pipeline(
                    self.reader, frame,
                    depth_m_img=depth_m, intr=intr, R=R, t=t,
                )
                # Z offset 적용해 표시·이동용으로 맞춰 둠 (로봇 base mm)
                for r in results:
                    Pb = r.get("base_mm")
                    if Pb is not None:
                        Pb = np.asarray(Pb, dtype=np.float64).copy()
                        Pb[2] += Z_OFFSET_MM
                        r["base_mm"] = Pb
                elapsed = time.time() - t0
                ocr.print_results(results)
            except Exception as exc:
                err = exc
            self.root.after(
                0,
                lambda: self._on_ocr_done(frame, depth_m, intr, results, elapsed, err),
            )

        threading.Thread(target=worker, daemon=True).start()

    def _on_ocr_done(
        self,
        frame: np.ndarray,
        depth_m: np.ndarray,
        intr: dict,
        results: list[dict],
        elapsed: float,
        err,
    ) -> None:
        self._busy = False
        self.btn_ocr.configure(state=tk.NORMAL)
        self._set_grip_buttons(True)
        if err is not None:
            self.status_var.set(f"OCR 실패: {err}")
            messagebox.showerror("OCR 오류", str(err), parent=self.root)
            return
        self.frozen_frame = frame
        self.frozen_depth_m = depth_m
        self.frozen_intr = intr
        self.results = results
        n_base = sum(1 for r in results if r.get("base_mm") is not None)
        self.status_var.set(
            f"OCR {len(results)}개 (base좌표 {n_base})  |  {elapsed:.1f}s  |  "
            "컨투어를 클릭하세요"
        )
        self.info_var.set("선택: 없음")

    def _build_target_posx(self) -> list[float] | None:
        if self.selected_idx is None or not (0 <= self.selected_idx < len(self.results)):
            return None
        r = self.results[self.selected_idx]
        Pb = r.get("base_mm")
        if Pb is None:
            return None
        x, y, z_surf = float(Pb[0]), float(Pb[1]), float(Pb[2])
        z = z_surf + float(TARGET_Z_UP_MM)
        skew_img, yaw_base = contour_yaw_base_deg(r["box_pts"], self.R)
        ori = gripper_ori_with_yaw(yaw_base)
        r["skew_img_deg"] = skew_img
        r["yaw_base_deg"] = yaw_base
        return [x, y, z, float(ori[0]), float(ori[1]), float(ori[2])]

    def _update_mode_label(self) -> None:
        if self.robot is None or not self.robot.connected:
            self.robot_mode_var.set("로봇: 미연결 (좌표만)")
            return
        if self.robot.is_manual_mode():
            self.robot_mode_var.set("로봇: MANUAL (수동조작 — 펜던트/티칭)")
        else:
            self.robot_mode_var.set("로봇: AUTONOMOUS (자동)")

    def _set_auto_motion_enabled(self, enabled: bool) -> None:
        """수동모드에서는 앱 자동 이동/조인트이동만 끄고, OCR·그리퍼는 유지."""
        can = bool(
            enabled
            and self.robot is not None
            and self.robot.connected
            and not self.robot.is_manual_mode()
        )
        # 실행 버튼은 선택 상태도 봐야 함
        if hasattr(self, "btn_exec"):
            if can and self.selected_idx is not None:
                self.btn_exec.configure(state=tk.NORMAL)
            else:
                self.btn_exec.configure(state=tk.DISABLED)
        if hasattr(self, "btn_joint_move"):
            self.btn_joint_move.configure(
                state=tk.NORMAL if can else tk.DISABLED
            )
        if hasattr(self, "btn_home"):
            self.btn_home.configure(state=tk.NORMAL if can else tk.DISABLED)
        if hasattr(self, "btn_auto_logistics"):
            self.btn_auto_logistics.configure(
                state=tk.NORMAL if can else tk.DISABLED
            )

    def _target_posx_for_idx(self, idx: int) -> list[float] | None:
        if not (0 <= idx < len(self.results)):
            return None
        r = self.results[idx]
        Pb = r.get("base_mm")
        if Pb is None:
            Pb = self._ensure_base_mm(r)
        if Pb is None:
            return None
        x, y, z_surf = float(Pb[0]), float(Pb[1]), float(Pb[2])
        z = z_surf + float(TARGET_Z_UP_MM)
        skew_img, yaw_base = contour_yaw_base_deg(r["box_pts"], self.R)
        ori = gripper_ori_with_yaw(yaw_base)
        r["skew_img_deg"] = skew_img
        r["yaw_base_deg"] = yaw_base
        return [x, y, z, float(ori[0]), float(ori[1]), float(ori[2])]

    def _auto_logistics_indices(self) -> list[int]:
        if self.selected_idx is not None and 0 <= self.selected_idx < len(self.results):
            return [self.selected_idx]
        out: list[int] = []
        for i, r in enumerate(self.results):
            Pb = r.get("base_mm")
            if Pb is None:
                Pb = self._ensure_base_mm(r)
            if Pb is not None:
                out.append(i)
        return out

    def on_auto_logistics(self) -> None:
        if self.robot is None or not self.robot.connected:
            self.status_var.set("로봇 미연결 — 자동물류 불가")
            return
        if self.robot.is_manual_mode():
            self.status_var.set("수동조작모드 — [자동모드]로 전환 후 자동물류하세요")
            return
        if self._moving or self._grip_busy or self._busy:
            self.status_var.set("다른 작업 중 — 자동물류 대기")
            return
        if not self.results:
            self.status_var.set("먼저 [OCR 실행]으로 글자 개체를 인식하세요")
            return
        idxs = self._auto_logistics_indices()
        if not idxs:
            self.status_var.set("유효한 base XYZ가 있는 개체가 없습니다")
            return

        self._moving = True
        self.btn_exec.configure(state=tk.DISABLED)
        self.btn_ocr.configure(state=tk.DISABLED)
        for name in ("btn_home", "btn_joint_move", "btn_auto_logistics"):
            btn = getattr(self, name, None)
            if btn is not None:
                btn.configure(state=tk.DISABLED)
        self._set_grip_buttons(False)
        self.status_var.set(
            f"자동물류 시작 — {len(idxs)}개 사이클 "
            f"(선택={'있음' if self.selected_idx is not None else '전체'})"
        )
        print(
            f"[자동물류] 시작 indices={idxs} "
            f"place_J1={AUTO_LOGISTICS_J1_DEG:.0f}°"
        )

        def worker():
            ok_all = True
            err_msg = ""
            try:
                if not self.robot.gripper_ready:
                    self.robot.ensure_gripper_tcp()
                for n, idx in enumerate(idxs):
                    if self._closed:
                        ok_all = False
                        err_msg = "앱 종료로 중단"
                        break
                    label = self.results[idx].get("text", "?")
                    self.root.after(
                        0,
                        lambda i=idx, lab=label, n=n: self.status_var.set(
                            f"자동물류 [{n+1}/{len(idxs)}] '{lab}' 픽업…"
                        ),
                    )
                    self.root.after(0, lambda i=idx: setattr(self, "selected_idx", i))
                    ok, msg = self._auto_logistics_cycle(idx, n + 1, len(idxs))
                    if not ok:
                        if "놓기 구역 키워드" in msg or "놓기 구역 없음" in msg:
                            print(f"[자동물류] 건너뜀: {msg}")
                            self.root.after(
                                0,
                                lambda m=msg: self.status_var.set(f"건너뜀 — {m}"),
                            )
                            continue
                        ok_all = False
                        err_msg = msg
                        break
            except Exception as exc:
                ok_all = False
                err_msg = str(exc)
                print(f"[자동물류] 예외: {exc}")
            self.root.after(
                0, lambda: self._on_auto_logistics_done(ok_all, err_msg)
            )

        threading.Thread(
            target=worker, daemon=True, name="auto_logistics_worker"
        ).start()

    def _auto_logistics_cycle(
        self, idx: int, num: int, total: int
    ) -> tuple[bool, str]:
        """
        1사이클:
          열기(완료) → 자세→픽업이동 → 커스텀닫기(완료) → 기본자세
          → 글자(수원/서울) 놓기 구역으로 XY이동 + 저장Z → 열기(완료) → 기본자세
        """
        r = self.results[idx]
        target = self._target_posx_for_idx(idx)
        if target is None:
            return False, f"[{idx}] base 좌표 없음"
        z_saved = float(target[2])
        text = str(r.get("text", "?"))
        if resolve_place_xy(text) is None:
            return False, (
                f"[{idx}] '{text}' — 놓기 구역 키워드(수원/서울) 없음, 건너뜀"
            )
        print()
        print("=" * 50)
        print(
            f"[자동물류] {num}/{total} '{text}' "
            f"저장 tip_Z={z_saved:.1f} mm  "
            f"pick=[{target[0]:.1f},{target[1]:.1f},{target[2]:.1f}]"
        )

        ok_ws, ws_msg = workspace_ok(np.array(target[:3], dtype=np.float64))
        if not ok_ws:
            return False, f"픽업 작업영역 밖 — {ws_msg}"

        # 1) 이동 전 그리퍼 열기 (완료·정착 후 이동)
        self.root.after(
            0,
            lambda: self.status_var.set(
                f"자동물류 [{num}/{total}] '{text}' → 이동 전 그리퍼 열기"
            ),
        )
        if not self.robot.gripper_open():
            return False, f"[{idx}] 픽업 전 그리퍼 열기 실패"

        def pick_phase(p: str, _n=num, _t=total, _lab=text) -> None:
            if p == "rotate":
                msg = f"자동물류 [{_n}/{_t}] '{_lab}' → 그리퍼 자세 조정"
            elif p == "move_xy":
                msg = f"자동물류 [{_n}/{_t}] '{_lab}' → 픽업 XY 이동"
            elif p == "move_z":
                msg = f"자동물류 [{_n}/{_t}] '{_lab}' → 픽업 Z 이동"
            else:
                msg = f"자동물류 [{_n}/{_t}] '{_lab}' → 픽업 이동"
            self.root.after(0, lambda m=msg: self.status_var.set(m))

        if not self.robot.movel_gripper(
            target[:3],
            vel=[VEL_XY, VEL_ORI],
            acc=[ACC_XY, ACC_ORI],
            straighten_first=True,
            rotate_first=True,
            z_up_mm=0.0,
            ori_deg=target[3:6],
            phase_cb=pick_phase,
        ):
            return False, f"[{idx}] 픽업 이동 실패"

        # 2) 닫기 완료 후 팔 이동
        self.root.after(
            0,
            lambda: self.status_var.set(
                f"자동물류 [{num}/{total}] '{text}' → 커스텀닫기 (완료 대기)"
            ),
        )
        if not self.robot.gripper_custom_close():
            return False, f"[{idx}] 커스텀닫기 실패"

        self.root.after(
            0,
            lambda: self.status_var.set(
                f"자동물류 [{num}/{total}] '{text}' → 기본자세"
            ),
        )
        if not self.robot.go_home_posj(
            vel=AUTO_LOGISTICS_VEL_J, acc=AUTO_LOGISTICS_ACC_J
        ):
            return False, f"[{idx}] 기본자세(픽업 후) 실패"

        place = resolve_place_xy(text)
        if place is None:
            return False, (
                f"[{idx}] 놓기 구역 없음 — 글자 '{text}'에 "
                f"'수원'/'서울'이 필요합니다"
            )
        place_x, place_y, place_key = place
        bounds = place_zone_bounds(place_key)
        print(
            f"[자동물류] 놓기 구역 '{place_key}' center=({place_x:.0f},{place_y:.0f}) "
            f"Z={z_saved:.1f} bounds={bounds}"
        )
        ok_ws, ws_msg = workspace_ok(
            np.array([place_x, place_y, z_saved], dtype=np.float64)
        )
        if not ok_ws:
            return False, f"[{idx}] 놓기 좌표 작업영역 밖 — {ws_msg}"

        cur = self.robot.get_posx_mm_deg()
        if cur is None or len(cur) < 6:
            return False, f"[{idx}] 놓기 전 pose 읽기 실패"
        place_ori = [float(cur[3]), float(cur[4]), float(cur[5])]

        def place_phase(p: str, _n=num, _t=total, _lab=text, _k=place_key) -> None:
            if p == "rotate":
                msg = f"자동물류 [{_n}/{_t}] '{_lab}' → 놓기 전 그리퍼 자세"
            elif p == "move_xy":
                msg = (
                    f"자동물류 [{_n}/{_t}] '{_lab}' → "
                    f"{_k}구역 XY ({place_x:.0f},{place_y:.0f})"
                )
            elif p == "move_z":
                msg = (
                    f"자동물류 [{_n}/{_t}] '{_lab}' → "
                    f"{_k}구역 Z={z_saved:.1f}"
                )
            else:
                msg = (
                    f"자동물류 [{_n}/{_t}] '{_lab}' → "
                    f"{_k}구역 ({place_x:.0f},{place_y:.0f},Z={z_saved:.1f})"
                )
            self.root.after(0, lambda m=msg: self.status_var.set(m))

        if not self.robot.movel_gripper(
            [float(place_x), float(place_y), float(z_saved)],
            vel=[VEL_XY, VEL_ORI],
            acc=[ACC_XY, ACC_ORI],
            straighten_first=True,
            rotate_first=True,
            z_up_mm=0.0,
            ori_deg=place_ori,
            phase_cb=place_phase,
        ):
            return False, f"[{idx}] '{place_key}' 구역 놓기 이동 실패"

        # 도착점이 구역 안인지 확인 (중심 배치가 기본)
        arrived = self.robot.get_posx_mm_deg()
        if arrived is not None and bounds is not None:
            xmin, xmax, ymin, ymax = bounds
            ax, ay = float(arrived[0]), float(arrived[1])
            if not (xmin <= ax <= xmax and ymin <= ay <= ymax):
                print(
                    f"[자동물류] 경고: 도착 XY=({ax:.1f},{ay:.1f}) 가 "
                    f"'{place_key}' 구역 [{xmin:.0f}~{xmax:.0f}], "
                    f"[{ymin:.0f}~{ymax:.0f}] 밖"
                )

        # 3) 열기 완료 후 기본자세 복귀
        self.root.after(
            0,
            lambda: self.status_var.set(
                f"자동물류 [{num}/{total}] '{text}' → 그리퍼 열기 (완료 대기)"
            ),
        )
        if not self.robot.gripper_open():
            return False, f"[{idx}] 그리퍼 열기 실패"

        self.root.after(
            0,
            lambda: self.status_var.set(
                f"자동물류 [{num}/{total}] '{text}' → 기본자세 복귀"
            ),
        )
        if not self.robot.go_home_posj(
            vel=AUTO_LOGISTICS_VEL_J, acc=AUTO_LOGISTICS_ACC_J
        ):
            return False, f"[{idx}] 기본자세(놓기 후) 실패"

        print(f"[자동물류] {num}/{total} '{text}' 사이클 완료")
        print("=" * 50)
        return True, "ok"

    def _on_auto_logistics_done(self, ok: bool, err_msg: str) -> None:
        self._moving = False
        self.btn_ocr.configure(state=tk.NORMAL)
        self._set_grip_buttons(True)
        self._set_auto_motion_enabled(True)
        if ok:
            self.status_var.set("자동물류 완료 — 기본자세")
            self.on_joint_refresh()
        else:
            self.status_var.set(f"자동물류 중단: {err_msg or '실패'}")
            messagebox.showerror(
                "자동물류", err_msg or "자동물류 실패", parent=self.root
            )
            try:
                self.on_joint_refresh()
            except Exception:
                pass

    def on_go_home(self) -> None:
        if self.robot is None or not self.robot.connected:
            self.status_var.set("로봇 미연결 — 기본자세 교정 불가")
            return
        if self.robot.is_manual_mode():
            self.status_var.set("수동조작모드 — [자동모드]로 전환 후 교정하세요")
            return
        if self._moving or self._grip_busy:
            self.status_var.set("다른 작업 중 — 기본자세 교정 대기")
            return

        home_txt = "[" + ", ".join(f"{v:.0f}" for v in HOME_POSJ_DEG) + "]"
        self._moving = True
        self.btn_exec.configure(state=tk.DISABLED)
        self.btn_ocr.configure(state=tk.DISABLED)
        if hasattr(self, "btn_home"):
            self.btn_home.configure(state=tk.DISABLED)
        if hasattr(self, "btn_joint_move"):
            self.btn_joint_move.configure(state=tk.DISABLED)
        self._set_grip_buttons(False)
        self.status_var.set(f"기본자세 교정 중 {home_txt}...")
        print(f"[앱] 기본자세 교정 → {home_txt}")

        def worker():
            ok = False
            err = None
            try:
                ok = self.robot.go_home_posj()
            except Exception as exc:
                err = exc
                print(f"[앱] 기본자세 교정 예외: {exc}")
            self.root.after(0, lambda: self._on_go_home_done(ok, err))

        threading.Thread(target=worker, daemon=True, name="go_home_worker").start()

    def _on_go_home_done(self, ok: bool, err) -> None:
        self._moving = False
        self.btn_ocr.configure(state=tk.NORMAL)
        self._set_grip_buttons(True)
        self._set_auto_motion_enabled(True)
        if err is not None:
            self.status_var.set(f"기본자세 교정 실패: {err}")
            messagebox.showerror("기본자세", str(err), parent=self.root)
            return
        if ok:
            self.status_var.set("기본자세 교정 완료")
            self.on_joint_refresh()
        else:
            self.status_var.set("기본자세 교정 실패")

    def on_manual_mode(self) -> None:
        if self.robot is None or not self.robot.connected:
            self.status_var.set("로봇 미연결 — 수동모드 불가")
            return
        if self._moving or self._grip_busy:
            self.status_var.set("다른 작업 중 — 모드 전환 대기")
            return

        self.status_var.set("수동조작모드로 전환 중...")

        def worker():
            ok = False
            err = None
            try:
                ok = self.robot.set_manual_mode()
            except Exception as exc:
                err = exc
            self.root.after(0, lambda: self._on_mode_done("수동", ok, err))

        threading.Thread(target=worker, daemon=True, name="mode_manual").start()

    def on_auto_mode(self) -> None:
        if self.robot is None or not self.robot.connected:
            self.status_var.set("로봇 미연결 — 자동모드 불가")
            return
        if self._moving or self._grip_busy:
            self.status_var.set("다른 작업 중 — 모드 전환 대기")
            return

        self.status_var.set("자동모드(AUTONOMOUS)로 전환 중...")

        def worker():
            ok = False
            err = None
            try:
                ok = self.robot.set_autonomous_mode(servo=True)
            except Exception as exc:
                err = exc
            self.root.after(0, lambda: self._on_mode_done("자동", ok, err))

        threading.Thread(target=worker, daemon=True, name="mode_auto").start()

    def _on_mode_done(self, kind: str, ok: bool, err) -> None:
        self._update_mode_label()
        if err is not None:
            self.status_var.set(f"{kind}모드 전환 실패: {err}")
            messagebox.showerror("모드", str(err), parent=self.root)
            return
        if not ok:
            self.status_var.set(f"{kind}모드 전환 실패")
            return
        if kind == "수동":
            self.status_var.set(
                "수동조작모드 — 펜던트/직접교시로 조작하세요. "
                "[자동모드]로 복귀 후 앱 이동 가능"
            )
            self._set_auto_motion_enabled(False)
        else:
            self.status_var.set("자동모드 — 앱에서 이동/조인트 명령 가능")
            self._set_auto_motion_enabled(True)

    def on_execute(self) -> None:
        if self._moving or self._grip_busy:
            return
        if self.robot is None or not self.robot.connected:
            self.status_var.set("로봇 미연결 — 이동 불가")
            return
        if self.robot.is_manual_mode():
            self.status_var.set("수동조작모드입니다 — [자동모드]로 전환 후 실행하세요")
            return
        target = self._build_target_posx()
        if target is None:
            self.status_var.set("유효한 목표가 없습니다 (컨투어 선택·base XYZ 확인)")
            return

        ok_ws, ws_msg = workspace_ok(np.array(target[:3]))
        if not ok_ws:
            self.status_var.set(f"이동 거부 — {ws_msg}")
            print(self.status_var.get())
            return

        cur = self.robot.get_posx_mm_deg()
        if cur is not None:
            step = float(np.linalg.norm(np.array(target[:3]) - np.array(cur[:3])))
            if step > MAX_STEP_MM:
                self.status_var.set(
                    f"이동 거부 — 한 번에 {step:.0f}mm (한도 {MAX_STEP_MM:.0f}mm)"
                )
                print(self.status_var.get())
                return

        self._moving = True
        self.btn_exec.configure(state=tk.DISABLED)
        self.btn_ocr.configure(state=tk.DISABLED)
        self._set_grip_buttons(False)
        self.status_var.set("그리퍼 자세 확인 후 이동...")
        r = self.results[self.selected_idx] if self.selected_idx is not None else None
        z_surf = (
            float(r["base_mm"][2])
            if r is not None and r.get("base_mm") is not None
            else target[2]
        )
        skew = float(r.get("skew_img_deg", 0.0)) if r is not None else 0.0
        yaw = float(r.get("yaw_base_deg", 0.0)) if r is not None else 0.0
        print()
        print("=" * 50)
        print(
            f"그리퍼 TCP '{GRIPPER_TCP_NAME}' pos={GRIPPER_TCP_POS}"
        )
        print(
            f"Z 보정: surface={z_surf:.1f} + 그리퍼높이={TARGET_Z_UP_MM:.1f} "
            f"→ tip_Z={target[2]:.1f} mm"
        )
        print(
            f"컨투어 각도(짧은 변): image_skew={skew:.1f}° → base_yaw={yaw:.1f}°"
        )
        print(
            "실행 target (자세 → XY → Z): "
            f"[{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}, "
            f"{target[3]:.2f}, {target[4]:.2f}, {target[5]:.2f}]"
        )

        def worker():
            ok = False
            actual = None
            err = None
            try:
                if not self.robot.gripper_ready:
                    self.robot.ensure_gripper_tcp()

                def phase(p: str) -> None:
                    if p == "rotate":
                        msg = "그리퍼 자세 조정 중..."
                    elif p == "move_xy":
                        msg = "목표 XY로 이동 중 (Z 유지)..."
                    elif p == "move_z":
                        msg = "목표 Z로 이동 중..."
                    else:
                        msg = "목표로 이동 중..."
                    self.root.after(0, lambda m=msg: self.status_var.set(m))

                # 이동 전 그리퍼 열기 완료 → 그다음 자세/이동
                self.root.after(
                    0,
                    lambda: self.status_var.set("이동 전 그리퍼 열기 (완료 대기)..."),
                )
                if not self.robot.gripper_open():
                    print("[앱] 이동 전 그리퍼 열기 실패")
                    ok = False
                else:
                    ok = self.robot.movel_gripper(
                        target[:3],
                        vel=[VEL_XY, VEL_ORI],
                        acc=[ACC_XY, ACC_ORI],
                        straighten_first=True,
                        rotate_first=True,
                        z_up_mm=0.0,
                        ori_deg=target[3:6],
                        phase_cb=phase,
                    )
                if ok:
                    actual = self.robot.get_posx_mm_deg()
                    if actual is not None:
                        err = float(
                            np.linalg.norm(
                                np.array(actual[:3]) - np.array(target[:3])
                            )
                        )
            except Exception as exc:
                print(f"[앱] movel 예외: {exc}")
                ok = False
            self.root.after(
                0, lambda: self._on_move_done(ok, actual, err, target)
            )

        threading.Thread(target=worker, daemon=True, name="movel_worker").start()

    def _on_move_done(self, ok, actual, err, target) -> None:
        self._moving = False
        self.btn_ocr.configure(state=tk.NORMAL)
        self._set_grip_buttons(True)
        if not ok:
            self.status_var.set("이동 실패")
            if self.selected_idx is not None and self.robot and self.robot.connected:
                self.btn_exec.configure(state=tk.NORMAL)
            return

        if actual is None:
            self.status_var.set("이동 완료 (pose 읽기 실패)")
        else:
            self.pose_var.set(
                f"Robot pose [mm/deg]: "
                f"X={actual[0]:.1f} Y={actual[1]:.1f} Z={actual[2]:.1f}  "
                f"Rx={actual[3]:.1f} Ry={actual[4]:.1f} Rz={actual[5]:.1f}"
            )
            self.status_var.set(
                f"도착 완료 | XYZ 오차 ≈ {err:.1f} mm"
                if err is not None
                else "도착 완료"
            )
            print("실제 로봇 pose [mm, deg]:")
            print(
                f"  [{actual[0]:.2f}, {actual[1]:.2f}, {actual[2]:.2f}, "
                f"{actual[3]:.2f}, {actual[4]:.2f}, {actual[5]:.2f}]"
            )
            if err is not None:
                print(f"  목표 대비 XYZ 오차 ≈ {err:.2f} mm")
            print("=" * 50)

        if self.selected_idx is not None and self.robot and self.robot.connected:
            self.btn_exec.configure(state=tk.NORMAL)

    def _set_grip_buttons(self, enabled: bool) -> None:
        can = bool(
            enabled and self.robot is not None and self.robot.connected
        )
        state = tk.NORMAL if can else tk.DISABLED
        for btn in (
            getattr(self, "btn_grip_open", None),
            getattr(self, "btn_grip_close", None),
            getattr(self, "btn_grip_custom", None),
            getattr(self, "btn_grip_settings", None),
            getattr(self, "btn_joint_refresh", None),
            getattr(self, "btn_joint_move", None),
        ):
            if btn is not None:
                btn.configure(state=state)
        if can and hasattr(self, "btn_grip_custom"):
            self.btn_grip_custom.configure(
                text=f"커스텀닫기 ({self.custom_close_pos})"
            )

    def _apply_joint_vars(self, posj: list[float] | None) -> None:
        if not hasattr(self, "joint_vars"):
            return
        if posj is None or len(posj) < 6:
            self.joint_status.set("조인트: 읽기 실패")
            return
        for i in range(6):
            self.joint_vars[i].set(f"{float(posj[i]):.2f}")
        self.joint_status.set(
            "조인트: "
            + "  ".join(f"J{i+1}={float(posj[i]):.1f}" for i in range(6))
        )

    def _apply_tcp_xyz(self, posx: list[float] | None) -> None:
        if not hasattr(self, "tcp_xyz_vars"):
            return
        if posx is None or len(posx) < 3:
            for axis in ("X", "Y", "Z"):
                self.tcp_xyz_vars[axis].set("-")
            if hasattr(self, "tcp_ori_var"):
                self.tcp_ori_var.set("ori: -")
            self.tcp_status.set("TCP: 읽기 실패")
            return
        self.tcp_xyz_vars["X"].set(f"{float(posx[0]):.2f}")
        self.tcp_xyz_vars["Y"].set(f"{float(posx[1]):.2f}")
        self.tcp_xyz_vars["Z"].set(f"{float(posx[2]):.2f}")
        if len(posx) >= 6 and hasattr(self, "tcp_ori_var"):
            self.tcp_ori_var.set(
                f"ori=({float(posx[3]):.1f}, {float(posx[4]):.1f}, {float(posx[5]):.1f})"
            )
        self.tcp_status.set("")

    def _poll_joints(self) -> None:
        if self._closed:
            return
        if (
            self.robot is not None
            and self.robot.connected
            and not self._moving
            and not self._grip_busy
        ):
            # 입력 중이면 조인트 덮어쓰지 않도록 포커스 검사
            focused = self.root.focus_get()
            editing = any(focused is e for e in getattr(self, "joint_entries", []))
            try:
                if not editing:
                    posj = self.robot.get_posj_deg()
                    self._apply_joint_vars(posj)
                posx = self.robot.get_posx_mm_deg()
                self._apply_tcp_xyz(posx)
            except Exception as exc:
                print(f"[조인트/TCP] poll: {exc}")
        if not self._closed:
            self.root.after(800, self._poll_joints)

    def on_joint_refresh(self) -> None:
        if self.robot is None or not self.robot.connected:
            self.joint_status.set("조인트: 로봇 미연결")
            if hasattr(self, "tcp_status"):
                self.tcp_status.set("TCP: 로봇 미연결")
            return
        posj = self.robot.get_posj_deg()
        self._apply_joint_vars(posj)
        posx = self.robot.get_posx_mm_deg()
        self._apply_tcp_xyz(posx)
        if posj is not None or posx is not None:
            self.status_var.set("조인트/그리퍼 TCP 값 갱신됨")

    def on_joint_move(self) -> None:
        if self.robot is None or not self.robot.connected:
            self.status_var.set("로봇 미연결 — 조인트 이동 불가")
            return
        if self.robot.is_manual_mode():
            self.status_var.set("수동조작모드 — [자동모드]로 전환 후 조인트 이동하세요")
            return
        if self._moving or self._grip_busy:
            self.status_var.set("다른 작업 중 — 조인트 이동 대기")
            return
        try:
            posj = [float(v.get()) for v in self.joint_vars]
        except Exception:
            messagebox.showerror(
                "조인트", "J1~J6에 숫자를 입력하세요.", parent=self.root
            )
            return
        if len(posj) != 6:
            return

        self._moving = True
        self.btn_exec.configure(state=tk.DISABLED)
        self.btn_ocr.configure(state=tk.DISABLED)
        self._set_grip_buttons(False)
        self.status_var.set("조인트 이동 중 (movej)...")
        print(
            "[앱] movej → "
            + ", ".join(f"J{i+1}={posj[i]:.2f}" for i in range(6))
        )

        def worker():
            ok = False
            err = None
            try:
                ok = self.robot.movej(posj, vel=25.0, acc=25.0)
            except Exception as exc:
                err = exc
                print(f"[앱] movej 예외: {exc}")
            self.root.after(0, lambda: self._on_joint_move_done(ok, err))

        threading.Thread(target=worker, daemon=True, name="movej_worker").start()

    def _on_joint_move_done(self, ok: bool, err) -> None:
        self._moving = False
        self.btn_ocr.configure(state=tk.NORMAL)
        self._set_grip_buttons(True)
        if self.selected_idx is not None and self.robot and self.robot.connected:
            self.btn_exec.configure(state=tk.NORMAL)
        if err is not None:
            self.status_var.set(f"조인트 이동 실패: {err}")
            messagebox.showerror("조인트", str(err), parent=self.root)
            return
        if ok:
            self.status_var.set("조인트 이동 완료")
            self.on_joint_refresh()
        else:
            self.status_var.set("조인트 이동 실패")

    def _run_gripper(self, label: str, fn) -> None:
        if self.robot is None or not self.robot.connected:
            self.status_var.set("로봇 미연결 — 그리퍼 불가")
            return
        if self._moving or self._grip_busy or self._busy:
            self.status_var.set("다른 작업 중 — 그리퍼 대기")
            return

        self._grip_busy = True
        self._set_grip_buttons(False)
        self.status_var.set(f"그리퍼 {label} 중...")

        def worker():
            ok = False
            err = None
            try:
                ok = bool(fn())
            except Exception as exc:
                err = exc
                print(f"[그리퍼] {label} 예외: {exc}")
            self.root.after(0, lambda: self._on_grip_done(label, ok, err))

        threading.Thread(target=worker, daemon=True, name="gripper_worker").start()

    def _on_grip_done(self, label: str, ok: bool, err) -> None:
        self._grip_busy = False
        self._set_grip_buttons(True)
        if err is not None:
            self.status_var.set(f"그리퍼 {label} 실패: {err}")
            messagebox.showerror("그리퍼", str(err), parent=self.root)
            return
        if ok:
            self.status_var.set(f"그리퍼 {label} 완료")
            self.grip_var.set(
                f"그리퍼: 마지막={label}  |  커스텀닫기={self.custom_close_pos}"
            )
        else:
            self.status_var.set(
                f"그리퍼 {label} 실패 — gripper_service 실행 여부 확인"
            )

    def on_grip_open(self) -> None:
        assert self.robot is not None
        self._run_gripper("열기", self.robot.gripper_open)

    def on_grip_close(self) -> None:
        assert self.robot is not None
        self._run_gripper("닫기", self.robot.gripper_close)

    def on_grip_custom_close(self) -> None:
        assert self.robot is not None
        self.robot.set_custom_close(self.custom_close_pos, self.custom_close_current)
        self._run_gripper(
            f"커스텀닫기({self.custom_close_pos})",
            self.robot.gripper_custom_close,
        )

    def on_grip_settings(self) -> None:
        """커스텀닫기 stroke(0~750)와 전류 설정. 기본 반닫힘=375."""
        if self.robot is None or not self.robot.connected:
            messagebox.showwarning("그리퍼 설정", "로봇이 연결되지 않았습니다.", parent=self.root)
            return

        dlg = tk.Toplevel(self.root)
        dlg.title("그리퍼 설정 — 커스텀닫기")
        dlg.transient(self.root)
        dlg.grab_set()
        dlg.resizable(False, False)

        frm = ttk.Frame(dlg, padding=12)
        frm.pack(fill=tk.BOTH, expand=True)

        ttk.Label(
            frm,
            text=(
                "커스텀닫기 위치 (0=완전닫힘, 750=완전열림)\n"
                f"권장 반만 닫힘: {GRIPPER_POS_HALF}"
            ),
        ).grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=(0, 8))

        pos_var = tk.IntVar(value=int(self.custom_close_pos))
        cur_var = tk.IntVar(value=int(self.custom_close_current))

        ttk.Label(frm, text="닫는 정도 (position)").grid(row=1, column=0, sticky=tk.W)
        pos_spin = ttk.Spinbox(
            frm, from_=0, to=750, textvariable=pos_var, width=8
        )
        pos_spin.grid(row=1, column=1, sticky=tk.W, padx=6)
        ttk.Button(
            frm,
            text="반만(375)",
            command=lambda: pos_var.set(int(GRIPPER_POS_HALF)),
        ).grid(row=1, column=2, padx=4)

        ttk.Label(frm, text="전류/힘 (current)").grid(
            row=2, column=0, sticky=tk.W, pady=(8, 0)
        )
        ttk.Spinbox(frm, from_=0, to=1000, textvariable=cur_var, width=8).grid(
            row=2, column=1, sticky=tk.W, padx=6, pady=(8, 0)
        )

        scale = ttk.Scale(
            frm,
            from_=0,
            to=750,
            orient=tk.HORIZONTAL,
            command=lambda v: pos_var.set(int(float(v))),
        )
        scale.set(float(self.custom_close_pos))
        scale.grid(row=3, column=0, columnspan=3, sticky=tk.EW, pady=10)

        def _sync_scale(*_a):
            try:
                scale.set(float(pos_var.get()))
            except Exception:
                pass

        pos_var.trace_add("write", _sync_scale)

        btns = ttk.Frame(frm)
        btns.grid(row=4, column=0, columnspan=3, sticky=tk.E)

        def on_ok():
            try:
                pos = int(pos_var.get())
                cur = int(cur_var.get())
            except Exception:
                messagebox.showerror("입력 오류", "숫자를 입력하세요.", parent=dlg)
                return
            pos = max(0, min(750, pos))
            cur = max(0, cur)
            self.custom_close_pos = pos
            self.custom_close_current = cur
            if self.robot is not None:
                self.robot.set_custom_close(pos, cur)
            save_gripper_settings(pos, cur)
            self.grip_var.set(
                f"그리퍼: 커스텀닫기={pos} (current={cur})"
            )
            self.btn_grip_custom.configure(text=f"커스텀닫기 ({pos})")
            self.status_var.set(f"커스텀닫기 설정 저장: pos={pos}")
            dlg.destroy()

        ttk.Button(btns, text="저장", command=on_ok).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="취소", command=dlg.destroy).pack(side=tk.LEFT)

        dlg.wait_window()

    def on_save(self) -> None:
        if self.frozen_frame is None and self.last_frame is None:
            messagebox.showwarning("저장", "프레임이 없습니다.", parent=self.root)
            return
        frame = self.frozen_frame if self.frozen_frame is not None else self.last_frame
        assert frame is not None
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        vis = draw_ocr_with_selection(frame, self.results, self.selected_idx)
        out_path = SAVE_DIR / f"ocr_move_{int(time.time())}.png"
        cv2.imwrite(str(out_path), vis)
        print(f"저장: {out_path}")
        self.status_var.set(f"저장됨: {out_path.name}")

    def on_close(self) -> None:
        self._closed = True
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.robot is not None:
            try:
                self.robot.close()
            except Exception:
                pass
        self.root.destroy()


def main() -> None:
    print("EasyOCR 로딩...")
    reader = ocr.create_reader()
    print(f"캘리브 경로 확인: {CALIB_NPZ} exists={CALIB_NPZ.is_file()}")
    R, t, meta = load_cam2base()
    warn_if_bad_calib(meta)
    print("P_base = R @ P_cam + t  (좌표는 로봇 base mm)")
    print(
        f"그리퍼 TCP '{GRIPPER_TCP_NAME}' = {GRIPPER_TCP_POS} mm/deg "
        f"(플랜지→tip, doosan_ws tcp/config_create_tcp)"
    )
    print(f"하향 정자세 ori = {GRIPPER_DOWN_ORI_DEG}")
    print(
        "기동 기본자세 J1..J6 = "
        + "["
        + ", ".join(f"{v:.0f}" for v in HOME_POSJ_DEG)
        + "] deg"
    )
    print(f"목표 Z 위로 보정 TARGET_Z_UP_MM = {TARGET_Z_UP_MM:.1f} mm (그리퍼 높이)")

    robot = DoosanClient(
        robot_id=ROBOT_ID,
        robot_model=ROBOT_MODEL,
        ws=DOOSAN_WS,
    )
    # 연결 시 TCP 설정 + 기본 조인트 자세(0,0,90,0,90,0) 교정 후 GUI 시작
    robot_ok = robot.connect_and_set_autonomous(
        setup_gripper=True,
        straighten_down=True,
    )
    if not robot_ok:
        print("로봇 없이 좌표 표시만 진행합니다. (실행 버튼 비활성)")
        robot = None

    root = tk.Tk()
    OcrClickMoveApp(root, reader, R, t, meta, robot)
    root.mainloop()


if __name__ == "__main__":
    main()
