"""
OpenCV로 일정 크기 이상 영역(컨투어)을 따서 화면에 표시합니다.

표시는 tkinter 사용 (OpenCV highgui/GTK 미빌드 환경 대응).

- RealSense 실시간 또는 이미지 파일 입력
- Otsu 이진화 + 외곽 컨투어
- MIN_AREA_PX / MIN_AREA_RATIO 이상만 표시

실행
  python3 opencv_large_regions.py
  python3 opencv_large_regions.py /path/to/image.jpg

버튼 / 키
  [min_px −/+]  [min_ratio −/+]  [반전]  [모폴로지]  [저장]  [일시정지]  [종료]
"""

from __future__ import annotations

import base64
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
WINDOW_NAME = "OpenCV Large Regions"
SAVE_DIR = Path(__file__).resolve().parent / "regions_out"

MIN_AREA_PX = 800
MAX_AREA_PX = None
MIN_AREA_RATIO = 0.001
MAX_AREA_RATIO = 0.35

BLUR_KSIZE = 5
MORPH_ON = True
MORPH_KERNEL = 5
USE_INVERT = True

DRAW_FILL_ALPHA = 0.25


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def find_large_regions(
    bgr: np.ndarray,
    *,
    min_area_px: float = MIN_AREA_PX,
    max_area_px: float | None = MAX_AREA_PX,
    min_area_ratio: float = MIN_AREA_RATIO,
    max_area_ratio: float = MAX_AREA_RATIO,
    use_invert: bool = USE_INVERT,
    morph_on: bool = MORPH_ON,
) -> tuple[list[dict], np.ndarray]:
    h, w = bgr.shape[:2]
    img_area = float(h * w)

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    k = BLUR_KSIZE if BLUR_KSIZE % 2 == 1 else BLUR_KSIZE + 1
    blur = cv2.GaussianBlur(gray, (k, k), 0)

    flag = (
        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        if use_invert
        else cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    _, mask = cv2.threshold(blur, 0, 255, flag)

    if morph_on:
        ker = cv2.getStructuringElement(
            cv2.MORPH_RECT, (MORPH_KERNEL, MORPH_KERNEL)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ker, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ker, iterations=1)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    regions: list[dict] = []
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < float(min_area_px):
            continue
        ratio = area / img_area if img_area > 0 else 0.0
        if ratio < float(min_area_ratio):
            continue
        if ratio > float(max_area_ratio):
            continue
        if max_area_px is not None and area > float(max_area_px):
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)
        m = cv2.moments(cnt)
        if m["m00"] > 1e-6:
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        else:
            cx, cy = x + bw * 0.5, y + bh * 0.5

        rect = cv2.minAreaRect(cnt)
        quad = cv2.boxPoints(rect).astype(np.float32)

        regions.append(
            {
                "area": area,
                "area_ratio": ratio,
                "center": (float(cx), float(cy)),
                "bbox": (int(x), int(y), int(bw), int(bh)),
                "contour": cnt,
                "quad": quad,
            }
        )

    regions.sort(key=lambda d: d["area"], reverse=True)
    for i, r in enumerate(regions):
        r["idx"] = i
    return regions, mask


def draw_regions(
    bgr: np.ndarray,
    regions: list[dict],
    *,
    mask: np.ndarray | None = None,
    hud: str = "",
    use_invert: bool = True,
    morph_on: bool = True,
) -> np.ndarray:
    out = bgr.copy()
    overlay = out.copy()

    colors = [
        (40, 180, 40),
        (0, 165, 255),
        (255, 128, 0),
        (200, 80, 255),
        (0, 220, 220),
        (80, 80, 255),
    ]

    for r in regions:
        color = colors[r["idx"] % len(colors)]
        cnt = r["contour"]
        cv2.drawContours(overlay, [cnt], -1, color, -1)
        cv2.drawContours(out, [cnt], -1, color, 2)
        quad = r["quad"].astype(np.int32)
        cv2.polylines(out, [quad], True, (0, 0, 255), 1, cv2.LINE_AA)

        cx, cy = int(round(r["center"][0])), int(round(r["center"][1]))
        label = f"#{r['idx']} {r['area']:.0f}px ({r['area_ratio']*100:.2f}%)"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        tx = max(0, min(cx - tw // 2, out.shape[1] - tw - 2))
        ty = max(th + 4, cy - 8)
        cv2.rectangle(
            out, (tx - 2, ty - th - 4), (tx + tw + 2, ty + 4), (0, 0, 0), -1
        )
        cv2.putText(
            out,
            label,
            (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.circle(out, (cx, cy), 4, (0, 0, 255), -1, cv2.LINE_AA)

    cv2.addWeighted(overlay, DRAW_FILL_ALPHA, out, 1.0 - DRAW_FILL_ALPHA, 0, out)

    lines = [
        hud,
        f"regions={len(regions)}  invert={'ON' if use_invert else 'OFF'}  "
        f"morph={'ON' if morph_on else 'OFF'}",
    ]
    y0 = 22
    for line in lines:
        if not line:
            continue
        cv2.putText(
            out, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 3, cv2.LINE_AA
        )
        cv2.putText(
            out, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1, cv2.LINE_AA
        )
        y0 += 22

    if mask is not None:
        mh = min(180, out.shape[0] // 3)
        mw = int(round(mask.shape[1] * (mh / mask.shape[0])))
        mask_s = cv2.resize(mask, (mw, mh), interpolation=cv2.INTER_NEAREST)
        mask_bgr = cv2.cvtColor(mask_s, cv2.COLOR_GRAY2BGR)
        x0 = out.shape[1] - mw - 10
        y1 = 10
        if x0 > 0:
            out[y1 : y1 + mh, x0 : x0 + mw] = mask_bgr
            cv2.rectangle(
                out, (x0 - 1, y1 - 1), (x0 + mw, y1 + mh), (200, 200, 200), 1
            )
            cv2.putText(
                out,
                "mask",
                (x0, y1 + mh + 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )

    return out


def process_frame(
    bgr: np.ndarray,
    *,
    min_area_px: float,
    min_area_ratio: float,
    use_invert: bool,
    morph_on: bool,
) -> tuple[np.ndarray, list[dict]]:
    regions, mask = find_large_regions(
        bgr,
        min_area_px=min_area_px,
        min_area_ratio=min_area_ratio,
        use_invert=use_invert,
        morph_on=morph_on,
    )
    hud = (
        f"min_px={min_area_px:.0f}  min_ratio={min_area_ratio:.4f}  "
        f"max_ratio={MAX_AREA_RATIO:.2f}"
    )
    vis = draw_regions(
        bgr,
        regions,
        mask=mask,
        hud=hud,
        use_invert=use_invert,
        morph_on=morph_on,
    )
    return vis, regions


def save_result(vis: np.ndarray, regions: list[dict]) -> Path:
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    img_path = SAVE_DIR / f"regions_{stamp}.png"
    txt_path = SAVE_DIR / f"regions_{stamp}.txt"
    cv2.imwrite(str(img_path), vis)
    lines = [
        "# idx area_px area_ratio center_x center_y bbox_x bbox_y bbox_w bbox_h"
    ]
    for r in regions:
        x, y, bw, bh = r["bbox"]
        cx, cy = r["center"]
        lines.append(
            f"{r['idx']}  {r['area']:.1f}  {r['area_ratio']:.6f}  "
            f"{cx:.1f}  {cy:.1f}  {x}  {y}  {bw}  {bh}"
        )
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[저장] {img_path}")
    print(f"[저장] {txt_path}")
    return img_path


def bgr_to_photo(bgr: np.ndarray) -> tk.PhotoImage:
    ok, buf = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError("이미지 인코딩 실패")
    return tk.PhotoImage(data=base64.b64encode(buf.tobytes()))


def fit_to_label(bgr: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    h, w = bgr.shape[:2]
    max_w = max(int(max_w), 1)
    max_h = max(int(max_h), 1)
    scale = min(max_w / w, max_h / h, 1.0)
    if scale >= 0.999:
        return bgr
    return cv2.resize(
        bgr,
        (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
        interpolation=cv2.INTER_AREA,
    )


class RegionViewerApp:
    def __init__(self, root: tk.Tk, image_path: Path | None = None):
        self.root = root
        self.image_path = image_path
        self._closed = False
        self._paused = False
        self._photo = None

        self.min_px = float(MIN_AREA_PX)
        self.min_ratio = float(MIN_AREA_RATIO)
        self.use_invert = bool(USE_INVERT)
        self.morph_on = bool(MORPH_ON)

        self.last_bgr: np.ndarray | None = None
        self.last_vis: np.ndarray | None = None
        self.last_regions: list[dict] = []

        self.pipeline = None
        self.static_bgr: np.ndarray | None = None

        root.title(WINDOW_NAME)
        root.geometry("1280x800")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.status_var = tk.StringVar(value="준비")
        ttk.Label(root, textvariable=self.status_var).pack(
            fill=tk.X, padx=8, pady=(8, 0)
        )

        btns = ttk.Frame(root, padding=8)
        btns.pack(fill=tk.X)
        ttk.Button(btns, text="min_px −", command=self.on_min_px_dec).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ttk.Button(btns, text="min_px +", command=self.on_min_px_inc).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(btns, text="min_ratio −", command=self.on_min_ratio_dec).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ttk.Button(btns, text="min_ratio +", command=self.on_min_ratio_inc).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(btns, text="반전", command=self.on_toggle_invert).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ttk.Button(btns, text="모폴로지", command=self.on_toggle_morph).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Button(btns, text="저장", command=self.on_save).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        self.btn_pause = ttk.Button(
            btns, text="일시정지", command=self.on_toggle_pause
        )
        self.btn_pause.pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(btns, text="종료", command=self.on_close).pack(side=tk.LEFT)

        self.video = tk.Label(root)
        self.video.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        root.bind("<KeyPress-bracketleft>", lambda _e: self.on_min_px_dec())
        root.bind("<KeyPress-bracketright>", lambda _e: self.on_min_px_inc())
        root.bind("<KeyPress-i>", lambda _e: self.on_toggle_invert())
        root.bind("<KeyPress-m>", lambda _e: self.on_toggle_morph())
        root.bind("<KeyPress-s>", lambda _e: self.on_save())
        root.bind("<space>", lambda _e: self.on_toggle_pause())
        root.bind("<Escape>", lambda _e: self.on_close())
        root.bind("<KeyPress-q>", lambda _e: self.on_close())

        if image_path is not None:
            self.static_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if self.static_bgr is None:
                raise SystemExit(f"이미지를 열 수 없습니다: {image_path}")
            self.btn_pause.configure(state=tk.DISABLED)
            self.status_var.set(f"이미지: {image_path}")
            print(f"이미지: {image_path}  shape={self.static_bgr.shape}")
            self.root.after(10, self.update_static)
        else:
            self._start_realsense()
            self.root.after(10, self.update_camera)

    def _start_realsense(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise SystemExit(
                "pyrealsense2 가 없습니다. 이미지 모드로 실행하세요.\n"
                "  python3 opencv_large_regions.py image.jpg"
            ) from exc

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.status_var.set("RealSense 시작")
        print("RealSense 시작 (tkinter 표시)")

    def _show(self, bgr: np.ndarray) -> None:
        self.root.update_idletasks()
        lw = max(self.video.winfo_width(), 640)
        lh = max(self.video.winfo_height(), 360)
        if lw < 40 or lh < 40:
            lw, lh = 960, 540
        disp = fit_to_label(bgr, lw, lh)
        self._photo = bgr_to_photo(disp)
        self.video.configure(image=self._photo)

    def _reprocess_last(self) -> None:
        if self.last_bgr is None:
            return
        self.last_vis, self.last_regions = process_frame(
            self.last_bgr,
            min_area_px=self.min_px,
            min_area_ratio=self.min_ratio,
            use_invert=self.use_invert,
            morph_on=self.morph_on,
        )
        self._show(self.last_vis)
        self._update_status()

    def _update_status(self) -> None:
        n = len(self.last_regions)
        mode = "PAUSE" if self._paused else ("IMG" if self.static_bgr is not None else "LIVE")
        self.status_var.set(
            f"[{mode}] regions={n}  min_px={self.min_px:.0f}  "
            f"min_ratio={self.min_ratio:.5f}  "
            f"invert={'ON' if self.use_invert else 'OFF'}  "
            f"morph={'ON' if self.morph_on else 'OFF'}"
        )

    def update_static(self) -> None:
        if self._closed or self.static_bgr is None:
            return
        self.last_bgr = self.static_bgr
        self._reprocess_last()
        # 파라미터만 바꾸는 경우 on_* 에서 재처리 — 루프 불필요

    def update_camera(self) -> None:
        if self._closed or self.pipeline is None:
            return
        try:
            if not self._paused:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                color = frames.get_color_frame()
                if color:
                    bgr = np.asanyarray(color.get_data())
                    self.last_bgr = bgr
                    self.last_vis, self.last_regions = process_frame(
                        bgr,
                        min_area_px=self.min_px,
                        min_area_ratio=self.min_ratio,
                        use_invert=self.use_invert,
                        morph_on=self.morph_on,
                    )
                    self._show(self.last_vis)
                    self._update_status()
            elif self.last_vis is not None:
                self._show(self.last_vis)
        except Exception as exc:
            if not self._closed:
                print(f"[카메라] {exc}")
        if not self._closed:
            self.root.after(30, self.update_camera)

    def on_min_px_dec(self) -> None:
        self.min_px = max(50.0, self.min_px * 0.8)
        print(f"min_area_px → {self.min_px:.0f}")
        self._reprocess_last()

    def on_min_px_inc(self) -> None:
        self.min_px = self.min_px * 1.25
        print(f"min_area_px → {self.min_px:.0f}")
        self._reprocess_last()

    def on_min_ratio_dec(self) -> None:
        self.min_ratio = max(1e-5, self.min_ratio * 0.8)
        print(f"min_area_ratio → {self.min_ratio:.5f}")
        self._reprocess_last()

    def on_min_ratio_inc(self) -> None:
        self.min_ratio = _clamp(self.min_ratio * 1.25, 1e-5, MAX_AREA_RATIO * 0.9)
        print(f"min_area_ratio → {self.min_ratio:.5f}")
        self._reprocess_last()

    def on_toggle_invert(self) -> None:
        self.use_invert = not self.use_invert
        print(f"invert → {self.use_invert}")
        self._reprocess_last()

    def on_toggle_morph(self) -> None:
        self.morph_on = not self.morph_on
        print(f"morph → {self.morph_on}")
        self._reprocess_last()

    def on_toggle_pause(self) -> None:
        if self.static_bgr is not None:
            return
        self._paused = not self._paused
        self.btn_pause.configure(text="재개" if self._paused else "일시정지")
        print("일시정지" if self._paused else "재개")
        self._update_status()

    def on_save(self) -> None:
        if self.last_vis is None:
            self.status_var.set("저장할 프레임이 없습니다")
            return
        path = save_result(self.last_vis, self.last_regions)
        self.status_var.set(f"저장됨: {path.name}")

    def on_close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            print("RealSense 종료")
        self.root.destroy()


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    image_path = Path(argv[0]) if argv else None
    root = tk.Tk()
    RegionViewerApp(root, image_path=image_path)
    root.mainloop()


if __name__ == "__main__":
    main()
