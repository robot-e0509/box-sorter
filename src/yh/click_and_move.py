"""
Eye-to-Hand 캘리브 결과를 사용해
  1) RealSense 영상에서 마우스 클릭 → 베이스 XYZ 표시
  2) 실행 버튼 → 그리퍼(TCP)를 그 좌표로 movel
  3) 이동 후 실제 로봇 pose 출력

GUI는 OpenCV Qt highgui 대신 tkinter 사용 (setMouseCallback 오류 회피).

실행:
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /home/newuser/ocr/doosan_ws/install/setup.bash
  # bringup은 다른 터미널에서 실행 중이어야 함
  python3 click_and_move.py
"""

from __future__ import annotations

import base64
import json
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import cv2
import numpy as np
import pyrealsense2 as rs

from doosan_client import (
    DEFAULT_ROBOT_ID,
    DEFAULT_ROBOT_MODEL,
    DOOSAN_WS,
    DoosanClient,
    TARGET_Z_UP_MM,
)

# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
CALIB_NPZ = Path(__file__).resolve().parent / "hand_eye_data" / "cam2base.npz"
CALIB_JSON = Path(__file__).resolve().parent / "hand_eye_data" / "cam2base.json"

ROBOT_ID = DEFAULT_ROBOT_ID
ROBOT_MODEL = DEFAULT_ROBOT_MODEL

VEL_XY = 40.0
VEL_ORI = 40.0
ACC_XY = 40.0
ACC_ORI = 40.0

Z_OFFSET_MM = 0.0
# 그리퍼 tip 기준 + 하향(정자세). keep 사용 시 교정 후에도 목표 ori가 흔들릴 수 있음
ORI_MODE = "fixed"
FIXED_ORI_DEG = [0.0, 180.0, 0.0]  # doosan_client.GRIPPER_DOWN_ORI_DEG 와 동일

# e0509 대략 작업 공간 (베이스 기준, mm) — 벗어나면 이동 거부
WORKSPACE_MIN_MM = np.array([-700.0, -700.0, 50.0])
WORKSPACE_MAX_MM = np.array([700.0, 700.0, 900.0])
# 현재 TCP에서 한 번에 허용할 최대 이동 거리
MAX_STEP_MM = 400.0


def load_cam2base() -> tuple[np.ndarray, np.ndarray, dict]:
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
        print(f"캘리브 로드: {CALIB_JSON}")
        return R, t, meta

    raise FileNotFoundError(
        f"캘리브 파일이 없습니다:\n  {CALIB_NPZ}\n  {CALIB_JSON}\n"
        "먼저 hand_eye_calib.py 에서 c 키로 캘리브를 완료하세요."
    )


def warn_if_bad_calib(meta: dict) -> None:
    residual = meta.get("residual_mm") or {}
    mean_mm = residual.get("mean_mm")
    if mean_mm is None:
        return
    print(f"캘리브 잔차 mean={mean_mm:.1f} mm, max={residual.get('max_mm', float('nan')):.1f} mm")
    if mean_mm > 20.0:
        print()
        print("!" * 60)
        print("경고: 핸드아이 캘리브가 거의 실패한 상태입니다.")
        print("      클릭 좌표가 로봇 작업공간을 크게 벗어날 수 있습니다.")
        print("      hand_eye_calib.py 로 다시 캘리브하세요.")
        print("      - SQUARE_SIZE_M / MARKER_SIZE_M 실측")
        print("      - 샘플 15~25개, 자세 다양하게")
        print("      - EULER_ORDER (ZYX↔XYZ) 확인")
        print("      - 목표 잔차: mean < 10mm")
        print("!" * 60)
        print()


def cam_to_base(P_cam_m: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return R @ P_cam_m.reshape(3) + t.reshape(3)


def workspace_ok(base_mm: np.ndarray) -> tuple[bool, str]:
    lo = WORKSPACE_MIN_MM
    hi = WORKSPACE_MAX_MM
    if np.any(base_mm < lo) or np.any(base_mm > hi):
        return False, (
            f"작업공간 밖: {base_mm.round(1)} "
            f"(허용 X[{lo[0]:.0f},{hi[0]:.0f}] "
            f"Y[{lo[1]:.0f},{hi[1]:.0f}] "
            f"Z[{lo[2]:.0f},{hi[2]:.0f}])"
        )
    return True, ""



class ClickAndMoveApp:
    def __init__(self, root: tk.Tk, R: np.ndarray, t: np.ndarray, robot: DoosanClient | None):
        self.root = root
        self.R = R
        self.t = t
        self.robot = robot
        self._moving = False

        self.state = {
            "depth_frame": None,
            "intrinsics": None,
            "width": 0,
            "height": 0,
            "clicked": None,
            "display_bgr": None,
        }

        self.root.title("Click Target → Move Gripper")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.status_var = tk.StringVar(value="영상을 클릭하세요")
        self.cam_var = tk.StringVar(value="Cam [m]: -")
        self.base_var = tk.StringVar(value="Base [mm]: -")
        self.pose_var = tk.StringVar(value="Robot pose: -")

        top = ttk.Frame(root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, textvariable=self.status_var, font=("Sans", 11)).pack(anchor=tk.W)
        ttk.Label(top, textvariable=self.cam_var).pack(anchor=tk.W)
        ttk.Label(top, textvariable=self.base_var).pack(anchor=tk.W)
        ttk.Label(top, textvariable=self.pose_var).pack(anchor=tk.W)

        btn_row = ttk.Frame(root, padding=8)
        btn_row.pack(fill=tk.X)
        self.exec_btn = ttk.Button(
            btn_row, text="실행 (이동)", command=self.on_execute, state=tk.DISABLED
        )
        self.exec_btn.pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(btn_row, text="종료", command=self.on_close).pack(side=tk.LEFT)
        robot_txt = "로봇: 연결됨 (AUTONOMOUS)" if robot and robot.connected else "로봇: 미연결 (좌표만)"
        ttk.Label(btn_row, text=robot_txt).pack(side=tk.RIGHT)

        self.video_label = tk.Label(root)
        self.video_label.pack(fill=tk.BOTH, expand=True)
        self.video_label.bind("<Button-1>", self.on_image_click)

        self._photo = None
        self._closed = False

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)
        self.colorizer = rs.colorizer()
        self.pipeline.start(config)

        print("조작: 영상 클릭 → XYZ 확인 → [실행] 버튼")
        self.root.after(10, self.update_frame)

    def on_image_click(self, event):
        display = self.state["display_bgr"]
        depth_frame = self.state["depth_frame"]
        intr = self.state["intrinsics"]
        w = self.state["width"]
        h = self.state["height"]
        if display is None or depth_frame is None or intr is None or w <= 0:
            return

        # Label에 표시된 이미지 크기 vs 원본 스케일
        lbl_w = max(self.video_label.winfo_width(), 1)
        lbl_h = max(self.video_label.winfo_height(), 1)
        img_h, img_w = display.shape[:2]
        scale = min(lbl_w / img_w, lbl_h / img_h)
        disp_w, disp_h = int(img_w * scale), int(img_h * scale)
        off_x = (lbl_w - disp_w) // 2
        off_y = (lbl_h - disp_h) // 2

        x = int((event.x - off_x) / scale)
        y = int((event.y - off_y) / scale)
        if not (0 <= x < img_w and 0 <= y < img_h):
            return

        # 왼쪽 color / 오른쪽 depth
        if x >= w:
            px, src = x - w, "Depth"
        else:
            px, src = x, "Color"
        py = y
        if not (0 <= px < w and 0 <= py < h):
            return

        depth_m = depth_frame.get_distance(int(px), int(py))
        if depth_m <= 0:
            self.status_var.set(f"깊이 없음 @ ({px},{py})")
            self.state["clicked"] = {"valid": False, "u": px, "v": py}
            self.exec_btn.configure(state=tk.DISABLED)
            print(self.status_var.get())
            return

        P_cam = np.array(
            rs.rs2_deproject_pixel_to_point(intr, [float(px), float(py)], depth_m),
            dtype=np.float64,
        )
        P_base = cam_to_base(P_cam, self.R, self.t)
        P_base_mm = P_base * 1000.0
        P_base_mm[2] += Z_OFFSET_MM

        self.state["clicked"] = {
            "valid": True,
            "u": int(px),
            "v": int(py),
            "src": src,
            "cam_m": P_cam,
            "base_mm": P_base_mm,
        }
        self.cam_var.set(
            f"Cam [m]:  X={P_cam[0]:.4f}  Y={P_cam[1]:.4f}  Z={P_cam[2]:.4f}"
        )
        self.base_var.set(
            f"Base [mm]: X={P_base_mm[0]:.1f}  Y={P_base_mm[1]:.1f}  Z={P_base_mm[2]:.1f}"
            f"   (그리퍼 TCP 목표)"
        )

        ok_ws, ws_msg = workspace_ok(P_base_mm)
        if not ok_ws:
            self.status_var.set(f"이동 불가 — {ws_msg}")
            self.exec_btn.configure(state=tk.DISABLED)
            print(self.status_var.get())
            print("→ 캘리브가 잘못된 경우가 많습니다. hand_eye_calib.py 를 다시 하세요.")
            return

        self.status_var.set(f"[{src}] 픽셀 ({px}, {py}) 선택됨 — 실행 버튼을 누르세요")
        print()
        print(self.status_var.get())
        print(self.cam_var.get())
        print(self.base_var.get())

        if self.robot is not None and self.robot.connected:
            self.exec_btn.configure(state=tk.NORMAL)
        else:
            self.exec_btn.configure(state=tk.DISABLED)
            self.status_var.set(self.status_var.get() + " (로봇 미연결)")

    def _build_target_posx(self) -> list[float] | None:
        clicked = self.state["clicked"]
        if not clicked or not clicked.get("valid"):
            return None
        x, y, z_surf = clicked["base_mm"]
        z = float(z_surf) + float(TARGET_Z_UP_MM)
        if ORI_MODE == "fixed":
            rx, ry, rz = FIXED_ORI_DEG
        else:
            if self.robot is None or not self.robot.connected:
                return None
            cur = self.robot.get_posx_mm_deg()
            if cur is None:
                return None
            rx, ry, rz = cur[3], cur[4], cur[5]
        return [float(x), float(y), float(z), float(rx), float(ry), float(rz)]

    def on_execute(self):
        if self._moving:
            return
        if self.robot is None or not self.robot.connected:
            self.status_var.set("로봇 미연결 — 이동 불가")
            return
        target = self._build_target_posx()
        if target is None:
            self.status_var.set("유효한 목표가 없습니다")
            return

        ok_ws, ws_msg = workspace_ok(np.array(target[:3]))
        if not ok_ws:
            self.status_var.set(f"이동 거부 — {ws_msg}")
            print(self.status_var.get())
            return

        if self.robot is not None and self.robot.connected:
            cur = self.robot.get_posx_mm_deg()
            if cur is not None:
                step = float(np.linalg.norm(np.array(target[:3]) - np.array(cur[:3])))
                if step > MAX_STEP_MM:
                    self.status_var.set(
                        f"이동 거부 — 한 번에 {step:.0f}mm (한도 {MAX_STEP_MM:.0f}mm). "
                        "캘리브/목표를 확인하세요."
                    )
                    print(self.status_var.get())
                    return

        self._moving = True
        self.exec_btn.configure(state=tk.DISABLED)
        self.status_var.set("이동 중...")
        print()
        print("=" * 50)
        print(
            "실행 target: "
            f"[{target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}, "
            f"{target[3]:.2f}, {target[4]:.2f}, {target[5]:.2f}]"
        )

        def worker():
            ok = False
            actual = None
            err = None
            try:
                print("[앱] 그리퍼 TCP·하향 교정 후 movel (Z 이미 보정됨)")
                ok = self.robot.movel_gripper(
                    target[:3],
                    vel=[VEL_XY, VEL_ORI],
                    acc=[ACC_XY, ACC_ORI],
                    straighten_first=True,
                    z_up_mm=0.0,
                )
                print(f"[앱] movel_gripper 반환 ok={ok}")
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
            finally:
                self.root.after(
                    0, lambda o=ok, a=actual, e=err, t=target: self._on_move_done(o, a, e, t)
                )

        threading.Thread(target=worker, daemon=True, name="movel_worker").start()

    def _on_move_done(self, ok, actual, err, target):
        self._moving = False
        if not ok:
            self.status_var.set("이동 실패")
            if self.state["clicked"] and self.state["clicked"].get("valid"):
                self.exec_btn.configure(state=tk.NORMAL)
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
                f"도착 완료 | XYZ 오차 ≈ {err:.1f} mm" if err is not None else "도착 완료"
            )
            print("실제 로봇 pose [mm, deg]:")
            print(
                f"  [{actual[0]:.2f}, {actual[1]:.2f}, {actual[2]:.2f}, "
                f"{actual[3]:.2f}, {actual[4]:.2f}, {actual[5]:.2f}]"
            )
            if err is not None:
                print(f"  목표 대비 XYZ 오차 ≈ {err:.2f} mm")
            print("=" * 50)

        if self.state["clicked"] and self.state["clicked"].get("valid"):
            self.exec_btn.configure(state=tk.NORMAL)

    def update_frame(self):
        if self._closed:
            return
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if color_frame and depth_frame:
                color = np.asanyarray(color_frame.get_data())
                depth_color = np.asanyarray(
                    self.colorizer.colorize(depth_frame).get_data()
                )
                h, w = color.shape[:2]
                if depth_color.shape[:2] != (h, w):
                    depth_color = cv2.resize(
                        depth_color, (w, h), interpolation=cv2.INTER_NEAREST
                    )

                self.state["depth_frame"] = depth_frame
                self.state["intrinsics"] = (
                    color_frame.profile.as_video_stream_profile().get_intrinsics()
                )
                self.state["width"] = w
                self.state["height"] = h

                color_d = color.copy()
                depth_d = depth_color.copy()
                clicked = self.state["clicked"]
                if clicked is not None:
                    u, v = clicked["u"], clicked["v"]
                    col = (0, 255, 0) if clicked.get("valid") else (0, 0, 255)
                    cv2.drawMarker(color_d, (u, v), col, cv2.MARKER_CROSS, 24, 2)
                    cv2.drawMarker(
                        depth_d, (u, v), (255, 255, 255), cv2.MARKER_CROSS, 24, 2
                    )

                cv2.putText(
                    color_d, "Color - click", (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                )
                cv2.putText(
                    depth_d, "Depth - click", (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                )
                combined = np.hstack((color_d, depth_d))
                self.state["display_bgr"] = combined

                # Pillow 없이 PNG(base64) → Tk PhotoImage
                ok, buf = cv2.imencode(".png", combined)
                if ok:
                    b64 = base64.b64encode(buf.tobytes())
                    self._photo = tk.PhotoImage(data=b64)
                    self.video_label.configure(image=self._photo)
        except Exception as exc:
            # 프레임 타임아웃 등은 무시하고 계속
            if not self._closed:
                print(f"[카메라] {exc}")

        if not self._closed:
            self.root.after(30, self.update_frame)

    def on_close(self):
        self._closed = True
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.robot is not None:
            self.robot.close()
        self.root.destroy()


def main() -> None:
    R, t, meta = load_cam2base()
    print("R_cam2base =\n", R)
    print("t_cam2base [m] =", t)
    warn_if_bad_calib(meta)

    robot = DoosanClient(
        robot_id=ROBOT_ID,
        robot_model=ROBOT_MODEL,
        ws=DOOSAN_WS,
    )
    robot_ok = robot.connect_and_set_autonomous(
        setup_gripper=True,
        straighten_down=True,
    )
    if not robot_ok:
        print("로봇 없이 좌표 표시만 진행합니다. (실행 버튼 비활성)")
        robot = None

    root = tk.Tk()
    root.geometry("1280x720")
    ClickAndMoveApp(root, R, t, robot)
    root.mainloop()


if __name__ == "__main__":
    main()
