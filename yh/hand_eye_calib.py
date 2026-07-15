"""
두산 e0509 Eye-to-Hand 캘리브레이션

카메라: 고정 RealSense
보드: 로봇 플랜지/그리퍼에 고정된 ChArUco 보드
로봇: doosan_ws (ROS2 DSR)로 Manual 모드 전환
      연결되면 s 키에서 posx 자동 읽기, 실패 시 수동 입력

사전 조건 (별도 터미널):
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /home/newuser/ocr/doosan_ws/install/setup.bash
  ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \\
    mode:=real model:=e0509 name:=dsr01 host:=<컨트롤러IP>

이 스크립트도 같은 setup.bash 를 source 한 뒤 실행.

조작 (tkinter GUI — OpenCV highgui 불필요)
  [저장] : 보드 검출 시 현재 프레임 + pose 저장
  [삭제] : 마지막 샘플 삭제
  [캘리브] : 샘플로 cam2base 계산
  [종료] : 종료
"""

from __future__ import annotations

import base64
import json
import math
import threading
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk

import cv2
import numpy as np
import pyrealsense2 as rs

from doosan_client import (
    DEFAULT_ROBOT_ID,
    DEFAULT_ROBOT_MODEL,
    DOOSAN_WS,
    DoosanClient,
)

# ---------------------------------------------------------------------------
# 보드 / 로봇 / 출력 설정
# ---------------------------------------------------------------------------
# ChArUco: 칸(square) 개수 (내부 코너가 아님). 사용자 보드 = 가로 7 x 세로 5
CHARUCO_SQUARES_X = 7
CHARUCO_SQUARES_Y = 5
SQUARE_SIZE_M = 0.025          # 큰 칸 한 변 [m] — 자로 재서 수정
MARKER_SIZE_M = 0.018          # ArUco 마커 한 변 [m] (칸보다 작아야 함)
# None이면 실행 중 자동으로 맞는 사전을 찾음
ARUCO_DICT_NAME = None         # 예: "DICT_4X4_50", "DICT_5X5_100"

# pose 추정에 필요한 최소 ChArUco 코너 수
MIN_CHARUCO_CORNERS = 6

# doosan_ws bringup 의 name / model 과 동일하게
ROBOT_ID = DEFAULT_ROBOT_ID          # "dsr01"
ROBOT_MODEL = DEFAULT_ROBOT_MODEL    # "e0509"
# True면 로봇 연결 실패해도 카메라 캘리브만 계속 (pose 수동 입력)
ALLOW_WITHOUT_ROBOT = True

# 샘플·결과 저장 폴더
OUTPUT_DIR = Path(__file__).resolve().parent / "hand_eye_data"

# 최소 샘플 수 (권장 15 이상)
MIN_SAMPLES = 10

# 두산 posx 기본 자세는 Euler ZYZ (DR_ELR_ZYZ).
# A=Rz, B=Ry', C=Rz''  (manual: get_current_posx ori_type 기본값)
# ZYX/XYZ 로 바꾸면 회전이 틀어져 잔차가 수백 mm 로 커집니다.
EULER_ORDER = "ZYZ"

WINDOW_NAME = "Eye-to-Hand Calibration"

# 런타임에 선택된 사전/보드 (detect_board가 채움)
_charuco_state: dict = {
    "dict_name": None,
    "board": None,
    "dictionary": None,
}


def euler_deg_to_R(rx: float, ry: float, rz: float, order: str = "ZYZ") -> np.ndarray:
    """
    도 단위 자세각 → 3x3 회전행렬.

    두산 기본(DR_ELR_ZYZ): 인자는 (A,B,C)=(rz_like, ry_like, rz2_like) 가 아니라
    posx의 [rx, ry, rz] 슬롯에 ZYZ의 (Z, Y', Z'') 가 들어옵니다.
    즉 rx→Z(A), ry→Y'(B), rz→Z''(C).
    """
    ax, ay, az = map(math.radians, (rx, ry, rz))
    cx, sx = math.cos(ax), math.sin(ax)
    cy, sy = math.cos(ay), math.sin(ay)
    cz, sz = math.cos(az), math.sin(az)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    # ZYZ용: 첫 각(rx=A)용 Rz, 둘째(ry=B)용 Ry, 셋째(rz=C)용 Rz
    Rz_a = np.array(
        [[math.cos(ax), -math.sin(ax), 0],
         [math.sin(ax), math.cos(ax), 0],
         [0, 0, 1]],
        dtype=np.float64,
    )
    Ry_b = np.array(
        [[math.cos(ay), 0, math.sin(ay)],
         [0, 1, 0],
         [-math.sin(ay), 0, math.cos(ay)]],
        dtype=np.float64,
    )
    Rz_c = np.array(
        [[math.cos(az), -math.sin(az), 0],
         [math.sin(az), math.cos(az), 0],
         [0, 0, 1]],
        dtype=np.float64,
    )

    order = order.upper()
    if order == "ZYZ":
        # R = Rz(A) * Ry(B) * Rz(C),  (A,B,C) = (rx, ry, rz)
        return Rz_a @ Ry_b @ Rz_c
    if order == "ZYX":
        return Rz @ Ry @ Rx
    if order == "XYZ":
        return Rx @ Ry @ Rz
    raise ValueError(f"지원하지 않는 EULER_ORDER: {order}")


def posx_to_T(x_mm: float, y_mm: float, z_mm: float,
              rx: float, ry: float, rz: float) -> np.ndarray:
    """두산 posx [mm, deg] → 4x4 (베이스 기준 그리퍼), 단위 m."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = euler_deg_to_R(rx, ry, rz, EULER_ORDER)
    T[:3, 3] = np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0
    return T


def invert_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def T_to_Rt(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return T[:3, :3].copy(), T[:3, 3].reshape(3, 1).copy()


def _aruco_dict_candidates() -> list[tuple[str, int]]:
    names = [
        "DICT_4X4_50",
        "DICT_4X4_100",
        "DICT_5X5_50",
        "DICT_5X5_100",
        "DICT_6X6_50",
        "DICT_6X6_250",
        "DICT_7X7_50",
    ]
    if ARUCO_DICT_NAME:
        names = [ARUCO_DICT_NAME] + [n for n in names if n != ARUCO_DICT_NAME]
    out = []
    for name in names:
        if hasattr(cv2.aruco, name):
            out.append((name, getattr(cv2.aruco, name)))
    return out


def _make_charuco_board(dictionary):
    return cv2.aruco.CharucoBoard(
        (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
        float(SQUARE_SIZE_M),
        float(MARKER_SIZE_M),
        dictionary,
    )


def _detect_markers(gray: np.ndarray, dictionary):
    params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def _detect_charuco(gray: np.ndarray, board, dictionary):
    """ChArUco 코너 검출. 반환: charuco_corners, charuco_ids, marker_corners, marker_ids"""
    if hasattr(cv2.aruco, "CharucoDetector"):
        detector = cv2.aruco.CharucoDetector(board)
        return detector.detectBoard(gray)

    marker_corners, marker_ids, _ = _detect_markers(gray, dictionary)
    if marker_ids is None or len(marker_ids) < 4:
        return None, None, marker_corners, marker_ids
    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board
    )
    return charuco_corners, charuco_ids, marker_corners, marker_ids


def _estimate_charuco_pose(charuco_corners, charuco_ids, board, K, dist):
    if hasattr(cv2.aruco, "estimatePoseCharucoBoard"):
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board, K, dist, None, None
        )
        if ok:
            return True, rvec, tvec

    # OpenCV 4.7+ 일부 빌드: board.matchImagePoints + solvePnP
    if hasattr(board, "matchImagePoints"):
        obj_pts, img_pts = board.matchImagePoints(charuco_corners, charuco_ids)
        if obj_pts is None or len(obj_pts) < MIN_CHARUCO_CORNERS:
            return False, None, None
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist)
        return bool(ok), rvec, tvec

    return False, None, None


def intrinsics_to_camera_matrix(intr) -> tuple[np.ndarray, np.ndarray]:
    K = np.array(
        [[intr.fx, 0, intr.ppx],
         [0, intr.fy, intr.ppy],
         [0, 0, 1]],
        dtype=np.float64,
    )
    dist = np.array(intr.coeffs, dtype=np.float64)
    return K, dist


def detect_board(gray: np.ndarray, K: np.ndarray, dist: np.ndarray):
    """
    ChArUco 검출 + pose.
    성공 시 (ok, R, t, charuco_corners, rvec, marker_corners, marker_ids)
    """
    # 이미 선택된 사전이 있으면 그것만 사용
    candidates = _aruco_dict_candidates()
    if _charuco_state["dict_name"] is not None:
        candidates = [
            (n, d) for n, d in candidates if n == _charuco_state["dict_name"]
        ] or candidates

    best = None
    for name, dict_id in candidates:
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        board = _make_charuco_board(dictionary)
        charuco_corners, charuco_ids, marker_corners, marker_ids = _detect_charuco(
            gray, board, dictionary
        )
        n_markers = 0 if marker_ids is None else len(marker_ids)
        n_charuco = 0 if charuco_ids is None else len(charuco_ids)
        if n_charuco < MIN_CHARUCO_CORNERS:
            # 마커만 많이 잡혀도 후보로 남겨 자동선택에 활용
            if best is None or n_markers > best[0]:
                best = (n_markers, n_charuco, name, None)
            continue

        ok, rvec, tvec = _estimate_charuco_pose(
            charuco_corners, charuco_ids, board, K, dist
        )
        if not ok:
            continue

        if _charuco_state["dict_name"] != name:
            print(f"[보드] ChArUco 사전 선택: {name}  (corners={n_charuco})")
        _charuco_state["dict_name"] = name
        _charuco_state["board"] = board
        _charuco_state["dictionary"] = dictionary
        R, _ = cv2.Rodrigues(rvec)
        return (
            True,
            R,
            tvec.reshape(3, 1),
            charuco_corners,
            rvec,
            marker_corners,
            marker_ids,
        )

    # 실패 시에도 마커 수는 HUD용으로 쓸 수 있게 상태만 갱신하지 않음
    if best is not None and best[0] > 0 and _charuco_state["dict_name"] is None:
        # 너무 자주 출력하지 않도록 가끔만
        pass
    return False, None, None, None, None, None, None


def parse_posx(text: str) -> list[float] | None:
    """
    'x y z rx ry rz' 또는 쉼표 구분 입력.
    단위: mm, deg (펜던트와 동일).
    """
    text = text.replace(",", " ").strip()
    if not text:
        return None
    parts = text.split()
    if len(parts) != 6:
        print("형식: x y z rx ry rz  (6개 숫자, mm / deg)")
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        print("숫자로 입력하세요.")
        return None


def ask_posx(parent: tk.Tk | None = None) -> list[float] | None:
    """펜던트 posx 수동 입력 (tkinter 다이얼로그)."""
    text = simpledialog.askstring(
        "posx 입력",
        "펜던트 posx (mm, deg)\n예: 400.0 0.0 300.0 0.0 180.0 0.0",
        parent=parent,
    )
    if text is None:
        return None
    return parse_posx(text)


def resolve_posx(
    robot: DoosanClient | None, parent: tk.Tk | None = None
) -> list[float] | None:
    """연결돼 있으면 컨트롤러에서 posx 자동 읽기, 아니면 수동 입력."""
    if robot is not None and robot.connected:
        posx = robot.get_posx_mm_deg()
        if posx is not None:
            msg = (
                "자동으로 읽은 posx:\n"
                f"{posx[0]:.3f} {posx[1]:.3f} {posx[2]:.3f} "
                f"{posx[3]:.3f} {posx[4]:.3f} {posx[5]:.3f}\n\n"
                "이 값으로 저장할까요?\n"
                "(아니오 = 수동 입력)"
            )
            if messagebox.askyesno("posx 확인", msg, parent=parent):
                return posx
            return ask_posx(parent)
        print("자동 posx 읽기 실패 → 수동 입력으로 전환")
    return ask_posx(parent)


def save_sample(sample_dir: Path, index: int, color_bgr: np.ndarray,
                posx: list[float], R_t2c: np.ndarray, t_t2c: np.ndarray) -> Path:
    sample_dir.mkdir(parents=True, exist_ok=True)
    stem = f"sample_{index:03d}"
    img_path = sample_dir / f"{stem}.png"
    meta_path = sample_dir / f"{stem}.json"
    cv2.imwrite(str(img_path), color_bgr)
    meta = {
        "index": index,
        "posx_mm_deg": posx,
        "euler_order": EULER_ORDER,
        "board_type": "charuco",
        "charuco_squares_x": CHARUCO_SQUARES_X,
        "charuco_squares_y": CHARUCO_SQUARES_Y,
        "square_size_m": SQUARE_SIZE_M,
        "marker_size_m": MARKER_SIZE_M,
        "aruco_dict": _charuco_state.get("dict_name"),
        "R_target2cam": R_t2c.tolist(),
        "t_target2cam_m": t_t2c.reshape(-1).tolist(),
        "image": img_path.name,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    return meta_path


def load_samples(sample_dir: Path) -> list[dict]:
    metas = sorted(sample_dir.glob("sample_*.json"))
    samples = []
    for path in metas:
        samples.append(json.loads(path.read_text(encoding="utf-8")))
    return samples


def _rt_to_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _skew3(v: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=np.float64).reshape(3)
    return np.array(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]],
        dtype=np.float64,
    )


def _rot_to_modified_rodrigues(R: np.ndarray) -> np.ndarray:
    """Tsai용: P = 2 sin(θ/2) * axis."""
    rvec, _ = cv2.Rodrigues(np.asarray(R, dtype=np.float64).reshape(3, 3))
    r = rvec.reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.zeros(3, dtype=np.float64)
    return (2.0 * math.sin(theta / 2.0) / theta) * r


def _modified_rodrigues_to_R(p: np.ndarray) -> np.ndarray:
    p = np.asarray(p, dtype=np.float64).reshape(3)
    n2 = float(np.dot(p, p))
    if n2 < 1e-16:
        return np.eye(3, dtype=np.float64)
    half = min(1.0, 0.5 * math.sqrt(n2))
    theta = 2.0 * math.asin(half)
    axis = p / math.sqrt(n2)
    rvec = (theta * axis).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return np.asarray(R, dtype=np.float64).reshape(3, 3)


def _calibrate_hand_eye_tsai_numpy(
    R_g2b_list: list[np.ndarray],
    t_g2b_list: list[np.ndarray],
    R_t2c_list: list[np.ndarray],
    t_t2c_list: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """
    OpenCV calibrateHandEye(TSAI) 입력 규약의 NumPy 폴백.
    eye-to-hand: base→gripper, target→cam → cam→base.
    """
    n = len(R_g2b_list)
    if n < 3:
        raise ValueError("Tsai hand-eye 에는 최소 3개 샘플이 필요합니다")

    Hg = [_rt_to_T(R_g2b_list[i], t_g2b_list[i]) for i in range(n)]
    Hc = [_rt_to_T(R_t2c_list[i], t_t2c_list[i]) for i in range(n)]

    Pg_list: list[np.ndarray] = []
    Pc_list: list[np.ndarray] = []
    Ra_list: list[np.ndarray] = []
    ta_list: list[np.ndarray] = []
    tb_list: list[np.ndarray] = []

    for i in range(n):
        for j in range(i + 1, n):
            A = np.linalg.inv(Hg[i]) @ Hg[j]
            B = Hc[i] @ np.linalg.inv(Hc[j])
            Ra, Rb = A[:3, :3], B[:3, :3]
            if (
                float(np.linalg.norm(cv2.Rodrigues(Ra)[0])) < 0.05
                or float(np.linalg.norm(cv2.Rodrigues(Rb)[0])) < 0.05
            ):
                continue
            Pg_list.append(_rot_to_modified_rodrigues(Ra))
            Pc_list.append(_rot_to_modified_rodrigues(Rb))
            Ra_list.append(Ra)
            ta_list.append(A[:3, 3].copy())
            tb_list.append(B[:3, 3].copy())

    if len(Pg_list) < 2:
        raise RuntimeError(
            "유효한 상대자세 쌍이 부족합니다 (자세를 더 다양하게 촬영하세요)"
        )

    rows = [_skew3(Pg + Pc) for Pg, Pc in zip(Pg_list, Pc_list)]
    rhs = [(Pc - Pg).reshape(3, 1) for Pg, Pc in zip(Pg_list, Pc_list)]
    Px, *_ = np.linalg.lstsq(np.vstack(rows), np.vstack(rhs), rcond=None)
    Px = Px.reshape(3)
    denom = math.sqrt(1.0 + float(np.dot(Px, Px)))
    Px = (2.0 / denom) * Px
    Rx = _modified_rodrigues_to_R(Px)

    I = np.eye(3, dtype=np.float64)
    rows_t = [Ra - I for Ra in Ra_list]
    rhs_t = [
        (Rx @ tb.reshape(3) - ta.reshape(3)).reshape(3, 1)
        for ta, tb in zip(ta_list, tb_list)
    ]
    tx, *_ = np.linalg.lstsq(np.vstack(rows_t), np.vstack(rhs_t), rcond=None)
    return Rx, tx.reshape(3, 1)


def calibrate_hand_eye(
    R_g2b_list: list[np.ndarray],
    t_g2b_list: list[np.ndarray],
    R_t2c_list: list[np.ndarray],
    t_t2c_list: list[np.ndarray],
    *,
    method: int | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """cv2.calibrateHandEye 래퍼. 없으면 Tsai NumPy 폴백."""
    if hasattr(cv2, "calibrateHandEye"):
        kw = {}
        if method is not None:
            kw["method"] = method
        return cv2.calibrateHandEye(
            R_g2b_list,
            t_g2b_list,
            R_t2c_list,
            t_t2c_list,
            **kw,
        )

    if not getattr(calibrate_hand_eye, "_warned", False):
        print(
            "[캘리브] cv2.calibrateHandEye 없음 "
            f"(OpenCV {cv2.__version__}) — Tsai NumPy 폴백 사용"
        )
        calibrate_hand_eye._warned = True  # type: ignore[attr-defined]
    return _calibrate_hand_eye_tsai_numpy(
        R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list
    )


def run_calibration(samples: list[dict]) -> dict | None:
    if len(samples) < MIN_SAMPLES:
        print(f"샘플이 {len(samples)}개뿐입니다. 최소 {MIN_SAMPLES}개 필요합니다.")
        return None

    R_base2grip_list = []
    t_base2grip_list = []
    R_target2cam_list = []
    t_target2cam_list = []

    for s in samples:
        x, y, z, rx, ry, rz = s["posx_mm_deg"]
        T_g2b = posx_to_T(x, y, z, rx, ry, rz)  # gripper → base
        T_b2g = invert_T(T_g2b)                   # base → gripper (eye-to-hand용)
        Rb, tb = T_to_Rt(T_b2g)
        R_base2grip_list.append(Rb)
        t_base2grip_list.append(tb)
        R_target2cam_list.append(np.array(s["R_target2cam"], dtype=np.float64))
        t_target2cam_list.append(
            np.array(s["t_target2cam_m"], dtype=np.float64).reshape(3, 1)
        )

    use_cv = hasattr(cv2, "calibrateHandEye")
    if use_cv:
        methods = [
            ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
            ("PARK", cv2.CALIB_HAND_EYE_PARK),
            ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
            ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]
    else:
        methods = [("TSAI", None)]

    results = {}
    print()
    print("=" * 60)
    print(f"Eye-to-Hand 캘리브레이션  (샘플 {len(samples)}개)")
    print(f"Euler 변환: {EULER_ORDER}  (두산 기본은 ZYZ)")
    if not use_cv:
        print(f"OpenCV {cv2.__version__}: calibrateHandEye 미바인딩 → Tsai 폴백")
    print("=" * 60)

    for name, method in methods:
        R_c2b, t_c2b = calibrate_hand_eye(
            R_base2grip_list,
            t_base2grip_list,
            R_target2cam_list,
            t_target2cam_list,
            method=method,
        )
        t_flat = t_c2b.reshape(-1)
        results[name] = {"R_cam2base": R_c2b, "t_cam2base_m": t_flat}
        print(f"\n[{name}]")
        print("R_cam2base =\n", np.array2string(R_c2b, precision=6, suppress_small=True))
        print(
            "t_cam2base [m] = "
            f"[{t_flat[0]:.6f}, {t_flat[1]:.6f}, {t_flat[2]:.6f}]"
        )
        print(
            "t_cam2base [mm] = "
            f"[{t_flat[0]*1000:.2f}, {t_flat[1]*1000:.2f}, {t_flat[2]*1000:.2f}]"
        )

    # 기본 결과는 TSAI
    primary = results["TSAI"]
    residual = estimate_residual(
        samples, primary["R_cam2base"], primary["t_cam2base_m"]
    )
    print()
    print(f"TSAI 잔차(대략) mean={residual['mean_mm']:.2f} mm, "
          f"max={residual['max_mm']:.2f} mm")
    return {
        "mode": "eye_to_hand",
        "euler_order": EULER_ORDER,
        "num_samples": len(samples),
        "methods": {
            name: {
                "R_cam2base": v["R_cam2base"].tolist(),
                "t_cam2base_m": v["t_cam2base_m"].tolist(),
            }
            for name, v in results.items()
        },
        "primary_method": "TSAI",
        "R_cam2base": primary["R_cam2base"].tolist(),
        "t_cam2base_m": primary["t_cam2base_m"].tolist(),
        "residual_mm": residual,
        "hand_eye_backend": "opencv" if use_cv else "tsai_numpy",
        "created_at": datetime.now().isoformat(timespec="seconds"),
    }


def estimate_residual(samples: list[dict], R_c2b: np.ndarray,
                      t_c2b: np.ndarray) -> dict:
    """
    보드 원점을 카메라→베이스로 변환한 뒤,
    (그리퍼→베이스)*(보드→그리퍼 추정)과 비교하기보다
    간단히: 보드 원점의 base 좌표가 자세마다 일정한지
    (보드가 그리퍼에 고정이면, gripper 기준 보드 원점은 상수)
    로 일관성 잔차를 봅니다.
    """
    # ^{grip}T_board 추정: ^{grip}T_base * ^{base}T_cam * ^{cam}T_board
    # ^{base}T_cam = [R_c2b|t], ^{cam}T_board = [R_t2c|t_t2c]
    board_in_grip = []
    for s in samples:
        x, y, z, rx, ry, rz = s["posx_mm_deg"]
        T_g2b = posx_to_T(x, y, z, rx, ry, rz)
        T_b2g = invert_T(T_g2b)

        R_t2c = np.array(s["R_target2cam"], dtype=np.float64)
        t_t2c = np.array(s["t_target2cam_m"], dtype=np.float64).reshape(3)

        T_c2b = np.eye(4)
        T_c2b[:3, :3] = R_c2b
        T_c2b[:3, 3] = t_c2b.reshape(3)

        T_t2c = np.eye(4)
        T_t2c[:3, :3] = R_t2c
        T_t2c[:3, 3] = t_t2c

        # board origin in gripper frame
        T_t2g = T_b2g @ T_c2b @ T_t2c
        board_in_grip.append(T_t2g[:3, 3])

    pts = np.stack(board_in_grip, axis=0)
    mean = pts.mean(axis=0)
    errs = np.linalg.norm(pts - mean, axis=1) * 1000.0  # mm
    return {
        "mean_mm": float(errs.mean()),
        "max_mm": float(errs.max()),
        "board_origin_in_gripper_mean_m": mean.tolist(),
    }


def _corners_nx1x2(corners) -> np.ndarray | None:
    """
    OpenCV 5 CharucoDetector 는 (N,2), 이전은 (N,1,2).
    drawDetectedCornersCharuco 는 channels==2 인 (N,1,2) 필요.
    """
    if corners is None:
        return None
    pts = np.asarray(corners, dtype=np.float32)
    if pts.size == 0:
        return None
    if pts.ndim == 2 and pts.shape[1] == 2:
        pts = pts.reshape(-1, 1, 2)
    elif pts.ndim == 3 and pts.shape[-1] == 2:
        pts = pts.reshape(-1, 1, 2)
    else:
        return None
    return np.ascontiguousarray(pts)


def fit_display(bgr: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    """라벨 크기에 맞게 축소 (tk PhotoImage 부담·렉 완화)."""
    h, w = bgr.shape[:2]
    max_w = max(int(max_w), 1)
    max_h = max(int(max_h), 1)
    scale = min(max_w / w, max_h / h, 1.0)
    if scale >= 0.999:
        return bgr
    dw = max(1, int(round(w * scale)))
    dh = max(1, int(round(h * scale)))
    return cv2.resize(bgr, (dw, dh), interpolation=cv2.INTER_AREA)


def bgr_to_photo(bgr: np.ndarray) -> tk.PhotoImage:
    ok, buf = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError("이미지 인코딩 실패")
    return tk.PhotoImage(data=base64.b64encode(buf.tobytes()))


def annotate_frame(
    color: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    ok: bool,
    charuco_corners,
    rvec,
    t_t2c,
    marker_corners,
    marker_ids,
) -> np.ndarray:
    display = color.copy()
    try:
        if marker_corners is not None and marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
        if ok and charuco_corners is not None:
            cc = _corners_nx1x2(charuco_corners)
            if cc is not None and hasattr(cv2.aruco, "drawDetectedCornersCharuco"):
                cv2.aruco.drawDetectedCornersCharuco(
                    display, cc, None, (0, 255, 0)
                )
            if rvec is not None and t_t2c is not None:
                if hasattr(cv2, "drawFrameAxes"):
                    cv2.drawFrameAxes(
                        display, K, dist, rvec, t_t2c, 3 * SQUARE_SIZE_M
                    )
                else:
                    axis = np.float32(
                        [
                            [0, 0, 0],
                            [3 * SQUARE_SIZE_M, 0, 0],
                            [0, 3 * SQUARE_SIZE_M, 0],
                            [0, 0, -3 * SQUARE_SIZE_M],
                        ]
                    )
                    imgpts, _ = cv2.projectPoints(axis, rvec, t_t2c, K, dist)
                    imgpts = imgpts.astype(int)
                    o = tuple(imgpts[0].ravel())
                    cv2.line(display, o, tuple(imgpts[1].ravel()), (0, 0, 255), 2)
                    cv2.line(display, o, tuple(imgpts[2].ravel()), (0, 255, 0), 2)
                    cv2.line(display, o, tuple(imgpts[3].ravel()), (255, 0, 0), 2)
    except Exception as exc:
        # OpenCV 버전별 드로잉 실패해도 원본 프레임은 표시
        print(f"[카메라] annotate 경고: {exc}")
    return display


class HandEyeCalibApp:
    def __init__(
        self,
        root: tk.Tk,
        robot: DoosanClient | None,
        robot_ok: bool,
        sample_dir: Path,
        samples: list[dict],
        next_index: int,
    ):
        self.root = root
        self.robot = robot
        self.robot_ok = robot_ok
        self.sample_dir = sample_dir
        self.samples = samples
        self.next_index = next_index
        self._closed = False
        self._photo = None
        self._busy = False

        self._latest = {
            "color": None,
            "ok": False,
            "R_t2c": None,
            "t_t2c": None,
        }

        self.status_var = tk.StringVar(value="보드가 보이면 [저장]을 누르세요")
        self.info_var = tk.StringVar(
            value=(
                f"Samples={len(samples)}  Board=?  "
                f"Robot={'MANUAL' if robot_ok else 'OFF'}  "
                f"Euler={EULER_ORDER}"
            )
        )

        root.title(WINDOW_NAME)
        root.geometry("1100x750")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        top = ttk.Frame(root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, textvariable=self.status_var, font=("Sans", 11)).pack(anchor=tk.W)
        ttk.Label(top, textvariable=self.info_var).pack(anchor=tk.W)

        btns = ttk.Frame(root, padding=8)
        btns.pack(fill=tk.X)
        self.btn_save = ttk.Button(btns, text="저장 (샘플)", command=self.on_save)
        self.btn_save.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btns, text="삭제 (마지막)", command=self.on_delete).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(btns, text="캘리브 계산", command=self.on_calibrate).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(btns, text="종료", command=self.on_close).pack(side=tk.LEFT)

        self.video = tk.Label(root)
        self.video.pack(fill=tk.BOTH, expand=True)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self.K, self.dist = intrinsics_to_camera_matrix(intr)
        print(
            f"intrinsic fx={intr.fx:.2f}, fy={intr.fy:.2f}, "
            f"ppx={intr.ppx:.2f}, ppy={intr.ppy:.2f}"
        )

        self.root.after(10, self.update_frame)

    def _refresh_info(self, board_ok: bool) -> None:
        self.info_var.set(
            f"Samples={len(self.samples)}  "
            f"Board={'OK' if board_ok else 'NOT FOUND'}  "
            f"Robot={'MANUAL' if self.robot_ok else 'OFF'}  "
            f"dict={_charuco_state.get('dict_name') or '?'}  "
            f"Euler={EULER_ORDER}"
        )
        self.btn_save.configure(state=tk.NORMAL if board_ok and not self._busy else tk.DISABLED)

    def _show_bgr(self, bgr: np.ndarray) -> None:
        self.root.update_idletasks()
        lbl_w = max(self.video.winfo_width(), 1)
        lbl_h = max(self.video.winfo_height(), 1)
        if lbl_w < 40 or lbl_h < 40:
            lbl_w = max(self.root.winfo_width() - 24, 640)
            lbl_h = max(self.root.winfo_height() - 120, 360)
        disp = fit_display(bgr, lbl_w, lbl_h)
        self._photo = bgr_to_photo(disp)
        self.video.configure(image=self._photo)

    def update_frame(self) -> None:
        if self._closed:
            return
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if color_frame:
                color = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
                ok, R_t2c, t_t2c, charuco_corners, rvec, marker_corners, marker_ids = (
                    detect_board(gray, self.K, self.dist)
                )
                self._latest = {
                    "color": color,
                    "ok": ok,
                    "R_t2c": R_t2c,
                    "t_t2c": t_t2c,
                }
                display = annotate_frame(
                    color,
                    self.K,
                    self.dist,
                    ok,
                    charuco_corners,
                    rvec,
                    t_t2c,
                    marker_corners,
                    marker_ids,
                )
                self._refresh_info(ok)
                self._show_bgr(display)
        except Exception as exc:
            if not self._closed:
                print(f"[카메라] {exc}")
                self.status_var.set(f"카메라 오류: {exc}")

        if not self._closed:
            self.root.after(30, self.update_frame)

    def on_save(self) -> None:
        if self._busy:
            return
        latest = self._latest
        if not latest["ok"] or latest["color"] is None:
            messagebox.showwarning(
                "저장 불가",
                "ChArUco가 검출되지 않았습니다.\n보드 전체를 보이게 하세요.",
                parent=self.root,
            )
            return

        posx = resolve_posx(self.robot if self.robot_ok else None, parent=self.root)
        if posx is None:
            self.status_var.set("저장 취소")
            return

        meta_path = save_sample(
            self.sample_dir,
            self.next_index,
            latest["color"],
            posx,
            latest["R_t2c"],
            latest["t_t2c"],
        )
        self.samples.append(json.loads(meta_path.read_text(encoding="utf-8")))
        msg = (
            f"저장: sample_{self.next_index:03d}  "
            f"posx={posx}  (총 {len(self.samples)}개)"
        )
        print(msg)
        self.status_var.set(msg)
        self.next_index += 1
        self._refresh_info(True)

    def on_delete(self) -> None:
        if not self.samples:
            messagebox.showinfo("삭제", "삭제할 샘플이 없습니다.", parent=self.root)
            return
        last = self.samples.pop()
        idx = last["index"]
        for path in self.sample_dir.glob(f"sample_{idx:03d}.*"):
            path.unlink(missing_ok=True)
        msg = f"삭제: sample_{idx:03d}  (남은 {len(self.samples)}개)"
        print(msg)
        self.status_var.set(msg)
        self._refresh_info(bool(self._latest.get("ok")))

    def on_calibrate(self) -> None:
        if self._busy:
            return
        self._busy = True
        self.status_var.set("캘리브 계산 중...")
        self.root.update_idletasks()

        def worker():
            err = None
            result = None
            try:
                samples = load_samples(self.sample_dir)
                self.samples = samples
                result = run_calibration(samples)
                if result is not None:
                    out_json = OUTPUT_DIR / "cam2base.json"
                    out_npz = OUTPUT_DIR / "cam2base.npz"
                    out_json.write_text(
                        json.dumps(result, indent=2), encoding="utf-8"
                    )
                    np.savez(
                        out_npz,
                        R_cam2base=np.array(result["R_cam2base"]),
                        t_cam2base_m=np.array(result["t_cam2base_m"]),
                    )
                    print(f"저장 완료: {out_json}")
                    print(f"저장 완료: {out_npz}")
            except Exception as exc:
                err = exc
            self.root.after(0, lambda: self._on_calib_done(result, err))

        threading.Thread(target=worker, daemon=True).start()

    def _on_calib_done(self, result, err) -> None:
        self._busy = False
        if err is not None:
            messagebox.showerror("캘리브 오류", str(err), parent=self.root)
            self.status_var.set("캘리브 실패")
            return
        if result is None:
            self.status_var.set(
                f"샘플 부족 (최소 {MIN_SAMPLES}개, 현재 {len(self.samples)}개)"
            )
            return
        residual = result.get("residual_mm", {})
        mean_mm = residual.get("mean_mm", float("nan"))
        max_mm = residual.get("max_mm", float("nan"))
        msg = f"캘리브 완료 — 잔차 mean={mean_mm:.1f}mm max={max_mm:.1f}mm"
        self.status_var.set(msg)
        messagebox.showinfo(
            "캘리브 완료",
            f"{msg}\n\n저장: hand_eye_data/cam2base.json / .npz",
            parent=self.root,
        )

    def on_close(self) -> None:
        self._closed = True
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.robot is not None:
            self.robot.close()
        self.root.destroy()


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    sample_dir = OUTPUT_DIR / "samples"
    sample_dir.mkdir(parents=True, exist_ok=True)

    samples = load_samples(sample_dir)
    next_index = (max((s["index"] for s in samples), default=-1) + 1)

    robot = DoosanClient(
        robot_id=ROBOT_ID,
        robot_model=ROBOT_MODEL,
        ws=DOOSAN_WS,
    )
    robot_ok = robot.connect_and_set_manual()
    if not robot_ok and not ALLOW_WITHOUT_ROBOT:
        print("로봇 연결/Manual 전환 실패로 종료합니다.")
        return
    if not robot_ok:
        robot = None
        print("로봇 미연결 — pose 수동 입력 모드")

    print("Eye-to-Hand 캘리브레이션 (ChArUco, tkinter GUI)")
    print(f"샘플 폴더: {sample_dir}")
    print(f"기존 샘플: {len(samples)}개")
    print(
        f"보드 설정: {CHARUCO_SQUARES_X}x{CHARUCO_SQUARES_Y} squares, "
        f"square={SQUARE_SIZE_M*1000:.1f}mm, marker={MARKER_SIZE_M*1000:.1f}mm"
    )
    print(f"Euler={EULER_ORDER}")

    root = tk.Tk()
    HandEyeCalibApp(root, robot, robot_ok, sample_dir, samples, next_index)
    root.mainloop()


if __name__ == "__main__":
    main()
