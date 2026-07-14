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

조작
  s : 보드 검출 + pose 저장
  c : 캘리브레이션 계산
  d : 마지막 샘플 삭제
  q / Esc : 종료
"""

from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path

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


def ask_posx() -> list[float] | None:
    print()
    print("펜던트 현재 posx를 입력하세요.")
    print("예: 400.0 0.0 300.0 0.0 180.0 0.0")
    try:
        line = input("posx> ").strip()
    except EOFError:
        return None
    return parse_posx(line)


def resolve_posx(robot: DoosanClient | None) -> list[float] | None:
    """연결돼 있으면 컨트롤러에서 posx 자동 읽기, 아니면 수동 입력."""
    if robot is not None and robot.connected:
        posx = robot.get_posx_mm_deg()
        if posx is not None:
            print(
                "자동 posx: "
                f"{posx[0]:.3f} {posx[1]:.3f} {posx[2]:.3f} "
                f"{posx[3]:.3f} {posx[4]:.3f} {posx[5]:.3f}"
            )
            try:
                confirm = input("이 값으로 저장할까요? [Y/n/수동입력 m] ").strip().lower()
            except EOFError:
                confirm = "y"
            if confirm in ("", "y", "yes"):
                return posx
            if confirm not in ("m", "manual"):
                print("저장 취소")
                return None
        else:
            print("자동 posx 읽기 실패 → 수동 입력으로 전환")
    return ask_posx()


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

    methods = [
        ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
        ("PARK", cv2.CALIB_HAND_EYE_PARK),
        ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
        ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
    ]

    results = {}
    print()
    print("=" * 60)
    print(f"Eye-to-Hand 캘리브레이션  (샘플 {len(samples)}개)")
    print(f"Euler 변환: {EULER_ORDER}  (두산 기본은 ZYZ)")
    print("=" * 60)

    for name, method in methods:
        R_c2b, t_c2b = cv2.calibrateHandEye(
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


def draw_hud(
    img: np.ndarray,
    n_samples: int,
    board_ok: bool,
    robot_ok: bool,
) -> np.ndarray:
    out = img.copy()
    lines = [
        f"Samples: {n_samples}",
        "Board: OK" if board_ok else "Board: NOT FOUND (ChArUco)",
        "Robot: MANUAL" if robot_ok else "Robot: OFF (manual pose)",
        "s: save | c: calibrate | d: delete last | q: quit",
        (
            f"ChArUco {CHARUCO_SQUARES_X}x{CHARUCO_SQUARES_Y}, "
            f"sq={SQUARE_SIZE_M*1000:.1f}mm, mk={MARKER_SIZE_M*1000:.1f}mm, "
            f"dict={_charuco_state.get('dict_name') or '?'}"
        ),
    ]
    y = 30
    for i, text in enumerate(lines):
        if i == 1:
            color = (0, 255, 0) if board_ok else (0, 0, 255)
        elif i == 2:
            color = (0, 255, 0) if robot_ok else (0, 165, 255)
        else:
            color = (0, 255, 0)
        cv2.putText(out, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        y += 28
    return out


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

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    print("Eye-to-Hand 캘리브레이션 (ChArUco)")
    print(f"샘플 폴더: {sample_dir}")
    print(f"기존 샘플: {len(samples)}개")
    print(
        f"보드 설정: {CHARUCO_SQUARES_X}x{CHARUCO_SQUARES_Y} squares, "
        f"square={SQUARE_SIZE_M*1000:.1f}mm, marker={MARKER_SIZE_M*1000:.1f}mm"
    )
    print("→ 칸/마커 길이는 자로 재서 코드 상단 값을 수정하세요.")
    if robot_ok:
        print("로봇: Manual 모드 — 조그/직접교시로 자세를 잡은 뒤 s")
    else:
        print("로봇 미연결 — 펜던트 Manual + s 후 pose 수동 입력")
    print()

    try:
        profile = pipeline.start(config)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        K, dist = intrinsics_to_camera_matrix(intr)
        print(
            f"intrinsic fx={intr.fx:.2f}, fy={intr.fy:.2f}, "
            f"ppx={intr.ppx:.2f}, ppy={intr.ppy:.2f}"
        )

        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            ok, R_t2c, t_t2c, charuco_corners, rvec, marker_corners, marker_ids = (
                detect_board(gray, K, dist)
            )

            display = color.copy()
            if marker_corners is not None and marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
            if ok and charuco_corners is not None:
                if hasattr(cv2.aruco, "drawDetectedCornersCharuco"):
                    cv2.aruco.drawDetectedCornersCharuco(
                        display, charuco_corners, None, (0, 255, 0)
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
                        cv2.line(
                            display, o, tuple(imgpts[1].ravel()), (0, 0, 255), 2
                        )
                        cv2.line(
                            display, o, tuple(imgpts[2].ravel()), (0, 255, 0), 2
                        )
                        cv2.line(
                            display, o, tuple(imgpts[3].ravel()), (255, 0, 0), 2
                        )

            display = draw_hud(display, len(samples), ok, robot_ok)
            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if key == ord("s"):
                if not ok:
                    print(
                        "ChArUco가 검출되지 않았습니다. "
                        "보드 전체가 보이게, SQUARE_SIZE_M/MARKER_SIZE_M을 실측값으로 맞추세요."
                    )
                    continue
                posx = resolve_posx(robot if robot_ok else None)
                if posx is None:
                    print("입력이 취소되었습니다.")
                    continue
                meta_path = save_sample(
                    sample_dir, next_index, color, posx, R_t2c, t_t2c
                )
                samples.append(json.loads(meta_path.read_text(encoding="utf-8")))
                print(
                    f"저장: sample_{next_index:03d}  "
                    f"posx={posx}  (총 {len(samples)}개)"
                )
                next_index += 1

            if key == ord("d"):
                if not samples:
                    print("삭제할 샘플이 없습니다.")
                    continue
                last = samples.pop()
                idx = last["index"]
                for path in sample_dir.glob(f"sample_{idx:03d}.*"):
                    path.unlink(missing_ok=True)
                print(f"삭제: sample_{idx:03d}  (남은 {len(samples)}개)")

            if key == ord("c"):
                # 디스크에서 다시 로드 (수동 편집 반영)
                samples = load_samples(sample_dir)
                result = run_calibration(samples)
                if result is None:
                    continue
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
                print()
                print(f"저장 완료: {out_json}")
                print(f"저장 완료: {out_npz}")
                print(
                    "사용 예: P_base = R @ P_cam + t  "
                    "(P_cam: test2.py 카메라 좌표, 단위 m)"
                )

            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break

    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        robot.close()


if __name__ == "__main__":
    main()
