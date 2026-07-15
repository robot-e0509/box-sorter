"""
두산 로봇 클라이언트 — /home/newuser/ocr/doosan_ws (ROS2 + dsr_msgs2)

GUI 앱에서도 안전하게 동작하도록 MultiThreadedExecutor 를 백그라운드에서 spin 합니다.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np


DOOSAN_WS = Path("/home/newuser/ocr/doosan_ws")
DEFAULT_ROBOT_ID = "dsr01"
DEFAULT_ROBOT_MODEL = "e0509"
DEFAULT_ROBOT_IP = "192.168.137.100"
DEFAULT_ROBOT_PORT = 12345

ROBOT_MODE_MANUAL = 0
ROBOT_MODE_AUTONOMOUS = 1
DR_BASE = 0
DR_MV_MOD_ABS = 0

# ---------------------------------------------------------------------------
# 그리퍼 TCP (플랜지 → RH-P12-RN tip)
# dsr_study README: E0509 + ROBOTIS RH-P12-RN(A) 플랜지 직결
# TCP pos = [x,y,z,rx,ry,rz] (mm, deg), 플랜지 좌표계 기준
# Z는 실측 손가락 tip까지 거리로 조정하세요.
# ---------------------------------------------------------------------------
GRIPPER_TCP_NAME = "rh_p12_rn"
GRIPPER_TCP_POS = [0.0, 0.0, 136.0, 0.0, 0.0, 0.0] #154 -> 152
# 감지된 표면 Z보다 tip을 이만큼 위로 (그리퍼 높이 감안, mm)
# 기본값 = TCP Z(=그리퍼 길이). 더/덜 올리고 싶으면 여기만 조정.
GRIPPER_HEIGHT_MM = float(GRIPPER_TCP_POS[2])
TARGET_Z_UP_MM = GRIPPER_HEIGHT_MM
# 두산 기본 ZYZ: [0, 180, 0] ≈ 툴(그리퍼)이 바닥을 향함 (dsr_tests movel 예제와 동일)
GRIPPER_DOWN_ORI_DEG = [0.0, 180.0, 0.0]
ORI_TOL_DEG = 5.0
# 기동 전 기본자세 (조인트 deg, J1..J6)
HOME_POSJ_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
HOME_POSJ_TOL_DEG = 2.0
# Ry=180°(하향) ZYZ 특이점: [0,180,yaw] ≡ [-yaw,180,0]
# → 항상 정규형 [yaw, 180, 0] 사용
GRIPPER_YAW_ON_RX = True
GRIPPER_YAW_SIGN = 1.0
# 손가락 축이 90° 어긋나면 ±90 조정
GRIPPER_YAW_OFFSET_DEG = 0.0

# RH-P12-RN stroke (dsr_gripper): 0=완전닫힘, 750=완전열림
GRIPPER_POS_OPEN = 750
GRIPPER_POS_CLOSE = 0
GRIPPER_POS_HALF = 375  # 커스텀닫기 기본 = 반만 닫힘
GRIPPER_CURRENT_OPEN = 200
GRIPPER_CURRENT_CLOSE = 300
GRIPPER_CURRENT_CUSTOM = 250
GRIPPER_CMD_TIMEOUT_S = 20.0
# 손가락 기구 정착 대기 (열기/닫기 서비스 응답 후, 이동 전)
GRIPPER_SETTLE_S = 4.0


def normalize_yaw_deg(yaw: float) -> float:
    y = float(yaw)
    while y > 180.0:
        y -= 360.0
    while y <= -180.0:
        y += 360.0
    return y


def _Rz_deg(a: float) -> np.ndarray:
    r = math.radians(float(a))
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _Ry_deg(a: float) -> np.ndarray:
    r = math.radians(float(a))
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def euler_zyz_deg_to_R(rx: float, ry: float, rz: float) -> np.ndarray:
    """두산 기본 Euler ZYZ: R = Rz(rx) @ Ry(ry) @ Rz(rz)."""
    return _Rz_deg(rx) @ _Ry_deg(ry) @ _Rz_deg(rz)


def ori_rot_diff_deg(ori_a: list[float], ori_b: list[float]) -> float:
    """두 Euler(ZYZ) 자세의 실제 회전각 차이 [deg] (특이점 표현 무시)."""
    Ra = euler_zyz_deg_to_R(float(ori_a[0]), float(ori_a[1]), float(ori_a[2]))
    Rb = euler_zyz_deg_to_R(float(ori_b[0]), float(ori_b[1]), float(ori_b[2]))
    R = Ra.T @ Rb
    tr = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    return float(math.degrees(math.acos(tr)))


def canonicalize_down_ori(ori_deg: list[float]) -> list[float]:
    """
    하향(Ry≈±180) 자세를 정규형 [yaw, 180, 0] 으로 변환.
    특이점 동등식: [a, 180, c] ≡ [a-c, 180, 0]
    """
    rx, ry, rz = float(ori_deg[0]), float(ori_deg[1]), float(ori_deg[2])
    if abs(abs(ry) - 180.0) <= 15.0:
        yaw = normalize_yaw_deg(rx - rz)
        return [yaw, 180.0, 0.0]
    return [rx, ry, rz]


def effective_tool_x_yaw_deg(ori_deg: list[float]) -> float:
    """하향 ori의 툴 X 축이 base XY에서 가리키는 각도."""
    R = euler_zyz_deg_to_R(float(ori_deg[0]), float(ori_deg[1]), float(ori_deg[2]))
    x = R[:, 0]
    return float(math.degrees(math.atan2(float(x[1]), float(x[0]))))


def gripper_ori_with_yaw(
    yaw_deg: float, *, offset_deg: float | None = None
) -> list[float]:
    """
    하향 자세 + base XY yaw(원하는 툴X 방향각) → posx ori [yaw,180,0].

    Rzyz(A,180,0) 의 tool X ≈ [-cos(A), -sin(A), 0]
    → tool X 각도 = A+180° → A = desired - 180°
    """
    off = float(GRIPPER_YAW_OFFSET_DEG if offset_deg is None else offset_deg)
    desired = normalize_yaw_deg(float(GRIPPER_YAW_SIGN) * float(yaw_deg) + off)
    A = normalize_yaw_deg(desired - 180.0)
    return [A, float(GRIPPER_DOWN_ORI_DEG[1]), 0.0]


def _ensure_doosan_ws_pythonpath(ws: Path = DOOSAN_WS) -> None:
    """doosan_ws install 의 Python/라이브러리 경로를 프로세스에 주입."""
    install = ws / "install"
    if not install.is_dir():
        return

    # --- Python path ---
    for site in install.glob("*/lib/python*/site-packages"):
        p = str(site)
        if p not in sys.path:
            sys.path.insert(0, p)
    for imp in install.glob("*/lib/*/imp"):
        p = str(imp)
        if p not in sys.path:
            sys.path.insert(0, p)

    # --- AMENT_PREFIX_PATH ---
    prefix = os.environ.get("AMENT_PREFIX_PATH", "")
    parts = [str(p) for p in install.iterdir() if p.is_dir()]
    if parts:
        os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join(
            parts + ([prefix] if prefix else [])
        )

    # --- LD_LIBRARY_PATH (dsr_msgs2 .so 등) ---
    lib_dirs: list[str] = []
    for lib in install.glob("*/lib"):
        if lib.is_dir():
            lib_dirs.append(str(lib))
    # 확장 모듈이 site-packages 아래에 있는 경우
    for site in install.glob("*/lib/python*/site-packages/*"):
        if site.is_dir() and any(site.glob("*.so")):
            lib_dirs.append(str(site))

    if lib_dirs:
        old = os.environ.get("LD_LIBRARY_PATH", "")
        merged = os.pathsep.join(lib_dirs + ([old] if old else []))
        os.environ["LD_LIBRARY_PATH"] = merged

    # 런타임에 LD_LIBRARY_PATH 만 바꿔서는 dlopen 이 실패하는 경우가 있어 preload
    try:
        import ctypes

        dsr_lib = install / "dsr_msgs2" / "lib"
        if dsr_lib.is_dir():
            # generator_py 가 typesupport 등을 필요로 하므로 관련 .so 전부 로드
            for name in (
                "libdsr_msgs2__rosidl_generator_c.so",
                "libdsr_msgs2__rosidl_typesupport_c.so",
                "libdsr_msgs2__rosidl_typesupport_cpp.so",
                "libdsr_msgs2__rosidl_typesupport_introspection_c.so",
                "libdsr_msgs2__rosidl_typesupport_fastrtps_c.so",
                "libdsr_msgs2__rosidl_typesupport_fastrtps_cpp.so",
                "libdsr_msgs2__rosidl_generator_py.so",
            ):
                so = dsr_lib / name
                if so.is_file():
                    try:
                        ctypes.CDLL(str(so), mode=ctypes.RTLD_GLOBAL)
                    except OSError as exc:
                        print(f"[로봇] preload 경고 {so.name}: {exc}")
    except Exception as exc:
        print(f"[로봇] shared lib preload 경고: {exc}")



@dataclass
class DoosanClient:
    robot_id: str = DEFAULT_ROBOT_ID
    robot_model: str = DEFAULT_ROBOT_MODEL
    ws: Path = DOOSAN_WS
    service_timeout_s: float = 30.0
    move_timeout_s: float = 120.0

    node: Any = field(default=None, repr=False)
    connected: bool = False
    service_prefix: str = ""
    _cli_set_mode: Any = field(default=None, repr=False)
    _cli_get_posx: Any = field(default=None, repr=False)
    _cli_get_posj: Any = field(default=None, repr=False)
    _cli_movel: Any = field(default=None, repr=False)
    _cli_movej: Any = field(default=None, repr=False)
    _cli_set_control: Any = field(default=None, repr=False)
    _cli_add_tcp: Any = field(default=None, repr=False)
    _cli_set_tcp: Any = field(default=None, repr=False)
    _cli_get_tcp: Any = field(default=None, repr=False)
    _cli_gripper: Any = field(default=None, repr=False)
    _rclpy: Any = field(default=None, repr=False)
    _executor: Any = field(default=None, repr=False)
    _spin_thread: Any = field(default=None, repr=False)
    _SetRobotMode: Any = field(default=None, repr=False)
    _GetCurrentPosx: Any = field(default=None, repr=False)
    _GetCurrentPosj: Any = field(default=None, repr=False)
    _MoveLine: Any = field(default=None, repr=False)
    _MoveJoint: Any = field(default=None, repr=False)
    _SetRobotControl: Any = field(default=None, repr=False)
    _ConfigCreateTcp: Any = field(default=None, repr=False)
    _SetCurrentTcp: Any = field(default=None, repr=False)
    _GetCurrentTcp: Any = field(default=None, repr=False)
    _GripperCmd: Any = field(default=None, repr=False)
    _lock: Any = field(default_factory=threading.Lock, repr=False)
    gripper_ready: bool = False
    # 커스텀 닫기 stroke (0=완전닫힘 ~ 750=완전열림). 기본=반만 닫힘
    custom_close_pos: int = GRIPPER_POS_HALF
    custom_close_current: int = GRIPPER_CURRENT_CUSTOM
    robot_mode: int = ROBOT_MODE_AUTONOMOUS

    def _wait_future(self, future, timeout_s: float):
        """executor가 spin 중이므로 future.done()만 기다림."""
        deadline = time.time() + timeout_s
        while not future.done():
            if time.time() > deadline:
                return None
            time.sleep(0.02)
        try:
            return future.result()
        except Exception as exc:
            print(f"[로봇] service future 예외: {exc}")
            return None

    def connect(self, timeout_s: float = 20.0, node_name: str = "dsr_app") -> bool:
        _ensure_doosan_ws_pythonpath(self.ws)

        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
            from dsr_msgs2.srv import (
                ConfigCreateTcp,
                GetCurrentPosj,
                GetCurrentPosx,
                GetCurrentTcp,
                MoveJoint,
                MoveLine,
                SetCurrentTcp,
                SetRobotControl,
                SetRobotMode,
            )
        except ImportError as exc:
            print()
            print("[로봇] doosan_ws ROS2 패키지를 import하지 못했습니다.")
            print("       아래를 같은 터미널에서 실행한 뒤 다시 시도하세요:")
            print("         source /opt/ros/$ROS_DISTRO/setup.bash")
            print(f"         source {self.ws}/install/setup.bash")
            print("       (setup.bash 가 LD_LIBRARY_PATH 에 dsr_msgs2/lib 을 넣어 줍니다)")
            print(f"       ImportError: {exc}")
            print()
            return False

        self._rclpy = rclpy
        self._SetRobotMode = SetRobotMode
        self._GetCurrentPosx = GetCurrentPosx
        self._GetCurrentPosj = GetCurrentPosj
        self._MoveLine = MoveLine
        self._MoveJoint = MoveJoint
        self._SetRobotControl = SetRobotControl
        self._ConfigCreateTcp = ConfigCreateTcp
        self._SetCurrentTcp = SetCurrentTcp
        self._GetCurrentTcp = GetCurrentTcp

        # 손가락 그리퍼 서비스 (dsr_gripper) — bringup과 별도 노드
        try:
            from dsr_gripper_interfaces.srv import GripperCmd

            self._GripperCmd = GripperCmd
        except ImportError:
            self._GripperCmd = None
            print(
                "[로봇] dsr_gripper_interfaces 없음 — "
                "손가락 열기/닫기 버튼은 비활성될 수 있습니다."
            )

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
        except Exception as exc:
            print(f"[로봇] rclpy.init: {exc}")

        try:
            import DR_init

            DR_init.__dsr__id = self.robot_id
            DR_init.__dsr__model = self.robot_model
        except ImportError:
            DR_init = None

        try:
            self.node = rclpy.create_node(node_name, namespace=self.robot_id)
        except Exception as exc:
            print(f"[로봇] ROS2 노드 생성 실패: {exc}")
            return False

        if DR_init is not None:
            DR_init.__dsr__node = self.node

        # 백그라운드 spin (GUI/다른 스레드에서 서비스 호출 가능)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="dsr_ros_spin"
        )
        self._spin_thread.start()

        prefix_candidates = ["dsr_controller2", ""]
        self._cli_set_mode = None
        deadline = time.time() + timeout_s
        print(
            f"[로봇] 연결 대기... ns=/{self.robot_id} model={self.robot_model}"
        )

        while time.time() < deadline and self._cli_set_mode is None:
            for prefix in prefix_candidates:
                set_mode_srv = (
                    f"{prefix}/system/set_robot_mode"
                    if prefix
                    else "system/set_robot_mode"
                )
                cli = self.node.create_client(SetRobotMode, set_mode_srv)
                if cli.wait_for_service(timeout_sec=0.5):
                    self.service_prefix = prefix
                    self._cli_set_mode = cli

                    def _srv(name: str) -> str:
                        return f"{prefix}/{name}" if prefix else name

                    self._cli_get_posx = self.node.create_client(
                        GetCurrentPosx, _srv("aux_control/get_current_posx")
                    )
                    self._cli_get_posj = self.node.create_client(
                        GetCurrentPosj, _srv("aux_control/get_current_posj")
                    )
                    self._cli_movel = self.node.create_client(
                        MoveLine, _srv("motion/move_line")
                    )
                    self._cli_movej = self.node.create_client(
                        MoveJoint, _srv("motion/move_joint")
                    )
                    self._cli_set_control = self.node.create_client(
                        SetRobotControl, _srv("system/set_robot_control")
                    )
                    self._cli_add_tcp = self.node.create_client(
                        ConfigCreateTcp, _srv("tcp/config_create_tcp")
                    )
                    self._cli_set_tcp = self.node.create_client(
                        SetCurrentTcp, _srv("tcp/set_current_tcp")
                    )
                    self._cli_get_tcp = self.node.create_client(
                        GetCurrentTcp, _srv("tcp/get_current_tcp")
                    )
                    print(f"[로봇] 서비스 발견: /{self.robot_id}/{set_mode_srv}")
                    break
                self.node.destroy_client(cli)
            else:
                print("[로봇] set_robot_mode 대기 중...")

        if self._cli_set_mode is None:
            print("[로봇] set_robot_mode 서비스가 없습니다. bringup을 확인하세요.")
            self.close()
            return False

        # 손가락 그리퍼: /dsr01/gripper/cmd (gripper_service 노드)
        if self._GripperCmd is not None:
            grip_srv = f"/{self.robot_id}/gripper/cmd"
            self._cli_gripper = self.node.create_client(self._GripperCmd, grip_srv)
            if self._cli_gripper.wait_for_service(timeout_sec=1.0):
                print(f"[로봇] 그리퍼 서비스 발견: {grip_srv}")
            else:
                print(
                    f"[로봇] 그리퍼 서비스 대기 중: {grip_srv}\n"
                    "       → ros2 run dsr_gripper gripper_service"
                )

        self.connected = True
        return True

    def gripper_service_ready(self, timeout_s: float = 0.5) -> bool:
        if self._cli_gripper is None or self._GripperCmd is None:
            return False
        return bool(self._cli_gripper.wait_for_service(timeout_sec=timeout_s))

    def gripper_cmd(
        self,
        position: int,
        current: int = GRIPPER_CURRENT_CUSTOM,
        timeout_s: float = GRIPPER_CMD_TIMEOUT_S,
        *,
        settle_s: Optional[float] = None,
    ) -> bool:
        """
        RH-P12-RN stroke 명령.
        position: 0=완전닫힘 ~ 750=완전열림 (dsr_gripper 규약)
        settle_s: 성공 후 정착 대기(초). None이면 GRIPPER_SETTLE_S.
                  이동 전에 반드시 열기/닫기가 끝나도록 기본 대기.
        """
        if not self.connected or self._cli_gripper is None or self._GripperCmd is None:
            print("[로봇] gripper_cmd: 클라이언트 없음 (dsr_gripper 패키지/연결 확인)")
            return False
        if not self._cli_gripper.wait_for_service(timeout_sec=3.0):
            print(
                "[로봇] /{}/gripper/cmd 서비스 없음 → "
                "'ros2 run dsr_gripper gripper_service' 실행 여부 확인".format(
                    self.robot_id
                )
            )
            return False
        pos = int(max(0, min(750, int(position))))
        cur = int(max(0, int(current)))
        with self._lock:
            req = self._GripperCmd.Request()
            req.position = pos
            req.current = cur
            future = self._cli_gripper.call_async(req)
            result = self._wait_future(future, timeout_s)
        ok = result is not None and bool(getattr(result, "success", False))
        print(
            f"[로봇] gripper_cmd(pos={pos}, current={cur}) "
            f"{'OK' if ok else 'FAIL'}"
        )
        if ok:
            wait = float(GRIPPER_SETTLE_S if settle_s is None else settle_s)
            if wait > 0:
                print(f"[로봇] 그리퍼 정착 대기 {wait:.1f}s (이후 이동 가능)")
                time.sleep(wait)
        return ok

    def gripper_open(
        self,
        current: int = GRIPPER_CURRENT_OPEN,
        *,
        settle_s: Optional[float] = None,
    ) -> bool:
        return self.gripper_cmd(GRIPPER_POS_OPEN, current, settle_s=settle_s)

    def gripper_close(
        self,
        current: int = GRIPPER_CURRENT_CLOSE,
        *,
        settle_s: Optional[float] = None,
    ) -> bool:
        return self.gripper_cmd(GRIPPER_POS_CLOSE, current, settle_s=settle_s)

    def gripper_custom_close(self, *, settle_s: Optional[float] = None) -> bool:
        """설정의 커스텀 닫기 stroke로 이동 (기본=반만 닫힘 375)."""
        return self.gripper_cmd(
            int(self.custom_close_pos),
            int(self.custom_close_current),
            settle_s=settle_s,
        )

    def set_custom_close(
        self, position: int, current: Optional[int] = None
    ) -> None:
        self.custom_close_pos = int(max(0, min(750, int(position))))
        if current is not None:
            self.custom_close_current = int(max(0, int(current)))
        print(
            f"[로봇] 커스텀닫기 설정: pos={self.custom_close_pos} "
            f"(0닫힘~750열림), current={self.custom_close_current}"
        )

    def set_mode(self, mode: int) -> bool:
        if not self.connected or self._cli_set_mode is None:
            return False
        with self._lock:
            req = self._SetRobotMode.Request()
            req.robot_mode = int(mode)
            future = self._cli_set_mode.call_async(req)
            result = self._wait_future(future, self.service_timeout_s)
        ok = result is not None and bool(result.success)
        label = "MANUAL" if mode == ROBOT_MODE_MANUAL else "AUTONOMOUS"
        if ok:
            self.robot_mode = int(mode)
            print(f"[로봇] 모드 → {label}")
        else:
            print(f"[로봇] set_robot_mode({label}) 실패 result={result}")
        return ok

    def is_manual_mode(self) -> bool:
        return int(self.robot_mode) == ROBOT_MODE_MANUAL

    def set_manual_mode(self) -> bool:
        """펜던트/직접교시용 MANUAL. 자동 movel/movej 는 앱에서 막습니다."""
        return self.set_mode(ROBOT_MODE_MANUAL)

    def set_autonomous_mode(self, servo: bool = True) -> bool:
        """앱 자동 이동용 AUTONOMOUS."""
        ok = self.set_mode(ROBOT_MODE_AUTONOMOUS)
        if ok and servo:
            self.servo_on()
        return ok

    def servo_on(self) -> bool:
        """가능하면 Servo On (enum 값 실패해도 이동은 시도)."""
        if self._cli_set_control is None or self._SetRobotControl is None:
            return False
        if not self._cli_set_control.wait_for_service(timeout_sec=1.0):
            print("[로봇] set_robot_control 서비스 없음 (건너뜀)")
            return False

        # CONTROL_ENABLE_OPERATION=1, RESET_SAFE_STOP=2, SERVO_ON/RESET_SAFE_OFF=3
        candidates = [3, 2, 1]
        for val in candidates:
            with self._lock:
                req = self._SetRobotControl.Request()
                req.robot_control = int(val)
                future = self._cli_set_control.call_async(req)
                result = self._wait_future(future, 5.0)
            if result is not None and getattr(result, "success", False):
                print(f"[로봇] set_robot_control({val}) 성공")
                return True
            print(f"[로봇] set_robot_control({val}) 실패/무시")
        return False

    def connect_and_set_manual(self, timeout_s: float = 20.0) -> bool:
        if not self.connect(timeout_s=timeout_s, node_name="hand_eye_calib_dsr"):
            return False
        if not self.set_mode(ROBOT_MODE_MANUAL):
            self.close()
            return False
        print("       펜던트 조그 또는 플랜지 직접교시로 자세를 잡으세요.")
        return True

    def connect_and_set_autonomous(
        self,
        timeout_s: float = 20.0,
        *,
        setup_gripper: bool = True,
        straighten_down: bool = True,
    ) -> bool:
        if not self.connect(timeout_s=timeout_s, node_name="click_and_move_dsr"):
            return False
        if not self.set_mode(ROBOT_MODE_AUTONOMOUS):
            self.close()
            return False
        self.servo_on()

        if setup_gripper:
            if not self.ensure_gripper_tcp():
                print("[로봇] 경고: 그리퍼 TCP 설정 실패 — flange 기준으로 동작할 수 있습니다.")
            else:
                self.gripper_ready = True

        # 연결 직후 pose 한 번 읽어 통신 확인
        pos = self.get_posx_mm_deg()
        if pos is not None:
            print(
                "[로봇] 현재 pose(TCP): "
                f"[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}, "
                f"{pos[3]:.1f}, {pos[4]:.1f}, {pos[5]:.1f}]"
            )
        else:
            print("[로봇] 경고: 현재 pose를 읽지 못했습니다.")

        # 기동 직후: 기본 조인트 자세로 교정한 뒤 이후 이동 허용
        if straighten_down:
            print(
                "[로봇] 기동 전 자세 교정: 기본자세 "
                + "["
                + ", ".join(f"{v:.0f}" for v in HOME_POSJ_DEG)
                + "] deg (J1..J6)..."
            )
            if not self.go_home_posj():
                print("[로봇] 경고: 기본자세 교정 실패 — 수동으로 자세를 확인하세요.")
            else:
                print("[로봇] 기본자세 교정 완료 — 이동 준비됨")
        return True

    def _srv_path(self, name: str) -> str:
        if self.service_prefix:
            return f"{self.service_prefix}/{name}"
        return name

    def get_tcp_name(self) -> Optional[str]:
        if not self.connected or self._cli_get_tcp is None:
            return None
        if not self._cli_get_tcp.wait_for_service(timeout_sec=1.0):
            print("[로봇] tcp/get_current_tcp 서비스 없음")
            return None
        with self._lock:
            req = self._GetCurrentTcp.Request()
            future = self._cli_get_tcp.call_async(req)
            result = self._wait_future(future, self.service_timeout_s)
        if result is None:
            return None
        if not bool(getattr(result, "success", True)):
            return None
        info = getattr(result, "info", None)
        if info is None:
            return None
        return str(info)

    def add_tcp(self, name: str, pos: list[float]) -> bool:
        """플랜지 기준 TCP 등록 (config_create_tcp). 이미 있으면 False일 수 있음."""
        if not self.connected or self._cli_add_tcp is None:
            return False
        if not self._cli_add_tcp.wait_for_service(timeout_sec=2.0):
            print("[로봇] tcp/config_create_tcp 서비스 없음")
            return False
        with self._lock:
            req = self._ConfigCreateTcp.Request()
            req.name = str(name)
            req.pos = [float(v) for v in pos[:6]]
            future = self._cli_add_tcp.call_async(req)
            result = self._wait_future(future, self.service_timeout_s)
        ok = result is not None and bool(getattr(result, "success", False))
        print(
            f"[로봇] add_tcp('{name}', {[round(v, 1) for v in pos[:6]]}) "
            f"{'OK' if ok else 'FAIL/이미존재'}"
        )
        return ok

    def set_tcp(self, name: str) -> bool:
        """현재 TCP 선택 (set_current_tcp). 이후 posx/movel 이 이 TCP 기준."""
        if not self.connected or self._cli_set_tcp is None:
            return False
        if not self._cli_set_tcp.wait_for_service(timeout_sec=2.0):
            print("[로봇] tcp/set_current_tcp 서비스 없음")
            return False
        with self._lock:
            req = self._SetCurrentTcp.Request()
            req.name = str(name)
            future = self._cli_set_tcp.call_async(req)
            result = self._wait_future(future, self.service_timeout_s)
        ok = result is not None and bool(getattr(result, "success", False))
        print(f"[로봇] set_tcp('{name}') {'OK' if ok else 'FAIL'}")
        return ok

    def ensure_gripper_tcp(
        self,
        name: str = GRIPPER_TCP_NAME,
        pos: Optional[list[float]] = None,
    ) -> bool:
        """
        그리퍼 tip TCP 등록·활성.
        posx / movel 목표가 플랜지가 아닌 그리퍼 기준으로 동작하게 합니다.
        """
        if pos is None:
            pos = list(GRIPPER_TCP_POS)
        # 이미 같은 TCP면 재설정만
        cur = self.get_tcp_name()
        if cur is not None:
            print(f"[로봇] 현재 TCP: '{cur}'")
        self.add_tcp(name, pos)  # 이미 있어도 실패할 수 있음 → set 은 계속
        ok = self.set_tcp(name)
        if ok:
            after = self.get_tcp_name()
            print(f"[로봇] 활성 TCP(그리퍼): '{after or name}'")
        return ok

    @staticmethod
    def _angle_diff_deg(a: float, b: float) -> float:
        d = abs(float(a) - float(b)) % 360.0
        return min(d, 360.0 - d)

    def ori_close(
        self,
        ori_a: list[float],
        ori_b: list[float],
        tol_deg: float = ORI_TOL_DEG,
    ) -> bool:
        """
        두 Euler 자세가 허용각 이내인지.
        Ry≈180 ZYZ 특이점에서 [0,180,yaw]≡[-yaw,180,0] 이므로
        성분 비교가 아니라 회전행렬 각도로 판정.
        """
        if len(ori_a) < 3 or len(ori_b) < 3:
            return False
        return ori_rot_diff_deg(ori_a, ori_b) <= float(tol_deg)

    def is_gripper_down(
        self,
        posx: Optional[list[float]] = None,
        tol_deg: float = ORI_TOL_DEG,
    ) -> bool:
        if posx is None:
            posx = self.get_posx_mm_deg()
        if posx is None or len(posx) < 6:
            return False
        return self.ori_close(posx[3:6], list(GRIPPER_DOWN_ORI_DEG), tol_deg)

    def rotate_gripper_to(
        self,
        ori_deg: list[float],
        vel: Optional[list[float]] = None,
        acc: Optional[list[float]] = None,
        *,
        force: bool = False,
    ) -> bool:
        """
        현재 TCP XYZ는 유지하고 그리퍼(자세)만 목표 ori로 맞춤.
        하향 목표는 [yaw,180,0] 으로 정규화. 완료 후 실제 툴X yaw 로그.
        """
        cur = self.get_posx_mm_deg()
        if cur is None or len(cur) < 6:
            print("[로봇] rotate_gripper_to: pose 읽기 실패")
            return False
        ori = canonicalize_down_ori([float(v) for v in ori_deg[:3]])
        diff = ori_rot_diff_deg(cur[3:6], ori)
        if not force and diff <= ORI_TOL_DEG:
            print(
                f"[로봇] 이미 목표 그리퍼 자세 (Δ={diff:.1f}°) — 회전 생략  "
                f"cur_toolX_yaw={effective_tool_x_yaw_deg(cur[3:6]):.1f}°"
            )
            return True
        rot_pose = [
            float(cur[0]),
            float(cur[1]),
            float(cur[2]),
            ori[0],
            ori[1],
            ori[2],
        ]
        print(
            "[로봇] 그리퍼 자세 조정 (위치 고정) → "
            f"ori=[{ori[0]:.1f}, {ori[1]:.1f}, {ori[2]:.1f}]  "
            f"(정규형 [yaw,180,0], 목표툴X={effective_tool_x_yaw_deg(ori):.1f}°, "
            f"현재Δ={diff:.1f}°)"
        )
        if vel is None:
            vel = [30.0, 30.0]
        if acc is None:
            acc = [30.0, 30.0]
        ok = self.movel(rot_pose, vel=vel, acc=acc)
        if ok:
            time.sleep(0.2)
            after = self.get_posx_mm_deg()
            if after is not None and len(after) >= 6:
                adiff = ori_rot_diff_deg(after[3:6], ori)
                print(
                    f"[로봇] 회전 후 pose ori="
                    f"[{after[3]:.1f},{after[4]:.1f},{after[5]:.1f}]  "
                    f"toolX_yaw={effective_tool_x_yaw_deg(after[3:6]):.1f}°  "
                    f"목표대비Δ={adiff:.1f}°"
                )
                if adiff > max(ORI_TOL_DEG * 2.0, 10.0):
                    print(
                        "[로봇] 경고: 회전 후에도 목표 자세와 큼 — "
                        "bringup Euler(ZYZ)·싱귤래리티/관절한계 확인"
                    )
        return ok

    def is_at_home_posj(
        self,
        posj: Optional[list[float]] = None,
        tol_deg: float = HOME_POSJ_TOL_DEG,
    ) -> bool:
        if posj is None:
            posj = self.get_posj_deg()
        if posj is None or len(posj) < 6:
            return False
        for i in range(6):
            if self._angle_diff_deg(float(posj[i]), float(HOME_POSJ_DEG[i])) > tol_deg:
                return False
        return True

    def go_home_posj(
        self,
        vel: float = 25.0,
        acc: float = 25.0,
        force: bool = False,
    ) -> bool:
        """
        기동/복귀용 기본 조인트 자세로 movej.
        HOME_POSJ_DEG = [0, 0, 90, 0, 90, 0] (J1..J6, deg).
        이미 기본자세면 True (이동 생략).
        """
        home = [float(v) for v in HOME_POSJ_DEG]
        if not force and self.is_at_home_posj():
            print("[로봇] 이미 기본 조인트 자세")
            return True
        print(
            "[로봇] 기본자세 교정 → "
            + ", ".join(f"J{i+1}={home[i]:.1f}" for i in range(6))
        )
        return self.movej(home, vel=vel, acc=acc)

    def orient_gripper_down(
        self,
        vel: Optional[list[float]] = None,
        acc: Optional[list[float]] = None,
        force: bool = False,
    ) -> bool:
        """현재 TCP XYZ는 유지하고 자세만 하향(정자세)으로 교정."""
        return self.rotate_gripper_to(
            list(GRIPPER_DOWN_ORI_DEG), vel=vel, acc=acc, force=force
        )

    def movel_gripper(
        self,
        xyz_mm: list[float],
        *,
        vel: Optional[list[float]] = None,
        acc: Optional[list[float]] = None,
        straighten_first: bool = True,
        z_up_mm: Optional[float] = None,
        ori_deg: Optional[list[float]] = None,
        yaw_deg: Optional[float] = None,
        rotate_first: bool = True,
        phase_cb: Optional[Any] = None,
    ) -> bool:
        """
        그리퍼 tip + 목표 자세로 이동.
        순서 (rotate_first=True):
          1) 자세가 다르면 현재 위치에서 그리퍼만 조정 (완료 대기)
          2) 현재 Z 유지한 채 목표 XY로 이동
          3) 목표 Z로 하강/상승
        """
        if vel is None:
            vel = [40.0, 40.0]
        if acc is None:
            acc = [40.0, 40.0]

        lift = float(TARGET_Z_UP_MM if z_up_mm is None else z_up_mm)
        z_cmd = float(xyz_mm[2]) + lift
        tx = float(xyz_mm[0])
        ty = float(xyz_mm[1])
        if ori_deg is not None:
            ori = canonicalize_down_ori([float(v) for v in ori_deg[:3]])
        elif yaw_deg is not None:
            ori = canonicalize_down_ori(gripper_ori_with_yaw(float(yaw_deg)))
        else:
            ori = list(GRIPPER_DOWN_ORI_DEG)

        def _phase(name: str) -> None:
            if phase_cb is None:
                return
            try:
                phase_cb(name)
            except Exception:
                pass

        cur = self.get_posx_mm_deg()
        do_rotate = bool(rotate_first or straighten_first)
        if do_rotate:
            need = True
            if cur is not None and len(cur) >= 6:
                need = not self.ori_close(cur[3:6], ori)
            if need:
                _phase("rotate")
                print("[로봇] 1/3 그리퍼 자세 조정 → 이후 XY → Z")
                if not self.rotate_gripper_to(ori, vel=vel, acc=acc):
                    print("[로봇] movel_gripper: 그리퍼 자세 조정 실패")
                    return False
                cur = self.get_posx_mm_deg()
            else:
                print("[로봇] 1/3 그리퍼 자세 이미 일치 — 바로 XY 이동")

        print(
            f"[로봇] Z 보정: surface={float(xyz_mm[2]):.1f} + lift={lift:.1f} "
            f"→ tip_Z={z_cmd:.1f} mm"
        )

        # 현재 Z를 유지한 채 목표 XY로 먼저 이동 (pose 없으면 XY·Z 한 번에)
        if cur is not None and len(cur) >= 3:
            z_hold = float(cur[2])
            xy_delta = math.hypot(tx - float(cur[0]), ty - float(cur[1]))
            if xy_delta > 1.0:
                target_xy = [tx, ty, z_hold, ori[0], ori[1], ori[2]]
                _phase("move_xy")
                print(
                    "[로봇] 2/3 목표 XY 이동 (Z 유지) → "
                    f"[{target_xy[0]:.1f}, {target_xy[1]:.1f}, {target_xy[2]:.1f}]"
                )
                if not self.movel(target_xy, vel=vel, acc=acc):
                    print("[로봇] movel_gripper: XY 이동 실패")
                    return False
            else:
                print(f"[로봇] 2/3 XY 이미 근접 (Δ={xy_delta:.1f}mm) — Z만 이동")
        else:
            print("[로봇] 현재 pose 없음 — XY·Z를 한 번에 이동")
            target = [tx, ty, z_cmd, ori[0], ori[1], ori[2]]
            _phase("move")
            return self.movel(target, vel=vel, acc=acc)

        target_z = [tx, ty, z_cmd, ori[0], ori[1], ori[2]]
        z_delta = abs(z_cmd - z_hold)
        if z_delta <= 1.0:
            print(f"[로봇] 3/3 Z 이미 근접 (Δ={z_delta:.1f}mm) — 완료")
            return True

        _phase("move_z")
        print(
            "[로봇] 3/3 목표 Z 이동 → "
            f"[{target_z[0]:.1f}, {target_z[1]:.1f}, {target_z[2]:.1f}]"
        )
        return self.movel(target_z, vel=vel, acc=acc)

    def get_posx_mm_deg(self) -> Optional[list[float]]:
        if not self.connected or self._cli_get_posx is None:
            return None
        if not self._cli_get_posx.wait_for_service(timeout_sec=1.0):
            print("[로봇] get_current_posx 서비스 없음")
            return None

        with self._lock:
            req = self._GetCurrentPosx.Request()
            req.ref = DR_BASE
            future = self._cli_get_posx.call_async(req)
            result = self._wait_future(future, self.service_timeout_s)

        if result is None or not result.success:
            print("[로봇] get_current_posx 실패")
            return None
        try:
            data = list(result.task_pos_info[0].data)
        except Exception as exc:
            print(f"[로봇] posx 파싱 실패: {exc}")
            return None
        if len(data) < 6:
            return None
        return [float(x) for x in data[:6]]

    def get_posj_deg(self) -> Optional[list[float]]:
        """현재 조인트각 [J1..J6] deg."""
        if not self.connected or self._cli_get_posj is None:
            return None
        if not self._cli_get_posj.wait_for_service(timeout_sec=1.0):
            print("[로봇] get_current_posj 서비스 없음")
            return None
        with self._lock:
            req = self._GetCurrentPosj.Request()
            future = self._cli_get_posj.call_async(req)
            result = self._wait_future(future, self.service_timeout_s)
        if result is None or not bool(getattr(result, "success", False)):
            print("[로봇] get_current_posj 실패")
            return None
        try:
            pos = [float(x) for x in list(result.pos)[:6]]
        except Exception as exc:
            print(f"[로봇] posj 파싱 실패: {exc}")
            return None
        if len(pos) < 6:
            return None
        return pos

    def movej(
        self,
        posj_deg: list[float],
        vel: float = 30.0,
        acc: float = 30.0,
    ) -> bool:
        """관절 공간 이동 (절대각, deg). posj = [J1..J6]."""
        if not self.connected or self._cli_movej is None:
            print("[로봇] movej: 미연결")
            return False
        if not self._cli_movej.wait_for_service(timeout_sec=2.0):
            print("[로봇] motion/move_joint 서비스 없음")
            return False
        if not self.set_mode(ROBOT_MODE_AUTONOMOUS):
            print("[로봇] AUTONOMOUS 전환 실패 — movej 중단")
            return False

        req = self._MoveJoint.Request()
        req.pos = [float(v) for v in posj_deg[:6]]
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 0

        print(
            "[로봇] movej 요청 → "
            + ", ".join(f"J{i+1}={req.pos[i]:.1f}" for i in range(6))
            + f"  vel={req.vel} acc={req.acc}"
        )
        with self._lock:
            future = self._cli_movej.call_async(req)
            result = self._wait_future(future, self.move_timeout_s)
        if result is None:
            print("[로봇] movej 타임아웃/무응답")
            return False
        if not result.success:
            print("[로봇] movej 거부됨 (success=False)")
            return False
        print("[로봇] movej 완료 (success=True)")
        return True

    def movel(
        self,
        posx_mm_deg: list[float],
        vel: Optional[list[float]] = None,
        acc: Optional[list[float]] = None,
    ) -> bool:
        """직선 이동 (베이스 절대좌표, TCP). posx = [x,y,z,rx,ry,rz] mm/deg."""
        if not self.connected or self._cli_movel is None:
            print("[로봇] movel: 미연결")
            return False
        if not self._cli_movel.wait_for_service(timeout_sec=2.0):
            print("[로봇] motion/move_line 서비스 없음")
            print("       ros2 service list | grep move_line  로 확인하세요.")
            return False

        if vel is None:
            vel = [40.0, 40.0]
        if acc is None:
            acc = [40.0, 40.0]

        # 이동 직전 모드 재확인
        if not self.set_mode(ROBOT_MODE_AUTONOMOUS):
            print("[로봇] AUTONOMOUS 전환 실패 — 이동 중단")
            return False

        req = self._MoveLine.Request()
        req.pos = [float(v) for v in posx_mm_deg[:6]]
        req.vel = [float(vel[0]), float(vel[1])]
        req.acc = [float(acc[0]), float(acc[1])]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 0  # SYNC (완료까지 대기)

        print(
            "[로봇] movel 요청 → "
            f"[{req.pos[0]:.1f}, {req.pos[1]:.1f}, {req.pos[2]:.1f}, "
            f"{req.pos[3]:.1f}, {req.pos[4]:.1f}, {req.pos[5]:.1f}] "
            f"vel={req.vel} acc={req.acc}"
        )

        with self._lock:
            future = self._cli_movel.call_async(req)
            result = self._wait_future(future, self.move_timeout_s)

        if result is None:
            print("[로봇] movel 타임아웃/무응답")
            print("       bringup 로그에 movel_cb 가 찍히는지 확인하세요.")
            return False
        if not result.success:
            print("[로봇] movel 거부됨 (success=False)")
            print("       원인 후보: 도달 불가 좌표, 충돌/세이프티, Manual 키, Servo Off")
            return False
        print("[로봇] movel 완료 (success=True)")
        return True

    def close(self) -> None:
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=2.0)
            except Exception:
                try:
                    self._executor.shutdown()
                except Exception:
                    pass
            self._executor = None

        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception as exc:
                print(f"[로봇] destroy_node: {exc}")
            self.node = None

        if self._rclpy is not None:
            try:
                if self._rclpy.ok():
                    self._rclpy.shutdown()
            except Exception:
                pass

        self.connected = False
        print("[로봇] ROS2 연결 종료")
