"""
두산 로봇 클라이언트 — /home/newuser/ocr/doosan_ws (ROS2 + dsr_msgs2)

GUI 앱에서도 안전하게 동작하도록 MultiThreadedExecutor 를 백그라운드에서 spin 합니다.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional


DOOSAN_WS = Path("/home/newuser/ocr/doosan_ws")
DEFAULT_ROBOT_ID = "dsr01"
DEFAULT_ROBOT_MODEL = "e0509"
DEFAULT_ROBOT_IP = "192.168.137.100"
DEFAULT_ROBOT_PORT = 12345

ROBOT_MODE_MANUAL = 0
ROBOT_MODE_AUTONOMOUS = 1
DR_BASE = 0
DR_MV_MOD_ABS = 0


def _ensure_doosan_ws_pythonpath(ws: Path = DOOSAN_WS) -> None:
    install = ws / "install"
    if not install.is_dir():
        return

    for site in install.glob("*/lib/python*/site-packages"):
        p = str(site)
        if p not in sys.path:
            sys.path.insert(0, p)
    for imp in install.glob("*/lib/*/imp"):
        p = str(imp)
        if p not in sys.path:
            sys.path.insert(0, p)

    prefix = os.environ.get("AMENT_PREFIX_PATH", "")
    parts = [str(p) for p in install.iterdir() if p.is_dir()]
    if parts:
        os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join(
            parts + ([prefix] if prefix else [])
        )


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
    _cli_movel: Any = field(default=None, repr=False)
    _cli_set_control: Any = field(default=None, repr=False)
    _rclpy: Any = field(default=None, repr=False)
    _executor: Any = field(default=None, repr=False)
    _spin_thread: Any = field(default=None, repr=False)
    _SetRobotMode: Any = field(default=None, repr=False)
    _GetCurrentPosx: Any = field(default=None, repr=False)
    _MoveLine: Any = field(default=None, repr=False)
    _SetRobotControl: Any = field(default=None, repr=False)
    _lock: Any = field(default_factory=threading.Lock, repr=False)

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
                GetCurrentPosx,
                MoveLine,
                SetRobotControl,
                SetRobotMode,
            )
        except ImportError as exc:
            print()
            print("[로봇] doosan_ws ROS2 패키지를 import하지 못했습니다.")
            print("       source /opt/ros/$ROS_DISTRO/setup.bash")
            print(f"       source {self.ws}/install/setup.bash")
            print(f"       ImportError: {exc}")
            print()
            return False

        self._rclpy = rclpy
        self._SetRobotMode = SetRobotMode
        self._GetCurrentPosx = GetCurrentPosx
        self._MoveLine = MoveLine
        self._SetRobotControl = SetRobotControl

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
                    self._cli_movel = self.node.create_client(
                        MoveLine, _srv("motion/move_line")
                    )
                    self._cli_set_control = self.node.create_client(
                        SetRobotControl, _srv("system/set_robot_control")
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

        self.connected = True
        return True

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
            print(f"[로봇] 모드 → {label}")
        else:
            print(f"[로봇] set_robot_mode({label}) 실패 result={result}")
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

    def connect_and_set_autonomous(self, timeout_s: float = 20.0) -> bool:
        if not self.connect(timeout_s=timeout_s, node_name="click_and_move_dsr"):
            return False
        if not self.set_mode(ROBOT_MODE_AUTONOMOUS):
            self.close()
            return False
        self.servo_on()
        # 연결 직후 pose 한 번 읽어 통신 확인
        pos = self.get_posx_mm_deg()
        if pos is not None:
            print(
                "[로봇] 현재 pose: "
                f"[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}, "
                f"{pos[3]:.1f}, {pos[4]:.1f}, {pos[5]:.1f}]"
            )
        else:
            print("[로봇] 경고: 현재 pose를 읽지 못했습니다.")
        return True

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
