#!/usr/bin/env python3
# =====================================================================
#  그리퍼 Python API  (노트북/스크립트에서 import 해서 사용)
#
#  DSR_ROBOT2 와 동일한 패턴:
#    1) rclpy 노드를 만들어 DR_init.__dsr__node 에 등록 (연결 셀)
#    2) from dsr_gripper import gripper_open, gripper_close, gripper_cmd
#
#  내부적으로 /<namespace>/gripper/cmd 서비스(GripperCmd)를 호출합니다.
#  → gripper_service 노드가 실행 중이어야 합니다.
#     (ros2 run dsr_gripper gripper_service)
# =====================================================================

import rclpy
from dsr_gripper_interfaces.srv import GripperCmd

_grip_cli = None


def _node():
    # DR_init 은 호출 시점에만 필요 (서비스 노드 실행에는 불필요하므로 지연 import)
    import DR_init
    node = getattr(DR_init, "__dsr__node", None)
    if node is None:
        raise RuntimeError(
            "노드가 없습니다. 먼저 rclpy 노드를 만들어 DR_init.__dsr__node 에 등록하세요 "
            "(DSR_ROBOT2 연결 셀 실행)."
        )
    return node


def _namespace():
    import DR_init
    return getattr(DR_init, "__dsr__id", "dsr01")


def _client():
    global _grip_cli
    if _grip_cli is None:
        _grip_cli = _node().create_client(GripperCmd, f"/{_namespace()}/gripper/cmd")
    return _grip_cli


def gripper_cmd(position, current=200, timeout=15.0):
    """그리퍼 이동. position: 0=완전닫힘 ~ 750=완전열림, current: 힘(전류) 제한."""
    node = _node()
    cli = _client()
    if not cli.wait_for_service(timeout_sec=3.0):
        raise RuntimeError("gripper 서비스 없음 → 'ros2 run dsr_gripper gripper_service' 확인")
    req = GripperCmd.Request()
    req.position = int(max(0, min(750, position)))
    req.current = int(current)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result()
    if res is None:
        raise RuntimeError("gripper 응답 timeout")
    return res.success


def gripper_open(current=200):
    """그리퍼 열기 (stroke 750)."""
    return gripper_cmd(750, current)


def gripper_close(current=300):
    """그리퍼 닫기 / 물체 잡기 (stroke 0)."""
    return gripper_cmd(0, current)
