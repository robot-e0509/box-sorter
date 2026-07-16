"""그리퍼 개방 (상자 교체용)."""
import time
import rclpy
import DR_init

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("release_grip", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import wait, get_robot_mode
from dsr_gripper import gripper_cmd

_need = [c for c in node.clients if "get_robot_mode" in c.srv_name]
_t0 = time.time()
while time.time() - _t0 < 20.0:
    if all(c.service_is_ready() for c in _need):
        break
    rclpy.spin_once(node, timeout_sec=0.1)

if get_robot_mode() != 1:
    raise RuntimeError("AUTONOMOUS 가 아닙니다")

print("▸ 그리퍼 개방 (stroke 750 = 109mm)")
gripper_cmd(750, current=50)
wait(1.5)
print("  ✓ 열림 — 상자를 꺼내 내용물을 바꾸세요")

rclpy.shutdown()
