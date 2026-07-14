"""그리퍼를 파지 조건으로 닫고, 물체가 물렸는지 토크로 확인만 한다. dither 없음."""
import statistics
import time

import rclpy
import DR_init

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("grip_only", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import wait, get_robot_mode, get_current_posj, get_tool, read_data_rt
from dsr_gripper import gripper_cmd

NEEDED = ("get_robot_mode", "get_current_posj", "get_current_tool")
_need = [c for c in node.clients if any(k in c.srv_name for k in NEEDED)]
_t0 = time.time()
while time.time() - _t0 < 20.0:
    if all(c.service_is_ready() for c in _need):
        break
    rclpy.spin_once(node, timeout_sec=0.1)

STROKE_GRIP = 564          # 폭 85mm - 3mm squeeze
CURRENT = 350
WEIGHT_JOINT = 1
EMPTY_TAU = -5.86          # 실측: 이 stroke/current 에서 물체 없이 닫았을 때
PRESENCE_MARGIN = 0.6


def motor_torque(n=20):
    rows = []
    for _ in range(n):
        d = read_data_rt()
        if d is not None:
            rows.append(list(d.actual_motor_torque))
        time.sleep(0.04)
    return [statistics.median(c) for c in zip(*rows)]


if get_robot_mode() != 1:
    raise RuntimeError("AUTONOMOUS 가 아닙니다")
if get_tool() != "rh_p12_rn":
    raise RuntimeError("툴 미등록")

print(f"▸ 파지 (stroke {STROKE_GRIP} = 82mm, current {CURRENT})")
gripper_cmd(STROKE_GRIP, current=CURRENT)
wait(2.0)

tau = motor_torque()[WEIGHT_JOINT]
print(f"\n  tau(J2) = {tau:+.3f} Nm")
print(f"  빈 그리퍼 기준선 = {EMPTY_TAU:+.2f} Nm")

if tau > EMPTY_TAU - PRESENCE_MARGIN:
    print(f"\n  ❌ 물체가 없는 것 같습니다 (빈 그리퍼와 거의 같음)")
else:
    print(f"\n  ✅ 물체 물림 — 빈 그리퍼보다 {abs(tau - EMPTY_TAU):.2f} Nm 무겁습니다")
    print(f"     (참고: 750g 일 때는 -10.044 Nm 였습니다)")

print("\n※ 이 상태로 대기합니다. dither 측정은 아직 안 돌렸습니다.")

rclpy.shutdown()
