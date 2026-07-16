"""토크만 읽는다. 팔도 그리퍼도 건드리지 않는다."""
import statistics
import time

import rclpy
import DR_init

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("read_tau", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import get_robot_mode, read_data_rt

_need = [c for c in node.clients if "get_robot_mode" in c.srv_name]
_t0 = time.time()
while time.time() - _t0 < 20.0:
    if all(c.service_is_ready() for c in _need):
        break
    rclpy.spin_once(node, timeout_sec=0.1)
get_robot_mode()

EMPTY_TAU = -5.86

print("▸ 토크 읽기 (움직임 없음) — 5초 간격 6회\n")
print(f"   {'경과':>5}  {'tau(J2)':>9}   {'빈그리퍼 대비':>12}")
for i in range(6):
    rows = []
    for _ in range(20):
        d = read_data_rt()
        if d is not None:
            rows.append(list(d.actual_motor_torque))
        time.sleep(0.04)
    tau = statistics.median([r[1] for r in rows])
    print(f"   {i*5:>4}s  {tau:+9.3f}   {tau - EMPTY_TAU:+12.3f} Nm")
    if i < 5:
        time.sleep(4)

print(f"\n  기준: 빈 그리퍼 {EMPTY_TAU:+.2f} · 750g {-10.044:+.2f} · 490g {-8.401:+.2f}")
print(f"  294g 이 제대로 매달려 있다면 -7.2 근처여야 합니다.")

rclpy.shutdown()
