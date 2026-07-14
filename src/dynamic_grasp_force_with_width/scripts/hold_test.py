"""홀드 테스트 — 주어진 current 로 상자를 물고 들어올려 버티는지 본다.

    python hold_test.py <current>

슬립 판정:
  - 픽 자세에서 문 채로 J2 토크 측정 (before)
  - 들어올렸다가 같은 자세로 복귀
  - 다시 J2 토크 측정 (after)
  - |after - before| 가 크면 물체가 빠진 것. 750g 이 빠지면 약 4.6 Nm 움직인다.
    (마찰 히스테리시스로 인한 정상 변동은 ~1 Nm 이하)
"""
import statistics
import sys
import time

import rclpy
import DR_init

CURRENT = int(sys.argv[1]) if len(sys.argv) > 1 else 350

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("hold_test", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import (
    movel, posx, wait, DR_BASE,
    get_robot_mode, get_current_posj, get_current_posx,
    get_tool, read_data_rt,
)
from dsr_gripper import gripper_cmd

NEEDED = ("get_robot_mode", "get_current_posj", "get_current_posx",
          "get_current_tool", "move_line")
_need = [c for c in node.clients if any(k in c.srv_name for k in NEEDED)]
_t0 = time.time()
while time.time() - _t0 < 20.0:
    _pending = [c for c in _need if not c.service_is_ready()]
    if not _pending:
        break
    rclpy.spin_once(node, timeout_sec=0.1)
if _pending:
    raise RuntimeError("디스커버리 실패: " + ", ".join(c.srv_name for c in _pending))

# ── 상수 ───────────────────────────────────────────
PICK_POSE = [-13.181, -23.339, 94.612, 15.686, 55.968, -15.458]
WIDTH_MM = 85.0
SQUEEZE_MM = 3.0
OPEN_WIDTH_MM, STROKE_OPEN = 109.0, 750

STROKE_GRIP = round((WIDTH_MM - SQUEEZE_MM) / OPEN_WIDTH_MM * STROKE_OPEN)   # 564
STROKE_FREE = round(WIDTH_MM / OPEN_WIDTH_MM * STROKE_OPEN)                  # 585

LIFT_MM = 60.0
SPEED_L = 40
DROP_THRESHOLD_NM = 1.5     # 이보다 크게 변하면 물체가 빠진 것으로 본다


def motor_torque(n=25):
    rows = []
    for _ in range(n):
        d = read_data_rt()
        if d is not None:
            rows.append(list(d.actual_motor_torque))
        time.sleep(0.04)
    if not rows:
        raise RuntimeError("read_data_rt 무응답")
    return [statistics.median(c) for c in zip(*rows)]


def tau_j2():
    return motor_torque()[1]


# ── 사전 확인 ──────────────────────────────────────
if get_robot_mode() != 1:
    raise RuntimeError("AUTONOMOUS 가 아닙니다")
if not get_tool():
    raise RuntimeError("툴 미등록")

q = get_current_posj()
dq = max(abs(a - b) for a, b in zip(q, PICK_POSE))
if dq > 0.5:
    raise RuntimeError(f"픽 자세가 아닙니다 (최대 오차 {dq:.2f}°). goto_pick.py 를 먼저 도세요.")

print(f"▸ 홀드 테스트   current={CURRENT}   폭 {WIDTH_MM:.0f}mm → stroke {STROKE_GRIP}")
print(f"  (자세 확인 OK, 오차 {dq:.3f}°)")

pick_x, _ = get_current_posx()

# ── ① 물기 ─────────────────────────────────────────
print(f"\n① 파지 (stroke {STROKE_GRIP}, current {CURRENT})")
gripper_cmd(STROKE_GRIP, current=CURRENT)
wait(2.0)

before = tau_j2()
print(f"   들기 전  tau(J2) = {before:+.3f} Nm")

# ── ② 들어올리기 ───────────────────────────────────
print(f"\n② 들어올림 (+{LIFT_MM:.0f}mm)")
up = list(pick_x)
up[2] += LIFT_MM
movel(posx(up), vel=SPEED_L, acc=SPEED_L, ref=DR_BASE)
wait(2.0)

lifted = tau_j2()
print(f"   들린 상태 tau(J2) = {lifted:+.3f} Nm")

# ── ③ 원위치 복귀 ──────────────────────────────────
print(f"\n③ 픽 자세로 복귀")
movel(posx(list(pick_x)), vel=SPEED_L, acc=SPEED_L, ref=DR_BASE)
wait(2.0)

after = tau_j2()
print(f"   복귀 후  tau(J2) = {after:+.3f} Nm")

# ── ④ 판정 ─────────────────────────────────────────
delta = abs(after - before)
print(f"\n{'='*46}")
print(f"  current {CURRENT}   Δtau = {delta:.3f} Nm")
if delta > DROP_THRESHOLD_NM:
    print(f"  ❌ 놓쳤을 가능성 큼 (Δ {delta:.3f} > {DROP_THRESHOLD_NM} Nm)")
    print(f"     → current 를 올려서 다시 시도하세요")
else:
    print(f"  ✅ 버틴 것으로 보임 (Δ {delta:.3f} ≤ {DROP_THRESHOLD_NM} Nm)")
    print(f"     → 눈으로도 확인해 주세요 (상자가 그리퍼에서 흘러내리지 않았는지)")
print(f"{'='*46}")
print("\n※ 상자는 아직 물고 있습니다. 다음 명령을 기다립니다.")

rclpy.shutdown()
