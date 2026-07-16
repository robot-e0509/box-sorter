"""그리퍼 개방 + 픽 자세로 이동. 저속."""
import json
import time
from pathlib import Path

import rclpy
import DR_init

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("goto_pick", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import (
    movej, posj, wait,
    get_robot_mode, get_current_posj, get_current_posx,
    get_tool, get_tcp,
)
from dsr_gripper import gripper_cmd

# 디스커버리 대기 — 안 하면 spin_until_future_complete 가 영원히 매달린다.
NEEDED = ("get_robot_mode", "get_current_posj", "get_current_posx",
          "get_current_tool", "get_current_tcp", "move_joint")
_need = [c for c in node.clients if any(k in c.srv_name for k in NEEDED)]
_t0 = time.time()
while time.time() - _t0 < 20.0:
    _pending = [c for c in _need if not c.service_is_ready()]
    if not _pending:
        break
    rclpy.spin_once(node, timeout_sec=0.1)
if _pending:
    raise RuntimeError("서비스 디스커버리 실패: " + ", ".join(c.srv_name for c in _pending))
print(f"✓ 디스커버리 완료 ({time.time()-_t0:.1f}s)")

PICK_POSE = [-13.181, -23.339, 94.612, 15.686, 55.968, -15.458]
STROKE_OPEN = 750

# ── 사전 확인 ──────────────────────────────────────
mode = get_robot_mode()
tool, tcp = get_tool(), get_tcp()
print(f"모드={mode} (1=autonomous)  tool={tool!r}  tcp={tcp!r}")
if mode != 1:
    raise RuntimeError("AUTONOMOUS 가 아닙니다. 그리퍼/모션이 안 됩니다.")
if not tool or not tcp:
    raise RuntimeError("툴이 등록돼 있지 않습니다. setup_tool.py 를 먼저 돌리세요.")

start = [round(v, 3) for v in get_current_posj()]
print("현재 자세:", start)

# ── 1. 그리퍼 완전 개방 ─────────────────────────────
print("\n▸ 그리퍼 개방 (stroke 750)")
gripper_cmd(STROKE_OPEN, current=50)
wait(1.5)
print("  ✓ 열림")

# ── 2. 픽 자세로 이동 (저속) ────────────────────────
print(f"\n▸ 픽 자세로 이동 (vel=20, acc=20)")
print("  목표:", PICK_POSE)
movej(posj(PICK_POSE), vel=20, acc=20)
wait(0.5)

# ── 3. 도착 확인 ───────────────────────────────────
now = get_current_posj()
err = [abs(a - b) for a, b in zip(now, PICK_POSE)]
print("\n도착 자세:", [round(v, 3) for v in now])
print("오차(deg):", [round(e, 3) for e in err])

if max(err) > 0.5:
    raise RuntimeError(f"목표 자세에 도달 못했습니다. 최대 오차 {max(err):.3f}°")

x, _ = get_current_posx()
print("TCP posx:", [round(v, 2) for v in x])
print("\n✓ 픽 자세 도착. 최대 오차 %.3f°" % max(err))

# 기록
out = Path("/tmp/doosan_ws_run/pick_pose.json")
out.write_text(json.dumps({
    "pose_deg": PICK_POSE,
    "reached_deg": [round(v, 4) for v in now],
    "posx": [round(v, 2) for v in x],
    "tool": tool, "tcp": tcp,
}, ensure_ascii=False, indent=2))
print("기록:", out)

rclpy.shutdown()
