"""캘리브레이션 점 1개 측정.

    python measure_point.py <mass_g> [--regrip]

  --regrip : 내용물을 바꿔 다시 물어야 할 때 (그리퍼를 열었다 다시 문다)
             첫 점처럼 이미 물고 있으면 생략.

dither(J2 ±1°) × 5 회, 매번 같은 방향에서 복귀 → 독립 표본 → 평균.
표준오차가 크면 미끄러진 것이다.
"""
import json
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
import DR_init

args = [a for a in sys.argv[1:]]
REGRIP = "--regrip" in args
NO_SAVE = "--no-save" in args


def _opt(name, default):
    if name in args:
        return type(default)(args[args.index(name) + 1])
    return default


SETTLE_S = _opt("--settle", 60)
N_CYCLES = _opt("--cycles", 5)
mass_args = [a for a in args if not a.startswith("--")
             and not (args.index(a) > 0 and args[args.index(a) - 1] in ("--settle", "--cycles"))]
if not mass_args:
    raise SystemExit("사용법: measure_point.py <mass_g> [--regrip] [--settle N] [--no-save]")
MASS_G = float(mass_args[0])

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("measure_point", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import (
    movej, posj, wait,
    get_robot_mode, get_current_posj, get_tool, read_data_rt,
)
from dsr_gripper import gripper_cmd

NEEDED = ("get_robot_mode", "get_current_posj", "get_current_tool", "move_joint")
_need = [c for c in node.clients if any(k in c.srv_name for k in NEEDED)]
_t0 = time.time()
while time.time() - _t0 < 20.0:
    _pending = [c for c in _need if not c.service_is_ready()]
    if not _pending:
        break
    rclpy.spin_once(node, timeout_sec=0.1)
if _pending:
    raise RuntimeError("디스커버리 실패: " + ", ".join(c.srv_name for c in _pending))

# ── 조건 (REVIEW 1-5: 하나라도 바뀌면 재캘리브레이션) ──
PICK_POSE = [-13.181, -23.339, 94.612, 15.686, 55.968, -15.458]
WIDTH_MM = 85.0
STROKE_GRIP = 564          # (85-3)/109 * 750
STROKE_FREE = 585
CURRENT = 350

WEIGHT_JOINT = 1           # J2 (0-based). J3 는 드리프트로 못 씀
DITHER_DEG = 1.0
DITHER_VEL = 15
CYCLES = N_CYCLES
TORQUE_SAMPLES = 25

OUT = Path("/home/sooya/doosan_ws/src/dynamic_grasp_force_with_width/data/weight_calib_box85.json")


def motor_torque(n=TORQUE_SAMPLES):
    rows = []
    for _ in range(n):
        d = read_data_rt()
        if d is not None:
            rows.append(list(d.actual_motor_torque))
        time.sleep(0.04)
    if not rows:
        raise RuntimeError("read_data_rt 무응답")
    return [statistics.median(c) for c in zip(*rows)]


# ── 사전 검증 ──────────────────────────────────────
if get_robot_mode() != 1:
    raise RuntimeError("AUTONOMOUS 가 아닙니다")
tool = get_tool()
if tool != "rh_p12_rn":
    raise RuntimeError(f"툴이 다릅니다: {tool!r}")

q0 = [round(v, 3) for v in get_current_posj()]
dq = max(abs(a - b) for a, b in zip(q0, PICK_POSE))
if dq > 0.5:
    raise RuntimeError(f"픽 자세가 아닙니다 (오차 {dq:.2f}°)")

print(f"▸ 캘리브레이션 점: {MASS_G:.0f} g")
print(f"  조건  폭 {WIDTH_MM:.0f}mm · stroke {STROKE_GRIP} · current {CURRENT} · tool {tool}")
print(f"  자세  오차 {dq:.3f}°  OK")

if REGRIP:
    print(f"\n▸ 다시 물기 (stroke {STROKE_GRIP}, current {CURRENT})")
    gripper_cmd(STROKE_GRIP, current=CURRENT)
    wait(2.0)

# ── 크리프 안정 대기 ───────────────────────────────
# 2026-07-14 실측: 물자마자 tau -7.023 → 1분 뒤 -9.275 (팔은 안 움직임).
# 골판지가 current 350 에 눌리면서 짜는 반력이 수십 초에 걸쳐 변한다.
# 점마다 대기시간이 다르면 그게 그대로 계통오차가 되므로 고정한다.
if SETTLE_S > 0:
    print(f"\n▸ 크리프 안정 대기 {SETTLE_S}s (골판지가 눌려 자리잡을 때까지)")
    _t = time.time()
    while time.time() - _t < SETTLE_S:
        time.sleep(5)
        el = time.time() - _t
        print(f"    {el:>4.0f}s   tau(J2) = {motor_torque(n=5)[WEIGHT_JOINT]:+8.3f} Nm")

# ── 물체 존재 확인 ─────────────────────────────────
# 2026-07-14 실측: stroke 564 / current 350 에서 '물체 없이' 닫으면 tau(J2) = -5.86 Nm.
# 물체가 있으면 무게 + 짜는 반력 때문에 확실히 더 음수가 된다.
# 이 검사가 없으면 빈 그리퍼를 1분간 dither 하고 나서야 쓰레기 데이터임을 알게 된다.
EMPTY_TAU = -5.86
PRESENCE_MARGIN = 0.6

t_now = motor_torque(n=15)[WEIGHT_JOINT]
print(f"\n▸ 물체 확인   tau(J2) = {t_now:+.3f} Nm   (빈 그리퍼 기준 {EMPTY_TAU:+.2f})")
if t_now > EMPTY_TAU - PRESENCE_MARGIN and "--force" not in args:
    raise SystemExit(
        f"\n  ❌ 중단 — 그리퍼에 물체가 없는 것 같습니다.\n"
        f"     tau {t_now:+.3f} 가 빈 그리퍼({EMPTY_TAU:+.2f})와 거의 같습니다.\n"
        f"     상자를 그리퍼에 넣고 다시 실행하세요.\n"
        f"     (판단이 틀렸다고 확신하면 --force 로 무시할 수 있습니다)")
print(f"  ✓ 물체 있음")

# ── dither + 평균 ──────────────────────────────────
print(f"\n▸ dither(J2 ±{DITHER_DEG}°) × {CYCLES}회")
vals = []
for i in range(CYCLES):
    away = list(q0)
    away[WEIGHT_JOINT] += DITHER_DEG
    movej(posj(away), vel=DITHER_VEL, acc=DITHER_VEL)
    wait(0.4)
    movej(posj(q0), vel=DITHER_VEL, acc=DITHER_VEL)   # 항상 같은 방향에서 복귀
    wait(1.2)
    t = motor_torque()
    vals.append(t[WEIGHT_JOINT])
    print(f"    {i+1}/{CYCLES}   tau(J2) = {t[WEIGHT_JOINT]:+8.3f} Nm")

mean = statistics.mean(vals)
sd = statistics.pstdev(vals)
sem = sd / len(vals) ** 0.5

print(f"\n  평균 tau(J2) = {mean:+.3f} Nm")
print(f"  표준편차     = {sd:.3f} Nm")
print(f"  표준오차     = {sem:.3f} Nm     (기존 75mm 상자: 0.024 ~ 0.060)")

if sem > 0.15:
    print(f"\n  ⚠️ 표준오차가 큽니다. 미끄러졌을 가능성이 있습니다.")
    print(f"     상자가 손가락 안에서 흘러내렸는지 확인하세요.")

# ── 저장 ───────────────────────────────────────────
if NO_SAVE:
    print("\n  (--no-save : 저장하지 않았습니다)")
    rclpy.shutdown()
    raise SystemExit(0)

OUT.parent.mkdir(parents=True, exist_ok=True)
if OUT.exists():
    doc = json.loads(OUT.read_text())
else:
    doc = {
        "measured_at": datetime.now().strftime("%Y-%m-%d"),
        "object": "새 골판지 상자 (폭 85mm)",
        "conditions": {
            "pose_deg": PICK_POSE,
            "tool": {"name": "rh_p12_rn", "weight_kg": 0.5, "cog_mm": [0, 0, 60]},
            "gripper": {"stroke_cmd": STROKE_GRIP, "current": CURRENT,
                        "object_width_mm": WIDTH_MM},
            "method": f"dither(J2 ±{DITHER_DEG}°) + {CYCLES}회 평균, 절대 J2 모터토크",
        },
        "points": [],
    }

doc["points"] = [p for p in doc["points"] if abs(p["mass_kg"] * 1000 - MASS_G) > 0.5]
doc["points"].append({
    "mass_kg": MASS_G / 1000.0,
    "tau": round(mean, 4),
    "sd": round(sd, 4),
    "sem": round(sem, 4),
    "samples": [round(v, 4) for v in vals],
    "t": datetime.now().isoformat(timespec="seconds"),
})
doc["points"].sort(key=lambda p: p["mass_kg"])
OUT.write_text(json.dumps(doc, ensure_ascii=False, indent=2))

print(f"\n  저장: {OUT.name}   (총 {len(doc['points'])}점)")
for p in doc["points"]:
    print(f"    {p['mass_kg']*1000:>6.0f} g  →  tau {p['tau']:+8.3f}  (sem {p['sem']:.3f})")

rclpy.shutdown()
