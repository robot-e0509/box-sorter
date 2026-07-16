"""weight_calib_box85.json 을 읽어 회귀 + R² + 잔차 + LOO-CV."""
import json
from pathlib import Path

import numpy as np

P = Path("/home/sooya/doosan_ws/src/dynamic_grasp_force_with_width/data/weight_calib_box85.json")
d = json.loads(P.read_text())
pts = sorted(d["points"], key=lambda p: p["mass_kg"])
n = len(pts)

x = np.array([p["tau"] for p in pts])
y = np.array([p["mass_kg"] * 1000 for p in pts])     # g

if n < 2:
    raise SystemExit(f"점이 {n}개뿐입니다")

A = np.vstack([x, np.ones_like(x)]).T
(a, b), *_ = np.linalg.lstsq(A, y, rcond=None)

pred = a * x + b
resid = pred - y
rmse = float(np.sqrt(np.mean(resid ** 2)))
r2 = 1 - np.sum(resid ** 2) / max(np.sum((y - y.mean()) ** 2), 1e-12)

print(f"  질량(g) = {a:.1f} · tau(J2) {b:+.1f}")
print(f"  {n}점 · R² {r2:.4f} · RMSE {rmse:.1f} g\n")
print(f"  {'실제':>7} {'tau':>9} {'예측':>8} {'잔차':>8} {'sem':>7}")
for p, pr, rs in zip(pts, pred, resid):
    print(f"  {p['mass_kg']*1000:>6.0f}g {p['tau']:>+9.3f} {pr:>7.0f}g {rs:>+7.0f}g {p['sem']:>7.3f}")

# 인접 점끼리의 국소 기울기 — 선형성이 깨지면 여기서 보인다
print(f"\n  구간별 기울기:")
for i in range(n - 1):
    m1, m2 = pts[i]["mass_kg"] * 1000, pts[i + 1]["mass_kg"] * 1000
    t1, t2 = pts[i]["tau"], pts[i + 1]["tau"]
    print(f"    {m1:>4.0f} → {m2:>4.0f} g   {(m2-m1)/(t2-t1):>8.1f} g/Nm")
print(f"    전체 회귀        {a:>8.1f} g/Nm")
print(f"    (참고: 75mm 상자 {-163.6:>7.1f} · 30mm 물병 {-214.2:>7.1f})")

if n >= 4:
    errs = []
    for i in range(n):
        k = [j for j in range(n) if j != i]
        Ai = np.vstack([x[k], np.ones(len(k))]).T
        (ai, bi), *_ = np.linalg.lstsq(Ai, y[k], rcond=None)
        e = abs(ai * x[i] + bi - y[i])
        errs.append(e)
        print(f"\n  LOO  {y[i]:>4.0f}g 를 빼고 적합 → 그 점 예측오차 {e:>5.0f} g")
    print(f"\n  LOO-CV 평균오차 {np.mean(errs):.1f} g")

print()
if r2 < 0.95:
    print("  ⚠️ R² < 0.95 — 못 씁니다. 미끄러졌거나 조건이 흔들렸습니다.")
elif rmse > 15:
    print(f"  ⚠️ R² 는 통과했지만 RMSE {rmse:.0f} g 가 큽니다.")
    print(f"     75mm 상자는 RMSE 6.2 g 였습니다. 점 하나가 튀는지 잔차를 보세요.")
else:
    print(f"  ✅ R² {r2:.4f} · RMSE {rmse:.1f} g")
