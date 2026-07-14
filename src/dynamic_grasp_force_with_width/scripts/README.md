# 무게 측정 스크립트 (2026-07-14 세션)

노트북 대신 **스크립트로** 돌리는 경로입니다. 각 파일이 독립 실행됩니다.

> ⚠️ 전부 `NEEDED` 서비스 디스커버리 대기 코드를 갖고 있습니다. **빼지 마세요.**
> 빼면 `spin_until_future_complete` 가 타임아웃 없이 **영원히 매달리고**,
> 매달린 프로세스가 컨트롤러를 물어서 `ros2 service call` 까지 먹통이 됩니다.
> 자세한 건 [../FINDINGS_85mm.md](../FINDINGS_85mm.md) 의 "운영 함정 B".

## 실행 전

```bash
./scripts/robot_up.sh                    # 워크스페이스 루트에서
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

전부 `/home/sooya/venv/ros/bin/python` 으로 돌리세요.

## 순서

```bash
P=/home/sooya/venv/ros/bin/python
D=src/dynamic_grasp_force_with_width/scripts

$P $D/setup_tool.py          # ① 툴 등록 (드라이버 재시작하면 날아갑니다. 매번 확인)
$P $D/goto_pick.py           # ② 그리퍼 개방 + 픽 자세로 이동
                             #    → 여기서 상자를 그리퍼에 넣으세요
$P $D/hold_test.py 350       # ③ (선택) 이 current 로 버티는지 60mm 들어서 확인
$P $D/measure_point.py 640 --regrip --settle 60 --cycles 10
                             # ④ 캘리브레이션 점 1개. 무게(g)를 인자로
$P $D/release.py             # ⑤ 그리퍼 개방 (상자 교체용. 매달려 있으면 받치세요!)
                             #    → ④⑤ 를 3~4번 반복
$P $D/fit.py                 # ⑥ 회귀 + R² + 잔차 + LOO-CV
```

## measure_point.py 옵션

| 옵션 | 뜻 |
|---|---|
| `--regrip` | 그리퍼를 다시 뭅니다. 상자를 교체했으면 필요 |
| `--settle N` | 파지 후 N초 대기 (기본 60). **15초 미만은 쓰지 마세요** — 과도구간입니다 |
| `--cycles N` | dither 횟수 (기본 5). 가벼운 물체는 흩어짐이 커서 **10** 을 권합니다 |
| `--no-save` | 저장 안 함. **held-out 검증점을 잴 때 쓰세요** (캘리브레이션에 섞이면 검증이 아닙니다) |
| `--force` | 물체 존재 확인을 무시. 가벼운 물체(<100 g)는 빈 그리퍼와 구분이 안 되므로 필요 |

## 그 외

- `grip.py` — 닫기만 하고 토크 확인 (dither 없음)
- `read_tau.py` — 아무것도 안 움직이고 토크만 읽음 (진단용)

## 하드코딩된 조건

`measure_point.py` 상단의 상수들입니다. **상자가 바뀌면 반드시 고치세요.**

```python
PICK_POSE   = [-13.181, -23.339, 94.612, 15.686, 55.968, -15.458]
STROKE_GRIP = 564      # 폭 85mm - squeeze 3mm
CURRENT     = 350
EMPTY_TAU   = -5.86    # 이 stroke/current 에서 물체 없이 닫았을 때
```
