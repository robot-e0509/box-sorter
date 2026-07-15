# v2 → v3 변경 정리 — 이동 안전성

**[`REVIEW.md`](REVIEW.md) 로부터 이동 안전성을 추가한 버전입니다.** 무게 측정·
파지력 계산 리뷰는 그쪽을 보시고, 여기는 그 이후(이동 중 안 떨어뜨리고 안
미끄러지기)만 다룹니다.

[`grasp_v3.ipynb`](grasp_v3.ipynb) 는 [`grasp_v2.ipynb`](grasp_v2.ipynb) 뒤에 **이동 안전성
테스트 셀(cell 27~31)만 이어붙인 것**입니다. **v2의 cell 0~26은 한 글자도 안
바꿨습니다** — 코드로 `nb["cells"][i]["source"] ==` 비교까지 해서 확인했습니다.

---

## 1. REVIEW.md 반영 현황

| REVIEW.md 항목 | 상태 | 비고 |
|---|---|---|
| 무게 측정 held-out 검증 | ✅ 완료 | `FINDINGS_85mm.md`, 실력 ±20g (in-sample ±6g는 과장이었음) |
| 파지력 회귀 (`current = A·질량 + B`) | ❌ 미착수 | `try_current()`/`accept()`/`reject()`/`planner.fit(trials)` — 로봇 필요, 오늘 못 함 |
| 실제로 다른 무게 상자 옮기기 (무게 기반 전체 흐름) | ❌ 미착수 | `weight_calib.json` 팀원분께 받아야 `pick_with_weight_feedback()` 실행 가능 |
| `TOOL_COG`/`TCP_POS` 실측 | ❌ 미착수 | 아직 근사값 (`[0,0,60]`, `[0,0,150,...]`) |
| **이동 안전성** (놓침 감지·release 확인·재시도) | 🆕 v3로 신규 대응 | REVIEW.md엔 없던 항목. v2가 픽·리프트까지만 다뤄서 이동 구간이 비어있었음 |

**정리: 오늘은 REVIEW.md의 원래 할 일(파지력 데이터, TCP 실측)은 진척이 없고, 대신 거기 없던 이동 안전성을 새로 만들었습니다.** 파지력 데이터 수집은 로봇 잡으면 여전히 1순위로 남아있습니다.

---

## 2. `TransportTest` — 무엇이고 어떻게 쓰나

`weight_calib.json`(무게 캘리브레이션) 없이도 고정 current로 동작하도록 만들어서, 팀원분 파일을 기다리지 않고 이동 로직만 먼저 검증할 수 있습니다.

```python
TransportTest(move='1')                      # 1단계: 근거리 단순 이동만
TransportTest(move='1', vel='measure')        # 1단계 + 이동 전후 토크를 로그에 기록
TransportTest(move='2')                       # 2단계: 체크포인트 이동 (놓침 감지 켜짐)
TransportTest(move='3', place_pos=[...])      # 3단계: 실좌표(OCR) 이동
TransportTest(move='3', place_pos='서울')      # 3단계: 놓기 구역 키워드로 이동
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `move` | `'1'` | `'1'` 근거리 단순 이동 / `'2'` 체크포인트 이동(놓침 감지) / `'3'` 실좌표 이동. 낮은 단계부터 순서대로 |
| `retry` | `'grip'` | 리프트 직후 아무것도 안 잡힌 것처럼 보이면 `CURRENT_STEP`만큼 세게 재시도 (`scale` 불필요, coarse 임계값 기준) |
| `vel` | `None` | `'measure'`면 `transport_test_log.json`에 폭·속도·토크 기록 (자동 판정 없음, `mark_slip()`으로 직접 라벨링) |
| `return_home` | `True` | place 끝나면 `PICK_POS`로 복귀 (새 좌표 아님, 이미 매 사이클 쓰는 검증된 위치) |
| `place_pos` | `None` | 6-vector 또는 `'수원'`/`'서울'` 문자열 |

**항상 실행되는 것** (안 쓰면 영향 없는 감지/보정 위주라 기본으로 켜둠):
- `release_confirmed()` — release 후 토크가 안 바뀌면 명령이 씹힌 것으로 보고 재전송 (실기에서 실제로 겪은 문제)
- `check_workspace()` — OCR팀 실측 작업공간·이동거리 제한 자동 검사 (`move='2'`/`'3'`)
- `DroppedError` 시 분기: `quick_torque_check()`로 완전히 놓친 건지(→`PICK_POS` 복귀) 뭔가 불안정하게 물려있는 건지(→ 제자리 정지) 구분

---

## 3. OCR팀 코드에서 확인한 것 — 좌표 인터페이스

실제 repo 로컬 클론(`doosan_ws~`, `origin/main`과 동기화됨)의 `yh/hand_eye_calib.py`
· `yh/click_and_move.py` · `yh/ocr_click_and_move.py` 를 확인했습니다.

```
위치: 로봇 base 기준 XYZ (mm), 카메라 hand-eye 캘리브레이션으로 산출
방향: 고정 하향 GRIPPER_DOWN_ORI_DEG = [0.0, 180.0, 0.0]
작업공간: WORKSPACE_MIN_MM=[-700,-700,50] ~ WORKSPACE_MAX_MM=[700,700,900]
한 번 이동 제한: MAX_STEP_MM = 400
놓기 구역(PLACE_ZONES): 수원 center(350,-500) / 서울 center(370,0), 각 300×300
```

**코드에 반영한 것**: 작업공간 범위·한 번 이동 제한·놓기 구역 X/Y (`check_workspace()`,
`PLACE_ZONE_XY`, `resolve_place_pos()`). 우리가 추측한 값이 아니라 OCR팀이 실제로
쓰는 값 그대로라 안전하다고 판단해 지금 넣었습니다.

**방향(하향 고정 `[0,180,0]`)은 선택 가능하게만 열어뒀지 기본으로 켜진 건 아닙니다.**
우리 `PICK_POS`는 기울어진 자세로 무게 캘리브레이션이 그 자세에 고정돼 있어서,
자세를 섞어 쓰면 캘리브레이션이 무효가 될 수 있습니다. 그래서 `resolve_place_pos()`는
**기본값은 안전한 `PICK_POS` 자세**를 쓰고, `ori='down'`을 명시적으로 넘길 때만
OCR팀의 `GRIPPER_DOWN_ORI_DEG`를 씁니다. **"픽업 자세를 OCR팀 방식(하향 고정)에
맞출지, 지금 방식을 유지할지"는 여전히 팀 논의가 필요합니다** — 코드는 양쪽 다
준비해뒀을 뿐, 어느 쪽을 표준으로 할지는 결정된 게 아닙니다.

### ⚠️ 팀에 확인해야 할 것 (저희가 임의로 못 고침)

- **"경기" 목적지가 코드에 없습니다.** README는 서울/수원/경기 3곳이라 했지만 `PLACE_ZONES`엔 수원/서울 2곳만 구현돼 있습니다.
- **수원 좌표 불일치.** 코드 주석은 `수원=(350,500)`인데 실제 `PLACE_ZONES` 값은 `cy=-500.0` — 부호가 반대입니다.
- **픽업 위치가 실제로 고정인지 가변인지.** OCR팀 코드(`click_and_move.py`)는 가변을 전제로 만들어져 있어서, 우리 쪽 무게 측정(`_check_cond`, 자세 0.5° 이상 벗어나면 예외)과 구조적으로 충돌할 수 있습니다.
- **박스 폭(width_mm)을 누가 알려주는지.** OCR팀 공식 범위(라벨 읽기)엔 없음.

---

## 4. 실측 필요 — 아직 다 추정치입니다

| 상수 | 지금 값 | 무엇을 실측해야 하나 | 어느 단계 |
|---|---|---|---|
| `DROP_THRESHOLD_NM` | 1.8 Nm | 정상 이동 오탐 없는 최소값 / 놓쳤을 때 못 잡지 않는 최대값 (마찰 밴드 ~1.5 Nm보다 커야 함) | `move='2'`/`'3'` |
| `QUICK_CHECK_SETTLE_S` | 0.4 s | 구간 도착 직후 동적 성분이 빠지는 시간 | `move='2'`/`'3'` |
| `TRANSIT_CHECKPOINTS` | 2 | 탐지 속도 vs 오버헤드 트레이드오프 | `move='2'`/`'3'` |
| `RELEASE_RETRIES`/`RELEASE_RETRY_WAIT_S` | 2회/0.5s | 재전송이 실제로 "명령 유실"을 고치는지 | 항상 |
| `CURRENT_STEP` | 50 | 재시도 증분 적절성, `CURRENT_MAX`와의 관계 | `retry='grip'` |
| 수평 이동 속도 (`SPEED_L=50` 고정) | — | place 방향(수평) 이동에서도 안전한지 — 지금까지 v2의 `try_current()`는 **수직** 리프트만 검증함 | `vel='measure'`로 기록 |

**속도/가속도 별도 스윕 실험은 불필요하다고 판단**: `VEL_MAX = SPEED_L`로 설계돼 있어
더 빠르게 갈 계획이 없는 한, 열린 질문은 "이미 쓰는 속도가 수평에서도 안전한가"
하나뿐이고 `vel='measure'` 데이터로 충분합니다.

---

## 5. 🐛 최종 점검에서 발견 (2026-07-15, 로봇 테스트 전)

- **버그(수정함)**: `_grip_and_lift()`의 `empty_tau` 기준선이 `PICK_POS` 도착 *전*에
  측정되고 있었습니다. J2 토크는 자세에 따라 달라지므로 기준선 자세가 안 맞을 수
  있었던 것 — `move_to(PICK_POS)` 뒤로 옮겨 수정했습니다.
- **⚠️ 한계(미해결, 공유용)**: `move_to_checked()`/`release_confirmed()`가 비교하는
  `held_tau`(PICK_POS 근처 기준선)엔 "물체 유무"뿐 아니라 **팔 자세(뻗은 정도) 차이도
  섞여 있습니다.** 짧은 이동(`move='1'`)은 무시할 만하지만, 긴 이동(`move='2'`/`'3'`,
  특히 수원·서울처럼 수백 mm 밖)에서는 자세 변화로 인한 토크 변화가
  `DROP_THRESHOLD_NM`을 넘어 **오탐(false DroppedError)을 만들 수 있습니다.**
  **완화 1**: `move_to_checked()`가 원점 대신 직전 체크포인트 대비 급변만 보도록
  바꿨습니다 (자세 drift는 완만하고 진짜 놓침은 급격하다는 전제 — 완전 해결은
  아니지만 오탐을 줄여줍니다). 체크포인트 통과마다 `print`로 tau·Δ를 찍어서
  실기에서 바로 patterns을 볼 수 있게 했습니다.
  **완화 2(진단용)**: `gravity_corrected_tau()` — 로봇이 자체 계산하는 중력 모델값
  (`raw_joint_torque`)을 `actual_motor_torque`에서 빼서 자세 성분을 줄이는 함수를
  추가했습니다. `quick_torque_check()`와 나란히 찍어서 어느 쪽이 자세 변화에 더
  안정적인지 실기에서 비교해볼 수 있습니다. **완전한 해결책은 아닙니다** —
  `TOOL_COG`가 아직 근사값이고, 이 프로젝트 핵심 노이즈원인 마찰 히스테리시스
  (~3 Nm)는 중력 모델이 애초에 반영하지 못합니다.

---

## 6. 안 만든 것 (의도적으로)

- **place 후 넘어짐 확인** — 원리적으로 불가능합니다. 물체를 놓는 순간 그리퍼와의
  물리적 연결이 끊겨서, 이후 넘어지든 안 넘어지든 로봇 쪽 센서(토크)에 신호가 없습니다.
  카메라 없이는 감지도 예방(놓는 높이 조절)도 못 합니다 — 비전/데이터 분류 파트 연결 필요.
- **`MAX_GRIP_RETRIES`(무게 기반 리프트 재시도)** — `scale.measure()`가 필요해서
  `weight_calib.json` 없인 못 씁니다. `TransportTest`는 coarse 임계값 재시도(`retry='grip'`)로
  우회했지만, 무게 기반 재시도 자체는 팀원분 파일 받은 뒤 v2의
  `pick_with_weight_feedback()`으로 따로 확인해야 합니다.

---

## 7. 테스트 순서

1. `TransportTest(move='1')` — 근거리 단순 이동 (1순위, 아직 안 함)
2. `TransportTest(move='2')` — 체크포인트 이동
3. 위 두 단계를 `vel='measure'`로 몇 번 더 돌려서 `transport_test_log.json`에
   수평 이동 데이터 쌓기 + `mark_slip()`으로 라벨링
4. OCR 팀 실좌표 들어오면 `TransportTest(move='3', place_pos=[x,y,z,rx,ry,rz])`,
   또는 `place_pos='서울'` (수원 좌표는 팀 확인 전까지 신뢰하지 말 것)
5. 팀원분에게 `weight_calib.json` 받으면 v2의 `pick_with_weight_feedback()`로
   `MAX_GRIP_RETRIES` 별도 확인
6. merge 끝나면 `doosan_ws~`에서 `git pull` 후 위 "팀에 확인해야 할 것" 항목 전달
7. **파지력 데이터 수집** (`try_current`/`accept`/`reject`/`planner.fit`) — REVIEW.md
   원래 할 일, 아직 미착수
