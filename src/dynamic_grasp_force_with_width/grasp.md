# Force Closure 기반 파지 근사방정식 정리

> 이 문서는 `physical_feedback_loop_v4_batch_camera.ipynb` 구현과 1:1로 맞춰서 정리했습니다.
> 담당 범위는 **동적 강도 파지**(폭 → 파지 강도(current) 계산)이며, 위치(어디를 잡을지)
> 관련 항목은 별도 섹션(6번)에 "구현됨 / 보류" 상태를 명시했습니다.
>
> **갱신**: 비전만으로는 물체 무게를 실측할 방법이 없다는 결론에 따라, 질량(M) 기반 힘 평형식(`physics` 모드)을 폐기했습니다. 이제 `naive`(LUT 실측값)와 `fitted`(실측 데이터로 회귀한 `F ≈ C·w²`) 두 모드만 남아 있습니다.

## 0. 프로젝트 목표와의 연결

| 단계 | 내용 | 이 문서/코드에서의 역할 |
|---|---|---|
| 1단계 | 고정 크기 상자, 파지 강도 하드코딩 | `try_force()`로 되는 숫자(폭/안 미끄러지는 최소 current)를 기록 |
| 2단계 | 크기 범위 실험, 강도 동적 계산으로 확장 | `GraspPlanner`의 `naive`→`fitted` 모드 확장이 이 단계에 대응 |

1단계에서 `accept_trial()`로 확정한 (폭, 힘) 쌍은 `grasp_lut.json`에 쌓이고, `fit_physics_constant()`가 이 데이터로 2단계의 예측 계수(`C`, `F ≈ C·w²`)를 역산합니다. **즉 1단계 실험값을 반드시 기록해야 2단계가 굴러갑니다.**

## 1. 핵심 변수 (Parameter Set)

| 기호 | 의미 | 코드 대응 | 상태 |
|---|---|---|---|
| `w` | 물체의 폭 (그리퍼가 벌어져야 할 거리) | `get_object_width()` | 구현됨 |
| `θ` | 윤곽선에서 추출한 법선 벡터의 각도 (접근 방향) | `get_approach_angle()` | **함수만 정의, 그립 자세에는 미반영** (아래 6-2 참고) |
| `μ` | 그리퍼 패드-물체 재질 간 마찰계수 (상수) | `GraspPlanner.mu` (기본 0.5) | 구현됨 |
| `F_grasp` | 그리퍼가 가하는 파지력 → 실제로는 그리퍼 **current(전류 제한)** 값으로 지령 | `target_force` → `gripper_close_fn(current=...)` | 구현됨 |
| `F_external` | 로봇 이동 중 발생하는 관성력 | `calculate_force(width, f_external=...)` | 인자로만 존재, 실측 소스는 미연결 |

~~`M`(물체 질량, 추정값)~~ — 비전만으로는 무게를 잴 방법이 없어 질량 추정 자체를 폐기했습니다. 대신 `fitted` 모드가 폭(`w`)에서 파지력을 직접 회귀합니다 (3절 참고).

**전류 기반 위치 제어 특성**: 그리퍼가 목표 지점까지 이동하다가 물체를 만나면 그 지점에서 지정한 current로 멈추는 방식입니다. `target_force`가 너무 낮으면 물체를 놓치고, 너무 높으면 찌그러뜨립니다. 이 문서와 코드가 다루는 "안전한 범위 찾기"가 정확히 이 문제입니다.

## 2. 힘 평형 조건 (미끄러짐 방지) — 폐기됨

```
2 · F_grasp · μ ≥ M·g + F_external
→ F_grasp ≥ (k · M · g) / (2 · μ)   (k: 안전계수, 기본 1.5)
```

질량 `M` 기반의 위 힘 평형식(`GraspPlanner._physics_force()`, `mode="physics"`)은 **더 이상 구현하지 않습니다.** 비전만으로는 재질(밀도)을 알 수 없어, 같은 겉넓이라도 실제 무게가 천차만별이라 `M` 항이 근본적으로 신뢰할 수 없는 추측이었기 때문입니다.

대신 `fitted` 모드가 이 식의 구조를 거치지 않고 실측 데이터로 `F ≈ C·w²` 관계를 직접 회귀합니다 (3절 참고). 실측 데이터가 아직 없으면 `GraspPlanner.default_force`(기본 20N)로 안전하게 대체됩니다.

## 3. 폭 기반 직접 회귀 (질량 근사는 폐기됨)

- **이전 구현**: `M ≈ area_ratio × w²`로 질량을 추정(`estimate_mass_area_based()`)한 뒤 `F = k·M·g / 2μ`로 힘을 계산했습니다. 하지만 질량은 재질(밀도)을 모르는 이상 겉넓이만으로는 추정할 수 없는 값이라, 이 중간 단계 자체를 제거했습니다.
- **현재 구현**: `fitted` 모드가 실측 (폭, 힘) 쌍으로 `F ≈ C·w²`의 계수 `C`를 직접 회귀합니다 (`fit_physics_constant()`). 질량이라는 중간 개념 없이, 실측된 폭-힘 관계를 그대로 사용합니다.
- 실측 데이터(`accept_trial()`된 것)가 2건 미만이면 `fitted_C`는 `None`이고, `calculate_force()`는 안전 기본값(`default_force`, 기본 20N)을 반환합니다.
- **3D 부피 기반(`M ≈ ρ·V`, 깊이맵 적분)**: 검토했으나 질량 접근 자체를 폐기하며 함께 폐기했습니다 (6-3 참고).

## 4. 기하학적 제약 (Force Closure 성립 조건)

- **Antipodal 조건**: `n1 + n2 ≈ 0` → `GraspPlanner.is_force_closure()`의 `antipodal`
- **마찰 원뿔 조건**: `α ≤ tan⁻¹(μ)` → 같은 함수의 `within_friction_cone`
- **안정성 점수** `S = 법선 평행도 + CoM 근접도` → `GraspPlanner.stability_score()`

이 셋을 만족하는 후보 중 `S`가 최대인 것을 `select_best_grasp()`가 선택합니다.

## 5. 힘 계산 2가지 모드와 실험 루프

`GraspPlanner.calculate_force(width, f_external=0.0, mode=...)`:

| 모드 | 동작 | 사용 시점 |
|---|---|---|
| `naive` | LUT(`grasp_lut.json`)에서 같은 폭(0.1mm 단위 반올림)을 찾아 그대로 반환, 없으면 안전 기본값(`default_force`, 20N) | 1단계 (하드코딩과 유사, 실측값 재생) |
| `fitted` | `accept_trial()`로 쌓인 실측 데이터를 `fit_physics_constant()`로 회귀한 `C`로 `F = C·w² + F_external` 계산. `fitted_C`가 없으면 `default_force`로 대체 | 2단계 (데이터가 쌓일수록 정확해짐) |

> `physics` 모드는 폐기되었습니다 (2, 3절 참고).

**실험 루프 사용법** (10절):
1. `try_force(width, force, approach_pos=PICK_POS)`를 여러 번 시도 (성공/실패 상관없이 `grasp_trials.json`에 전부 기록)
2. `review_trials()`로 표 확인
3. `accept_trial(idx)` / `reject_trial(idx)`로 판정 → accept된 것만 `grasp_lut.json`(LUT)에 반영
4. `fit_physics_constant()`로 `fitted_C` 갱신 → `mode="fitted"`에 즉시 반영

## 6. 위치(어디를 잡을지) — 별도 트랙, 상태 명시

담당 역할(동적 강도)과는 별개로, 노트북에는 위치 관련 코드도 포함되어 있습니다. **상자 위치가 고정 teach point인지 매번 달라지는지 아직 팀 내 결정 전**이므로, 상태를 명확히 구분해둡니다.

### 6-1. 핸드-아이 캘리브레이션 (구현됨)
- `pixel_depth_to_camera_point()`: 픽셀+depth → 카메라 3D 좌표(mm)
- `collect_calibration_point()` + `estimate_hand_eye_transform()`: 로봇을 직접교시로 지점에 갖다 대고(`get_current_posx()`) 동시에 카메라로 같은 지점을 클릭해 대응쌍 수집 → Kabsch(SVD)로 회전 R, 이동 t 추정. 3쌍 이상 필요, `hand_eye_calib.json`에 영구 저장.
- `camera_to_robot()`: 카메라 3D 좌표를 로봇 base 좌표로 변환. `RobotSystem.run_task()`가 접촉점 중점을 여기 통과시켜 `pick_pos`의 x,y,z에 반영 (자세는 `approach_pos` 값 유지).
- 캘리브레이션이 안 되어 있으면 `run_task()`는 `approach_pos`를 그대로 사용하도록 fallback 처리되어 있음 (안전장치).

### 6-2. 접근각 θ / Quaternion 기반 자세 (보류)
- `get_approach_angle()`은 정의만 되어 있고 실제 그립 자세에는 반영되지 않습니다. 현재는 `approach_pos`(티칭된 고정 자세)를 그대로 씁니다.
- Quaternion 기반 동적 접근방향 계산은 미구현입니다.
- **상자 위치가 고정으로 결정되면 이 항목은 사실상 불필요**해질 수 있어, "고정 강도 파지" 팀과 위치 결정 이후 재논의합니다.

### 6-3. 3D 부피 기반 질량 (폐기됨)
- 깊이맵을 적분해 부피 V를 구하고 `M ≈ ρ·V`로 질량을 추정하는 방식을 검토했으나, 비전으로는 재질(밀도)을 알 수 없어 결국 무게를 실측할 방법이 없다는 결론에 따라 질량 기반 접근 자체를 폐기했습니다 (3절 참고).
- 대신 `fitted` 모드가 폭 기반 실측 회귀(`F ≈ C·w²`)로 이 역할을 대신합니다.

## 7. 실행 흐름 (실제 구현 기준)

문서 이전 버전에는 "realsense2_camera(ROS2) 퍼블리시 → 별도 연산 노드 → doosan_robot2 서비스" 같은 분리된 ROS2 노드 구조로 적혀 있었지만, **실제 구현은 전부 하나의 Jupyter 노트북 안에서 함수 호출로 직접 이어집니다**:

```
get_frame() (RealSense)
   → RobotSystem.build_candidates()   -- 윤곽선, 폭, 접촉점(카메라 3D), 법선
   → GraspPlanner.select_best_grasp() -- force closure 필터 + 안정성 점수
   → GraspPlanner.calculate_force()   -- naive/fitted 중 선택한 모드로 목표 current 계산
   → camera_to_robot()                -- (캘리브레이션 있으면) 접촉점을 로봇 좌표로 변환
   → GraspController.execute_grasp()  -- APPROACH→DESCEND→GRASP→HOLD→LIFT 상태머신으로 실제 구동
```

## 8. 다음 단계 아이디어

- 상자 위치 고정/가변 여부 확정 (고정 강도 파지 팀과 조율) → 확정 후 6-2 항목 착수 여부 결정
- 물체 강성/변형 가능성 → '표면 적합도' 항을 `S`에 추가하는 안
- `S`(안정성 점수) 가중치를 우선순위에 따라 조정 (미끄럼 방지 우선 vs 무게중심 정렬 우선)
- `fitted` 모드의 회귀 정확도를 높이기 위해, `try_force()`/`accept_trial()` 실측 데이터를 다양한 폭 구간에 걸쳐 고르게 수집
