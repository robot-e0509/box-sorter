# dsr_study — RH-P12-RN 그리퍼 ROS2 패키지

Doosan **E0509** 로봇팔 플랜지에 직결된 **ROBOTIS RH-P12-RN(A)** 그리퍼를
ROS2(Jazzy)에서 제어하는 패키지입니다.

플랜지 RS485(Modbus-RTU) 시리얼은 **컨트롤러만** 접근할 수 있어, 그리퍼 제어는
컨트롤러 DRL(`drl_start` 서비스)을 통해서만 가능합니다. 이 패키지는 그 과정을
ROS2 서비스 `/dsr01/gripper/cmd` 하나로 감싸줍니다.

## 구성
| 경로 | 내용 |
|--------|------|
| `DSR_ROBOT2.py` | doosan-robot2 라이브러리 **수정본** (아래 "DSR_ROBOT2.py 패치" 참고) |

| 패키지 | 내용 |
|--------|------|
| `dsr_gripper_interfaces` | 서비스 정의 `GripperCmd.srv` (ament_cmake) |
| `dsr_gripper` | 그리퍼 서비스 노드 + Python API (ament_python) |

## 사전 준비 (필수)
- ROS2 **Jazzy**
- **doosan-robot2** (jazzy) 가 같은 워크스페이스에 빌드돼 있어야 함
  → `dsr_msgs2`(DrlStart, SetOutputRegisterInt), `DR_init` 사용
- 로봇: **auto 모드 + STANDBY** 상태에서 실행
- 하드웨어: RH-P12-RN(A), 플랜지 RS485 57600bps, Modbus slave id 1

## 빌드
```bash
cd ~/doosan_ws/src
git clone <이 저장소 URL> dsr_study
cd ~/doosan_ws
colcon build --packages-select dsr_gripper_interfaces dsr_gripper
source install/setup.bash
```
> 인터페이스 패키지가 노드보다 먼저 빌드돼야 합니다.

## 실행
```bash
# 기본: 상주 루프 방식(고속). 첫 명령 때 DRL 루프 1회 기동 후 이후 즉시 반응
ros2 run dsr_gripper gripper_service

# (백업) 매 호출마다 open→move→close 하는 단순 방식
ros2 run dsr_gripper gripper_service_a
```
> 두 노드 모두 서비스명이 `/dsr01/gripper/cmd` 로 같습니다. **동시에 띄우지 마세요.**

## 서비스 API — `dsr_gripper_interfaces/srv/GripperCmd`
```
int32 position   # 0 = 완전 닫힘, 750 = 완전 열림
int32 current    # 힘(전류) 제한, 0이면 노드 기본값. 파지 200~400 권장
---
bool  success
```
```bash
# 닫기(힘 300) / 열기
ros2 service call /dsr01/gripper/cmd dsr_gripper_interfaces/srv/GripperCmd "{position: 0,   current: 300}"
ros2 service call /dsr01/gripper/cmd dsr_gripper_interfaces/srv/GripperCmd "{position: 750, current: 200}"
```

## Python API (Jupyter/스크립트)
`DSR_ROBOT2` 와 동일한 패턴 — rclpy 노드를 만들어 `DR_init.__dsr__node` 에 등록한 뒤:
```python
from dsr_gripper import gripper_open, gripper_close, gripper_cmd

gripper_open()                 # 열기
gripper_close(current=300)     # 닫기 / 물체 잡기
gripper_cmd(375, current=200)  # 임의 위치
```
(로봇 연결 셀에서 만든 `node` 를 자동으로 재사용합니다. `gripper_service` 노드가 실행 중이어야 합니다.)

## 참고
- 그리퍼 하드웨어 stroke가 0=열림/750=닫힘이라, 노드 내부에서 반전해 문서 규약
  (position 0=닫힘, 750=열림)에 맞춥니다.
- `current`(Goal Current) = 힘 제한. 전류기반 위치제어라 물체를 만나면 그 힘으로 잡고 멈춥니다.

## DSR_ROBOT2.py 패치 (doosan-robot2 버그 수정본)
`DSR_ROBOT2.py` 는 doosan-robot2(jazzy)의 동명 파일을 **수정한 것**입니다.
그리퍼 자체와는 무관하지만, 실기에서 겪은 두 가지 버그를 고쳐 함께 둡니다.

1. **srv 클래스명 오타** — 존재하지 않는 `SetSingularityHandlingForce` → 실제 이름 `SetSingularHandlingForce` (import 시 NameError)
2. **서비스 접두사 누락** — `_srv_name_prefix = ''` → `'dsr_controller2/'` (set_robot_mode "service not available")

적용: 원본 `~/doosan_ws/src/doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py` 에 덮어쓴 뒤 `colcon build` → `source install/setup.bash`.

> 이 파일은 Doosan Robotics의 Apache License 2.0 소스를 수정한 것이며, 원 저작권/라이선스 헤더는 그대로 유지합니다.

## 라이선스
- `dsr_gripper`, `dsr_gripper_interfaces` : Apache License 2.0 (본 저장소 저작물)
- `DSR_ROBOT2.py` : © Doosan Robotics, Apache License 2.0 (수정본)
