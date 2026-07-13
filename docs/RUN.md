# 설치 및 실행 가이드

두산 E0509 + RH-P12-RN 그리퍼를 처음부터 돌려보는 절차입니다.
처음 합류했다면 이 문서를 위에서부터 그대로 따라 하면 됩니다.

## 사전 준비

- Ubuntu 24.04 + **ROS2 Jazzy**
- 로봇: 두산 E0509 (**auto 모드 + STANDBY**), 그리퍼 RH-P12-RN(A) 플랜지 직결
- 로봇 컨트롤러 IP: **110.120.1.68** (PC 를 공유기에 **유선** 연결)

## 설치

`src/` 아래 두 패키지는 **외부 저장소**입니다. 우리 저장소에 커밋하지 않고 따로 받습니다.

```bash
# 1. 우리 저장소 (ROS 관례상 워크스페이스 폴더명은 doosan_ws 로 받습니다)
cd ~
git clone https://github.com/robot-e0509/box-sorter.git doosan_ws
cd doosan_ws/src

# 2. 외부 패키지 (두산 공식 드라이버 + 그리퍼 서비스)
git clone -b jazzy https://github.com/doosan-robotics/doosan-robot2.git
git clone https://github.com/pinklab-art/dsr_study.git

# 3. 의존 패키지 설치
cd ~/doosan_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. ★ DSR_ROBOT2.py 패치 (안 하면 로봇 연결이 실패합니다)
cp src/dsr_study/DSR_ROBOT2.py src/doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py

# 5. 빌드
colcon build --symlink-install
source install/setup.bash
```

> **4번을 빼먹으면** `set_robot_mode` 가 "service not available" 로 죽습니다.
> 두산 원본의 서비스 접두사 누락 + 클래스명 오타를 고친 파일입니다.
> 자세한 내용은 [dsr_study README](https://github.com/pinklab-art/dsr_study) 참고.

## 실행

빌드가 끝났다는 가정입니다. **모든 터미널에서 `source install/setup.bash` 를 먼저 해야 합니다.**
(안 하면 `Package 'dsr_gripper' not found`, `ModuleNotFoundError: DR_init` 이 납니다)

### 0. 연결 확인

PC 를 공유기에 **유선으로** 연결한 뒤:

```bash
ping 110.120.1.68
```

응답이 없으면 랜선·공유기부터 확인하세요. 여기서 막히면 아래는 전부 실패합니다.

### 1. 그리퍼 서버 (터미널 1)

```bash
cd ~/doosan_ws && source install/setup.bash
ros2 run dsr_gripper gripper_service
```

> 로봇이 **auto 모드 + STANDBY** 여야 동작합니다.

### 2. 로봇 서버 (터미널 2)

```bash
cd ~/doosan_ws && source install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.68 model:=e0509
```

### 3. 명령 보내기 (터미널 3)

```bash
cd ~/doosan_ws && source install/setup.bash

# 그리퍼 닫기 (힘 300)
ros2 service call /dsr01/gripper/cmd dsr_gripper_interfaces/srv/GripperCmd \
  "{position: 0, current: 300}"

# 그리퍼 열기
ros2 service call /dsr01/gripper/cmd dsr_gripper_interfaces/srv/GripperCmd \
  "{position: 750, current: 200}"
```

## 그리퍼 제어

파지 강도는 `current`(전류 제한)로 줍니다. **전류 기반 위치 제어**라서,
물체를 만나면 그 힘으로 잡고 멈춥니다. **이게 이 프로젝트의 핵심 손잡이입니다** —
고정 팀은 이 값을 상수로 박고, 동적 팀은 이 값을 계산해냅니다.

| 값 | 의미 |
|----|------|
| `position` | `0` = 완전 닫힘 ~ `750` = 완전 열림 |
| `current` | 파지 힘. 파지 시 200~400 권장. 종이상자는 그 이상에서 찌그러집니다 |

## 주피터 노트북

`notebooks/00_robot_connect.ipynb` 에 로봇 연결 + 현재 좌표 확인 예제가 있습니다.
로봇을 티칭해서 좌표를 얻을 때 (`get_current_posj()`) 여기서 합니다.

**반드시 `source` 한 터미널에서 띄우세요.** `DSR_ROBOT2` 는 폴더가 아니라 `install/` 에서
오기 때문에, source 를 빼먹으면 어느 폴더에서 열든 `ModuleNotFoundError: DR_init` 이 납니다.

```bash
cd ~/doosan_ws && source install/setup.bash
jupyter notebook          # ← 반드시 source 한 이 터미널에서
```

## 자주 겪는 문제

| 증상 | 원인 / 해결 |
|------|-------------|
| `Package 'dsr_gripper' not found`<br>`ModuleNotFoundError: DR_init` | 그 터미널에서 `source install/setup.bash` 안 함. **터미널마다 매번** 해야 합니다 |
| 로봇 서버가 안 붙음 | `ping 110.120.1.68` 부터. 유선 연결·공유기 확인 |
| `set_robot_mode` → "service not available" | DSR_ROBOT2.py 패치 안 함 → 설치 4번 |
| `NameError: SetSingularityHandlingForce` | 같은 원인 → 설치 4번 |
| 그리퍼가 반응 없음 | `gripper_service` 가 안 떠 있음. 로봇이 **auto 모드 + STANDBY** 인지 확인 |
| 그리퍼가 이상하게 동작 | `gripper_service` 가 **두 개** 떠 있을 수 있음 → `ros2 node list \| grep gripper` |
| 상자가 미끄러짐 | `current` 를 올리세요 |
| 상자가 찌그러짐 | `current` 가 너무 큼 |

## 안전

**로봇은 한 명씩.** 두 명이 동시에 명령을 보내면 섞여서 위험합니다.
사용 전후로 팀 채팅에 남기고, 비상정지 버튼을 손 닿는 곳에 두세요.
