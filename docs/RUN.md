# 설치 및 실행 가이드

두산 E0509 + RH-P12-RN 그리퍼를 처음부터 돌려보는 절차입니다.
처음 합류했다면 이 문서를 위에서부터 그대로 따라 하면 됩니다.

## 사전 준비

- Ubuntu 24.04 + **ROS2 Jazzy**
- 로봇: 두산 E0509 (**auto 모드 + STANDBY**), 그리퍼 RH-P12-RN(A) 플랜지 직결
- 로봇 컨트롤러 IP: **110.120.1.68** (PC 를 공유기에 **유선** 연결)

## 설치

외부 패키지(두산 공식 드라이버, 그리퍼 서비스)가 저장소에 **함께 들어 있습니다.**
따로 clone 하거나 패치할 필요 없이, 받아서 빌드만 하면 됩니다.

```bash
# 1. 받기 (ROS 관례상 워크스페이스 폴더명은 doosan_ws 로)
cd ~
git clone https://github.com/robot-e0509/box-sorter.git doosan_ws
cd doosan_ws

# 2. 의존 패키지 설치
rosdep install --from-paths src --ignore-src -r -y

# 3. 빌드 (5~10분 걸립니다)
colcon build --symlink-install
source install/setup.bash
```

> 저장소 용량이 약 400MB 입니다. 두산 드라이버(`doosan-robot2`)의 3D 메시 파일이 큽니다.
> clone 이 좀 오래 걸려도 정상입니다.

### 빌드할 때 빨간 글씨가 잔뜩 나오는데, 에러인가요?

**아닙니다.** 대부분 두산 코드에서 나오는 경고입니다. **맨 마지막 줄만** 보세요.

```
Summary: 31 packages finished       ← 성공
  7 packages had stderr output      ← 경고일 뿐, 무시
```

`Failed` 나 `Aborted` 가 없으면 성공입니다.
(colcon 은 경고도 에러도 똑같이 빨갛게 보여줍니다.)

### 저장소에 포함된 외부 패키지

| 패키지 | 출처 | 버전 |
|--------|------|------|
| `src/doosan-robot2` | [doosan-robotics/doosan-robot2](https://github.com/doosan-robotics/doosan-robot2) | jazzy · `816ecb5` |
| `src/dsr_study` | [pinklab-art/dsr_study](https://github.com/pinklab-art/dsr_study) | main · `61f45e0` |

`doosan-robot2` 의 `DSR_ROBOT2.py` 는 **패치가 적용된 상태**로 들어 있습니다.
두산 원본에는 서비스 접두사 누락(`_srv_name_prefix = ''`)과 클래스명 오타
(`SetSingularityHandlingForce`) 버그가 있어서, 그대로 쓰면 로봇 연결이 실패합니다.
`dsr_study` 의 수정본으로 덮어쓴 것이니 **다시 패치하지 마세요.**

## 실행 — 스크립트로 (권장)

터미널 하나에서 끝납니다. ping 확인 → 로봇 드라이버 → 그리퍼 서버를 순서대로 띄우고,
각 단계가 실제로 올라왔는지 확인한 뒤 다음으로 넘어갑니다. 실패하면 어디서 막혔는지 알려줍니다.

```bash
cd ~/doosan_ws

./scripts/robot_up.sh              # 로봇 + 그리퍼 서버 기동 (RViz 포함)
./scripts/robot_up.sh --no-rviz    # RViz 없이 (가볍고 빠름)

./scripts/gripper.sh open          # 그리퍼 열기
./scripts/gripper.sh close 300     # 닫기 (파지 힘 300)
./scripts/gripper.sh set 375 200   # 임의 위치(0~750) + 힘

./scripts/robot_down.sh            # 종료 ★ 쓰고 나면 반드시 내려주세요
```

> **끝내고 나면 꼭 `robot_down.sh` 를 실행하세요.** 드라이버가 떠 있는 채로 두면
> 다음 사람이 로봇을 못 씁니다 (컨트롤러 접속이 하나만 허용됩니다).

로그는 `/tmp/doosan_ws_run/` 에 남습니다.

```bash
tail -f /tmp/doosan_ws_run/bringup.log     # 로봇 드라이버
tail -f /tmp/doosan_ws_run/gripper.log     # 그리퍼 서버
```

### 파지 강도 실험

동적 강도 팀에 필요한 데이터(폭 / 무게 / 안 미끄러지는 최소 `current`)를 만드는 명령입니다.
`current` 를 낮은 값부터 올려가며, 매번 상자를 들어올려 버티는지 확인합니다.

```bash
./scripts/gripper.sh sweep 150 400 50    # current 150 → 400 까지 50 씩
```

버티기 시작한 값이 **그 상자의 최소 파지력**입니다. 그 값을 기록해서 공유하세요.

## 실행 — 손으로 (터미널 3개)

스크립트가 뭘 하는지 알고 싶거나, 단계를 나눠서 보고 싶을 때.
**모든 터미널에서 `source install/setup.bash` 를 먼저 해야 합니다.**
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
| 빌드 중 빨간 글씨가 잔뜩 나옴 | 대부분 경고입니다. 마지막 줄에 `Summary: N packages finished` 가 있으면 성공 → [설명](#빌드할-때-빨간-글씨가-잔뜩-나오는데-에러인가요) |
| `Package 'dsr_gripper' not found`<br>`ModuleNotFoundError: DR_init` | 그 터미널에서 `source install/setup.bash` 안 함. **터미널마다 매번** 해야 합니다 |
| 로봇 서버가 안 붙음 | `ping 110.120.1.68` 부터. 유선 연결·공유기 확인 |
| `set_robot_mode` → "service not available"<br>`NameError: SetSingularityHandlingForce` | `DSR_ROBOT2.py` 가 두산 원본으로 덮여씀. 저장소 버전으로 되돌리세요:<br>`git checkout src/doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py` |
| 그리퍼가 반응 없음 | `gripper_service` 가 안 떠 있음. 로봇이 **auto 모드 + STANDBY** 인지 확인 |
| 그리퍼가 이상하게 동작 | `gripper_service` 가 **두 개** 떠 있을 수 있음 → `ros2 node list \| grep gripper` |
| 상자가 미끄러짐 | `current` 를 올리세요 |
| 상자가 찌그러짐 | `current` 가 너무 큼 |

## 안전

**로봇은 한 명씩.** 두 명이 동시에 명령을 보내면 섞여서 위험합니다.
사용 전후로 팀 채팅에 남기고, 비상정지 버튼을 손 닿는 곳에 두세요.
