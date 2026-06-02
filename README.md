# TurtleBot3 Platooning Follower Workspace

이 워크스페이스는 플래투닝 시스템의 **팔로워 로봇**에서 실행하는 패키지 모음이다. 현재 기본 구성은 ArUco 카메라 추적을 제외하고, 리더 odometry와 팔로워 odometry를 이용해 목표 차간 거리를 유지한다. `follower_platooning`이 원시 속도 명령을 만들고, `follower_safety`가 안전 조건을 확인한 뒤 최종 `/cmd_vel`을 발행한다.

현재 도메인 ID는 SSH 설정에 맞춘 실제 로봇 기준값이다.

| 구분 | ROS Domain ID | 역할 |
| --- | ---: | --- |
| 리더 로봇 | `25` | 물체 접근, 파지, 적재, 리더 상태 발행 |
| 팔로워 로봇 | `73` | odometry 기반 거리 제어, 안전 필터, 최종 주행 |
| 호스트 PC | `16` | 태블릿 모니터, 상태 브릿지, 실험 관찰 |

## 패키지 구성

| 패키지 | 역할 |
| --- | --- |
| `follower_bringup` | 팔로워 전체 실행 런치, 로봇 모델, RViz 옵션 |
| `follower_vision` | 선택 기능. 현재 기본 실행에서는 사용하지 않음 |
| `follower_platooning` | odometry 기반 목표 거리 유지 제어 및 `/follower/cmd_vel_raw` 발행 |
| `follower_safety` | 장애물, heartbeat, 속도 제한을 확인하고 최종 `/cmd_vel` 발행 |
| `platooning_bridge_config` | 리더 도메인 `25`의 상태 토픽을 팔로워 도메인 `73`으로 브릿지 |

기존 TurtleBot3, DynamixelSDK, manipulation, simulation 패키지는 upstream 의존성으로 유지한다. 플래투닝 전용 수정은 위 신규 패키지들 안에서 관리한다.

## 핵심 토픽

### 팔로워 내부 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 팔로워 로봇 odometry |
| `/follower/cmd_vel_raw` | `geometry_msgs/msg/Twist` | 플래투닝 제어기의 원시 속도 명령 |
| `/follower/status` | `std_msgs/msg/String` | 플래투닝 제어 상태 |
| `/follower/distance_error` | `std_msgs/msg/Float32` | 목표 거리 대비 오차 |
| `/follower/safety_state` | `std_msgs/msg/String` | 안전 필터 상태 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 팔로워 로봇 최종 주행 명령 |

`/cmd_vel`은 반드시 `follower_safety`만 발행한다. `follower_platooning`은 `/follower/cmd_vel_raw`까지만 발행한다.

### 리더에서 브릿지되는 토픽

| 토픽 | 설명 |
| --- | --- |
| `/leader/task_state` | 리더 작업 상태 |
| `/leader/cargo_state` | 물체 파지/적재 상태 |
| `/leader/follower_enable` | 팔로워 추종 허용 여부 |
| `/leader/platoon_mode` | `FOLLOW`, `STANDBY`, `STOP` 등 플래투닝 모드 |
| `/leader/heartbeat` | 리더 생존 신호 |
| `/leader/cmd_vel` | 리더 주행 명령 |
| `/leader/odom` | 브릿지로 들어온 리더 원본 odometry |
| `/leader/odom_aligned` | 팔로워 odom 좌표계로 정렬한 리더 odometry |

## 기본 파라미터

| 항목 | 기본값 |
| --- | ---: |
| 목표 차간 거리 | `0.47 m` |
| 최소 허용 거리 | `0.32 m` |
| 최대 유효 거리 | `0.70 m` |
| 긴급 정지 거리 | `0.20 m` |
| 최대 선속도 | `0.10 m/s` |
| 최대 각속도 | `0.45 rad/s` |
| odom timeout | `0.5 s` |
| heartbeat timeout | `1.0 s` |
| 초기 리더 오프셋 | `x=0.47 m`, `y=0.0 m` |

설정 파일:

```text
follower_platooning/config/platooning_params.yaml
follower_safety/config/safety_params.yaml
```

현재 실물 실험은 UWB 센서 없이 진행한다. 따라서 도메인 브릿지로 받은 리더
`/leader/odom`을 절대 좌표로 직접 비교하지 않고, `leader_odom_aligner`가 팔로워
odom 좌표계 기준 `/leader/odom_aligned`를 새로 발행한다. 노드 시작 시점에는
리더가 팔로워 전방 `0.47 m` 위치에 있다고 가정한다. 실제 배치는 두 로봇의
방향을 같게 맞추고, 리더 뒤쪽에 팔로워를 약 `47 cm` 간격으로 둔 뒤 팔로워
플래투닝 런치를 시작한다.

UWB가 추가되기 전까지는 초기 배치 오차가 곧 거리 제어 기준 오차가 된다. 실험
시작 전 줄자 등으로 47 cm 간격을 맞추고, 저속에서 `/follower/target_distance`와
`/follower/distance_error`를 먼저 확인한다.

## 설치 및 빌드

팔로워 로봇에 이 저장소를 받은 뒤 워크스페이스 루트에서 빌드한다.

```bash
cd ~/Desktop/Turtlebot3_Platooning

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

리더-팔로워 도메인 브릿지를 이 워크스페이스에서 실행하려면 `domain_bridge`도 필요하다.

```bash
sudo apt install ros-humble-domain-bridge
```

## 실행 방법

### 실행 런치 파일 요약

| 실행 목적 | 실행 위치 | 런치 파일 | 비고 |
| --- | --- | --- | --- |
| 팔로워 전체 실행 | 팔로워 로봇 | `follower_bringup follower_system.launch.py` | 기본 팔로워 bringup, odom aligner, platooning, safety 실행 |
| 리더 키보드 텔레옵 추종 | 팔로워 로봇 | `follower_bringup teleop_follow.launch.py` | 카메라 없이 리더 `/leader/odom`, `/leader/cmd_vel` 기반 추종만 실행 |
| 팔로워 플래투닝 제어만 실행 | 팔로워 로봇 또는 디버그 PC | `follower_platooning follower_platooning.launch.py` | base driver/safety 없이 odom 추종 제어 노드만 확인 |
| 팔로워 safety 필터만 실행 | 팔로워 로봇 또는 디버그 PC | `follower_safety follower_safety.launch.py` | `/follower/cmd_vel_raw`를 받아 최종 `/cmd_vel` 안전 필터링 |
| 리더-팔로워 도메인 브릿지 | 리더 또는 호스트 | `platooning_bridge_config bridge.launch.py` | 리더 domain `25`의 `/leader/*` 토픽을 팔로워 domain `73`으로 전달 |

리더가 키보드 텔레옵으로 움직일 때 팔로워에서 사용하는 기본 명령:

```bash
ros2 launch follower_bringup teleop_follow.launch.py
```

### 1. 팔로워 로봇 전체 실행

팔로워 로봇에서는 도메인 `73`으로 실행한다.

```bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=0

source /opt/ros/humble/setup.bash
source ~/Desktop/Turtlebot3_Platooning/install/setup.bash

ros2 launch follower_bringup follower_system.launch.py start_rviz:=false
```

현재 기본 실행은 `use_camera:=false`, `start_vision:=false`, `monitor_video_enabled:=false`이다. 팔로워 USB 카메라를 제거했기 때문에 기본 런치는 카메라 노드와 팔로워 영상 업로드를 켜지 않고, 리더 odometry 기반 거리 제어만 실행한다.

팔로워에 USB 카메라를 다시 연결해서 영상 확인이 필요할 때만 다음처럼 명시적으로 켠다.

```bash
ros2 launch follower_bringup follower_system.launch.py \
  use_camera:=true \
  monitor_video_enabled:=true \
  video_device:=/dev/video2 \
  start_rviz:=false
```

### 2. 팔로워 RViz 확인

팔로워 로봇 또는 같은 도메인의 PC에서 RViz를 같이 띄운다.

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/Desktop/Turtlebot3_Platooning/install/setup.bash

ros2 launch follower_bringup follower_system.launch.py start_rviz:=true
```

RViz fixed frame은 기본적으로 `base_footprint` 기준 모델 확인용이다. 리더/호스트와 함께 볼 때는 각 시스템의 TF/odom 구성을 맞춰서 확인한다.

### 3. 리더 토픽 브릿지 실행

리더 도메인 `25`의 상태를 팔로워 도메인 `73`으로 넘긴다. 브릿지는 네트워크상에서 두 도메인을 모두 볼 수 있는 PC에서 실행한다.

```bash
export ROS_LOCALHOST_ONLY=0

source /opt/ros/humble/setup.bash
source ~/Desktop/Turtlebot3_Platooning/install/setup.bash

ros2 launch platooning_bridge_config bridge.launch.py
```

브릿지 대상은 `platooning_bridge_config/config/leader_to_follower_bridge.yaml`에서 관리한다.

## 정상 동작 확인

팔로워 도메인 `73`에서 확인한다.

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/Desktop/Turtlebot3_Platooning/install/setup.bash

ros2 topic echo /odom --once
ros2 topic echo /follower/status --once
ros2 topic echo /follower/safety_state --once
ros2 topic echo /follower/target_distance --once
ros2 topic echo /follower/distance_error --once
ros2 topic echo /follower/cmd_vel_raw --once
ros2 topic echo /cmd_vel --once
```

리더 브릿지가 정상이라면 다음 토픽도 팔로워 도메인에서 보여야 한다.

```bash
ros2 topic echo /leader/heartbeat --once
ros2 topic echo /leader/platoon_mode --once
ros2 topic echo /leader/cmd_vel --once
ros2 topic echo /leader/odom --once
ros2 topic echo /leader/odom_aligned --once
```

## 운용 흐름

1. 리더 로봇이 `/leader/heartbeat`, `/leader/platoon_mode`, `/leader/follower_enable`, `/leader/cmd_vel`을 발행한다.
2. 브릿지가 리더 도메인 `25`에서 팔로워 도메인 `73`으로 상태 토픽을 전달한다.
3. `leader_odom_aligner`는 시작 시점의 수동 초기 정렬을 `0.47 m` 리더 오프셋으로
   잡고, 이후 브릿지된 `/leader/odom` 변화량을 팔로워 `/odom` 좌표계의
   `/leader/odom_aligned`로 변환한다.
4. `follower_platooning`은 `/leader/odom_aligned`와 자신의 `/odom`을 비교해 리더와의
   상대 거리를 추정한다.
5. `follower_platooning`이 목표 거리 `0.47 m`를 기준으로 `/follower/cmd_vel_raw`를 계산한다.
6. `follower_safety`가 heartbeat, 장애물, 속도 제한을 확인한 뒤 최종 `/cmd_vel`을 발행한다.

이 odometry 기반 구성은 UWB/공통 위치 추정 없이 실험하기 위한 임시 구성이다.
리더와 팔로워의 odom 원점이 서로 같다고 가정하지 않으며, 시작 시점의 물리적
47 cm 정렬을 기준으로 이후 이동 변화량만 사용한다. UWB가 탑재되면 이 초기
오프셋 가정 대신 실제 상대 거리 측정값으로 보정하는 구성이 필요하다.

## 안전 조건

`follower_safety`는 다음 조건에서 최종 `/cmd_vel`을 정지 또는 제한한다.

- 전방 장애물이 `0.12 m` 이내
- 측면 장애물이 `0.08 m` 이내
- 리더 heartbeat timeout
- 리더 또는 팔로워 odometry timeout
- 원시 속도 명령 timeout
- 최대 선속도/각속도 초과

후진 및 방향 전환 실험을 위해 현재 설정은 `allow_reverse: true`, `allow_untracked_reverse_or_turn: true`이다. 실제 로봇 적용 전에는 주변 공간을 확보하고 저속에서 먼저 확인한다.

## 실제 로봇 적용 순서

팔로워 로봇에서:

```bash
cd ~/Desktop/Turtlebot3_Platooning
git pull

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=0
ros2 launch follower_bringup follower_system.launch.py start_rviz:=false
```

리더/호스트와 함께 실행할 때는 네트워크가 같은지, `ROS_LOCALHOST_ONLY=0`인지, 각 장치의 domain ID가 현재 계획과 일치하는지 먼저 확인한다.

## GitHub 자동 업데이트

팔로워 로봇에서는 Git 저장소가 `src/` 자체이므로 `update_from_github.sh`도 `src/` 안에 둔다. 스크립트 내부에서 자동으로 워크스페이스 루트를 `~/Turtlebot3_Platooning`으로 계산한 뒤 `src`를 업데이트하고 빌드한다.

```bash
cd ~/Turtlebot3_Platooning
./src/update_from_github.sh
```

`src` 안에서 직접 실행해도 같은 방식으로 동작한다.

```bash
cd ~/Turtlebot3_Platooning/src
./update_from_github.sh
```

첫 실행 시 `src`가 Git 리포가 아니면 기존 `src`를 `src.backup.YYYYMMDD_HHMMSS`로 백업하고, GitHub main 브랜치를 새 `src`로 clone한다. 이후 실행부터는 `origin/main`을 fetch/reset/clean 한 뒤 빌드한다.

이 스크립트는 리더 로봇과 같은 방식으로 **받기 전용**이다. 터틀봇 내부에서 수정한 파일은 GitHub로 업로드하지 않고, 다음 실행 때 `origin/main` 기준으로 덮어쓴다. 또한 `origin`의 push URL을 비활성화해서 로봇에서 실수로 push하지 못하게 한다.

```text
GitHub repo: https://github.com/KweonTJ/Turtlebot3_Platooning.git
기본 branch: main
```

## Git 자동 커밋 및 push

개발 PC에서 팔로워 패키지를 수정하는 동안 변경사항을 바로 GitHub까지 올리려면 `src/git_auto.sh`를 실행한다. 이 스크립트는 `src` Git 저장소를 감시하고 파일 변경이 생기면 `git add -A`, 자동 커밋, `origin/main` push까지 수행한다.

```bash
cd ~/Desktop/Turtlebot3_Platooning/src
./git_auto.sh
```

기본 동작은 자동 push까지 포함한다. 로컬 커밋까지만 하고 싶으면 명시적으로 `AUTO_PUSH=0`을 붙인다.

```bash
cd ~/Desktop/Turtlebot3_Platooning/src
AUTO_PUSH=0 ./git_auto.sh
```

감시는 `inotifywait`가 있으면 이벤트 기반으로 동작하고, 없으면 5초마다 변경 여부를 확인한다. `.git`, `build`, `install`, `log`, `__pycache__`는 감시 대상에서 제외한다.

매번 쉘을 직접 실행하지 않고 자동으로 켜지게 하려면 user systemd 서비스로 등록한다.

```bash
cd ~/Desktop/Turtlebot3_Platooning/src
./git_auto.sh install
```

이후 로그인 세션에서 자동으로 `git_auto.sh watch`가 실행된다. 상태 확인과 해제는 다음 명령을 사용한다.

```bash
./git_auto.sh status
./git_auto.sh uninstall
```

서비스도 기본적으로 GitHub까지 자동 push한다. 로컬 커밋 전용 서비스로 설치하려면 `AUTO_PUSH=0`을 명시한다.

```bash
cd ~/Desktop/Turtlebot3_Platooning/src
AUTO_PUSH=0 ./git_auto.sh install
```

## 문제 확인

### odometry가 들어오지 않을 때

```bash
ros2 topic echo /odom --once
ros2 topic echo /leader/odom --once
ros2 topic echo /leader/odom_aligned --once
```

- 팔로워 로봇의 기본 odometry가 `/odom`으로 발행되는지 확인
- 리더 워크스페이스가 `/leader/odom`을 발행하는지 확인
- `platooning_bridge_config`가 리더 도메인 `25`에서 팔로워 도메인 `73`으로 `/leader/odom`을 브릿지하는지 확인
- `leader_odom_aligner`가 `/leader/odom_aligned`를 발행하는지 확인

### 팔로워가 움직이지 않을 때

```bash
ros2 topic echo /leader/heartbeat --once
ros2 topic echo /leader/follower_enable --once
ros2 topic echo /leader/platoon_mode --once
ros2 topic echo /follower/status --once
ros2 topic echo /follower/safety_state --once
```

- `/leader/platoon_mode`가 `FOLLOW`인지 확인
- `/leader/follower_enable`이 `true`인지 확인
- `/leader/odom`, `/leader/odom_aligned`, `/odom`이 모두 들어오는지 확인
- `/follower/safety_state`가 `SAFE` 또는 허용 상태인지 확인

### `/follower/cmd_vel_raw`는 나오는데 `/cmd_vel`이 안 나올 때

`follower_safety`가 막고 있는 상태다. `/follower/safety_state`, `/scan`, `/leader/heartbeat`를 우선 확인한다.

## 참고

- 이 워크스페이스는 팔로워 로봇 전용이다.
- 리더 로봇 작업 데모와 물체 파지/적재 로직은 리더 워크스페이스에서 관리한다.
- 태블릿 모니터와 호스트 브릿지는 호스트 워크스페이스에서 관리한다.
- 현재 domain ID는 실제 로봇 기준으로 리더 25, 팔로워 73, 호스트 16을 사용한다.
