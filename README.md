# TurtleBot3 Platooning Follower Workspace

이 워크스페이스는 플래투닝 시스템의 **팔로워 로봇**에서 실행하는 패키지 모음이다. 현재 기본 구성은 ArUco 카메라 추적을 제외하고, 리더 odometry와 팔로워 odometry를 이용해 목표 차간 거리를 유지한다. `follower_platooning`이 원시 속도 명령을 만들고, `follower_safety`가 안전 조건을 확인한 뒤 최종 `/cmd_vel`을 발행한다.

현재 도메인 ID는 실험용 임시값이다.

| 구분 | ROS Domain ID | 역할 |
| --- | ---: | --- |
| 리더 로봇 | `10` | 물체 접근, 파지, 적재, 리더 상태 발행 |
| 팔로워 로봇 | `20` | odometry 기반 거리 제어, 안전 필터, 최종 주행 |
| 호스트 PC | `16` | 태블릿 모니터, 상태 브릿지, 실험 관찰 |

## 패키지 구성

| 패키지 | 역할 |
| --- | --- |
| `follower_bringup` | 팔로워 전체 실행 런치, 로봇 모델, RViz 옵션 |
| `follower_vision` | 선택 기능. 현재 기본 실행에서는 사용하지 않음 |
| `follower_platooning` | odometry 기반 목표 거리 유지 제어 및 `/follower/cmd_vel_raw` 발행 |
| `follower_safety` | 장애물, heartbeat, 속도 제한을 확인하고 최종 `/cmd_vel` 발행 |
| `platooning_bridge_config` | 리더 도메인 `10`의 상태 토픽을 팔로워 도메인 `20`으로 브릿지 |

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
| `/leader/odom` | 리더 odometry |

## 기본 파라미터

| 항목 | 기본값 |
| --- | ---: |
| 목표 차간 거리 | `0.45 m` |
| 최소 허용 거리 | `0.32 m` |
| 최대 유효 거리 | `0.70 m` |
| 긴급 정지 거리 | `0.20 m` |
| 최대 선속도 | `0.10 m/s` |
| 최대 각속도 | `0.45 rad/s` |
| odom timeout | `0.5 s` |
| heartbeat timeout | `1.0 s` |

설정 파일:

```text
follower_platooning/config/platooning_params.yaml
follower_safety/config/safety_params.yaml
```

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

### 1. 팔로워 로봇 전체 실행

팔로워 로봇에서는 도메인 `20`으로 실행한다.

```bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0

source /opt/ros/humble/setup.bash
source ~/Desktop/Turtlebot3_Platooning/install/setup.bash

ros2 launch follower_bringup follower_system.launch.py start_rviz:=false
```

현재 기본 실행은 `use_camera:=false`, `start_vision:=false`이다. ArUco 없이 odometry 기반 제어만 실행한다.

선택적으로 카메라/vision 노드를 다시 켜야 할 때는 다음처럼 실행한다.

```bash
ros2 launch follower_bringup follower_system.launch.py \
  use_camera:=true \
  start_vision:=true \
  video_device:=/dev/video2 \
  start_rviz:=false
```

### 2. 팔로워 RViz 확인

팔로워 로봇 또는 같은 도메인의 PC에서 RViz를 같이 띄운다.

```bash
export ROS_DOMAIN_ID=20
source /opt/ros/humble/setup.bash
source ~/Desktop/Turtlebot3_Platooning/install/setup.bash

ros2 launch follower_bringup follower_system.launch.py start_rviz:=true
```

RViz fixed frame은 기본적으로 `base_footprint` 기준 모델 확인용이다. 리더/호스트와 함께 볼 때는 각 시스템의 TF/odom 구성을 맞춰서 확인한다.

### 3. 리더 토픽 브릿지 실행

리더 도메인 `10`의 상태를 팔로워 도메인 `20`으로 넘긴다. 브릿지는 네트워크상에서 두 도메인을 모두 볼 수 있는 PC에서 실행한다.

```bash
export ROS_LOCALHOST_ONLY=0

source /opt/ros/humble/setup.bash
source ~/Desktop/Turtlebot3_Platooning/install/setup.bash

ros2 launch platooning_bridge_config bridge.launch.py
```

브릿지 대상은 `platooning_bridge_config/config/leader_to_follower_bridge.yaml`에서 관리한다.

## 정상 동작 확인

팔로워 도메인 `20`에서 확인한다.

```bash
export ROS_DOMAIN_ID=20
source /opt/ros/humble/setup.bash
source ~/Desktop/Turtlebot3_Platooning/install/setup.bash

ros2 topic echo /odom --once
ros2 topic echo /follower/status --once
ros2 topic echo /follower/safety_state --once
ros2 topic echo /follower/cmd_vel_raw --once
ros2 topic echo /cmd_vel --once
```

리더 브릿지가 정상이라면 다음 토픽도 팔로워 도메인에서 보여야 한다.

```bash
ros2 topic echo /leader/heartbeat --once
ros2 topic echo /leader/platoon_mode --once
ros2 topic echo /leader/cmd_vel --once
ros2 topic echo /leader/odom --once
```

## 운용 흐름

1. 리더 로봇이 `/leader/heartbeat`, `/leader/platoon_mode`, `/leader/follower_enable`, `/leader/cmd_vel`을 발행한다.
2. 브릿지가 리더 도메인 `10`에서 팔로워 도메인 `20`으로 상태 토픽을 전달한다.
3. 팔로워 로봇은 `/leader/odom`과 자신의 `/odom`을 비교해 리더와의 상대 거리를 계산한다.
4. `follower_platooning`이 목표 거리 `0.45 m`를 기준으로 `/follower/cmd_vel_raw`를 계산한다.
5. `follower_safety`가 heartbeat, 장애물, 속도 제한을 확인한 뒤 최종 `/cmd_vel`을 발행한다.

이 odometry 기반 구성은 리더와 팔로워 odometry가 같은 기준으로 해석될 수 있다는 전제가 있다. 실제 로봇에서는 초기 정렬과 좌표 기준을 맞춘 뒤 저속으로 먼저 검증한다.

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

export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
ros2 launch follower_bringup follower_system.launch.py start_rviz:=false
```

리더/호스트와 함께 실행할 때는 네트워크가 같은지, `ROS_LOCALHOST_ONLY=0`인지, 각 장치의 domain ID가 현재 계획과 일치하는지 먼저 확인한다.

## GitHub 자동 업데이트

팔로워 로봇에는 워크스페이스 루트에 `update_from_github.sh`를 두고 실행한다.

```bash
cd ~/Turtlebot3_Platooning
./update_from_github.sh
```

첫 실행 시 `src`가 Git 리포가 아니면 기존 `src`를 `src.backup.YYYYMMDD_HHMMSS`로 백업하고, GitHub main 브랜치를 새 `src`로 clone한다. 이후 실행부터는 `origin/main`을 fetch/reset/clean 한 뒤 빌드한다.

```text
GitHub repo: https://github.com/KweonTJ/Turtlebot3_Platooning.git
기본 branch: main
```

## 문제 확인

### odometry가 들어오지 않을 때

```bash
ros2 topic echo /odom --once
ros2 topic echo /leader/odom --once
```

- 팔로워 로봇의 기본 odometry가 `/odom`으로 발행되는지 확인
- 리더 워크스페이스가 `/leader/odom`을 발행하는지 확인
- `platooning_bridge_config`가 리더 도메인 `10`에서 팔로워 도메인 `20`으로 `/leader/odom`을 브릿지하는지 확인

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
- `/leader/odom`과 `/odom`이 모두 들어오는지 확인
- `/follower/safety_state`가 `SAFE` 또는 허용 상태인지 확인

### `/follower/cmd_vel_raw`는 나오는데 `/cmd_vel`이 안 나올 때

`follower_safety`가 막고 있는 상태다. `/follower/safety_state`, `/scan`, `/leader/heartbeat`를 우선 확인한다.

## 참고

- 이 워크스페이스는 팔로워 로봇 전용이다.
- 리더 로봇 작업 데모와 물체 파지/적재 로직은 리더 워크스페이스에서 관리한다.
- 태블릿 모니터와 호스트 브릿지는 호스트 워크스페이스에서 관리한다.
- 현재 domain ID는 임시값이며 실제 운용 환경에 맞춰 수정할 예정이다.
