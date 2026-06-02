# follower_bringup

팔로워 터틀봇의 실제 실행 런치 패키지다. 기본 실행 파일은 `follower_system.launch.py`다.

## 실행

팔로워 터틀봇에서 실행한다.

```bash
cd ~/Turtlebot3_Platooning
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch follower_bringup follower_system.launch.py
```

기본 구성:

```text
turtlebot3_node
robot_state_publisher
leader_odom_aligner_node
follower_platooning_node
follower_safety_node
robot_status_uploader.py
```

현재 팔로워 로봇은 USB 카메라를 제거한 상태라서 기본 런치에서는 `follower_camera`를 켜지 않는다. 기본값은 다음과 같다.

```text
use_camera:=false
start_vision:=false
monitor_video_enabled:=false
```

## 리더 키보드 텔레옵 추종

리더가 pick/place 작업 런치가 아니라 키보드 텔레옵으로만 움직일 때는 팔로워에서 전용 별칭 런치를 사용한다.

```bash
cd ~/Turtlebot3_Platooning
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch follower_bringup teleop_follow.launch.py
```

이 런치는 `follower_system.launch.py`를 아래 값으로 감싼다.

```text
use_camera:=false
start_vision:=false
monitor_video_enabled:=false
start_base_driver:=true
start_platooning:=true
start_safety:=true
```

즉 USB 카메라 없이 기본 터틀봇 드라이버, odom aligner, platooning, safety만 켠다. 리더 쪽에서는 `leader_platooning_beacon leader_teleop_platooning.launch.py`가 `/leader/odom`, `/leader/cmd_vel`, `/leader/follower_enable`, `/leader/platoon_mode`, `/leader/heartbeat`를 팔로워 도메인으로 넘겨야 한다.

팔로워에서 먼저 확인할 토픽:

```bash
ros2 topic echo /leader/follower_enable --once
ros2 topic echo /leader/platoon_mode --once
ros2 topic echo /leader/heartbeat --once
ros2 topic echo /leader/odom --once
ros2 topic echo /leader/cmd_vel --once
ros2 topic echo /leader/odom_aligned --once
ros2 topic echo /follower/status --once
ros2 topic echo /follower/cmd_vel_raw --once
ros2 topic echo /follower/safety_state --once
ros2 topic echo /cmd_vel --once
```

`/leader/cmd_vel`이 들어오는데 `/follower/cmd_vel_raw`가 0이면 `follower_platooning`의 거리 판단이 멈춘 것이다. `/follower/cmd_vel_raw`는 움직이는데 `/cmd_vel`이 0이면 `follower_safety`가 막고 있는 것이다.

카메라를 다시 연결해서 팔로워 원본 영상을 모니터에 올려야 할 때만 명시적으로 켠다.

```bash
ros2 launch follower_bringup follower_system.launch.py \
  use_camera:=true \
  monitor_video_enabled:=true \
  video_device:=/dev/video0
```

## 추종 제어 오프셋

`leader_odom_aligner_node`가 `/leader/odom`을 팔로워 `/odom` 기준의 `/leader/odom_aligned`로 변환한다. 이 과정에서 초기 리더 IMU 오프셋 0.30 m가 이미 적용된다.

따라서 `follower_system.launch.py`는 `follower_platooning_node`에 다음 값을 명시적으로 넘긴다.

```yaml
use_initial_odom_offset: false
```

이 값을 다시 `true`로 켜면 0.30 m 오프셋이 두 번 적용되어 팔로워가 목표 거리에 있다고 판단하고 움직이지 않을 수 있다.

`follower_platooning_node` 코드에도 같은 방어가 들어가 있다. `leader_odom_topic`이 `/leader/odom_aligned`이면 런치나 설치본에서 `use_initial_odom_offset`이 잘못 `true`로 들어와도 시작 시 자동으로 비활성화한다. 정상 로그에는 `use_initial_odom_offset=false`가 보여야 한다.

## 확인 토픽

팔로워가 움직이지 않을 때:

```bash
ros2 topic echo /follower/status --once
ros2 topic echo /follower/safety_state --once
ros2 topic echo /follower/cmd_vel_raw --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /leader/odom_aligned --once
ros2 topic echo /leader/follower_enable --once
ros2 topic echo /leader/platoon_mode --once
```

`/follower/status`의 `cmd_x`가 0이면 `follower_platooning`에서 정지 판단을 한 것이다. `/follower/cmd_vel_raw`는 0이 아닌데 `/cmd_vel`이 0이면 `follower_safety`가 막고 있는 것이다.

## 키보드 텔레옵

기본 `turtlebot3_bringup`만 켠 상태에서는 safety 노드가 없으므로 표준 `/cmd_vel`을 그대로 사용한다. 이 경우에는 리맵을 넣지 않는다.

```bash
cd ~/Turtlebot3_Platooning
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```

기본 브링업 확인:

```bash
ros2 param get /turtlebot3_node enable_stamped_cmd_vel
ros2 topic info -v /cmd_vel
```

Humble 실제 로봇에서는 `enable_stamped_cmd_vel`이 `false`이고 `/cmd_vel` 타입이 `geometry_msgs/msg/Twist`여야 한다. 현재 패키지는 `ROS_DISTRO` 환경변수가 비어 있어도 Humble 기본값을 사용하도록 보강되어 있다.

팔로워 런치가 켜진 상태에서는 `follower_safety_node`가 최종 `/cmd_vel`을 발행한다. 표준 `turtlebot3_teleop`을 그대로 `/cmd_vel`에 붙이면 safety 출력과 충돌하므로, 수동 텔레옵 입력 토픽으로 리맵한다.

```bash
cd ~/Turtlebot3_Platooning
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/follower/teleop_cmd_vel
```

확인:

```bash
ros2 topic echo /follower/teleop_cmd_vel --once
ros2 topic echo /follower/safety_state --once
ros2 topic echo /cmd_vel --once
```

키 입력 중 `/follower/safety_state`가 `TELEOP_SAFE`로 나오면 safety를 통해 수동 명령이 최종 `/cmd_vel`로 전달되고 있는 상태다.

## 실제 런치 토픽 충돌 기준

기본 `follower_system.launch.py`에서는 `start_safety:=true`이므로 명령 경로는 하나로 고정된다.

```text
follower_platooning_node -> /follower/cmd_vel_raw
follower_safety_node     -> /cmd_vel
```

`start_safety:=false`로 실행할 때만 `follower_platooning_node`가 직접 `/cmd_vel`을 발행한다. 두 경로가 동시에 켜지면 `/cmd_vel` publisher가 중복되어 팔로워가 튀는 것처럼 보일 수 있으므로 실제 주행에서는 기본 safety 경로를 사용한다.

확인 명령:

```bash
ros2 topic info -v /cmd_vel
ros2 topic info -v /follower/cmd_vel_raw
ros2 topic echo /follower/status --once
ros2 topic echo /follower/safety_state --once
```
