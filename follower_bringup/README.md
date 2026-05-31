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
follower_camera
leader_odom_aligner_node
follower_platooning_node
follower_safety_node
robot_status_uploader.py
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
