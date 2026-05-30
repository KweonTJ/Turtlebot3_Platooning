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
