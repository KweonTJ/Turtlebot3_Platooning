# follower_platooning

팔로워 터틀봇의 리더 추종 제어 패키지다. 현재 실제 로봇에서는 비전 기반 추종이 아니라 리더/팔로워 odom을 이용해 IMU 간 거리를 맞춘다.

## 현재 추종 경로

실제 팔로워 런치에서는 다음 두 노드가 같이 실행된다.

```text
leader_odom_aligner_node
follower_platooning_node
```

`leader_odom_aligner_node`는 `/leader/odom`과 팔로워의 `/odom`을 받아서 `/leader/odom_aligned`를 발행한다. 이 토픽은 이미 팔로워 odom 좌표계 기준으로 변환되어 있고, 시작 시 리더 IMU가 팔로워 IMU보다 0.30 m 앞에 있다는 초기 오프셋도 포함한다.

```yaml
leader_odom_aligner:
  initial_leader_offset_x: 0.30
  aligned_leader_odom_topic: "/leader/odom_aligned"
```

따라서 `follower_platooning_node`가 `/leader/odom_aligned`를 사용할 때는 내부 초기 odom 오프셋을 다시 적용하면 안 된다.

```yaml
follower_platooning:
  leader_odom_topic: "/leader/odom_aligned"
  use_initial_odom_offset: false
```

이 값이 `true`이면 0.30 m 오프셋이 두 번 적용된다. 그러면 실제로는 리더가 멀어져도 컨트롤러가 목표 거리 근처라고 판단해서 `cmd_x=0`을 발행할 수 있다. 이때 팔로워는 리더 토픽을 받고 있어도 움직이지 않는다.

## 런타임 확인

팔로워가 움직이지 않을 때 팔로워 터틀봇에서 다음을 확인한다.

```bash
ros2 topic echo /follower/status --once
ros2 topic echo /follower/safety_state --once
ros2 topic echo /follower/cmd_vel_raw --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /leader/odom_aligned --once
ros2 topic echo /leader/follower_enable --once
ros2 topic echo /leader/platoon_mode --once
```

해석 기준:

- `/follower/status`가 `ODOM_FOLLOWING` 또는 `ODOM_REACQUIRE`이고 `cmd_x>0`이면 추종 제어는 움직이려고 한다.
- `/follower/cmd_vel_raw`는 움직이는데 `/cmd_vel`이 0이면 `follower_safety`가 막고 있는 것이다. 이때 `/follower/safety_state`를 본다.
- `WAITING_ENABLE`이면 `/leader/follower_enable` 또는 `/leader/platoon_mode`가 아직 안 왔거나 비활성 상태다.
- `ODOM_TIMEOUT`이면 `/leader/odom_aligned` 또는 `/odom`이 없거나 너무 오래된 상태다.
- `ODOM_HOLD_STOPPED_LEADER`와 `cmd_x=0`이 계속 나오면 리더 속도 토픽 `/leader/cmd_vel`이 0으로 들어오거나 거리 오차가 deadband 안에 있는 상태다.

## 주요 파라미터

```yaml
target_distance: 0.30
min_distance: 0.25
max_distance: 0.40
leader_odom_topic: "/leader/odom_aligned"
follower_odom_topic: "/odom"
use_initial_odom_offset: false
use_leader_linear_feedforward: true
far_catchup_use_max_speed: true
```

`target_distance`는 리더 IMU와 팔로워 IMU 사이 목표 거리다. 현재 실제 로봇 기준은 0.30 m다.
