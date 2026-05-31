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

코드에서도 같은 조건을 방어한다. `leader_odom_topic`에 `odom_aligned`가 포함되어 있으면, 설치본이나 런치 인자 실수로 `use_initial_odom_offset`이 `true`로 들어와도 노드가 시작할 때 자동으로 `false`로 낮춘다. 정상 실행 로그에는 다음처럼 실제 적용값이 찍혀야 한다.

```text
Follower platooning started: ... leader_odom=/leader/odom_aligned use_initial_odom_offset=false ...
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
- `ODOM_HOLD_STOPPED_LEADER`와 `cmd_x=0`이 계속 나오면 설치본 설정이 오래된 것이다. 실제 주행 설정은 `hold_when_leader_stopped: false`라서 리더 속도 토픽이 0이어도 odom 거리 오차가 있으면 계속 보정한다.

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

## 방향 튐 방지

기존 odom 제어는 팔로워에서 리더까지의 벡터를 `atan2()`로 계산한 뒤, 팔로워가 그 방향을 바라보도록 조향했다. 이 방식은 리더와 팔로워가 약간만 좌우로 어긋나도 팔로워가 리더를 향해 꺾어 들어가므로, 나란히 이동해야 하는 플래투닝에서 방향이 튀는 현상이 생긴다.

현재 제어는 리더-팔로워 벡터를 팔로워 기준 좌표로 변환해서 사용한다.

```text
forward_gap   = 팔로워 전방축 기준 리더와의 거리
lateral_error = 팔로워 좌우축 기준 리더와의 치우침
yaw_error     = 정렬된 리더 yaw - 팔로워 yaw
```

`forward_gap`은 전진 속도 제어에만 쓰고, `lateral_error`와 `yaw_error`는 각속도 제어에만 쓴다. 따라서 팔로워는 리더를 직접 바라보며 꺾지 않고, 리더와 같은 방향을 유지하면서 옆 오차를 줄인다. `FOLLOW`, `STANDBY`, `HANDOFF` 모두 거리 제어 허용 모드이며, 리더 `/leader/cmd_vel` feedforward도 이 세 모드에서 같이 적용된다.

각속도는 PID 형태로 계산한다.

```text
cmd_z = yaw_pid(yaw_error) + lateral_control_sign * lateral_pid(lateral_error)
```

`yaw_pid`는 리더와 같은 방향을 유지하는 항이고, `lateral_pid`는 리더와 나란한 라인으로 복귀하는 항이다. 실제 로봇에서는 적분항이 누적되면 오히려 방향이 튈 수 있어서 기본값은 `ki=0`으로 두고, 미분항으로 갑작스러운 치우침 변화만 감쇠한다.

실제 팔로워에서 오른쪽으로 튀는 증상이 반복되어 `lateral_control_sign` 기본값은 `-1.0`으로 둔다. 이 값은 좌우 보정 방향만 바꾸며, 거리 제어나 리더 yaw 정렬은 그대로 유지한다. 만약 반대로 왼쪽으로 튀면 이 값만 `1.0`으로 되돌려 확인한다.

관련 파라미터:

```yaml
kp_yaw: 0.45
kp_lateral: 0.35
lateral_control_sign: -1.0
ki_yaw: 0.0
kd_yaw: 0.04
ki_lateral: 0.0
kd_lateral: 0.0
use_leader_angular_feedforward: false
angular_slew_rate: 0.35
hold_when_leader_stopped: false
distance_deadband: 0.015
```

`use_leader_angular_feedforward`는 기본 `false`다. 직진 추종 중 `/leader/cmd_vel`의 angular 값을 그대로 더하면 방향 튐이 커질 수 있기 때문이다. 후진이나 제자리 회전 mirror 동작은 기존 `mirror_leader_reverse_turn` 경로를 유지한다.

`angular_slew_rate`는 `/follower/cmd_vel_raw`의 각속도 변화량 제한이다. 값이 너무 작으면 반응이 느리고, 너무 크면 방향 튐이 다시 커진다.

`hold_when_leader_stopped`는 기본 `false`다. 리더 속도 토픽이 늦게 오거나 0으로 들어와도 `/leader/odom_aligned` 거리 오차가 생기면 팔로워가 바로 보정하게 하기 위해서다.
