# follower_safety

팔로워 로봇의 최종 `/cmd_vel`을 발행하는 안전 필터 패키지다. 플래투닝 제어 명령은 `/follower/cmd_vel_raw`로 받고, 필터를 통과한 명령만 `/cmd_vel`로 내보낸다.

## 일반 추종 경로

```text
follower_platooning_node -> /follower/cmd_vel_raw -> follower_safety_node -> /cmd_vel
```

`follower_safety_node`는 heartbeat, 거리, 마커, 장애물 조건을 확인한 뒤 명령을 제한한다.

## 코드 구조

플래투닝 명령과 키보드 텔레옵 명령은 같은 `TimedTwistCommand` 구조체로 관리한다. 이 구조체는 마지막 명령, 수신 시각, 수신 여부를 함께 저장하고 timeout/active 판정을 담당한다. 따라서 안전 필터 본문은 heartbeat, 거리, 마커, 장애물 조건만 판단하도록 유지한다.

## 키보드 텔레옵 경로

팔로워 런치가 실행 중일 때 표준 `turtlebot3_teleop`을 그대로 `/cmd_vel`에 붙이면 `follower_safety_node`의 출력과 충돌한다. 따라서 키보드 텔레옵은 `/follower/teleop_cmd_vel`로 리맵해서 넣는다.

```bash
cd ~/Turtlebot3_Platooning
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/follower/teleop_cmd_vel
```

`/follower/teleop_cmd_vel`에 최근 명령이 들어오면 safety 노드는 이를 수동 조작으로 판단한다. 이때 heartbeat, 거리, 마커 게이트는 우회하고, 최종 속도 제한과 scan safety만 적용해서 `/cmd_vel`로 발행한다.

## 확인

```bash
ros2 topic echo /follower/teleop_cmd_vel --once
ros2 topic echo /follower/safety_state --once
ros2 topic echo /cmd_vel --once
```

키보드 입력 중 `/follower/safety_state`가 `TELEOP_SAFE` 또는 `TELEOP_STOPPED`로 나와야 한다.
