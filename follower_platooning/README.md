# follower_platooning

Follower-side distance controller for the TurtleBot3 platooning robot.

## Odom Follow Path

The current real follower launch starts both nodes from
`launch/follower_platooning.launch.py`:

```text
leader_odom_aligner_node
follower_platooning_node
```

`leader_odom_aligner_node` subscribes to `/leader/odom` and `/odom`, then
publishes `/leader/odom_aligned`. That aligned odometry is already expressed in
the follower odom frame and already includes the configured initial leader IMU
offset:

```yaml
leader_odom_aligner:
  initial_leader_offset_x: 0.30
  aligned_leader_odom_topic: "/leader/odom_aligned"
```

For that reason `follower_platooning` must not apply its own initial odom offset
when it consumes `/leader/odom_aligned`:

```yaml
follower_platooning:
  leader_odom_topic: "/leader/odom_aligned"
  use_initial_odom_offset: false
```

If both offsets are enabled, the 0.30 m leader gap is applied twice. The
controller can then report a near-zero distance error and publish `cmd_x=0`, so
the follower appears to ignore the leader even though odom messages are present.

## Runtime Checks

Use these topics on the follower when it does not move:

```bash
ros2 topic echo /follower/status --once
ros2 topic echo /follower/safety_state --once
ros2 topic echo /follower/cmd_vel_raw --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /leader/odom_aligned --once
ros2 topic echo /leader/follower_enable --once
ros2 topic echo /leader/platoon_mode --once
```

Interpretation:

- `/follower/status` `ODOM_FOLLOWING` or `ODOM_REACQUIRE` with `cmd_x>0` means
  the controller is trying to move.
- `/follower/cmd_vel_raw` nonzero but `/cmd_vel` zero means `follower_safety` is
  blocking motion; inspect `/follower/safety_state`.
- `WAITING_ENABLE` means `/leader/follower_enable` or `/leader/platoon_mode` is
  missing or disabled.
- `ODOM_TIMEOUT` means `/leader/odom_aligned` or `/odom` is missing or stale.
