# Patches for `unimelb_project/hardware_code`

Fixes that belong in the `hardware_code` repo but were found while debugging the
lab portal. They are kept here because **`hardware_code` on the server is a pull
target, not a working copy** — edits made there are destroyed by the next
`git pull` or `git clean` by anyone working on that box, and never make it back
to the real origin.

So: apply these in your **local** clone, commit, push, then `git pull` on the
server. Do not patch the server copy.

## 0001-relay-map-qos-split.patch

`ros2_ws/src/go2_hardware_autonomy/go2_hardware_autonomy/occupancy_grid_relay.py`

The relay built **one** `QoSProfile` and used it for both the publisher and the
subscription. That profile is `TRANSIENT_LOCAL`, so the relay's subscription to
`/map` requested `TRANSIENT_LOCAL` while the upstream SLAM publishes `/map` as
`VOLATILE`. ROS 2 treats that as incompatible and silently delivers nothing:

```
[occupancy_grid_relay_node]: New publisher discovered on topic '/map',
offering incompatible QoS. No messages will be received from it.
Last incompatible policy: DURABILITY
```

The relay then never republished `/luna/lidar_slam_2d_map`, so the explorer
never got a map.

The fix splits the profile: `VOLATILE` on the input (accepts either kind of
publisher) and `TRANSIENT_LOCAL` on the output (so the portal, which joins late,
still receives the current map).

Apply with:

```bash
cd <local clone>/hardware_code
git apply /path/to/0001-relay-map-qos-split.patch
colcon build --symlink-install --packages-select go2_hardware_autonomy
```

### Still latent after this patch

Three other nodes in the same launch have the identical bug and are **not**
fixed here, because each needs its own decision about which end to change:

| node | symptom in the launch log |
|---|---|
| `scan_matcher` | `New subscription discovered on topic '/map', requesting incompatible QoS` |
| `mapping_completion_manager` | `New publisher discovered on topic '/map', offering incompatible QoS` |
| `frontier_explorer` | `New publisher discovered on topic '/map', offering incompatible QoS` |

The portal map works regardless, because the portal subscribes to
`/luna/lidar_slam_2d_map` directly. But `frontier_explorer` not receiving `/map`
means genuine autonomous frontier selection is degraded — worth checking before
relying on the robot to actually explore.
