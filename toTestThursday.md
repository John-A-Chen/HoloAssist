# To Test Thursday

## Branch: `nic-cubes` (from clean `nic`)

### What was added

| File | Purpose |
|---|---|
| `cube_pose_relay.py` | ROS node — transforms cube poses from `workspace_frame` → `base_link`, republishes to `/holoassist/unity/cube_{1-4}_pose` |
| `CubePoseSubscriber.cs` | Unity script — subscribes to relayed cube poses, spawns coloured cubes (red/green/blue/orange) at correct positions relative to robot base |
| `launch.py` | Added `--perception` flag — starts sim perception + static TF + cube relay on top of existing teleop stack |

No mode switching, no auto-sort, no MoveIt. Just cubes in Unity.

### ROS side

```bash
# Fake hardware + perception sim
./launch.sh --perception

# Real robot + real camera
./launch.sh --perception --robot-ip 192.168.0.194
```

### Unity Editor setup (one-time)

1. Create empty GameObject → name it `CubePoseManager`
2. Attach `Assets/Scripts/CubePoseSubscriber.cs`
3. Set **Robot Base** field to the `base_link` transform inside `ur3e_rg2` hierarchy
4. Leave prefabs empty for auto-generated coloured cubes (or assign Seb's prefabs later)
5. Hit Play

### Verify

```bash
# Check sim cubes are publishing
ros2 topic echo /holoassist/sim/perception/april_cube_1_pose --once

# Check relay is transforming to base_link
ros2 topic echo /holoassist/unity/cube_1_pose --once

# Randomise cube positions
ros2 service call /holoassist/sim/randomise_april_cubes std_srvs/srv/Trigger "{}"
```

### What to look for

- Virtual cubes appear in Unity at the same position as the sim cubes in RViz
- Moving sim cubes (randomise service) updates Unity positions
- Cubes disappear after 3s if perception stops publishing
- Cubes are positioned correctly relative to robot base (should be in front of / on the workspace table)
