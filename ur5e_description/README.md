# UR5e Description for Drake

## Files

| File | Description |
|------|-------------|
| `ur5e.urdf` | UR5e robot (6-DOF, reusable) |
| `netft_probe.urdf` | NetFT sensor + P42V probe |
| `ur5e_netft_probe.dmd.yaml` | Model directives (robot + end-effector) |

## Usage

```python
from pydrake.all import MultibodyPlant, Parser

plant = MultibodyPlant(time_step=0.001)
parser = Parser(plant)
parser.package_map().Add('ur5e_description', '/path/to/ur5e_description')

# Option 1: Robot + NetFT probe
parser.AddModels('ur5e_netft_probe.dmd.yaml')

# Option 2: Robot only (for other end-effectors)
parser.AddModels('ur5e.urdf')
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))

plant.Finalize()
```

## Kinematic Chain

```
base_link → base_link_inertia → shoulder_link → upper_arm_link → forearm_link
  → wrist_1_link → wrist_2_link → wrist_3_link → flange → tool0
                                                      ↳ ft_frame
```

## Key Frames

| Frame | Use |
|-------|-----|
| `base_link` | World attachment (empty due to KDL limitation) |
| `base_link_inertia` | Physical properties (mass, inertia, meshes) |
| `flange` | Attach end-effectors |
| `tool0` | Default TCP |
| `ft_frame` | Force/torque sensor reference |

> **Note:** `base_link` has no inertia because KDL doesn't support inertia on root links. See [KDL Issue](https://github.com/ros/kdl_parser/issues/27).

## End-Effector Chain (netft_probe.urdf)

```
netft_link → p42v_link → tcp
```

## Source

Derived from [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).

## Converting ROS URDF to Drake Format

Drake only supports **OBJ** mesh format (not DAE or STL).

### Step 1: Get Original ROS URDF

```bash
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
```

Source files:
- URDF: `Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro`
- Visual meshes: `Universal_Robots_ROS2_Description/meshes/ur5e/visual/*.dae`
- Collision meshes: `Universal_Robots_ROS2_Description/meshes/ur5e/collision/*.stl`

### Step 2: Convert Visual Meshes (DAE → OBJ)

Use [ImageToStl.com](https://imagetostl.com/convert/file/dae/to/obj#convert):
1. Upload all `.dae` files from `meshes/ur5e/visual/`
2. Download converted `.obj` + `.mtl` files
3. Place in `ur5e_description/meshes/ur5e/visual/`

**Why not other tools?**
| Tool | Issue |
|------|-------|
| assimp | Swaps Y/Z axes (Z_UP → Y_UP), breaks geometry |
| MeshLab | Preserves geometry but loses per-face materials |
| Blender 4.x | No COLLADA import (addon removed) |

### Step 3: Convert Collision Meshes (STL → OBJ)

Use MeshLab (preserves geometry, colors not needed):
```bash
sudo apt install meshlab
meshlabserver -i input.stl -o output.obj
```

### Step 4: Update URDF

1. Change package name: `package://ur_description/` → `package://ur5e_description/`
2. Change mesh extension: `.dae` → `.obj` and `.stl` → `.obj`

## Visualization

```bash
python visualize.py
```

## References

- [UR Robot Frames](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_description/doc/robot_frames.html)
- [UR ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [REP-103](https://ros.org/reps/rep-0103.html) | [REP-199](https://gavanderhoorn.github.io/rep/rep-0199.html)
