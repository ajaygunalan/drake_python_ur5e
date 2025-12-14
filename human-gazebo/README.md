# Human Model for Drake Visualization

Human mannequin model for robotics simulation with Drake/Meshcat.

## Original Source

Forked from: https://github.com/robotology/human-gazebo

The original repo contains multiple human subjects with different body measurements, spinal cord models, and Gazebo control files. This version is simplified to include only the mesh-based human model for Drake visualization.

## Usage

```bash
source ../drake-env/bin/activate
python visualize_human_models.py
```

Opens Meshcat with human model + UR5e robot with NetFT probe.

## Contents

- `meshes/` - 51 DAE mesh files (converted from original STL for Meshcat compatibility)
- `humanSubjectWithMeshes/` - URDF model
- `visualize_human_models.py` - Drake visualization script

## License

Original meshes under CC-BY-SA license. See LICENSE file.
