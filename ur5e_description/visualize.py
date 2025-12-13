"""
UR5e + NetFT Probe Visualization in Drake Meshcat

Usage:
    python visualize.py --pose initialization  (Upright, default)
    python visualize.py --pose home            (Elbow straight)

Then open the Meshcat URL printed in the terminal.
Use joint sliders to move robot. Press Escape in browser or Stop button to exit.
"""

import argparse
from pathlib import Path
from pydrake.visualization import ModelVisualizer

# Home pose: same as initialization but with elbow at 0
HOME_POSE = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.5708,   # -90 deg
    "elbow_joint": 0.0,               # 0 deg (differs from initialization)
    "wrist_1_joint": -1.5708,         # -90 deg
    "wrist_2_joint": -1.5708,         # -90 deg
    "wrist_3_joint": 0.0,
}

def main():
    parser = argparse.ArgumentParser(description="Visualize UR5e robot in Drake.")
    parser.add_argument(
        "--pose",
        choices=["initialization", "home"],
        default="initialization",
        help="Initial pose: 'initialization' (upright from dmd.yaml) or 'home' (elbow straight)"
    )
    args = parser.parse_args()

    pkg_dir = Path(__file__).parent

    visualizer = ModelVisualizer()
    visualizer.parser().package_map().Add('ur5e_description', str(pkg_dir))
    visualizer.AddModels(str(pkg_dir / "ur5e_netft_probe.dmd.yaml"))

    if args.pose == "home":
        plant = visualizer.parser().plant()
        ur5e = plant.GetModelInstanceByName("ur5e")
        for joint, angle in HOME_POSE.items():
            plant.GetJointByName(joint, ur5e).set_default_angle(angle)
        print("Robot initialized to 'home' pose [0, -90, 0, -90, -90, 0] deg.")
    else:
        print("Robot initialized to 'initialization' pose (from dmd.yaml).")

    visualizer.Run()

if __name__ == "__main__":
    main()
