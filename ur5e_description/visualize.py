"""
UR5e + NetFT Probe Visualization in Drake Meshcat

Usage:
    python visualize.py

Then open the Meshcat URL printed in the terminal.
Use joint sliders to move robot. Press Escape in browser or Stop button to exit.
"""

from pydrake.visualization import ModelVisualizer

def main():
    visualizer = ModelVisualizer()
    visualizer.parser().package_map().Add(
        'ur5e_description', '/home/ajay/drake/ur5e_description'
    )
    visualizer.AddModels('/home/ajay/drake/ur5e_description/ur5e_netft_probe.dmd.yaml')
    visualizer.Run()

if __name__ == "__main__":
    main()
