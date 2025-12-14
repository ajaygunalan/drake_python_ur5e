"""Visualize human mannequin model with UR5e robot in Drake/Meshcat.

Usage:
    python visualize_human_models.py
"""
import sys
import numpy as np
from pathlib import Path

# Add parent folder to path for ur5e_description import
sys.path.insert(0, str(Path(__file__).parent.parent))

from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph, Parser,
    MeshcatVisualizer, StartMeshcat, RigidTransform, RotationMatrix, RollPitchYaw
)
from ur5e_description.ur5e_driver import UR5E_PACKAGE_XML

# Package paths
HUMAN_GAZEBO_PKG = str(Path(__file__).parent / "package.xml")
HUMAN_URDF = "humanSubjectWithMeshes/humanSubjectWithMesh.urdf"


def main():
    print("Loading human model with UR5e + NetFT probe...")

    meshcat = StartMeshcat()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)

    # Add package paths
    parser.package_map().AddPackageXml(UR5E_PACKAGE_XML)
    parser.package_map().AddPackageXml(HUMAN_GAZEBO_PKG)

    # Add UR5e robot
    ur5e = parser.AddModelsFromUrl("package://ur5e_description/ur5e.urdf")[0]

    # Add NetFT + Probe end-effector
    netft = parser.AddModelsFromUrl("package://ur5e_description/netft_probe.urdf")[0]

    # Weld probe to UR5e flange
    X_FlangeProbe = RigidTransform(
        RotationMatrix(RollPitchYaw(np.radians([90, 0, 90]))),
        [-0.01, 0.0, 0.0]
    )
    plant.WeldFrames(
        plant.GetFrameByName("flange", ur5e),
        plant.GetFrameByName("netft_link", netft),
        X_FlangeProbe
    )

    # Add human model
    human_path = Path(__file__).parent / HUMAN_URDF
    human = parser.AddModels(str(human_path))[0]

    # Position human lying on ground (on back, face up toward +Z, rotated 90° about Z)
    human_frame = plant.GetFrameByName("Pelvis", human)
    R = RotationMatrix.MakeZRotation(np.pi/2) @ RotationMatrix.MakeYRotation(-np.pi/2)
    X_WH = RigidTransform(R, [0.8, 0.0, 0.1])
    plant.WeldFrames(plant.world_frame(), human_frame, X_WH)

    # Set arm joints to bring arms parallel to body (down at sides)
    # Rotate arms around X-axis (opposite directions for mirrored arms)
    plant.GetJointByName("jRightShoulder_rotx", human).set_default_angle(np.pi/2)
    plant.GetJointByName("jLeftShoulder_rotx", human).set_default_angle(-np.pi/2)

    plant.Finalize()

    # Add meshcat visualizer
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

    # Set UR5e to home position
    plant_context = plant.GetMyContextFromRoot(context)
    home_q = np.array([0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0])
    plant.SetPositions(plant_context, ur5e, home_q)

    diagram.ForcedPublish(context)

    print(f"\nMeshcat URL: {meshcat.web_url()}")
    print("Human model + UR5e + NetFT probe loaded. Press Enter to exit...")
    input()


if __name__ == "__main__":
    main()
