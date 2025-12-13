"""
UR5e Hardware Station - Hardware abstraction for simulation and real UR5e robot.

This module provides a unified interface similar to IIWA's MakeHardwareStation,
supporting both simulation (via Drake) and real hardware (via ur_rtde).

Usage:
    # Simulation mode (default)
    station = MakeUR5eHardwareStation(meshcat=meshcat)

    # Real hardware mode
    station = MakeUR5eHardwareStation(robot_ip="192.168.1.102", meshcat=meshcat)
"""

import numpy as np
from pathlib import Path

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ApplyVisualizationConfig,
    Demultiplexer,
    DiagramBuilder,
    InverseDynamicsController,
    LeafSystem,
    Parser,
    VisualizationConfig,
)

from manipulation import ConfigureParser

# Check for ur_rtde availability
try:
    from ur_rtde import RTDEControlInterface, RTDEReceiveInterface
    UR_RTDE_AVAILABLE = True
except ImportError:
    UR_RTDE_AVAILABLE = False


# Package path for UR5e description
UR5E_DESCRIPTION_PATH = Path(__file__).parent


class VelocityToDesiredStateAdapter(LeafSystem):
    """
    Adapts velocity commands (6) to desired_state (12) for InverseDynamicsController.

    For velocity control, we pass zeros for q_d and the velocity command for v_d.
    """
    def __init__(self, num_joints=6):
        super().__init__()
        self._n = num_joints

        # Input: velocity command (6)
        self.DeclareVectorInputPort("velocity_command", num_joints)

        # Output: desired_state [q_d, v_d] (12)
        self.DeclareVectorOutputPort(
            "desired_state", 2 * num_joints, self.CalcDesiredState
        )

    def CalcDesiredState(self, context, output):
        v_cmd = self.get_input_port().Eval(context)
        # q_d is zeros, v_d is the commanded velocity
        # Note: InverseDynamicsController with kp=0, kd>0 acts as velocity damper/servo
        desired_state = np.concatenate([np.zeros(self._n), v_cmd])
        output.SetFromVector(desired_state)


class PositionVelocityToDesiredStateAdapter(LeafSystem):
    """
    Combines position command and velocity feedforward into desired_state for
    InverseDynamicsController.

    Takes both position (q_d) and velocity (v_d) as external inputs to avoid
    the damping term fighting against robot motion during trajectory tracking.
    """
    def __init__(self, num_joints=6):
        super().__init__()
        self._n = num_joints

        # Inputs: position command (6) and velocity feedforward (6)
        self.DeclareVectorInputPort("position_command", num_joints)
        self.DeclareVectorInputPort("velocity_feedforward", num_joints)

        # Output: desired_state [q_d, v_d] (12)
        self.DeclareVectorOutputPort(
            "desired_state", 2 * num_joints, self.CalcDesiredState
        )

    def CalcDesiredState(self, context, output):
        q_cmd = self.get_input_port(0).Eval(context)
        v_ff = self.get_input_port(1).Eval(context)
        desired_state = np.concatenate([q_cmd, v_ff])
        output.SetFromVector(desired_state)


class UR5eHardwareInterface(LeafSystem):
    """
    Hardware interface for real UR5e robot using ur_rtde.
    """
    def __init__(self, robot_ip: str, control_mode: str = "velocity"):
        super().__init__()
        if not UR_RTDE_AVAILABLE:
            raise ImportError("ur_rtde not installed. Run: pip install ur-rtde")
        self._robot_ip = robot_ip
        self._control_mode = control_mode
        self._n = 6
        self._rtde_c = RTDEControlInterface(robot_ip)
        self._rtde_r = RTDEReceiveInterface(robot_ip)
        if control_mode == "velocity":
            self.DeclareVectorInputPort("ur5e.velocity", self._n)
        else:
            self.DeclareVectorInputPort("ur5e.position", self._n)
        self.DeclareVectorOutputPort("ur5e.position_measured", self._n, self.CalcPositionOutput)
        self.DeclareVectorOutputPort("ur5e.velocity_measured", self._n, self.CalcVelocityOutput)
        self.DeclarePeriodicPublishEvent(period_sec=0.002, offset_sec=0.0, publish=self.SendCommand)
        self._last_q = np.zeros(self._n)
        self._last_qd = np.zeros(self._n)

    def CalcPositionOutput(self, _context, output):
        self._last_q = np.array(self._rtde_r.getActualQ())
        output.SetFromVector(self._last_q)

    def CalcVelocityOutput(self, _context, output):
        self._last_qd = np.array(self._rtde_r.getActualQd())
        output.SetFromVector(self._last_qd)

    def SendCommand(self, context):
        cmd = self.get_input_port(0).Eval(context)
        if self._control_mode == "velocity":
            self._rtde_c.speedJ(cmd.tolist(), acceleration=1.0, time=0.1)
        else:
            self._rtde_c.servoJ(cmd.tolist(), speed=0.5, acceleration=1.0, time=0.002, lookahead_time=0.1, gain=300)

    def stop(self):
        if self._control_mode == "velocity":
            self._rtde_c.speedStop(0.5)
        else:
            self._rtde_c.servoStop()
        self._rtde_c.stopScript()


def MakeUR5eHardwareStation(
    robot_ip: str = None,
    meshcat=None,
    control_mode: str = "velocity",
    time_step: float = 1e-3,
):
    """
    Factory function to create UR5e hardware station.
    """
    if robot_ip is not None:
        return _MakeUR5eHardwareStation_Hardware(robot_ip, control_mode)
    else:
        return _MakeUR5eSimulationStation(meshcat, control_mode, time_step)


def _MakeUR5eSimulationStation(meshcat, control_mode: str, time_step: float):
    """Create simulation station using Drake."""
    builder = DiagramBuilder()

    # Create plant and scene graph
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    plant.set_name("plant")

    parser = Parser(plant)
    ConfigureParser(parser)

    # Register UR5e package
    parser.package_map().Add('ur5e_description', str(UR5E_DESCRIPTION_PATH))

    # Load UR5e + NetFT probe composite model
    parser.AddModels(str(UR5E_DESCRIPTION_PATH / 'ur5e_netft_probe.dmd.yaml'))
    
    # Extract default positions from the YAML file manually
    # (Since Drake's Parser might ignore them depending on context)
    import yaml
    
    # Handle Drake custom tags like !Rpy
    def rpy_constructor(loader, node):
        return loader.construct_mapping(node)
    
    yaml.SafeLoader.add_constructor('!Rpy', rpy_constructor)

    with open(UR5E_DESCRIPTION_PATH / 'ur5e_netft_probe.dmd.yaml', 'r') as f:
        dmd_config = yaml.safe_load(f)
        
    # Find the ur5e model directive
    ur5e_directive = next(d for d in dmd_config['directives'] if d.get('add_model', {}).get('name') == 'ur5e')
    default_positions = ur5e_directive['add_model'].get('default_joint_positions', {})

    # Apply default positions to the plant
    ur5e = plant.GetModelInstanceByName("ur5e")
    for joint_name, angle_list in default_positions.items():
        # The YAML list might be [angle], extract the float
        angle = angle_list[0] if isinstance(angle_list, list) else angle_list
        plant.GetJointByName(joint_name, ur5e).set_default_angle(angle)

    plant.Finalize()

    # Add visualizer if meshcat provided (with all visualization types)
    if meshcat is not None:
        meshcat.Delete()
        ApplyVisualizationConfig(
            VisualizationConfig(), builder, meshcat=meshcat
        )

    n = 6  # UR5e has 6 joints

    # Controller gains for InverseDynamicsController
    # These are applied to (q_d - q) and (v_d - v) then multiplied by M(q)
    # kp=100 (w=10 rad/s), kd=20 (damping ratio 1), ki=1
    if control_mode == "velocity":
        # Velocity mode: Zero stiffness, just track velocity
        kp = np.zeros(n)
        kd = np.full(n, 20.0) 
        ki = np.zeros(n)
    else:
        # Position mode: stiff tracking
        kp = np.full(n, 100.0)
        kd = np.full(n, 20.0)
        ki = np.full(n, 1.0)

    # Use InverseDynamicsController for gravity compensation and better tracking
    controller = builder.AddSystem(InverseDynamicsController(
        plant, kp=kp, ki=ki, kd=kd, has_reference_acceleration=False))
    controller.set_name("controller")

    # Connect controller output to plant actuation
    builder.Connect(
        controller.get_output_port_generalized_force(),
        plant.get_actuation_input_port()
    )

    # Connect plant state to controller estimated_state
    builder.Connect(
        plant.get_state_output_port(),
        controller.get_input_port_estimated_state()
    )

    # Create command adapter based on control mode
    if control_mode == "velocity":
        adapter = builder.AddSystem(VelocityToDesiredStateAdapter(n))
        adapter.set_name("velocity_adapter")
    else:
        adapter = builder.AddSystem(PositionVelocityToDesiredStateAdapter(n))
        adapter.set_name("position_velocity_adapter")

    # Connect adapter to controller desired_state
    builder.Connect(
        adapter.get_output_port(),
        controller.get_input_port_desired_state()
    )

    # Demultiplex plant state [q, v] (12) into position (6) and velocity (6)
    demux = builder.AddSystem(Demultiplexer(2 * n, n))
    demux.set_name("state_demux")
    builder.Connect(plant.get_state_output_port(), demux.get_input_port())

    # Export ports
    if control_mode == "velocity":
        builder.ExportInput(adapter.get_input_port(), "ur5e.velocity")
    else:
        # Position mode: export both position and velocity feedforward ports
        builder.ExportInput(adapter.get_input_port(0), "ur5e.position")
        builder.ExportInput(adapter.get_input_port(1), "ur5e.velocity_ff")

    builder.ExportOutput(demux.get_output_port(0), "ur5e.position_measured")
    builder.ExportOutput(demux.get_output_port(1), "ur5e.velocity_measured")

    return builder.Build()


def _MakeUR5eHardwareStation_Hardware(robot_ip: str, control_mode: str):
    """Create hardware station for real robot."""
    builder = DiagramBuilder()

    # Create hardware interface
    hw_interface = builder.AddSystem(
        UR5eHardwareInterface(robot_ip, control_mode)
    )
    hw_interface.set_name("hardware_interface")

    # For hardware mode, we still need a plant for Jacobian calculations
    # Create a "shadow" plant that's not connected to the real robot
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    plant.set_name("plant")

    parser = Parser(plant)
    ConfigureParser(parser)
    parser.package_map().Add('ur5e_description', str(UR5E_DESCRIPTION_PATH))
    parser.AddModels(str(UR5E_DESCRIPTION_PATH / 'ur5e_netft_probe.dmd.yaml'))
    plant.Finalize()

    # Export ports
    if control_mode == "velocity":
        builder.ExportInput(hw_interface.get_input_port(0), "ur5e.velocity")
    else:
        builder.ExportInput(hw_interface.get_input_port(0), "ur5e.position")

    builder.ExportOutput(
        hw_interface.GetOutputPort("ur5e.position_measured"),
        "ur5e.position_measured"
    )
    builder.ExportOutput(
        hw_interface.GetOutputPort("ur5e.velocity_measured"),
        "ur5e.velocity_measured"
    )

    return builder.Build()


# Convenience function to get plant from station
def GetPlantFromStation(station):
    """Get the MultibodyPlant from a UR5e station (works for both sim and hardware)."""
    return station.GetSubsystemByName("plant")


def MakeUR5eHardwareStationWithScene(
    scene_models: list = None,
    robot_ip: str = None,
    meshcat=None,
    control_mode: str = "velocity",
    time_step: float = 1e-3,
):
    """
    Factory function to create UR5e station with additional scene objects.

    Args:
        scene_models: List of (model_url, X_WModel) tuples for objects to add.
            Example: [("package://manipulation/hydro/061_foam_brick.sdf",
                       RigidTransform([0.4, 0, 0]))]
        robot_ip: None for simulation, IP string for real hardware.
        meshcat: Meshcat instance for visualization.
        control_mode: "velocity" or "position".
        time_step: Simulation time step.

    Returns:
        Built diagram with UR5e station and scene objects.
    """
    if robot_ip is not None:
        # Hardware mode doesn't support scene objects (real world has its own scene)
        return _MakeUR5eHardwareStation_Hardware(robot_ip, control_mode)
    else:
        return _MakeUR5eSimulationStationWithScene(
            scene_models, meshcat, control_mode, time_step
        )


def _MakeUR5eSimulationStationWithScene(
    scene_models: list,
    meshcat,
    control_mode: str,
    time_step: float,
):
    """Create simulation station with scene objects using Drake."""
    builder = DiagramBuilder()

    # Create plant and scene graph
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    plant.set_name("plant")

    parser = Parser(plant)
    ConfigureParser(parser)

    # Register UR5e package
    parser.package_map().Add('ur5e_description', str(UR5E_DESCRIPTION_PATH))

    # Load UR5e + NetFT probe composite model
    parser.AddModels(str(UR5E_DESCRIPTION_PATH / 'ur5e_netft_probe.dmd.yaml'))

    # Add scene objects (welded to world)
    if scene_models:
        parser.SetAutoRenaming(True)
        for model_url, X_WModel in scene_models:
            model_instance = parser.AddModelsFromUrl(model_url)[0]
            # Get the base link of the added model
            body_indices = plant.GetBodyIndices(model_instance)
            if body_indices:
                base_body = plant.get_body(body_indices[0])
                plant.WeldFrames(
                    plant.world_frame(),
                    base_body.body_frame(),
                    X_WModel
                )

    # Extract default positions from the YAML file manually
    import yaml

    # Handle Drake custom tags like !Rpy
    def rpy_constructor(loader, node):
        return loader.construct_mapping(node)

    yaml.SafeLoader.add_constructor('!Rpy', rpy_constructor)

    with open(UR5E_DESCRIPTION_PATH / 'ur5e_netft_probe.dmd.yaml', 'r') as f:
        dmd_config = yaml.safe_load(f)

    # Find the ur5e model directive
    ur5e_directive = next(
        d for d in dmd_config['directives']
        if d.get('add_model', {}).get('name') == 'ur5e'
    )
    default_positions = ur5e_directive['add_model'].get('default_joint_positions', {})

    # Apply default positions to the plant
    ur5e = plant.GetModelInstanceByName("ur5e")
    for joint_name, angle_list in default_positions.items():
        # The YAML list might be [angle], extract the float
        angle = angle_list[0] if isinstance(angle_list, list) else angle_list
        plant.GetJointByName(joint_name, ur5e).set_default_angle(angle)

    plant.Finalize()

    # Add visualizer if meshcat provided (with all visualization types)
    if meshcat is not None:
        meshcat.Delete()
        ApplyVisualizationConfig(
            VisualizationConfig(), builder, meshcat=meshcat
        )

    n = 6  # UR5e has 6 joints

    # Controller gains for InverseDynamicsController
    if control_mode == "velocity":
        kp = np.zeros(n)
        kd = np.full(n, 20.0)
        ki = np.zeros(n)
    else:
        kp = np.full(n, 100.0)
        kd = np.full(n, 20.0)
        ki = np.full(n, 1.0)

    # Use InverseDynamicsController for gravity compensation and better tracking
    controller = builder.AddSystem(InverseDynamicsController(
        plant, kp=kp, ki=ki, kd=kd, has_reference_acceleration=False))
    controller.set_name("controller")

    # Connect controller output to plant actuation
    builder.Connect(
        controller.get_output_port_generalized_force(),
        plant.get_actuation_input_port()
    )

    # Connect plant state to controller estimated_state
    builder.Connect(
        plant.get_state_output_port(),
        controller.get_input_port_estimated_state()
    )

    # Create command adapter based on control mode
    if control_mode == "velocity":
        adapter = builder.AddSystem(VelocityToDesiredStateAdapter(n))
        adapter.set_name("velocity_adapter")
    else:
        adapter = builder.AddSystem(PositionVelocityToDesiredStateAdapter(n))
        adapter.set_name("position_velocity_adapter")

    # Connect adapter to controller desired_state
    builder.Connect(
        adapter.get_output_port(),
        controller.get_input_port_desired_state()
    )

    # Demultiplex plant state [q, v] (12) into position (6) and velocity (6)
    demux = builder.AddSystem(Demultiplexer(2 * n, n))
    demux.set_name("state_demux")
    builder.Connect(plant.get_state_output_port(), demux.get_input_port())

    # Export ports
    if control_mode == "velocity":
        builder.ExportInput(adapter.get_input_port(), "ur5e.velocity")
    else:
        # Position mode: export both position and velocity feedforward ports
        builder.ExportInput(adapter.get_input_port(0), "ur5e.position")
        builder.ExportInput(adapter.get_input_port(1), "ur5e.velocity_ff")

    builder.ExportOutput(demux.get_output_port(0), "ur5e.position_measured")
    builder.ExportOutput(demux.get_output_port(1), "ur5e.velocity_measured")

    return builder.Build()
