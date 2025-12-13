# Drake + UR5e Hardware Integration Guide

This guide documents how to run Drake controllers on both simulation and real UR5e hardware.

## Drake's Official Recommendation

Per the MIT Manipulation Course (Russ Tedrake):

> "We use **LCM** for this instead of ROS messages, primarily because LCM is a
> lighter-weight dependency for our public repository (also because **multicast
> UDP is a better choice than TCP/IP for the driver-level interface**)."
>
> — http://manipulation.csail.mit.edu/station.html

**However**, for UR5e specifically with velocity control, **ur_rtde is a practical choice** because:

1. Drake doesn't have a native LCM-based UR driver
2. ur_rtde provides direct `speedJ()` for velocity control with lower latency (~2ms vs ~10ms)
3. Simpler setup than ROS2 (just `pip install ur-rtde`)

> **Note**: ROS2 ur_robot_driver now supports velocity control via `forward_velocity_controller`, but ur_rtde remains simpler for direct velocity control use cases.

---

## Option 1: Native UR Control (ur_rtde) - RECOMMENDED FOR UR5e

### Why This Approach?

| Advantage | Description |
|-----------|-------------|
| **Lower latency** | ~2ms vs ~10ms with ROS2 |
| **Simpler setup** | Just `pip install ur-rtde` |
| **Direct mapping** | `speedJ()` takes exactly what pseudoinverse controller outputs |
| **No ROS2 dependency** | Python-only, no ROS2 infrastructure needed |

### Required Components

#### 1. ur_rtde Library (C++ with Python bindings)

- **GitLab**: https://gitlab.com/sdurobotics/ur_rtde
- **PyPI**: `pip install ur-rtde`
- **Documentation**: https://sdurobotics.gitlab.io/ur_rtde/

> **Note**: This is the SDU Robotics ur_rtde library, which is different from Universal Robots' official [RTDE_Python_Client_Library](https://github.com/UniversalRobots/RTDE_Python_Client_Library) reference implementation.

#### 2. RTDE Protocol (Port 30004)

- **Official docs**: https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html
- **Update rate**: Up to 500 Hz on e-Series (default 125 Hz, configurable 1-500 Hz)
- **Protocol**: Bidirectional TCP data exchange

### Architecture

```
┌─────────────────────┐     ┌──────────────────────┐     ┌─────────────┐
│  Drake Controller   │ ──▶ │ UR5eHardwareInterface │ ──▶ │  Real UR5e  │
│ (PseudoInverse)     │     │     (LeafSystem)      │     │   Robot     │
└─────────────────────┘     └──────────────────────┘     └─────────────┘
         │                            │
         │                            ▼
         │                   RTDEControlInterface.speedJ()
         │
         ▼
    Joint velocities [6]
```

### Implementation

#### Step 1: Create Hardware Abstraction LeafSystem

```python
from pydrake.all import LeafSystem
import numpy as np

# Only import ur_rtde when using real hardware
try:
    from ur_rtde import rtde_control, rtde_receive
    UR_RTDE_AVAILABLE = True
except ImportError:
    UR_RTDE_AVAILABLE = False


class UR5eHardwareInterface(LeafSystem):
    """
    Hardware abstraction layer for UR5e.
    Works with both simulation (Drake plant) and real hardware (ur_rtde).
    """

    def __init__(self, robot_ip: str = None, simulation_plant=None):
        """
        Args:
            robot_ip: IP address of real UR5e (e.g., "192.168.1.102").
                      If None, uses simulation mode.
            simulation_plant: Drake MultibodyPlant for simulation mode.
        """
        super().__init__()
        self._is_simulation = (robot_ip is None)

        # Common ports (same interface for sim and real)
        self.DeclareVectorInputPort("joint_velocity_command", 6)
        self.DeclareVectorOutputPort("joint_positions", 6, self.CalcPositionOutput)
        self.DeclareVectorOutputPort("joint_velocities", 6, self.CalcVelocityOutput)

        if self._is_simulation:
            assert simulation_plant is not None, "Must provide plant for simulation"
            self._plant = simulation_plant
            self._plant_context = simulation_plant.CreateDefaultContext()
            self._ur5e = simulation_plant.GetModelInstanceByName("ur5e")
        else:
            # Real hardware
            assert UR_RTDE_AVAILABLE, "ur_rtde not installed. Run: pip install ur-rtde"
            self._rtde_c = rtde_control.RTDEControlInterface(robot_ip)
            self._rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
            self._last_q = np.zeros(6)
            self._last_qd = np.zeros(6)

        # Periodic event for sending commands (500 Hz)
        self.DeclarePeriodicPublishEvent(
            period_sec=0.002,  # 500 Hz
            offset_sec=0.0,
            publish=self.SendCommand
        )

    def CalcPositionOutput(self, context, output):
        if self._is_simulation:
            # Get from plant state (would be connected via diagram)
            output.SetFromVector(np.zeros(6))  # Placeholder
        else:
            self._last_q = np.array(self._rtde_r.getActualQ())
            output.SetFromVector(self._last_q)

    def CalcVelocityOutput(self, context, output):
        if self._is_simulation:
            output.SetFromVector(np.zeros(6))  # Placeholder
        else:
            self._last_qd = np.array(self._rtde_r.getActualQd())
            output.SetFromVector(self._last_qd)

    def SendCommand(self, context):
        """Send velocity command to robot."""
        v_cmd = self.get_input_port(0).Eval(context)

        if not self._is_simulation:
            # Send to real robot
            # speedJ(qd, acceleration, time)
            # - qd: joint velocities [rad/s]
            # - acceleration: joint acceleration [rad/s²]
            # - time: duration before timeout [s]
            self._rtde_c.speedJ(
                v_cmd.tolist(),
                acceleration=1.0,  # rad/s²
                time=0.1  # slightly > loop period for smooth motion
            )

    def stop(self):
        """Stop robot motion (call when done)."""
        if not self._is_simulation:
            self._rtde_c.speedStop(0.5)  # deceleration = 0.5 rad/s²
            self._rtde_c.stopScript()
```

#### Step 2: Usage Pattern

```python
# ========== SIMULATION MODE ==========
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
# ... load UR5e model ...
plant.Finalize()

interface = UR5eHardwareInterface(simulation_plant=plant)


# ========== REAL HARDWARE MODE ==========
interface = UR5eHardwareInterface(robot_ip="192.168.1.102")

# Don't forget to stop when done!
try:
    # ... run controller ...
finally:
    interface.stop()
```

### ur_rtde API Reference

#### RTDEControlInterface Methods

| Method | Description |
|--------|-------------|
| `speedJ(qd, a, t)` | Joint velocity control |
| `speedL(xd, a, t)` | Cartesian velocity control |
| `servoJ(q, speed, accel, time, lookahead, gain)` | Position servo |
| `speedStop(a)` | Stop with deceleration |
| `stopScript()` | Clean shutdown |

#### speedJ() Parameters

```python
rtde_c.speedJ(qd, acceleration, time)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `qd` | `list[float]` | 6 joint velocities [rad/s] |
| `acceleration` | `float` | Joint acceleration limit [rad/s²] |
| `time` | `float` | Duration before timeout [s] |

**Important**: Keep `time` slightly larger than your control loop period and call `speedJ()` continuously.

---

## Option 2: ROS2 Integration

### When to Use ROS2?

- Already have ROS2 infrastructure
- Need MoveIt2 integration
- Want trajectory controller (position-based, not velocity)

### Components

1. **ur_robot_driver**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
2. **drake-ros**: https://github.com/RobotLocomotion/drake-ros

### Limitations

- Higher latency than direct RTDE (~10ms vs ~2ms)
- More complex setup (full ROS2 installation required)
- Velocity control available via `forward_velocity_controller`, but requires ros2_control setup

### Setup

```bash
# Install ROS2 (Ubuntu)
sudo apt install ros-humble-desktop

# Install UR driver
sudo apt install ros-humble-ur-robot-driver

# Launch driver
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.1.102
```

---

## Comparison Table

| Aspect | Option 1: ur_rtde | Option 2: ROS2 |
|--------|-------------------|----------------|
| **Latency** | ~2ms | ~10ms |
| **Setup complexity** | Low (`pip install`) | High (ROS2 ecosystem) |
| **Velocity control** | Yes (`speedJ`) | Yes (`forward_velocity_controller`) |
| **Position control** | Yes (`servoJ`) | Yes (trajectory controller) |
| **Dependencies** | Python only | ROS2 + driver packages |
| **Drake recommendation** | Practical choice for UR | Supported alternative |

---

## References

1. **ur_rtde Documentation**: https://sdurobotics.gitlab.io/ur_rtde/
2. **RTDE Protocol Guide**: https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html
3. **URScript Manual**: https://www.universal-robots.com/download/manuals-e-seriesur-series/script/script-manual-e-series-and-ur-series-sw-517/
4. **Drake Hardware Sim Example**: https://github.com/RobotLocomotion/drake/tree/master/examples/hardware_sim
5. **UR ROS2 Driver**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
6. **drake-ros Bridge**: https://github.com/RobotLocomotion/drake-ros
7. **MIT Manipulation Course - Hardware Setup**: http://manipulation.csail.mit.edu/station.html
