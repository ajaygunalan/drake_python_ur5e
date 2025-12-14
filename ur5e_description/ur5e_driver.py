"""UR5e Driver - Hardware interface using ur_rtde.

Simulation: Use MakeHardwareStation with !InverseDynamicsDriver (no custom code needed)
Hardware: Use UR5eDriver class directly
"""
import numpy as np
from pathlib import Path
from pydrake.all import LeafSystem

UR5E_PACKAGE_XML = str(Path(__file__).parent / 'package.xml')

try:
    from ur_rtde import RTDEControlInterface, RTDEReceiveInterface
    _UR_RTDE_AVAILABLE = True
except ImportError:
    _UR_RTDE_AVAILABLE = False


class UR5eDriver(LeafSystem):
    """Hardware driver for UR5e using ur_rtde (like IiwaStatusReceiver for IIWA)."""

    def __init__(self, robot_ip):
        super().__init__()
        if not _UR_RTDE_AVAILABLE:
            raise ImportError("ur_rtde not installed. Run: pip install ur-rtde")

        self._rtde_c = RTDEControlInterface(robot_ip)
        self._rtde_r = RTDEReceiveInterface(robot_ip)

        self.DeclareVectorInputPort("desired_state", 12)
        self.DeclareVectorOutputPort("state_estimated", 12, self._CalcState)
        self.DeclarePeriodicPublishEvent(0.002, 0.0, self._SendCommand)

    def _CalcState(self, context, output):
        q = np.array(self._rtde_r.getActualQ())
        v = np.array(self._rtde_r.getActualQd())
        output.SetFromVector(np.concatenate([q, v]))

    def _SendCommand(self, context):
        state = self.get_input_port(0).Eval(context)
        self._rtde_c.speedJ(state[6:].tolist(), acceleration=1.0, time=0.1)

    def stop(self):
        self._rtde_c.speedStop(0.5)
        self._rtde_c.stopScript()
