from wpimath.geometry import Pose2d

from .math import kMetersPerFoot, kMetersPerInch

# Simulation Parameters
kSimulationRotationalInertia = 0.0002
kSimulationRotationalInertiaFlywheel = 0.002
kSimMotorResistance = 0.002
"""[meters, meters, radians]"""

kSimDefaultRobotLocation = Pose2d(0, 0, 0)
kSimDefaultTargetHeight = 8 * kMetersPerFoot + 8 * kMetersPerInch  # 8ft 8in

kSimRobotPoseArrayKey = "SimRobotPoseArray"
kSimRobotVelocityArrayKey = "SimRobotVelocityArray"

kMotorBaseKey = "motors"
