from wpimath.geometry import Pose2d, Rotation2d

from .math import kMetersPerFoot, kMetersPerInch

# Simulation Parameters
kSimulationRotationalInertia = 0.0002
kSimulationRotationalInertiaFlywheel = 0.002
kSimMotorResistance = 0.002
"""[meters, meters, radians]"""

kSimDefaultRobotLocation = Pose2d(4, 4, 0)
kSimDefaultTargetHeight = 8 * kMetersPerFoot + 8 * kMetersPerInch  # 8ft 8in

kSimRobotPoseArrayKey = "SimRobotPoseArray"
kSimRobotVelocityArrayKey = "SimRobotVelocityArray"

kMotorBaseKey = "motors"

kFuelSimEffectiveWheelRadius = 0.027  # meters
# the effective radius of the shooter wheels when exiting the system,
# used as a velocity calibration factor
kFuelSimEffectiveExitAngle = lambda originalAngle: Rotation2d.fromDegrees(
    90 - (8 + originalAngle.degrees())
)
# based on testing
kFuelSimEffectiveVelocity = (
    lambda originalVelocity: originalVelocity * kFuelSimEffectiveWheelRadius
)
kSimGravity = 9.81  # m/s^2

kSimBPS = 10  # number of simulated balls per second
kSimSecondsPerBall = 1 / kSimBPS
