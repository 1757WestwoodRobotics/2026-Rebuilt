# pylint:disable=too-many-lines
"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!

Physical constants must have their units specified
Default units:
    Length: meters
    Angle: radians

Axes Convention (right hand rule):
    Translation:
        +X: forward
        +Y: left
        +Z: up

    Rotation:
        +rotate around X: counterclockwise looking from the front, 0 aligned with +Y
        +rotate around Y: counterclockwise looking from the left, 0 aligned with +Z
        +rotate around Z: counterclockwise looking from the top, 0 aligned with +X

Swerve Module Layout:
    Drive (input) -> Drive Gearing -> Wheel (output)
    Steer (input) -> Steer Gearing -> Swerve (output)
"""

import math
from phoenix6.configs.config_groups import CurrentLimitsConfigs
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import (
    Pose3d,
    Pose2d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Translation2d,
)
from wpimath.system.plant import DCMotor
from pathplannerlib.config import (
    PIDConstants,
)
from pathplannerlib.auto import PathConstraints
from util.keyorganization import OptionalValueKeys


# Debug parameters
kPrintFrequency = 2
""" 1 / second"""

kPrintPeriod = 1 / kPrintFrequency


# CTRE

kRobotPoseArrayKeys = OptionalValueKeys("RobotOdometryPose")

kDriveVelocityKeys = "robotVelocity"
kDriveAccelLimit = 7
kRobotUpdatePeriod = 1 / 50
"""seconds"""
kLimelightUpdatePeriod = 1 / 10
"""seconds"""

# Vision parameters
kTargetAngleRelativeToRobotKeys = OptionalValueKeys("TargetAngleRelativeToRobot")
kTargetDistanceRelativeToRobotKeys = OptionalValueKeys("TargetDistanceRelativeToRobot")
kTargetFacingAngleRelativeToRobotKeys = OptionalValueKeys(
    "TargetFacingAngleRelativeToRobot"
)
kTargetPoseArrayKeys = OptionalValueKeys("TargetPoseArray")
kRobotVisionPose1ArrayKeys = OptionalValueKeys("EstimatedRobotPoseLL1")
kRobotVisionPose2ArrayKeys = OptionalValueKeys("EstimatedRobotPoseLL2")
kRobotToTagPoseKey = "vision/poses"
kRobotToTagIdKey = "vision/ids"
kRobotToTagAmbiguityKey = "vision/ambiguity"

kXyStdDevCoeff = 0.005
kThetaStdDevCoeff = 0.01

kTargetName = "Target"

kApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagField.k2024Crescendo)
kApriltagPositionDict = {
    1: Pose3d(
        (kMetersPerInch * 657.37),
        (kMetersPerInch * 25.80),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 126 * kRadiansPerDegree),
    ),
    2: Pose3d(
        (kMetersPerInch * 657.37),
        (kMetersPerInch * 291.20),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 234 * kRadiansPerDegree),
    ),
    3: Pose3d(
        (kMetersPerInch * 455.15),
        (kMetersPerInch * 317.15),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    4: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 241.64),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    5: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 75.39),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    6: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
    7: Pose3d(
        (kMetersPerInch * 546.87),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    8: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    9: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    10: Pose3d(
        (kMetersPerInch * 481.39),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    11: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    12: Pose3d(
        (kMetersPerInch * 33.51),
        (kMetersPerInch * 25.80),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 54 * kRadiansPerDegree),
    ),
    13: Pose3d(
        (kMetersPerInch * 33.51),
        (kMetersPerInch * 291.20),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 306 * kRadiansPerDegree),
    ),
    14: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 241.64),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    15: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 75.39),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    16: Pose3d(
        (kMetersPerInch * 235.73),
        (kMetersPerInch * -0.15),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 90 * kRadiansPerDegree),
    ),
    17: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    18: Pose3d(
        (kMetersPerInch * 144.00),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    19: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    20: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    21: Pose3d(
        (kMetersPerInch * 209.49),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    22: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
}

kApriltagPositionDictAndyMark = {
    1: Pose3d(
        (kMetersPerInch * 656.98),
        (kMetersPerInch * 24.73),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 126 * kRadiansPerDegree),
    ),
    2: Pose3d(
        (kMetersPerInch * 656.98),
        (kMetersPerInch * 291.90),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 234 * kRadiansPerDegree),
    ),
    3: Pose3d(
        (kMetersPerInch * 452.40),
        (kMetersPerInch * 316.21),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    4: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 241.44),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    5: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 75.19),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    6: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 129.97),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
    7: Pose3d(
        (kMetersPerInch * 546.87),
        (kMetersPerInch * 158.30),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    8: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 186.63),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    9: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 186.63),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    10: Pose3d(
        (kMetersPerInch * 481.39),
        (kMetersPerInch * 158.30),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    11: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 129.97),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    12: Pose3d(
        (kMetersPerInch * 33.91),
        (kMetersPerInch * 24.73),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 54 * kRadiansPerDegree),
    ),
    13: Pose3d(
        (kMetersPerInch * 33.91),
        (kMetersPerInch * 291.90),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 306 * kRadiansPerDegree),
    ),
    14: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 241.44),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    15: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 75.19),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    16: Pose3d(
        (kMetersPerInch * 238.49),
        (kMetersPerInch * 0.42),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 90 * kRadiansPerDegree),
    ),
    17: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 129.97),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    18: Pose3d(
        (kMetersPerInch * 144.00),
        (kMetersPerInch * 158.30),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    19: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 186.63),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    20: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 186.63),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    21: Pose3d(
        (kMetersPerInch * 209.49),
        (kMetersPerInch * 158.30),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    22: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 129.97),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
}

kReefBufferDistance = 0.5 * kMetersPerInch
kLeftReefOffsetX = 17.628 * kMetersPerInch
kLeftReefOffsetY = -16.455 * kMetersPerInch
kLeftReefOffset = Transform3d(
    kLeftReefOffsetX + kReefBufferDistance,
    kLeftReefOffsetY,
    0,
    Rotation3d(0, 0, math.pi),
)

kLeftReefToOffsetPositionBlue = {}
kLeftReefToOffsetPositionRed = {}

for i in range(17, 23):
    apriltag = kApriltagPositionDictAndyMark[i]
    kLeftReefToOffsetPositionBlue[i] = apriltag + kLeftReefOffset
for i in range(6, 12):
    apriltag = kApriltagPositionDictAndyMark[i]
    kLeftReefToOffsetPositionRed[i] = apriltag + kLeftReefOffset

kRightReefOffsetX = 17.628 * kMetersPerInch
kRightReefOffsetY = -2.688 * kMetersPerInch
kRightReefOffset = Transform3d(
    kRightReefOffsetX + kReefBufferDistance,
    kRightReefOffsetY,
    0,
    Rotation3d(0, 0, math.pi),
)

kRightReefToOffsetPositionBlue = {}
kRightReefToOffsetPositionRed = {}

for i in range(17, 23):
    apriltag = kApriltagPositionDictAndyMark[i]
    kRightReefToOffsetPositionBlue[i] = apriltag + kRightReefOffset
for i in range(6, 12):
    apriltag = kApriltagPositionDictAndyMark[i]
    kRightReefToOffsetPositionRed[i] = apriltag + kRightReefOffset

# Autonomous
kAutoDriveDistance = -8 * kWheelCircumference
"""meters"""

kAutoFrontwaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoSidewaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoDistanceThreshold = 6 * kMetersPerInch
"""meters"""

kAutoDriveSpeedFactor = 0.5
"""dimensionless"""

kAutoWaitDuration = 1
"""seconds"""

kAutoTargetOffset = Translation2d(2, 0)
"""[meters, meters]"""

kAutoDuration = 15
"""seconds"""

# Target relative drive
kTargetRelativeDriveAnglePGain = 1
kTargetRelativeDriveAngleIGain = 0
kTargetRelativeDriveAngleDGain = 0

kRotationPGain = 0.1
kRotationIGain = 0
kRotationDGain = 0.00

# Drive to Target
kDriveToTargetDistancePGain = 0.5
kDriveToTargetDistanceIGain = 0
kDriveToTargetDistanceDGain = 0

kDriveToTargetAnglePGain = 0.5
kDriveToTargetAngleIGain = 0
kDriveToTargetAngleDGain = 0

kDriveToTargetDistanceTolerance = 10 / kCentimetersPerMeter
"""meters"""

kDriveToTargetLinearVelocityTolerance = 1 / kCentimetersPerMeter / 1
"""meters / second"""

kDriveToTargetAngleTolerance = 5 * kRadiansPerDegree
"""radians"""

kDriveToTargetAngularVelocityTolerance = 5 * kRadiansPerDegree / 1
"""radians / second"""

# Trajectory Following
kTrajectoryPositionPGainAuto = 9
kTrajectoryPositionPGainVision = 5
kTrajectoryPositionIGain = 0
kTrajectoryPositionDGain = 0

kTrajectoryAnglePGain = 7
kTrajectoryAngleIGain = 0
kTrajectoryAngleDGain = 0

kPathFollowingTranslationConstantsAuto = PIDConstants(
    kTrajectoryPositionPGainAuto, kTrajectoryPositionIGain, kTrajectoryPositionDGain
)
kPathFollowingTranslationConstantsVision = PIDConstants(
    kTrajectoryPositionPGainVision, kTrajectoryPositionIGain, kTrajectoryPositionDGain
)
kPathFollowingRotationConstants = PIDConstants(
    kTrajectoryAnglePGain, kTrajectoryAngleIGain, kTrajectoryAngleDGain
)

kPathfindingConstraints = PathConstraints(
    kMaxWheelLinearVelocity / 4,
    kMaxWheelLinearAcceleration / 4,
    kMaxRotationAngularVelocity / 2,
    kMaxRotationAngularAcceleration / 2,
)

# Operator Interface
kXboxJoystickDeadband = 0.1
"""dimensionless"""

kKeyboardJoystickDeadband = 0.0
"""dimensionless"""

kControllerMappingFilename = "ControlScheme.json"

kChassisRotationXAxisName = "chassisXRotation"
kChassisRotationYAxisName = "chassisYRotation"
kChassisForwardsBackwardsAxisName = "chassisForwardsBackwards"
kChassisSideToSideAxisName = "chassisSideToSide"

kFieldRelativeCoordinateModeControlButtonName = "fieldRelativeCoordinateModeControl"
kResetGyroButtonName = "resetGyro"
kTargetRelativeCoordinateModeControlButtonName = "targetRelativeCoordinateModeControl"
kDriveToTargetControlButtonName = "driveToTargetControl"
kXboxTriggerActivationThreshold = 0.5

kTurboSpeedButtonName = "turboSpeed"
kNormalSpeedMultiplier = 0.50  # half full on normal
kTurboSpeedMultiplier = 0.95  # full speed!!!

# Simulation Parameters
kSimulationRotationalInertia = 0.0002
kSimulationRotationalInertiaFlywheel = 0.002
kSimMotorResistance = 0.002
"""[meters, meters, radians]"""

kSimDefaultRobotLocation = Pose2d(0, 0, 0)
kSimDefaultTargetHeight = 8 * kMetersPerFoot + 8 * kMetersPerInch  # 8ft 8in

kSimRobotPoseArrayKey = "SimRobotPoseArray"
kSimRobotVelocityArrayKey = "SimRobotVelocityArray"

"""meters"""

kMotorBaseKey = "motors"

# waypoint setter constraints
kMaxWaypointTranslationalVelocity = kMaxForwardLinearVelocity
kMaxWaypointTranslationalAcceleration = kMaxWaypointTranslationalVelocity * 3

kPossibleWaypoints = []
kWaypointJoystickVariation = 0.1
"""meters"""
kWaypointActiveKey = "waypoint/active"
kWaypointAtTargetKey = "waypoint/atPosition"

kTargetWaypointPoseKey = "waypoint/target"
kTargetWaypointXControllerKey = "waypoint/x"
kTargetWaypointYControllerKey = "waypoint/y"
kTargetWaypointThetaControllerKey = "waypoint/theta"
kWaypointLeftReefKey = "waypoint/leftReef"
kWaypointRightReefKey = "waypoint/rightReef"

# Logging
kSwerveActualStatesKey = "swerve/actual"
kSwerveExpectedStatesKey = "swerve/expected"
kConsoleLog = "log"
kPDHCanID = 1
kPDHPublishKey = "powerDistribution"

kJoystickKeyLogPrefix = "DriverStation"
kFieldSimTargetKey = "SimTargets"
kFieldRelativeTargets = "RelTargets"

# Velocity Dynamic Control
kVelocitySetpoint1ControlKey = "controls/velocity/Setpoint 1"
kVelocitySetpoint2ControlKey = "controls/velocity/Setpoint 2"
kVelocityControlGearRatio = "controls/velocity/ratio"

kVelocityControlCANId = 3
kVelocityControlPGain = 0.001
kVelocityControlIGain = 0
kVelocityControlDGain = 0

kVelocityControlMotorType = DCMotor.falcon500()
kVelocityControlkV = 0.01

# taken from last year, update when cad finished
kIntakeCANID = 25
kIntakeName = "IntakeMotor"
kIntakePGain = 0.8
kIntakeIGain = 0
kIntakeDGain = 0
kIntakeKv = 0.00200  # stolen from shooter :)
kIntakeKs = 0.33329

kPivotCANID = 19
kPivotName = "PivotMotor"
kPivotPGain = 0.9
kPivotIGain = 0
kPivotDGain = 0

kPivotAccel = 200  # rotations / s^2
kPivotVel = 150  # rotations / s

# from CAD
kPivotGearRatio = (5 / 1) * (50 / 16) * (84 / 16)

kIntakeInverted = True
kPivotInverted = False

kPivotEncoderID = 46
kPivotEncoderOffset = 0.23

kIntakeMotorSpeed = 0.25
kIntakeL1MotorSpeed = 0.35
kIntakeL2ThroughL4MotorSpeed = 0.20

# CAD angles, taken from horizontal

kIntakeToArmOffset = 51.944769

kIntakingAngle = Rotation2d.fromDegrees(257)
kMaxPivotAngle = Rotation2d.fromDegrees(260)
kScoreAngle = Rotation2d.fromDegrees(180 - kIntakeToArmOffset - 45.047053)
kKnockAngle = Rotation2d(0)

kIntakeAtPositionKey = "intake/atPosition"
kPivotAngleKey = "intake/pivotAngle"
kIntakeStateKey = "intake/state"
kIntakeCoralKey = "intake/intakingSpeed"
kIntakeL1SpeedKey = "intake/L1Speed"
kIntakeL2ThroughL4SpeedKey = "intake/L2-L4Speed"
kIntakeFudgeCoralKey = "intake/fudgeCoral"
kIntakeFudgeScoreKey = "intake/fudgeScore"
kIntakeFudgeAmount = 1

kIntakeCurrentLimit = (
    CurrentLimitsConfigs()
    .with_stator_current_limit(60)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(60)
    .with_supply_current_limit_enable(True)
)
kPivotCurrentLimit = (
    CurrentLimitsConfigs()
    .with_stator_current_limit(40)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

kIntakeRealZero = (
    -0.25
)  # MAY NEED CHANGING LATER: "I think we just set the starting config, looked at what the encoder said, and offset it by that much" - Landon
kIntakePivotTolerance = 0.1  # radians

# constants copied from last year, update when cad is done
kElevator1CANID = 55
kElevator1Name = "Elevator1Motor"
kElevator1PGain = 0.9
kElevator1IGain = 0
kElevator1DGain = 0
kElevator1Inverted = True

kElevatorMaxAccel = 390  # rotations / s^2
kElevatorMaxVel = 300  # rotations / s

kElevator2CANID = 56
kElevator2Name = "Elevator2Motor"
kElevator2PGain = 0.12
kElevator2IGain = 0
kElevator2DGain = 0
kElevator2Inverted = False

kElevatorTolerance = 0.05

kElevatorCurrentLimit = (
    CurrentLimitsConfigs()
    .with_stator_current_limit(40)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

# from CAD
kMotorPulleyGearRatio = (60 / 18) * (60 / 18)

# 36 teeth 5mm pitch
kPulleyGearPitchDiameter = 36 * 0.005 / math.pi
"""meters"""

kElevatorPositionKey = "elevator/position"
kElevatorStateKey = "elevator/state"
kElevatorAtPositionKey = "elevator/atPosition"
kElevatorFudgeKey = "elevator/fudge"
kElevatorPoseKey = "elevator/pose"
kElevatorManualModeKey = "elevator/manualMode"
kCoralSpaceKey = "elevator/coralSpace"
kElevatorManualIncrement = 0.01

# taken from cad
kL4PositionBeltPosition = 52 * kMetersPerInch
kL3PositionBeltPosition = 30.5 * kMetersPerInch
kL2PositionBeltPosition = 14.5 * kMetersPerInch
kL1PositionBeltPosition = 0.5 * kMetersPerInch
kIntakePositionBeltPosition = 41 * kMetersPerInch
kIntakePositionCoralSpaceBeltPosition = 38.5 * kMetersPerInch
kAlgaeLowBeltPosition = 25 * kMetersPerInch
kAlgaeHighBeltPosition = 31.5 * kMetersPerInch

kElevatorFudgeAmount = 0.5 * kMetersPerInch

kRobotToElevatorTransform = Transform3d(
    0, -5.5 * kMetersPerInch, 11.35 * kMetersPerInch, Rotation3d()
)

kPivotToArmRoot = 3 * kMetersPerInch

# from CAD, pivot to middle of intake shoulders
kArmRootToArmEndTransform = Transform3d(
    20.809180 * kMetersPerInch,
    0,
    -5.543945 * kMetersPerInch,
    Rotation3d(0, (90 + 51.944769) * kRadiansPerDegree, 0),
)
# Climber constants
kClimberCANID = 57
kClimberName = "ClimberMotor"
kClimberPGain = 0.2
kClimberIGain = 0
kClimberDGain = 0

kClimberPositionKey = "climber/position"
kClimberStateKey = "climber/state"
kClimberTargetKey = "climber/target"
kClimberAtPositionKey = "climber/atPosition"
kClimberEndClimbKey = "climber/endClimb"
kClimberManualTargetPositionKey = "climber/manualTargetPosition"

kClimberGearRatio = 100 / 1
kClimberWinchDiameter = 1.2 * kMetersPerInch
kClimberWinchCircumferance = kClimberWinchDiameter * math.pi
kClimberPositionTolerance = 1
kClimberManualIncrement = 20

kClimberCurrentLimit = (
    CurrentLimitsConfigs()
    .with_stator_current_limit(60)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(60)
    .with_supply_current_limit_enable(True)
)

# in rotations
kClimberTuckedPosition = 25
kClimberAtFramePosition = 240
kClimberMiniDeployPosition = (kClimberTuckedPosition + kClimberAtFramePosition) / 2
