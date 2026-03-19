#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

from phoenix6.unmanaged import feed_enable
from pykit.logger import Logger
from wpilib import RobotController, Timer
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Transform2d,
    Translation3d,
)
from wpimath.kinematics import ChassisSpeeds
from pyfrc.physics.core import PhysicsInterface
from robot import Orion
from robotstate import RobotState
from subsystems.drive.swervemoduleiotalonfx import SwerveModuleIOCTRE
from subsystems.drive.drivesubsystem import DriveSubsystem
from subsystems.hood.hoodsubsystem import HoodSubsystem
from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem
from subsystems.indexer.indexersubsystem import IndexerSubsystem, IndexerSubsystemGoal

from constants.turret import kTurretLocation
from constants.sim import (
    kSimDefaultRobotLocation,
    kFuelSimEffectiveVelocity,
    kFuelSimEffectiveExitAngle,
    kSimGravity,
    kSimSecondsPerBall,
)


class SwerveDriveSim:
    def __init__(self, driveSubsystem: DriveSubsystem) -> None:
        self.driveSubsystem = driveSubsystem
        self.pose = kSimDefaultRobotLocation

    def resetPose(self, pose) -> None:
        self.pose = pose

    def getSimPose(self) -> Pose2d:
        return self.pose

    def getPose(self) -> Pose2d:
        return self.pose

    def getHeading(self) -> Rotation2d:
        return self.pose.rotation()

    def update(self, tm_diff: float, _robotVoltage: float) -> None:
        deltaT = tm_diff

        chassisSpeed = self.driveSubsystem.getRobotRelativeSpeeds()
        deltaHeading = chassisSpeed.omega * deltaT
        deltaX = chassisSpeed.vx * deltaT
        deltaY = chassisSpeed.vy * deltaT

        deltaTrans = Transform2d(deltaX, deltaY, deltaHeading)

        newPose = self.pose + deltaTrans
        self.pose = newPose


class FuelSim:
    class SimulatedFuel:
        position: Translation3d
        velocity: Translation3d

        def __init__(self, position: Translation3d, velocity: Translation3d) -> None:
            self.position = position
            self.velocity = velocity

        def update(self, tm_diff: float) -> bool:
            """
            Updates the position and velocity of the fuel based on its current velocity
            and gravity. Returns True if the fuel is still in the air,
            and False if it has hit the ground.
            """
            self.position += self.velocity * tm_diff
            self.velocity += Translation3d(0, 0, -kSimGravity) * tm_diff  # gravity

            return self.position.z > 0

    def __init__(
        self,
        indexer: IndexerSubsystem,
        flywheel: FlywheelSubsystem,
        hood: HoodSubsystem,
    ) -> None:
        self.indexer = indexer
        self.flywheel = flywheel
        self.hood = hood
        self.fuel: list[FuelSim.SimulatedFuel] = []
        self.lastFuelTime = 0.0

    def addFuel(self) -> None:
        robotPose = RobotState.getSimPose()
        robotVelocity = RobotState.robotFieldVelocity

        exitVelocity = kFuelSimEffectiveVelocity(self.flywheel.inputs.flywheelSpeed)
        exitAngle = kFuelSimEffectiveExitAngle(self.hood.inputs.hoodPosition)

        turretRobotFrameVel = (
            Translation3d(
                -kTurretLocation.rotation().toRotation2d().sin(),
                kTurretLocation.rotation().toRotation2d().cos(),
                0,
            )
            * robotVelocity.omega
            * kTurretLocation.translation().toTranslation2d().norm()
        )
        turretFieldRefVel = turretRobotFrameVel.rotateBy(
            Rotation3d(0, 0, robotPose.rotation().radians())
        )
        turretVelocity = robotVelocity + ChassisSpeeds(
            turretFieldRefVel.x, turretFieldRefVel.y, 0
        )

        turretExitPosition = RobotState.getSimTurretPose()
        turretExitVelocity = Translation3d(
            exitVelocity * exitAngle.cos(), 0, exitVelocity * exitAngle.sin()
        ).rotateBy(turretExitPosition.rotation()) + Translation3d(
            turretVelocity.vx, turretVelocity.vy, 0
        )

        self.fuel.append(
            FuelSim.SimulatedFuel(
                turretExitPosition.translation(),
                turretExitVelocity,
            )
        )

    def update(self, tm_diff: float) -> None:
        """
        Updates the position of all fuel in the air, and removes any fuel that has hit the ground.
        """
        self.fuel = [f for f in self.fuel if f.update(tm_diff)]

        Logger.recordOutput(
            "FuelSim/Fuel", [Pose3d(f.position, Rotation3d()) for f in self.fuel]
        )

        if (
            self.indexer.subsystemGoal == IndexerSubsystemGoal.KICK
            and Timer.getFPGATimestamp() - self.lastFuelTime > kSimSecondsPerBall
        ):
            self.addFuel()
            self.lastFuelTime = Timer.getFPGATimestamp()


class PhysicsEngine:
    """
    Simulates a drivetrain
    """

    # pylint: disable-next=unused-argument
    def __init__(self, physics_controller: PhysicsInterface, robot: Orion):
        assert robot.container is not None
        self.physics_controller = physics_controller
        self.bot = robot

        driveSubsystem: DriveSubsystem = robot.container.drive
        indexer: IndexerSubsystem = robot.container.indexer
        flywheel: FlywheelSubsystem = robot.container.flywheel
        hood: HoodSubsystem = robot.container.hood

        if not isinstance(driveSubsystem.frontLeftModule.io, SwerveModuleIOCTRE):
            # do not simulation
            self.doSim = False
            print("[Physics] WARNING: Not simulating")
            return

        self.doSim = True
        print("[Physics] beginning simulation")

        self.driveSim = SwerveDriveSim(driveSubsystem)
        self.fuelSim = FuelSim(indexer, flywheel, hood)

        RobotState.registerSimPoseResetConsumer(self.driveSim.resetPose)
        RobotState.registerSimPoseReceiverConsumer(self.driveSim.getSimPose)

        self.sim_initialized = False

    # pylint: disable-next=unused-argument
    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        if not self.doSim:
            return
        feed_enable(tm_diff)

        if not self.sim_initialized:
            self.sim_initialized = True
            # self.physics_controller.field, is not set until simulation_init

        # Simulate the drivetrain
        voltage = RobotController.getInputVoltage()

        self.driveSim.update(tm_diff, voltage)
        self.fuelSim.update(tm_diff)

        simRobotPose = self.driveSim.getPose()
        self.physics_controller.field.setRobotPose(simRobotPose)

        Logger.recordOutput("Sim/RobotPose", simRobotPose)
