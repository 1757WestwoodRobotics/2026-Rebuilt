from commands2 import Command, Subsystem, cmd
from commands2.sysid import SysIdRoutine
from pykit.autolog import autolog_output
from pykit.logger import Logger
from wpilib.sysid import State
from wpimath.geometry import Rotation2d

from subsystems.turret.turretsubsystemio import TurretSubsystemIO
from util.convenientmath import clamp, clampRotation
from util.logtracer import LogTracer

from constants.turret import kTurretMinAngle, kTurretMaxAngle, kTurretTolerance


class TurretSubsystem(Subsystem):
    def __init__(self, io: TurretSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.io = io
        self.inputs = (
            TurretSubsystemIO.TurretSubsystemIOInputs()
        )  # initialize IO inputs
        self.isClosedLoop = True
        self.turretGoal = Rotation2d()

    def periodic(self) -> None:
        """Run ongoing subsystem periodic process."""
        LogTracer.resetOuter("TurretSubsystem periodic")
        self.io.updateInputs(self.inputs)  # update state of motor
        Logger.processInputs("Turret", self.inputs)
        LogTracer.record("UpdateInputs")

        if self.isClosedLoop:
            self.io.set_turret_angle(
                clampRotation(
                    self.turretGoal, kTurretMinAngle, kTurretMaxAngle
                )  # if isClosedLoop, move the motor to the turretGoal angle (if in allowable range)
            )
        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Turret/goal", self.turretGoal)
        Logger.recordOutput("Turret/ClosedLoop", self.isClosedLoop)
        LogTracer.recordTotal()

    def setClosedLoop(self, closedLoop: bool) -> None:
        self.isClosedLoop = closedLoop

    def setTurretRawVolts(self, volts: float) -> None:
        """Apply a specific amount of volts to motor."""
        Logger.recordOutput("Turret/RawVolts", volts)
        self.io.set_turret_volts(
            volts
        )  # call the appropriate overridden method with the IO class

    def setTurretGoal(self, goal: Rotation2d) -> None:
        self.turretGoal = goal

    def isAtGoal(self, goal: Rotation2d) -> bool:
        """Determine whether turret is at goal (within a small tolerance)."""
        return (
            abs(self.inputs.turretPosition.radians() - goal.radians())
            < kTurretTolerance.radians()
        )

    @autolog_output(key="Turret/at target")
    def atTarget(self) -> bool:
        return self.isAtGoal(self.turretGoal)

    def sysIdRoutine(self, subsystem: Subsystem) -> Command:
        """Model the behavior of the system (for better control) by sweeping through the max and min angles."""

        def logState(state: State) -> None:
            loggedStateStr = ""
            match state:
                case State.kQuasistaticForward:
                    loggedStateStr = "quasistatic-forward"
                case State.kQuasistaticReverse:
                    loggedStateStr = "quasistatic-reverse"
                case State.kDynamicForward:
                    loggedStateStr = "dynamic-forward"
                case State.kDynamicReverse:
                    loggedStateStr = "dynamic-reverse"
                case State.kNone:
                    loggedStateStr = "none"
            Logger.recordOutput("Shooter/SysID State", loggedStateStr)

        charactarizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(0.5, 6, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.set_turret_volts,
                (lambda _: None),
                subsystem,
            ),
        )

        return cmd.sequence(
            cmd.runOnce(lambda: self.setClosedLoop(False), self),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(
                lambda: self.isAtGoal(kTurretMaxAngle)
            ),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(
                lambda: self.isAtGoal(kTurretMinAngle)
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(
                lambda: self.isAtGoal(kTurretMaxAngle)
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(
                lambda: self.isAtGoal(kTurretMinAngle)
            ),
            cmd.runOnce(lambda: self.setClosedLoop(True), self),
        )
