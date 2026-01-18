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
        self.inputs = TurretSubsystemIO.TurretSubsystemIOInputs()
        """grabs the inputs from TurretSubsystemIO"""
        self.isClosedLoop = True
        self.turretGoal = Rotation2d()

    def periodic(self) -> None:
        LogTracer.resetOuter("TurretSubsystem periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Turret", self.inputs)
        LogTracer.record("UpdateInputs")

        if self.isClosedLoop:
            self.io.set_turret_angle(
                clampRotation(
                    self.turretGoal, kTurretMinAngle, kTurretMaxAngle
                ).radians()
            )
        """sets max and min angle for turret"""
        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Turret/goal", self.turretGoal)
        Logger.recordOutput("Turret/ClosedLoop", self.isClosedLoop)
        LogTracer.recordTotal()

    def setClosedLoop(self, closedLoop: bool) -> None:
        """
        Sets whether the turret is in closed loop control or not
        """
        self.isClosedLoop = closedLoop

    def setTurretRawVolts(self, volts: float) -> None:
        Logger.recordOutput("Turret/RawVolts", volts)
        self.io.set_turret_volts(volts)

    def setTurretGoal(self, goal: Rotation2d) -> None:
        """
        Sets the turret goal position
        """
        self.turretGoal = goal

    def isAtGoal(self, goal: Rotation2d) -> bool:
        return (
            abs(self.inputs.turretPosition - goal.radians())
            < kTurretTolerance.radians()
        )

    """returns whether or not the angle is less than the goal tolerance angle"""

    @autolog_output(key="Turret/at target")
    def atTarget(self) -> bool:
        return self.isAtGoal(self.turretGoal)

    def sysIdRoutine(self, subsystem: Subsystem) -> Command:
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
        """defines logging state"""
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


"""runs a sequence of turning the turret to find the goal once"""
