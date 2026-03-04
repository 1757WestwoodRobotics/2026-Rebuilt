from commands2 import Command, Subsystem, cmd
from commands2.sysid import SysIdRoutine
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib.sysid import State
from wpimath.geometry import Rotation2d

from subsystems.hood.hoodsubsystemio import HoodSubsystemIO
from util.convenientmath import clamp
from util.logtracer import LogTracer

from constants.hood import (
    kHoodMinAngle,
    kHoodMaxAngle,
    kHoodTolerance,
    kHoodMaxVelocity,
    kHoodMaxAcceleration,
)


@autologgable_output
class HoodSubsystem(Subsystem):
    def __init__(self, io: HoodSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.io = io
        self.inputs = HoodSubsystemIO.HoodSubsystemIOInputs()

        self.isClosedLoop = True
        self.hoodGoal = Rotation2d()

    def periodic(self) -> None:
        LogTracer.resetOuter("HoodSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Hood", self.inputs)
        LogTracer.record("UpdateInputs")

        if self.isClosedLoop:
            self.io.set_hood_position(
                clamp(
                    self.hoodGoal.radians(),
                    kHoodMinAngle.radians(),
                    kHoodMaxAngle.radians(),
                )
            )

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Hood/goal", self.hoodGoal)
        Logger.recordOutput("Hood/ClosedLoop", self.isClosedLoop)
        LogTracer.recordTotal()

    def setClosedLoop(self, closedLoop: bool) -> None:
        """
        Sets whether the hood is in closed loop control or not
        """
        self.isClosedLoop = closedLoop

    def setHoodRawVolts(self, volts: float) -> None:
        Logger.recordOutput("Hood/RawVolts", volts)
        self.io.set_hood_volts(volts)

    def setHoodGoal(self, goal: Rotation2d) -> None:
        """
        Sets the hood goal position in radians
        """
        self.hoodGoal = goal

    def isAtGoal(self, goal: Rotation2d) -> bool:
        """
        Returns whether the hood is at the given goal position
        """
        return abs(self.inputs.hoodPosition - goal.radians()) < kHoodTolerance.radians()

    def isAtMin(self) -> bool:
        """
        Returns whether the hood is at the minimum position
        """
        return self.isAtGoal(kHoodMinAngle)

    def isAtMax(self) -> bool:
        """
        Returns whether the hood is at the maximum position
        """
        return self.isAtGoal(kHoodMaxAngle)

    def isBeyondGoal(self) -> bool:
        """
        Returns whether the hood is beyond target position
        """
        return (
            self.inputs.hoodPosition
            > self.hoodGoal.radians() + kHoodTolerance.radians()
        ) or (
            self.inputs.hoodPosition
            < self.hoodGoal.radians() - kHoodTolerance.radians()
        )

    def getMaxVelocity(self) -> Rotation2d:
        """
        Returns the maximum velocity the hood can achieve.
        """
        return kHoodMaxVelocity

    def getMaxAcceleration(self) -> Rotation2d:
        """
        Returns the maximum acceleration the hood can achieve.
        """
        return kHoodMaxAcceleration

    @autolog_output(key="Hood/at target")
    def atTarget(self) -> bool:
        """
        Returns whether the hood is at the target position
        """
        return self.isAtGoal(self.hoodGoal)

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
            Logger.recordOutput("Hood/SysID State", loggedStateStr)

        charactarizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(0.5, 6, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.set_hood_volts,
                (lambda _: None),
                subsystem,
            ),
        )

        return cmd.sequence(
            cmd.runOnce(lambda: self.setClosedLoop(False), self),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(
                lambda: self.isAtGoal(kHoodMaxAngle)
            ),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(
                lambda: self.isAtGoal(kHoodMinAngle)
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(
                lambda: self.isAtGoal(kHoodMaxAngle)
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(
                lambda: self.isAtGoal(kHoodMinAngle)
            ),
            cmd.runOnce(lambda: self.setClosedLoop(True), self),
        )
