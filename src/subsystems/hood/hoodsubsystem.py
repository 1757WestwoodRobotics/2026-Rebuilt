from commands2 import Command, Subsystem, cmd
from commands2.sysid import SysIdRoutine
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib.sysid import State
from wpimath.geometry import Rotation2d

from robotstate import RobotState
from subsystems.hood.hoodsubsystemio import HoodSubsystemIO
from util.convenientmath import clamp
from util.logtracer import LogTracer

from constants.hood import (
    kHoodMinAngle,
    kHoodMaxAngle,
    kHoodStartAngle,
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
        self.io.set_hood_position(kHoodStartAngle)
        self.inputs = HoodSubsystemIO.HoodSubsystemIOInputs()

        self.isClosedLoop = True
        self.hoodGoal = Rotation2d()
        self.hoodFudge = Rotation2d()

    def periodic(self) -> None:
        LogTracer.resetOuter("HoodSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Hood", self.inputs)
        LogTracer.record("UpdateInputs")

        if self.isClosedLoop:
            self.io.set_hood_target(
                Rotation2d(
                    clamp(
                        self.hoodGoal.radians() + self.hoodFudge.radians(),
                        kHoodMinAngle.radians(),
                        kHoodMaxAngle.radians(),
                    )
                )
            )

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Hood/goal", self.hoodGoal)
        Logger.recordOutput("Hood/ClosedLoop", self.isClosedLoop)
        RobotState.hoodAtAngle = self.atTarget()
        LogTracer.recordTotal()

    def bumpAngle(self, bumpAmount: Rotation2d) -> None:
        self.hoodFudge += bumpAmount

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
        return (
            abs(self.inputs.hoodPosition.radians() - goal.radians())
            < kHoodTolerance.radians()
        )

    @autolog_output(key="Hood/atMin")
    def isAtMin(self) -> bool:
        """
        Returns whether the hood is at or beyond the minimum position
        """
        return (
            self.inputs.hoodPosition.radians()
            <= kHoodMinAngle.radians() + kHoodTolerance.radians()
        )

    @autolog_output(key="Hood/atMax")
    def isAtMax(self) -> bool:
        """
        Returns whether the hood is at or beyond the maximum position
        """
        return (
            self.inputs.hoodPosition.radians()
            >= kHoodMaxAngle.radians() - kHoodTolerance.radians()
        )

    def isBeyondGoal(self) -> bool:
        """
        Returns whether the hood is beyond target position
        """
        return (
            self.inputs.hoodPosition.radians()
            > self.hoodGoal.radians() + kHoodTolerance.radians()
        ) or (
            self.inputs.hoodPosition.radians()
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

        characterizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(0.1, 1, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.set_hood_volts,
                (lambda _: None),
                subsystem,
            ),
        )

        return cmd.sequence(
            cmd.runOnce(lambda: self.setClosedLoop(False), self),
            characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(
                self.isAtMax
            ),
            characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(
                self.isAtMin
            ),
            characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(
                self.isAtMax
            ),
            characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(
                self.isAtMin
            ),
            cmd.runOnce(lambda: self.setClosedLoop(True), self),
        )
