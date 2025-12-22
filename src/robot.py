#!/usr/bin/env python3

import os
import typing

from commands2.commandscheduler import CommandScheduler
from pathplannerlib.commands import PathPlannerLogging
import wpilib
import commands2
from phoenix6.signal_logger import SignalLogger

from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger

import constants
from robotcontainer import RobotContainer
from util.logtracer import LogTracer
from util.phoenixutil import PhoenixUtil


class MentorBot(LoggedRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def __init__(self):
        super().__init__()
        Logger.recordMetadata("Robot", type(self).__name__)
        match constants.kRobotMode:
            case constants.RobotModes.REAL:
                deploy_config = wpilib.deployinfo.getDeployData()
                if deploy_config is not None:
                    Logger.recordMetadata(
                        "Deploy Host", deploy_config.get("deploy-host", "")
                    )
                    Logger.recordMetadata(
                        "Deploy User", deploy_config.get("deploy-user", "")
                    )
                    Logger.recordMetadata(
                        "Deploy Date", deploy_config.get("deploy-date", "")
                    )
                    Logger.recordMetadata(
                        "Code Path", deploy_config.get("code-path", "")
                    )
                    Logger.recordMetadata("Git Hash", deploy_config.get("git-hash", ""))
                    Logger.recordMetadata(
                        "Git Branch", deploy_config.get("git-branch", "")
                    )
                    Logger.recordMetadata(
                        "Git Description", deploy_config.get("git-desc", "")
                    )
                Logger.addDataReciever(NT4Publisher(True))
                Logger.addDataReciever(WPILOGWriter())
            case constants.RobotModes.SIMULATION:
                Logger.addDataReciever(WPILOGWriter())
                Logger.addDataReciever(NT4Publisher(True))
            case constants.RobotModes.REPLAY:
                self.useTiming = (
                    False  # Disable timing in replay mode, run as fast as possible
                )
                log_path = os.environ["LOG_PATH"]
                log_path = os.path.abspath(log_path)
                print(f"Starting log from {log_path}")
                Logger.setReplaySource(WPILOGReader(log_path))
                Logger.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))
        Logger.start()
        self.container = RobotContainer()

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        SignalLogger.enable_auto_logging(False)
        wpilib.LiveWindow.disableAllTelemetry()
        CommandScheduler.getInstance().setPeriod(
            1
        )  # 1s period for command scheduler watchdog

        commandCount: dict[str, int] = {}

        def logCommandFunction(command: commands2.Command, active: bool) -> None:
            name = command.getName()
            count = commandCount.get(name, 0) + (1 if active else -1)
            commandCount[name] = count
            Logger.recordOutput(f"Commands/{name}", count > 0)

        CommandScheduler.getInstance().onCommandInitialize(
            lambda c: logCommandFunction(c, True)
        )
        CommandScheduler.getInstance().onCommandFinish(
            lambda c: logCommandFunction(c, False)
        )
        CommandScheduler.getInstance().onCommandInterrupt(
            lambda c: logCommandFunction(c, False)
        )

        PathPlannerLogging.setLogCurrentPoseCallback(
            lambda pose: Logger.recordOutput("PathPlanner/CurrentPose", pose)
        )
        PathPlannerLogging.setLogTargetPoseCallback(
            lambda pose: Logger.recordOutput("PathPlanner/TargetPose", pose)
        )
        PathPlannerLogging.setLogActivePathCallback(
            lambda poses: Logger.recordOutput("PathPlanner/CurrentPath", poses)
        )

    def robotPeriodic(self) -> None:
        LogTracer.resetOuter("RobotPeriodic")
        PhoenixUtil.updateSignals()
        LogTracer.record("PhoenixUpdate")
        self.container.robotPeriodic()
        LogTracer.record("ContainerPeriodic")
        commands2.CommandScheduler.getInstance().run()
        LogTracer.record("CommandsPeriodic")
        LogTracer.recordTotal()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically when in teleop"""

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MentorBot)
