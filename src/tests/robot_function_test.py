"""
Integration and crash tests for virtual robot functionality.
Ensures all commands, button bindings, state updates, and autonomous routines
can schedule and execute without raising exceptions or crashing the virtual robot.
"""

import pytest
import commands2
import wpilib.simulation
from pyfrc.test_support.controller import TestController

from robot import Orion


@pytest.mark.filterwarnings("ignore")
def test_all_button_actions_and_commands(control: TestController):
    """
    Runs a virtual robot simulation through teleop, actuating all driver XboxController
    and operator FarmController buttons, executing all default and scheduled commands to
    ensure no runtime crashes occur.
    """
    # pylint: disable=protected-access
    robot_ref: Orion = control._robot
    with control.run_robot():
        # Step timing disabled briefly
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)
        # Enable teleop mode
        control.step_timing(seconds=0.2, autonomous=False, enabled=True)

        container = robot_ref.container

        # 1. Trigger all Driver Xbox Controller buttons & triggers
        driver_hid = container.oi.driverController.getHID()
        xbox_sim = wpilib.simulation.GenericHIDSim(driver_hid)

        for button_num in range(1, 11):
            xbox_sim.setRawButton(button_num, True)
            control.step_timing(seconds=0.1, autonomous=False, enabled=True)
            xbox_sim.setRawButton(button_num, False)
            control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        for axis_num in range(0, 6):
            xbox_sim.setRawAxis(axis_num, 1.0)
            control.step_timing(seconds=0.2, autonomous=False, enabled=True)
            xbox_sim.setRawAxis(axis_num, 0.0)

        xbox_sim.setPOV(180)  # POV Down -> ResetGyro
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        xbox_sim.setPOV(0)  # POV Up -> ResetDrive
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        # 2. Trigger all Operator Farm Controller buttons
        operator_hid = container.oi.operatorController.getHID()
        farm_sim = wpilib.simulation.GenericHIDSim(operator_hid)

        for btn in range(1, 22):
            farm_sim.setRawButton(btn, True)
            control.step_timing(seconds=0.1, autonomous=False, enabled=True)
            farm_sim.setRawButton(btn, False)
            control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        # 3. Schedule all subsystem default commands to verify stability
        subsystems = [
            container.drive,
            container.turret,
            container.indexer,
            container.hood,
            container.flywheel,
            container.intake,
        ]
        for sub in subsystems:
            cmd = sub.getDefaultCommand()
            if cmd is not None:
                cmd.schedule()
                control.step_timing(seconds=0.1, autonomous=False, enabled=True)


@pytest.mark.filterwarnings("ignore")
def test_all_autonomous_commands_executes_without_crash(control: TestController):
    """
    Cycles through every registered autonomous routine in the LoggedDashboardChooser,
    enables autonomous mode, and runs each auto for 1 second in simulation.
    """
    # pylint: disable=protected-access
    robot_ref: Orion = control._robot
    with control.run_robot():
        container = robot_ref.container
        chooser = container.chooser
        options = list(chooser.options.keys())

        for option in options:
            chooser.selectedValue = option
            # Step timing in autonomous enabled mode
            control.step_timing(seconds=1.0, autonomous=True, enabled=True)
            commands2.CommandScheduler.getInstance().cancelAll()
            control.step_timing(seconds=0.1, autonomous=False, enabled=False)
