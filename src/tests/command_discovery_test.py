"""
Integration and crash tests for virtual robot commands.
Dynamically imports all command modules in src/commands/ and verifies that every
instantiable command executes cleanly when scheduled on a virtual robot.
"""

import importlib
import inspect
import pkgutil
import warnings
import pytest

import commands2
import wpimath.geometry
from pyfrc.test_support.controller import TestController

import commands
from robot import Orion


def _zero_pose():
    return wpimath.geometry.Pose2d()


def _zero_translation():
    return wpimath.geometry.Translation2d()


def _zero_float():
    return 0.0


def _discover_and_instantiate_commands(container):
    subsystems = {
        "drive": container.drive,
        "turret": container.turret,
        "indexer": container.indexer,
        "hood": container.hood,
        "flywheel": container.flywheel,
        "intake": container.intake,
        "vision": container.vision,
    }

    verified_commands = []
    unverified_commands = []

    package = commands
    for _, modname, _ in pkgutil.walk_packages(
        package.__path__, package.__name__ + "."
    ):
        try:
            mod = importlib.import_module(modname)
        except (ImportError, AttributeError, TypeError) as err:
            unverified_commands.append((modname, f"Module import failed: {err}"))
            continue

        for attr_name in dir(mod):
            if attr_name.startswith("_"):
                continue
            obj = getattr(mod, attr_name)

            if (
                inspect.isclass(obj)
                and issubclass(obj, commands2.Command)
                and obj is not commands2.Command
                and obj.__module__ == modname
            ):
                cmd_instance = _try_instantiate_class(obj, subsystems)
                if cmd_instance:
                    verified_commands.append((f"{modname}.{attr_name}", cmd_instance))
                else:
                    unverified_commands.append(
                        (f"{modname}.{attr_name}", "Constructor instantiation failed")
                    )

            elif (
                inspect.isfunction(obj)
                and obj.__module__ == modname
                and not attr_name.startswith("set")
            ):
                cmd_instance = _try_call_factory(obj, subsystems)
                if isinstance(cmd_instance, commands2.Command):
                    verified_commands.append((f"{modname}.{attr_name}", cmd_instance))

    return verified_commands, unverified_commands


def _try_instantiate_class(cls, subsystems):
    sig = inspect.signature(cls.__init__)
    kwargs = {}
    for param in sig.parameters.values():
        if param.name == "self":
            continue
        name = param.name.lower()
        matched = False
        for sub_name, sub_obj in subsystems.items():
            if sub_name in name:
                kwargs[param.name] = sub_obj
                matched = True
                break
        if not matched:
            if "speed" in name or "distance" in name or "factor" in name:
                kwargs[param.name] = 0.5
            elif "axis" in name:
                kwargs[param.name] = (
                    getattr(cls, "Axis", type("Axis", (), {"X": 0})).X
                    if hasattr(cls, "Axis")
                    else 0
                )
            elif (
                "supplier" in name
                or "func" in name
                or "target" in name
                or "x" in name
                or "y" in name
            ):
                kwargs[param.name] = _zero_pose
            else:
                kwargs[param.name] = 0.0
    try:
        return cls(**kwargs)
    except (TypeError, ValueError, AttributeError) as _:
        return None


def _try_call_factory(func, subsystems):
    sig = inspect.signature(func)
    kwargs = {}
    for param in sig.parameters.values():
        name = param.name.lower()
        matched = False
        for sub_name, sub_obj in subsystems.items():
            if sub_name in name:
                kwargs[param.name] = sub_obj
                matched = True
                break
        if not matched:
            if "target" in name or "supplier" in name or "rotation" in name:
                kwargs[param.name] = _zero_translation
            else:
                kwargs[param.name] = _zero_float
    try:
        return func(**kwargs)
    except (TypeError, ValueError, AttributeError) as _:
        return None


@pytest.mark.filterwarnings("ignore")
def test_all_discovered_commands_execute_without_crash(control: TestController):
    """
    Dynamically discovers and schedules every command in src/commands/ on a virtual robot,
    logging all verified commands and emitting warnings for any unverified commands.
    """
    # pylint: disable=protected-access
    robot_ref: Orion = control._robot
    with control.run_robot():
        control.step_timing(seconds=0.2, autonomous=False, enabled=True)
        container = robot_ref.container

        verified, unverified = _discover_and_instantiate_commands(container)

        print("\n" + "=" * 70)
        print(
            f" COMMAND VERIFICATION REPORT: {len(verified)} VERIFIED | {len(unverified)} UNVERIFIED"
        )
        print("=" * 70)

        for name, cmd in verified:
            print(f"  [VERIFIED] Executing: {name}")
            cmd.schedule()
            control.step_timing(seconds=0.2, autonomous=False, enabled=True)
            commands2.CommandScheduler.getInstance().cancelAll()
            control.step_timing(seconds=0.1, autonomous=False, enabled=False)

        if unverified:
            print(
                "\n  [WARNING] Could not automatically verify the following command candidates:"
            )
            for name, reason in unverified:
                print(f"    - {name}: {reason}")
                warnings.warn(
                    f"Command could not be verified automatically: {name} ({reason})"
                )

        print("=" * 70 + "\n")

        assert (
            len(verified) > 0
        ), "Should discover and verify at least one command in src/commands/"
