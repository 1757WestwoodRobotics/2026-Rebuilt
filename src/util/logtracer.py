import typing
from pykit.logger import Logger, RobotController


class TraceScope:
    """Stack frame holding timing state for a nested trace scope."""

    __slots__ = ("prefix", "outer_start", "inner_start", "path")

    def __init__(self, prefix: str, parent_path: str = ""):
        self.prefix = prefix
        self.path = f"{parent_path}/{prefix}" if parent_path else prefix
        now = RobotController.getFPGATime() if LogTracer.enabled else 0.0
        self.outer_start = now
        self.inner_start = now


class LogTracerSection:
    """Context manager for nested profiling."""

    __slots__ = ("name", "scope")

    def __init__(self, name: str):
        self.name = name
        self.scope: typing.Optional[TraceScope] = None

    def __enter__(self):
        if LogTracer.enabled:
            parent_path = LogTracer.currentPath()
            self.scope = TraceScope(self.name, parent_path)
            LogTracer.stack.append(self.scope)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if LogTracer.enabled and self.scope:
            try:
                now = RobotController.getFPGATime()
                Logger.recordOutput(
                    f"LogTracer/{self.scope.path}/TotalMS",
                    (now - self.scope.outer_start) / 1000.0,
                )
            finally:
                if LogTracer.stack and LogTracer.stack[-1] is self.scope:
                    LogTracer.stack.pop()


class LogTracer:
    """
    Execution profiler supporting infinite recursive/nested trace scopes.
    Uses a frame stack to manage nested functions and sub-block timings accurately.
    """

    enabled: bool = True
    stack: list[TraceScope] = []


    @classmethod
    def resetCycle(cls) -> None:
        """Reset the frame stack at the start of each robot periodic loop cycle."""
        cls.stack.clear()

    @classmethod
    def setEnabled(cls, enabled: bool) -> None:
        cls.enabled = enabled


    @classmethod
    def currentPath(cls) -> str:
        return cls.stack[-1].path if cls.stack else ""

    @classmethod
    def resetOuter(cls, prefix: str) -> None:
        if not cls.enabled:
            return
        # Start or reset top-level scope frame
        if cls.stack:
            # Re-initialize top-level frame if existing
            top = cls.stack[-1]
            top.prefix = prefix
            top.path = prefix
            now = RobotController.getFPGATime()
            top.outer_start = now
            top.inner_start = now
        else:
            cls.stack.append(TraceScope(prefix))

    @classmethod
    def reset(cls) -> None:
        if not cls.enabled or not cls.stack:
            return
        cls.stack[-1].inner_start = RobotController.getFPGATime()

    @classmethod
    def record(cls, action: str) -> None:
        if not cls.enabled or not cls.stack:
            return
        now = RobotController.getFPGATime()
        scope = cls.stack[-1]
        Logger.recordOutput(
            f"LogTracer/{scope.path}/{action}MS",
            (now - scope.inner_start) / 1000.0,
        )
        scope.inner_start = now

    @classmethod
    def recordTotal(cls) -> None:
        if not cls.enabled or not cls.stack:
            return
        now = RobotController.getFPGATime()
        scope = cls.stack[-1]
        Logger.recordOutput(
            f"LogTracer/{scope.path}/TotalMS",
            (now - scope.outer_start) / 1000.0,
        )

    @classmethod
    def trace(cls, section_name: str) -> typing.ContextManager:
        """Usage: with LogTracer.trace("SubsystemStep"): ..."""
        return LogTracerSection(section_name)
