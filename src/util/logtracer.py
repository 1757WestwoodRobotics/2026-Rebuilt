from pykit.logger import Logger, RobotController


class LogTracer:
    innerStart: float = 0.0
    outerStart: float = 0.0

    prefix: str = ""

    @classmethod
    def resetOuter(cls, prefix: str) -> None:
        cls.outerStart = RobotController.getFPGATime()
        cls.reset()
        cls.prefix = prefix

    @classmethod
    def reset(cls) -> None:
        cls.innerStart = RobotController.getFPGATime()

    @classmethod
    def record(cls, action: str) -> None:
        now = RobotController.getFPGATime()
        Logger.recordOutput(
            f"LogTracer/{cls.prefix}/{action}MS", (now - cls.innerStart) / 1000.0
        )
        cls.innerStart = now

    @classmethod
    def recordTotal(cls) -> None:
        now = RobotController.getFPGATime()
        Logger.recordOutput(
            f"LogTracer/{cls.prefix}/TotalMS", (now - cls.outerStart) / 1000.0
        )
