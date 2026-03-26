"""
Fire Control Solver for Shoot-On-The-Move (SOTM)

Based on the open-source fire control system by Team 5962 (perSEVERE):
https://github.com/eeveemara/frc-fire-control
https://www.chiefdelphi.com/t/open-source-shoot-on-the-move-sotm-solver-ball-physics-sim-3-java-files-drop-in/516109

Improvements over the previous simple fixed-point iteration:
1. Drag-compensated drift (exponential decay instead of linear v*t)
2. Warm start from previous cycle's TOF for faster convergence
3. Latency compensation (predicts pose forward by pipeline + mechanism delay)
4. Confidence scoring (0-100 weighted geometric mean)
5. Newton-Raphson iteration with analytical derivatives
"""

import math
from dataclasses import dataclass, field
from typing import Callable

from pykit.logger import Logger
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds


@dataclass
class FireControlConfig:
    """Configuration for the fire control solver. All times in seconds, distances in meters."""

    # Newton solver parameters
    max_iterations: int = 10
    convergence_tolerance: float = (
        0.001  # seconds — stop iterating when TOF changes less than this
    )

    # TOF bounds (prevents runaway solver)
    tof_min: float = 0.05
    tof_max: float = 3.0

    # Drag compensation coefficient (1/s). Models exponential decay of the ball's
    # inherited horizontal velocity due to air resistance. 0 disables drag compensation.
    # Typical value: 0.20-0.30 for a foam ball.
    sotm_drag_coeff: float = 0.24

    # Speed thresholds (m/s)
    min_sotm_speed: float = 0.1  # below this, use static aiming (no compensation)

    # Latency compensation (seconds)
    # Vision pipeline delay: time from camera frame capture to pose estimate availability
    vision_latency: float = 0.030
    # Mechanism latency: time for turret/hood/flywheel to respond to a new setpoint
    mechanism_latency: float = 0.020

    # Confidence scoring weights (weighted geometric mean — one zero kills the score)
    w_convergence: float = 1.0
    w_velocity_stability: float = 0.8
    w_heading_accuracy: float = 1.5
    w_distance_in_range: float = 0.5

    # Scoring range (meters) — confidence degrades near boundaries
    min_scoring_distance: float = 1.0
    max_scoring_distance: float = 6.0

    # Heading tolerance for confidence scoring
    heading_max_error_rad: float = math.pi / 12  # 15 degrees base tolerance


@dataclass
class FireControlResult:
    """Output of the fire control solver."""

    effective_location: Translation2d = field(default_factory=Translation2d)
    effective_distance: float = 0.0
    time_of_flight: float = 0.0
    confidence: float = 0.0  # 0-100
    iterations_used: int = 0
    is_static: bool = True  # true if robot speed was below threshold
    converged: bool = False


class FireControlSolver:
    """
    Newton-Raphson SOTM solver with drag compensation, warm start, and confidence scoring.

    Call solve() once per robot periodic cycle. The solver maintains state across cycles
    for warm-starting the Newton iteration.
    """

    def __init__(
        self,
        config: FireControlConfig,
        shot_time_map: Callable[[float], float],
        feed_time_map: Callable[[float], float],
    ) -> None:
        self.config = config
        self.shot_time_map = shot_time_map
        self.feed_time_map = feed_time_map

        # Warm start state
        self._previous_tof: float = 0.0
        self._previous_speed: float = 0.0
        self._has_warm_start: bool = False

    def reset_warm_start(self) -> None:
        """Call after a pose reset or mode change to prevent stale warm start."""
        self._has_warm_start = False
        self._previous_tof = 0.0
        self._previous_speed = 0.0

    def _drag_compensated_drift(self, tof: float) -> float:
        """
        Compute the effective drift time accounting for air resistance.

        A ball launched from a moving robot inherits the robot's velocity, but air
        resistance decays that inherited velocity exponentially. Instead of linear
        drift = v * t, the actual drift is v * (1 - e^(-c*t)) / c.

        At c=0.24 and t=0.8s, this gives ~0.70s effective drift instead of 0.80s.
        The ball drifts ~12% less than the linear model predicts, meaning the simple
        model over-compensates at longer flight times.
        """
        c = self.config.sotm_drag_coeff
        if c < 1e-6:
            return tof  # no drag, linear drift
        return (1.0 - math.exp(-c * tof)) / c

    def _get_tof_for_distance(self, distance: float, is_shooting: bool) -> float:
        """Look up TOF from the appropriate interpolation map."""
        if is_shooting:
            return float(self.shot_time_map(distance))
        else:
            return float(self.feed_time_map(distance))

    def _compensate_latency(
        self,
        robot_pose: Pose2d,
        velocity: ChassisSpeeds,
    ) -> Pose2d:
        """
        Predict the robot pose forward by the total system latency.

        The robot has moved since the vision frame was captured. We predict forward
        by vision_latency + mechanism_latency using first-order kinematics.
        """
        dt = self.config.vision_latency + self.config.mechanism_latency
        if dt < 1e-6:
            return robot_pose

        predicted_x = robot_pose.X() + velocity.vx * dt
        predicted_y = robot_pose.Y() + velocity.vy * dt
        predicted_heading = robot_pose.rotation().radians() + velocity.omega * dt

        return Pose2d(predicted_x, predicted_y, predicted_heading)

    def _compute_confidence(
        self,
        iterations_used: int,
        converged: bool,
        robot_speed: float,
        effective_distance: float,
        heading_error_rad: float,
    ) -> float:
        """
        Compute a 0-100 confidence score using weighted geometric mean.

        Five components (any zero kills the entire score):
        1. Solver convergence quality
        2. Velocity stability (penalizes rapid speed changes)
        3. Heading accuracy (how well aligned the turret is)
        4. Distance in scoring range (penalty near min/max boundaries)
        """
        cfg = self.config
        components: list[tuple[float, float]] = []

        # 1. Solver convergence
        if not converged or iterations_used > cfg.max_iterations:
            conv_score = 0.0
        elif iterations_used <= 3:
            conv_score = 1.0
        else:
            t = (iterations_used - 3) / max(1, cfg.max_iterations - 3)
            conv_score = max(0.0, 1.0 - 0.9 * t)
        components.append((conv_score, cfg.w_convergence))

        # 2. Velocity stability (penalize rapid speed changes)
        speed_delta = abs(robot_speed - self._previous_speed)
        vel_score = max(0.0, min(1.0, 1.0 - speed_delta / 0.5))
        components.append((vel_score, cfg.w_velocity_stability))

        # 3. Heading accuracy
        heading_score = max(
            0.0, min(1.0, 1.0 - abs(heading_error_rad) / cfg.heading_max_error_rad)
        )
        components.append((heading_score, cfg.w_heading_accuracy))

        # 4. Distance in scoring range
        if effective_distance < cfg.min_scoring_distance:
            dist_score = 0.0
        elif effective_distance > cfg.max_scoring_distance:
            dist_score = 0.0
        else:
            range_span = cfg.max_scoring_distance - cfg.min_scoring_distance
            range_fraction = (
                effective_distance - cfg.min_scoring_distance
            ) / range_span
            dist_score = max(0.0, 1.0 - 2.0 * abs(range_fraction - 0.5))
        components.append((dist_score, cfg.w_distance_in_range))

        # Weighted geometric mean (in log space to avoid underflow)
        total_weight = 0.0
        log_sum = 0.0
        for score, weight in components:
            if score <= 0:
                return 0.0  # one zero kills the score
            log_sum += weight * math.log(score)
            total_weight += weight

        if total_weight <= 0:
            return 0.0

        return 100.0 * math.exp(log_sum / total_weight)

    # pylint: disable-next=too-many-statements,too-many-locals
    def solve(
        self,
        target_location: Translation2d,
        turret_location: Translation2d,
        turret_velocity: ChassisSpeeds,
        robot_speed: float,
        is_shooting: bool,
        heading_error_rad: float = 0.0,
    ) -> FireControlResult:
        """
        Run the SOTM solver for one cycle.

        Args:
            target_location: Field-frame position of the scoring target
            turret_location: Field-frame position of the turret
            turret_velocity: Field-frame velocity of the turret (includes robot + turret rotation)
            robot_speed: Scalar speed of the robot (m/s)
            is_shooting: True for shoot mode, False for feed mode
            heading_error_rad: Current turret heading error (for confidence scoring)

        Returns:
            FireControlResult with compensated target location, distance, TOF, and confidence
        """
        result = FireControlResult()

        # Raw distance from turret to target
        target_relative = target_location - turret_location
        raw_distance = target_relative.norm()

        # Static shot if robot is barely moving
        if robot_speed < self.config.min_sotm_speed:
            result.effective_location = target_location
            result.effective_distance = raw_distance
            result.time_of_flight = self._get_tof_for_distance(
                raw_distance, is_shooting
            )
            result.is_static = True
            result.converged = True
            result.iterations_used = 0
            result.confidence = self._compute_confidence(
                0, True, robot_speed, raw_distance, heading_error_rad
            )
            self._previous_speed = robot_speed
            self._log(result)
            return result

        # Initialize TOF with warm start or lookup
        if self._has_warm_start:
            tof = self._previous_tof
        else:
            tof = self._get_tof_for_distance(raw_distance, is_shooting)

        # Newton-Raphson iteration
        vx = turret_velocity.vx
        vy = turret_velocity.vy
        c = self.config.sotm_drag_coeff
        converged = False
        iterations = 0

        for i in range(self.config.max_iterations):
            iterations = i + 1

            # Drag-compensated drift time
            drift_tof = self._drag_compensated_drift(tof)

            # Project where the target appears from the turret's perspective,
            # accounting for the ball inheriting the robot's velocity
            rx = target_relative.X()
            ry = target_relative.Y()
            proj_x = rx - vx * drift_tof
            proj_y = ry - vy * drift_tof
            proj_dist = max(math.sqrt(proj_x * proj_x + proj_y * proj_y), 0.01)

            # Look up expected TOF at the projected distance
            lookup_tof = self._get_tof_for_distance(proj_dist, is_shooting)

            # Check convergence
            if abs(lookup_tof - tof) < self.config.convergence_tolerance:
                converged = True
                tof = lookup_tof
                break

            # Newton step: f(t) = g(d(t)) - t, where g is the TOF lookup and d(t) is projected distance
            # Derivative of drift_tof w.r.t. tof: e^(-c*t)
            drag_exp = math.exp(-c * tof) if c > 1e-6 else 1.0

            # Derivative of projected distance w.r.t. drift_tof
            d_prime = -drag_exp * (rx * vx + ry * vy) / proj_dist

            # Numerical derivative of TOF lookup w.r.t. distance
            epsilon = 0.01
            g_prime = (
                self._get_tof_for_distance(proj_dist + epsilon, is_shooting)
                - self._get_tof_for_distance(proj_dist - epsilon, is_shooting)
            ) / (2.0 * epsilon)

            # Newton step: t_new = t - f(t) / f'(t)
            f_prime = g_prime * d_prime - 1.0
            if abs(f_prime) < 1e-10:
                break  # degenerate — can't take Newton step

            tof = tof - (lookup_tof - tof) / f_prime

            # Clamp to valid range
            tof = max(self.config.tof_min, min(self.config.tof_max, tof))

        # Compute final effective location using converged TOF
        final_drift = self._drag_compensated_drift(tof)
        result.effective_location = target_location - Translation2d(
            vx * final_drift, vy * final_drift
        )
        result.effective_distance = result.effective_location.distance(turret_location)
        result.time_of_flight = tof
        result.is_static = False
        result.converged = converged
        result.iterations_used = iterations
        result.confidence = self._compute_confidence(
            iterations,
            converged,
            robot_speed,
            result.effective_distance,
            heading_error_rad,
        )

        # Save state for next cycle
        self._previous_tof = tof
        self._previous_speed = robot_speed
        self._has_warm_start = True

        self._log(result)
        return result

    def _log(self, result: FireControlResult) -> None:
        Logger.recordOutput("Robot/SOTM/TOF", result.time_of_flight)
        Logger.recordOutput("Robot/SOTM/Confidence", result.confidence)
        Logger.recordOutput("Robot/SOTM/Iterations", result.iterations_used)
        Logger.recordOutput("Robot/SOTM/IsStatic", result.is_static)
        Logger.recordOutput("Robot/SOTM/Converged", result.converged)
