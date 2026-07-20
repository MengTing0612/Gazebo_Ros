"""PX4 offboard hover with bounded SDK residuals and a SITL visual demo."""

from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from flight_bridge.action_adapter import ActionAdapter
from flight_bridge.command_types import ShieldInfo, VelocityCmd
from flight_bridge.config_loader import load_config
from flight_bridge.sdk_client import DEFAULT_PASSWORD, DEFAULT_USERNAME
from flight_bridge.state_builder import CopterStateBuilder, HoverFeedbackBuilder
from flight_bridge.safety_shield import SafetyShield

from .frames import body_flu_to_world_ned, quaternion_frd_to_ned_to_euler
from .hover_logic import (
    HoverTargetNed,
    LocalNedState,
    SdkResidualSchedule,
    SitlVisualDisturbance,
    add_velocity_commands,
    build_sdk_state,
    capture_hover_target,
    position_setpoint_velocity,
    scale_velocity_command,
)
from .sdk_worker import SdkWorker


class OffboardHoverNode(Node):
    """Hold an EKF-relative position using PX4 MPC and bounded SDK feed-forward."""

    def __init__(self) -> None:
        super().__init__("offboard_hover")
        self._declare_parameters()
        self._config = load_config(Path(self.get_parameter("config_file").value))
        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._target_altitude_m = float(self.get_parameter("target_altitude_m").value)
        self._auto_arm = bool(self.get_parameter("auto_arm").value)
        self._warmup_frames = max(1, int(round(
            float(self.get_parameter("offboard_warmup_s").value) * self._control_rate_hz
        )))
        self._state_timeout_s = float(self.get_parameter("state_timeout_s").value)
        self._sdk_max_age_s = float(self.get_parameter("sdk_max_age_s").value)
        self._sdk_output_log_period_s = float(
            self.get_parameter("sdk_output_log_period_s").value,
        )
        self._auto_arm_retry_s = float(self.get_parameter("auto_arm_retry_s").value)
        self._sdk_mode = str(self.get_parameter("sdk_mode").value)
        self._sdk_resume = bool(self.get_parameter("sdk_resume").value)
        self._sdk_residual_schedule = SdkResidualSchedule(
            start_weight=float(self.get_parameter("sdk_residual_start_weight").value),
            hold_s=float(self.get_parameter("sdk_residual_hold_s").value),
            decay_s=float(self.get_parameter("sdk_residual_decay_s").value),
            final_weight=float(self.get_parameter("sdk_residual_final_weight").value),
        )
        self._sitl_visual_demo_enabled = bool(
            self.get_parameter("sitl_visual_demo_enabled").value,
        )
        self._sitl_visual_disturbance = SitlVisualDisturbance(
            hold_s=float(self.get_parameter("sitl_visual_demo_hold_s").value),
            decay_s=float(self.get_parameter("sitl_visual_demo_decay_s").value),
            horizontal_mps=float(
                self.get_parameter("sitl_visual_demo_horizontal_mps").value,
            ),
            vertical_mps=float(self.get_parameter("sitl_visual_demo_vertical_mps").value),
            period_s=float(self.get_parameter("sitl_visual_demo_period_s").value),
        )
        self._vehicle_local_position_topic = str(
            self.get_parameter("vehicle_local_position_topic").value,
        )
        self._vehicle_status_topic = str(
            self.get_parameter("vehicle_status_topic").value,
        )
        self._limits = self._read_residual_limits()

        if self._control_rate_hz < 10.0:
            raise ValueError("control_rate_hz must be at least 10 Hz for PX4 Offboard")
        if self._target_altitude_m <= 0.0:
            raise ValueError("target_altitude_m must be positive")
        if self._auto_arm_retry_s <= 0.0:
            raise ValueError("auto_arm_retry_s must be positive")
        if self._sdk_output_log_period_s <= 0.0:
            raise ValueError("sdk_output_log_period_s must be positive")

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos,
        )
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos,
        )
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos,
        )
        self.create_subscription(
            VehicleLocalPosition, self._vehicle_local_position_topic, self._on_local_position, qos,
        )
        self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_attitude, qos,
        )
        self.create_subscription(
            VehicleStatus, self._vehicle_status_topic, self._on_vehicle_status, qos,
        )

        self._state: LocalNedState | None = None
        self._last_state_monotonic_s = 0.0
        self._last_stale_warning_monotonic_s = 0.0
        self._latest_roll_flu_rad = 0.0
        self._latest_pitch_flu_rad = 0.0
        self._target: HoverTargetNed | None = None
        self._status = VehicleStatus()
        self._setpoint_count = 0
        self._offboard_requested = False
        self._arm_requested = False
        self._last_arm_request_monotonic_s = float("-inf")
        self._last_sdk_command = VelocityCmd.hover()
        self._last_shield = ShieldInfo()
        self._last_sdk_unavailable_warning_monotonic_s = float("-inf")
        self._last_sdk_unavailable_reason = ""
        self._last_sdk_output_log_monotonic_s = float("-inf")
        self._sdk_residual_started_monotonic_s: float | None = None
        self._sdk_residual_faded_logged = False
        self._sitl_visual_demo_started_monotonic_s: float | None = None
        self._sitl_visual_demo_complete_logged = False

        self._state_builder = CopterStateBuilder()
        self._feedback_builder = HoverFeedbackBuilder(self._config.feedback)
        self._action_adapter = ActionAdapter(
            self._config.control, self._config.limits, self._config.copter,
        )
        self._safety_shield = SafetyShield(
            self._config.safety, self._config.limits, self._config.filter,
        )
        self._sdk_worker = SdkWorker(
            mode=self._sdk_mode,
            host=str(self.get_parameter("sdk_host").value),
            port=int(self.get_parameter("sdk_port").value),
            timeout_s=float(self.get_parameter("sdk_timeout_ms").value) / 1000.0,
            username=str(self.get_parameter("sdk_username").value),
            password=str(self.get_parameter("sdk_password").value),
            resume=self._sdk_resume,
        )
        self._sdk_worker.start()
        self._timer = self.create_timer(1.0 / self._control_rate_hz, self._control_tick)
        self.get_logger().info(
            "PX4 hover controller ready: "
            f"target altitude {self._target_altitude_m:.2f} m, "
            f"auto_arm={self._auto_arm}, SDK={self._sdk_mode}, "
            f"sdk_resume={self._sdk_resume}, "
            f"sitl_visual_demo={self._sitl_visual_demo_enabled}",
        )

    def destroy_node(self) -> bool:
        self._sdk_worker.close()
        return super().destroy_node()

    def _declare_parameters(self) -> None:
        default_config = Path(get_package_share_directory("micoair_hover")) / "config" / "hover_sim.yaml"
        self.declare_parameter("config_file", str(default_config))
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("target_altitude_m", 2.0)
        self.declare_parameter("offboard_warmup_s", 1.0)
        self.declare_parameter("auto_arm", False)
        self.declare_parameter("auto_arm_retry_s", 1.0)
        self.declare_parameter("state_timeout_s", 0.25)
        self.declare_parameter(
            "vehicle_local_position_topic",
            "/fmu/out/vehicle_local_position_v1",
        )
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("sdk_mode", "disabled")
        self.declare_parameter("sdk_resume", False)
        self.declare_parameter("sdk_host", "127.0.0.1")
        self.declare_parameter("sdk_port", 9000)
        # The SDK reference client allows ten seconds for login, resume, and
        # initial JIT inference. This runs in SdkWorker, never in the 20 Hz loop.
        self.declare_parameter("sdk_timeout_ms", 10_000)
        self.declare_parameter("sdk_max_age_s", 0.25)
        self.declare_parameter("sdk_output_log_period_s", 1.0)
        self.declare_parameter("sdk_residual_start_weight", 1.0)
        self.declare_parameter("sdk_residual_hold_s", 2.0)
        self.declare_parameter("sdk_residual_decay_s", 20.0)
        self.declare_parameter("sdk_residual_final_weight", 0.0)
        self.declare_parameter("sitl_visual_demo_enabled", False)
        self.declare_parameter("sitl_visual_demo_hold_s", 8.0)
        self.declare_parameter("sitl_visual_demo_decay_s", 14.0)
        self.declare_parameter("sitl_visual_demo_horizontal_mps", 0.55)
        self.declare_parameter("sitl_visual_demo_vertical_mps", 0.12)
        self.declare_parameter("sitl_visual_demo_period_s", 7.0)
        self.declare_parameter("sdk_username", DEFAULT_USERNAME)
        self.declare_parameter("sdk_password", DEFAULT_PASSWORD)
        self.declare_parameter("max_horizontal_residual_mps", 0.10)
        self.declare_parameter("max_vertical_residual_mps", 0.05)
        self.declare_parameter("max_yaw_residual_rad_s", 0.10)

    def _read_residual_limits(self) -> tuple[float, float, float]:
        limits = (
            float(self.get_parameter("max_horizontal_residual_mps").value),
            float(self.get_parameter("max_vertical_residual_mps").value),
            float(self.get_parameter("max_yaw_residual_rad_s").value),
        )
        if any(limit <= 0.0 for limit in limits):
            raise ValueError("all SDK residual limits must be positive")
        return limits

    def _on_local_position(self, message: VehicleLocalPosition) -> None:
        if not (message.xy_valid and message.z_valid and message.v_xy_valid and message.v_z_valid):
            return
        self._state = LocalNedState(
            north_m=message.x,
            east_m=message.y,
            down_m=message.z,
            north_mps=message.vx,
            east_mps=message.vy,
            down_mps=message.vz,
            yaw_ned_rad=message.heading,
            roll_flu_rad=self._latest_roll_flu_rad,
            pitch_flu_rad=self._latest_pitch_flu_rad,
        )
        self._last_state_monotonic_s = time.monotonic()

    def _on_attitude(self, message: VehicleAttitude) -> None:
        roll_frd, pitch_frd, _ = quaternion_frd_to_ned_to_euler(tuple(message.q))
        self._latest_roll_flu_rad = roll_frd
        self._latest_pitch_flu_rad = -pitch_frd

    def _on_vehicle_status(self, message: VehicleStatus) -> None:
        self._status = message

    def _control_tick(self) -> None:
        state = self._state
        if state is None or time.monotonic() - self._last_state_monotonic_s > self._state_timeout_s:
            now = time.monotonic()
            if now - self._last_stale_warning_monotonic_s >= 2.0:
                self.get_logger().warning("PX4 local position is stale; withholding offboard setpoints")
                self._last_stale_warning_monotonic_s = now
            return

        if self._target is None:
            self._target = capture_hover_target(state, self._target_altitude_m)
            self.get_logger().info(
                "Captured NED hover target: "
                f"north={self._target.north_m:.2f} east={self._target.east_m:.2f} "
                f"down={self._target.down_m:.2f}",
            )

        sdk_residual = self._sdk_residual(state, self._target)
        demo_residual = self._sitl_visual_demo_residual()
        residual = position_setpoint_velocity(
            add_velocity_commands(sdk_residual, demo_residual),
            *self._feedforward_limits(),
        )
        self._publish_offboard_mode()
        self._publish_position_setpoint(self._target, state.yaw_ned_rad, residual)
        self._setpoint_count += 1
        if self._auto_arm and self._setpoint_count >= self._warmup_frames:
            self._request_offboard_and_arm()

    def _sdk_residual(self, state: LocalNedState, target: HoverTargetNed) -> VelocityCmd:
        sdk_state = build_sdk_state(
            state,
            target,
            armed=self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            mode=str(self._status.nav_state),
        )
        observation = self._state_builder.build_hover_obs(sdk_state, self._last_sdk_command)
        feedback = self._feedback_builder.build(sdk_state, self._last_shield, self._last_sdk_command)
        self._sdk_worker.submit(observation, feedback)

        if self._sdk_mode == "disabled":
            return self._safe_hover_residual()

        if not self._is_sdk_control_active():
            self._reset_sdk_residual_schedule()
            return self._safe_hover_residual()

        result = self._sdk_worker.latest()
        is_fresh = time.monotonic() - result.received_monotonic_s <= self._sdk_max_age_s
        if result.error is not None:
            self._warn_sdk_unavailable(result.error)
            return self._safe_hover_residual()
        if not is_fresh:
            self._warn_sdk_unavailable("no fresh inference result")
            return self._safe_hover_residual()

        raw_command = self._action_adapter.to_velocity_cmd(result.action)
        blended = self._action_adapter.residual_blend(VelocityCmd.hover(), raw_command)
        residual_weight = self._sdk_residual_weight()
        scheduled_command = scale_velocity_command(blended, residual_weight)
        safe_command, shield_info = self._safety_shield.filter(scheduled_command)
        self._last_sdk_command = position_setpoint_velocity(safe_command, *self._limits)
        self._last_shield = shield_info
        self._log_sdk_output(result.action, self._last_sdk_command, residual_weight)
        return self._last_sdk_command

    def _warn_sdk_unavailable(self, reason: str) -> None:
        """Report an SDK fallback without flooding the 20 Hz control log."""
        now = time.monotonic()
        should_report = (
            reason != self._last_sdk_unavailable_reason
            or now - self._last_sdk_unavailable_warning_monotonic_s >= 5.0
        )
        if not should_report:
            return
        self.get_logger().warning(f"SDK residual unavailable; using PX4-only hover: {reason}")
        self._last_sdk_unavailable_reason = reason
        self._last_sdk_unavailable_warning_monotonic_s = now

    def _log_sdk_output(
        self,
        action: list[float],
        residual: VelocityCmd,
        residual_weight: float,
    ) -> None:
        """Emit the raw SDK action and the bounded command passed to PX4."""
        now = time.monotonic()
        if now - self._last_sdk_output_log_monotonic_s < self._sdk_output_log_period_s:
            return

        action_text = ", ".join(f"{value:+.3f}" for value in action)
        self.get_logger().info(
            "SDK output: "
            f"action=[{action_text}] "
            f"weight={residual_weight:.3f} "
            "residual_body_flu_mps="
            f"({residual.vx:+.3f}, {residual.vy:+.3f}, {residual.vz:+.3f}) "
            f"yaw_rate={residual.yaw_rate:+.3f}",
        )
        self._last_sdk_output_log_monotonic_s = now

    def _is_sdk_control_active(self) -> bool:
        return (
            self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _sdk_residual_weight(self) -> float:
        now = time.monotonic()
        if self._sdk_residual_started_monotonic_s is None:
            self._sdk_residual_started_monotonic_s = now
            self.get_logger().info(
                "SDK residual perturbation started: "
                f"weight={self._sdk_residual_schedule.start_weight:.2f} for "
                f"{self._sdk_residual_schedule.hold_s:.1f}s, then decays over "
                f"{self._sdk_residual_schedule.decay_s:.1f}s",
            )

        elapsed_s = now - self._sdk_residual_started_monotonic_s
        weight = self._sdk_residual_schedule.weight_at(elapsed_s)
        if weight == self._sdk_residual_schedule.final_weight and not self._sdk_residual_faded_logged:
            self._sdk_residual_faded_logged = True
            self.get_logger().info(
                "SDK residual attenuation complete: "
                f"weight={weight:.2f}; PX4 native position control is holding hover",
            )
        return weight

    def _reset_sdk_residual_schedule(self) -> None:
        if self._sdk_residual_started_monotonic_s is None:
            return
        self._sdk_residual_started_monotonic_s = None
        self._sdk_residual_faded_logged = False
        self._last_sdk_command = VelocityCmd.hover()
        self._safety_shield.reset()

    def _sitl_visual_demo_residual(self) -> VelocityCmd:
        """Inject a deterministic visual disturbance only while SITL is in Offboard."""
        if not self._sitl_visual_demo_enabled:
            return VelocityCmd.hover()
        if not self._is_sdk_control_active():
            self._reset_sitl_visual_demo()
            return VelocityCmd.hover()

        now = time.monotonic()
        if self._sitl_visual_demo_started_monotonic_s is None:
            self._sitl_visual_demo_started_monotonic_s = now
            self.get_logger().warning(
                "SITL visual disturbance started: this is a scripted recovery demo, "
                "not SDK learning",
            )

        elapsed_s = now - self._sitl_visual_demo_started_monotonic_s
        command = self._sitl_visual_disturbance.command_at(elapsed_s)
        if (
            self._sitl_visual_disturbance.is_complete(elapsed_s)
            and not self._sitl_visual_demo_complete_logged
        ):
            self._sitl_visual_demo_complete_logged = True
            self.get_logger().info(
                "SITL visual disturbance attenuated to zero; "
                "PX4 native position control is recovering the hover target",
            )
        return command

    def _reset_sitl_visual_demo(self) -> None:
        if self._sitl_visual_demo_started_monotonic_s is None:
            return
        self._sitl_visual_demo_started_monotonic_s = None
        self._sitl_visual_demo_complete_logged = False

    def _feedforward_limits(self) -> tuple[float, float, float]:
        """Keep the combined SDK and SITL-demo feed-forward inside fixed limits."""
        sdk_horizontal, sdk_vertical, sdk_yaw_rate = self._limits
        if not self._sitl_visual_demo_enabled:
            return self._limits
        return (
            min(0.9, sdk_horizontal + self._sitl_visual_disturbance.horizontal_mps),
            min(0.3, sdk_vertical + self._sitl_visual_disturbance.vertical_mps),
            sdk_yaw_rate,
        )

    def _safe_hover_residual(self) -> VelocityCmd:
        self._last_sdk_command, self._last_shield = self._safety_shield.filter(VelocityCmd.hover())
        return self._last_sdk_command

    def _publish_offboard_mode(self) -> None:
        message = OffboardControlMode()
        message.timestamp = self._timestamp_us()
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False
        self._offboard_mode_pub.publish(message)

    def _publish_position_setpoint(
        self,
        target: HoverTargetNed,
        yaw_ned_rad: float,
        residual_body_flu: VelocityCmd,
    ) -> None:
        velocity_ned = body_flu_to_world_ned(
            residual_body_flu.vx,
            residual_body_flu.vy,
            residual_body_flu.vz,
            yaw_ned_rad,
        )
        message = TrajectorySetpoint()
        message.timestamp = self._timestamp_us()
        message.position = [target.north_m, target.east_m, target.down_m]
        message.velocity = list(velocity_ned)
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = target.yaw_ned_rad
        message.yawspeed = -residual_body_flu.yaw_rate
        self._trajectory_pub.publish(message)

    def _request_offboard_and_arm(self) -> None:
        if not self._offboard_requested:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
            self._offboard_requested = True
            self.get_logger().info(
                f"Requested PX4 Offboard mode after {self._setpoint_count} warm-up frames",
            )
        is_armed = self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        should_retry_arm = (
            not is_armed
            and time.monotonic() - self._last_arm_request_monotonic_s >= self._auto_arm_retry_s
        )
        if should_retry_arm:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self._last_arm_request_monotonic_s = time.monotonic()
            is_initial_request = not self._arm_requested
            self._arm_requested = True
            if is_initial_request:
                self.get_logger().info("Requested vehicle arm for simulation")
            else:
                self.get_logger().debug("PX4 remains disarmed; retried arm request")

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        message = VehicleCommand()
        message.timestamp = self._timestamp_us()
        message.param1 = param1
        message.param2 = param2
        message.command = command
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self._vehicle_command_pub.publish(message)

    def _timestamp_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OffboardHoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
