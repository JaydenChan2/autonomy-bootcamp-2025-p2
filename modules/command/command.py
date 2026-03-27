"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Constants matching the mock drone test expectations
HEIGHT_TOLERANCE = 0.5  # metres: if off by more than this, adjust altitude
ANGLE_TOLERANCE = 5  # degrees: if facing more than this away, adjust yaw
Z_SPEED = 1  # m/s used in MAV_CMD_CONDITION_CHANGE_ALT
TURNING_SPEED = 5  # deg/s used in MAV_CMD_CONDITION_YAW


class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on received telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        return True, Command(cls.__private_key, connection, target, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        # Store connection, target position, and logger
        self.__connection = connection
        self.__target = target
        self.__logger = local_logger

        # Running average velocity tracking (accumulate sum of velocity vectors per sample)
        self.__total_vx = 0.0
        self.__total_vy = 0.0
        self.__total_vz = 0.0
        self.__sample_count = 0

    def run(
        self,
        telemetry_data: telemetry.TelemetryData,
    ) -> "tuple[bool, str | None]":
        """
        Make a decision based on received telemetry data.
        Returns (True, command_string) when a command is issued,
        (True, None) when no adjustment is needed,
        (False, None) on error.
        """
        # Validate required telemetry fields
        if (
            telemetry_data.x is None
            or telemetry_data.y is None
            or telemetry_data.z is None
            or telemetry_data.yaw is None
            or telemetry_data.x_velocity is None
            or telemetry_data.y_velocity is None
            or telemetry_data.z_velocity is None
        ):
            self.__logger.error("Received telemetry data with missing fields")
            return False, None

        # Log average velocity for this trip so far
        # The given velocity is the average for the entire time period between data reports,
        # so we track a running cumulative average across all samples.
        self.__sample_count += 1
        self.__total_vx += telemetry_data.x_velocity
        self.__total_vy += telemetry_data.y_velocity
        self.__total_vz += telemetry_data.z_velocity

        avg_vx = self.__total_vx / self.__sample_count
        avg_vy = self.__total_vy / self.__sample_count
        avg_vz = self.__total_vz / self.__sample_count

        self.__logger.info(
            f"Average velocity so far: ({avg_vx:.4f}, {avg_vy:.4f}, {avg_vz:.4f}) m/s"
        )

        # Use COMMAND_LONG (76) message, target_system=1, target_component=0

        # Adjust height using MAV_CMD_CONDITION_CHANGE_ALT (113)
        # If more than HEIGHT_TOLERANCE off target altitude, send altitude command
        delta_z = self.__target.z - telemetry_data.z
        if abs(delta_z) > HEIGHT_TOLERANCE:
            self.__connection.mav.command_long_send(
                1,                # target_system
                0,                # target_component
                mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                0,                # confirmation
                Z_SPEED,          # param1: ascent/descent speed (m/s)
                0,                # param2: unused
                0,                # param3: unused
                0,                # param4: unused
                0,                # param5: unused
                0,                # param6: unused
                self.__target.z,  # param7: target altitude (metres)
            )
            command_str = f"CHANGE ALTITUDE: {delta_z}"
            self.__logger.info(command_str)
            return True, command_str

        # Adjust direction (yaw) using MAV_CMD_CONDITION_YAW (115) — relative angle
        # Compute the angle from the drone to the target in x-y plane
        dx = self.__target.x - telemetry_data.x
        dy = self.__target.y - telemetry_data.y
        angle_to_target = math.atan2(dy, dx)  # radians, standard right-handed CCW positive

        # Relative yaw adjustment needed (positive = CCW)
        yaw_diff_rad = angle_to_target - telemetry_data.yaw
        # Normalize to [-pi, pi]
        yaw_diff_rad = (yaw_diff_rad + math.pi) % (2 * math.pi) - math.pi
        # Convert to degrees
        yaw_diff_deg = math.degrees(yaw_diff_rad)

        if abs(yaw_diff_deg) > ANGLE_TOLERANCE:
            # param3: direction — 1=CW (negative diff), -1=CCW (positive diff)
            direction = -1 if yaw_diff_deg > 0 else 1

            self.__connection.mav.command_long_send(
                1,                    # target_system
                0,                    # target_component
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,                    # confirmation
                abs(yaw_diff_deg),    # param1: angle magnitude (degrees)
                TURNING_SPEED,        # param2: angular speed (deg/s)
                direction,            # param3: -1=CCW, 1=CW
                1,                    # param4: 1=relative angle
                0,                    # param5: unused
                0,                    # param6: unused
                0,                    # param7: unused
            )
            command_str = f"CHANGE YAW: {yaw_diff_deg}"
            self.__logger.info(command_str)
            return True, command_str

        # No adjustment needed
        self.__logger.info("No adjustment needed for this telemetry frame")
        return True, None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
