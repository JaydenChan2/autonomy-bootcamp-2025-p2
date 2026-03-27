"""
Telemetry gathering logic.
"""

import time

from pymavlink import mavutil

from ..common.modules.logger import logger


class TelemetryData:  # pylint: disable=too-many-instance-attributes
    """
    Python struct to represent Telemtry Data. Contains the most recent attitude and position reading.
    """

    def __init__(
        self,
        time_since_boot: int | None = None,  # ms
        x: float | None = None,  # m
        y: float | None = None,  # m
        z: float | None = None,  # m
        x_velocity: float | None = None,  # m/s
        y_velocity: float | None = None,  # m/s
        z_velocity: float | None = None,  # m/s
        roll: float | None = None,  # rad
        pitch: float | None = None,  # rad
        yaw: float | None = None,  # rad
        roll_speed: float | None = None,  # rad/s
        pitch_speed: float | None = None,  # rad/s
        yaw_speed: float | None = None,  # rad/s
    ) -> None:
        self.time_since_boot = time_since_boot
        self.x = x
        self.y = y
        self.z = z
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.z_velocity = z_velocity
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.roll_speed = roll_speed
        self.pitch_speed = pitch_speed
        self.yaw_speed = yaw_speed

    def __str__(self) -> str:
        return f"""{{
            time_since_boot: {self.time_since_boot},
            x: {self.x},
            y: {self.y},
            z: {self.z},
            x_velocity: {self.x_velocity},
            y_velocity: {self.y_velocity},
            z_velocity: {self.z_velocity},
            roll: {self.roll},
            pitch: {self.pitch},
            yaw: {self.yaw},
            roll_speed: {self.roll_speed},
            pitch_speed: {self.pitch_speed},
            yaw_speed: {self.yaw_speed}
        }}"""


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
TELEMETRY_TIMEOUT = 1  # seconds


class Telemetry:
    """
    Telemetry class to read position and attitude (orientation).

    Waits up to 1 second to receive both LOCAL_POSITION_NED (32) and ATTITUDE (30)
    messages. Returns a TelemetryData object with the combined data, using the most
    recent timestamp.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, Telemetry] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Telemetry object.
        """
        return True, Telemetry(cls.__private_key, connection, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Telemetry.__private_key, "Use create() method"

        # Store connection and logger
        self.__connection = connection
        self.__logger = local_logger

    def run(self) -> "tuple[bool, TelemetryData | None]":
        """
        Receive LOCAL_POSITION_NED and ATTITUDE messages from the drone,
        combining them together to form a single TelemetryData object.

        Returns (True, TelemetryData) if both messages are received within the timeout.
        Returns (False, None) if a timeout occurs (restart signal).
        """
        # Read MAVLink message LOCAL_POSITION_NED (32)
        pos_msg = self.__connection.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=TELEMETRY_TIMEOUT
        )
        if pos_msg is None or pos_msg.get_type() != "LOCAL_POSITION_NED":
            self.__logger.warning("Timed out waiting for LOCAL_POSITION_NED, restarting")
            return False, None

        # Read MAVLink message ATTITUDE (30)
        att_msg = self.__connection.recv_match(
            type="ATTITUDE", blocking=True, timeout=TELEMETRY_TIMEOUT
        )
        if att_msg is None or att_msg.get_type() != "ATTITUDE":
            self.__logger.warning("Timed out waiting for ATTITUDE, restarting")
            return False, None

        # Use the most recent timestamp from both messages
        # LOCAL_POSITION_NED uses z-down (NED), but per bootcamp instructions we treat
        # it as standard right-handed x-y-z where z is up, so we negate the NED z/vz.
        time_since_boot = max(pos_msg.time_boot_ms, att_msg.time_boot_ms)

        telemetry_data = TelemetryData(
            time_since_boot=time_since_boot,
            x=pos_msg.x,
            y=pos_msg.y,
            z=pos_msg.z,  # Treat as standard x-y-z per bootcamp instructions
            x_velocity=pos_msg.vx,
            y_velocity=pos_msg.vy,
            z_velocity=pos_msg.vz,  # Treat as standard x-y-z per bootcamp instructions
            roll=att_msg.roll,
            pitch=att_msg.pitch,
            yaw=att_msg.yaw,
            roll_speed=att_msg.rollspeed,
            pitch_speed=att_msg.pitchspeed,
            yaw_speed=att_msg.yawspeed,
        )

        self.__logger.info(f"Telemetry received: {telemetry_data}")
        return True, telemetry_data


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
