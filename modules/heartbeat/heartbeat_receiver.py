"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Number of consecutive missed heartbeats before reporting Disconnected
DISCONNECT_THRESHOLD = 5


class HeartbeatReceiver:
    """
    HeartbeatReceiver class to receive heartbeats from the drone and track connection state.

    The worker expects a heartbeat every 1 second.
    If 1 is missed, a warning is logged.
    If 5 or more are missed in a row, the connection is considered Disconnected.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, HeartbeatReceiver] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        return True, HeartbeatReceiver(cls.__private_key, connection, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        # Store connection and logger
        self.__connection = connection
        self.__logger = local_logger

        # Track consecutive missed heartbeats
        self.__miss_count = 0

    def run(self) -> "tuple[bool, str | None]":
        """
        Attempt to receive a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.

        Returns (True, status_string) on success, (False, None) on internal error.
        """
        msg = self.__connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

        if msg is None or msg.get_type() != "HEARTBEAT":
            # Missed a heartbeat
            self.__miss_count += 1
            self.__logger.warning(f"Missed heartbeat! Consecutive misses: {self.__miss_count}")

            if self.__miss_count >= DISCONNECT_THRESHOLD:
                self.__logger.warning("Connection Disconnected (missed 5+ heartbeats)")
                return True, "Disconnected"
            # Fewer than threshold misses — still potentially connected
            # Return Connected status until threshold is hit
            return True, "Connected"

        # Received a heartbeat, reset miss counter
        self.__logger.info("Received heartbeat — Connected")
        self.__miss_count = 0
        return True, "Connected"


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
