"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import heartbeat_receiver
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def heartbeat_receiver_worker(
    connection: mavutil.mavfile,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process that receives HEARTBEAT messages from the drone and reports connection status.

    connection: MAVLink connection to the drone.
    output_queue: Queue to put the connection status string ("Connected"/"Disconnected") into.
    controller: WorkerController for pause/exit signals from main.
    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate HeartbeatReceiver
    result, receiver = heartbeat_receiver.HeartbeatReceiver.create(connection, local_logger)
    if not result:
        local_logger.error("Failed to create HeartbeatReceiver")
        return

    # Get Pylance to stop complaining
    assert receiver is not None

    local_logger.info("HeartbeatReceiver created, starting main loop")

    # Main loop: receive heartbeat and put status to queue until exit is requested
    while not controller.is_exit_requested():
        controller.check_pause()

        result, status = receiver.run()
        if not result:
            local_logger.error("HeartbeatReceiver.run() failed")
            continue

        # Get Pylance to stop complaining
        assert status is not None

        local_logger.info(f"Connection status: {status}")
        output_queue.queue.put(status)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
