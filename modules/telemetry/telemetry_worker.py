"""
Telemtry worker that gathers GPS data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import telemetry
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def telemetry_worker(
    connection: mavutil.mavfile,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process that reads ATTITUDE and LOCAL_POSITION_NED messages from the drone
    and outputs TelemetryData objects.

    connection: MAVLink connection to the drone.
    output_queue: Queue to put TelemetryData objects into.
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
    # Instantiate Telemetry
    result, telem = telemetry.Telemetry.create(connection, local_logger)
    if not result:
        local_logger.error("Failed to create Telemetry")
        return

    # Get Pylance to stop complaining
    assert telem is not None

    local_logger.info("Telemetry created, starting main loop")

    # Main loop: receive telemetry and put to queue until exit is requested
    while not controller.is_exit_requested():
        controller.check_pause()

        result, telemetry_data = telem.run()
        if not result:
            # Timeout or failed to receive both messages, restart
            local_logger.warning("Telemetry run() timed out, restarting")
            continue

        # Get Pylance to stop complaining
        assert telemetry_data is not None

        output_queue.queue.put(telemetry_data)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
