"""
Command worker to make decisions based on Telemetry Data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import command
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def command_worker(
    connection: mavutil.mavfile,
    target: command.Position,
    input_queue: queue_proxy_wrapper.QueueProxyWrapper,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process that reads TelemetryData from the input queue and sends COMMAND_LONG
    messages to the drone to keep it aimed at the target.

    connection: MAVLink connection to the drone.
    target: The 3D target position the drone should face.
    input_queue: Queue of TelemetryData objects from the telemetry worker.
    output_queue: Queue to put command result strings into (for main process to log).
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
    # Instantiate Command
    result, cmd = command.Command.create(connection, target, local_logger)
    if not result:
        local_logger.error("Failed to create Command")
        return

    # Get Pylance to stop complaining
    assert cmd is not None

    local_logger.info("Command created, starting main loop")

    # Main loop: get telemetry from queue, run command logic, put result to output queue
    while not controller.is_exit_requested():
        controller.check_pause()

        try:
            telemetry_data = input_queue.queue.get(timeout=0.1)
        except Exception:  # pylint: disable=broad-except
            # Queue empty or timeout, loop back
            continue

        if telemetry_data is None:
            # Sentinel value — exit signal from fill_and_drain_queue
            break

        result, command_str = cmd.run(telemetry_data)
        if not result:
            local_logger.error("Command.run() failed")
            continue

        if command_str is not None:
            local_logger.info(f"Command issued: {command_str}")
            output_queue.queue.put(command_str)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
