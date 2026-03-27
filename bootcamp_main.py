"""
Bootcamp F2025

Main process to setup and manage all the other working processes
"""

import multiprocessing as mp
import queue
import time

from pymavlink import mavutil

from modules.common.modules.logger import logger
from modules.common.modules.logger import logger_main_setup
from modules.common.modules.read_yaml import read_yaml
from modules.command import command
from modules.command import command_worker
from modules.heartbeat import heartbeat_receiver_worker
from modules.heartbeat import heartbeat_sender_worker
from modules.telemetry import telemetry_worker
from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from utilities.workers import worker_manager


# MAVLink connection
CONNECTION_STRING = "tcp:localhost:12345"

# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Set queue max sizes (<= 0 for infinity)
HEARTBEAT_STATUS_QUEUE_MAX_SIZE = 5  # heartbeat receiver -> main
TELEMETRY_TO_COMMAND_QUEUE_MAX_SIZE = 5  # telemetry -> command
COMMAND_OUTPUT_QUEUE_MAX_SIZE = 5  # command -> main

# Set worker counts
HEARTBEAT_SENDER_WORKER_COUNT = 1
HEARTBEAT_RECEIVER_WORKER_COUNT = 1
TELEMETRY_WORKER_COUNT = 1
COMMAND_WORKER_COUNT = 1

# Target position for command worker (the object the drone should face)
TARGET = command.Position(10, 20, 30)

# How long to run before shutting down (seconds)
RUN_DURATION = 100

# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================


def main() -> int:
    """
    Main function.
    """
    # Configuration settings
    result, config = read_yaml.open_config(logger.CONFIG_FILE_PATH)
    if not result:
        print("ERROR: Failed to load configuration file")
        return -1

    # Get Pylance to stop complaining
    assert config is not None

    # Setup main logger
    result, main_logger, _ = logger_main_setup.setup_main_logger(config)
    if not result:
        print("ERROR: Failed to create main logger")
        return -1

    # Get Pylance to stop complaining
    assert main_logger is not None

    # Create a connection to the drone. Assume that this is safe to pass around to all processes
    # In reality, this will not work, but to simplify the bootamp, preetend it is allowed
    # To test, you will run each of your workers individually to see if they work
    # (test "drones" are provided for you test your workers)
    # NOTE: If you want to have type annotations for the connection, it is of type mavutil.mavfile
    connection = mavutil.mavlink_connection(CONNECTION_STRING)
    connection.wait_heartbeat(timeout=30)  # Wait for the "drone" to connect

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Create a worker controller
    controller = worker_controller.WorkerController()

    # Create a multiprocess manager for synchronized queues
    mp_manager = mp.Manager()

    # Create queues
    # heartbeat_receiver outputs "Connected"/"Disconnected" to main
    heartbeat_status_queue = queue_proxy_wrapper.QueueProxyWrapper(
        mp_manager,
        HEARTBEAT_STATUS_QUEUE_MAX_SIZE,
    )
    # telemetry outputs TelemetryData to command
    telemetry_to_command_queue = queue_proxy_wrapper.QueueProxyWrapper(
        mp_manager,
        TELEMETRY_TO_COMMAND_QUEUE_MAX_SIZE,
    )
    # command outputs command strings to main
    command_output_queue = queue_proxy_wrapper.QueueProxyWrapper(
        mp_manager,
        COMMAND_OUTPUT_QUEUE_MAX_SIZE,
    )

    # Create worker properties for each worker type (what inputs it takes, how many workers)

    # Heartbeat sender: no input queue, no output queue — just sends to drone
    result, heartbeat_sender_properties = worker_manager.WorkerProperties.create(
        count=HEARTBEAT_SENDER_WORKER_COUNT,
        target=heartbeat_sender_worker.heartbeat_sender_worker,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[],
        controller=controller,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create arguments for Heartbeat Sender")
        return -1

    assert heartbeat_sender_properties is not None

    # Heartbeat receiver: no input queue, outputs status string to main
    result, heartbeat_receiver_properties = worker_manager.WorkerProperties.create(
        count=HEARTBEAT_RECEIVER_WORKER_COUNT,
        target=heartbeat_receiver_worker.heartbeat_receiver_worker,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[heartbeat_status_queue],
        controller=controller,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create arguments for Heartbeat Receiver")
        return -1

    assert heartbeat_receiver_properties is not None

    # Telemetry: no input queue, outputs TelemetryData to command
    result, telemetry_properties = worker_manager.WorkerProperties.create(
        count=TELEMETRY_WORKER_COUNT,
        target=telemetry_worker.telemetry_worker,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[telemetry_to_command_queue],
        controller=controller,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create arguments for Telemetry")
        return -1

    assert telemetry_properties is not None

    # Command: reads TelemetryData from telemetry queue, outputs command strings to main
    result, command_properties = worker_manager.WorkerProperties.create(
        count=COMMAND_WORKER_COUNT,
        target=command_worker.command_worker,
        work_arguments=(connection, TARGET),
        input_queues=[telemetry_to_command_queue],
        output_queues=[command_output_queue],
        controller=controller,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create arguments for Command")
        return -1

    assert command_properties is not None

    # Create the workers (processes) and obtain their managers
    worker_managers: list[worker_manager.WorkerManager] = []

    result, sender_manager = worker_manager.WorkerManager.create(
        worker_properties=heartbeat_sender_properties,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create manager for Heartbeat Sender")
        return -1
    assert sender_manager is not None
    worker_managers.append(sender_manager)

    result, receiver_manager = worker_manager.WorkerManager.create(
        worker_properties=heartbeat_receiver_properties,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create manager for Heartbeat Receiver")
        return -1
    assert receiver_manager is not None
    worker_managers.append(receiver_manager)

    result, telem_manager = worker_manager.WorkerManager.create(
        worker_properties=telemetry_properties,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create manager for Telemetry")
        return -1
    assert telem_manager is not None
    worker_managers.append(telem_manager)

    result, cmd_manager = worker_manager.WorkerManager.create(
        worker_properties=command_properties,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create manager for Command")
        return -1
    assert cmd_manager is not None
    worker_managers.append(cmd_manager)

    # Start worker processes
    for mgr in worker_managers:
        mgr.start_workers()

    main_logger.info("Started")

    # Main's work: read from all queues that output to main, and log any commands that we make
    # Continue running for RUN_DURATION seconds or until the drone disconnects
    start_time = time.time()
    disconnected = False
    while time.time() - start_time < RUN_DURATION and not disconnected:
        # Check heartbeat status queue
        try:
            status = heartbeat_status_queue.queue.get(timeout=0.1)
            main_logger.info(f"Heartbeat status: {status}")
            if status == "Disconnected":
                main_logger.warning("Drone disconnected! Stopping.")
                disconnected = True
                break
        except queue.Empty:
            pass

        # Check command output queue
        try:
            command_str = command_output_queue.queue.get(timeout=0.1)
            main_logger.info(f"Command output: {command_str}")
        except queue.Empty:
            pass

    # Stop the processes
    controller.request_exit()

    main_logger.info("Requested exit")

    # Fill and drain queues from END TO START (order: command output, telemetry-to-command,
    # heartbeat status — i.e., consumer-side last)
    command_output_queue.fill_and_drain_queue()
    telemetry_to_command_queue.fill_and_drain_queue()
    heartbeat_status_queue.fill_and_drain_queue()

    main_logger.info("Queues cleared")

    # Clean up worker processes
    for mgr in worker_managers:
        mgr.join_workers()

    main_logger.info("Stopped")

    # We can reset controller in case we want to reuse it
    # Alternatively, create a new WorkerController instance
    controller.clear_exit()

    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Failed with return code {result_main}")
    else:
        print("Success!")
