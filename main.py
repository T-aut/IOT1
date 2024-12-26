import time
import can
import random
import threading


def send_with_dynamic_timing(bus, msg, initial_period, skew_s, duration):
    """
    Sends a CAN message periodically with an artificial time skew using a loop.

    Args:
        bus: CAN bus instance.
        msg: CAN message to send.
        initial_period: Initial period between sends in seconds.
        skew_s: Skew in seconds
        duration: Total duration to run the periodic sending.
    """
    start_time = time.time()  # Track the start time
    elapsed_time = 0  # Track elapsed time
    current_period = initial_period

    while elapsed_time < duration:
        # send_time = time.time()  # Capture the time at the start of the loop

        # Send the CAN message
        try:
            bus.send(msg)
            # print(f"Message sent at {time.time() - start_time:.2f}s")
        except can.CanError as e:
            print(f"Error sending message: {e}")

        # Compute the next period with skew
        current_period = max(
            0, initial_period + skew_s
        )  # Ensure period is non-negative

        # Sleep for the computed period
        time.sleep(current_period)

        # Update elapsed time
        elapsed_time = time.time() - start_time


class CANDevice:
    def __init__(
        self,
        arbitration_id,
        data,
        period,
        channel="can0",
        interface="virtual",
        bitrate=500000,
    ):
        self.channel = channel
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self.task = None
        self.arbitration_id = arbitration_id
        self.data = data
        self.period = period

    def start(self):
        # Initialize the CAN bus
        self.bus = can.Bus(
            channel=self.channel, interface=self.interface, bitrate=self.bitrate
        )

        msg = can.Message(self.arbitration_id, data=self.data, is_extended_id=True)

        thread = threading.Thread(
            target=send_with_dynamic_timing,
            args=(self.bus, msg, 1.0, 0.1, 1000),
            daemon=True,
        )
        self.task = thread
        thread.start()
        # Create a CAN message with the specified data

        # Start sending the message every 5 seconds
        # self.task = can.CyclicSendTask(self.bus, msg, period=self.period)
        # self.task = self.bus.send_periodic(msg, period=self.period)
        # self.task.start()


device1 = CANDevice(arbitration_id=0x123, data=[1, 2, 3], period=1)
device2 = CANDevice(arbitration_id=0x234, data=[4, 5, 6], period=2)
device3 = CANDevice(arbitration_id=0x356, data=[7, 8, 9], period=3)

try:
    bus = can.Bus(channel="can0", interface="virtual", bitrate=500000)
    device1.start()
    # device2.start()
    # device3.start()
    last_message_timestamp = None
    while True:
        try:
            message = bus.recv(timeout=1.0)
            if message:
                timestamp = time.strftime(
                    "%Y-%m-%d %H:%M:%S", time.localtime(message.timestamp)
                )
                print(
                    f"[{timestamp}] [diff: {message.timestamp - last_message_timestamp if last_message_timestamp is not None else 0}] Received: {message}"
                )
                last_message_timestamp = message.timestamp
        except can.CanError as e:
            print(f"Error reading from CAN bus: {e}")
except KeyboardInterrupt:
    print("Stopping CAN device.")
finally:
    bus.shutdown()
