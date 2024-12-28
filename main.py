import statistics
import threading
import time

import can


# SkewUpdate function: Recursive Least Squares (RLS) algorithm
def skew_update(t, e, P_prev, S_prev, lam=0.9995):
    """
    Perform the Recursive Least Squares (RLS) update for clock skew estimation.
    :param t: Input data (time interval).
    :param e: Identification error.
    :param P_prev: Previous covariance matrix.
    :param S_prev: Previous skew estimate.
    :param lam: Forgetting factor.
    :return: Updated skew (S[k]) and covariance matrix (P[k]).
    """
    G_k = ((lam**-1) * P_prev * t) / (1 + (lam**-1) * (t**2) * P_prev)

    P_k = (lam**-1) * (P_prev - G_k * t * P_prev)
    S_k = S_prev + G_k * e  # Updated skew
    return S_k, P_k


# Main IDS algorithm
# Paper introduces the condition "message has not been received for a significantly long time",
# but does not elaborate or provide guidelines for deriving the "significantly long time", so this
# condition check will be skipped, and skews will be updated with this time


# To use: call this with every newly received message
def clock_skew_estimation(
    arrival_timestamp, S, P, arrival_timestamps, O_acc, error, mu_T, memory=20
):
    """
    Perform clock skew estimation based on Algorithm 1 (from the paper).
    """
    # Initialization
    arrival_timestamps.append(arrival_timestamp)
    N = len(arrival_timestamps)

    if N <= 1:
        return

    # As per the paper, we are only interested in some K last message transmissions, to estimate a rolling skew, error, etc.
    if N > memory:
        S.pop(0)
        O_acc.pop(0)
        arrival_timestamps.pop(0)
        error.pop(0)
        mu_T.pop(0)
        P.pop(0)

    N = len(arrival_timestamps)

    # Array of distances between messages
    T = []

    for n in range(1, N):
        # Arrival timestamps and intervals
        a_n = arrival_timestamps[n]
        a_prev = arrival_timestamps[n - 1]
        T.append(a_n - a_prev)

    # According to the paper, below we are in step k
    # Average timestamp interval
    mu_T.append(statistics.mean(T))

    # Offset calculation
    O_k = (1 / (N - 1)) * sum(
        arrival_timestamps[i] - (arrival_timestamps[1] + (i - 1) * mu_T[-2])
        for i in range(2, N)
    )

    O_acc.append(O_acc[-1] + abs(O_k))  # Accumulated offset

    # Elapsed time
    T_k = arrival_timestamps[-1] - arrival_timestamps[0]

    # Identification error
    error_k = O_acc[-1] - S[-1] * T_k
    error.append(error_k)

    # Update skew using RLS
    S_k, P_k = skew_update(T_k, error_k, P[-1], S[-1])

    P.append(P_k)
    S.append(S_k)

    return S


def CUSUM(
    prev_L_plus,
    prev_L_minus,
    timestamp,
    error,
    K_param=1.5,
    threshold=5,
):
    if (
        len(error) > 2
        and abs((error[-1] - statistics.mean(error)) / statistics.variance(error)) >= 3
    ):
        return prev_L_plus, prev_L_minus, False

    positive = max(
        0.0,
        prev_L_plus
        + ((error[-1] - statistics.mean(error)) / statistics.variance(error))
        - K_param,
    )
    negative = max(
        0.0,
        prev_L_minus
        - ((error[-1] - statistics.mean(error)) / statistics.variance(error))
        - K_param,
    )

    is_intrusion = False
    if positive >= threshold or negative >= threshold:
        is_intrusion = True
        print(f"Intrusion detected for timestamp {timestamp}")
        print(f"positive: {positive} negative: {negative}")

    return positive, negative, is_intrusion


def send_with_dynamic_timing(bus, msg, initial_period, skew_s, message_count):
    """
    Sends a CAN message periodically with an artificial time skew using a loop.

    Args:
        bus: CAN bus instance.
        msg: CAN message to send.
        initial_period: Initial period between sends in seconds.
        skew_s: Skew in seconds
        duration: Total duration to run the periodic sending.
    """
    current_message_count = 0
    current_period = initial_period

    while current_message_count < message_count:
        try:
            bus.send(msg)
        except can.CanError as e:
            print(f"Error sending message: {e}")

        current_period = max(0, initial_period + skew_s)
        time.sleep(current_period)

        current_message_count += 1


class CANDevice:
    def __init__(
        self,
        messages,
        period,
        skew_per_period,
        message_count=1000,
        channel="can0",
        interface="virtual",
        bitrate=500000,
    ):
        self.channel = channel
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self.messages = messages
        self.period = period
        self.skew_per_period = skew_per_period
        self.message_count = message_count

    def start(self):
        # Initialize local CAN bus instance
        self.bus = can.Bus(
            channel=self.channel, interface=self.interface, bitrate=self.bitrate
        )

        for message in self.messages:
            thread = threading.Thread(
                target=send_with_dynamic_timing,
                args=(
                    self.bus,
                    message,
                    self.period,
                    self.skew_per_period,
                    self.message_count,
                ),
                daemon=True,
            )
            thread.start()


delta_I = 1.0


# Class for storing fingerprint metrics for each arbitration_id
class ArbitrationFingerprint:
    def __init__(self, arbitration_id):
        self.arbitration_id = arbitration_id
        self.skew = [0]  # Initial skew (accumulated)
        self.O_acc = [0]  # Initial accumulated offset
        self.P = [delta_I]  # Initial covariance matrix
        self.mu_T = []  # Mean of message distances (in time)
        self.arrival_timestamps = []
        self.error = []  # Accumulated error over time
        self.L_positive = 0.0  # CUSUM positive boundary
        self.L_negative = 0.0  # CUSUM negative boundary


class CANDeviceListener:
    def __init__(
        self,
        channel="can0",
        interface="virtual",
        bitrate=500000,
    ):
        self.channel = channel
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self.task = None
        self.fingerprint_map = {}
        self.number_of_intrusions = 0

    def start(self):
        # Initialize local CAN bus instance
        self.bus = can.Bus(
            channel=self.channel, interface=self.interface, bitrate=self.bitrate
        )

        while True:
            try:
                message = self.bus.recv(timeout=1.0)
                if message:
                    if message.arbitration_id not in self.fingerprint_map:
                        self.fingerprint_map[message.arbitration_id] = (
                            ArbitrationFingerprint(message.arbitration_id)
                        )

                    fingerprint = self.fingerprint_map[message.arbitration_id]

                    clock_skew_estimation(
                        message.timestamp,
                        fingerprint.skew,
                        fingerprint.P,
                        fingerprint.arrival_timestamps,
                        fingerprint.O_acc,
                        fingerprint.error,
                        fingerprint.mu_T,
                    )

                    if len(fingerprint.error) >= 2:
                        fingerprint.L_positive, fingerprint.L_negative, is_intrusion = (
                            CUSUM(
                                fingerprint.L_positive,
                                fingerprint.L_negative,
                                message.timestamp,
                                fingerprint.error,
                            )
                        )

                        if is_intrusion:
                            self.number_of_intrusions += 1

                    timestamp = time.strftime(
                        "%Y-%m-%d %H:%M:%S", time.localtime(message.timestamp)
                    )

                    # CAUTION: message.str() truncates the timestamp by 1 digit after period (lost accuracy via print)
                    print(f"[{timestamp}] Received: {message}")
                    print(f"Intrusions: {self.number_of_intrusions}")

            except can.CanError as e:
                print(f"Error reading from CAN bus: {e}")


def experiment_1():
    deviceA_messages = [
        can.Message(arbitration_id=0xAA, data=[0, 0, 0], is_extended_id=True)
    ]
    deviceA = CANDevice(deviceA_messages, period=2, skew_per_period=0.01)

    deviceC = CANDeviceListener()

    deviceA.start()
    deviceC.start()


def experiment_2():
    deviceC = CANDeviceListener()

    deviceA_messages = [
        can.Message(arbitration_id=0xAA, data=[0, 0, 0], is_extended_id=True)
    ]

    deviceB_messages = [
        can.Message(arbitration_id=0xFF, data=[1, 1, 1], is_extended_id=True)
    ]

    deviceA = CANDevice(deviceA_messages, period=2, skew_per_period=0.01)
    deviceB = CANDevice(deviceB_messages, period=2, skew_per_period=0.02)
    deviceA.start()
    deviceB.start()

    deviceC.start()


if __name__ == "__main__":
    experiment_1()
