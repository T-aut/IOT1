import time
import can
import random
import threading
import statistics

# The accumulated clock offset is derived for each message ID (not device)
# Clock offset p (p > 0.8) means the messages are highly correlated


# SkewUpdate function: Recursive Least Squares (RLS) algorithm
# ✅ Verbatum from paper


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
    # G_k = P_prev / (lam + P_prev * t**2)  # Gain
    G_k = ((lam**-1) * P_prev * t) / (1 + (lam**-1) * (t**2) * P_prev)

    # P_k = (1 / lam) * (P_prev - G_k * t**2 * P_prev)  # Updated covariance
    P_k = (lam**-1) * (P_prev - G_k * t * P_prev)
    S_k = S_prev + G_k * e  # Updated skew
    return S_k, P_k


delta_I = 1.0
S = [0]  # Initial skew (accumulated)
O_acc = [0]  # Initial accumulated offset
P = [delta_I]  # Initial covariance matrix
mu_T = []
arrival_timestamps = []
error = []

# Main IDS algorithm
# Paper introduces the condition "message has not been received for a significantly long time",
# but does not elaborate or provide guidelines for deriving the "significantly long time", so this
# condition check will be skipped, and skews will be updated with this time


# To use: call this with every newly received message
def clock_skew_estimation(arrival_timestamp, memory=20):
    """
    Perform clock skew estimation based on Algorithm 1.
    :param arrival_timestamps: List of message arrival timestamps. MUST BE AT LEAST 2
    :param delta_I: Initial covariance value.
    :return: Estimated skew values.
    """
    # Initialization
    arrival_timestamps.append(arrival_timestamp)
    N = len(arrival_timestamps)

    if N <= 1:
        return

    # # As per the paper, we are only interested in some K last message transmissions, to estimate a rolling skew, error, etc.
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
        # Tn = a_k - a_prev  # Timestamp interval
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

    # if (
    #     len(error) > 2
    #     and abs((error[-1] - statistics.mean(error)) / statistics.variance(error)) < 3
    # ):
    #     print("Error within boundary")
    # elif len(error) > 2:
    #     print(
    #         f"Error too large: {abs((error[-1] - statistics.mean(error)) / statistics.variance(error)) }"
    #     )

    P.append(P_k)
    S.append(S_k)

    return S


def CUSUM(prev_L_plus, prev_L_minus, timestamp, K_param=1.5, threshold=5):
    if (
        len(error) > 2
        and abs((error[-1] - statistics.mean(error)) / statistics.variance(error)) >= 3
    ):
        return prev_L_plus, prev_L_minus

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

    if positive >= threshold or negative >= threshold:
        print(f"Intrusion detected for timestamp {timestamp}")
        print(f"positive: {positive} negative: {negative}")

    return positive, negative


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
        skew_per_period,
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
        self.skew_per_period = skew_per_period

    def start(self):
        # Initialize local CAN bus instance
        self.bus = can.Bus(
            channel=self.channel, interface=self.interface, bitrate=self.bitrate
        )

        msg = can.Message(
            arbitration_id=self.arbitration_id, data=self.data, is_extended_id=True
        )

        thread = threading.Thread(
            target=send_with_dynamic_timing,
            args=(self.bus, msg, self.period, self.skew_per_period, 1000),
            daemon=True,
        )
        self.task = thread
        thread.start()


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
        self.skew_map = {}  # This store the couple [ ID | Estimated CK (Clock Skews) ]
        self.skew_offset_map = {}  # This store the couple [ ID | Acc Offset ] 

    def start(self):
        # Initialize local CAN bus instance
        self.bus = can.Bus(
            channel=self.channel, interface=self.interface, bitrate=self.bitrate
        )

        L_plus = 0.0
        L_minus = 0.0

        while True:
            try:
                message = self.bus.recv(timeout=1.0)
                if message:
                    # store the skew estimation value here 
                    skew_est_value = clock_skew_estimation(message.timestamp) 

                    if skew_est_value:
                        # store couple ID <-> Est Skew
                        if message.arbitration_id not in self.skew_map:
                            self.skew_map[message.arbitration_id] = []
                        self.skew_map[message.arbitration_id].append(skew_est_value[-1])

                        # store couple ID <-> Offset
                        if message.arbitration_id not in self.skew_offset_map:
                            self.skew_offset_map[message.arbitration_id] = []
                        self.skew_offset_map[message.arbitration_id].append(O_acc[-1])

                    if len(error) >= 2:
                        L_plus, L_minus = CUSUM(L_plus, L_minus, message.timestamp)

                    timestamp = time.strftime(
                        "%Y-%m-%d %H:%M:%S", time.localtime(message.timestamp)
                    )

                    # CAUTION: message.str() truncates the timestamp by 1 digit after period (lost accuracy via print)
                    print(f"[{timestamp}] Received: {message}")

                    # Added for Logging
                    if self.skew_map and self.skew_offset_map:
                        print(f"ID: [{message.arbitration_id}] | Skews : {self.skew_map[message.arbitration_id]}")
                        print(f"ID: [{message.arbitration_id}] | Offsets : {self.skew_offset_map[message.arbitration_id]}")


                    # We should include the logic for finger printing here

            except can.CanError as e:
                print(f"Error reading from CAN bus: {e}")


# try:
#     bus = can.Bus(channel="can0", interface="virtual", bitrate=500000)
#     device1.start()
#     device2.start()
#     # device3.start()
#     last_message_timestamp = None
#     while True:
#         try:
#             message = bus.recv(timeout=1.0)

#             if message:
#                 clock_skew_estimation(message.timestamp)
#                 if len(S) >= 2:
#                     L_plus, L_minus = CUSUM(L_plus, L_minus, message.timestamp)

#                 timestamp = time.strftime(
#                     "%Y-%m-%d %H:%M:%S", time.localtime(message.timestamp)
#                 )
#                 print(
#                     f"[{timestamp}] [diff: {message.timestamp - last_message_timestamp if last_message_timestamp is not None else 0}] Received: {message}"
#                 )
#                 last_message_timestamp = message.timestamp
#         except can.CanError as e:
#             print(f"Error reading from CAN bus: {e}")
# except KeyboardInterrupt:
#     print("Stopping CAN device.")
# finally:
#     bus.shutdown()

# Simulated example
if __name__ == "__main__":
    device1 = CANDeviceListener()

    deviceA = CANDevice(
        arbitration_id=0xAA, data=[0, 0, 0], period=2, skew_per_period=0.01
    )
    deviceA.start()
    # Testing figerprint 
    deviceB = CANDevice(
        arbitration_id=0xFF, data=[1, 1, 1], period=2, skew_per_period=0.02
    )
    deviceB.start()

    
    
    device1.start()
