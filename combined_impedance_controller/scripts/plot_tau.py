#!/usr/bin/env python3
"""Subscribe to debug_tau and debug_control_mode, record until Ctrl+C, then plot."""

import argparse
import signal
import sys
import threading

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

CONTROL_MODE_NAMES = {0: "CARTESIAN", 1: "JOINT_TRAJECTORY"}


class TauRecorder(Node):
    def __init__(self, tau_topic: str, mode_topic: str):
        super().__init__("tau_recorder")
        self.timestamps: list[float] = []
        self.samples: list[list[float]] = []
        self.mode_timestamps: list[float] = []
        self.mode_values: list[int] = []
        self.t0: float | None = None

        self.create_subscription(Float64MultiArray, tau_topic, self._tau_cb, 10)
        self.create_subscription(Int32, mode_topic, self._mode_cb, 10)
        self.get_logger().info(
            f"Subscribing to {tau_topic} and {mode_topic} — press Ctrl+C to stop and plot"
        )

    def _stamp(self) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def _tau_cb(self, msg: Float64MultiArray):
        self.timestamps.append(self._stamp())
        self.samples.append(list(msg.data))

    def _mode_cb(self, msg: Int32):
        self.mode_timestamps.append(self._stamp())
        self.mode_values.append(msg.data)


def main():
    parser = argparse.ArgumentParser(description="Record and plot tau from debug topic")
    parser.add_argument(
        "--topic",
        default="/combined_impedance_controller/debug_tau",
        help="Tau topic (default: /combined_impedance_controller/debug_tau)",
    )
    parser.add_argument(
        "--mode-topic",
        default="/combined_impedance_controller/debug_control_mode",
        help="Control mode topic (default: /combined_impedance_controller/debug_control_mode)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TauRecorder(args.topic, args.mode_topic)

    # Spin in a thread so the main thread can catch Ctrl+C cleanly
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    stop.wait()

    node.destroy_node()
    rclpy.try_shutdown()

    if not node.samples:
        print("No data recorded.")
        sys.exit(0)

    # Find mode transition times (where value changes from previous)
    transition_times: list[float] = []
    transition_labels: list[str] = []
    for i, (t, v) in enumerate(zip(node.mode_timestamps, node.mode_values)):
        if i == 0 or v != node.mode_values[i - 1]:
            transition_times.append(t)
            transition_labels.append(CONTROL_MODE_NAMES.get(v, f"MODE_{v}"))

    n_axes = len(node.samples[0])
    fig, ax = plt.subplots()
    for i in range(n_axes):
        ax.plot(node.timestamps, [s[i] for s in node.samples], label=f"joint {i}")

    for t, label in zip(transition_times, transition_labels):
        ax.axvline(t, color="k", linestyle="--", alpha=0.6, label=label)

    # De-duplicate legend entries
    handles, labels = ax.get_legend_handles_labels()
    seen = set()
    unique = [
        (handle, label)
        for handle, label in zip(handles, labels)
        if label not in seen and not seen.add(label)
    ]
    ax.legend(*zip(*unique))

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Tau [Nm]")
    ax.set_title("Joint torques over time")
    ax.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
