#!/usr/bin/env python3
"""Subscribe to debug_tau and debug_control_mode, record until Ctrl+C, then plot."""

import argparse
import signal
import sys
import threading

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Int32

CONTROL_MODE_NAMES = {0: "CARTESIAN", 1: "JOINT_TRAJECTORY"}


class TauRecorder(Node):
    def __init__(
        self,
        tau_topic: str,
        tau_damping_topic: str,
        tau_total_topic: str,
        tau_commanded_topic: str,
        tau_velocity_limit_topic: str,
        mode_topic: str,
        target_frame_topic: str,
        current_frame_topic: str,
    ):
        super().__init__("tau_recorder")
        self.timestamps: list[float] = []
        self.samples: list[list[float]] = []
        self.damping_timestamps: list[float] = []
        self.damping_samples: list[list[float]] = []
        self.total_timestamps: list[float] = []
        self.total_samples: list[list[float]] = []
        self.commanded_timestamps: list[float] = []
        self.commanded_samples: list[list[float]] = []
        self.mode_timestamps: list[float] = []
        self.mode_values: list[int] = []
        self.target_timestamps: list[float] = []
        self.target_xyz: list[list[float]] = []
        self.current_timestamps: list[float] = []
        self.current_xyz: list[list[float]] = []
        self.t0: float | None = None

        self.create_subscription(Float64MultiArray, tau_topic, self._tau_cb, 10)
        self.create_subscription(
            Float64MultiArray, tau_damping_topic, self._tau_damping_cb, 10
        )
        self.create_subscription(
            Float64MultiArray, tau_total_topic, self._tau_total_cb, 10
        )
        self.create_subscription(
            Float64MultiArray, tau_commanded_topic, self._tau_commanded_cb, 10
        )
        self.create_subscription(Int32, mode_topic, self._mode_cb, 10)
        self.create_subscription(PoseStamped, target_frame_topic, self._target_cb, 10)
        self.create_subscription(PoseStamped, current_frame_topic, self._current_cb, 10)
        self.get_logger().info(
            f"Subscribing to {tau_topic}, {tau_damping_topic}, {tau_total_topic}, {tau_commanded_topic}, {mode_topic}, {target_frame_topic}, and {current_frame_topic} — press Ctrl+C to stop and plot"
        )

    def _stamp(self) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def _tau_cb(self, msg: Float64MultiArray):
        self.timestamps.append(self._stamp())
        self.samples.append(list(msg.data))

    def _tau_damping_cb(self, msg: Float64MultiArray):
        self.damping_timestamps.append(self._stamp())
        self.damping_samples.append(list(msg.data))

    def _tau_total_cb(self, msg: Float64MultiArray):
        self.total_timestamps.append(self._stamp())
        self.total_samples.append(list(msg.data))

    def _tau_commanded_cb(self, msg: Float64MultiArray):
        self.commanded_timestamps.append(self._stamp())
        self.commanded_samples.append(list(msg.data))

    def _mode_cb(self, msg: Int32):
        self.mode_timestamps.append(self._stamp())
        self.mode_values.append(msg.data)

    def _target_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.target_timestamps.append(self._stamp())
        self.target_xyz.append([p.x, p.y, p.z])

    def _current_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.current_timestamps.append(self._stamp())
        self.current_xyz.append([p.x, p.y, p.z])


def _show_scrollable(fig):
    """Display a matplotlib figure in a Tk window with a vertical scrollbar."""
    import tkinter as tk
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

    root = tk.Tk()
    root.title("Tau Plot")

    screen_h = root.winfo_screenheight()
    win_h = min(int(screen_h * 0.85), 900)
    win_w = 1200
    root.geometry(f"{win_w}x{win_h}")

    canvas = tk.Canvas(root)
    scrollbar = tk.Scrollbar(root, orient="vertical", command=canvas.yview)
    canvas.configure(yscrollcommand=scrollbar.set)
    scrollbar.pack(side="right", fill="y")
    canvas.pack(side="left", fill="both", expand=True)

    frame = tk.Frame(canvas)
    canvas.create_window((0, 0), window=frame, anchor="nw")

    fig_canvas = FigureCanvasTkAgg(fig, master=frame)
    fig_canvas.draw()
    fig_canvas.get_tk_widget().pack()

    frame.update_idletasks()
    canvas.configure(scrollregion=canvas.bbox("all"))

    canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-3, "units"))
    canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(3, "units"))

    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="Record and plot tau from debug topic")
    parser.add_argument(
        "--topic",
        default="/combined_impedance_controller_right/debug_tau_stiffness",
        help="Tau topic (default: /combined_impedance_controller_right/debug_tau_stiffness)",
    )
    parser.add_argument(
        "--damping-topic",
        default="/combined_impedance_controller_right/debug_tau_damping",
        help="Tau damping topic (default: /combined_impedance_controller_right/debug_tau_damping)",
    )
    parser.add_argument(
        "--total-topic",
        default="/combined_impedance_controller_right/debug_tau_total",
        help="Tau total topic (default: /combined_impedance_controller_right/debug_tau_total)",
    )
    parser.add_argument(
        "--commanded-topic",
        default="/combined_impedance_controller_right/debug_tau_commanded",
        help="Tau commanded topic (default: /combined_impedance_controller_right/debug_tau_commanded)",
    )
    parser.add_argument(
        "--mode-topic",
        default="/combined_impedance_controller_right/debug_control_mode",
        help="Control mode topic (default: /combined_impedance_controller_right/debug_control_mode)",
    )
    parser.add_argument(
        "--target-frame-topic",
        default="/combined_impedance_controller_right/target_frame",
        help="Target frame topic (default: /combined_impedance_controller_right/target_frame)",
    )
    parser.add_argument(
        "--current-frame-topic",
        default="/combined_impedance_controller_right/debug_current_frame",
        help="Current frame topic (default: /combined_impedance_controller_right/debug_current_frame)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TauRecorder(
        args.topic,
        args.damping_topic,
        args.total_topic,
        args.commanded_topic,
        args.mode_topic,
        args.target_frame_topic,
        args.current_frame_topic,
    )

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

    has_damping = bool(node.damping_samples)
    has_xyz = bool(node.target_xyz) or bool(node.current_xyz)
    n_joints = len(node.samples[0])
    n_xyz = 3 if has_xyz else 0
    n_rows = n_joints + n_xyz
    fig, axes = plt.subplots(
        n_rows, 1, sharex=True, squeeze=False, figsize=(12, 3 * n_rows)
    )

    for j in range(n_joints):
        ax = axes[j, 0]
        ax.plot(
            node.timestamps,
            [s[j] for s in node.samples],
            label="tau stiffness",
        )
        if has_damping:
            ax.plot(
                node.damping_timestamps,
                [s[j] for s in node.damping_samples],
                label="tau damping",
            )
        if node.total_samples:
            ax.plot(
                node.total_timestamps,
                [s[j] for s in node.total_samples],
                label="tau total",
            )
        if node.commanded_samples:
            ax.plot(
                node.commanded_timestamps,
                [s[j] for s in node.commanded_samples],
                label="tau commanded",
            )
        for t, label in zip(transition_times, transition_labels):
            ax.axvline(t, color="k", linestyle="--", alpha=0.6, label=label)

        ax.set_ylabel("Tau [Nm]")
        ax.set_title(f"Joint {j}")
        ax.grid(True)

        # De-duplicate legend entries
        handles, labels = ax.get_legend_handles_labels()
        seen = set()
        unique = [
            (h, l) for h, l in zip(handles, labels) if l not in seen and not seen.add(l)
        ]
        ax.legend(*zip(*unique), loc="upper right")

    # Target vs Current XYZ plots
    if has_xyz:
        axis_names = ["X", "Y", "Z"]
        for i, name in enumerate(axis_names):
            ax = axes[n_joints + i, 0]
            if node.target_xyz:
                ax.plot(
                    node.target_timestamps,
                    [s[i] for s in node.target_xyz],
                    label=f"target {name}",
                )
            if node.current_xyz:
                ax.plot(
                    node.current_timestamps,
                    [s[i] for s in node.current_xyz],
                    label=f"current {name}",
                )
            for t, label in zip(transition_times, transition_labels):
                ax.axvline(t, color="k", linestyle="--", alpha=0.6, label=label)
            ax.set_ylabel(f"{name} [m]")
            ax.set_title(f"Target vs Current — {name}")
            ax.grid(True)
            handles, labels = ax.get_legend_handles_labels()
            seen = set()
            unique = [
                (h, l)
                for h, l in zip(handles, labels)
                if l not in seen and not seen.add(l)
            ]
            ax.legend(*zip(*unique), loc="upper right")

    axes[-1, 0].set_xlabel("Time [s]")
    fig.tight_layout()
    _show_scrollable(fig)


if __name__ == "__main__":
    main()
