#!/usr/bin/env python3
"""Preview and explicitly submit simulation targets in the CRA base_link frame."""

import argparse
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def load_workspace(path):
    """Read the active Planning generator's meter-valued EOF samples."""
    with np.load(path, allow_pickle=False) as data:
        if not {"eof", "coordinate_frame"}.issubset(data.files):
            raise ValueError("Workspace requires eof and coordinate_frame arrays")
        frame = data["coordinate_frame"]
        if frame.shape != () or str(frame.item()) != "base_link":
            raise ValueError("Workspace coordinate_frame must be base_link")
        points = np.asarray(data["eof"], dtype=float).copy()

    if (points.ndim != 2 or points.shape[1] != 3 or len(points) == 0
            or not np.isfinite(points).all()):
        raise ValueError("Workspace eof must contain finite (N, 3) positions")
    return points


class TargetGuiNode(Node):
    """Own ROS communication; never publish from feedback or preview events."""

    FEEDBACK_TIMEOUT = 2.0
    CONFIRMATION_TIMEOUT = 5.0

    def __init__(self):
        super().__init__("cra_target_control")
        self.current_target = None
        self.last_feedback = None
        self.feedback_version = 0
        self.feedback_error = None
        self.pending_target = None
        self.pending_since = None
        self.command_status = ""
        self.target_pub = self.create_publisher(
            Point, "/simulation/set_target", 10,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_sub = self.create_subscription(
            Marker, "/simulation/target_marker", self.target_callback,
            marker_qos,
        )

    def target_callback(self, message):
        """Accept only finite simulation feedback in the verified frame."""
        position = message.pose.position
        target = np.array([position.x, position.y, position.z], dtype=float)
        if (message.action != Marker.ADD
                or message.header.frame_id != "base_link"
                or not np.isfinite(target).all()):
            self.feedback_error = "Invalid target feedback (expected base_link)"
            return
        self.feedback_error = None
        changed = (self.current_target is None
                   or not np.array_equal(target, self.current_target))
        self.current_target = target
        self.last_feedback = time.monotonic()
        if changed:
            self.feedback_version += 1
        if (self.pending_target is not None
                and np.allclose(target, self.pending_target,
                                atol=1e-4, rtol=0.0)):
            self.command_status = "Target update confirmed (feedback matches)"
            self.pending_target = None
            self.pending_since = None

    def connection_status(self):
        """Require a command subscriber and recent live target feedback."""
        if not rclpy.ok():
            return "ROS 2 connection unavailable"
        if self.feedback_error:
            return self.feedback_error
        if self.last_feedback is None:
            return "Waiting for simulation"
        if (time.monotonic() - self.last_feedback > self.FEEDBACK_TIMEOUT
                or self.count_publishers("/simulation/target_marker") == 0
                or self.target_pub.get_subscription_count() == 0):
            return "ROS 2 connection unavailable (missing or stale simulation)"
        return None

    def send_target(self, target):
        """Publish exactly once; confirmation is reserved for later feedback."""
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("Target must have three finite coordinates")
        unavailable = self.connection_status()
        if unavailable:
            raise RuntimeError(unavailable)
        self.target_pub.publish(Point(
            x=float(target[0]), y=float(target[1]), z=float(target[2]),
        ))
        self.pending_target = target.copy()
        self.pending_since = time.monotonic()
        self.command_status = "Target command sent; waiting for confirmation"

    def check_confirmation_timeout(self):
        """Expire an unconfirmed command without retrying it."""
        if (self.pending_since is not None
                and time.monotonic() - self.pending_since
                > self.CONFIRMATION_TIMEOUT):
            self.command_status = "Target command sent; confirmation timed out"
            self.pending_target = None
            self.pending_since = None


class TargetGUI:
    """Keep a shared local preview for the XY and XZ mouse projections."""

    def __init__(self, root, node, workspace_path, sample_warning_distance):
        self.root = root
        self.node = node
        self.workspace = load_workspace(workspace_path)
        self.minimum = self.workspace.min(axis=0)
        self.maximum = self.workspace.max(axis=0)
        self.sample_warning_distance = sample_warning_distance
        self.preview = None
        self.drag_axes = None
        self.feedback_version = -1
        self.command_status = ""
        self.was_connected = False
        self.entry_snapshot = ("", "", "")
        self.timer = None
        self.closed = False

        root.title("CRA Target Control")
        root.minsize(860, 680)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)
        ttk.Label(root, text="CRA Target Control — base_link (meters)",
                  font=("TkDefaultFont", 14, "bold")).grid(
                      row=0, column=0, sticky="w", padx=12, pady=8)

        figure = Figure(figsize=(10, 4.5), dpi=100, constrained_layout=True)
        self.top = figure.add_subplot(121)
        self.side = figure.add_subplot(122)
        self.canvas = FigureCanvasTkAgg(figure, master=root)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
        self.current_markers = []
        self.preview_markers = []
        # Boundaries and proximity checks always use all samples.
        indices = np.random.default_rng(42).choice(
            len(self.workspace), min(12000, len(self.workspace)), replace=False,
        )
        visible = self.workspace[indices]
        padding = np.maximum((self.maximum - self.minimum) * 0.04, 0.001)
        for axes, horizontal, vertical, title in (
            (self.top, 1, 0, "Top View (XY)"),
            (self.side, 0, 2, "Side View (XZ)"),
        ):
            axes.scatter(visible[:, horizontal], visible[:, vertical],
                         s=2, alpha=0.12, color="#47799c", rasterized=True)
            axes.set_title(title)
            axes.set_xlabel(f"{'XYZ'[horizontal]} (m)")
            axes.set_ylabel(f"{'XYZ'[vertical]} (m)")
            axes.set_xlim(self.minimum[horizontal] - padding[horizontal],
                          self.maximum[horizontal] + padding[horizontal])
            axes.set_ylim(self.minimum[vertical] - padding[vertical],
                          self.maximum[vertical] + padding[vertical])
            axes.set_aspect("equal", adjustable="box")
            axes.grid(alpha=0.2)
            current, = axes.plot([], [], "o", color="#555555", markersize=9,
                                 label="Current Target", zorder=3)
            preview, = axes.plot([], [], "D", color="#e67e22", markersize=9,
                                 markeredgecolor="black", picker=10,
                                 label="Preview Target", zorder=4)
            self.current_markers.append(current)
            self.preview_markers.append(preview)
            axes.legend(loc="upper right", fontsize=8)
        # FrameTransform maps MuJoCo (x, y, z) to CRA (x, -z, y).
        # With +X forward/up on a top view and +Z toward the viewer,
        # the right-handed base_link frame puts +Y on the left.
        self.top.invert_xaxis()
        self.top.set_xlabel("Y (m)   +Y left / -Y right")
        self.top.set_ylabel("X (m)   +X forward")
        self.side.set_ylabel("Z (m)   +Z up")
        self.canvas.mpl_connect("button_press_event", self.on_press)
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        root.bind("<FocusOut>", self.cancel_drag)

        controls = ttk.Frame(root, padding=12)
        controls.grid(row=2, column=0, sticky="ew")
        controls.columnconfigure(2, weight=1)
        current_box = ttk.LabelFrame(controls, text="Current Target", padding=8)
        current_box.grid(row=0, column=0, sticky="ns", padx=(0, 12))
        self.current_text = tk.StringVar(value="X: — m\nY: — m\nZ: — m")
        ttk.Label(current_box, textvariable=self.current_text).grid()
        preview_box = ttk.LabelFrame(controls, text="Preview Target", padding=8)
        preview_box.grid(row=0, column=1, sticky="ns", padx=(0, 12))
        self.entries = []
        for index, axis in enumerate("XYZ"):
            ttk.Label(preview_box, text=f"{axis} (m)").grid(row=index, column=0)
            value = tk.StringVar()
            entry = ttk.Entry(preview_box, textvariable=value, width=12)
            entry.grid(row=index, column=1, padx=6, pady=1)
            entry.bind("<Return>", self.apply_entries)
            self.entries.append(value)
        buttons = ttk.Frame(controls)
        buttons.grid(row=0, column=2, sticky="w")
        ttk.Button(buttons, text="Apply Preview", command=self.apply_entries
                   ).grid(row=0, column=0, padx=4, pady=4)
        self.reset_button = ttk.Button(buttons, text="RESET", command=self.reset)
        self.reset_button.grid(row=0, column=1, padx=4, pady=4)
        self.set_button = ttk.Button(buttons, text="SET TARGET",
                                     command=self.set_target)
        self.set_button.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4)
        self.status = tk.StringVar(value="Waiting for simulation")
        ttk.Label(controls, textvariable=self.status, wraplength=820).grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(10, 2))
        self.validity = tk.StringVar(value="Waiting for target feedback")
        ttk.Label(controls, textvariable=self.validity, foreground="#915300",
                  wraplength=820).grid(row=2, column=0, columnspan=3, sticky="w")
        ttk.Label(controls, text=(
            "Drag the orange marker, or edit coordinates and Apply Preview. "
            "Only SET TARGET sends a command.\n"
            "Workspace samples are visual references; reachability, collisions "
            "and paths remain Planning's responsibility."
        ), wraplength=820).grid(
            row=3, column=0, columnspan=3, sticky="w", pady=(6, 0),
        )
        ttk.Label(controls, text=f"Workspace: {workspace_path}",
                  wraplength=820).grid(row=4, column=0, columnspan=3, sticky="w")
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.poll_ros()

    def preview_validity(self):
        """Check bounds and report sampling distance without claiming reachability."""
        if self.preview is None:
            return False, "Waiting for target feedback"
        if not np.isfinite(self.preview).all():
            return False, "Target must have three finite coordinates"
        if np.any(self.preview < self.minimum) or np.any(
                self.preview > self.maximum):
            return False, "Target outside workspace bounds"
        distance = float(np.sqrt(np.min(np.sum(
            (self.workspace - self.preview) ** 2, axis=1,
        ))))
        if distance > self.sample_warning_distance:
            return True, (
                f"Warning: target is {distance:.3f} m from the nearest sample "
                f"(warning threshold {self.sample_warning_distance:.3f} m); "
                "reachability is unverified."
            )
        return True, (f"Nearest sample: {distance:.3f} m. "
                      "Reachability is still unverified.")

    def refresh_preview(self):
        """Update both projections and coordinate fields from one preview."""
        if self.preview is None:
            return
        x, y, z = self.preview
        for marker, horizontal, vertical in (
            (self.preview_markers[0], y, x),
            (self.preview_markers[1], x, z),
        ):
            marker.set_data([horizontal], [vertical])
        self.entry_snapshot = tuple(f"{value:.3f}" for value in self.preview)
        for variable, value in zip(self.entries, self.entry_snapshot):
            variable.set(value)
        _, message = self.preview_validity()
        self.validity.set(message)
        self.canvas.draw_idle()

    def apply_entries(self, event=None):
        """Commit numeric edits to preview only, preserving unchanged precision."""
        if self.preview is None:
            self.status.set("Waiting for simulation")
            return False
        try:
            target = self.preview.copy()
            for index, (variable, previous) in enumerate(zip(
                    self.entries, self.entry_snapshot)):
                if variable.get() != previous:
                    target[index] = float(variable.get())
            if not np.isfinite(target).all():
                raise ValueError("Nonfinite target")
        except ValueError:
            self.status.set("Enter three finite numeric coordinates")
            return False
        self.preview = target
        self.refresh_preview()
        self.status.set("Target preview updated")
        return True

    def on_press(self, event):
        """Start dragging only when the preview marker is selected."""
        self.drag_axes = None
        if (event.button != 1 or self.preview is None
                or event.inaxes not in (self.top, self.side)):
            return
        index = 0 if event.inaxes is self.top else 1
        if self.preview_markers[index].contains(event)[0]:
            self.drag_axes = event.inaxes

    def on_motion(self, event):
        """Change only the coordinates represented by the selected axes."""
        buttons = getattr(event, "buttons", None)
        if buttons is not None and 1 not in buttons:
            self.drag_axes = None
        if (self.drag_axes is None or event.inaxes is not self.drag_axes
                or event.xdata is None or event.ydata is None
                or not np.isfinite([event.xdata, event.ydata]).all()):
            return
        if self.drag_axes is self.top:
            self.preview[0], self.preview[1] = event.ydata, event.xdata
        else:
            self.preview[0], self.preview[2] = event.xdata, event.ydata
        self.refresh_preview()
        self.status.set("Target preview updated")

    def on_release(self, event):
        """End dragging even when released outside the plot."""
        self.drag_axes = None

    def cancel_drag(self, event=None):
        """Avoid a stuck drag after the window loses focus."""
        self.drag_axes = None

    def reset(self):
        """Copy the last simulation target without publishing."""
        if self.node.current_target is not None:
            self.preview = self.node.current_target.copy()
            self.refresh_preview()
            self.status.set("Target preview reset to Current Target")

    def set_target(self):
        """Validate the preview and explicitly publish one target command."""
        if not self.apply_entries():
            return
        valid, message = self.preview_validity()
        if not valid:
            self.status.set(message)
            return
        try:
            self.node.send_target(self.preview)
        except (ValueError, RuntimeError) as exc:
            self.status.set(str(exc))
            return
        self.command_status = self.node.command_status
        self.status.set(self.command_status)

    def poll_ros(self):
        """Process ROS callbacks without a thread or a blocking Tk callback."""
        if self.closed:
            return
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.0)
            self.node.check_confirmation_timeout()
            if self.feedback_version != self.node.feedback_version:
                self.feedback_version = self.node.feedback_version
                current = self.node.current_target
                if current is not None:
                    self.current_text.set("\n".join(
                        f"{axis}: {value:.3f} m"
                        for axis, value in zip("XYZ", current)
                    ))
                    self.current_markers[0].set_data([current[1]], [current[0]])
                    self.current_markers[1].set_data([current[0]], [current[2]])
                    if self.preview is None:
                        self.preview = current.copy()
                        self.refresh_preview()
                    self.canvas.draw_idle()
            unavailable = self.node.connection_status()
            self.set_button.state(
                ["disabled"] if unavailable or self.preview is None else ["!disabled"]
            )
            self.reset_button.state(
                ["disabled"] if self.node.current_target is None else ["!disabled"]
            )
            if unavailable:
                self.status.set(unavailable)
                self.was_connected = False
            else:
                if not self.was_connected:
                    self.status.set("Ready")
                if self.command_status != self.node.command_status:
                    self.command_status = self.node.command_status
                    self.status.set(self.command_status)
                self.was_connected = True
        except Exception as exc:
            self.set_button.state(["disabled"])
            self.status.set(f"ROS 2 connection unavailable: {exc}")
            self.was_connected = False
        self.timer = self.root.after(30, self.poll_ros)

    def close(self):
        """Stop the timer before destroying the Tk window."""
        self.closed = True
        if self.timer is not None:
            self.root.after_cancel(self.timer)
        self.root.destroy()


def main(args=None):
    """Run as ros2 run simulation target_gui [--workspace FILE]."""
    argv = sys.argv if args is None else [sys.argv[0], *args]
    parser = argparse.ArgumentParser(description="CRA Target Control")
    # Match cra.launch.py rather than silently choosing another workspace.
    parser.add_argument(
        "--workspace", type=Path,
        default=Path.home() / "living-lab/cra_ws/src/planning/data/raw_workspace.npz",
        help="Use the same raw_workspace.npz as the running Planning node",
    )
    parser.add_argument(
        "--sample-warning-distance", type=float, default=0.03,
        help="Visual sampling-distance warning in meters (default: 0.03)",
    )
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    if (not np.isfinite(options.sample_warning_distance)
            or options.sample_warning_distance <= 0):
        parser.error("--sample-warning-distance must be finite and positive")
    node = None
    root = None
    rclpy.init(args=argv[1:])
    try:
        node = TargetGuiNode()
        root = tk.Tk()
        TargetGUI(root, node, options.workspace.expanduser().resolve(),
                  options.sample_warning_distance)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, tk.TclError) as exc:
        parser.exit(1, f"CRA Target Control: {exc}\n")
    finally:
        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
