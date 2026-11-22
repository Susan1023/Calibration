#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters


PARAM_GROUPS = {
    "Visualization": [
        ("point_size", "int", 1, 8, 1),
        ("min_depth", "double", 0.1, 10.0, 0.1),
        ("max_depth", "double", 1.0, 200.0, 0.5),
        ("show_statistics", "bool", None, None, None),
        ("show_edge_overlay", "bool", None, None, None),
        ("show_depth_colorbar", "bool", None, None, None),
    ],
    "Performance": [
        ("enable_downsampling", "bool", None, None, None),
        ("max_points", "int", 10000, 2000000, 10000),
        ("enable_accumulation", "bool", None, None, None),
        ("accumulation_time_sec", "double", 0.01, 2.0, 0.01),
        ("accumulation_frames", "int", 1, 30, 1),
        ("accumulation_max_points", "int", 1000, 3000000, 1000),
    ],
    "Filtering": [
        ("filter_by_distance", "bool", None, None, None),
        ("min_distance", "double", 0.1, 5.0, 0.1),
        ("max_distance", "double", 1.0, 300.0, 1.0),
    ],
    "Metrics": [
        ("enable_metrics", "bool", None, None, None),
        ("edge_threshold", "int", 10, 300, 1),
        ("edge_distance_threshold", "double", 1.0, 30.0, 0.1),
        ("min_points_for_nmi", "int", 10, 3000, 10),
        ("metrics_compute_interval", "int", 1, 10, 1),
    ],
    "Sync": [
        ("latest_pair_time_threshold", "double", 0.01, 2.0, 0.01),
        ("render_rate_hz", "int", 1, 120, 1),
        ("projection_cache_ttl_sec", "double", 0.01, 2.0, 0.01),
    ],
}


class ParamPanelNode(Node):
    def __init__(self, target_node: str):
        super().__init__("validator_param_panel")
        self.target_node = target_node
        self.get_client = self.create_client(GetParameters, f"{target_node}/get_parameters")
        self.set_client = self.create_client(SetParameters, f"{target_node}/set_parameters")

    def get_parameters_async(self, names):
        req = GetParameters.Request()
        req.names = list(names)
        return self.get_client.call_async(req)

    def set_parameters_async(self, params):
        req = SetParameters.Request()
        req.parameters = [p.to_parameter_msg() for p in params]
        return self.set_client.call_async(req)


class ParamPanelApp:
    def __init__(self, root: tk.Tk, ros_node: ParamPanelNode):
        self.root = root
        self.ros_node = ros_node
        self.controls = {}
        self.status_var = tk.StringVar(value=f"Connecting to {ros_node.target_node} ...")

        self.root.title("LiDAR-Camera Validator Parameter Panel")
        self.root.geometry("760x760")

        header = ttk.Frame(self.root)
        header.pack(fill=tk.X, padx=12, pady=8)
        ttk.Label(header, text="Real-time Parameter Panel", font=("Arial", 13, "bold")).pack(side=tk.LEFT)
        ttk.Button(header, text="Refresh", command=self.refresh_all).pack(side=tk.RIGHT)

        ttk.Label(self.root, textvariable=self.status_var, foreground="#1f6f8b").pack(
            fill=tk.X, padx=12, pady=(0, 8)
        )

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=12, pady=6)

        for group_name, params in PARAM_GROUPS.items():
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=group_name)
            self._build_group(frame, params)

        bottom = ttk.Frame(self.root)
        bottom.pack(fill=tk.X, padx=12, pady=10)
        ttk.Button(bottom, text="Reset To Defaults", command=self.reset_defaults).pack(side=tk.LEFT)
        ttk.Button(bottom, text="Quit", command=self.root.destroy).pack(side=tk.RIGHT)

        self.root.after(800, self.refresh_all)

    def _build_group(self, frame: ttk.Frame, params):
        canvas = tk.Canvas(frame, highlightthickness=0)
        scroll = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)

        inner.bind(
            "<Configure>",
            lambda _e: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=scroll.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)

        for row, (name, ptype, pmin, pmax, pstep) in enumerate(params):
            ttk.Label(inner, text=name, width=28).grid(row=row, column=0, sticky="w", padx=8, pady=6)

            if ptype == "bool":
                var = tk.BooleanVar(value=False)
                chk = ttk.Checkbutton(
                    inner,
                    variable=var,
                    command=lambda n=name, v=var: self.set_param(n, bool(v.get()), "bool"),
                )
                chk.grid(row=row, column=1, sticky="w", padx=8, pady=6)
                self.controls[name] = ("bool", var)
                continue

            if ptype == "int":
                var = tk.IntVar(value=pmin)
            else:
                var = tk.DoubleVar(value=pmin)

            spin = tk.Spinbox(
                inner,
                from_=pmin,
                to=pmax,
                increment=pstep,
                textvariable=var,
                width=12,
            )
            spin.grid(row=row, column=1, sticky="w", padx=8, pady=6)
            ttk.Button(
                inner,
                text="Set",
                command=lambda n=name, t=ptype, v=var: self.set_param(n, v.get(), t),
            ).grid(row=row, column=2, sticky="w", padx=8, pady=6)
            self.controls[name] = (ptype, var)

    def _param_to_typed(self, value, ptype):
        if ptype == "bool":
            return bool(value)
        if ptype == "int":
            return int(float(value))
        return float(value)

    def set_param(self, name: str, value, ptype: str):
        typed_value = self._param_to_typed(value, ptype)
        parameter = Parameter(name=name, value=typed_value)
        future = self.ros_node.set_parameters_async([parameter])
        future.add_done_callback(lambda f: self._on_set_done(name, typed_value, f))

    def _on_set_done(self, name: str, value, future):
        try:
            response = future.result()
            results = response.results if response else []
            if results and results[0].successful:
                self.status_var.set(f"Set {name}={value}")
            else:
                reason = results[0].reason if results else "unknown"
                self.status_var.set(f"Set failed: {name}, reason={reason}")
        except Exception as exc:
            self.status_var.set(f"Set failed: {name}, error={exc}")

    def refresh_all(self):
        names = list(self.controls.keys())
        future = self.ros_node.get_parameters_async(names)
        future.add_done_callback(lambda f: self._on_refresh_done(names, f))

    def _on_refresh_done(self, names, future):
        try:
            response = future.result()
            values = response.values if response else []
            for name, value in zip(names, values):
                ptype, var = self.controls[name]
                if ptype == "bool":
                    var.set(bool(value.bool_value))
                elif ptype == "int":
                    var.set(int(value.integer_value))
                else:
                    var.set(float(value.double_value))
            self.status_var.set(f"Connected: {self.ros_node.target_node}")
        except Exception as exc:
            self.status_var.set(f"Refresh failed: {exc}")

    def reset_defaults(self):
        self.set_param("reset_to_defaults", True, "bool")
        self.root.after(500, self.refresh_all)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", default="/validator_node", help="Target node name")
    args = parser.parse_args()

    rclpy.init()
    ros_node = ParamPanelNode(args.node)
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    if not ros_node.get_client.wait_for_service(timeout_sec=5.0):
        ros_node.get_logger().warning("get_parameters service not available yet")
    if not ros_node.set_client.wait_for_service(timeout_sec=5.0):
        ros_node.get_logger().warning("set_parameters service not available yet")

    root = tk.Tk()
    app = ParamPanelApp(root, ros_node)
    try:
        root.mainloop()
    finally:
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
