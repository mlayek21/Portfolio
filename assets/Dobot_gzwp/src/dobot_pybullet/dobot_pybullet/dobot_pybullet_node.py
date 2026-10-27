#!/usr/bin/env python3
"""
Dobot Nova 5 PyBullet High-Performance Physics Simulator & Visualizer (macOS Native)
Subscribes to ROS 2 joint trajectories and gripper commands, simulates rigid-body
dynamics and contact mechanics at 240 Hz, and streams real-time joint states and 3D visual trace lines.
"""

import math
import os
import sys
import time
from typing import List, Optional

import numpy as np
import pybullet as p
import pybullet_data
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DobotPybulletNode(Node):
    """
    ROS 2 PyBullet Physics Simulator Node for 6-axis Dobot Nova 5 Manipulator.
    """

    ARM_JOINT_NAMES = [
        "joint1", "joint2", "joint3",
        "joint4", "joint5", "joint6"
    ]

    GRIPPER_JOINT_NAMES = [
        "left_finger_joint", "right_finger_joint"
    ]

    ALL_JOINT_NAMES = ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES

    HOME_POSITIONS = [-0.75, -0.30, 0.50, -0.20, 0.40, 0.0]

    # Predefined high-contrast, vivid trace color palettes for daylight workcell
    TRACE_PALETTES = {
        "crimson": [0.95, 0.12, 0.20],   # Radiant Electric Crimson Red
        "red":     [0.95, 0.12, 0.20],
        "emerald": [0.05, 0.85, 0.35],   # Vivid Neon Emerald Green
        "green":   [0.05, 0.85, 0.35],
        "azure":   [0.0, 0.48, 0.95],    # Electric Cobalt Azure Blue
        "blue":    [0.0, 0.48, 0.95],
        "purple":  [0.68, 0.18, 0.96],   # Vivid Electric Purple / Violet
        "violet":  [0.68, 0.18, 0.96],
        "magenta": [0.95, 0.15, 0.75],   # High-Def Vivid Magenta
        "amber":   [1.0, 0.55, 0.05],    # Radiant Sunset Amber / Orange
        "orange":  [1.0, 0.55, 0.05],
        "cyan":    [0.0, 0.92, 0.95],    # Glowing Laser Cyan
        "gold":    [0.98, 0.82, 0.10],   # Bright Golden Yellow
        "yellow":  [0.98, 0.82, 0.10],
        "coral":   [1.0, 0.35, 0.45],    # Vivid Coral Pink
        "pink":    [1.0, 0.35, 0.45],
    }

    # Distinct high-contrast color sequence cycled across consecutive trajectory phases
    PHASE_CYCLE_COLORS = [
        ("Crimson Red",     [0.95, 0.12, 0.20]),
        ("Electric Azure",  [0.0, 0.48, 0.95]),
        ("Emerald Green",   [0.05, 0.85, 0.35]),
        ("Vivid Purple",    [0.68, 0.18, 0.96]),
        ("Sunset Amber",    [1.0, 0.55, 0.05]),
        ("Laser Cyan",      [0.0, 0.92, 0.95]),
        ("Coral Pink",      [1.0, 0.35, 0.45]),
    ]

    def __init__(self):
        super().__init__("dobot_pybullet_node")

        self.declare_parameter("gui", True)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("sample_dt", 0.02)  # 50 Hz ROS 2 loop (20 ms)
        self.declare_parameter("window_width", 2560)  # High Definition QHD / Retina resolution
        self.declare_parameter("window_height", 1440)
        self.declare_parameter("trace_dwell_time", 0.5)  # Dwell in seconds before erasing path trace
        self.declare_parameter("ribbon_width", 4.0)
        self.declare_parameter("trace_color", "cycle")  # 'cycle', 'crimson', 'emerald', 'azure', 'purple', 'amber', 'cyan'

        self.use_gui = self.get_parameter("gui").get_parameter_value().bool_value
        self.dt = self.get_parameter("sample_dt").get_parameter_value().double_value
        self.window_width = self.get_parameter("window_width").get_parameter_value().integer_value
        self.window_height = self.get_parameter("window_height").get_parameter_value().integer_value
        self.trace_dwell_time = self.get_parameter("trace_dwell_time").get_parameter_value().double_value
        self.ribbon_width = self.get_parameter("ribbon_width").get_parameter_value().double_value
        trace_color_param = self.get_parameter("trace_color").get_parameter_value().string_value

        # Locate URDF
        custom_urdf = self.get_parameter("urdf_path").get_parameter_value().string_value
        if not custom_urdf or not os.path.exists(custom_urdf):
            try:
                pkg_dir = get_package_share_directory("dobot_description")
                urdf_candidate = os.path.join(pkg_dir, "urdf", "nova5.urdf")
                if os.path.exists(urdf_candidate):
                    self.urdf_path = urdf_candidate
                else:
                    self.urdf_path = "/Users/monojitlayek/Desktop/Robotics/Dobot_gzwp/src/dobot_description/urdf/nova5.urdf"
            except Exception:
                self.urdf_path = "/Users/monojitlayek/Desktop/Robotics/Dobot_gzwp/src/dobot_description/urdf/nova5.urdf"
        else:
            self.urdf_path = custom_urdf

        self.get_logger().info(f"Using Dobot URDF description: {self.urdf_path}")

        # State Variables (must be initialized before _load_robot)
        self.trajectory_queue: List[JointTrajectoryPoint] = []
        self.traj_start_time = 0.0
        self.has_active_goal = False
        self.start_positions = list(self.HOME_POSITIONS)
        self.current_target_q = list(self.HOME_POSITIONS)

        self.gripper_target = 0.015  # Nominal 15mm open aperture
        self.gripper_current = 0.015

        self.prev_tcp_pos: Optional[List[float]] = None
        self.trace_lines: List[int] = []
        self.clear_trace_pending = False
        self.clear_trace_time = 0.0

        # Path Trace Color Management
        self.cycle_index = -1
        self.active_color_name = "Electric Azure"
        self.active_trace_color = [0.0, 0.48, 0.95]
        self._set_color_mode(trace_color_param)

        # Initialize PyBullet Physics Client
        self._init_pybullet()

        # Load Scene & Manipulator
        self._load_environment()
        self._load_robot()

        # ROS 2 Communication Interfaces
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, "/dobot/end_effector_pose", 10)

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/dobot_arm_controller/joint_trajectory",
            self.trajectory_callback,
            10
        )

        self.gripper_sub = self.create_subscription(
            Float64,
            "/dobot/gripper/command",
            self.gripper_callback,
            10
        )

        self.clear_trace_sub = self.create_subscription(
            Empty,
            "/dobot/clear_trace",
            lambda msg: self.clear_trace(),
            10
        )

        self.trace_color_sub = self.create_subscription(
            String,
            "/dobot/trace_color",
            self.trace_color_callback,
            10
        )

        # Workpiece perception & reset interfaces (Phase 6 Pick-and-Place)
        self.workpiece_pose_pub = self.create_publisher(PoseStamped, "/dobot/workpiece_pose", 10)
        self.reset_workpiece_sub = self.create_subscription(
            Empty,
            "/dobot/reset_workpiece",
            lambda msg: self.reset_workpiece(),
            10
        )

        # Simulation & Telemetry Timer (50 Hz = 20 ms)
        self.sim_timer = self.create_timer(self.dt, self.sim_step)

        self.get_logger().info("================================================================")
        self.get_logger().info("  DOBOT NOVA 5 PYBULLET HD PHYSICS SIMULATOR & TRACER (macOS)")
        self.get_logger().info(f"  - Display:          High Definition ({self.window_width}x{self.window_height})")
        self.get_logger().info("  - Lighting:         Realistic Daylight Industrial Laboratory Environment")
        self.get_logger().info(f"  - Trace Color:      {self.active_color_name} (Mode: {self.trace_color_mode}, Topic: /dobot/trace_color)")
        self.get_logger().info("  - Trajectory Topic: /dobot_arm_controller/joint_trajectory")
        self.get_logger().info("  - Gripper Topic:    /dobot/gripper/command (0.0 to 0.025 m)")
        self.get_logger().info("  - Workpiece Pose:   /dobot/workpiece_pose (50 Hz)")
        self.get_logger().info("  - Reset Workpiece:  /dobot/reset_workpiece")
        self.get_logger().info("  - Clear Trace:      /dobot/clear_trace (Auto-erased on phase completion)")
        self.get_logger().info("  - Joint States:     /joint_states (50 Hz)")
        self.get_logger().info("================================================================")

    def _init_pybullet(self):
        """Configure native Metal/OpenGL PyBullet physics server with realistic daylight laboratory lighting."""
        if self.use_gui:
            try:
                # Realistic bright laboratory daylight background (clean neutral light-grey)
                options = (
                    f"--width={self.window_width} --height={self.window_height} "
                    "--background_color_red=0.90 "
                    "--background_color_green=0.91 "
                    "--background_color_blue=0.93"
                )
                self.client_id = p.connect(p.GUI, options=options)
                if self.client_id < 0:
                    raise RuntimeError("Could not open PyBullet GUI display")

                # Configure High-Definition visualizer features
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
                p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Natural dynamic contact shadows
                p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
                p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION, 0)  # Avoid dark ground mirror
                p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)

                # Cinematic HD camera framing
                p.resetDebugVisualizerCamera(
                    cameraDistance=1.12,
                    cameraYaw=54.0,
                    cameraPitch=-26.0,
                    cameraTargetPosition=[0.32, 0.14, 0.20]
                )
            except Exception as e:
                self.get_logger().warn(f"PyBullet GUI display could not be initialized ({e}), falling back to DIRECT mode.")
                self.use_gui = False
                self.client_id = p.connect(p.DIRECT)
        else:
            self.client_id = p.connect(p.DIRECT)

        p.setGravity(0.0, 0.0, -9.81)
        p.setTimeStep(1.0 / 240.0)

        # Register search paths
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        src_dir = "/Users/monojitlayek/Desktop/Robotics/Dobot_gzwp/src"
        if os.path.exists(src_dir):
            p.setAdditionalSearchPath(src_dir)

    def _load_environment(self):
        """Load realistic robotics laboratory floor, workbench, and precision workpiece."""
        # 1. Clean Light-Grey Laboratory Epoxy Floor with natural contact shadows
        plane_file = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
        self.plane_id = p.loadURDF(plane_file)
        p.changeVisualShape(
            self.plane_id, -1,
            rgbaColor=[0.82, 0.84, 0.86, 1.0],
            specularColor=[0.25, 0.25, 0.25]
        )

        # 2. Heavy-Duty Steel Mounting Pedestal Base for Robot
        pedestal_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.13, height=0.08)
        pedestal_vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.13, length=0.08,
            rgbaColor=[0.34, 0.36, 0.40, 1.0],
            specularColor=[0.45, 0.45, 0.45]
        )
        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=pedestal_col,
            baseVisualShapeIndex=pedestal_vis,
            basePosition=[0.0, 0.0, -0.04]
        )

        # 3. Industrial Laboratory Workbench in Quadrant 1 (Light Grey Industrial Laminate)
        table_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.36, 0.36, 0.05])
        table_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.36, 0.36, 0.05],
            rgbaColor=[0.68, 0.71, 0.75, 1.0],
            specularColor=[0.35, 0.35, 0.35]
        )
        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=table_col,
            baseVisualShapeIndex=table_vis,
            basePosition=[0.36, 0.16, -0.05]
        )

        # 4. Precision Workpiece Fixture Plate (Clean Pure Matte White, No Watermark)
        pad_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.20, 0.16, 0.003])
        pad_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.20, 0.16, 0.003],
            rgbaColor=[0.96, 0.97, 0.98, 1.0],
            specularColor=[0.5, 0.5, 0.5]
        )
        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=pad_col,
            baseVisualShapeIndex=pad_vis,
            basePosition=[0.36, 0.15, 0.001]
        )

        # Clean machined aluminum perimeter bevel (Subtle boundary border)
        b_x0, b_x1 = 0.16, 0.56
        b_y0, b_y1 = -0.01, 0.31
        b_z = 0.004
        border_col = [0.55, 0.58, 0.64]
        p.addUserDebugLine([b_x0, b_y0, b_z], [b_x1, b_y0, b_z], border_col, lineWidth=1.5)
        p.addUserDebugLine([b_x1, b_y0, b_z], [b_x1, b_y1, b_z], border_col, lineWidth=1.5)
        p.addUserDebugLine([b_x1, b_y1, b_z], [b_x0, b_y1, b_z], border_col, lineWidth=1.5)
        p.addUserDebugLine([b_x0, b_y1, b_z], [b_x0, b_y0, b_z], border_col, lineWidth=1.5)

        # 5. Interactive Machined Aluminum Workpiece Billet (Pick Station in Quadrant 1)
        # Position: X = 0.36m, Y = 0.08m, Z = 0.0265m (Resting flat on white fixture plate)
        # Dimensions: Diameter = 25mm (radius 0.0125m), Height = 45mm, Mass = 0.05 kg
        self.workpiece_initial_pos = [0.36, 0.08, 0.0265]
        wp_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.0125, height=0.045)
        wp_vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.0125, length=0.045,
            rgbaColor=[0.88, 0.18, 0.22, 1.0],  # Vibrant Anodized Crimson Billet
            specularColor=[0.7, 0.4, 0.4]
        )
        self.workpiece_id = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=wp_col,
            baseVisualShapeIndex=wp_vis,
            basePosition=self.workpiece_initial_pos
        )
        p.changeDynamics(
            self.workpiece_id, -1,
            lateralFriction=1.4,
            spinningFriction=0.08,
            rollingFriction=0.02,
            restitution=0.02
        )

        # 6. Industrial Parts Collection Drop Bin (Place Station in Quadrant 1)
        # Position: X = 0.38m, Y = 0.26m, Z = 0.004m
        self._create_drop_bin([0.38, 0.26, 0.004])

    def _create_drop_bin(self, base_pos: List[float]):
        """Construct an open-top collection bin with base plate and 4 perimeter walls."""
        bx, by, bz = base_pos
        wall_color = [0.12, 0.62, 0.42, 1.0]  # Industrial Emerald Safety Green
        specular = [0.35, 0.45, 0.35]

        # Bin Base Plate (100mm x 80mm x 6mm)
        base_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.050, 0.040, 0.003])
        base_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.050, 0.040, 0.003],
            rgbaColor=[0.20, 0.22, 0.26, 1.0],
            specularColor=[0.3, 0.3, 0.3]
        )
        p.createMultiBody(0.0, base_col, base_vis, basePosition=[bx, by, bz + 0.003])

        # 4 Sidewalls (Height 32mm, thickness 4mm)
        h_wall = 0.016
        z_wall = bz + 0.006 + h_wall
        # North & South walls (along X)
        wall_ns_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.050, 0.002, h_wall])
        wall_ns_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.050, 0.002, h_wall],
            rgbaColor=wall_color, specularColor=specular
        )
        p.createMultiBody(0.0, wall_ns_col, wall_ns_vis, basePosition=[bx, by + 0.040, z_wall])
        p.createMultiBody(0.0, wall_ns_col, wall_ns_vis, basePosition=[bx, by - 0.040, z_wall])

        # East & West walls (along Y)
        wall_ew_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.002, 0.038, h_wall])
        wall_ew_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.002, 0.038, h_wall],
            rgbaColor=wall_color, specularColor=specular
        )
        p.createMultiBody(0.0, wall_ew_col, wall_ew_vis, basePosition=[bx + 0.050, by, z_wall])
        p.createMultiBody(0.0, wall_ew_col, wall_ew_vis, basePosition=[bx - 0.050, by, z_wall])

    def reset_workpiece(self):
        """Reset workpiece back to its initial pick station coordinates."""
        if hasattr(self, "workpiece_id") and self.workpiece_id is not None:
            p.resetBasePositionAndOrientation(
                self.workpiece_id,
                self.workpiece_initial_pos,
                [0.0, 0.0, 0.0, 1.0]
            )
            p.resetBaseVelocity(self.workpiece_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            self.get_logger().info("PyBullet: Workpiece reset to initial pick station.")

    def _load_robot(self):
        """Load Dobot Nova 5 URDF and map joint/link indices."""
        self.robot_id = p.loadURDF(
            self.urdf_path,
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE
        )

        self.joint_name_to_id = {}
        self.link_name_to_id = {}
        self.arm_joint_ids = []
        self.gripper_joint_ids = []
        self.tcp_link_id = -1

        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            self.joint_name_to_id[joint_name] = i
            self.link_name_to_id[link_name] = i

            if joint_name in self.ARM_JOINT_NAMES:
                self.arm_joint_ids.append(i)
            elif joint_name in self.GRIPPER_JOINT_NAMES:
                self.gripper_joint_ids.append(i)

            if link_name == "tcp_link":
                self.tcp_link_id = i

        # Fallback for tcp_link if merged with fixed parent
        if self.tcp_link_id == -1:
            if "gripper_base_link" in self.link_name_to_id:
                self.tcp_link_id = self.link_name_to_id["gripper_base_link"]
            elif "Link6" in self.link_name_to_id:
                self.tcp_link_id = self.link_name_to_id["Link6"]

        self.get_logger().info(f"PyBullet Arm Joint Indices: {self.arm_joint_ids}")
        self.get_logger().info(f"PyBullet Gripper Joint Indices: {self.gripper_joint_ids}")
        self.get_logger().info(f"PyBullet TCP Link Index: {self.tcp_link_id}")

        # Apply High-Definition materials and specular lighting shaders
        self._apply_hd_materials()

        # Set initial canonical configuration
        for j_id, q_val in zip(self.arm_joint_ids, self.HOME_POSITIONS):
            p.resetJointState(self.robot_id, j_id, targetValue=q_val, targetVelocity=0.0)

        for g_id in self.gripper_joint_ids:
            p.resetJointState(self.robot_id, g_id, targetValue=self.gripper_target, targetVelocity=0.0)
            # Increase gripper finger friction for realistic physical grasping
            p.changeDynamics(
                self.robot_id, g_id,
                lateralFriction=1.6,
                spinningFriction=0.1,
                rollingFriction=0.05,
                frictionAnchor=1
            )

    def _apply_hd_materials(self):
        """Apply high-definition metallic, carbon, and specular shaders to the Dobot Nova 5 model."""
        hd_palette = {
            # Metallic Warm Amber/Gold Main Casting Arms (Warm & Natural)
            "Link2": ([0.94, 0.76, 0.20, 1.0], [0.65, 0.55, 0.30]),
            "Link3": ([0.94, 0.76, 0.20, 1.0], [0.65, 0.55, 0.30]),
            "Link5": ([0.94, 0.76, 0.20, 1.0], [0.65, 0.55, 0.30]),
            # Modern Architectural Satin Graphite Housings (Realistic, Not Pitch Black)
            "base_link": ([0.26, 0.28, 0.31, 1.0], [0.35, 0.35, 0.35]),
            "Link1": ([0.26, 0.28, 0.31, 1.0], [0.35, 0.35, 0.35]),
            "Link4": ([0.26, 0.28, 0.31, 1.0], [0.35, 0.35, 0.35]),
            # Signature High-Visibility Crimson Red Flange Accent
            "Link6": ([0.85, 0.16, 0.18, 1.0], [0.55, 0.25, 0.25]),
            # Precision Anodized Gunmetal Gripper Housing
            "gripper_base_link": ([0.30, 0.32, 0.36, 1.0], [0.55, 0.55, 0.55]),
            # Polished Stainless Steel Gripper Fingers
            "left_finger_link": ([0.62, 0.65, 0.70, 1.0], [0.75, 0.75, 0.75]),
            "right_finger_link": ([0.62, 0.65, 0.70, 1.0], [0.75, 0.75, 0.75]),
        }
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            link_name = info[12].decode("utf-8")
            if link_name in hd_palette:
                rgba, spec = hd_palette[link_name]
                p.changeVisualShape(self.robot_id, i, rgbaColor=rgba, specularColor=spec)

    def clear_trace(self):
        """Erase all active trajectory line items from the PyBullet 3D viewer."""
        if self.trace_lines:
            for line_id in self.trace_lines:
                p.removeUserDebugItem(line_id)
            self.trace_lines.clear()
        self.prev_tcp_pos = None
        self.clear_trace_pending = False

    def _set_color_mode(self, mode_str: str):
        """Set trace color mode: a specific palette name or 'cycle' across phases."""
        clean = mode_str.strip().lower()
        if clean in self.TRACE_PALETTES:
            self.trace_color_mode = clean
            self.active_trace_color = list(self.TRACE_PALETTES[clean])
            self.active_color_name = clean.capitalize()
            self.get_logger().info(
                f"PyBullet: Path trace color set to static '{self.active_color_name}' ({self.active_trace_color})."
            )
        else:
            self.trace_color_mode = "cycle"
            self.get_logger().info(
                "PyBullet: Path trace color set to dynamic phase cycling (each phase will use a distinct vibrant color)."
            )

    def trace_color_callback(self, msg: String):
        """Dynamic ROS 2 topic to switch trace color or enable phase cycle mode at runtime."""
        self._set_color_mode(msg.data)

    def trajectory_callback(self, msg: JointTrajectory):
        """Receive multi-point trajectory for execution."""
        if not msg.points:
            self.get_logger().warn("Received empty JointTrajectory message!")
            return

        # Erase previous trajectory/phase path trace on arrival of new trajectory
        self.clear_trace()

        # Advance phase trace color if in cycle mode
        if self.trace_color_mode == "cycle":
            self.cycle_index = (self.cycle_index + 1) % len(self.PHASE_CYCLE_COLORS)
            color_name, rgb = self.PHASE_CYCLE_COLORS[self.cycle_index]
            self.active_color_name = color_name
            self.active_trace_color = list(rgb)
            self.get_logger().info(
                f"PyBullet: Starting trajectory phase {self.cycle_index + 1} trace in {color_name} {rgb}."
            )

        self.trajectory_queue = msg.points
        self.traj_start_time = time.time()
        self.has_active_goal = True

        # Read current positions as trajectory start configuration
        curr_states = p.getJointStates(self.robot_id, self.arm_joint_ids)
        self.start_positions = [s[0] for s in curr_states]

        total_duration = (
            msg.points[-1].time_from_start.sec +
            msg.points[-1].time_from_start.nanosec * 1e-9
        )
        self.get_logger().info(
            f"PyBullet: Received trajectory with {len(msg.points)} waypoints (Duration: {total_duration:.3f} s)."
        )

    def gripper_callback(self, msg: Float64):
        """Receive gripper aperture command in meters."""
        self.gripper_target = max(0.0, min(0.025, float(msg.data)))
        desc = "CLOSED" if self.gripper_target < 0.005 else ("OPEN" if self.gripper_target > 0.020 else "PARTIAL")
        self.get_logger().info(f"PyBullet: Gripper target aperture = {self.gripper_target * 1000.0:.1f} mm ({desc})")

    def sim_step(self):
        """Periodic simulation tick: trajectory playback, physics stepping, and state telemetry."""
        now_time = time.time()

        # ---------------------------------------------------------------------
        # 1. Time-Parameterized Trajectory Interpolation & Motor Control
        # ---------------------------------------------------------------------
        target_v = [0.0] * len(self.arm_joint_ids)
        if self.has_active_goal and self.trajectory_queue:
            t_elapsed = now_time - self.traj_start_time
            t_final = (
                self.trajectory_queue[-1].time_from_start.sec +
                self.trajectory_queue[-1].time_from_start.nanosec * 1e-9
            )

            if t_elapsed >= t_final:
                # Goal reached: set motors to final configuration
                final_pt = self.trajectory_queue[-1]
                self.current_target_q = list(final_pt.positions[:len(self.arm_joint_ids)])
                self.has_active_goal = False
                self.get_logger().info("PyBullet: Trajectory motion completed at goal configuration.")

                # Schedule trace erasure upon phase completion
                if self.trace_lines:
                    self.clear_trace_pending = True
                    self.clear_trace_time = now_time + self.trace_dwell_time
            else:
                # Search matching interval [k, k+1]
                k = 0
                while k + 1 < len(self.trajectory_queue):
                    t_k1 = (
                        self.trajectory_queue[k + 1].time_from_start.sec +
                        self.trajectory_queue[k + 1].time_from_start.nanosec * 1e-9
                    )
                    if t_k1 > t_elapsed:
                        break
                    k += 1

                pt_k = self.trajectory_queue[k]
                t_k = pt_k.time_from_start.sec + pt_k.time_from_start.nanosec * 1e-9

                if k + 1 < len(self.trajectory_queue):
                    pt_k1 = self.trajectory_queue[k + 1]
                    t_k1 = pt_k1.time_from_start.sec + pt_k1.time_from_start.nanosec * 1e-9
                    dt_seg = max(1e-5, t_k1 - t_k)
                    alpha = (t_elapsed - t_k) / dt_seg

                    target_q = []
                    target_v = []
                    for i in range(len(self.arm_joint_ids)):
                        p0 = pt_k.positions[i] if i < len(pt_k.positions) else 0.0
                        p1 = pt_k1.positions[i] if i < len(pt_k1.positions) else p0
                        v0 = pt_k.velocities[i] if i < len(pt_k.velocities) else 0.0
                        v1 = pt_k1.velocities[i] if i < len(pt_k1.velocities) else 0.0

                        pos_interp = p0 + alpha * (p1 - p0)
                        vel_interp = v0 + alpha * (v1 - v0)
                        target_q.append(pos_interp)
                        target_v.append(vel_interp)
                    self.current_target_q = target_q
                else:
                    self.current_target_q = list(pt_k.positions[:len(self.arm_joint_ids)])
                    target_v = [0.0] * len(self.arm_joint_ids)

        # Check if phase path trace erasure is due
        if self.clear_trace_pending and now_time >= self.clear_trace_time:
            self.clear_trace()
            self.get_logger().info("PyBullet: Erased end-effector path trace upon phase completion.")

        # Continually hold current arm joint targets with high stiffness
        p.setJointMotorControlArray(
            bodyIndex=self.robot_id,
            jointIndices=self.arm_joint_ids,
            controlMode=p.POSITION_CONTROL,
            targetPositions=self.current_target_q,
            targetVelocities=target_v,
            forces=[250.0] * len(self.arm_joint_ids),
            positionGains=[0.2] * len(self.arm_joint_ids)
        )

        # ---------------------------------------------------------------------
        # 2. Dynamic Prismatic Gripper Jaw Motion
        # ---------------------------------------------------------------------
        max_grip_step = 0.05 * self.dt  # 50 mm/s linear velocity limit
        grip_diff = self.gripper_target - self.gripper_current
        if abs(grip_diff) <= max_grip_step:
            self.gripper_current = self.gripper_target
        else:
            self.gripper_current += max_grip_step if grip_diff > 0.0 else -max_grip_step

        if self.gripper_joint_ids:
            p.setJointMotorControlArray(
                bodyIndex=self.robot_id,
                jointIndices=self.gripper_joint_ids,
                controlMode=p.POSITION_CONTROL,
                targetPositions=[self.gripper_current] * len(self.gripper_joint_ids),
                forces=[40.0] * len(self.gripper_joint_ids)
            )

        # ---------------------------------------------------------------------
        # 3. Step Bullet Physics (Sub-stepping for ultra-high numerical stability)
        # ---------------------------------------------------------------------
        for _ in range(4):
            p.stepSimulation()

        # ---------------------------------------------------------------------
        # 4. Read Live Joint States & Publish to ROS 2
        # ---------------------------------------------------------------------
        arm_states = p.getJointStates(self.robot_id, self.arm_joint_ids)
        arm_positions = [s[0] for s in arm_states]
        arm_velocities = [s[1] for s in arm_states]

        full_positions = arm_positions + [self.gripper_current, self.gripper_current]
        full_velocities = arm_velocities + [0.0, 0.0]

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.ALL_JOINT_NAMES
        joint_msg.position = full_positions
        joint_msg.velocity = full_velocities
        self.joint_state_pub.publish(joint_msg)

        # ---------------------------------------------------------------------
        # 5. Tool Center Point (TCP) Trajectory Tracer in PyBullet
        # ---------------------------------------------------------------------
        link_state = p.getLinkState(self.robot_id, self.tcp_link_id)
        tcp_pos = list(link_state[0])
        tcp_orn = list(link_state[1])

        # Publish End-Effector PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = joint_msg.header.stamp
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = tcp_pos[0]
        pose_msg.pose.position.y = tcp_pos[1]
        pose_msg.pose.position.z = tcp_pos[2]
        pose_msg.pose.orientation.x = tcp_orn[0]
        pose_msg.pose.orientation.y = tcp_orn[1]
        pose_msg.pose.orientation.z = tcp_orn[2]
        pose_msg.pose.orientation.w = tcp_orn[3]
        self.ee_pose_pub.publish(pose_msg)

        # ---------------------------------------------------------------------
        # 6. Publish Live Workpiece Pose (Phase 6 Perception Feedback)
        # ---------------------------------------------------------------------
        if hasattr(self, "workpiece_id") and self.workpiece_id is not None:
            wp_pos, wp_orn = p.getBasePositionAndOrientation(self.workpiece_id)
            wp_msg = PoseStamped()
            wp_msg.header.stamp = joint_msg.header.stamp
            wp_msg.header.frame_id = "base_link"
            wp_msg.pose.position.x = wp_pos[0]
            wp_msg.pose.position.y = wp_pos[1]
            wp_msg.pose.position.z = wp_pos[2]
            wp_msg.pose.orientation.x = wp_orn[0]
            wp_msg.pose.orientation.y = wp_orn[1]
            wp_msg.pose.orientation.z = wp_orn[2]
            wp_msg.pose.orientation.w = wp_orn[3]
            self.workpiece_pose_pub.publish(wp_msg)

        # Draw high-contrast dynamic trajectory ribbon in 3D PyBullet viewer
        if self.use_gui:
            if self.prev_tcp_pos is None:
                self.prev_tcp_pos = tcp_pos
            else:
                dist = math.dist(self.prev_tcp_pos, tcp_pos)
                if dist >= 0.0015:  # 1.5 mm distance increment
                    line_id = p.addUserDebugLine(
                        self.prev_tcp_pos,
                        tcp_pos,
                        lineColorRGB=self.active_trace_color,  # Dynamic multi-color / phase palette
                        lineWidth=self.ribbon_width,
                        lifeTime=0.0  # Controlled dynamically by phase completion
                    )
                    self.trace_lines.append(line_id)
                    # Limit memory to 5000 trace segments
                    if len(self.trace_lines) > 5000:
                        old_line = self.trace_lines.pop(0)
                        p.removeUserDebugItem(old_line)
                    self.prev_tcp_pos = tcp_pos

    def destroy_node(self):
        """Clean shutdown and disconnect from PyBullet."""
        p.disconnect(self.client_id)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DobotPybulletNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
