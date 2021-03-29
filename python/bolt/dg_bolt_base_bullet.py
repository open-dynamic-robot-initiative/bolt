"""

License: BSD 3-Clause License
Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

from pathlib import Path
import numpy as np
import pybullet
from bullet_utils.env import BulletEnvWithGround
from robot_properties_bolt.bolt_wrapper import BoltRobot, BoltConfig
from dynamic_graph_manager.dynamic_graph.device import Device
from dynamic_graph_manager.robot import Robot
from dynamic_graph.sot.core.vector_constant import VectorConstant


class DgBoltBaseRobot(Robot):
    """
    Base implementation for bolt robot.
    """

    def __init__(
        self,
        bolt_config,
        useFixedBase=False,
        record_video=False,
        init_sliders_pose=4 * [0.5],
    ):
        # Copy the arguments internally
        self.config = bolt_config
        self.useFixedBase = useFixedBase
        self.record_video = record_video
        self.init_sliders_pose = init_sliders_pose

        # Create an instance of the simulator initializes the simulator as well.
        self.bullet_env = BulletEnvWithGround(pybullet.GUI)

        # Load the robot
        robotStartPos = [0.0, 0, 0.26487417]
        robotStartOrientation = pybullet.getQuaternionFromEuler([0.0, 0.0, 0.0])

        # Load the robot
        self.robot_bullet = self.bullet_env.add_robot(
            BoltRobot,
            pos=robotStartPos,
            orn=robotStartOrientation,
            useFixedBase=self.useFixedBase,
        )

        self.q0 = np.zeros(self.robot_bullet.pin_robot.nq)

        # Initialize the device.
        self.device = Device(self.config.robot_name)
        assert Path(self.config.dgm_yaml_path).exists()
        self.device.initialize(self.config.dgm_yaml_path)

        # Create signals for the base.
        self.signal_base_pos_ = VectorConstant("bullet_bolt_base_pos")
        self.signal_base_vel_ = VectorConstant("bullet_bolt_base_vel")
        self.signal_base_vel_world_ = VectorConstant(
            "bullet_bolt_base_vel_world")
        self.signal_base_pos_.sout.value = np.hstack(
            [robotStartPos, robotStartOrientation]
        )
        self.signal_base_vel_.sout.value = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.signal_base_vel_world_.sout.value = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )

        # Initialize signals that are not filled in sim2signals.
        if self.device.hasSignal("contact_sensors"):
            self.device.contact_sensors.value = np.array(4 * [0.0])

        # Sync the current robot state to the graph input signals.
        self._sim2signal()

        super(DgBoltBaseRobot, self).__init__(
            self.config.robot_name, self.device)

    def base_signals(self):
        return self.signal_base_pos_.sout, self.signal_base_vel_.sout

    def _sim2signal(self):
        """Reads the state from the simulator and fills
        the corresponding signals."""

        q, dq = self.robot_bullet.get_state()

        device = self.device
        device.joint_positions.value = q[7:]
        device.joint_velocities.value = dq[6:]

        self.signal_base_pos_.sout.value = q[0:7]
        self.signal_base_vel_.sout.value = dq[0:6]
        self.signal_base_vel_world_.sout.value = self.robot_bullet.get_base_velocity_world()

        device.slider_positions.value = np.array(
            [
                self.robot_bullet.get_slider_position("a"),
                self.robot_bullet.get_slider_position("b"),
                self.robot_bullet.get_slider_position("c"),
                self.robot_bullet.get_slider_position("d")
            ]
        )

    def run(self, steps=1, sleep=False):
        """ Get input from Device, Step the simulation, feed the Device. """

        for _ in range(steps):
            self.device.execute_graph()
            print("self.pin_robot.nv = ", self.robot_bullet.pin_robot.nv)
            print("self.pin_robot.nv - 6 = ", self.robot_bullet.pin_robot.nv - 6)
            print("self.device.ctrl_joint_torques.value = ", self.device.ctrl_joint_torques.value)
            self.robot_bullet.send_joint_command(
                self.device.ctrl_joint_torques.value
            )
            self.bullet_env.step(sleep=sleep)
            self._sim2signal()

    def reset_state(self, q, dq):
        """Sets the bullet simulator and the signals to
        the provided state values."""
        self.robot_bullet.reset_state(q, dq)
        self._sim2signal()

    def add_ros_and_trace(
        self, client_name, signal_name, topic_name=None, topic_type=None
    ):
        # for vicon entity
        self.signal_name = signal_name

    def start_video_recording(self, file_name):
        if self.record_video:
            self.bullet_env.start_video_recording(file_name)

    def stop_video_recording(self):
        if self.record_video:
            self.bullet_env.stop_video_recording()
