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
from robot_properties_bolt.bolt_wrapper import BoltRobot
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
        use_fixed_base=False,
        init_sliders_pose=4 * [0.5],
    ):
        # Copy the arguments internally
        self._bolt_config = bolt_config
        self._use_fixed_base = use_fixed_base
        self._init_sliders_pose = init_sliders_pose

        # Create an instance of the simulator.
        self._bullet_env = BulletEnvWithGround()

        # Load the robot
        robotStartPos = [0.0, 0, 0.26487417]
        robotStartOrientation = pybullet.getQuaternionFromEuler(
            [0.0, 0.0, 0.0]
        )

        # Load the robot
        self._simulated_robot = BoltRobot(
            robotStartPos,
            robotStartOrientation,
            self._init_sliders_pose,
            self._use_fixed_base,
        )

        self._bullet_env.add_robot(self._simulated_robot)

        self.q0 = np.zeros(self._simulated_robot.pin_robot.nq)

        # Initialize the device.
        self.device = Device(self._bolt_config.robot_name)
        assert Path(self._bolt_config.dgm_yaml_path).exists()
        self.device.initialize(self._bolt_config.dgm_yaml_path)

        # Create signals for the base.
        self._signal_base_pos = VectorConstant("bullet_bolt_base_pos")
        self._signal_base_vel = VectorConstant("bullet_bolt_base_vel")
        self._signal_base_vel_world = VectorConstant(
            "bullet_bolt_base_vel_world"
        )
        # Sync the current robot state to the graph input signals.
        self._sim2signal()

        super(DgBoltBaseRobot, self).__init__(
            self._bolt_config.robot_name, self.device
        )

    def base_signals(self):
        return self._signal_base_pos.sout, self._signal_base_vel.sout

    def _sim2signal(self):
        """Reads the state from the simulator and fills
        the corresponding signals."""

        q, dq = self._simulated_robot.get_state()

        # Fill in the device.
        self.device.joint_positions.value = q[7:]
        self.device.joint_velocities.value = dq[6:]
        self.device.slider_positions.value = np.array(
            [
                self._simulated_robot.get_slider_position("a"),
                self._simulated_robot.get_slider_position("b"),
                self._simulated_robot.get_slider_position("c"),
                self._simulated_robot.get_slider_position("d"),
            ]
        )
        if self.device.hasSignal("contact_sensors"):
            self.device.contact_sensors.value = np.array(4 * [0.0])

        # Base related signals
        self._signal_base_pos.sout.value = q[0:7]
        self._signal_base_vel.sout.value = dq[0:6]
        self._signal_base_vel_world.sout.value = (
            self._simulated_robot.get_base_velocity_world()
        )

    def run(self, steps=1, sleep=False):
        """ Get input from Device, Step the simulation, feed the Device. """

        for _ in range(steps):
            self.device.execute_graph()
            self._simulated_robot.send_joint_command(
                self.device.ctrl_joint_torques.value
            )
            self._bullet_env.step(sleep=sleep)
            self._sim2signal()

    def reset_state(self, q, dq):
        """Reset the simulator and robot state. """
        
        self._simulated_robot.reset_state(q, dq)
        self._sim2signal()

    def add_ros_and_trace(
        self, client_name, signal_name, topic_name=None, topic_type=None
    ):
        # for vicon entity
        self.signal_name = signal_name

    def start_video_recording(self, file_name):
        self._bullet_env.start_video_recording(file_name)

    def stop_video_recording(self):
        self._bullet_env.stop_video_recording()
