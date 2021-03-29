"""

License: BSD 3-Clause License
Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

from robot_properties_bolt.config import BoltConfig
from bolt.dg_bolt_base_bullet import DgBoltBaseRobot


class BoltBulletRobot(DgBoltBaseRobot):
    def __init__(
        self,
        useFixedBase=False,
        record_video=False,
        init_sliders_pose=4 * [0.5],
    ):
        super(BoltBulletRobot, self).__init__(
            BoltConfig(), useFixedBase, record_video, init_sliders_pose
        )

        self.q0[2] = 0.26487417
        # self.q0[5] = -0.1736482
        self.q0[6] = 1.0
        self.q0[7] = 0.0
        self.q0[8] = 0.78539816
        self.q0[9] = -1.57079633
        self.q0[10] = 0.0
        self.q0[11] = 0.78539816
        self.q0[12] = -1.57079633

        # Sync the current robot state to the graph input signals.
        self._sim2signal()


def get_bolt_robot(
    useFixedBase=False, record_video=False, init_sliders_pose=4 * [0.5]
):
    return BoltBulletRobot(useFixedBase, record_video, init_sliders_pose)
