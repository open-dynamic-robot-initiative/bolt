from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="bolt",
                node_executable="dg",
                output="screen",
                prefix=['xterm -fa "Monospace" -fs 14 -hold -e'],
                # prefix=['xterm -e gdb -ex=r --args'],
            ),
            Node(
                package="bolt",
                node_executable="hw",
                output="screen",
                prefix=['xterm -fa "Monospace" -fs 14 -hold -e'],
                # prefix=['xterm -e gdb -ex=r --args'],
            ),
        ]
    )