import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    pkg_webots = get_package_share_directory("tbot3_webots")
    pkg_costmap = get_package_share_directory("tbot3_costmap")

    simulation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_webots, "launch", "tbot3_webots_proto.launch.py")
        )
    )

    costmap = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_costmap, 'launch', "costmap_2d.launch.py")
        )
    )

    return launch.launch_description.LaunchDescription([
        simulation,
        launch.actions.TimerAction(
            period=6.0,
            actions=[costmap]
        )
    ])