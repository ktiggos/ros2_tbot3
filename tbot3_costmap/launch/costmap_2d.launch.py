import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    costmap_params = os.path.join(
        get_package_share_directory("tbot3_costmap"),
        "config",
        "costmap.yaml"
    )

    costmap_2d = launch_ros.actions.Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        output="screen",
        parameters=[costmap_params]
    )

    return launch.launch_description.LaunchDescription([
        costmap_2d
    ])