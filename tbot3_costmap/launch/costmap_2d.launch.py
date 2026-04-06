import os
import launch
import launch_ros
import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.event_handlers import OnStateTransition

def generate_launch_description():

    costmap_params = os.path.join(
        get_package_share_directory("tbot3_costmap"),
        "config",
        "costmap.yaml"
    )

    costmap_2d = launch_ros.actions.LifecycleNode(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        namespace="",
        output="screen",
        parameters=[costmap_params]
    )

    lifecycle_manager = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[{
            'use_sim_time': True,
            'node_names': ["/costmap/costmap"],
            'autostart': True
        }]
    )

    return launch.launch_description.LaunchDescription([
        costmap_2d,
        lifecycle_manager
    ])