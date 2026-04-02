import os
import launch
import launch_ros
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.urdf_spawner import URDFSpawner
from webots_ros2_driver.utils import controller_url_prefix
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory("tbot3_webots")
    pkg_rviz = get_package_share_directory("tbot3_rviz")
    
    urdf_proto = os.path.join(pkg_dir,'description/urdf',"tbot3_waffle_proto.urdf")
    urdf_state = os.path.join(pkg_dir,'description/urdf',"turtlebot3_waffle_clean.urdf")
    with open(urdf_state,'r') as urdf:
        lines = urdf.readlines()
        robot_description = ''.join(line for line in lines if not line.strip().startswith('<?xml'))

    webots = WebotsLauncher(
        world=os.path.join(pkg_dir,'worlds','obstacle_world.wbt'),
        gui = True,
        ros2_supervisor=True,
        output='screen'
    )

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description' : robot_description,
            'use_sim_time': True
        }]
    )

    joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    driver = WebotsController(
        robot_name="Turtlebot3Waffle",
        parameters=[{
            'robot_description': urdf_proto,
            'use_sim_time': True,
        }]
    )


    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rviz, 'launch', 'tbot3_rviz.launch.py')
        )
    )

    return launch.launch_description.LaunchDescription([
        webots,
        webots._supervisor,
        robot_state_publisher,
        joint_state_publisher,
        driver,
        rviz_launch,
        launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        )
    ])