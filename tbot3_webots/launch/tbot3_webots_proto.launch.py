import os
import launch
import launch_ros
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.urdf_spawner import URDFSpawner
from webots_ros2_driver.utils import controller_url_prefix
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessIO

def generate_launch_description():
    pkg_dir = get_package_share_directory("tbot3_webots")
    
    urdf_path = os.path.join(pkg_dir,'description/urdf',"tbot3_waffle_proto.urdf")
    with open(urdf_path,'r') as urdf:
        lines = urdf.readlines()
        robot_description = ''.join(line for line in lines if not line.strip().startswith('<?xml'))

    webots = WebotsLauncher(
        world=os.path.join(pkg_dir,'worlds','empty_world_proto.wbt'),
        gui = True,
        ros2_supervisor=True,
        output='screen'
    )

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description' : robot_description
        }]
    )

    driver = WebotsController(
        robot_name="Turtlebot3Waffle",
        parameters=[
            {'robot_description' : urdf_path}
        ]
    )

    return launch.launch_description.LaunchDescription([
        webots,
        webots._supervisor,
        robot_state_publisher,
        driver,
        launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        )
    ])