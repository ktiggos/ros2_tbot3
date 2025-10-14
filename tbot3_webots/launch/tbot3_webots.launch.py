import os
import launch
import launch_ros
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.urdf_spawner import URDFSpawner
from webots_ros2_driver.utils import controller_url_prefix
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessIO

PKG_NAME = "tbot3_webots"
PROTO = False

def generate_launch_description():
    pkg_dir = get_package_share_directory(PKG_NAME)

    if PROTO:
        urdf_name = "tbot3_waffle_proto.urdf"
    else:
        urdf_name = "turtlebot3_waffle.urdf"
    
    urdf_path = os.path.join(pkg_dir,'description/urdf',urdf_name)
    with open(urdf_path,'r') as urdf:
        lines = urdf.readlines()
        robot_description = ''.join(line for line in lines if not line.strip().startswith('<?xml'))

    webots = WebotsLauncher(
        world=os.path.join(pkg_dir,'worlds','empty_world.wbt'),
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

    if not PROTO:
        spawner = URDFSpawner(
            name="turtlebot3_waffle",
            urdf_path=urdf_path,
        )

    driver = WebotsController(
        robot_name="turtlebot3_waffle",
        parameters=[
            {'robot_description' : urdf_path}
        ]
    )

    return launch.launch_description.LaunchDescription([
        webots,
        webots._supervisor,
        robot_state_publisher,
        spawner,
        # driver,
        launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        )
    ])