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

def generate_launch_description():
    pkg_dir = get_package_share_directory(PKG_NAME)

    urdf_path = os.path.join(pkg_dir,'description/urdf',"turtlebot3_waffle.urdf")
    with open(urdf_path,'r') as urdf:
        lines = urdf.readlines()
        robot_description = ''.join(line for line in lines if not line.strip().startswith('<?xml'))

    webots = WebotsLauncher(
        world=os.path.join(pkg_dir,'worlds','empty_world.wbt'),
        gui = True,
        ros2_supervisor=True,
        output='screen'
    )

    return launch.launch_description.LaunchDescription([
        webots,
        webots._supervisor,
        launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )
        )
    ])