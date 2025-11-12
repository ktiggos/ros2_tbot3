import launch
import launch_ros

def generate_launch_description():

    xbox_driver = launch_ros.actions.Node(
        package='tbot3_teleop',
        executable='xbox_driver',
        output='screen',
    )

    joy_teleop = launch_ros.actions.Node(
        package='tbot3_teleop',
        executable='joy_teleop',
        output='screen',
    )

    return launch.launch_description.LaunchDescription([
        xbox_driver,
        joy_teleop,
    ])