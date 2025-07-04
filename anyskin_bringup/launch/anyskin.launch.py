from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="/dev/ttyACM0",
        description="Serial device to connect to the AnySkin",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace to prepend to the node",
    )

    anyskin_node = Node(
        package="anyskin_bringup",
        executable="tactile_sensor_broadcaster",
        name="tactile_sensor_broadcaster",
        namespace=LaunchConfiguration("namespace"),
        parameters=[{"device": LaunchConfiguration("device")}],
    )

    return LaunchDescription(
        [
            device_arg,
            namespace_arg,
            anyskin_node,
        ]
    )
