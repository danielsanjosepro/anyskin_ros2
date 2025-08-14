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

    image_size_arg = DeclareLaunchArgument(
        "image_size",
        default_value="256",
        description="Size of the tactile sensor image in pixels",
    )
    should_filter_arg = DeclareLaunchArgument(
        "should_filter",
        default_value="true",
        description="Whether to filter the tactile sensor data with a high-pass filter",
    )
    all_values_arg = DeclareLaunchArgument(
        "all_values",
        default_value="false",
        description="Whether to publish all values from the tactile sensor",
    )
    args = [
        device_arg,
        namespace_arg,
        image_size_arg,
        should_filter_arg,
        all_values_arg,
    ]

    anyskin_node = Node(
        package="anyskin_bringup",
        executable="tactile_sensor_broadcaster",
        name="tactile_sensor_broadcaster",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            {"port": LaunchConfiguration("device")},
            {"image_size": LaunchConfiguration("image_size")},
            {"should_filter": LaunchConfiguration("should_filter")},
            {"all_values": LaunchConfiguration("all_values")},
        ],
    )

    return LaunchDescription(
        [
            *args,
            anyskin_node,
        ]
    )
