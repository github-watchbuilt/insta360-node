from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    compress = LaunchConfiguration("compress")
    undistort = LaunchConfiguration("undistort")
    debug = LaunchConfiguration("debug")
    get_images = LaunchConfiguration("get_images")
    config = LaunchConfiguration("config")

    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument("compress", default_value="true"))
    ld.add_action(DeclareLaunchArgument("undistort", default_value="false"))
    ld.add_action(DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(DeclareLaunchArgument("get_images", default_value="false"))
    ld.add_action(DeclareLaunchArgument("config", default_value="config.yaml"))

    # Main camera node
    ld.add_action(
        Node(
            package="insta360_ros_driver",
            executable="insta360_ros_driver_node",
            name="bringup",
            output="screen",
        )
    )

    # Group for compressed image processing
    ld.add_action(
        GroupAction(
            [
                Node(
                    package="insta360_ros_driver",
                    executable="live_processing.py",
                    name="live_processing",
                    output="screen",
                    condition=UnlessCondition(debug),
                ),
                Node(
                    package="insta360_ros_driver",
                    executable="live_processing.py",
                    name="live_processing",
                    output="screen",
                    prefix=Command(["xterm -e"]),
                    condition=IfCondition(debug),
                ),
                Node(
                    package="rqt_image_view",
                    executable="rqt_image_view",
                    name="rqt_image_view",
                    arguments=["/back_camera_image/compressed"],
                    condition=IfCondition(debug),
                ),
                Node(
                    package="insta360_ros_driver",
                    executable="rostopic_hz.sh",
                    name="rostopic_hz",
                    output="screen",
                    prefix=Command(["xterm -e"]),
                    condition=IfCondition(debug),
                ),
                Node(
                    package="insta360_ros_driver",
                    executable="get_images.py",
                    name="get_images",
                    output="screen",
                    parameters=[{"topic": "/back_camera_image/compressed"}],
                    condition=IfCondition(get_images),
                ),
            ],
            condition=IfCondition(compress),
        )
    )

    # Group for raw YUV image processing
    ld.add_action(
        GroupAction(
            [
                Node(
                    package="rqt_image_view",
                    executable="rqt_image_view",
                    name="rqt_image_view",
                    arguments=["/insta_image_yuv"],
                    condition=IfCondition(debug),
                ),
                Node(
                    package="insta360_ros_driver",
                    executable="rostopic_hz_raw.sh",
                    name="rostopic_hz_raw",
                    output="screen",
                    prefix=Command(["xterm -e"]),
                    condition=IfCondition(debug),
                ),
            ],
            condition=UnlessCondition(compress),
        )
    )

    return ld
