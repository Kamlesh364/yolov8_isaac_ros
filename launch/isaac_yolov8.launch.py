import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'engine',
            default_value='',
            description='Yolov8 enigne file path'),
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='Device to run the engine on'),
    ]

    engine = launch.substitutions.LaunchConfiguration('engine')
    device = launch.substitutions.LaunchConfiguration('device')

    image_publisher_node = Node(
        name='image_publisher',
        package='yolov8_isaac_ros',
        executable='image_publisher',
        output='screen',
    )

    yolov8_visualizer_node = Node(
        name='yolov8_visualizer',
        package='yolov8_isaac_ros',
        executable='yolov8_visualizer',
        output='screen',
        parameters=[{
            'engine': engine,
            'device': device,
            'show': True,
        }]
    )

    rqt_node = Node(
        name='image_view',
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/yolov8_processed_image']
    )

    realsense_camera_node = Node(
        name='realsense2_camera_node',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        parameters=[{
            'rgb_camera.color_format': 'bgr8',
        }]
    )

    final_launch_description = launch_args + [yolov8_visualizer_node] + [rqt_node] 
    return launch.LaunchDescription(final_launch_description)
