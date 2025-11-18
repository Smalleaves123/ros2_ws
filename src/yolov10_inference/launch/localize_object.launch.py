from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/color/image_raw'
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/aligned_depth_to_color/image_raw'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/color/camera_info'
    )
    target_class_arg = DeclareLaunchArgument(
        'target_class', default_value=''  # leave empty to use highest confidence
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value=''  # e.g., base_link
    )

    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    target_class = LaunchConfiguration('target_class')
    target_frame = LaunchConfiguration('target_frame')

    yolo_node = Node(
        package='yolov10_inference',
        executable='yolo_rgs',
        name='yolov10_yolo',
        output='screen',
        parameters=[{
            'image_topic': image_topic
        }]
    )

    localizer_node = Node(
        package='yolov10_inference',
        executable='bbox_to_3d',
        name='bbox_to_3d',
        output='screen',
        parameters=[{
            'detection_topic': 'detections',
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'target_class': target_class,
            'target_frame': target_frame,
            'publish_marker': True
        }]
    )

    return LaunchDescription([
        image_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        target_class_arg,
        target_frame_arg,
        yolo_node,
        localizer_node
    ])



