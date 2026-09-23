from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    color_segmentation_node = Node(
        package='detection_lane_opencv',
        executable='color_segmentation_node',
        name='color_segmentation_node',
        output='screen',
    )

    lane_geometry_node = Node(
        package='detection_lane_opencv',
        executable='lane_geometry_node',
        name='lane_geometry_node',
        output='screen',
    )

    return LaunchDescription([color_segmentation_node, lane_geometry_node])
