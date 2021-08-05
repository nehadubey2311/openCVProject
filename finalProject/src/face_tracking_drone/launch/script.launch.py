from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='face_tracking_drone', executable='drone_node', output='screen'),
        launch_ros.actions.Node(
            package='face_tracking_drone', executable='opencv_node', output='screen'),
    ])