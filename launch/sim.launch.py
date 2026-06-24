import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('vision_tracker')

def generate_launch_description():

    # Start Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args':os.path.join(pkg_share,'models','world.sdf')
        }.items()
    )


    # Spawn vehicle
    vehicle = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'follower_vehicle',
            '-file',
            os.path.join(pkg_share,'models','model.sdf')
        ],
        output='screen'
    )
    sphere = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'sphere',
            '-file',
            os.path.join(pkg_share,'models','sphere.sdf'),
            '-x',
            '3',
            '-y',
            '0',
            '-z',
            '1'
        ],
        output='screen'
    )
    detector = Node(
        package='vision_tracker',
        executable='detector',
        output='screen'
    )
    camera_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/world/empty/model/follower_vehicle/link/chassis/sensor/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/empty/model/follower_vehicle/link/chassis/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

                    # RGB image  <-- ADD THIS
                    '/world/empty/model/follower_vehicle/link/chassis/sensor/rgb_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',

                    # RGB info
                    '/world/empty/model/follower_vehicle/link/chassis/sensor/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
                ],
                output='screen'
            )
        ]
    )


    return LaunchDescription([
        gazebo,
        vehicle,
        sphere,
        camera_bridge,
        detector
    ])
