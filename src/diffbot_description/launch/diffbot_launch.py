import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg            = get_package_share_directory('diffbot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg, 'urdf', 'diffbot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # ── 1. Robot State Publisher ──────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ── 2. Gazebo ─────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # ── 3. Spawn robot ────────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'diffbot',
            '-allow_renaming', 'true',
            '-z', '0.1',
        ]
    )

    # ── 4. Spawn chair ────────────────────────────────────────────────
    # Chair is at x=1.2m directly in front of the robot camera
    # Camera is mounted at x=0.15 on the robot, so 1.2m gives ~1m gap
    spawn_chair = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', '/home/salah/Chair/model.sdf',
            '-name', 'Chair',
            '-x', '1.2',   # 1.2m in front of robot
            '-y', '0.0',   # centred
            '-z', '0.0',   # on the ground
        ]
    )

    # ── 5. ROS <-> Gazebo bridge ──────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/diffbot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[('/model/diffbot/tf', '/tf')]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_robot,
        spawn_chair,
        bridge,
    ])