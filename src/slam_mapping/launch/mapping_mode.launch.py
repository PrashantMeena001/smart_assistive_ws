# =============================================================================
# mapping_mode.launch.py — SLAM Mapping Mode (Caregiver)
# =============================================================================
# Launches RTAB-Map in mapping mode to build a 3D occupancy map
# of the environment. Intended for use by a caregiver who walks
# through the space to create a map for later localization.
#
# Usage: ros2 launch smart_assistive_system mapping_mode.launch.py
#
# The map is saved to: src/slam_mapping/maps/house_map.db
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # =========================================================================
    # Launch arguments
    # =========================================================================
    map_db_path_arg = DeclareLaunchArgument(
        'map_db_path',
        default_value=os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'maps', 'house_map.db'
        ),
        description='Path to save the RTAB-Map database'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Use simulated time'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/vision/raw_image',
        description='Camera image topic'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu/data',
        description='IMU data topic'
    )

    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate', default_value='5.0',
        description='RTAB-Map update rate (Hz) — keep low for Pi'
    )

    # =========================================================================
    # Substitutions
    # =========================================================================
    map_db_path = LaunchConfiguration('map_db_path')
    use_sim = LaunchConfiguration('use_sim')
    camera_topic = LaunchConfiguration('camera_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    frame_rate = LaunchConfiguration('frame_rate')

    # =========================================================================
    # RTAB-Map SLAM Node — Mapping Mode
    # =========================================================================
    # Parameters optimized for Raspberry Pi 4:
    #   - Reduced feature extraction
    #   - Lower resolution processing
    #   - Memory management for 4GB RAM
    # =========================================================================
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # Database
            'database_path': map_db_path,
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_scan': False,

            # Frame IDs
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # Mapping parameters
            'Mem/IncrementalMemory': 'true',    # Enable mapping (not just localization)
            'Mem/InitWMWithAllNodes': 'false',

            # Feature detection — reduced for Pi performance
            'Kp/MaxFeatures': '200',            # Max keypoints (default 500)
            'Kp/DetectorStrategy': '6',          # ORB detector (fast on ARM)
            'Kp/NNStrategy': '1',                # kd-tree search

            # Loop closure detection
            'RGBD/LoopClosureReextractFeatures': 'true',
            'Vis/MaxFeatures': '200',
            'Vis/MinInliers': '10',

            # Optimization
            'RGBD/OptimizeMaxError': '3.0',
            'Optimizer/Strategy': '1',           # g2o
            'Optimizer/Iterations': '10',

            # Memory management (critical for 4GB Pi)
            'Mem/STMSize': '20',
            'Rtabmap/MemoryThr': '300',          # Max nodes in working memory
            'Rtabmap/DetectionRate': frame_rate,

            # Grid map generation
            'Grid/FromDepth': 'false',
            'Grid/CellSize': '0.10',             # 10cm grid cells

            # IMU integration
            'wait_for_transform_duration': '0.2',
        }],
        remappings=[
            ('rgb/image', camera_topic),
            ('rgb/camera_info', '/camera/camera_info'),
            ('imu', imu_topic),
        ],
    )

    # =========================================================================
    # Visual Odometry Node
    # =========================================================================
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': False,
            'subscribe_rgb': True,

            # ORB features for speed on ARM
            'Odom/Strategy': '0',                # Frame-to-map
            'OdomF2M/MaxSize': '1000',
            'Vis/MaxFeatures': '200',
            'Vis/CorType': '0',                  # Features matching

            'publish_tf': True,
            'wait_for_transform': True,
        }],
        remappings=[
            ('rgb/image', camera_topic),
            ('rgb/camera_info', '/camera/camera_info'),
            ('imu', imu_topic),
        ],
    )

    # =========================================================================
    # Static TF: base_link → camera_frame
    # =========================================================================
    static_tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.0', '0.0', '0.15',    # x, y, z (camera ~15cm above base)
            '0.0', '0.0', '0.0',     # roll, pitch, yaw
            'base_link', 'camera_frame'
        ],
    )

    # =========================================================================
    # Static TF: base_link → imu_frame
    # =========================================================================
    static_tf_base_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=[
            '0.0', '0.0', '0.12',
            '0.0', '0.0', '0.0',
            'base_link', 'imu_frame'
        ],
    )

    # =========================================================================
    # Launch description
    # =========================================================================
    return LaunchDescription([
        # Arguments
        map_db_path_arg,
        use_sim_arg,
        camera_topic_arg,
        imu_topic_arg,
        frame_rate_arg,

        # Log
        LogInfo(msg=['MAPPING MODE — Saving map to: ', map_db_path]),

        # TF
        static_tf_base_camera,
        static_tf_base_imu,

        # SLAM
        rtabmap_odom_node,
        rtabmap_slam_node,
    ])
