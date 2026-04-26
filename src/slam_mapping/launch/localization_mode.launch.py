# =============================================================================
# localization_mode.launch.py — SLAM Localization Mode (Runtime)
# =============================================================================
# Launches RTAB-Map in localization-only mode using a pre-built map.
# Used during normal operation — the user navigates with position
# awareness relative to the known environment.
#
# Prerequisites: Run mapping_mode.launch.py first to create house_map.db
# Usage: ros2 launch smart_assistive_system localization_mode.launch.py
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        description='Path to the pre-built RTAB-Map database'
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
        'frame_rate', default_value='3.0',
        description='Localization rate (Hz) — lower than mapping for battery'
    )

    # =========================================================================
    # Substitutions
    # =========================================================================
    map_db_path = LaunchConfiguration('map_db_path')
    camera_topic = LaunchConfiguration('camera_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    frame_rate = LaunchConfiguration('frame_rate')

    # =========================================================================
    # RTAB-Map — Localization Mode
    # =========================================================================
    # Key differences from mapping mode:
    #   - Mem/IncrementalMemory = false (no new nodes)
    #   - Mem/InitWMWithAllNodes = true (load all map data)
    #   - Lower detection rate (save CPU/battery)
    # =========================================================================
    rtabmap_localization_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # Database — load existing map
            'database_path': map_db_path,
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_scan': False,

            # Frame IDs
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # Localization-only mode
            'Mem/IncrementalMemory': 'false',     # Don't add new nodes
            'Mem/InitWMWithAllNodes': 'true',      # Load entire map into memory

            # Feature detection — same as mapping for consistency
            'Kp/MaxFeatures': '200',
            'Kp/DetectorStrategy': '6',            # ORB
            'Kp/NNStrategy': '1',

            # Loop closure for relocalization
            'RGBD/LoopClosureReextractFeatures': 'true',
            'Vis/MaxFeatures': '200',
            'Vis/MinInliers': '8',                 # Slightly lower for robustness

            # Optimization
            'RGBD/OptimizeMaxError': '3.0',
            'Optimizer/Strategy': '1',

            # Detection rate — lower for localization (saves CPU)
            'Rtabmap/DetectionRate': frame_rate,

            # Publish known map
            'Grid/FromDepth': 'false',
            'Grid/CellSize': '0.10',

            # Transform
            'wait_for_transform_duration': '0.2',

            # Publish localization result as pose
            'publish_tf': True,
        }],
        remappings=[
            ('rgb/image', camera_topic),
            ('rgb/camera_info', '/camera/camera_info'),
            ('imu', imu_topic),
        ],
    )

    # =========================================================================
    # Visual Odometry — same as mapping mode
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

            'Odom/Strategy': '0',
            'OdomF2M/MaxSize': '1000',
            'Vis/MaxFeatures': '200',
            'Vis/CorType': '0',

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
    # Localization publisher — publishes current position for cognitive layer
    # =========================================================================
    # This node converts RTAB-Map's pose output to a simple JSON message
    # on /slam/position for the cognitive engine to consume.
    # =========================================================================
    localization_bridge_node = Node(
        package='smart_assistive_system',
        executable='cognitive_engine_node',
        name='localization_status',
        output='screen',
        # The cognitive engine will read /rtabmap/localization_pose
        # This reuses the engine with localization awareness
    )

    # =========================================================================
    # Static transforms
    # =========================================================================
    static_tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.0', '0.0', '0.15',
            '0.0', '0.0', '0.0',
            'base_link', 'camera_frame'
        ],
    )

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
        camera_topic_arg,
        imu_topic_arg,
        frame_rate_arg,

        # Log
        LogInfo(msg=['LOCALIZATION MODE — Loading map from: ', map_db_path]),

        # TF
        static_tf_base_camera,
        static_tf_base_imu,

        # SLAM (localization only)
        rtabmap_odom_node,
        rtabmap_localization_node,
    ])
