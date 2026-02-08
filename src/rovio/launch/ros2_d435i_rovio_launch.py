#!/usr/bin/env python3
"""
Launch file for Intel RealSense D435i + ROVIO VIO
Starts RealSense camera with IMU and ROVIO node with remapped topics
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paths
    rovio_share = get_package_share_directory('rovio')
    realsense_share = get_package_share_directory('realsense2_camera')
    
    # D435i config file
    config_file = os.path.join(rovio_share, 'cfg', 'd435i_config.yaml')
    
    # Launch arguments
    enable_visualization = DeclareLaunchArgument(
        'enable_visualization',
        default_value='false',
        description='Enable ROVIO visualization'
    )
    
    # RealSense D435i launch with IMU enabled
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            # Enable sensors
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_infra1': 'true',  # Use infrared camera (grayscale) for ROVIO
            'enable_infra2': 'false',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',  # Publish combined IMU topic
            
            # Infrared camera resolution/fps
            'infra_rgb': 'false',  # Keep as grayscale (mono8)
            
            # Pointcloud/alignment (disable for performance)
            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
            
            # Camera namespace
            'camera_name': 'camera',
        }.items()
    )
    
    # ROVIO node with topic remapping
    rovio_node = Node(
        package='rovio',
        executable='rovio_node',
        name='rovio',
        output='screen',
        parameters=[{
            'filter_config': config_file,
            'camera0_config': '',  # Using intrinsics from config_file
            'enable_frame_visualization': LaunchConfiguration('enable_visualization'),
        }],
        remappings=[
            # Map RealSense topics to ROVIO expected topics
            ('/cam0/image_raw', '/camera/camera/infra1/image_rect_raw'),  # Use IR camera (grayscale)
            ('/imu0', '/camera/camera/imu'),
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        enable_visualization,
        realsense_launch,
        rovio_node,
    ])
