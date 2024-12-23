import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the gazebo project launch file
    gazebo_pkg_dir = get_package_share_directory('gazebo_project')
    gazebo_launch_file = os.path.join(gazebo_pkg_dir, 'launch', 'town.launch.py')
    
    # Include the gazebo simulation launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
    )
    
    # Launch your perception node
    perception_node = Node(
        package='perception_and_controller',
        executable='perception',
        name='perception_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch your controller node
    controller_node = Node(
        package='perception_and_controller',
        executable='controller',
        name='controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch the occupancy grid node
    occupancy_grid_node = Node(
        package='perception_and_controller',
        executable='occupancy_grid',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Add a static transform from world to base_link
    world_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )

    # Launch RViz
    rviz_config_dir = os.path.join(gazebo_pkg_dir, 'rviz', 'config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo_launch,
        world_to_base_link,  # Add this before your nodes
        perception_node,
        #controller_node,
        #occupancy_grid_node,
        rviz_node,
    ])