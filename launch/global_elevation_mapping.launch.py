import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    global_elevation_mapping_config = os.path.join(
        get_package_share_directory('global_elevation_mapping'),
        'config', 'global_elevation_mapping.yaml'
    )
        
    global_elevation_mapping = Node(
            package='global_elevation_mapping',            
            executable='global_elevation_mapping_node',
            name='global_elevation_mapping',
            parameters = [
                global_elevation_mapping_config,                
                {'use_sim_time': use_sim_time}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
    )  
    
    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[global_elevation_mapping_config]
    )

    return LaunchDescription([
        global_elevation_mapping,
        grid_map_visualization_node
        ])