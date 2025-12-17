from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Trova il percorso del file YAML
    # Assicurati che il file si chiami 'kdl_params.yaml' e sia nella cartella config
    config = os.path.join(
        get_package_share_directory('ros2_kdl_package'),
        'config',
        'kdl_params.yaml'
    )

    return LaunchDescription([
        # 2. Avvia il nodo CLIENT
        Node(
            package='ros2_kdl_package',
            executable='ros2_kdl_client', # Questo deve corrispondere al nome in CMakeLists.txt
            name='ros2_kdl_client',
            output='screen',
            parameters=[config] # <--- Qui carichiamo i parametri dal file YAML dentro il Client!
        )
    ])