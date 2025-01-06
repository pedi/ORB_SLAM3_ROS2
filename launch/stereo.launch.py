from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vocabulary',
            default_value=PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'),
                'vocabulary',  # Assuming your vocab file is in the vocabulary directory
                'ORBvoc.txt'   # Replace with your actual vocabulary file name
            ]),
            description='Path to the ORB_SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'pangolin',
            default_value="True",
            description='Use the viewer'
        ),
        DeclareLaunchArgument(
            'yaml_file',
            default_value='sm2_stereo.yaml',
            description='Name of the ORB_SLAM3 YAML configuration file'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='SM2',
            description='Namespace of system'
        ),
        
        Node(
            package='orbslam3_ros2',
            executable='stereo',
            name='stereo_orbslam3',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[
                LaunchConfiguration('vocabulary'),
                PathJoinSubstitution([
                    FindPackageShare('orbslam3_ros2'),
                    'config',  
                    'stereo',
                    LaunchConfiguration('yaml_file')
                ]),
                'False',
                LaunchConfiguration('pangolin')
            ],
            remappings=[
                ('camera/left', 'left/image_raw'),
                ('camera/right', 'right/image_raw')
            ]
        )
    ])
