import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_namespace = launch.substitutions.LaunchConfiguration('robot_namespace', default='')
    rviz_enabled = launch.substitutions.LaunchConfiguration('rviz', default='true')
    pointcloud_topic = launch.substitutions.LaunchConfiguration('pointcloud_topic', default='lidar')
    imu_topic = launch.substitutions.LaunchConfiguration('imu_topic', default='imu')
    
    dlo_package_path = get_package_share_directory('direct_lidar_odometry')

    dlo_odom_node = Node(
        package='direct_lidar_odometry',
        executable='dlo_odom_node',
        namespace=[robot_namespace],
        name='dlo_odom',
        output='screen',
        parameters=[{
            'dlo_param_file': dlo_package_path + '/cfg/dlo.yaml',
            'params_file': dlo_package_path + '/cfg/params.yaml'
        }],
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('odom', 'dlo/odom_node/odom'),
            ('pose', 'dlo/odom_node/pose'),
            ('kfs', 'dlo/odom_node/odom/keyframe'),
            ('keyframe', 'dlo/odom_node/pointcloud/keyframe')
        ]
    )

    dlo_map_node = Node(
        package='direct_lidar_odometry',
        executable='dlo_map_node',
        namespace=[robot_namespace],
        name='dlo_map',
        output='screen',
        parameters=[{
            'dlo_param_file': dlo_package_path + '/cfg/dlo.yaml'
        }],
        remappings=[
            ('keyframes', 'dlo/odom_node/pointcloud/keyframe')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlo_rviz',
        arguments=['-d', dlo_package_path + '/launch/dlo.rviz']
    )

    return LaunchDescription([
        dlo_odom_node,
        dlo_map_node,
        rviz_node
    ])
