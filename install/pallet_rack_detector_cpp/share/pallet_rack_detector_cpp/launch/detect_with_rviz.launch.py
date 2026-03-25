from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pallet_rack_detector_cpp')
    rviz_config = os.path.join(pkg_share, 'rviz', 'detect.rviz')

    # SICK driver
    sick_node = Node(
        package='sick_safetyscanners2',
        executable='sick_safetyscanners2_node',
        name='sick_safetyscanners2_node',
        output='screen',
        parameters=[{
            'sensor_ip': '192.168.1.85',
            'host_ip':   '192.168.1.222',
            'interface_ip': '192.168.1.222',
            'host_udp_port': 0,
            'frame_id': 'laser'
        }],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    # Detector
    detector_node = Node(
        package='pallet_rack_detector_cpp',
        executable='pallet_rack_detector',
        name='pallet_rack_detector',
        output='screen',
        parameters=[{
            'rack_width': 1.09,
            'rack_length': 1.09,
            'geom_tol': 0.12,
            'cluster_max_gap': 0.08,
            'cluster_min_pts': 3,
            'leg_radius_min': 0.01,
            'leg_radius_max': 0.06,

            # ROI: front only, within 3m and +-45 deg
            'roi_max_range': 3.0,
            'roi_min_x': 0.0,
            'roi_half_fov_deg': 45.0,
        }]
    )

    # RViz2 (chạy trong môi trường sạch để né snap libpthread)
    rviz_cmd_str = (
        "source /opt/ros/humble/setup.bash && "
        + ("rviz2 -d " + rviz_config if os.path.exists(rviz_config) else "rviz2")
    )

    rviz_node = ExecuteProcess(
        cmd=[
            'env', '-i',
            f'HOME={os.environ.get("HOME","")}',
            f'USER={os.environ.get("USER","")}',
            f'DISPLAY={os.environ.get("DISPLAY","")}',
            f'XAUTHORITY={os.environ.get("XAUTHORITY","")}',
            'PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
            'bash', '-lc', rviz_cmd_str
        ],
        output='screen'
    )

    return LaunchDescription([
        sick_node,
        detector_node,
        # rviz_node
    ])
