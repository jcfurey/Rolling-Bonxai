from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction


def generate_launch_description():
    # ------------------------------------------------------------------
    # HARD-CODED CONFIG — EDIT HERE ONLY
    # ------------------------------------------------------------------

    BAG_NAME = 'rolling_bonxai_fieldtest'

    TOPICS = [
        '/tf',
        '/tf_static',
        '/rolling_bonxai/occupied_voxels',
        '/rolling_bonxai/chunk_stats',
        '/rolling_bonxai/quick_occupancy_stats',
        '/rolling_bonxai/chunk_markers',
        '/rolling_bonxai/transition_state',
        '/zed/zed_node/point_cloud/cloud_registered'
    ]

    START_DELAY_SEC = 2.0  # give nodes time to come up

    # ------------------------------------------------------------------
    # Rosbag record process
    # ------------------------------------------------------------------

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'mcap',
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '-o', BAG_NAME,
            '--topics', *TOPICS,
        ],
        output='screen',
        emulate_tty=True,
    )

    # Delay start slightly to avoid empty bags
    rosbag_record_delayed = TimerAction(
        period=START_DELAY_SEC,
        actions=[rosbag_record],
    )

    return LaunchDescription([
        rosbag_record_delayed
    ])
