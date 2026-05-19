"""기지국(노트북) launch: GStreamer 수신 + 통합 GUI."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # ── 수신 설정 ─────────────────────────────────────────────
        DeclareLaunchArgument('listen_port',   default_value='5000'),
        DeclareLaunchArgument('stream_fps',    default_value='30.0'),

        # ── 저장 경로 ─────────────────────────────────────────────
        DeclareLaunchArgument('save_dir',      default_value='~/camera_captures'),

        # ═══════════════════════════════════════════════════════════
        # gst_receiver: UDP H.264 수신 → /camera/merged/image_raw
        # ═══════════════════════════════════════════════════════════
        Node(
            package='science_gui',
            executable='gst_receiver',
            name='gst_receiver',
            output='screen',
            parameters=[{
                'output_topic': '/camera/merged/image_raw',
                'listen_port':  LaunchConfiguration('listen_port'),
                'fps':          LaunchConfiguration('stream_fps'),
            }],
        ),

        # ═══════════════════════════════════════════════════════════
        # mission_gui: 통합 제어 GUI
        #   - /camera/merged/image_raw  구독 (영상)
        #   - /panorama/result          구독 (파노라마 자동저장)
        #   - /scilab/cmd               발행 (사이언스랩 제어)
        #   - /scilab/feedback          구독
        #   - /npk/data                 구독 (자동 CSV 저장)
        #   - /spectrometer/*           구독/발행
        #   - /mission/panorama/trigger 서비스 클라이언트
        #   - /rover/trigger_launch     서비스 클라이언트 (로버 rover_daemon 전제)
        # ═══════════════════════════════════════════════════════════
        Node(
            package='science_gui',
            executable='mission_gui',
            name='mission_gui',
            output='screen',
            parameters=[{
                'topic_merged': '/camera/merged/image_raw',
                'save_dir':     LaunchConfiguration('save_dir'),
            }],
        ),
    ])
