"""로버(UMPC) launch: 카메라 드라이버 + 합성 송신 + 사이언스랩 브릿지들."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # ── 네트워크 ──────────────────────────────────────────────
        DeclareLaunchArgument('basestation_ip', default_value='192.168.0.90'),
        DeclareLaunchArgument('stream_port',    default_value='5000'),

        # ── 카메라 장치 ────────────────────────────────────────────
        # USB 허브 사용 시 /dev/videoN 은 재연결마다 바뀔 수 있음.
        # 고정하려면: ls -l /dev/v4l/by-path/
        # 각 물리 포트에 맞는 '*-video-index0' 경로를 device0~3에 매핑 (전·후·soil·cashe 순).
        # 실행 예:
        #   ros2 launch science_gui rover.launch.py \
        #     device0:=/dev/v4l/by-path/<본인경로>-video-index0 \
        #     device1:=/dev/v4l/by-path/<본인경로>-video-index0 ...
        DeclareLaunchArgument('device0', default_value='/dev/video0'),
        DeclareLaunchArgument('device1', default_value='/dev/video2'),
        DeclareLaunchArgument('device2', default_value='/dev/video4'),
        DeclareLaunchArgument('device3', default_value='/dev/video6'),

        # ── 카메라 해상도 / FPS ───────────────────────────────────
        DeclareLaunchArgument('cam_width',  default_value='640'),
        DeclareLaunchArgument('cam_height', default_value='480'),
        DeclareLaunchArgument('cam_fps',    default_value='30.0'),
        # 전면(cam0): 파노라마 등 고화질용 (USB 대역 여유 없으면 낮추거나 cam_*와 동일하게)
        DeclareLaunchArgument('pano_cam_width',  default_value='1280'),
        DeclareLaunchArgument('pano_cam_height', default_value='720'),

        # ── 스트리밍 인코딩 ───────────────────────────────────────
        DeclareLaunchArgument('tile_width',  default_value='640'),
        DeclareLaunchArgument('tile_height', default_value='360'),
        DeclareLaunchArgument('bitrate',     default_value='2000'),

        # ── 시리얼 장치 ───────────────────────────────────────────
        DeclareLaunchArgument('scilab_device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('scilab_baud',   default_value='115200'),
        DeclareLaunchArgument('npk_device',    default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('npk_baud',      default_value='4800'),
        DeclareLaunchArgument('npk_interval',  default_value='4.0'),

        # ── 파노라마 ──────────────────────────────────────────────
        DeclareLaunchArgument('pano_camera',    default_value='/camera/front/image_raw'),
        DeclareLaunchArgument('pano_save_dir',  default_value='~/camera_captures/panorama'),
        DeclareLaunchArgument('pano_start_ang', default_value='-90.0'),
        DeclareLaunchArgument('pano_end_ang',   default_value='90.0'),
        DeclareLaunchArgument('pano_step_ang',  default_value='30.0'),
        DeclareLaunchArgument('pano_settle',    default_value='2.0'),

        # ═══════════════════════════════════════════════════════════
        # cam_driver: USB 웹캠 → ROS2 Image 토픽
        #   /camera/front/image_raw
        #   /camera/back/image_raw
        #   /camera/soil/image_raw
        #   (device3/cashe 는 미연결 시 무시)
        # ═══════════════════════════════════════════════════════════
        Node(
            package='science_gui',
            executable='cam_driver',
            name='cam_driver',
            output='screen',
            parameters=[{
                'device0': LaunchConfiguration('device0'),
                'device1': LaunchConfiguration('device1'),
                'device2': LaunchConfiguration('device2'),
                'device3': LaunchConfiguration('device3'),
                'topic0':  '/camera/front/image_raw',
                'topic1':  '/camera/back/image_raw',
                'topic2':  '/camera/soil/image_raw',
                'topic3':  '/camera/cashe/image_raw',
                'width':   LaunchConfiguration('cam_width'),
                'height':  LaunchConfiguration('cam_height'),
                'width0':  LaunchConfiguration('pano_cam_width'),
                'height0': LaunchConfiguration('pano_cam_height'),
                'fps':     LaunchConfiguration('cam_fps'),
            }],
        ),

        # ═══════════════════════════════════════════════════════════
        # gst_sender: 카메라 3개 → 1x3 가로 합성 → UDP H.264 송신
        #   목적지: basestation_ip:stream_port
        #   최종 해상도: (tile_width×3) × tile_height
        # ═══════════════════════════════════════════════════════════
        Node(
            package='science_gui',
            executable='gst_sender',
            name='gst_sender',
            output='screen',
            parameters=[{
                'num_cameras': 3,
                'topic_cam0':  '/camera/front/image_raw',
                'topic_cam1':  '/camera/back/image_raw',
                'topic_cam2':  '/camera/soil/image_raw',
                'host':        LaunchConfiguration('basestation_ip'),
                'port':        LaunchConfiguration('stream_port'),
                'fps':         LaunchConfiguration('cam_fps'),
                'tile_width':  LaunchConfiguration('tile_width'),
                'tile_height': LaunchConfiguration('tile_height'),
                'bitrate':     LaunchConfiguration('bitrate'),
            }],
        ),

        # ═══════════════════════════════════════════════════════════
        # scilab_bridge: /scilab/cmd → Arduino Due 시리얼 → 피드백
        # ═══════════════════════════════════════════════════════════
        Node(
            package='science_gui',
            executable='scilab_bridge',
            name='scilab_bridge',
            output='screen',
            parameters=[{
                'device':    LaunchConfiguration('scilab_device'),
                'baud_rate': LaunchConfiguration('scilab_baud'),
            }],
        ),

        # ═══════════════════════════════════════════════════════════
        # npk_bridge: NPK 토양 센서 Modbus RTU → /npk/data (JSON)
        # ═══════════════════════════════════════════════════════════
        Node(
            package='science_gui',
            executable='npk_bridge',
            name='npk_bridge',
            output='screen',
            parameters=[{
                'device':        LaunchConfiguration('npk_device'),
                'baud_rate':     LaunchConfiguration('npk_baud'),
                'read_interval': LaunchConfiguration('npk_interval'),
            }],
        ),

        # ═══════════════════════════════════════════════════════════
        # mission_panorama: /mission/panorama/trigger 서비스 수신 →
        #   서보 자동 회전 → 촬영 → 스티칭 → /panorama/result 발행
        # ═══════════════════════════════════════════════════════════
        Node(
            package='science_gui',
            executable='mission_panorama',
            name='mission_panorama',
            output='screen',
            parameters=[{
                'camera_topic':        LaunchConfiguration('pano_camera'),
                'gps_topic':           '/mb00b/fix',
                'heading_topic':           '/imu/gnss_heading',
                'heading_yaw_deg_topic':   '/gnss_heading/yaw_deg',
                'heading_valid_topic':     '/gnss_heading/valid',
                'result_topic':        '/panorama/result',
                'save_dir':            LaunchConfiguration('pano_save_dir'),
                'start_angle':         LaunchConfiguration('pano_start_ang'),
                'end_angle':           LaunchConfiguration('pano_end_ang'),
                'step_angle':          LaunchConfiguration('pano_step_ang'),
                'settle_time':         LaunchConfiguration('pano_settle'),
            }],
        ),
    ])
