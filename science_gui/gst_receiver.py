"""기지국에서 실행: GStreamer UDP H.264 수신 → ROS2 Image 토픽 발행."""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class GstReceiverNode(Node):
    """GStreamer UDP H.264 수신 → ROS2 Image 토픽 발행.

    송신 노드보다 먼저 시작되어도 스트림이 올 때까지 대기하며,
    스트림이 끊기면 자동으로 재연결을 시도한다.
    """

    def __init__(self) -> None:
        super().__init__('gst_receiver')

        self.declare_parameter('output_topic', '/camera/merged/image_raw')
        self.declare_parameter('listen_port',  5000)
        self.declare_parameter('fps',          30.0)

        topic      = self.get_parameter('output_topic').get_parameter_value().string_value
        port       = int(self.get_parameter('listen_port').value)
        self._fps  = float(self.get_parameter('fps').value)

        # Wi‑Fi 등에서 RTP 패킷 순서/지터가 크면 디코더가 초록·깨짐을 낼 수 있음.
        # rtpjitterbuffer + udpsrc buffer-size 로 완화 (유선이면 더 안정적).
        self._pipe_str = (
            f'udpsrc port={port} buffer-size=2097152 '
            f'caps="application/x-rtp,encoding-name=H264,payload=96" ! '
            f'rtpjitterbuffer latency=200 ! '
            f'rtph264depay ! h264parse ! avdec_h264 ! '
            f'videoconvert ! video/x-raw,format=BGR ! '
            f'appsink drop=1 max-buffers=1 sync=false'
        )

        self._bridge  = CvBridge()
        self._pub     = self.create_publisher(Image, topic, SENSOR_QOS)
        self._running = True

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f'GStreamer receiver: udp://:{port} → {topic}')

    def _read_loop(self) -> None:
        cap:       cv2.VideoCapture | None = None
        fail_cnt:  int = 0

        try:
            while self._running:
                # 파이프라인 열기 (또는 재연결)
                if cap is None or not cap.isOpened():
                    self.get_logger().info('스트림 대기중...')
                    cap = cv2.VideoCapture(self._pipe_str, cv2.CAP_GSTREAMER)
                    if not cap.isOpened():
                        cap.release()
                        cap = None
                        time.sleep(1.0)
                        continue
                    self.get_logger().info('스트림 연결됨')
                    fail_cnt = 0

                ret, frame = cap.read()
                if not ret:
                    fail_cnt += 1
                    if fail_cnt > self._fps * 3:
                        self.get_logger().warn('스트림 끊김, 재연결...')
                        cap.release()
                        cap = None
                        fail_cnt = 0
                    else:
                        time.sleep(0.01)
                    continue

                fail_cnt = 0
                msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_merged'
                self._pub.publish(msg)

        finally:
            if cap is not None:
                cap.release()

    def destroy_node(self) -> bool:
        self._running = False
        self._thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GstReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
