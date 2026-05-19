"""UMPC에서 실행: USB 웹캠 4개 → ROS2 Image 토픽 발행."""
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


class CamDriverNode(Node):
    """USB 웹캠 4개를 각각 별도 스레드로 읽어 ROS2 Image 토픽으로 발행.

    카메라 연결이 끊기면 자동으로 재연결을 시도한다.
    MJPG 포맷으로 캡처해 USB 대역폭 사용을 줄인다.
    """

    def __init__(self) -> None:
        super().__init__('cam_driver')

        for i in range(4):
            self.declare_parameter(f'device{i}', f'/dev/video{i * 2}')
            self.declare_parameter(f'topic{i}',  f'/camera/cam{i}/image_raw')

        self.declare_parameter('width',  640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps',    30.0)
        # 각 카메라별 해상도 (0이면 위 width/height 사용). 예: 파노라마용 전면(cam0)만 고해상도.
        for i in range(4):
            self.declare_parameter(f'width{i}',  0)
            self.declare_parameter(f'height{i}', 0)

        gw = int(self.get_parameter('width').value)
        gh = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)

        dims: list[tuple[int, int]] = []
        for i in range(4):
            wi = int(self.get_parameter(f'width{i}').value)
            hi = int(self.get_parameter(f'height{i}').value)
            dims.append((wi if wi > 0 else gw, hi if hi > 0 else gh))

        self._bridge  = CvBridge()
        self._running = True
        self._threads: list[threading.Thread] = []

        for i in range(4):
            device = self.get_parameter(f'device{i}').get_parameter_value().string_value
            topic  = self.get_parameter(f'topic{i}').get_parameter_value().string_value
            pub    = self.create_publisher(Image, topic, SENSOR_QOS)
            width, height = dims[i]

            t = threading.Thread(
                target=self._cam_loop,
                args=(i, device, topic, pub, width, height, fps),
                daemon=True,
            )
            t.start()
            self._threads.append(t)
            self.get_logger().info(f'cam{i}: {device} → {topic} @ {width}x{height}')

    def _open_cam(self, device: str, width: int, height: int, fps: float) -> cv2.VideoCapture:
        try:
            src = int(device)
        except ValueError:
            src = device

        cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # USB 대역폭 절약
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS,          fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)    # 지연 최소화
        return cap

    def _cam_loop(
        self,
        idx: int,
        device: str,
        topic: str,
        pub,
        width: int,
        height: int,
        fps: float,
    ) -> None:
        frame_id  = f'cam{idx}'
        fail_cnt  = 0
        cap: cv2.VideoCapture | None = None

        while self._running:
            # 카메라 열기 (또는 재연결)
            if cap is None or not cap.isOpened():
                try:
                    cap = self._open_cam(device, width, height, fps)
                    if not cap.isOpened():
                        raise RuntimeError('open failed')
                    self.get_logger().info(f'cam{idx} connected: {device}')
                    fail_cnt = 0
                except Exception as e:
                    self.get_logger().warn(f'cam{idx} open error ({device}): {e}')
                    time.sleep(2.0)
                    continue

            ret, frame = cap.read()
            if not ret:
                fail_cnt += 1
                if fail_cnt > 10:
                    self.get_logger().warn(f'cam{idx} stream lost, reconnecting...')
                    cap.release()
                    cap = None
                    fail_cnt = 0
                time.sleep(0.01)
                continue

            fail_cnt = 0
            msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            pub.publish(msg)

    def destroy_node(self) -> bool:
        self._running = False
        for t in self._threads:
            t.join(timeout=3.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CamDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
