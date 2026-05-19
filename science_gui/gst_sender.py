"""UMPC에서 실행: USB 카메라 4개 → 2x2 모자이크 → GStreamer UDP H.264 송신."""
from __future__ import annotations

import queue
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class GstSenderNode(Node):
    """USB 카메라 4개 토픽 → 2x2 모자이크 → GStreamer UDP H.264 송신.

    카메라별로 독립 구독해 최신 프레임을 저장하고,
    타이머가 일정 FPS로 모자이크를 합성해 인코딩 스레드에 전달한다.
    인코더가 바쁘면 프레임을 드롭해 지연 누적을 방지한다.
    """

    def __init__(self) -> None:
        super().__init__('gst_sender')

        self.declare_parameter('topic_cam0',  '/camera/front/image_raw')
        self.declare_parameter('topic_cam1',  '/camera/back/image_raw')
        self.declare_parameter('topic_cam2',  '/camera/soil/image_raw')
        self.declare_parameter('topic_cam3',  '/camera/cashe/image_raw')
        self.declare_parameter('host',        '192.168.1.30')
        self.declare_parameter('port',        5000)
        self.declare_parameter('fps',         30.0)
        self.declare_parameter('tile_width',  640)
        self.declare_parameter('tile_height', 360)
        self.declare_parameter('bitrate',     2000)   # kbps

        topics = [
            self.get_parameter(f'topic_cam{i}').get_parameter_value().string_value
            for i in range(4)
        ]
        host     = self.get_parameter('host').get_parameter_value().string_value
        port     = int(self.get_parameter('port').value)
        fps      = float(self.get_parameter('fps').value)
        self._tw = int(self.get_parameter('tile_width').value)
        self._th = int(self.get_parameter('tile_height').value)
        bitrate  = int(self.get_parameter('bitrate').value)

        self._bridge = CvBridge()

        # 카메라별 최신 프레임 (None = 아직 수신 전)
        self._frames: list[np.ndarray | None] = [None] * 4
        self._frame_lock = threading.Lock()

        # 인코딩 스레드 큐: maxsize=1 → 느리면 드롭
        self._send_queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)
        self._running = True
        self._writer = self._open_pipeline(host, port, fps, bitrate)
        self._encode_thread = threading.Thread(target=self._encode_loop, daemon=True)
        self._encode_thread.start()

        # 카메라별 독립 구독 (타임스탬프 동기화 없음)
        for i, topic in enumerate(topics):
            self.create_subscription(
                Image, topic,
                lambda msg, idx=i: self._on_frame(msg, idx),
                SENSOR_QOS,
            )

        # 모자이크 합성 타이머
        self.create_timer(1.0 / fps, self._tick)

        self.get_logger().info(
            f'GstSender ready\n'
            f'  cam0: {topics[0]}\n'
            f'  cam1: {topics[1]}\n'
            f'  cam2: {topics[2]}\n'
            f'  cam3: {topics[3]}\n'
            f'  dst : udp://{host}:{port}  '
            f'tile={self._tw}x{self._th}  fps={fps}  bitrate={bitrate}kbps'
        )

    def _open_pipeline(self, host: str, port: int, fps: float, bitrate: int) -> cv2.VideoWriter:
        w, h = self._tw * 2, self._th * 2
        pipe = (
            f'appsrc ! videoconvert ! '
            f'video/x-raw,format=I420,width={w},height={h},framerate={int(fps)}/1 ! '
            f'x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast ! '
            f'rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={host} port={port}'
        )
        writer = cv2.VideoWriter(pipe, cv2.CAP_GSTREAMER, 0, fps, (w, h))
        if not writer.isOpened():
            raise RuntimeError(f'GStreamer send pipeline failed:\n{pipe}')
        return writer

    def _on_frame(self, msg: Image, idx: int) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (self._tw, self._th))
        with self._frame_lock:
            self._frames[idx] = frame

    def _tick(self) -> None:
        with self._frame_lock:
            frames = list(self._frames)

        # 아직 수신 안 된 카메라는 검은 화면으로 대체
        blank = np.zeros((self._th, self._tw, 3), dtype=np.uint8)
        tiles = [f if f is not None else blank for f in frames]

        mosaic = np.vstack([
            np.hstack([tiles[0], tiles[1]]),
            np.hstack([tiles[2], tiles[3]]),
        ])

        try:
            self._send_queue.put_nowait(mosaic)
        except queue.Full:
            pass  # 인코더 바쁨 → 이번 프레임 드롭

    def _encode_loop(self) -> None:
        while self._running:
            try:
                mosaic = self._send_queue.get(timeout=1.0)
                self._writer.write(mosaic)
            except queue.Empty:
                continue

    def destroy_node(self) -> bool:
        self._running = False
        self._encode_thread.join(timeout=3.0)
        self._writer.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GstSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
