"""Jetson에서 실행: 파노라마 촬영 시퀀스.

Arduino Due의 Pan Servo (0~180도)를 scilab_bridge를 통해 제어하고,
카메라 영상을 캡처 → 스티칭 → 어노테이션 → 저장/발행.
"""
from __future__ import annotations

import os
import threading
from enum import Enum, auto
from datetime import datetime
from math import atan2, degrees, sqrt

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, Imu, NavSatFix
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class _St(Enum):
    IDLE = auto()
    ROTATING = auto()
    RETURNING = auto()
    STITCHING = auto()


class MissionPanoramaNode(Node):
    """
    시퀀스: trigger → 서보 0도(=-90)로 이동 → 30도씩 회전하며 캡처 →
    180도(=+90)까지 → 중앙(90도=0)으로 복귀 → 스티칭 → 어노테이션 → 저장
    
    Arduino 서보 매핑: 논리 -90~+90도 → 서보 0~180도 (offset=90)
    """

    SERVO_OFFSET = 90

    def __init__(self) -> None:
        super().__init__('mission_panorama')

        self.declare_parameter('camera_topic', '/camera/panorama/image_raw')
        self.declare_parameter('gps_topic', '/mb00b/fix')
        self.declare_parameter('heading_topic', '/imu/gnss_heading')
        self.declare_parameter('heading_yaw_deg_topic', '/gnss_heading/yaw_deg')
        self.declare_parameter('heading_valid_topic', '/gnss_heading/valid')
        self.declare_parameter('result_topic', '/panorama/result')
        self.declare_parameter('save_dir', os.path.expanduser('~/camera_captures/panorama'))
        self.declare_parameter('start_angle', -90.0)
        self.declare_parameter('end_angle', 90.0)
        self.declare_parameter('step_angle', 30.0)
        self.declare_parameter('settle_time', 2.0)

        cam_topic = self._p_str('camera_topic')
        gps_topic = self._p_str('gps_topic')
        heading_topic = self._p_str('heading_topic')
        heading_yaw_topic = self._p_str('heading_yaw_deg_topic')
        heading_valid_topic = self._p_str('heading_valid_topic')
        result_topic = self._p_str('result_topic')

        self._save_dir = self._p_str('save_dir')
        self._settle = float(self.get_parameter('settle_time').value)

        start = float(self.get_parameter('start_angle').value)
        end = float(self.get_parameter('end_angle').value)
        step = float(self.get_parameter('step_angle').value)
        self._angles: list[float] = []
        a = start
        while a <= end + 0.01:
            self._angles.append(round(a, 1))
            a += step

        self._bridge = CvBridge()
        self._state = _St.IDLE
        self._idx = 0
        self._captured: list[np.ndarray] = []
        self._move_t = None

        self._frame: np.ndarray | None = None
        self._gps: NavSatFix | None = None
        self._heading: float = 0.0
        self._hdg_std_deg: float | None = None
        self._hdg_valid: bool = False
        self._heading_has_data: bool = False
        self._heading_from_imu: bool = False
        self._snap_gps: NavSatFix | None = None
        self._snap_heading: float = 0.0
        self._snap_hdg_std_deg: float | None = None
        self._snap_hdg_valid: bool = False
        self._snap_heading_has_data: bool = False

        self.create_subscription(Image, cam_topic, self._cb_img, SENSOR_QOS)
        self.create_subscription(NavSatFix, gps_topic, self._cb_gps, SENSOR_QOS)
        self.create_subscription(Imu, heading_topic, self._cb_heading, SENSOR_QOS)
        self.create_subscription(Float32, heading_yaw_topic, self._cb_heading_yaw_deg, 10)
        self.create_subscription(Bool, heading_valid_topic, self._cb_heading_valid, 10)

        self._cmd_pub = self.create_publisher(String, '/scilab/cmd', 10)
        self._result_pub = self.create_publisher(Image, result_topic, 1)

        self.create_service(Trigger, '/mission/panorama/trigger', self._on_trigger)
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'mission_panorama ready  angles={self._angles}  settle={self._settle}s\n'
            f'  camera: {cam_topic}\n'
            f'  motor:  Arduino Pan Servo via /scilab/cmd\n'
            f'  gps:    {gps_topic}\n'
            f'  heading:{heading_topic}\n'
            f'  fallback:{heading_yaw_topic}, {heading_valid_topic}\n'
            f'  Call /mission/panorama/trigger to start.'
        )

    def _p_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    # ── callbacks ──────────────────────────────────────────────

    def _cb_img(self, msg: Image) -> None:
        self._frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _cb_gps(self, msg: NavSatFix) -> None:
        self._gps = msg

    @staticmethod
    def _quat_to_enu_yaw_deg(q) -> float:
        """Quaternion → REP-103 ENU yaw (deg). 0=East, CCW+."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return degrees(atan2(siny_cosp, cosy_cosp))

    @staticmethod
    def _enu_yaw_to_compass_bearing_deg(enu_yaw_deg: float) -> float:
        """ENU yaw → compass bearing (deg). 0=North, CW+ (u-blox relPosHeading convention)."""
        return (90.0 - enu_yaw_deg) % 360.0

    @classmethod
    def _parse_heading_imu(cls, msg: Imu) -> tuple[float, float | None, bool]:
        """Imu 쿼터니언 → compass bearing, orientation_covariance[8](yaw 분산, rad²) → 1σ."""
        q = msg.orientation
        enu_yaw_deg = cls._quat_to_enu_yaw_deg(q)
        bearing_deg = cls._enu_yaw_to_compass_bearing_deg(enu_yaw_deg)

        cov = msg.orientation_covariance
        if len(cov) < 9 or cov[8] < 0.0:
            return bearing_deg, None, False

        yaw_std_deg = degrees(sqrt(cov[8]))
        # gnss_heading_bridge: invalid 시 std ≈ 999 deg
        valid = yaw_std_deg < 90.0
        return bearing_deg, yaw_std_deg, valid

    def _cb_heading(self, msg: Imu) -> None:
        self._heading, self._hdg_std_deg, self._hdg_valid = self._parse_heading_imu(msg)
        self._heading_has_data = True
        self._heading_from_imu = True

    def _cb_heading_yaw_deg(self, msg: Float32) -> None:
        if self._heading_from_imu:
            return
        self._heading = self._enu_yaw_to_compass_bearing_deg(msg.data)
        self._heading_has_data = True

    def _cb_heading_valid(self, msg: Bool) -> None:
        if self._heading_from_imu:
            return
        self._hdg_valid = msg.data

    # ── service ────────────────────────────────────────────────

    def _on_trigger(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self._state != _St.IDLE:
            res.success = False
            res.message = 'Already running'
            return res
        if self._frame is None:
            res.success = False
            res.message = 'No camera frame yet'
            return res
        self._begin()
        res.success = True
        res.message = f'Started: {len(self._angles)} captures'
        return res

    # ── motor command ─────────────────────────────────────────

    def _goto(self, logical_angle: float) -> None:
        """논리 각도(-90~+90)를 서보 각도(0~180)로 변환하여 전송."""
        servo_angle = int(round(logical_angle + self.SERVO_OFFSET))
        servo_angle = max(0, min(180, servo_angle))
        msg = String()
        msg.data = f'PAN_ANGLE:{servo_angle}'
        self._cmd_pub.publish(msg)
        self._move_t = self.get_clock().now()
        self.get_logger().info(f'Pan servo -> {servo_angle} deg (logical {logical_angle:.1f})')

    def _elapsed(self) -> float:
        if self._move_t is None:
            return 999.0
        return (self.get_clock().now() - self._move_t).nanoseconds / 1e9

    # ── sequence ───────────────────────────────────────────────

    def _begin(self) -> None:
        self._captured.clear()
        self._idx = 0
        self._snap_gps = self._gps
        self._snap_heading = self._heading
        self._snap_hdg_std_deg = self._hdg_std_deg
        self._snap_hdg_valid = self._hdg_valid
        self._snap_heading_has_data = self._heading_has_data
        if not self._heading_has_data:
            self.get_logger().warn(
                'No heading data received — check gnss_heading_bridge and GNSS fix'
            )
        else:
            self.get_logger().info(
                f'Heading snapshot: brg={self._heading:.1f} deg '
                f'valid={self._hdg_valid} source={"imu" if self._heading_from_imu else "yaw_deg"}'
            )
        self._goto(self._angles[0])
        self._state = _St.ROTATING
        self.get_logger().info('Panorama sequence started')

    def _tick(self) -> None:
        if self._state in (_St.IDLE, _St.STITCHING):
            return

        if self._state == _St.ROTATING:
            if self._elapsed() < self._settle:
                return
            if self._frame is not None:
                self._captured.append(self._frame.copy())
                self.get_logger().info(
                    f'Captured [{self._idx + 1}/{len(self._angles)}] '
                    f'at {self._angles[self._idx]:.1f} deg'
                )
            self._idx += 1
            if self._idx < len(self._angles):
                self._goto(self._angles[self._idx])
            else:
                self._goto(0.0)  # 중앙 복귀
                self._state = _St.RETURNING

        elif self._state == _St.RETURNING:
            if self._elapsed() >= self._settle:
                self._state = _St.STITCHING
                threading.Thread(target=self._stitch, daemon=True).start()

    # ── stitching ──────────────────────────────────────────────

    def _stitch(self) -> None:
        n = len(self._captured)
        self.get_logger().info(f'Stitching {n} images...')

        if n < 2:
            self.get_logger().error('Not enough images')
            self._state = _St.IDLE
            return

        stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)
        status, pano = stitcher.stitch(self._captured)

        if status != cv2.Stitcher_OK:
            self.get_logger().warn(
                f'Stitcher failed (status={status}), falling back to hconcat'
            )
            h_min = min(im.shape[0] for im in self._captured)
            resized = []
            for im in self._captured:
                if im.shape[0] != h_min:
                    s = h_min / im.shape[0]
                    im = cv2.resize(im, None, fx=s, fy=s)
                resized.append(im)
            pano = np.hstack(resized)
        else:
            pano = self._crop_black(pano)

        pano = self._annotate(pano)

        os.makedirs(self._save_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self._save_dir, f'panorama_{ts}.png')
        cv2.imwrite(path, pano)
        self.get_logger().info(f'Saved: {path}')

        out = self._bridge.cv2_to_imgmsg(pano, encoding='bgr8')
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'panorama'
        self._result_pub.publish(out)

        self._state = _St.IDLE
        self.get_logger().info('Panorama complete')

    # ── image processing ───────────────────────────────────────

    @staticmethod
    def _crop_black(img: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return img
        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        return img[y:y + h, x:x + w]

    def _annotate(self, img: np.ndarray) -> np.ndarray:
        gps = self._snap_gps
        heading = self._snap_heading
        hdg_std = self._snap_hdg_std_deg
        hdg_valid = self._snap_hdg_valid
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        lines = [ts]
        if gps is not None:
            lines.append(f'Lat:  {gps.latitude:.8f}')
            lines.append(f'Lon:  {gps.longitude:.8f}')
            cov = gps.position_covariance
            if gps.position_covariance_type > 0 and len(cov) >= 5 and cov[0] > 0:
                drms = sqrt(cov[0] + cov[4])
                lines.append(f'Acc:  {drms:.3f} m (DRMS)')
        else:
            lines.append('GPS: N/A')

        if not self._snap_heading_has_data:
            lines.append('Brg:  N/A')
        else:
            v_tag = 'valid' if hdg_valid else 'invalid'
            if hdg_std is not None and hdg_valid:
                lines.append(f'Brg:  {heading:.1f} deg +/- {hdg_std:.2f} deg ({v_tag})')
            else:
                lines.append(f'Brg:  {heading:.1f} deg ({v_tag})')

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.5, img.shape[1] / 2400.0)
        thick = max(1, int(scale * 2))
        lh = int(28 * scale)
        pad = int(10 * scale)

        max_tw = 0
        for ln in lines:
            (tw, _), _ = cv2.getTextSize(ln, font, scale, thick)
            max_tw = max(max_tw, tw)

        bw = max_tw + pad * 2
        bh = lh * len(lines) + pad * 2
        ih, iw = img.shape[:2]
        x0 = iw - bw - pad
        y0 = ih - bh - pad

        roi = img[y0:y0 + bh, x0:x0 + bw]
        dark = np.zeros_like(roi)
        img[y0:y0 + bh, x0:x0 + bw] = cv2.addWeighted(roi, 0.35, dark, 0.65, 0)

        for i, ln in enumerate(lines):
            ty = y0 + pad + lh * (i + 1) - int(4 * scale)
            cv2.putText(img, ln, (x0 + pad, ty), font, scale,
                        (255, 255, 255), thick, cv2.LINE_AA)

        return img


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionPanoramaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
