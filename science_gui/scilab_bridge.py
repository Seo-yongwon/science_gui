"""Jetson에서 실행: Arduino Due 시리얼 ↔ ROS2 토픽 브릿지.

GUI(기지국)에서 /scilab/cmd 로 명령 String을 보내면 시리얼로 전달하고,
Arduino가 보내는 피드백 라인을 /scilab/feedback 으로 발행한다.
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class ScilabBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('scilab_bridge')

        self.declare_parameter('device', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        device = self.get_parameter('device').get_parameter_value().string_value
        baud = int(self.get_parameter('baud_rate').value)

        self._ser = serial.Serial(device, baud, timeout=0.2)
        if not self._ser.is_open:
            raise RuntimeError(f'Cannot open {device}')
        self.get_logger().info(f'Scilab serial: {device} @ {baud}')

        self._fb_pub = self.create_publisher(String, '/scilab/feedback', 10)
        self.create_subscription(String, '/scilab/cmd', self._on_cmd, 10)

        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def _on_cmd(self, msg: String) -> None:
        cmd = msg.data.strip()
        if not cmd:
            return
        try:
            self._ser.write((cmd + '\n').encode())
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write failed: {e}')

    def _read_loop(self) -> None:
        while self._running:
            try:
                line = self._ser.readline().decode(errors='ignore').strip()
                if line:
                    msg = String()
                    msg.data = line
                    self._fb_pub.publish(msg)
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read error: {e}')
                import time
                time.sleep(0.5)

    def destroy_node(self) -> bool:
        self._running = False
        self._read_thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScilabBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
