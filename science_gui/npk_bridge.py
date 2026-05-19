"""Jetson에서 실행: NPK 토양 센서(Modbus RTU) ↔ ROS2 토픽 브릿지.

4초 주기로 센서를 읽어 /npk/data (JSON String)로 발행한다.
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

NPK_REQUEST = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08])


class NPKBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('npk_bridge')

        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 4800)
        self.declare_parameter('read_interval', 4.0)

        device = self.get_parameter('device').get_parameter_value().string_value
        baud = int(self.get_parameter('baud_rate').value)
        interval = float(self.get_parameter('read_interval').value)

        self._ser = serial.Serial(device, baud, timeout=0.2)
        if not self._ser.is_open:
            raise RuntimeError(f'Cannot open {device}')
        self.get_logger().info(f'NPK serial: {device} @ {baud}')

        self._pub = self.create_publisher(String, '/npk/data', 10)
        self.create_timer(interval, self._read_sensor)

    def _read_sensor(self) -> None:
        try:
            self._ser.reset_input_buffer()
            self._ser.write(NPK_REQUEST)
            time.sleep(0.15)
            data = self._ser.read(19)
            if len(data) != 19:
                self.get_logger().debug('NPK: incomplete response')
                return
            if not self._check_crc(data):
                self.get_logger().debug('NPK: CRC mismatch')
                return
            parsed = self._parse(data)
            msg = String()
            msg.data = json.dumps(parsed)
            self._pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'NPK read failed: {e}')

    @staticmethod
    def _crc16(buf: bytes) -> int:
        crc = 0xFFFF
        for b in buf:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    @classmethod
    def _check_crc(cls, data: bytes) -> bool:
        crc = cls._crc16(data[:17])
        recv_crc = data[17] | (data[18] << 8)
        return crc == recv_crc

    @staticmethod
    def _parse(data: bytes) -> dict:
        return {
            'Moist': ((data[3] << 8) | data[4]) / 10.0,
            'Temp': ((data[5] << 8) | data[6]) / 10.0,
            'EC': (data[7] << 8) | data[8],
            'pH': ((data[9] << 8) | data[10]) / 10.0,
            'N': (data[11] << 8) | data[12],
            'P': (data[13] << 8) | data[14],
            'K': (data[15] << 8) | data[16],
        }

    def destroy_node(self) -> bool:
        if self._ser and self._ser.is_open:
            self._ser.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NPKBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
