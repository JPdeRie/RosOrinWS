#!/usr/bin/env python3

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import serial


class LD19Node(Node):
    HEADER = 0x54
    VER_LEN = 0x2C
    PACKET_SIZE = 47
    POINTS_PER_PACKET = 12

    def __init__(self):
        super().__init__("ld19_lidar")

        self.declare_parameter("port", "/dev/lidar")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("topic", "/scan")
        self.declare_parameter("angle_increment_deg", 0.5)
        self.declare_parameter("range_min", 0.02)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("reverse_scan", False)
        self.declare_parameter("angle_offset_deg", 0.0)

        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        topic = str(self.get_parameter("topic").value)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.angle_increment_deg = float(
            self.get_parameter("angle_increment_deg").value
        )
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.reverse_scan = bool(
            self.get_parameter("reverse_scan").value
        )
        self.angle_offset_deg = float(
            self.get_parameter("angle_offset_deg").value
        )

        if self.angle_increment_deg <= 0.0:
            raise ValueError("angle_increment_deg must be greater than zero")

        self.bin_count = int(
            round(360.0 / self.angle_increment_deg)
        )
        self.angle_increment = (
            2.0 * math.pi / self.bin_count
        )

        self.ranges = [math.inf] * self.bin_count
        self.intensities = [0.0] * self.bin_count

        self.previous_angle = None
        self.scan_start_time = time.monotonic()
        self.last_scan_time = 0.1
        self.packet_buffer = bytearray()

        self.publisher = self.create_publisher(
            LaserScan,
            topic,
            qos_profile_sensor_data,
        )

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
            )
        except serial.SerialException as error:
            self.get_logger().fatal(
                f"Could not open LD19 on {port}: {error}"
            )
            raise

        self.timer = self.create_timer(
            0.002,
            self.read_serial,
        )

        self.get_logger().info(
            f"LD19 started: port={port}, "
            f"baud={baudrate}, topic={topic}"
        )

    @staticmethod
    def crc8(data: bytes) -> int:
        """
        Calculate the LD19 CRC-8.

        Polynomial: 0x4D
        Initial value: 0x00
        Final XOR: 0x00
        """
        crc = 0

        for byte in data:
            crc ^= byte

            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x4D) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF

        return crc

    def read_serial(self):
        try:
            waiting = self.serial_port.in_waiting
            data = self.serial_port.read(max(waiting, 1))
        except serial.SerialException as error:
            self.get_logger().error(
                f"Serial read error: {error}"
            )
            return

        if not data:
            return

        self.packet_buffer.extend(data)

        while len(self.packet_buffer) >= 2:
            header_index = self.packet_buffer.find(
                bytes([self.HEADER, self.VER_LEN])
            )

            if header_index < 0:
                # Preserve a possible first header byte at the end.
                if self.packet_buffer[-1] == self.HEADER:
                    self.packet_buffer[:] = bytes([self.HEADER])
                else:
                    self.packet_buffer.clear()
                return

            if header_index > 0:
                del self.packet_buffer[:header_index]

            if len(self.packet_buffer) < self.PACKET_SIZE:
                return

            packet = bytes(
                self.packet_buffer[:self.PACKET_SIZE]
            )

            expected_crc = packet[-1]
            calculated_crc = self.crc8(packet[:-1])

            if calculated_crc != expected_crc:
                del self.packet_buffer[0]
                continue

            del self.packet_buffer[:self.PACKET_SIZE]
            self.process_packet(packet)

    def process_packet(self, packet: bytes):
        rotation_speed = struct.unpack_from(
            "<H",
            packet,
            2,
        )[0]

        start_angle = (
            struct.unpack_from("<H", packet, 4)[0] / 100.0
        )

        end_angle = (
            struct.unpack_from("<H", packet, 42)[0] / 100.0
        )

        if end_angle < start_angle:
            end_angle += 360.0

        angle_step = (
            (end_angle - start_angle)
            / (self.POINTS_PER_PACKET - 1)
        )

        for point_index in range(self.POINTS_PER_PACKET):
            data_offset = 6 + point_index * 3

            distance_mm = struct.unpack_from(
                "<H",
                packet,
                data_offset,
            )[0]

            confidence = packet[data_offset + 2]

            raw_angle = (
                start_angle + point_index * angle_step
            ) % 360.0

            # A transition from a high angle to a low angle means
            # the LiDAR completed one revolution.
            if (
                self.previous_angle is not None
                and self.previous_angle > 300.0
                and raw_angle < 60.0
            ):
                self.publish_scan(rotation_speed)

            self.previous_angle = raw_angle

            output_angle = raw_angle

            if self.reverse_scan:
                output_angle = 360.0 - output_angle

            output_angle = (
                output_angle + self.angle_offset_deg
            ) % 360.0

            bin_index = int(
                round(
                    output_angle
                    / self.angle_increment_deg
                )
            ) % self.bin_count

            distance_m = distance_mm / 1000.0

            if (
                distance_mm == 0
                or confidence == 0
                or distance_m < self.range_min
                or distance_m > self.range_max
            ):
                continue

            # Keep the closest valid measurement if multiple
            # measurements occupy the same angular bin.
            if distance_m < self.ranges[bin_index]:
                self.ranges[bin_index] = distance_m
                self.intensities[bin_index] = float(confidence)

    def publish_scan(self, rotation_speed: int):
        if not any(
            math.isfinite(value) for value in self.ranges
        ):
            self.clear_scan()
            self.scan_start_time = time.monotonic()
            return

        current_time = time.monotonic()
        measured_scan_time = (
            current_time - self.scan_start_time
        )
        self.scan_start_time = current_time

        if 0.02 < measured_scan_time < 1.0:
            self.last_scan_time = measured_scan_time
        elif rotation_speed > 0:
            self.last_scan_time = 360.0 / rotation_speed

        message = LaserScan()

        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.header.frame_id = self.frame_id

        message.angle_min = 0.0
        message.angle_max = (
            2.0 * math.pi - self.angle_increment
        )
        message.angle_increment = self.angle_increment

        message.scan_time = self.last_scan_time
        message.time_increment = (
            self.last_scan_time / self.bin_count
        )

        message.range_min = self.range_min
        message.range_max = self.range_max

        message.ranges = self.ranges.copy()
        message.intensities = self.intensities.copy()

        self.publisher.publish(message)
        self.clear_scan()

    def clear_scan(self):
        self.ranges = [math.inf] * self.bin_count
        self.intensities = [0.0] * self.bin_count

    def destroy_node(self):
        if (
            hasattr(self, "serial_port")
            and self.serial_port.is_open
        ):
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = LD19Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"LD19 driver failed: {error}")
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()