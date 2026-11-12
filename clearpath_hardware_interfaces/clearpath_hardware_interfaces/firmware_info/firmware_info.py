#!/usr/bin/env python3

import hashlib
from pathlib import Path
import socket
import struct
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Constants and structures for firmware-info protocol
MAGIC = b'CPFI'
WIRE_VERSION = 2
ADVERTISEMENT = 1
ACKNOWLEDGMENT = 2
PROTOCOL_PROTON = 1
PROTOCOL_NAMES = {PROTOCOL_PROTON: 'Proton'}
PLATFORM_NAMES = {
    0: 'Warthog/w200',
    1: 'Dingo/d1x0',
    2: 'Ridgeback/r100',
    3: 'Core/a300',
    4: 'Jackal/j100',
}
HEADER = struct.Struct('!4sBBHI')
TLV_HEADER = struct.Struct('!BB')
MAX_PACKET_SIZE = 256
MAX_PACKETS_PER_CALLBACK = 16
FIRMWARE_INFO_PORT = 11417

# TLV (Type-Length-Value) field types for firmware-info packets
TLV_PROTOCOL_ID = 1
TLV_PROTOCOL_MAJOR = 2
TLV_PROTOCOL_MINOR = 3
TLV_PROTOCOL_PATCH = 4
TLV_PLATFORM_ID = 5
TLV_FIRMWARE_VERSION = 6
TLV_HARDWARE_ID = 7
TLV_SCHEMA_SHA256 = 8
TLV_CAPABILITIES = 9
TLV_SOURCE_COMMIT = 10
TLV_UDP4_VERSION = 11


def encode_packet(message_type, nonce, fields):
    payload = bytearray()
    for field_type, value in fields.items():
        value = bytes(value)
        if len(value) > 255:
            raise ValueError(f'firmware-info field {field_type} is too long')
        payload.extend(TLV_HEADER.pack(field_type, len(value)))
        payload.extend(value)
    if HEADER.size + len(payload) > MAX_PACKET_SIZE:
        raise ValueError('firmware-info packet exceeds maximum size')
    return HEADER.pack(MAGIC, WIRE_VERSION, message_type, len(payload), nonce) + payload


def decode_packet(packet):
    if len(packet) < HEADER.size or len(packet) > MAX_PACKET_SIZE:
        raise ValueError('invalid firmware-info packet size')
    magic, wire_version, message_type, payload_length, nonce = HEADER.unpack_from(
        packet
    )
    if magic != MAGIC or wire_version != WIRE_VERSION:
        raise ValueError('invalid firmware-info header')
    if payload_length != len(packet) - HEADER.size:
        raise ValueError('firmware-info payload length mismatch')

    fields = {}
    offset = HEADER.size
    while offset < len(packet):
        if len(packet) - offset < TLV_HEADER.size:
            raise ValueError('truncated firmware-info TLV header')
        field_type, field_length = TLV_HEADER.unpack_from(packet, offset)
        offset += TLV_HEADER.size
        end = offset + field_length
        if end > len(packet):
            raise ValueError('truncated firmware-info TLV value')
        if field_type in fields:
            raise ValueError(f'duplicate firmware-info field {field_type}')
        fields[field_type] = packet[offset:end]
        offset = end
    return message_type, nonce, fields


def field_u8(fields, field_type):
    value = fields.get(field_type)
    if value is None or len(value) != 1:
        raise ValueError(f'missing or malformed firmware-info field {field_type}')
    return value[0]


def field_u32(fields, field_type):
    value = fields.get(field_type)
    if value is None or len(value) != 4:
        raise ValueError(f'missing or malformed firmware-info field {field_type}')
    return struct.unpack('!I', value)[0]


def field_text(fields, field_type):
    value = fields.get(field_type, b'')
    return value.decode('utf-8', errors='replace')


class FirmwareInfoNode(Node):
    def __init__(self):
        super().__init__('firmware_info')
        diagnostics_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticStatus, 'platform/firmware_info', diagnostics_qos
        )
        self.declare_parameter('expected_proton_major', 1)
        self.declare_parameter('expected_proton_minor', 0)
        self.declare_parameter('expected_proton_patch', 1)
        self.declare_parameter('expected_platform_id', -1)
        self.declare_parameter('firmware_info_timeout_sec', 3.0)
        self.declare_parameter('robot_ip', '192.168.131.1')
        self.declare_parameter('firmware_ip', '192.168.131.2')
        self.declare_parameter('firmware_info_port', FIRMWARE_INFO_PORT)

        self.expected_major = self.get_parameter('expected_proton_major').value
        self.expected_minor = self.get_parameter('expected_proton_minor').value
        self.expected_patch = self.get_parameter('expected_proton_patch').value
        self.expected_platform_id = self.get_parameter('expected_platform_id').value
        self.timeout_sec = self.get_parameter('firmware_info_timeout_sec').value
        self.firmware_ip = self.get_parameter('firmware_ip').value
        self.firmware_info_port = self.get_parameter('firmware_info_port').value
        if not (0 <= self.expected_major <= 255 and
                0 <= self.expected_minor <= 255 and
                0 <= self.expected_patch <= 255):
            raise ValueError('expected Proton version components must be in [0, 255]')
        if self.expected_platform_id not in (-1, 0, 1, 2, 3, 4):
            raise ValueError('expected_platform_id must be -1 or a known platform ID')
        self.expected_schema_sha256 = None
        schema_names = {0: 'w200', 1: 'd1x0', 2: 'r100', 3: 'core', 4: 'j100'}
        if self.expected_platform_id >= 0:
            schema_path = (
                Path(get_package_share_directory('clearpath_firmware')) /
                'proton' / f'{schema_names[self.expected_platform_id]}.yaml'
            )
            self.expected_schema_sha256 = hashlib.sha256(
                schema_path.read_bytes()
            ).hexdigest()
        if self.timeout_sec <= 0:
            raise ValueError('firmware_info_timeout_sec must be positive')

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(
            (self.get_parameter('robot_ip').value, self.firmware_info_port)
        )
        self.socket.setblocking(False)
        self.started_at = time.monotonic()
        self.finished = False
        self.timer = self.create_timer(0.1, self._check_firmware_info)

    def _check_firmware_info(self):
        if self.finished:
            return
        for _packet_index in range(MAX_PACKETS_PER_CALLBACK):
            try:
                payload, source = self.socket.recvfrom(MAX_PACKET_SIZE + 1)
            except BlockingIOError:
                break

            if source[0] != self.firmware_ip:
                continue

            try:
                message_type, nonce, fields = decode_packet(payload)
                protocol_id = field_u8(fields, TLV_PROTOCOL_ID)
                major = field_u8(fields, TLV_PROTOCOL_MAJOR)
                minor = field_u8(fields, TLV_PROTOCOL_MINOR)
                patch = field_u8(fields, TLV_PROTOCOL_PATCH)
                metadata = {
                    'platform_id': bytes([field_u8(fields, TLV_PLATFORM_ID)]),
                    'firmware_version': field_text(fields, TLV_FIRMWARE_VERSION),
                    'hardware_id': field_text(fields, TLV_HARDWARE_ID),
                    'schema_sha256': field_text(fields, TLV_SCHEMA_SHA256),
                    'capabilities': field_u32(fields, TLV_CAPABILITIES),
                    'source_commit': field_text(fields, TLV_SOURCE_COMMIT),
                    'udp4_version': field_u8(fields, TLV_UDP4_VERSION),
                }
            except ValueError:
                continue
            if message_type != ADVERTISEMENT:
                continue

            self.get_logger().info(
                f'Firmware info advertisement received from {source[0]}:{source[1]}; '
                f'wire_version={WIRE_VERSION}, session_nonce={nonce}, '
                f'protocol_id={protocol_id} [{self._protocol_name(protocol_id)}], '
                f'protocol_version={self._format_version((major, minor, patch))}; '
                f'{self._metadata_summary(metadata)}'
            )

            acknowledgment = encode_packet(
                ACKNOWLEDGMENT,
                nonce,
                {
                    TLV_PROTOCOL_ID: bytes([protocol_id]),
                    TLV_PROTOCOL_MAJOR: bytes([major]),
                    TLV_PROTOCOL_MINOR: bytes([minor]),
                    TLV_PROTOCOL_PATCH: bytes([patch]),
                },
            )
            try:
                self.socket.sendto(acknowledgment, source)
            except OSError as error:
                self.get_logger().error(f'Failed to acknowledge firmware info: {error}')
                return

            if protocol_id != PROTOCOL_PROTON:
                self._finish_incompatible(
                    f'Firmware info reported unsupported protocol ID '
                    f'{protocol_id} [{self._protocol_name(protocol_id)}]; '
                    f'{self._metadata_summary(metadata)}'
                )
            else:
                self._finish_proton_check((major, minor, patch), metadata)
            return

        if time.monotonic() - self.started_at >= self.timeout_sec:
            message = 'No firmware info advertisement received; assuming Proton 1'
            self.finished = True
            self.get_logger().warning(message)
            self._publish_diagnostic(
                DiagnosticStatus.WARN, message, detected_version=(1, 0, 0)
            )

    def _finish_proton_check(self, detected_version, metadata=None):
        if metadata is not None and self.expected_platform_id >= 0:
            platform_id = metadata['platform_id']
            if len(platform_id) != 1 or platform_id[0] != self.expected_platform_id:
                detected_platform = (
                    platform_id[0] if len(platform_id) == 1 else 'missing'
                )
                self._finish_incompatible(
                    f'Firmware platform mismatch: robot expects platform ID '
                    f'{self.expected_platform_id}, firmware reports {detected_platform}',
                    detected_version,
                    metadata,
                )
                return

        if (metadata is not None and self.expected_schema_sha256 is not None and
                metadata['schema_sha256'].lower() != self.expected_schema_sha256):
            self._finish_incompatible(
                f"Proton schema mismatch: robot has {self.expected_schema_sha256}, "
                f"firmware has {metadata['schema_sha256']}",
                detected_version,
                metadata,
            )
            return

        expected_version = (
            self.expected_major,
            self.expected_minor,
            self.expected_patch,
        )
        if detected_version[0] != self.expected_major:
            self._finish_incompatible(
                f'Proton protocol incompatibility: robot expects Proton '
                f'{self._format_version(expected_version)}, firmware uses Proton '
                f'{self._format_version(detected_version)}; '
                f'{self._metadata_summary(metadata)}',
                detected_version,
                metadata,
            )
            return

        if detected_version != expected_version:
            message = (
                'Proton minor or patch version mismatch: robot expects Proton '
                f'{self._format_version(expected_version)}, firmware uses Proton '
                f'{self._format_version(detected_version)}; major version matches; '
                f'{self._metadata_summary(metadata)}'
            )
            self.get_logger().warning(
                message
            )
            level = DiagnosticStatus.WARN
        else:
            message = (
                'Proton protocol matches: firmware uses Proton '
                f'{self._format_version(detected_version)}; '
                f'{self._metadata_summary(metadata)}'
            )
            self.get_logger().info(
                message
            )
            level = DiagnosticStatus.OK

        self.finished = True
        self._publish_diagnostic(level, message, detected_version, metadata)

    def _publish_diagnostic(self, level, message, detected_version=None, metadata=None):
        status = DiagnosticStatus()
        status.level = level
        status.name = 'Proton Firmware Info'
        status.message = message
        status.hardware_id = (
            metadata.get('hardware_id') or self.firmware_ip
            if metadata is not None else self.firmware_ip
        )
        expected_version = (
            self.expected_major,
            self.expected_minor,
            self.expected_patch,
        )
        values = {
            'expected_proton_version': self._format_version(expected_version),
            'detected_proton_version': (
                self._format_version(detected_version)
                if detected_version is not None else 'unknown'
            ),
        }
        if metadata is not None:
            for key, value in metadata.items():
                if key == 'platform_id':
                    value = str(value[0]) if len(value) == 1 else 'unknown'
                elif isinstance(value, bytes):
                    value = value.decode('utf-8', errors='replace')
                values[key] = str(value)
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()
        ]

        self.diagnostics_publisher.publish(status)

    @staticmethod
    def _format_version(version):
        return '.'.join(str(part) for part in version)

    @staticmethod
    def _protocol_name(protocol_id):
        return PROTOCOL_NAMES.get(protocol_id, 'Unknown')

    @staticmethod
    def _metadata_summary(metadata):
        if metadata is None:
            return 'firmware metadata unavailable'
        platform_id = metadata['platform_id']
        platform_name = (
            PLATFORM_NAMES.get(platform_id[0], 'Unknown')
            if len(platform_id) == 1 else 'Unknown'
        )
        return (
            f"platform_id={platform_id[0] if len(platform_id) == 1 else 'unknown'} "
            f'[{platform_name}], '
            f"firmware={metadata['firmware_version'] or 'unknown'}, "
            f"hardware_id={metadata['hardware_id'] or 'unknown'}, "
            f"schema_sha256={metadata['schema_sha256'] or 'unknown'}, "
            f"source_commit={metadata['source_commit'] or 'unknown'}, "
            f"capabilities=0x{metadata['capabilities']:08x}, "
            f"udp4_version={metadata['udp4_version']}"
        )

    def _finish_incompatible(self, message, detected_version=None, metadata=None):
        self.finished = True
        self.get_logger().error(message)
        self._publish_diagnostic(
            DiagnosticStatus.ERROR, message, detected_version, metadata
        )

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FirmwareInfoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
