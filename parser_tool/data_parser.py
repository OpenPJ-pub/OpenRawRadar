import re
import struct
from collections import namedtuple

import h5py
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tqdm import tqdm

from utils.radar_frame_utils import compute_bytes_per_frame, create_radar_frame_data
from utils.parser_common import load_total_message_count, load_topic_type_map, open_mcap_reader
from utils.ros_msg_define import BYTES_IN_PACKET, DATA_IN_PACKET, Timestamp, topic_map


SensorFrame = namedtuple('SensorFrame', ['sensor_type', 'utc_sec', 'utc_nanosec', 'index', 'data'])

RESIZE_STEP_SMALL = 1000
RESIZE_STEP_DATA = 1
RADAR_SENSOR_TYPE = 'radar'
BAG_NAME_PATTERN = re.compile(r'rosbag2_(\d{4})_(\d{2})_(\d{2})-(\d{2})_(\d{2})_(\d{2})')


def timestamp_to_sec(ts: Timestamp):
    if ts.utc_sec == 0 or ts.tick_per_sec == 0 or ts.subtick_per_tick == 0:
        return None, None
    nanosec = int(1e9 / float(ts.tick_per_sec) * (ts.tick + float(ts.subtick) / float(ts.subtick_per_tick)))
    utc_sec = ts.utc_sec + int(nanosec // 1e9)
    nanosec = int(nanosec % 1e9)
    return utc_sec, nanosec


def build_sensor_indices():
    return {sensor: 0 for sensor in topic_map.values()}


def ensure_bag_name_matches_expected_format(bag_path: str) -> None:
    match = BAG_NAME_PATTERN.search(bag_path)
    if not match:
        raise ValueError(f"no utc time in the filename: {bag_path}")


def build_frame_buffer_entry(utc_sec, utc_nanosec):
    return {
        'utc_sec': utc_sec,
        'utc_nanosec': utc_nanosec,
        'packets': [],
        'received_pkt_indices': []
    }


def append_packet_to_frame_buffer(frame_buffer_entry: dict, pkt_data: bytes, pkt_index: int):
    frame_buffer_entry['packets'].append(pkt_data)
    frame_buffer_entry['received_pkt_indices'].append(pkt_index)


def consume_frame_bytes(frame_buffer_entry: dict) -> bytes:
    return b''.join(frame_buffer_entry['packets'])


def extract_radar_timestamp(msg):
    return timestamp_to_sec(
        Timestamp(msg.ts.pwr_sec,
                  msg.ts.utc_sec,
                  msg.ts.tick,
                  msg.ts.tick_per_sec,
                  msg.ts.subtick,
                  msg.ts.subtick_per_tick,
                  msg.ts.clk,
                  msg.ts.clk_per_sec))


def initialize_first_valid_packet(pkt_seq: int, byte_per_frame: int, pkt_data: bytes):
    total_bytes = pkt_seq * DATA_IN_PACKET
    invalid_bytes = (total_bytes // byte_per_frame) * byte_per_frame
    left_bytes = total_bytes - invalid_bytes
    if 0 < left_bytes <= DATA_IN_PACKET:
        return pkt_seq - 1, pkt_data[-left_bytes:]
    return None, None


def ensure_dataset(sensor_group, field: str, value: np.ndarray):
    if field in sensor_group:
        return sensor_group[field]

    maxshape = (None,) + value.shape if field == 'data' else (None,)
    if field == 'data':
        initial_shape = (RESIZE_STEP_DATA,) + value.shape
        chunks = (RESIZE_STEP_DATA,) + value.shape
    else:
        initial_shape = (RESIZE_STEP_SMALL,)
        chunks = (RESIZE_STEP_SMALL,)

    sensor_group.create_dataset(
        field,
        shape=initial_shape,
        maxshape=maxshape,
        dtype=value.dtype,
        chunks=chunks,
        compression='gzip')
    return sensor_group[field]


def write_sensor_frame(h5_writer, frame: SensorFrame, current_idx: int):
    sensor_group = h5_writer.require_group(frame.sensor_type)

    for field in ['utc_sec', 'utc_nanosec', 'index', 'data']:
        value = np.asarray(getattr(frame, field))
        dataset = ensure_dataset(sensor_group, field, value)
        step = RESIZE_STEP_DATA if field == 'data' else RESIZE_STEP_SMALL
        if current_idx >= dataset.shape[0]:
            dataset.resize(dataset.shape[0] + step, axis=0)
        dataset[current_idx] = value


def finalize_datasets(h5_writer, sensor_indices):
    for sensor_type, sensor_group in h5_writer.items():
        actual_size = sensor_indices[sensor_type]
        for _, dataset in sensor_group.items():
            dataset.resize(actual_size, axis=0)


def rosbag_parser(bag_path: str, save_file: str, radar_param: dict):
    ensure_bag_name_matches_expected_format(bag_path)

    sensor_indices = build_sensor_indices()
    reader = open_mcap_reader(bag_path)
    type_map = load_topic_type_map(reader)
    total_messages = load_total_message_count(bag_path)

    h5_writer = h5py.File(save_file, 'w')

    radar_frame_buffer = {}
    radar_buffered_bytes = 0
    byte_per_frame = compute_bytes_per_frame(radar_param)

    utc_sec = None
    nanosec = None
    radar_first_valid_packet_seq = None
    radar_frame_idx = 0

    with tqdm(total=total_messages, desc="Reading rosbag2 messages") as progress_bar:
        while reader.has_next():
            try:
                topic, data, ros_time_ns = reader.read_next()
                progress_bar.update(1)
                if topic not in topic_map:
                    continue

                sensor_data = None
                sensor_type = topic_map[topic]
                msg = deserialize_message(data, get_message(type_map[topic]))

                if sensor_type == RADAR_SENSOR_TYPE:
                    utc_sec, nanosec = extract_radar_timestamp(msg)

                    if len(msg.data) % BYTES_IN_PACKET != 0:
                        print(f"Warning: msg.data length {len(msg.data)} not divisible by packet size {BYTES_IN_PACKET}")

                    for packet_index in range(len(msg.data) // BYTES_IN_PACKET):
                        raw_packet = msg.data[packet_index * BYTES_IN_PACKET:(packet_index + 1) * BYTES_IN_PACKET]
                        pkt_seq = struct.unpack('<1L', raw_packet[:4])[0]
                        pkt_data = raw_packet[10:]

                        if radar_first_valid_packet_seq is None:
                            radar_first_valid_packet_seq, pkt_data = initialize_first_valid_packet(
                                pkt_seq, byte_per_frame, pkt_data)
                            if radar_first_valid_packet_seq is None:
                                continue

                        adjusted_pkt_seq = pkt_seq - radar_first_valid_packet_seq

                        if radar_frame_idx not in radar_frame_buffer:
                            radar_frame_buffer[radar_frame_idx] = build_frame_buffer_entry(utc_sec, nanosec)

                        append_packet_to_frame_buffer(
                            radar_frame_buffer[radar_frame_idx],
                            pkt_data,
                            adjusted_pkt_seq)
                        radar_buffered_bytes += len(pkt_data)

                        if radar_buffered_bytes >= byte_per_frame:
                            frame_bytes = consume_frame_bytes(radar_frame_buffer[radar_frame_idx])
                            radar_frame_idx += 1

                            if len(frame_bytes) > byte_per_frame:
                                current_frame_bytes = frame_bytes[:byte_per_frame]
                                extra_bytes = frame_bytes[byte_per_frame:]
                                radar_frame_buffer[radar_frame_idx] = build_frame_buffer_entry(utc_sec, nanosec)
                                radar_buffered_bytes = len(extra_bytes)
                                append_packet_to_frame_buffer(
                                    radar_frame_buffer[radar_frame_idx],
                                    extra_bytes,
                                    adjusted_pkt_seq)
                            else:
                                current_frame_bytes = frame_bytes
                                radar_buffered_bytes = 0

                            sensor_data = create_radar_frame_data(current_frame_bytes, radar_param)
                            if sensor_data is None:
                                print(
                                    f"Incomplete frame detected at frame {sensor_indices[sensor_type]}: "
                                    f"{len(current_frame_bytes) // 2} != {byte_per_frame // 2}")
                                del radar_frame_buffer[radar_frame_idx - 1]
                                continue

                            del radar_frame_buffer[radar_frame_idx - 1]

                if utc_sec is None:
                    utc_sec = ros_time_ns // 1e9
                    nanosec = ros_time_ns % 1e9

                if sensor_data is None:
                    continue

                frame = SensorFrame(sensor_type, utc_sec, nanosec, sensor_indices[sensor_type], sensor_data)
                write_sensor_frame(h5_writer, frame, sensor_indices[sensor_type])
                sensor_indices[sensor_type] += 1
            except Exception as error:
                h5_writer.close()
                progress_bar.close()
                raise RuntimeError(f"read bag error: {error}")

    finalize_datasets(h5_writer, sensor_indices)
    h5_writer.close()
    print(f"Data saved to {save_file}")
