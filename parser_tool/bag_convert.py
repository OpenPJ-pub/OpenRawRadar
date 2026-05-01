import argparse
import json
import os
import shutil
import struct

import h5py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from data_parser import compute_bytes_per_frame, initialize_first_valid_packet, rosbag_parser
from tools.rd_vis import render_radar_bin_frames_to_video, render_radar_frames_to_video
from utils.parser_common import load_topic_type_map, open_mcap_reader
from utils.radar_config_parser import read_radar_config
from utils.ros_msg_define import BYTES_IN_PACKET, DATA_IN_PACKET, radar_topic


def _load_radar_message_type(bag_path: str):
    reader = open_mcap_reader(bag_path)
    type_map = load_topic_type_map(reader)
    if radar_topic not in type_map:
        raise RuntimeError(f"Radar topic not found in bag: {radar_topic}")

    return get_message(type_map[radar_topic])


def _write_json(path: str, payload: dict) -> None:
    with open(path, 'w', encoding='utf-8') as metadata_file:
        json.dump(payload, metadata_file, indent=2)


def _reset_output_dir(output_dir: str) -> None:
    if os.path.isdir(output_dir):
        for entry in os.listdir(output_dir):
            entry_path = os.path.join(output_dir, entry)
            if os.path.isdir(entry_path):
                shutil.rmtree(entry_path)
            else:
                os.remove(entry_path)
    else:
        os.makedirs(output_dir, exist_ok=True)


def _load_h5_frame_count(h5_path: str) -> int:
    with h5py.File(h5_path, 'r') as h5_file:
        return int(h5_file['radar']['data'].shape[0])


def export_radar_bin(bag_path: str, save_path: str, radar_params: dict, radar_config_path: str) -> dict:
    radar_message_type = _load_radar_message_type(bag_path)
    chunk_count = 0
    packet_count = 0
    total_bytes = 0
    first_ros_time_ns = None
    last_ros_time_ns = None
    first_valid_packet_seq = None
    frame_buffer = bytearray()
    reader = open_mcap_reader(bag_path)
    bytes_per_frame = compute_bytes_per_frame(radar_params)

    with open(save_path, 'wb') as bin_file:
        while reader.has_next():
            topic, data, ros_time_ns = reader.read_next()
            if topic != radar_topic:
                continue

            msg = deserialize_message(data, radar_message_type)
            chunk_count += 1
            if first_ros_time_ns is None:
                first_ros_time_ns = ros_time_ns
            last_ros_time_ns = ros_time_ns

            if len(msg.data) % BYTES_IN_PACKET != 0:
                print(f"Warning: msg.data length {len(msg.data)} not divisible by packet size {BYTES_IN_PACKET}")

            for packet_index in range(len(msg.data) // BYTES_IN_PACKET):
                raw_packet = msg.data[packet_index * BYTES_IN_PACKET:(packet_index + 1) * BYTES_IN_PACKET]
                pkt_seq = struct.unpack('<1L', raw_packet[:4])[0]
                pkt_data = raw_packet[10:]

                if first_valid_packet_seq is None:
                    first_valid_packet_seq, pkt_data = initialize_first_valid_packet(
                        pkt_seq, bytes_per_frame, pkt_data)
                    if first_valid_packet_seq is None:
                        continue

                frame_buffer.extend(pkt_data)
                packet_count += 1

                while len(frame_buffer) >= bytes_per_frame:
                    frame_bytes = bytes(frame_buffer[:bytes_per_frame])
                    bin_file.write(frame_bytes)
                    total_bytes += len(frame_bytes)
                    del frame_buffer[:bytes_per_frame]

    frame_remainder_bytes = len(frame_buffer)
    total_frames_floor = total_bytes // bytes_per_frame
    metadata = {
        "bag_path": bag_path,
        "radar_topic": radar_topic,
        "chunk_count": chunk_count,
        "total_bytes": total_bytes,
        "total_packets": packet_count,
        "bytes_per_frame": bytes_per_frame,
        "estimated_frame_count_floor": total_frames_floor,
        "frame_remainder_bytes": frame_remainder_bytes,
        "discarded_tail_bytes": frame_remainder_bytes,
        "data_size_consistent": total_bytes % bytes_per_frame == 0,
        "radar_config_path": radar_config_path,
        "radar_config": radar_params,
        "first_ros_time_ns": first_ros_time_ns,
        "last_ros_time_ns": last_ros_time_ns,
    }
    metadata_path = os.path.splitext(save_path)[0] + ".json"
    _write_json(metadata_path, metadata)
    return metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert a recorded radar rosbag to h5 or bin.")
    parser.add_argument("--bag-path", required=True, help="Path to rosbag2 folder")
    parser.add_argument("--radar-config-path", required=True, help="Path to radar config txt")
    parser.add_argument("--dca-config-path", required=True, help="Path to DCA config txt")
    parser.add_argument("--output-dir", default=None, help="Output folder, default: <bag>/export")
    parser.add_argument("--format", choices=["h5", "bin", "both"], default="h5",
                        help="Output format")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir or os.path.join(args.bag_path, "export")
    _reset_output_dir(output_dir)
    print(f"[1/5] output directory: {output_dir}")

    print("[2/5] loading radar and DCA configuration ...")
    radar_params, _, _ = read_radar_config(
        radar_config_path=args.radar_config_path,
        dca_config_path=args.dca_config_path,
        display_config=False,
    )

    h5_frame_count = None
    bin_frame_count = None
    metadata = None
    bin_metadata_path = os.path.join(output_dir, "adc_data.json")

    if args.format in ("h5", "both"):
        h5_path = os.path.join(output_dir, "data.h5")
        print(f"[3/5] converting rosbag to h5: {h5_path}")
        rosbag_parser(args.bag_path, h5_path, radar_params)
        h5_frame_count = _load_h5_frame_count(h5_path)
        print(f"      h5 stored frames: {h5_frame_count}")
        print("[4/5] rendering RD map video from h5 ...")
        _, h5_video_path, rendered_h5_frame_count = render_radar_frames_to_video(
            h5_path, args.radar_config_path, output_dir, "h5")
        print(f"      h5 rendered RD map frames: {rendered_h5_frame_count}, video: {h5_video_path}")

    if args.format in ("bin", "both"):
        bin_path = os.path.join(output_dir, "adc_data.bin")
        print(f"[3/5] exporting raw bin: {bin_path}")
        metadata = export_radar_bin(args.bag_path, bin_path, radar_params, args.radar_config_path)
        print("[4/5] rendering RD map video from bin ...")
        _, bin_video_path, bin_frame_count = render_radar_bin_frames_to_video(
            bin_path, args.radar_config_path, output_dir, "bin")
        print(f"      bin RD map frames: {bin_frame_count}, video: {bin_video_path}")
        print(
            f"      total_bytes={metadata['total_bytes']}, bytes_per_frame={metadata['bytes_per_frame']}, "
            f"data_size_consistent={metadata['data_size_consistent']}, "
            f"discarded_tail_bytes={metadata['discarded_tail_bytes']}")

    if metadata is not None:
        metadata["rd_channel"] = "tx0_rx0"
        metadata["rd_frame_count_from_bin"] = bin_frame_count
        metadata["rd_frame_count_from_h5"] = h5_frame_count
        metadata["rd_frame_count_consistent"] = (
            h5_frame_count is None or bin_frame_count is None or h5_frame_count == bin_frame_count
        )
        _write_json(bin_metadata_path, metadata)
        print(
            f"      rd_frame_count_from_bin={metadata['rd_frame_count_from_bin']}, "
            f"rd_frame_count_from_h5={metadata['rd_frame_count_from_h5']}, "
            f"rd_frame_count_consistent={metadata['rd_frame_count_consistent']}")

    print("[5/5] export completed")


if __name__ == "__main__":
    main()
