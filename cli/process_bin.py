#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT_ROOT = ROOT_DIR / "output"
DEFAULT_RADAR_CONFIG_FILENAME = "AWR2243_mmwaveconfig_max15.txt"

sys.path.insert(0, str(ROOT_DIR / "parser_tool"))

from tools.rd_vis import render_radar_bin_frames_to_video  # noqa: E402


def find_latest_cli_output(output_root: Path) -> Path | None:
    candidates = [path for path in output_root.glob("cli_*") if path.is_dir()]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render RD map video from CLI adc_data.bin output.")
    parser.add_argument(
        "--cli-output-path",
        type=Path,
        default=None,
        help="Path to cli_* folder. Default: latest under ./output",
    )
    parser.add_argument(
        "--radar-config-filename",
        default=DEFAULT_RADAR_CONFIG_FILENAME,
        help="Radar config file name under ./configs",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory. Default: same as cli output directory",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    cli_output_path = args.cli_output_path or find_latest_cli_output(DEFAULT_OUTPUT_ROOT)
    if cli_output_path is None:
        print(f"No cli_* folder found under {DEFAULT_OUTPUT_ROOT}", file=sys.stderr)
        return 1
    if not cli_output_path.is_dir():
        print(f"CLI output path does not exist: {cli_output_path}", file=sys.stderr)
        return 1

    output_dir = args.output_dir or cli_output_path
    bin_path = cli_output_path / "adc_data.bin"
    if not bin_path.is_file():
        print(f"adc_data.bin not found: {bin_path}", file=sys.stderr)
        return 1

    radar_config_path = ROOT_DIR / "configs" / args.radar_config_filename
    if not radar_config_path.is_file():
        print(f"Radar config not found: {radar_config_path}", file=sys.stderr)
        return 1

    print(f"[1/3] cli output directory: {cli_output_path}")
    print("[2/3] rendering RD map video from bin ...")

    frames_dir, video_path, frame_count = render_radar_bin_frames_to_video(
        str(bin_path),
        str(radar_config_path),
        str(output_dir),
        "bin",
    )

    print(f"      bin RD map frames: {frame_count}, frames_dir: {frames_dir}")
    print(f"      video: {video_path}")
    print("[3/3] export completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
