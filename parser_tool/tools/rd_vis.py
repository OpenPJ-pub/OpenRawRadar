import glob
import os
from dataclasses import dataclass, replace

import cv2
import h5py
import matplotlib
matplotlib.use('Agg')
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm

from utils.radar_config_parser import get_radar_config_params
from utils.radar_frame_utils import compute_bytes_per_frame, create_radar_frame_data

LIGHT_SPEED_MPS = 3.0e8
RD_FLOOR_EPS = 1e-6
DEFAULT_COLOR_MAP = "jet"
DEFAULT_RD_CHANNEL_NAME = "tx0_rx0"


@dataclass(frozen=True)
class RdRenderSettings:
    tx_index: int = 0
    rx_index: int = 0
    channel_name: str = DEFAULT_RD_CHANNEL_NAME
    image_width: int = 1280
    image_height: int = 720
    dpi: int = 100
    fps: float = 10.0
    color_map: str = DEFAULT_COLOR_MAP
    lower_quantile: float = 0.55
    upper_quantile: float = 0.995


@dataclass(frozen=True)
class RdAxisExtent:
    doppler_min: float
    doppler_max: float
    range_min: float
    range_max: float


DEFAULT_RENDER_SETTINGS = RdRenderSettings()


def load_h5_to_mem(h5_path: str, sensor_type: str, utc_sec_start: int | float = None, utc_sec_end: int | float = None):
    data = {}
    with h5py.File(h5_path, 'r') as h5_file:
        radar_group = h5_file[sensor_type]

        utc_sec = np.array(radar_group['utc_sec'])

        if utc_sec_start is not None:
            condition_start = utc_sec >= utc_sec_start
        else:
            condition_start = np.ones_like(utc_sec, dtype=bool)

        if utc_sec_end is not None:
            condition_end = utc_sec <= utc_sec_end
        else:
            condition_end = np.ones_like(utc_sec, dtype=bool)

        condition = condition_start & condition_end
        indices = np.where(condition)[0]

        if len(indices) == 0:
            print("No data matches the given utc_sec interval.")
            return {}

        data['data'] = np.array(radar_group['data'])[indices]
        data['index'] = np.array(radar_group['index'])[indices]
        data['utc_sec'] = utc_sec[indices]
        data['utc_nanosec'] = np.array(radar_group['utc_nanosec'])[indices]

        print(f"Extracted {len(data['index'])} frames, index from {data['index'][0]} to {data['index'][-1]}")

    return data


def build_rd_windows(adc_params: dict) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.kaiser(adc_params['samples'], 4),
        np.kaiser(adc_params['chirps'], 2),
    )


def derive_render_settings(adc_params: dict,
                           settings: RdRenderSettings = DEFAULT_RENDER_SETTINGS) -> RdRenderSettings:
    frame_periodicity_ms = float(adc_params.get('frame_periodicity', 0.0))
    if frame_periodicity_ms <= 0.0:
        return settings

    fps = 1000.0 / frame_periodicity_ms
    if fps <= 0.0:
        return settings

    return replace(settings, fps=fps)


def compute_rd_axis_extent(adc_params: dict) -> RdAxisExtent:
    adc_sample_time = 1e3 * adc_params['samples'] / adc_params['sample_rate']
    mid_freq_hz = (
        adc_params['startFreq'] * 1e9 +
        (adc_params['adc_valid_start_time'] + adc_sample_time / 2.0) * adc_params['freq_slope'] * 1e6
    )
    chirp_repetition_period_us = adc_params['tx'] * (adc_params['idleTime'] + adc_params['rampEndTime'])
    wavelength_m = LIGHT_SPEED_MPS / mid_freq_hz
    max_velocity = wavelength_m / (4.0 * chirp_repetition_period_us * 1e-6)
    max_range = (300.0 * 0.8 * adc_params['sample_rate']) / (2.0 * adc_params['freq_slope'] * 1e3)
    return RdAxisExtent(
        doppler_min=-max_velocity,
        doppler_max=max_velocity,
        range_min=0.0,
        range_max=max_range,
    )


def compute_rd_map_db(adc_frame: np.ndarray,
                      adc_params: dict,
                      settings: RdRenderSettings) -> np.ndarray:
    window_r, window_d = build_rd_windows(adc_params)
    adc_data = adc_frame[settings.tx_index, settings.rx_index, 0]
    range_fft = np.fft.fft(adc_data * window_r[..., None], axis=-2)
    range_doppler = np.fft.fftshift(np.fft.fft(range_fft * window_d, axis=-1), axes=-1)
    rd = np.abs(range_doppler)
    return 20.0 * np.log10(np.maximum(rd, RD_FLOOR_EPS))


def build_rd_map_image(adc_frame: np.ndarray,
                       adc_params: dict,
                       settings: RdRenderSettings = DEFAULT_RENDER_SETTINGS):
    rd_map = compute_rd_map_db(adc_frame, adc_params, settings)
    axis_extent = compute_rd_axis_extent(adc_params)

    color_min = np.quantile(rd_map, settings.lower_quantile)
    color_max = np.quantile(rd_map, settings.upper_quantile)
    if color_max <= color_min:
        color_max = color_min + 1.0

    fig = plt.Figure(
        figsize=(settings.image_width / settings.dpi, settings.image_height / settings.dpi),
        dpi=settings.dpi)
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_axes([0.10, 0.10, 0.84, 0.82])
    ax.imshow(
        rd_map,
        cmap=settings.color_map,
        vmin=color_min,
        vmax=color_max,
        aspect='auto',
        origin='lower',
        extent=[
            axis_extent.doppler_min,
            axis_extent.doppler_max,
            axis_extent.range_min,
            axis_extent.range_max,
        ],
    )
    ax.set_xlabel('Doppler (m/s)')
    ax.set_ylabel('Range (m)')
    ax.tick_params(width=0.8, labelsize=10)

    canvas.draw()
    width, height = canvas.get_width_height()
    image = np.frombuffer(canvas.buffer_rgba(), dtype=np.uint8).reshape(height, width, 4)
    plt.close(fig)
    return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)


def iter_h5_radar_frames(h5_path: str):
    with h5py.File(h5_path, 'r') as h5_file:
        radar_group = h5_file['radar']
        radar_data = radar_group['data']
        radar_index = radar_group['index']
        radar_utc_nanosec = radar_group['utc_nanosec']
        radar_utc_sec = radar_group['utc_sec']

        for i in range(radar_data.shape[0]):
            yield radar_data[i], int(radar_index[i]), int(radar_utc_sec[i]), int(radar_utc_nanosec[i])


def get_h5_radar_frame_count(h5_path: str) -> int:
    with h5py.File(h5_path, 'r') as h5_file:
        return int(h5_file['radar']['data'].shape[0])


def iter_bin_radar_frames(bin_path: str, radar_params: dict):
    byte_per_frame = compute_bytes_per_frame(radar_params)
    frame_index = 0

    with open(bin_path, 'rb') as bin_file:
        while True:
            frame_bytes = bin_file.read(byte_per_frame)
            if not frame_bytes:
                break
            if len(frame_bytes) != byte_per_frame:
                break

            adc_frame = create_radar_frame_data(frame_bytes, radar_params)
            if adc_frame is None:
                frame_index += 1
                continue

            yield adc_frame, frame_index, 0, 0
            frame_index += 1


def get_bin_radar_frame_count(bin_path: str, radar_params: dict) -> int:
    byte_per_frame = compute_bytes_per_frame(radar_params)
    if byte_per_frame <= 0:
        return 0
    return os.path.getsize(bin_path) // byte_per_frame


def _clear_png_outputs(frames_dir: str):
    os.makedirs(frames_dir, exist_ok=True)
    for png_path in glob.glob(os.path.join(frames_dir, "*.png")):
        os.remove(png_path)


def _render_frame_sequence(frame_iter,
                           export_dir: str,
                           prefix: str,
                           adc_params: dict,
                           video_filename: str,
                           settings: RdRenderSettings = DEFAULT_RENDER_SETTINGS,
                           total_frames: int | None = None):
    frames_dir = os.path.join(export_dir, f"rd_map_frames_{prefix}_{settings.channel_name}")
    _clear_png_outputs(frames_dir)

    video_writer = None
    video_basename = video_filename or f"rd_map_{prefix}_{settings.channel_name}.mp4"
    video_path = os.path.join(export_dir, video_basename)

    frame_count = 0
    progress = tqdm(
        frame_iter,
        total=total_frames,
        desc=f"render {prefix} rd video",
        unit="frame")
    for adc_frame, frame_index, utc_sec, utc_nanosec in progress:
        frame = build_rd_map_image(adc_frame, adc_params, settings)

        frame_path = os.path.join(
            frames_dir,
            f"{frame_index:06d}_{utc_sec}-{utc_nanosec:09d}.png")
        cv2.imwrite(frame_path, frame)

        if video_writer is None:
            height, width = frame.shape[:2]
            video_writer = cv2.VideoWriter(
                video_path,
                cv2.VideoWriter_fourcc(*'mp4v'),
                settings.fps,
                (width, height))

        video_writer.write(frame)
        frame_count += 1

    progress.close()

    if video_writer is not None:
        video_writer.release()

    return frames_dir, video_path, frame_count


def render_radar_frames_to_video(h5_path: str,
                                 radar_config_path: str,
                                 export_dir: str,
                                 source: str = "h5",
                                 settings: RdRenderSettings = DEFAULT_RENDER_SETTINGS) -> tuple[str, str, int]:
    adc_params, _ = get_radar_config_params(radar_config_path)
    resolved_settings = derive_render_settings(adc_params, settings)
    total_frames = get_h5_radar_frame_count(h5_path)
    return _render_frame_sequence(
        iter_h5_radar_frames(h5_path),
        export_dir,
        source,
        adc_params,
        f"rd_map_{source}_{resolved_settings.channel_name}.mp4",
        settings=resolved_settings,
        total_frames=total_frames)

def render_radar_bin_frames_to_video(bin_path: str,
                                     radar_config_path: str,
                                     export_dir: str,
                                     source: str = "bin",
                                     settings: RdRenderSettings = DEFAULT_RENDER_SETTINGS) -> tuple[str, str, int]:
    adc_params, _ = get_radar_config_params(radar_config_path)
    resolved_settings = derive_render_settings(adc_params, settings)
    total_frames = get_bin_radar_frame_count(bin_path, adc_params)
    return _render_frame_sequence(
        iter_bin_radar_frames(bin_path, adc_params),
        export_dir,
        source,
        adc_params,
        f"rd_map_{source}_{resolved_settings.channel_name}.mp4",
        settings=resolved_settings,
        total_frames=total_frames)


def radar_vis(folder: str, radar_config_path: str):
    h5_path = os.path.join(folder, "data.h5")
    return render_radar_frames_to_video(h5_path, radar_config_path, folder, source="h5")
