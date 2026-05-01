import numpy as np


def compute_bytes_per_frame(radar_param: dict) -> int:
    return (radar_param['tx'] *
            radar_param['chirps'] *
            radar_param['rx'] *
            radar_param['samples'] *
            radar_param['IQ'] *
            radar_param['bytes'])


def create_radar_frame_data(current_frame_bytes: bytes, radar_param: dict):
    byte_per_frame = compute_bytes_per_frame(radar_param)
    data_int16 = np.frombuffer(current_frame_bytes, dtype=np.int16)

    if data_int16.size != byte_per_frame // 2:
        return None

    adc_data_raw = np.reshape(
        data_int16,
        (-1,
         radar_param['chirps'] * radar_param['tx'],
         radar_param['samples'],
         radar_param['IQ'],
         radar_param['rx']))

    adc_data = np.zeros(
        (adc_data_raw.shape[0],
         radar_param['tx'],
         radar_param['chirps'],
         radar_param['samples'],
         radar_param['IQ'],
         radar_param['rx']),
        dtype=adc_data_raw.dtype)

    for tx_idx in range(radar_param['tx']):
        adc_data[:, tx_idx, :, :, :, :] = adc_data_raw[:, tx_idx::radar_param['tx'], :, :, :]

    if radar_param['IQ'] == 2:
        adc_data = (adc_data[..., 0, :] + 1j * adc_data[..., 1, :]).astype(np.complex64)
    else:
        adc_data = adc_data[..., 0, :].astype(np.float32)

    return np.transpose(adc_data, (1, 4, 0, 3, 2))
