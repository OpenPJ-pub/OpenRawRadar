import math
import pprint


def read_radar_config(radar_config_path = None, dca_config_path = None, display_config = False):
    adc_params, channel_params = get_radar_config_params(radar_config_path)
    dca_params = get_dca_config_params(dca_config_path)

    if display_config:
        params_check(adc_params, channel_params, dca_params)

    return adc_params, channel_params, dca_params


def get_radar_config_params(path, print_out=False):
    chirp_params, channel_params = radar_config_parser(path)

    if print_out:
        print("chirp_params:")
        pprint.pprint(chirp_params, width=1, sort_dicts=False)

        print("channel_params:")
        pprint.pprint(channel_params, width=1, sort_dicts=False)

    return chirp_params, channel_params


def get_dca_config_params(path, print_out=False):
    dca_params = {}
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if line and "=" in line:
                key, value = line.split("=", 1)
                dca_params[key.strip()] = value.strip()

    if print_out:
        print("dca_params:")
        pprint.pprint(dca_params, width=1, sort_dicts=False)

    return dca_params


def params_check(adc_params, channel_params, dca_params):
    """
    Programming Chirp Parameters in TI Radar Devices (Rev. A).pdf
    """
    if adc_params['startFreq'] >= 76:
        cli_freq_scale_factor = 3.6  # 77GHz
    else:
        cli_freq_scale_factor = 2.7  # 60GHz

    start_freq_mmw = math.trunc(adc_params['startFreq'] * (1 << 26) / cli_freq_scale_factor)
    start_freq_actual = start_freq_mmw * (cli_freq_scale_factor / (1 << 26))
    print(f'start_freq_mmw: {start_freq_mmw:.0f} -> start_freq_actual: {start_freq_actual:.3f} GHz')

    max_slope = 266.0215  # MHz/us
    freq_slope_mmw = math.trunc(adc_params['freq_slope'] * (1 << 26) / (cli_freq_scale_factor * 1e3 * 900))
    freq_slope_actual = freq_slope_mmw * ((cli_freq_scale_factor * 1e3 * 900) / (1 << 26))
    print(f'freq_slope_mmw: {freq_slope_mmw:.1f} -> freq_slope_actual: {freq_slope_actual:.4f} '
          f'must <= {max_slope:.4f} MHz/us')
    if freq_slope_actual > max_slope:
        print(f"[Error] freq slope must <= {max_slope:.4f} MHz/us")

    # 0.300ms <= frame_periodicity <= 1342ms
    frame_periodicity_mmw = math.trunc(adc_params['frame_periodicity'] / (5 * 1e-6))
    frame_periodicity_actual = frame_periodicity_mmw * 5 * 1e-6
    print(f'frame_periodicity_mmw: {frame_periodicity_mmw} -> frame_periodicity_actual: {frame_periodicity_actual} ms '
          f'must in [0.3, 1342] ms')
    if not (0.3 <= frame_periodicity_actual <= 1342):
        print(f"[Error] frame_periodicity: {frame_periodicity_actual} ms must in [0.3, 1342] ms")

    # active_frame_time < frame_periodicity_actual
    chirp_repetition_period = adc_params['tx'] * (adc_params['idleTime'] + adc_params['rampEndTime'])  # us
    print(f'chirp repetition period: {chirp_repetition_period:.2f} us')
    active_frame_time = adc_params['chirps'] * chirp_repetition_period / 1e3  # ms
    all_tx_chirp_freq = adc_params['tx'] * adc_params['chirps'] / frame_periodicity_actual
    print(f'active frame time: {active_frame_time:.3f} ms '
          f'must < frame periodicity: {frame_periodicity_actual:.3f} ms')
    print(
        f'All tx chirp freq: {all_tx_chirp_freq:.2f} kHz, cycle duty: {active_frame_time * 100 / frame_periodicity_actual:.2f} %')
    if active_frame_time >= frame_periodicity_actual:
        print(f"[Error] active_frame_time must < frame_periodicity_actual")

    # sampling bandwidth < ramp bandwidth（slope * ramp time）
    adc_sample_time = 1e3 * adc_params['samples'] / adc_params['sample_rate']
    max_adc_sample_time = adc_params['rampEndTime'] - adc_params[
        'adc_valid_start_time'] - 0.15  # 0.15: to avoid tail sample distortion
    print(f"adc start time: {adc_params['adc_valid_start_time']:.2f} us, "
          f"ramp end time: {adc_params['rampEndTime']:.2f}us, "
          f"adc sample time: {adc_sample_time:.2f} us must < max adc sample time:{max_adc_sample_time:.2f} us")

    adc_sample_sweep_bandwidth = freq_slope_actual * 1e6 * adc_sample_time
    max_adc_sweep_bandwidth = freq_slope_actual * 1e6 * max_adc_sample_time
    print(f'adc_sample_sweep_bandwidth: {adc_sample_sweep_bandwidth / 1e6:.2f} MHz '
          f'must < max_adc_sweep_bandwidth: {max_adc_sweep_bandwidth / 1e6:.2f} MHz')
    if adc_sample_sweep_bandwidth >= max_adc_sweep_bandwidth:
        print(f"[Error] adc_sample_sweep_bandwidth must < max_adc_sweep_bandwidth")

    # ramp bandwidth < 4GHz - start_freq
    max_sweep_bandwidth = (81 - start_freq_actual) * 1e9
    total_ramp_sweep_bandwidth = freq_slope_actual * 1e6 * adc_params['rampEndTime']
    print(f'total_ramp_sweep_bandwidth:{total_ramp_sweep_bandwidth / 1e6:.2f} MHz '
          f'must <= {max_sweep_bandwidth / 1e6:.0f} MHz')
    if total_ramp_sweep_bandwidth / 1e6 > max_sweep_bandwidth / 1e6:
        print(f"[Error] total_ramp_sweep_bandwidth must < max_sweep_bandwidth")

    # ADC_PARAMS['idleTime'] >= synthesizer_ramp_down_time
    if total_ramp_sweep_bandwidth / 1e6 < 1000:
        synthesizer_ramp_down_time = 2
    elif total_ramp_sweep_bandwidth / 1e6 < 2000:
        synthesizer_ramp_down_time = 3.5
    elif total_ramp_sweep_bandwidth / 1e6 < 3000:
        synthesizer_ramp_down_time = 5
    else:
        synthesizer_ramp_down_time = 7
    print(f'idle time: {adc_params["idleTime"]:.3f} us '
          f'must >= Synthesizer Ramp Down Time: {synthesizer_ramp_down_time:.3f} us')
    if adc_params['idleTime'] < synthesizer_ramp_down_time:
        print(f"[Error] idle time must >= synthesizer ramp down time")

    # range, doppler resolution
    mid_freq = start_freq_actual * 1e9 + (
                adc_params['adc_valid_start_time'] + adc_sample_time / 2) * freq_slope_actual * 1e6
    mid_freq_wavelength = 3e8 / mid_freq

    num_doppler_bins = 2 ** math.ceil(math.log2(adc_params['chirps']))
    num_range_bins = 2 ** math.ceil(math.log2(adc_params['samples']))

    range_resolution_meters = 3e8 / (2 * adc_sample_sweep_bandwidth)
    range_idx_to_meters = (3e8 * adc_params['sample_rate'] * 1e3) / (2 * freq_slope_actual * 1e12 * num_range_bins)
    doppler_resolution_mps = mid_freq_wavelength / (2 * num_doppler_bins * chirp_repetition_period * 1e-6)
    max_range = (300 * 0.8 * adc_params['sample_rate']) / (2 * freq_slope_actual * 1e3)
    max_velocity = mid_freq_wavelength / (4 * chirp_repetition_period * 1e-6)

    print(
        f'[info] max range: {max_range:.2f} m, range resolution: {range_resolution_meters:.4f} m, interbin resolution: {range_idx_to_meters:.4f} m')
    print(f'[info] max velocity: {max_velocity:.2f} m/s, velocity resolution: {doppler_resolution_mps:.4f} m/s')

    # maximum_beat_frequency <= 20 MHz
    maximum_beat_frequency = 2e6 * freq_slope_actual * max_range / 3e8
    print(f'max beat frequency {maximum_beat_frequency:.2f} MHz '
          'must <= max I/F bandwith 20 MHz')
    if maximum_beat_frequency > 20:
        print(f"[Error] maximum beat frequency must <= 20 MHz")

    # maximum_sampling_frequency <= 22.5 MHz
    print(f'sample rate:{adc_params["sample_rate"] / 1e3:.2f} MHz must <= max sampling frequency 22.5 MHz')
    if adc_params['sample_rate'] / 1e3 > 22.5:
        print(f"[Error] sample rate must <= 22.5 MHz")

    # LVDS Data Size Per Chirp <= max Send Bytes Per Chirp
    lvds_data_size_per_chirp = adc_params['samples'] * adc_params['rx'] * adc_params['IQ'] * adc_params['bytes'] + 52
    lvds_data_size_per_chirp = math.ceil(lvds_data_size_per_chirp / 256) * 256
    max_send_bytes_per_chirp = (adc_params['idleTime'] + adc_params['rampEndTime']) * channel_params['numlaneEn'] * \
                               channel_params['lvdsBW'] / 8
    print(f"LVDS Data Size Per Chirp {lvds_data_size_per_chirp:.0f} Bytes "
          f"must <= Send Bytes Per Chirp {max_send_bytes_per_chirp:.0f} Bytes")
    if lvds_data_size_per_chirp > max_send_bytes_per_chirp:
        print(f"[Error] LVDS Data Size Per Chirp must <= max Send Bytes Per Chirp")

    # FPGA Packet Delay <= min required Packet Delay
    BYTES_IN_PACKET = 1456  # Data in payload per packet from FPGA
    BYTES_OF_PACKET = 1466  # payload size per packet from FPGA
    UDP_PACKET_OVERHEAD = 8
    IP_OVERHEAD = 20
    ETH_OVERHEAD = 14
    SizeInMBperSec = adc_params['samples'] * adc_params['rx'] * adc_params['IQ'] * adc_params['bytes'] * adc_params[
        'tx'] * adc_params['chirps'] * (1e-3 / frame_periodicity_actual)
    overheadRatio = (BYTES_OF_PACKET + UDP_PACKET_OVERHEAD + IP_OVERHEAD + ETH_OVERHEAD) / BYTES_IN_PACKET
    FPGA_throughput = 8 * SizeInMBperSec * overheadRatio
    print(f'Expected FPGA throughput: {FPGA_throughput:.2f} Mbps')

    minPacketDelay = 138.57 * math.exp(-FPGA_throughput / 179.157) + 2.73  # fitting curve
    print(f"min required Packet Delay {minPacketDelay:.2f} us "
          f"should >= FPGA Packet Delay {float(dca_params['packetDelay_us'])} us ")
    if float(dca_params['packetDelay_us']) > minPacketDelay:
        print(f"[Error] min required Packet Delay must >= FPGA Packet Delay")


def radar_config_parser(config_file_name):
    channel_params = {}
    radar_params = {}
    config = open(config_file_name, 'r')
    cur_profile_id = 0
    chirp_start_idx_fcf = 0
    chirp_end_idx_fcf = 0
    loop_count = 0
    chirp_start_idx_buf = []
    chirp_end_idx_buf = []
    profile_id_cpcfg_buf = []
    tx_enable_buf = []
    frame_cfg_flag = False

    for line in config:
        line_split = line.replace(" ", "").split("=")

        if 'channelTx' == line_split[0]:
            channel_params['txAntMask'] = int(line_split[1].split(";")[0])
            num_tx_chan = 0
            for chanIdx in range(3):
                if (channel_params['txAntMask'] >> chanIdx) & 1 == 1:
                    num_tx_chan = num_tx_chan + 1
            channel_params['numTxChan'] = num_tx_chan
        if 'channelRx' == line_split[0]:
            channel_params['rxAntMask'] = int(line_split[1].split(";")[0])
            num_rx_chan = 0
            for chanIdx in range(4):
                if (channel_params['rxAntMask'] >> chanIdx) & 1 == 1:
                    num_rx_chan = num_rx_chan + 1
            channel_params['numRxChan'] = num_rx_chan
            radar_params['rx'] = num_rx_chan
        if 'adcBitsD' == line_split[0]:
            radar_params['bytes'] = int(line_split[1].split(";")[0])
        if 'adcFmt' == line_split[0]:
            fmt_idx = int(line_split[1].split(";")[0])
            if fmt_idx == 1 or fmt_idx == 2:
                radar_params['IQ'] = 2
            else:
                radar_params['IQ'] = 1
        if 'dataRate' == line_split[0]:
            data_rate = int(line_split[1].split(";")[0])
            if data_rate == 0b0001:  # 0001b - 600 Mbps (DDR only)
                channel_params['lvdsBW'] = 600
            if data_rate == 0b0010:  # 0010b - 450 Mbps (SDR, DDR)
                channel_params['lvdsBW'] = 450
            if data_rate == 0b0011:  # 0011b - 400 Mbps (DDR only)
                channel_params['lvdsBW'] = 400
            if data_rate == 0b0100:  # 0100b - 300 Mbps (SDR, DDR)
                channel_params['lvdsBW'] = 300
            if data_rate == 0b0101:  # 0101b - 225 Mbps (DDR only)
                channel_params['lvdsBW'] = 225
            if data_rate == 0b0110:  # 0110b - 150 Mbps (DDR only)
                channel_params['lvdsBW'] = 150
        if 'laneEn' == line_split[0]:
            numlane_en = 0
            for laneIdx in range(4):
                if (int(line_split[1].split(";")[0]) >> laneIdx) & 1 == 1:
                    numlane_en = numlane_en + 1
            channel_params['numlaneEn'] = numlane_en

        if 'profileId' == line_split[0]:
            cur_profile_id = int(line_split[1].split(";")[0])
        if 'startFreqConst' == line_split[0]:
            if cur_profile_id == 0:
                radar_params['startFreq'] = int(line_split[1].split(";")[0]) * (3.6 / (1 << 26))
        if 'idleTimeConst' == line_split[0]:
            if cur_profile_id == 0:
                radar_params['idleTime'] = int(line_split[1].split(";")[0]) / 100
        if 'adcStartTimeConst' == line_split[0]:
            if cur_profile_id == 0:
                radar_params['adc_valid_start_time'] = int(line_split[1].split(";")[0]) / 100
        if 'rampEndTime' == line_split[0]:
            if cur_profile_id == 0:
                radar_params['rampEndTime'] = int(line_split[1].split(";")[0]) / 100
        if 'freqSlopeConst' == line_split[0]:
            if cur_profile_id == 0:
                radar_params['freq_slope'] = int(line_split[1].split(";")[0]) * (3.6e3 * 900 / (1 << 26))
        if 'txStartTime' == line_split[0]:
            if cur_profile_id == 0:
                radar_params['txStartTime'] = int(line_split[1].split(";")[0]) / 100
        if 'numAdcSamples' == line_split[0]:
            if cur_profile_id == 0:
                radar_params['samples'] = int(line_split[1].split(";")[0])
        if 'digOutSampleRate' == line_split[0]:
            if cur_profile_id == 0:
                radar_params['sample_rate'] = int(line_split[1].split(";")[0])
        if 'rxGain' == line_split[0]:
            cur_profile_id += 1

        # chirp cfg
        if 'chirpStartIdx' == line_split[0]:
            chirp_start_idx_buf.append(int(line_split[1].split(";")[0]))
        if 'chirpEndIdx' == line_split[0]:
            chirp_end_idx_buf.append(int(line_split[1].split(";")[0]))
        if 'profileIdCPCFG' == line_split[0]:
            profile_id_cpcfg_buf.append(int(line_split[1].split(";")[0]))
        if 'txEnable' == line_split[0]:
            tx_enable_buf.append(int(line_split[1].split(";")[0]))

        # frame cfg
        if 'chirpStartIdxFCF' == line_split[0]:
            chirp_start_idx_fcf = int(line_split[1].split(";")[0])
        if 'chirpEndIdxFCF' == line_split[0]:
            chirp_end_idx_fcf = int(line_split[1].split(";")[0])
        if 'loopCount' == line_split[0]:
            loop_count = int(line_split[1].split(";")[0])
        if 'periodicity' == line_split[0]:
            radar_params['frame_periodicity'] = int(line_split[1].split(";")[0]) / 200000
        if 'frameCount' == line_split[0]:
            radar_params['frameCount'] = int(line_split[1].split(";")[0])
            frame_cfg_flag = True
        if 'numAdcSamples' == line_split[0]:
            if not frame_cfg_flag:
                continue
            value = int(line_split[1].split(";")[0])
            if radar_params['IQ'] == 2 and value // 2 != radar_params['samples']:
                raise ValueError("numAdcSamples in frame cfg must the twice if complex, but got %d, %d" % (value, radar_params['samples']))

    radar_params['tx'] = len(tx_enable_buf)

    if radar_params['tx'] > channel_params['numTxChan']:
        raise ValueError(
            "exceed max tx num, check channelTx(%d) and chirp cfg(%d)." % (
                channel_params['numTxChan'], radar_params['tx']))
    tmp_chirp_num = chirp_end_idx_buf[0] - chirp_start_idx_buf[0] + 1
    all_chirp_num = tmp_chirp_num
    for i in range(radar_params['tx'] - 1):
        all_chirp_num += chirp_end_idx_buf[i + 1] - chirp_start_idx_buf[i + 1] + 1
        if tmp_chirp_num != chirp_end_idx_buf[i + 1] - chirp_start_idx_buf[i + 1] + 1:
            raise ValueError("AWR2243_read_config does not support different chirp number in different tx ant yet.")
    if all_chirp_num != chirp_end_idx_fcf - chirp_start_idx_fcf + 1:
        raise ValueError("chirp number in frame cfg and all chirp cfgs mismatched")
    radar_params['chirps'] = loop_count * tmp_chirp_num

    return radar_params, channel_params
