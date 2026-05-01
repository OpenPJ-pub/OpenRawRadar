from collections import namedtuple

radar_topic = '/radar/sensor_data'

topic_map = {
    radar_topic: 'radar',
}

Timestamp = namedtuple('Timestamp',
                       ['pwr_sec', 'utc_sec', 'tick', 'tick_per_sec',
                        'subtick', 'subtick_per_tick', 'clk', 'clk_per_sec'])

# radar parameters
DATA_IN_PACKET = 1456
BYTES_IN_PACKET = 1466
