import yaml
import rosbag2_py


def open_mcap_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('cdr', 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def load_topic_type_map(reader: rosbag2_py.SequentialReader) -> dict[str, str]:
    topic_types = reader.get_all_topics_and_types()
    return {topic.name: topic.type for topic in topic_types}


def load_total_message_count(bag_path: str) -> int:
    metadata_file = f"{bag_path}/metadata.yaml"
    with open(metadata_file, 'r', encoding='utf-8') as file_handle:
        metadata = yaml.safe_load(file_handle)

    total_messages = 0
    for topic_info in metadata['rosbag2_bagfile_information']['topics_with_message_count']:
        total_messages += topic_info['message_count']
    return total_messages
