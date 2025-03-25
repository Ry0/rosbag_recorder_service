# ROS2 Rosbag Recorder Service

## Overview
The `rosbag_recorder_service` is a ROS2 package that provides a service-based interface for controlling rosbag recording. This package allows you to start, stop, and check the status of rosbag recordings via ROS2 services.

## Features

- Start rosbag recording with customizable options
- Stop ongoing rosbag recording
- Get current recording status

## Prerequisites

- ROS2 Jazzy (Confirmed to work)

## Installation

1. Clone the package into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone git@github.com:Ry0/rosbag_recorder_service.git
```

2. Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select rosbag_recorder_service
source install/setup.bash
```

## Usage

### Running the Node

Launch the rosbag recorder service node:

```bash
ros2 run rosbag_recorder_service rosbag_recorder_service_node
```

### Service Calls

#### Start Recording

full

```bash
ros2 service call /rosbag_recorder_service/record_start rosbag_recorder_service/srv/RecordStart "{
  bag_name: 'my_recording', 
  topics: ['/topic1', '/topic2'], 
  compression_mode: 'file', 
  compression_format: 'zstd', 
  storage_id: 'mcap', 
  uri: '.', 
  max_bagfile_size: 0, 
  max_cache_size: 100.0
}"
```

simple

```bash
ros2 service call /rosbag_recorder_service/record_start rosbag_recorder_service/srv/RecordStart "{
  bag_name: 'my_recording', 
  uri: '.', 
  max_bagfile_size: 0, 
  max_cache_size: 100.0
}"
```

#### Stop Recording

```bash
ros2 service call /rosbag_recorder_service/record_stop rosbag_recorder_service/srv/RecordStop "{}"
```

#### Get Recording Status

```bash
ros2 service call /rosbag_recorder_service/get_status rosbag_recorder_service/srv/GetStatus "{}"
```

## Service Descriptions

### RecordStart Service

- `bag_name`: Name of the bag file
- `topics`: List of topics to record
- `compression_mode`: Compression mode ('none', 'file', 'message')
- `compression_format`: Compression format ('zstd', 'cdr', etc.)
- `storage_id`: Storage format ('sqlite3', 'mcap')
- `uri`: Path to save the bag file
- `max_bagfile_size`: Maximum file size (bytes)
- `max_cache_size`: Maximum cache size (MB)

### RecordStop Service

Stops the current recording and returns the bag file path.

### GetStatus Service

Returns:
- `is_recording`: Whether recording is in progress
- `current_bag_path`: Path of the current bag file
- `recorded_messages`: Number of messages recorded
- `elapsed_time`: Recording duration

## Limitations

- Only one recording can be active at a time

## License

Distributed under the MIT License. See [LICENSE](LICENSE) for more information.
