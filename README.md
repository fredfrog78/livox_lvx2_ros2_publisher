# Livox LVX2 ROS 2 Publisher

## Overview
This ROS 2 package parses Livox LVX (version 2.0.0.0) files and publishes the LiDAR data as `sensor_msgs/PointCloud2` messages and IMU data (from device extrinsics) as `sensor_msgs/Imu` messages. It also publishes static TF transforms based on the extrinsic parameters found in the LVX file.

## Features
- Parses LVX v2.0.0.0 format.
- Publishes `sensor_msgs/PointCloud2` for each LiDAR unit.
- Publishes `sensor_msgs/Imu` for each LiDAR unit with configured extrinsics (orientation only, from static extrinsics).
- Publishes static TF transforms from `base_frame_id` to `lidar_frame_id_prefix<LIDAR_ID>`.
- Configurable topic names and frame IDs.
- Option to use original timestamps from the LVX file or playback at a fixed rate.
- Option to loop playback.

## Dependencies
The package depends on the following ROS 2 packages:
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`
- `builtin_interfaces`

These are standard ROS 2 packages and should be installed as part of your ROS 2 installation. If any are missing, they can typically be installed using `rosdep install --from-paths src --ignore-src -r -y`.

## Building the Package
1. Navigate to your ROS 2 workspace (e.g., `cd ~/your_ros2_ws`).
2. If you haven't already, source your ROS 2 installation (e.g., `source /opt/ros/humble/setup.bash` for ROS 2 Humble).
3. Clone this repository into your workspace's `src` directory:
   ```bash
   git clone <repository_url> src/livox_lvx2_ros2_publisher
   ```
   (If you have already cloned it or downloaded the code, ensure the `livox_lvx2_ros2_publisher` package directory is inside your `src` folder.)
4. Build the package:
   ```bash
   colcon build --packages-select livox_lvx2_ros2_publisher
   ```

## Running the Node
1. Source your workspace's setup file from the root of your workspace:
   ```bash
   source install/setup.bash
   ```
   (Ensure you are in `~/your_ros2_ws` when running this, or adjust the path accordingly).
2. Launch the node using the provided launch file. You **must** provide the `lvx_file_path` argument:
   ```bash
   ros2 launch livox_lvx2_ros2_publisher lvx2_publisher.launch.py lvx_file_path:="/path/to/your/file.lvx2"
   ```
   Replace `/path/to/your/file.lvx2` with the actual absolute path to your LVX file.

## Launch Arguments / Parameters
The following launch arguments are available and are passed as parameters to the node:

- `lvx_file_path` (string): Absolute path to the LVX file. **Required.**
- `point_cloud_topic_prefix` (string, default: `'livox/lidar_'`): Prefix for PointCloud2 topics. The final topic name will be `<prefix><LIDAR_ID>` (e.g., `livox/lidar_000000000000001`).
- `imu_topic_prefix` (string, default: `'livox/imu_'`): Prefix for IMU topics. The final topic name will be `<prefix><LIDAR_ID>` (e.g., `livox/imu_000000000000001`).
- `base_frame_id` (string, default: `'livox_base'`): The base frame_id for TF transforms and message headers.
- `lidar_frame_id_prefix` (string, default: `'livox_lidar_'`): Prefix for individual LiDAR frame IDs in TF. The final frame ID will be `<prefix><LIDAR_ID>` (e.g., `livox_lidar_000000000000001`).
- `use_original_timestamps` (bool, default: `'True'`): If `True`, the node attempts to use timestamps from the LVX file and replay data at the original recorded speed. If `False`, it uses the `playback_rate_hz` parameter.
- `playback_rate_hz` (double, default: `'20.0'`): The rate (in Hz) at which to publish messages if `use_original_timestamps` is set to `False`.
- `loop_playback` (bool, default: `'False'`): If `True`, the LVX file playback will loop continuously.
- `list_lidars` (bool, default: `'False'`): If `True`, the node will parse the LVX file to find all LiDAR devices, print their information (Serial Number, Lidar ID, Device Type, Extrinsic Parameters), and then exit without replaying point cloud data. This is useful for inspecting the contents of an LVX file.
  Examples:
  Using `ros2 run`:
  ```bash
  ros2 run livox_lvx2_ros2_publisher livox_lvx2_ros2_publisher --ros-args -p lvx_file_path:="/path/to/your/file.lvx2" -p list_lidars:=True
  ```
  Using `ros2 launch`:
  ```bash
  ros2 launch livox_lvx2_ros2_publisher lvx2_publisher.launch.py lvx_file_path:="/path/to/your/file.lvx2" list_lidars:=True
  ```
- `lidar_ids` (string, default: `''`): A comma-separated list of Lidar IDs (integers) to filter playback. If provided, only data from the specified LiDAR IDs will be published. If empty or omitted, data from all LiDARs in the file will be published.
  Examples (publish data only from LiDARs with ID 0 and ID 2):
  Using `ros2 run`:
  ```bash
  ros2 run livox_lvx2_ros2_publisher livox_lvx2_ros2_publisher --ros-args -p lvx_file_path:="/path/to/your/file.lvx2" -p lidar_ids:="0,2"
  ```
  Using `ros2 launch`:
  ```bash
  ros2 launch livox_lvx2_ros2_publisher lvx2_publisher.launch.py lvx_file_path:="/path/to/your/file.lvx2" lidar_ids:="0,2"
  ```

## Maintainer Information
The maintainer listed in the `package.xml` is `ROS Developer <rosdeveloper@example.com>`. If you fork or adapt this package for your own use, you may want to update this information in your version of the `package.xml` file.
