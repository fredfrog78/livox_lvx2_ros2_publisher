#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Imu
from builtin_interfaces.msg import Time as RosTime
import geometry_msgs.msg
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math
import struct
import time
import os
import numpy as np
import sys # Import sys for explicit exit

# Helper function to convert Livox timestamp (nanoseconds) to ROS Time
def livox_ts_to_ros_time(timestamp_ns):
    return RosTime(sec=int(timestamp_ns // 1_000_000_000),
                   nanosec=int(timestamp_ns % 1_000_000_000))

# Helper function to create PointField
def make_point_field(name, offset, datatype, count=1):
    return PointField(name=name, offset=offset, datatype=datatype, count=count)

class Lvx2ParserNode(Node):
    def __init__(self):
        super().__init__('lvx2_parser_node')

        # Declare parameters
        self.declare_parameter('lvx_file_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('point_cloud_topic_prefix', 'livox/lidar_')
        self.declare_parameter('imu_topic_prefix', 'livox/imu_') # New parameter for IMU topics
        self.declare_parameter('base_frame_id', 'livox_base')
        self.declare_parameter('lidar_frame_id_prefix', 'livox_lidar_')
        self.declare_parameter('use_original_timestamps', False)
        self.declare_parameter('playback_rate_hz', 20.0) # Fallback if not using original timestamps
        self.declare_parameter('loop_playback', False)
        self.declare_parameter('list_lidars',
                               value=False,
                               descriptor=ParameterDescriptor(
                                   type=rclpy.Parameter.Type.BOOL,
                                   description='Set to True to list LiDAR info and exit.'))
        self.declare_parameter('lidar_ids',
                               '',  # Default value
                               ParameterDescriptor(
                                   description='Comma-separated list of LiDAR IDs to filter playback. Can be set as a string or an integer if only one ID and no comma is used.',
                                   dynamic_typing=True)) # Enable dynamic typing

        # Get parameters
        self.lvx_file_path = self.get_parameter('lvx_file_path').get_parameter_value().string_value
        self.pc_topic_prefix = self.get_parameter('point_cloud_topic_prefix').get_parameter_value().string_value
        self.imu_topic_prefix = self.get_parameter('imu_topic_prefix').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.lidar_frame_id_prefix = self.get_parameter('lidar_frame_id_prefix').get_parameter_value().string_value
        self.use_original_timestamps = self.get_parameter('use_original_timestamps').get_parameter_value().bool_value
        self.playback_rate_hz = self.get_parameter('playback_rate_hz').get_parameter_value().double_value
        self.loop_playback = self.get_parameter('loop_playback').get_parameter_value().bool_value
        self.list_lidars = self.get_parameter('list_lidars').get_parameter_value().bool_value

        # Get the lidar_ids parameter (which will be a ParameterValue object due to dynamic_typing)
        param_value_msg = self.get_parameter('lidar_ids').get_parameter_value()

        self.lidar_ids_str = '' # Default to empty string

        if param_value_msg.type == ParameterType.PARAMETER_STRING:
            self.lidar_ids_str = param_value_msg.string_value
            self.get_logger().info(f"Received lidar_ids as string: '{self.lidar_ids_str}'")
        elif param_value_msg.type == ParameterType.PARAMETER_INTEGER:
            # This case handles when a single numeric ID is passed and gets coerced to an integer
            # by the parameter system before rclpy receives it.
            self.lidar_ids_str = str(param_value_msg.integer_value)
            self.get_logger().info(f"Received lidar_ids as integer: {param_value_msg.integer_value}, converted to string: '{self.lidar_ids_str}'")
        elif param_value_msg.type == ParameterType.PARAMETER_NOT_SET and param_value_msg.string_value == '':
            # This handles the default case where the parameter is not set and defaults to empty string
            self.get_logger().info("lidar_ids parameter not set or empty, no filtering will be applied.")
            self.lidar_ids_str = '' # Explicitly ensure it's an empty string
        else:
            self.get_logger().warning(
                f"lidar_ids parameter has unexpected type ({param_value_msg.type}) or value. "
                f"String: '{param_value_msg.string_value}', Int: {param_value_msg.integer_value}. "
                f"Defaulting to no filtering."
            )
            self.lidar_ids_str = ''

        # The rest of the parsing logic for self.lidar_ids_str into self.selected_lidar_ids remains the same:
        self.selected_lidar_ids = []
        if self.lidar_ids_str:
            try:
                processed_ids_str = self.lidar_ids_str.strip().rstrip(',')
                if processed_ids_str:
                    self.selected_lidar_ids = [int(id_str.strip()) for id_str in processed_ids_str.split(',') if id_str.strip()]
                    if self.selected_lidar_ids:
                        self.get_logger().info(f"Playback will be filtered for LiDAR IDs: {self.selected_lidar_ids}")
                    elif self.lidar_ids_str:
                        self.get_logger().info(f"Lidar IDs string '{self.lidar_ids_str}' resulted in no valid IDs for filtering; no filtering applied.")
                elif self.lidar_ids_str:
                     self.get_logger().info(f"Lidar IDs string '{self.lidar_ids_str}' became empty after processing; no filtering applied.")
            except ValueError:
                self.get_logger().error(f"Invalid format for lidar_ids string: '{self.lidar_ids_str}' after processing. Please use comma-separated integers. Disabling filtering.")
                self.selected_lidar_ids = []

        self.publishers_ = {}  # Dict to store PointCloud2 publishers: {lidar_id: publisher}
        self.imu_publishers_ = {} # Dict to store IMU publishers: {lidar_id: publisher}
        self.device_infos_ = {} # Dict to store device specific info: {lidar_id: info}
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        self.device_frame_duration_ms = 50 # Default, will be updated from private header [cite: 31]

        self.get_logger().info("LVX2 Parser Node started. Processing file...")
        # Defer processing to a separate method or timer to allow __init__ to complete
        self.timer = self.create_timer(0.01, self.start_processing_once)

    def display_lidar_information_and_exit(self):
        self.get_logger().info("Displaying LiDAR information and preparing to exit...")
        try:
            with open(self.lvx_file_path, 'rb') as f:
                if not self._parse_public_header(f):
                    self.get_logger().error("Failed to parse public header for listing. Aborting list.")
                    return # This will lead to the finally block
                if not self._parse_private_header(f):
                    self.get_logger().error("Failed to parse private header for listing. Aborting list.")
                    return # This will lead to the finally block

                # Use print for actual Lidar details, logger for status messages
                print("-----------------------------------------------------")
                print("             LiDAR Device Information")
                print("-----------------------------------------------------")
                self.get_logger().info("Successfully parsed headers. Reading device info blocks...")

                for i in range(self.device_count):
                    dev_info_data = f.read(63)
                    if len(dev_info_data) < 63:
                        self.get_logger().error(f"Device Info {i}: Unexpected EOF while listing. Expected 63 bytes, got {len(dev_info_data)}. Aborting list.")
                        return # This will lead to the finally block

                    lidar_sn_bytes, hub_sn_bytes, lidar_id, lidar_type_reserved, device_type, \
                    extrinsic_enable, roll, pitch, yaw, x, y, z = \
                    struct.unpack('<16s16sIBBBffffff', dev_info_data)

                    lidar_sn = lidar_sn_bytes.split(b'\0',1)[0].decode('ascii', errors='ignore')
                    hub_sn = hub_sn_bytes.split(b'\0',1)[0].decode('ascii', errors='ignore')

                    # Use print for direct console output for this CLI-like feature
                    print(f"  LIDAR ID: {lidar_id}")
                    print(f"    SN: {lidar_sn}")
                    if hub_sn:
                        print(f"    Hub SN: {hub_sn}")
                    print(f"    Device Type Code: {device_type}") # Renamed for clarity
                    device_type_map = {0: "HAP (TX)", 1: "MID40", 2: "TELE", 3: "AVIA", 6: "MID70", 9: "MID360", 10: "HAP (RX)"}
                    print(f"    Device Type Name: {device_type_map.get(device_type, 'Unknown')}")
                    print(f"    Extrinsics Enabled: {'Yes' if extrinsic_enable == 1 else 'No'}")
                    if extrinsic_enable == 1:
                        print(f"    Extrinsics (Roll,Pitch,Yaw,X,Y,Z): {roll:.2f}deg, {pitch:.2f}deg, {yaw:.2f}deg, {x:.2f}m, {y:.2f}m, {z:.2f}m")
                    print("-----------------------------------------------------") # Separator after each device
            self.get_logger().info("Finished processing LiDAR information from file.")
        except FileNotFoundError:
            self.get_logger().error(f"LVX file not found: {self.lvx_file_path}. Cannot display LiDAR information.")
        except Exception as e:
            self.get_logger().error(f"An error occurred while displaying LiDAR information: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info("Display LiDAR information task complete. Initiating node shutdown.")
            self._initiate_shutdown()

    def start_processing_once(self):
        if self.timer:
            self.timer.cancel() # Ensure this runs only once

        if self.list_lidars:
            self.get_logger().info("List Lidars mode activated.") # Add log
            if not self.lvx_file_path or not os.path.exists(self.lvx_file_path):
                self.get_logger().error(f"LVX file path is invalid or file does not exist: {self.lvx_file_path} for listing.")
                self._initiate_shutdown() # Initiate shutdown directly
                return
            self.display_lidar_information_and_exit() # This method handles printing and its own shutdown
            return # Crucial: prevent process_lvx_file() call

        # If not list_lidars, proceed to normal processing
        self.process_lvx_file()


    def _parse_public_header(self, f):
        self.get_logger().info("Parsing Public Header...")
        try:
            data = f.read(24)
            if len(data) < 24:
                self.get_logger().error("Public Header: Unexpected EOF.")
                return False

            sig_bytes, vA, vB, vC, vD, magic_code = struct.unpack('<16scBBBI', data) # [cite: 23]
            file_signature = sig_bytes.split(b'\0', 1)[0].decode('ascii', errors='ignore') # [cite: 25]
            version_tuple = (ord(vA), vB, vC, vD) # vA is char, convert to int [cite: 23]

            self.get_logger().info(f"  File Signature: {file_signature}")
            self.get_logger().info(f"  Version: {version_tuple}") # [cite: 28]
            self.get_logger().info(f"  Magic Code: {hex(magic_code)}") # [cite: 23]

            if file_signature != "livox_tech" or sig_bytes[10:] != b'\0\0\0\0\0\0': # [cite: 25, 27]
                self.get_logger().error("Invalid file signature.")
                return False
            if version_tuple != (2, 0, 0, 0): # [cite: 28]
                self.get_logger().warning(f"Unexpected LVX2 version: {version_tuple}. Expected (2,0,0,0).")
            if magic_code != 0xAC0EA767: # [cite: 23]
                self.get_logger().error(f"Invalid magic code: {hex(magic_code)}. Expected 0xAC0EA767.")
                return False
            return True
        except struct.error as e:
            self.get_logger().error(f"Error parsing Public Header: {e}")
            return False


    def _parse_private_header(self, f):
        self.get_logger().info("Parsing Private Header...")
        try:
            data = f.read(5)
            if len(data) < 5:
                self.get_logger().error("Private Header: Unexpected EOF.")
                return False
            frame_duration, device_count = struct.unpack('<IB', data) # [cite: 30]
            self.get_logger().info(f"  Frame Duration: {frame_duration} ms") # [cite: 31]
            self.get_logger().info(f"  Device Count: {device_count}") # [cite: 34]
            if frame_duration != 50: # For version 2.0.0.0 [cite: 33]
                 self.get_logger().warning(f"Frame duration is {frame_duration}ms, expected 50ms for LVX2 v2.0.0.0.")
            self.device_count = device_count
            self.device_frame_duration_ms = frame_duration # Store for potential use in timestamp calculations
            return True
        except struct.error as e:
            self.get_logger().error(f"Error parsing Private Header: {e}")
            return False

    def _parse_device_info_block(self, f):
        self.get_logger().info("Parsing Device Info Block...")
        # More robust QoS for data that should ideally not be lost and is published frequently
        data_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Changed to RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10 # Keep a few messages
        )
        # QoS for TF static transforms (already handled by StaticTransformBroadcaster)

        for i in range(self.device_count):
            self.get_logger().info(f"  Parsing Device Info {i}...")
            try:
                # Device Info structure (SN, HubSN, ID, LidarType, DevType, ExtEnable, 6xExtrinsics), total 63 bytes as per spec fields [cite: 36, 39, 42]
                dev_info_data = f.read(63)
                if len(dev_info_data) < 63:
                    self.get_logger().error(f"Device Info {i}: Unexpected EOF. Expected 63 bytes, got {len(dev_info_data)}.")
                    return False

                # [cite: 39, 42]
                lidar_sn_bytes, hub_sn_bytes, lidar_id, lidar_type_reserved, device_type, \
                extrinsic_enable, roll, pitch, yaw, x, y, z = \
                struct.unpack('<16s16sIBBBffffff', dev_info_data)

                lidar_sn = lidar_sn_bytes.split(b'\0',1)[0].decode('ascii', errors='ignore') # [cite: 39]
                hub_sn = hub_sn_bytes.split(b'\0',1)[0].decode('ascii', errors='ignore') # [cite: 39]

                self.get_logger().info(f"    LIDAR SN: {lidar_sn}")
                self.get_logger().info(f"    Hub SN: {hub_sn if hub_sn else 'N/A'}")
                self.get_logger().info(f"    LIDAR ID: {lidar_id}") # [cite: 39]
                self.get_logger().info(f"    Device Type: {device_type}") # e.g. 9 for Mid-360, 10 for HAP [cite: 39]
                self.get_logger().info(f"    Extrinsic Enable: {extrinsic_enable}") # [cite: 42]
                self.get_logger().info(f"    Extrinsics (R,P,Y,X,Y,Z): {roll:.2f}deg, {pitch:.2f}deg, {yaw:.2f}deg, {x:.2f}m, {y:.2f}m, {z:.2f}m") # [cite: 42]

                device_frame_id = f"{self.lidar_frame_id_prefix}{lidar_id}"
                imu_frame_id = f"{self.lidar_frame_id_prefix}{lidar_id}_imu" # Consistent frame ID naming

                current_device_info = {
                    'sn': lidar_sn,
                    'hub_sn': hub_sn,
                    'frame_id': device_frame_id,
                    'imu_frame_id': imu_frame_id,
                    'type': device_type, # Store original type code
                    'extrinsic_enable': extrinsic_enable
                }
                if extrinsic_enable == 1:
                    current_device_info.update({
                        'roll_deg': roll,
                        'pitch_deg': pitch,
                        'yaw_deg': yaw,
                        'x': x, 'y': y, 'z': z
                    })
                self.device_infos_[lidar_id] = current_device_info

                # Conditional creation based on selected_lidar_ids
                if self.selected_lidar_ids and lidar_id not in self.selected_lidar_ids:
                    self.get_logger().info(f"    LiDAR ID {lidar_id} (SN: {lidar_sn}) is filtered out by lidar_ids parameter. Skipping ROS resource creation (publishers, TF).")
                    continue # Skip to the next device in the loop

                self.get_logger().info(f"    LiDAR ID {lidar_id} (SN: {lidar_sn}) is selected. Creating publishers and TF.")

                # Create PointCloud2 publisher
                pc_topic_name = f"{self.pc_topic_prefix}{lidar_id}"
                self.publishers_[lidar_id] = self.create_publisher(PointCloud2, pc_topic_name, data_qos_profile)
                self.get_logger().info(f"    Created PointCloud2 publisher for LiDAR ID {lidar_id} on topic {pc_topic_name}")

                if extrinsic_enable == 1: # [cite: 42]
                    # Publish static transform for position and orientation
                    t = geometry_msgs.msg.TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg() # TF is timeless but StaticBroadcaster needs a stamp
                    t.header.frame_id = self.base_frame_id
                    # Use frame_id from current_device_info for consistency, though it's same as local device_frame_id here
                    t.child_frame_id = current_device_info['frame_id']

                    t.transform.translation.x = float(current_device_info['x'])
                    t.transform.translation.y = float(current_device_info['y'])
                    t.transform.translation.z = float(current_device_info['z'])

                    q = self.euler_to_quaternion(math.radians(current_device_info['roll_deg']),
                                                 math.radians(current_device_info['pitch_deg']),
                                                 math.radians(current_device_info['yaw_deg']))
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
                    self.static_broadcaster_.sendTransform(t)
                    self.get_logger().info(f"    Published static transform for {current_device_info['frame_id']} relative to {self.base_frame_id}")

                    # Create and broadcast the static transform for the IMU relative to the LiDAR
                    t_imu = geometry_msgs.msg.TransformStamped()
                    t_imu.header.stamp = self.get_clock().now().to_msg() # Use current time for static transform
                    t_imu.header.frame_id = current_device_info['frame_id']  # Parent is the LiDAR frame
                    t_imu.child_frame_id = current_device_info['imu_frame_id']   # Child is the new IMU frame

                    # Set the translation based on the Livox Mid-360 User Manual (page 17) for Mid-360
                    # For other LiDARs, these values might be different or (0,0,0) if IMU is co-located.
                    # This example uses Mid-360 values. A more robust solution might involve
                    # device_type specific offsets or reading them from a config if they vary.
                    # For now, assuming these are generally applicable or a placeholder.
                    t_imu.transform.translation.x = 0.011
                    t_imu.transform.translation.y = 0.02329
                    t_imu.transform.translation.z = -0.04412

                    # The IMU shares the same orientation as the LiDAR frame, so rotation is identity
                    t_imu.transform.rotation.x = 0.0
                    t_imu.transform.rotation.y = 0.0
                    t_imu.transform.rotation.z = 0.0
                    t_imu.transform.rotation.w = 1.0

                    self.static_broadcaster_.sendTransform(t_imu)
                    self.get_logger().info(f"    Published static transform for {current_device_info['imu_frame_id']} relative to {current_device_info['frame_id']}")

                    # Create IMU publisher
                    imu_topic_name = f"{self.imu_topic_prefix}{lidar_id}"
                    self.imu_publishers_[lidar_id] = self.create_publisher(Imu, imu_topic_name, data_qos_profile)
                    self.get_logger().info(f"    Created IMU publisher for LiDAR ID {lidar_id} on topic {imu_topic_name}")

            except struct.error as e:
                self.get_logger().error(f"Error parsing Device Info {i}: {e}")
                return False
        return True

    def euler_to_quaternion(self, roll_rad, pitch_rad, yaw_rad):
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        q = [0.0]*4
        q[3] = cr * cp * cy + sr * sp * sy  # w
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        return q

    def _initiate_shutdown(self):
        self.get_logger().info("Node self-initiating shutdown as processing is complete.")
        if rclpy.ok():
            rclpy.shutdown()

    def _parse_point_cloud_data_block(self, f):
        self.get_logger().info("Parsing Point Cloud Data Block...")
        file_start_of_point_cloud_data_block = f.tell()
        current_frame_offset_in_file = file_start_of_point_cloud_data_block

        frame_count_overall = 0
        previous_frame_original_ts_for_delay_calc_ns = None
        # System time reference for precise delay calculation
        time_system_at_start_of_prev_frame_publish_cycle_ns = self.get_clock().now().nanoseconds


        while rclpy.ok():
            f.seek(current_frame_offset_in_file)
            frame_header_data = f.read(24) # [cite: 47] Frame Header size
            if len(frame_header_data) < 24:
                if frame_count_overall > 0:
                    self.get_logger().info("End of frames.")
                else:
                    self.get_logger().error("Point Cloud Data Block: Could not read Frame Header or block is empty.")
                break

            # [cite: 50] Frame Header fields
            current_offset_abs, next_offset_abs, frame_idx = struct.unpack('<QQQ', frame_header_data)
            self.get_logger().debug(f"Frame {frame_idx}: CurrentAbsOffset={current_offset_abs}, NextAbsOffset={next_offset_abs}")

            if current_offset_abs != current_frame_offset_in_file:
                self.get_logger().warning(f"Frame {frame_idx}: Mismatch in current offset. File says {current_offset_abs}, expected {current_frame_offset_in_file}. Seeking.")
                f.seek(current_offset_abs)
                current_frame_offset_in_file = current_offset_abs
                # It's good practice to re-read if you seek, but here we assume the initial read was from the correct new current_frame_offset_in_file

            points_by_lidar_in_frame = {}
            current_frame_first_pkg_ts_ns = None # Timestamp of the very first package in this LVX frame

            package_read_pos = current_frame_offset_in_file + 24 # Start of packages for this frame [cite: 47]
            frame_end_pos = next_offset_abs if next_offset_abs != 0 else os.fstat(f.fileno()).st_size

            while package_read_pos < frame_end_pos and rclpy.ok():
                f.seek(package_read_pos)
                pkg_header_bytes = f.read(27) # Package header (Version, LiDAR ID, LiDAR_Type, Timestamp Type, Timestamp(8s), Udp Counter, Data Type(B), Length(I), Frame_Counter, Reserve(4s)), total 27 bytes [cite: 52]
                if len(pkg_header_bytes) < 27:
                    self.get_logger().warning(f"Frame {frame_idx}: Truncated package header at {package_read_pos}.")
                    break

                # Unpack package header fields [cite: 52]
                # Adjust unpack format for 28 bytes. Assuming the extra byte is at the end as part of 'reserved_bytes' or similar.
                # Original: <BIBBccccccccHIHBcccc (27 bytes)
                # New format might be <BIBBccccccccHIHBccccc if the last 'cccc' (4 bytes) becomes 'ccccc' (5 bytes)
                # Or, if a new field is added. The comment implies point_data_len changed size or a new field was added.
                # Comment calculation: 1+4+1+1+8+2+4+2+1+4 = 28.
                # Original fields:         pkg_ver (B,1), lidar_id (I,4), lidar_type_res (B,1), ts_type (B,1), timestamp (8c,8), udp_counter (H,2), data_type (I,4 -> this seems to be the change from B,1 in old comment), point_data_len (H,2), frame_ctr_res (B,1), res (4c, 4)
                # Old comment calculation: 1+4+1+1+8+2+1+4+1+4 = 27.  Fields: pkg_ver, lidar_id, lidar_type_res, ts_type, timestamp, udp_counter, data_type(B,1), point_data_len(I,4), frame_ctr_res(B,1), res(4c,4)
                # The new comment says data_type is 2 bytes (H) and point_data_len is 4 bytes (I)
                # And frame_ctr_res is 1 byte (B), and a final reserved_bytes is 4 bytes (say, '4s' or 'cccc')
                # Let's re-evaluate based on the new comment's sum:
                # pkg_ver (B, 1)
                # lidar_id (I, 4)
                # lidar_type_res (B, 1) -> this is 'reserved' in the new comment
                # ts_type (B, 1)
                # timestamp (Q, 8) -> this is `cccccccc` (8c) in old unpack, so 8 bytes
                # udp_counter (H, 2)
                # data_type (H, 2) -> THIS IS THE CHANGE. Old comment: (B,1), New comment: (H,2)
                # point_data_len (I, 4) -> THIS IS THE CHANGE. Old comment: (H,2), New comment: (I,4)
                # frame_counter_reserved (B,1)
                # reserved_bytes (4s or I, 4) -> this is `cccc` (4c) in old unpack.
                # Old unpack: B I B B 8s H B I B 4s  (1+4+1+1+8+2+1+4+1+4 = 27)
                # New unpack: B I B B 8s H H I B 4s  (1+4+1+1+8+2+2+4+1+4 = 28)

                pkg_ver, lidar_id, lidar_type_res, ts_type, \
                timestamp_bytes, udp_counter, data_type, point_data_len, \
                frame_ctr_res, reserved_final_bytes = struct.unpack('<BIBB8sHBIB4s', pkg_header_bytes)

                pkg_timestamp_ns = struct.unpack('<Q', timestamp_bytes)[0] # [cite: 52] nanosecond timestamp

                if current_frame_first_pkg_ts_ns is None:
                    current_frame_first_pkg_ts_ns = pkg_timestamp_ns

                self.get_logger().debug(f"  Package for LiDAR ID {lidar_id}: DataType={data_type}, PointDataLen={point_data_len}, TimestampNS={pkg_timestamp_ns}")

                package_read_pos += 27
                f.seek(package_read_pos)
                raw_points_data = f.read(point_data_len)
                if len(raw_points_data) < point_data_len:
                    self.get_logger().warning(f"Frame {frame_idx}, LiDAR {lidar_id}: Truncated points data. Expected {point_data_len}, got {len(raw_points_data)}.")
                    package_read_pos += len(raw_points_data) # Adjust position by actual bytes read
                    break # Stop processing this frame if data is corrupt
                package_read_pos += point_data_len

                if lidar_id not in points_by_lidar_in_frame:
                    points_by_lidar_in_frame[lidar_id] = {
                        'x_coords': [], 'y_coords': [], 'z_coords': [],
                        'intensity_vals': [], 'tag_vals': [],
                        'representative_ts_ns': pkg_timestamp_ns,
                        'data_type': data_type
                    }
                elif points_by_lidar_in_frame[lidar_id]['data_type'] != data_type:
                     self.get_logger().warning(f"Frame {frame_idx}, LiDAR {lidar_id}: Mixed data types in packages. Using type {points_by_lidar_in_frame[lidar_id]['data_type']}.")

                bytes_per_point_lvx = 0
                if data_type == 0x01 or data_type == 0x00: bytes_per_point_lvx = 14 # [cite: 52]
                elif data_type == 0x02: bytes_per_point_lvx = 8 # [cite: 52]
                else:
                    self.get_logger().warning(f"Frame {frame_idx}, LiDAR {lidar_id}: Unknown data_type {data_type}. Skipping package.")
                    continue
                
                num_points_in_package = point_data_len // bytes_per_point_lvx if bytes_per_point_lvx > 0 else 0

                current_lidar_data = points_by_lidar_in_frame[lidar_id]
                if num_points_in_package > 0:
                    if data_type == 0x01 or data_type == 0x00: # Cartesian Coordinate System with Timestamp; Point Data Type: x, y, z (int32, mm), reflectivity (uint8), tag (uint8)
                        # LVX2 Spec: x(int, mm), y(int, mm), z(int, mm), intensity(uchar), tag(uchar)
                        dtype_mm = np.dtype([
                            ('x_mm', '<i4'), ('y_mm', '<i4'), ('z_mm', '<i4'),
                            ('intensity', 'u1'), ('tag', 'u1')
                        ])
                        points_array = np.frombuffer(raw_points_data, dtype=dtype_mm, count=num_points_in_package)
                        current_lidar_data['x_coords'].extend((points_array['x_mm'].astype(np.float32) / 1000.0).tolist())
                        current_lidar_data['y_coords'].extend((points_array['y_mm'].astype(np.float32) / 1000.0).tolist())
                        current_lidar_data['z_coords'].extend((points_array['z_mm'].astype(np.float32) / 1000.0).tolist())
                        current_lidar_data['intensity_vals'].extend(points_array['intensity'].astype(np.float32).tolist())
                        current_lidar_data['tag_vals'].extend(points_array['tag'].tolist())
                    elif data_type == 0x02: # Spherical Coordinate System; Point Data Type: depth (int32), theta (uint16), phi (uint16), reflectivity (uint8), tag (uint8) -> This is incorrect, data_type 0x02 is Cartesian Short
                        # LVX2 Spec: x(short, cm), y(short, cm), z(short, cm), intensity(uchar), tag(uchar)
                        dtype_cm = np.dtype([
                            ('x_cm', '<h'), ('y_cm', '<h'), ('z_cm', '<h'),
                            ('intensity', 'u1'), ('tag', 'u1')
                        ])
                        points_array = np.frombuffer(raw_points_data, dtype=dtype_cm, count=num_points_in_package)
                        current_lidar_data['x_coords'].extend((points_array['x_cm'].astype(np.float32) / 100.0).tolist())
                        current_lidar_data['y_coords'].extend((points_array['y_cm'].astype(np.float32) / 100.0).tolist())
                        current_lidar_data['z_coords'].extend((points_array['z_cm'].astype(np.float32) / 100.0).tolist())
                        current_lidar_data['intensity_vals'].extend(points_array['intensity'].astype(np.float32).tolist())
                        current_lidar_data['tag_vals'].extend(points_array['tag'].tolist())


            # Determine the timestamp for publishing messages for THIS frame
            # and for calculating delay relative to the PREVIOUS frame.
            frame_publication_ts_ns_for_header = None
            if self.use_original_timestamps:
                if current_frame_first_pkg_ts_ns is not None:
                    frame_publication_ts_ns_for_header = current_frame_first_pkg_ts_ns
                # Fallback if current frame had no packages with timestamps (e.g. empty frame)
                elif previous_frame_original_ts_for_delay_calc_ns is not None and self.device_frame_duration_ms > 0:
                    frame_publication_ts_ns_for_header = previous_frame_original_ts_for_delay_calc_ns + (self.device_frame_duration_ms * 1_000_000)
                    self.get_logger().debug(f"Frame {frame_idx} had no packages, using previous TS + frame_duration for header.")
                else: # First frame and no packages
                    frame_publication_ts_ns_for_header = self.get_clock().now().nanoseconds
                    self.get_logger().warning(f"Frame {frame_idx} is first and had no packages. Using current system time for header.")
            else: # Not using original timestamps
                frame_publication_ts_ns_for_header = self.get_clock().now().nanoseconds

            # --- Timestamp-based replay delay calculation ---
            if self.use_original_timestamps:
                if previous_frame_original_ts_for_delay_calc_ns is not None and \
                   current_frame_first_pkg_ts_ns is not None:
                    # Target delay based on original timestamps
                    target_delay_from_timestamps_ns = current_frame_first_pkg_ts_ns - previous_frame_original_ts_for_delay_calc_ns
                    if target_delay_from_timestamps_ns < 0: # Timestamps not monotonic
                        self.get_logger().warning(f"Frame {frame_idx}: Non-monotonic original timestamps detected ({target_delay_from_timestamps_ns}ns). Using default frame duration.")
                        target_delay_from_timestamps_ns = self.device_frame_duration_ms * 1_000_000
                    
                    # Time spent on processing/IO for the *previous* frame cycle
                    time_spent_on_prev_cycle_ns = self.get_clock().now().nanoseconds - time_system_at_start_of_prev_frame_publish_cycle_ns
                    
                    sleep_duration_ns = max(0, target_delay_from_timestamps_ns - time_spent_on_prev_cycle_ns)
                    if sleep_duration_ns > 0:
                        time.sleep(sleep_duration_ns / 1_000_000_000.0)
                        self.get_logger().debug(f"Frame {frame_idx}: Slept for {sleep_duration_ns / 1e6:.2f} ms to match original timestamps.")
                # Update for next iteration's delay calculation
                if current_frame_first_pkg_ts_ns is not None: # Only update if current frame provided a valid timestamp
                     previous_frame_original_ts_for_delay_calc_ns = current_frame_first_pkg_ts_ns
            else: # Fallback to fixed rate playback
                if self.playback_rate_hz > 0:
                    time.sleep(1.0 / self.playback_rate_hz)
            
            # Record system time after potential sleep, before current frame's heavy processing/publishing
            time_system_at_start_of_prev_frame_publish_cycle_ns = self.get_clock().now().nanoseconds
            ros_frame_publish_time = livox_ts_to_ros_time(frame_publication_ts_ns_for_header)

            # After all packages for this frame are read, process and then publish PointCloud2 and IMU messages
            for lidar_id, collected_data in points_by_lidar_in_frame.items():
                if self.selected_lidar_ids and lidar_id not in self.selected_lidar_ids:
                    self.get_logger().debug(f"Skipping LiDAR ID {lidar_id} as it's not in the selected list for frame {frame_idx}.")
                    continue

                num_total_points_for_lidar = len(collected_data['x_coords'])
                if num_total_points_for_lidar == 0:
                    continue

                # Create a single NumPy structured array from the collected lists
                dtype_pointcloud_point = np.dtype([
                    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                    ('intensity', '<f4'), ('tag', 'u1')
                ])
                final_points_array = np.empty(num_total_points_for_lidar, dtype=dtype_pointcloud_point)
                final_points_array['x'] = collected_data['x_coords']
                final_points_array['y'] = collected_data['y_coords']
                final_points_array['z'] = collected_data['z_coords']
                final_points_array['intensity'] = collected_data['intensity_vals']
                final_points_array['tag'] = collected_data['tag_vals']

                all_points_bytes = final_points_array.tobytes()
                collected_data['all_points_bytes'] = all_points_bytes # Store for publishing

                if lidar_id not in self.publishers_:
                    self.get_logger().warning(f"Frame {frame_idx}: LiDAR ID {lidar_id} has data but no PointCloud2 publisher. Skipping.")
                    continue

                pc_msg = PointCloud2()
                pc_msg.header.stamp = ros_frame_publish_time
                pc_msg.header.frame_id = self.device_infos_.get(lidar_id, {}).get('frame_id', f"{self.lidar_frame_id_prefix}{lidar_id}")
                pc_msg.height = 1
                pc_msg.width = num_total_points_for_lidar
                pc_msg.is_dense = True
                pc_msg.is_bigendian = False
                pc_msg.fields = [
                    make_point_field("x", 0, PointField.FLOAT32),
                    make_point_field("y", 4, PointField.FLOAT32),
                    make_point_field("z", 8, PointField.FLOAT32),
                    make_point_field("intensity", 12, PointField.FLOAT32),
                    make_point_field("tag", 16, PointField.UINT8)
                ]
                pc_msg.point_step = 17 # 3*4 + 4 + 1 bytes
                pc_msg.row_step = pc_msg.point_step * num_total_points_for_lidar
                pc_msg.data = collected_data['all_points_bytes'] # Use the newly created bytes
                self.publishers_[lidar_id].publish(pc_msg)
                self.get_logger().debug(f"Published PointCloud2 for LiDAR {lidar_id} from frame {frame_idx} with {num_total_points_for_lidar} points.")

                # Publish IMU message if publisher exists for this LiDAR ID
                if lidar_id in self.imu_publishers_ and self.device_infos_[lidar_id].get('extrinsic_enable') == 1:
                    dev_info = self.device_infos_[lidar_id]
                    imu_msg = Imu()
                    imu_msg.header.stamp = ros_frame_publish_time
                    # Set the frame_id to the specific IMU frame
                    imu_msg.header.frame_id = dev_info['imu_frame_id']

                    # Orientation is now provided by the TF tree, so set to identity and mark as unavailable.
                    imu_msg.orientation.x = 0.0
                    imu_msg.orientation.y = 0.0
                    imu_msg.orientation.z = 0.0
                    imu_msg.orientation.w = 1.0
                    # Mark orientation covariance as unavailable, as it is defined by the static transform
                    imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                    # Angular velocity and linear acceleration are not in LVX2 file
                    imu_msg.angular_velocity.x = 0.0
                    imu_msg.angular_velocity.y = 0.0
                    imu_msg.angular_velocity.z = 0.0
                    imu_msg.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Mark as unavailable

                    imu_msg.linear_acceleration.x = 0.0
                    imu_msg.linear_acceleration.y = 0.0
                    imu_msg.linear_acceleration.z = 0.0
                    imu_msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Mark as unavailable
                    
                    self.imu_publishers_[lidar_id].publish(imu_msg)
                    self.get_logger().debug(f"Published IMU for LiDAR {lidar_id} from frame {frame_idx}.")


            frame_count_overall += 1
            if next_offset_abs == 0: # Last frame processed
                self.get_logger().info(f"Processed last frame ({frame_idx}). Total frames: {frame_count_overall}")
                break
            current_frame_offset_in_file = next_offset_abs
            
            # Allow ROS to process events, especially if loop_playback is true or processing is very fast
            # rclpy.spin_once(self, timeout_sec=0.0001) # Very short timeout

        return True # Finished parsing this run of the point cloud data block


    def process_lvx_file(self):
        try:
            with open(self.lvx_file_path, 'rb') as f:
                if not self._parse_public_header(f): return
                if not self._parse_private_header(f): return
                if not self._parse_device_info_block(f): return

                start_of_point_cloud_data_block = f.tell()

                while rclpy.ok():
                    f.seek(start_of_point_cloud_data_block)
                    if not self._parse_point_cloud_data_block(f):
                        self.get_logger().error("Failed to parse point cloud data block.")
                        break

                    if self.loop_playback and rclpy.ok():
                        self.get_logger().info("Looping playback from the beginning.")
                        # Reset timestamp tracking for loop
                        # These are reset inside _parse_point_cloud_data_block at its start
                        time.sleep(0.1) # Brief pause before looping
                    else:
                        self.get_logger().info("Playback finished (or loop_playback is false).")
                        self._initiate_shutdown()
                        break
        except FileNotFoundError:
            self.get_logger().error(f"LVX file not found: {self.lvx_file_path}")
        except Exception as e:
            self.get_logger().error(f"An error occurred during LVX processing: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.get_logger().info("LVX2 Parser Node processing finished or stopped.")
            # Consider if shutdown is appropriate here or if node should remain alive for other services
            # rclpy.shutdown() # Removed to allow node to be managed externally

def main(args=None):
    rclpy.init(args=args)
    node = Lvx2ParserNode()
    try:
        # The node starts its processing via a timer in __init__.
        # We spin to keep the node alive for its publishers, subscribers, timers, etc.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception in main: {e}")
            import traceback
            node.get_logger().error(traceback.format_exc())
    finally:
        if rclpy.ok() and node:
            node.destroy_node()
        if rclpy.ok(): # Check if shutdown already called
            rclpy.shutdown()

if __name__ == '__main__':
    main()
