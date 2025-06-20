#!/usr/bin/env python3

import struct
import os
import argparse
import sys
import numpy as np # For potential array operations later

# Define color codes for terminal output (optional, but helpful for readability)
class TermColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_ok(message):
    print(f"{TermColors.OKGREEN}[OK] {message}{TermColors.ENDC}")

def print_warn(message):
    print(f"{TermColors.WARNING}[WARN] {message}{TermColors.ENDC}")

def print_fail(message):
    print(f"{TermColors.FAIL}[FAIL] {message}{TermColors.ENDC}")

def print_info(message):
    print(f"{TermColors.OKBLUE}[INFO] {message}{TermColors.ENDC}")

def print_header(message):
     print(f"{TermColors.HEADER}{TermColors.BOLD}{message}{TermColors.ENDC}")

class Lvx2Checker:
    def __init__(self, filepath):
        self.filepath = filepath
        self.file_size = 0
        self.f = None # File object

        # Data extracted from headers
        self.public_header_data = {}
        self.private_header_data = {}
        self.device_infos = [] # List of device info dicts
        self.frame_duration_ms = 0 # From private header
        self.device_count = 0 # From private header

        # For timestamp and frame analysis
        self.frame_timestamps_ns = [] # For overall, first-packet-per-frame analysis
        self.frame_read_errors = 0
        self.package_read_errors = 0
        self.offset_mismatches = 0

        # Per-LiDAR timestamp data and stats
        self.lidar_frame_timestamps_ns = {}    # Key: lidar_id, Value: list of timestamps
        self.lidar_non_monotonic_jumps = {}    # Key: lidar_id, Value: count
        self.lidar_avg_delta_ms = {}         # Key: lidar_id, Value: avg delta in ms
        self.lidar_min_delta_ms = {}         # Key: lidar_id, Value: min delta in ms
        self.lidar_max_delta_ms = {}         # Key: lidar_id, Value: max delta in ms
        self.lidar_std_delta_ms = {}         # Key: lidar_id, Value: std dev of deltas in ms
        self.lidar_estimated_fps = {}        # Key: lidar_id, Value: estimated FPS


    def check_file(self):
        print_header(f"Starting check for LVX2 file: {self.filepath}")

        if not os.path.exists(self.filepath):
            print_fail(f"File not found: {self.filepath}")
            return False

        self.file_size = os.path.getsize(self.filepath)
        print_info(f"File size: {self.file_size} bytes")

        try:
            with open(self.filepath, 'rb') as f:
                self.f = f
                if not self._parse_public_header():
                    print_fail("Public header parsing failed. Aborting.")
                    return False
                print_info("Public header parsed successfully.")

                if not self._parse_private_header():
                    print_fail("Private header parsing failed. Aborting.")
                    return False
                print_info("Private header parsed successfully.")

                # --- Future steps will add calls to parsing methods here ---
                if not self._parse_device_info_block():
                    print_fail("Device info block parsing failed. Aborting.")
                    return False
                print_info("Device info block parsed successfully.")

                if not self._analyze_point_cloud_data_block():
                    print_fail("Point cloud data block analysis failed or encountered critical errors.")
                    # Decide if this should return False or just report errors
                else:
                    print_info("Point cloud data block analysis completed.")

                self._perform_timestamp_analysis() # Call timestamp analysis

            # Consolidated summary report at the end
            self._print_summary_report()

        except FileNotFoundError:
            print_fail(f"File not found during processing: {self.filepath}")
            # Ensure summary is printed even on some errors if desired, or handle in finally
            # For now, summary is only on "successful" run up to this point.
            return False
        except Exception as e:
            print_fail(f"An unexpected error occurred: {e}")
            import traceback
            print(traceback.format_exc())
            return False
        finally:
            if self.f and not self.f.closed:
                self.f.close()
        return True

    def _parse_public_header(self):
        print_info("Parsing Public Header...")
        try:
            data = self.f.read(24)
            if len(data) < 24:
                print_fail("Public Header: Unexpected EOF. Expected 24 bytes.")
                return False

            sig_bytes, vA_char, vB, vC, vD, magic_code = struct.unpack('<16scBBBI', data)

            file_signature = sig_bytes.split(b'\0', 1)[0].decode('ascii', errors='ignore')
            # Version is stored as char, byte, byte, byte. Convert char to int using ord().
            version_tuple = (ord(vA_char), vB, vC, vD)

            self.public_header_data['signature'] = file_signature
            self.public_header_data['version'] = version_tuple
            self.public_header_data['magic_code'] = magic_code

            valid = True
            if file_signature == "livox_tech" and sig_bytes[10:] == b'\0\0\0\0\0\0':
                print_ok(f"  File Signature: {file_signature} (Correct)")
            else:
                print_fail(f"  File Signature: {file_signature} (Incorrect or malformed!) Expected 'livox_tech' followed by null bytes.")
                valid = False # Critical

            expected_version = (2, 0, 0, 0)
            if version_tuple == expected_version:
                print_ok(f"  Version: {version_tuple} (Matches LVX2 spec)")
            else:
                print_warn(f"  Version: {version_tuple} (Differs from LVX2 spec {expected_version})")
                # Not a critical failure for the checker's purpose, but good to note.

            expected_magic_code = 0xAC0EA767
            if magic_code == expected_magic_code:
                print_ok(f"  Magic Code: {hex(magic_code)} (Correct)")
            else:
                print_fail(f"  Magic Code: {hex(magic_code)} (Incorrect!) Expected {hex(expected_magic_code)}.")
                valid = False # Critical

            return valid
        except struct.error as e:
            print_fail(f"Public Header: Error during unpacking: {e}")
            return False
        except Exception as e:
            print_fail(f"Public Header: An unexpected error occurred: {e}")
            return False

    def _parse_private_header(self):
        print_info("Parsing Private Header...")
        try:
            data = self.f.read(5)
            if len(data) < 5:
                print_fail("Private Header: Unexpected EOF. Expected 5 bytes.")
                return False

            frame_duration, device_count = struct.unpack('<IB', data)

            self.private_header_data['frame_duration_ms'] = frame_duration
            self.private_header_data['device_count'] = device_count
            self.frame_duration_ms = frame_duration
            self.device_count = device_count

            valid = True
            # For LVX2 version 2.0.0.0, frame duration is typically 50ms.
            # This might vary for other versions or configurations.
            if frame_duration == 50:
                print_ok(f"  Frame Duration: {frame_duration} ms (Typical for LVX2 v2.0.0.0)")
            else:
                print_warn(f"  Frame Duration: {frame_duration} ms (Note: LVX2 v2.0.0.0 typically uses 50ms)")

            if device_count > 0:
                print_ok(f"  Device Count: {device_count}")
            else:
                print_fail(f"  Device Count: {device_count} (No devices listed, this is unusual or problematic!)")
                valid = False # Potentially critical depending on expectations

            return valid
        except struct.error as e:
            print_fail(f"Private Header: Error during unpacking: {e}")
            return False
        except Exception as e:
            print_fail(f"Private Header: An unexpected error occurred: {e}")
            return False

    def _parse_device_info_block(self):
        print_info("Parsing Device Info Block(s)...")
        device_type_map = {
            0: "HAP (TX)", 1: "MID40", 2: "TELE", 3: "AVIA",
            6: "MID70", 9: "MID360", 10: "HAP (RX)",
            # Add other known types if necessary
        }

        for i in range(self.device_count):
            print_info(f"  Parsing Device Info {i+1}/{self.device_count}...")
            try:
                dev_info_data = self.f.read(63)
                if len(dev_info_data) < 63:
                    print_fail(f"Device Info {i+1}: Unexpected EOF. Expected 63 bytes, got {len(dev_info_data)}.")
                    return False

                lidar_sn_bytes, hub_sn_bytes, lidar_id, lidar_type_reserved, device_type, \
                extrinsic_enable, roll, pitch, yaw, x, y, z = \
                struct.unpack('<16s16sIBBBffffff', dev_info_data)

                lidar_sn = lidar_sn_bytes.split(b'\0',1)[0].decode('ascii', errors='ignore')
                hub_sn = hub_sn_bytes.split(b'\0',1)[0].decode('ascii', errors='ignore')
                device_type_name = device_type_map.get(device_type, 'Unknown')

                current_device_dict = {
                    'lidar_sn': lidar_sn,
                    'hub_sn': hub_sn,
                    'lidar_id': lidar_id,
                    'lidar_type_reserved': lidar_type_reserved,
                    'device_type': device_type,
                    'device_type_name': device_type_name,
                    'extrinsic_enable': extrinsic_enable,
                    'roll_deg': roll,
                    'pitch_deg': pitch,
                    'yaw_deg': yaw,
                    'x_m': x,
                    'y_m': y,
                    'z_m': z
                }
                self.device_infos.append(current_device_dict)

                print_ok(f"    LIDAR SN: {lidar_sn}")
                if hub_sn:
                    print_ok(f"    Hub SN: {hub_sn}")
                else:
                    print_info(f"    Hub SN: N/A")
                print_ok(f"    LIDAR ID: {lidar_id}")
                print_ok(f"    Device Type Code: {device_type} (Name: {device_type_name})")
                print_ok(f"    Extrinsic Enable: {extrinsic_enable}")
                if extrinsic_enable == 1:
                    print_ok(f"      Roll: {roll:.2f} deg, Pitch: {pitch:.2f} deg, Yaw: {yaw:.2f} deg")
                    print_ok(f"      X: {x:.3f} m, Y: {y:.3f} m, Z: {z:.3f} m")

            except struct.error as e:
                print_fail(f"Device Info {i+1}: Error during unpacking: {e}")
                return False
            except Exception as e:
                print_fail(f"Device Info {i+1}: An unexpected error occurred: {e}")
                return False
        return True

    def _analyze_point_cloud_data_block(self):
        print_header("Analyzing Point Cloud Data Block Structure...")
        if self.f.tell() >= self.file_size:
            print_warn("No data found after device info block or file is truncated.")
            return True # Technically not an error of this function, but no data to analyze

        current_frame_offset_in_file = self.f.tell()
        frame_count_for_eof_check = 0

        while True:
            self.f.seek(current_frame_offset_in_file)
            frame_header_data = self.f.read(24) # Frame Header size

            if len(frame_header_data) < 24:
                if frame_count_for_eof_check == 0: # First attempt to read a frame
                    print_fail(f"Point Cloud Data Block: Could not read first Frame Header. Expected 24 bytes, got {len(frame_header_data)} at offset {current_frame_offset_in_file}.")
                    self.frame_read_errors +=1
                    return False # Critical error if first frame header is missing/incomplete
                else: # EOF after successfully reading some frames
                    print_info("End of frames detected by partial read of frame header.")
                    break # Normal end of file processing

            current_offset_abs, next_offset_abs, frame_idx = struct.unpack('<QQQ', frame_header_data)
            print_info(f"Frame {frame_idx}: FileOffset={current_frame_offset_in_file}, HeaderCurrentOffset={current_offset_abs}, HeaderNextOffset={next_offset_abs}")
            frame_count_for_eof_check += 1

            # For overall first-package-in-frame timestamp (used for self.frame_timestamps_ns)
            overall_first_pkg_in_frame_ts_collected = False
            # For per-LiDAR first-package-in-frame timestamp for the current LVX frame
            lidars_seen_in_current_frame = set()

            if current_offset_abs != current_frame_offset_in_file:
                print_warn(f"Frame {frame_idx}: Mismatch in current offset. File says {current_offset_abs}, expected {current_frame_offset_in_file}. Seeking to header's offset.")
                self.offset_mismatches += 1
                self.f.seek(current_offset_abs)
                current_frame_offset_in_file = current_offset_abs

            package_read_pos = current_frame_offset_in_file + 24 # Start of packages for this frame
            frame_end_pos = next_offset_abs if next_offset_abs != 0 else self.file_size

            if frame_end_pos > self.file_size :
                print_warn(f"Frame {frame_idx}: frame_end_pos {frame_end_pos} (from next_offset_abs or file_size) is beyond actual file size {self.file_size}. Clamping to file_size.")
                frame_end_pos = self.file_size # Clamp to prevent reading past EOF

            packages_in_frame_count = 0
            while package_read_pos < frame_end_pos:
                if package_read_pos + 27 > frame_end_pos: # Check if there's enough space for a package header
                    if frame_end_pos < self.file_size or (next_offset_abs !=0 and package_read_pos < next_offset_abs) : # only warn if not at very end of file due to clamping
                         print_warn(f"Frame {frame_idx}: Insufficient data for a full package header at {package_read_pos}. Expected 27 bytes, remaining in frame: {frame_end_pos - package_read_pos}. Remainder of frame potentially corrupt or padding.")
                    self.package_read_errors +=1 # Count this as an error or anomaly
                    break # Cannot read next package header

                self.f.seek(package_read_pos)
                pkg_header_bytes = self.f.read(27)
                if len(pkg_header_bytes) < 27:
                    print_warn(f"Frame {frame_idx}: Truncated package header at {package_read_pos}. Expected 27, got {len(pkg_header_bytes)}.")
                    self.package_read_errors += 1
                    package_read_pos += len(pkg_header_bytes) # Advance by what was read
                    break # Stop processing this frame

                pkg_ver, lidar_id, lidar_type_res, ts_type, \
                timestamp_bytes, udp_counter, data_type, point_data_len, \
                frame_ctr_res, reserved_final_bytes = struct.unpack('<BIBB8sHBIB4s', pkg_header_bytes)

                pkg_timestamp_ns = struct.unpack('<Q', timestamp_bytes)[0]

                # Overall first package timestamp for the frame
                if not overall_first_pkg_in_frame_ts_collected:
                    self.frame_timestamps_ns.append(pkg_timestamp_ns)
                    overall_first_pkg_in_frame_ts_collected = True

                # Per-LiDAR first package timestamp for this frame
                if lidar_id not in lidars_seen_in_current_frame:
                    self.lidar_frame_timestamps_ns.setdefault(lidar_id, []).append(pkg_timestamp_ns)
                    lidars_seen_in_current_frame.add(lidar_id)

                package_read_pos += 27
                packages_in_frame_count += 1

                bytes_per_point_lvx = 0
                if data_type == 0x01 or data_type == 0x00: bytes_per_point_lvx = 14
                elif data_type == 0x02: bytes_per_point_lvx = 8
                else:
                    print_warn(f"Frame {frame_idx}, Pkg {packages_in_frame_count} LiDAR {lidar_id}: Unknown package data_type {data_type}. Cannot validate point_data_len consistency.")

                if bytes_per_point_lvx > 0 and point_data_len % bytes_per_point_lvx != 0:
                    print_warn(f"Frame {frame_idx}, Pkg {packages_in_frame_count} LiDAR {lidar_id}, DataType {data_type}: point_data_len {point_data_len} is not a multiple of bytes_per_point {bytes_per_point_lvx}.")

                if package_read_pos + point_data_len > frame_end_pos:
                    print_warn(f"Frame {frame_idx}, Pkg {packages_in_frame_count} LiDAR {lidar_id}: Declared point_data_len {point_data_len} exceeds remaining bytes in frame ({frame_end_pos - package_read_pos}). Reading available bytes.")
                    actual_read_len = frame_end_pos - package_read_pos
                    self.package_read_errors += 1
                    # self.f.seek(package_read_pos) # Already at correct position due to logic flow
                    # raw_points_data = self.f.read(actual_read_len) # Not strictly needed for checker, but good to note
                    package_read_pos += actual_read_len
                    break # Stop processing this frame as data is corrupt/incomplete

                # If just checking structure, actual read of point data can be skipped
                # self.f.seek(package_read_pos)
                # raw_points_data = self.f.read(point_data_len)
                # if len(raw_points_data) < point_data_len:
                #     print_warn(f"Frame {frame_idx}, Pkg {packages_in_frame_count} LiDAR {lidar_id}: Truncated points data. Expected {point_data_len}, got {len(raw_points_data)}.")
                #     self.package_read_errors += 1
                #     package_read_pos += len(raw_points_data)
                #     break
                package_read_pos += point_data_len

            print_info(f"Frame {frame_idx}: Parsed {packages_in_frame_count} package headers.")

            if next_offset_abs == 0:
                print_info(f"Frame {frame_idx}: Next offset is 0, marking end of data block.")
                break

            if next_offset_abs >= self.file_size: # Check if next_offset_abs points beyond the file
                print_warn(f"Frame {frame_idx}: Next offset {next_offset_abs} is at or beyond file size {self.file_size}. Treating as end of data.")
                if next_offset_abs > self.file_size: # Only count as error if it strictly points past EOF
                    self.frame_read_errors += 1
                break

            current_frame_offset_in_file = next_offset_abs
            if current_frame_offset_in_file < self.f.tell() and current_frame_offset_in_file != 0 : # Basic check for non-monotonic progression
                 print_warn(f"Frame {frame_idx}: Next offset {current_frame_offset_in_file} is less than current read position {self.f.tell()}. This could indicate corruption or loop.")
                 self.frame_read_errors += 1
                 break


        return True

    def _perform_timestamp_analysis(self):
        # Per-LiDAR Analysis Section
        # Overall analysis has been removed as per request.
        print_header("\nPer-LiDAR Timestamp Analysis (based on first package from each LiDAR per LVX frame):")
        if not self.lidar_frame_timestamps_ns:
            print_warn("  No per-LiDAR timestamp data collected to analyze.")
            return

        for lidar_id, timestamps_ns_list in self.lidar_frame_timestamps_ns.items():
            print_header(f"  LiDAR ID: {lidar_id}")
            if len(timestamps_ns_list) < 2:
                print_warn(f"    Not enough timestamps for LiDAR {lidar_id} to perform detailed analysis.")
                self.lidar_non_monotonic_jumps[lidar_id] = 0 # Ensure key exists even if no analysis
                self.lidar_estimated_fps[lidar_id] = 0.0
                self.lidar_avg_delta_ms[lidar_id] = 0.0
                continue

            current_lidar_non_monotonic_jumps = 0
            current_lidar_valid_deltas_ns = []

            for i in range(1, len(timestamps_ns_list)):
                prev_ts = timestamps_ns_list[i-1]
                curr_ts = timestamps_ns_list[i]
                delta_ns = curr_ts - prev_ts

                if delta_ns < 0:
                    current_lidar_non_monotonic_jumps += 1
                    print_warn(f"    Frame index {i} (LiDAR {lidar_id}): Non-monotonic! Prev: {prev_ts}, Curr: {curr_ts}, Delta: {delta_ns/1e6:.2f} ms")
                else:
                    current_lidar_valid_deltas_ns.append(delta_ns)
                    # print_info(f"    Frame {i} vs {i-1} (LiDAR {lidar_id}): Delta: {delta_ns/1e6:.2f} ms") # Verbose
                    if self.frame_duration_ms > 0: # Using overall frame duration for comparison
                        expected_delta_ns = self.frame_duration_ms * 1_000_000
                        if expected_delta_ns > 1000: # Avoid division by zero for tiny expected deltas
                            diff_from_expected_percentage = abs((delta_ns - expected_delta_ns) / expected_delta_ns)
                            if diff_from_expected_percentage > 0.10:
                                print_warn(f"      Timestamp {i} for LiDAR {lidar_id}: Delta {delta_ns/1e6:.2f} ms significantly deviates from overall expected frame duration {self.frame_duration_ms} ms.")
                        elif delta_ns > (expected_delta_ns * 1.1) or delta_ns < (expected_delta_ns * 0.9) : # Fallback for very small expected durations
                             print_warn(f"      Timestamp {i} for LiDAR {lidar_id}: Delta {delta_ns/1e6:.2f} ms deviates from overall expected frame duration {self.frame_duration_ms} ms.")

            self.lidar_non_monotonic_jumps[lidar_id] = current_lidar_non_monotonic_jumps
            print_info(f"    LiDAR {lidar_id} - Total non-monotonic jumps: {current_lidar_non_monotonic_jumps}")

            if current_lidar_valid_deltas_ns:
                self.lidar_min_delta_ms[lidar_id] = np.min(current_lidar_valid_deltas_ns) / 1e6
                self.lidar_max_delta_ms[lidar_id] = np.max(current_lidar_valid_deltas_ns) / 1e6
                self.lidar_avg_delta_ms[lidar_id] = np.mean(current_lidar_valid_deltas_ns) / 1e6
                self.lidar_std_delta_ms[lidar_id] = np.std(current_lidar_valid_deltas_ns) / 1e6
                print_info(f"    LiDAR {lidar_id} - Inter-frame deltas (ms): Min={self.lidar_min_delta_ms[lidar_id]:.2f}, Max={self.lidar_max_delta_ms[lidar_id]:.2f}, Avg={self.lidar_avg_delta_ms[lidar_id]:.2f}, StdDev={self.lidar_std_delta_ms[lidar_id]:.2f}")

                total_duration_s_lidar = (timestamps_ns_list[-1] - timestamps_ns_list[0]) / 1e9
                if total_duration_s_lidar > 0 and len(timestamps_ns_list) > 1:
                    self.lidar_estimated_fps[lidar_id] = (len(timestamps_ns_list) - 1) / total_duration_s_lidar
                    print_info(f"    LiDAR {lidar_id} - Estimated average frame rate: {self.lidar_estimated_fps[lidar_id]:.2f} Hz (based on {len(timestamps_ns_list)} timestamps)")
                else:
                    self.lidar_estimated_fps[lidar_id] = 0.0
                    if len(timestamps_ns_list) > 1 :
                         print_warn(f"    LiDAR {lidar_id} - Total duration of timestamped frames is zero. Cannot estimate FPS.")
            elif current_lidar_non_monotonic_jumps > 0:
                print_warn(f"    LiDAR {lidar_id} - No valid positive deltas to calculate statistics due to non-monotonic timestamps.")
                self.lidar_avg_delta_ms[lidar_id] = 0.0 # Ensure keys exist
                self.lidar_estimated_fps[lidar_id] = 0.0
            else:
                print_warn(f"    LiDAR {lidar_id} - No valid deltas to calculate statistics.")
                self.lidar_avg_delta_ms[lidar_id] = 0.0 # Ensure keys exist
                self.lidar_estimated_fps[lidar_id] = 0.0


    def _print_summary_report(self):
        print_header("\n--- LVX2 File Integrity Check Summary ---")
        print_info(f"File Path: {self.filepath}")
        print_info(f"File Size: {self.file_size} bytes")

        print_header("\nPublic Header Data:")
        if self.public_header_data:
            for key, value in self.public_header_data.items():
                if key == 'magic_code':
                    print_info(f"  {key.replace('_', ' ').title()}: {hex(value)}")
                else:
                    print_info(f"  {key.replace('_', ' ').title()}: {value}")
        else:
            print_warn("  No public header data parsed.")

        print_header("\nPrivate Header Data:")
        if self.private_header_data:
            for key, value in self.private_header_data.items():
                print_info(f"  {key.replace('_', ' ').title()}: {value}")
        else:
            print_warn("  No private header data parsed.")

        print_header("\nDevice Information:")
        if self.device_infos:
            for i, dev_info in enumerate(self.device_infos):
                print_info(f"  Device {i+1}:")
                print_info(f"    LIDAR ID: {dev_info.get('lidar_id', 'N/A')}")
                print_info(f"    SN: {dev_info.get('lidar_sn', 'N/A')}")
                print_info(f"    Type: {dev_info.get('device_type_name', dev_info.get('device_type', 'N/A'))}")
                if dev_info.get('extrinsic_enable', 0) == 1:
                    print_info(f"    Extrinsics (R,P,Y): {dev_info.get('roll_deg',0):.2f}, {dev_info.get('pitch_deg',0):.2f}, {dev_info.get('yaw_deg',0):.2f} deg")
                    print_info(f"    Extrinsics (X,Y,Z): {dev_info.get('x_m',0):.3f}, {dev_info.get('y_m',0):.3f}, {dev_info.get('z_m',0):.3f} m")
        else:
            print_warn("  No device information parsed.")

        print_header("\nStructural Integrity:")
        print_info(f"  Frame Offset Mismatches: {self.offset_mismatches}")
        print_info(f"  Package Read/Format Errors: {self.package_read_errors}")
        print_info(f"  Frame Read/Offset Errors: {self.frame_read_errors}")

        # Overall Timestamp Analysis Summary section removed as per request.

        print_header("\nPer-LiDAR Timestamp Analysis Summary:") # Ensures header is clear
        if self.lidar_frame_timestamps_ns:
            if not any(self.lidar_frame_timestamps_ns.values()): # Check if all lists are empty
                 print_info("  No per-LiDAR timestamp data was successfully collected for analysis (e.g. no packages found).")
            else:
                for lidar_id in sorted(self.lidar_frame_timestamps_ns.keys()):
                    # Check if there's actual analysis data for this LiDAR before printing header
                    if lidar_id in self.lidar_non_monotonic_jumps or lidar_id in self.lidar_estimated_fps:
                        print_info(f"  LiDAR ID: {lidar_id}")
                        print_info(f"    Non-monotonic Jumps: {self.lidar_non_monotonic_jumps.get(lidar_id, 'N/A')}")
                        # Check if FPS is meaningful or if enough timestamps were present
                        if self.lidar_estimated_fps.get(lidar_id, 0) > 0 or len(self.lidar_frame_timestamps_ns.get(lidar_id,[])) >=2 :
                            print_info(f"    Estimated Avg Frame Rate: {self.lidar_estimated_fps.get(lidar_id, 0.0):.2f} Hz")
                            print_info(f"    Avg Inter-frame Delta: {self.lidar_avg_delta_ms.get(lidar_id, 0.0):.2f} ms (Min: {self.lidar_min_delta_ms.get(lidar_id,0.0):.2f}, Max: {self.lidar_max_delta_ms.get(lidar_id,0.0):.2f}, StdDev: {self.lidar_std_delta_ms.get(lidar_id,0.0):.2f})")
                        else:
                            print_warn(f"    Not enough valid timestamps for LiDAR {lidar_id} for detailed FPS/Delta stats.")
                    elif not self.lidar_frame_timestamps_ns.get(lidar_id): # List exists but is empty
                         print_info(f"  LiDAR ID: {lidar_id} - No timestamps recorded (e.g. no packages from this LiDAR).")
                    # else: # This case should ideally not be hit if keys are added consistently
                    #    print_warn(f"  LiDAR ID: {lidar_id} - Data present in lidar_frame_timestamps_ns but no analysis results found.")

        else: # self.lidar_frame_timestamps_ns is empty
            print_info("  No per-LiDAR timestamp data structures initialized (e.g. no device info).")


        total_errors = self.offset_mismatches + self.package_read_errors + self.frame_read_errors
        # Summing up non_monotonic_jumps from per-LiDAR analysis for total_errors
        for lidar_id_key in self.lidar_non_monotonic_jumps:
            total_errors += self.lidar_non_monotonic_jumps[lidar_id_key]

        print_header("\nOverall Status:")
        if total_errors > 0:
            print_warn(f"File check completed. Issues found: YES (Total major issues: {total_errors})")
        else:
            print_ok("File check completed. No major issues detected in parsed sections.")
        print_header("--- End of Summary ---")

    def truncate_file(self, num_frames_to_keep, output_filepath):
        print_header(f"Starting truncation of LVX2 file: {self.filepath}")
        print_info(f"Number of frames to keep: {num_frames_to_keep}")
        print_info(f"Output file: {output_filepath}")

        if num_frames_to_keep < 0:
            print_fail("Number of frames to keep cannot be negative.")
            return False

        # Note: self.file_size, self.device_count, public/private headers, device_infos
        # should already be populated by the logic in main() before this method is called.
        if not hasattr(self, 'public_header_data') or not self.public_header_data:
             print_fail("Public header data not parsed. Call relevant parsing functions first.")
             return False
        if not hasattr(self, 'private_header_data') or not self.private_header_data:
             print_fail("Private header data not parsed. Call relevant parsing functions first.")
             return False
        if not hasattr(self, 'device_infos'): # device_infos can be empty if device_count is 0
             print_fail("Device info not parsed. Call relevant parsing functions first.")
             return False


        try:
            with open(self.filepath, 'rb') as infile, open(output_filepath, 'wb') as outfile:
                # 1. Header Copying
                infile.seek(0)
                public_header = infile.read(24)
                if len(public_header) < 24:
                    print_fail("Failed to read public header from input file.")
                    return False
                outfile.write(public_header)
                # print_info("Public header copied.") # Verbose, consider removing for cleaner output

                private_header = infile.read(5)
                if len(private_header) < 5:
                    print_fail("Failed to read private header from input file.")
                    return False
                outfile.write(private_header)
                # print_info("Private header copied.")

                device_info_size = self.device_count * 63
                if device_info_size > 0:
                    device_info_block = infile.read(device_info_size)
                    if len(device_info_block) < device_info_size:
                        print_fail(f"Failed to read full device info block. Expected {device_info_size}, got {len(device_info_block)}")
                        return False
                    outfile.write(device_info_block)
                    # print_info(f"Device info block ({self.device_count} devices, {device_info_size} bytes) copied.")
                # else:
                    # print_info("No device info block to copy (device_count is 0).")

                if num_frames_to_keep == 0:
                    print_info("Requested 0 frames. Output file will contain only headers.")
                    return True

                frames_written = 0
                # current_frame_offset_in_file should be the position after headers
                current_frame_offset_in_file = 24 + 5 + device_info_size

                initial_first_frame_offset_check = infile.tell()
                if initial_first_frame_offset_check != current_frame_offset_in_file:
                    print_warn(f"File pointer after reading headers is {initial_first_frame_offset_check}, expected {current_frame_offset_in_file}. This might indicate issues if not 0 frames were requested.")
                    # For safety, explicitly seek to the calculated start of frames if proceeding.
                    # However, if num_frames_to_keep is > 0, an error here is more likely.
                    # If this warning appears, it means the infile.read() sequence above didn't leave the pointer where expected.

                while frames_written < num_frames_to_keep:
                    infile.seek(current_frame_offset_in_file)
                    frame_header_bytes = infile.read(24)

                    if not frame_header_bytes or len(frame_header_bytes) < 24:
                        if frames_written > 0:
                             print_warn(f"Reached EOF or short read for frame header {frames_written + 1} at offset {current_frame_offset_in_file}. "
                                       f"Input file has {frames_written} frames. Output will contain these frames.")
                        else:
                            print_fail(f"Reached EOF or short read for the first frame header (frame {frames_written + 1}) at offset {current_frame_offset_in_file}. "
                                       f"Input file might be header-only or corrupted. Cannot write {num_frames_to_keep} frames.")
                        break

                    # Unpack original frame header to get offsets and frame_idx
                    # We use current_frame_offset_in_file as the definitive current_offset_abs for the output file.
                    _, next_offset_abs_original, frame_idx_original = struct.unpack('<QQQ', frame_header_bytes)

                    frame_size = 0
                    if next_offset_abs_original == 0: # Last frame in the input file
                        frame_size = self.file_size - current_frame_offset_in_file
                    else:
                        # Check for malformed next_offset_abs_original
                        if next_offset_abs_original < current_frame_offset_in_file :
                             print_fail(f"Frame {frame_idx_original} (input) has next_offset_abs {next_offset_abs_original} which is before its current_offset_abs {current_frame_offset_in_file}. Corruption suspected.")
                             return False # Critical error
                        frame_size = next_offset_abs_original - current_frame_offset_in_file

                    if frame_size <= 0 :
                        print_fail(f"Frame {frame_idx_original} (input) would have non-positive size {frame_size} (current_abs: {current_frame_offset_in_file}, next_abs_original: {next_offset_abs_original}). Stopping.")
                        return False

                    frame_data_without_header_size = frame_size - 24
                    if frame_data_without_header_size < 0:
                        print_fail(f"Frame {frame_idx_original} (input) has calculated data_without_header_size < 0 ({frame_data_without_header_size}). Frame size: {frame_size}. Stopping.")
                        return False

                    # Determine if this is the last frame for the output.
                    is_last_frame_for_output = (frames_written == num_frames_to_keep - 1) or \
                                               (next_offset_abs_original == 0 and frames_written < num_frames_to_keep)

                    output_next_offset_abs = 0
                    if not is_last_frame_for_output:
                        # The 'next_offset_abs' for the output frame points to the start of the *next* output frame.
                        # This is current_frame_offset_in_file (start of current frame) + frame_size (size of current frame)
                        output_next_offset_abs = current_frame_offset_in_file + frame_size

                    # Frame index in output should be sequential starting from 0 or 1.
                    # Using frames_written as 0-indexed frame_idx for output.
                    output_frame_header_bytes = struct.pack('<QQQ', current_frame_offset_in_file, output_next_offset_abs, frames_written)

                    outfile.write(output_frame_header_bytes)

                    # Read frame data from input (current_frame_offset_in_file + 24) and write to output
                    infile.seek(current_frame_offset_in_file + 24)
                    frame_data = infile.read(frame_data_without_header_size)

                    if len(frame_data) < frame_data_without_header_size:
                        print_warn(f"Frame {frame_idx_original} (input): Read only {len(frame_data)} bytes of data, expected {frame_data_without_header_size}. "
                                   f"Output frame {frames_written} will be truncated. This will likely be the last frame in the output.")
                        outfile.write(frame_data)
                        frames_written += 1
                        # Since data is incomplete, this effectively becomes the last frame.
                        # We need to ensure its header (already written) had next_offset_abs = 0.
                        # This requires seeking back and rewriting the header if output_next_offset_abs was not 0.
                        if output_next_offset_abs != 0:
                            print_info(f"Correcting header for partially written output frame {frames_written-1} to set next_offset_abs to 0.")
                            current_out_pos = outfile.tell()
                            outfile.seek(current_frame_offset_in_file) # Go to start of this frame in output
                            corrected_header = struct.pack('<QQQ', current_frame_offset_in_file, 0, frames_written -1)
                            outfile.write(corrected_header)
                            outfile.seek(current_out_pos) # Return to end for subsequent operations if any (though loop will break)
                        break

                    outfile.write(frame_data)
                    frames_written += 1
                    print_info(f"Written frame {frames_written}/{num_frames_to_keep} (Input Frame Index: {frame_idx_original}, Size: {frame_size} bytes). Output NextOffset: {output_next_offset_abs}")

                    if is_last_frame_for_output:
                        print_info(f"Last designated frame written. Total frames written: {frames_written}.")
                        break

                    # Prepare for next iteration for the input file reading
                    if next_offset_abs_original == 0: # Reached end of input file naturally
                        print_info(f"Reached end of input file (original next_offset_abs is 0). Total frames written: {frames_written}.")
                        if frames_written < num_frames_to_keep:
                             print_warn(f"Input file had only {frames_written} frames, less than requested {num_frames_to_keep}.")
                        break

                    if next_offset_abs_original >= self.file_size: # Safety break
                        print_warn(f"Original next_offset_abs {next_offset_abs_original} is at/beyond EOF. Total frames written: {frames_written}.")
                        break

                    # The current_frame_offset_in_file for the *output* is advanced by frame_size.
                    # The current_frame_offset_in_file for the *input* for the next loop iteration is next_offset_abs_original.
                    current_frame_offset_in_file = next_offset_abs_original


                print_info(f"Truncation loop finished. Total frames written: {frames_written} to {output_filepath}")
                if frames_written == 0 and num_frames_to_keep > 0:
                    print_warn("No frames were written to the output file, though frames were requested. Input file might be header-only or too short.")
                elif frames_written < num_frames_to_keep and num_frames_to_keep > 0 :
                     print_warn(f"Wrote {frames_written} frames, but {num_frames_to_keep} were requested. Input file was shorter than expected.")


        except FileNotFoundError:
            # This specific check for self.filepath should ideally be done before opening.
            # The main function already checks for input filepath existence.
            print_fail(f"Error: Input file {self.filepath} not accessible or output path for {output_filepath} is invalid.")
            return False
        except IOError as e:
            print_fail(f"IOError during file operations: {e}")
            return False
        except Exception as e:
            print_fail(f"An unexpected error occurred during truncation: {e}")
            import traceback
            print(traceback.format_exc())
            return False

        return True


def main():
    parser = argparse.ArgumentParser(description="Checks the integrity, analyzes, or truncates LVX2 files.")
    parser.add_argument("filepath", help="Path to the LVX2 file.")
    parser.add_argument("-t", "--truncate", type=int, help="Number of frames to keep in the output file.")
    parser.add_argument("-o", "--output", help="Path to the output LVX2 file (required if --truncate is used).")
    args = parser.parse_args()

    checker = Lvx2Checker(args.filepath)

    if args.truncate is not None:
        if not args.output:
            print_fail("--output <output_filepath> is required when --truncate is specified.")
            sys.exit(1)
        if args.truncate < 0:
            print_fail("Number of frames for truncate must be non-negative.")
            sys.exit(1)

        # Ensure necessary headers are parsed before truncation
        if not os.path.exists(args.filepath):
            print_fail(f"Input file not found: {args.filepath}")
            sys.exit(1)

        checker.file_size = os.path.getsize(args.filepath)
        try:
            with open(args.filepath, 'rb') as f_check:
                checker.f = f_check # Temporarily assign for header parsing
                if not checker._parse_public_header():
                    print_fail("Public header parsing failed. Cannot truncate.")
                    sys.exit(1)
                if not checker._parse_private_header():
                    print_fail("Private header parsing failed. Cannot truncate.")
                    sys.exit(1)
                if not checker._parse_device_info_block(): # This reads from checker.f
                    print_fail("Device info block parsing failed. Cannot truncate.")
                    sys.exit(1)
            checker.f = None # Release the file handle
        except Exception as e:
            print_fail(f"Error during initial header parsing for truncation: {e}")
            sys.exit(1)

        if checker.truncate_file(args.truncate, args.output):
            print_ok(f"File truncated successfully to {args.output}")
            sys.exit(0)
        else:
            print_fail("File truncation failed.")
            sys.exit(1)
    else:
        if not checker.check_file():
            sys.exit(1)
        sys.exit(0)

if __name__ == '__main__':
    main()
