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
        self.frame_timestamps_ns = [] # List of first package timestamp for each frame
        self.non_monotonic_timestamp_jumps = 0
        self.estimated_fps_from_timestamps = 0.0 # For summary report
        self.avg_delta_ms_from_timestamps = 0.0 # For summary report
        self.frame_read_errors = 0
        self.package_read_errors = 0
        self.offset_mismatches = 0


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
            first_package_in_frame = True # For timestamp extraction

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

                if first_package_in_frame:
                    pkg_timestamp_ns = struct.unpack('<Q', timestamp_bytes)[0]
                    self.frame_timestamps_ns.append(pkg_timestamp_ns)
                    first_package_in_frame = False

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
        print_header("\nTimestamp Analysis:")
        if len(self.frame_timestamps_ns) < 2:
            print_warn("Not enough frame timestamps collected to perform detailed analysis.")
            return

        valid_deltas_ns = []
        # Reset non_monotonic_timestamp_jumps for multiple calls if file is re-analyzed (though not typical for this script)
        # self.non_monotonic_timestamp_jumps = 0 # Decided against resetting here, it's an accumulator for the whole file check

        for i in range(1, len(self.frame_timestamps_ns)):
            prev_ts = self.frame_timestamps_ns[i-1]
            curr_ts = self.frame_timestamps_ns[i]
            delta_ns = curr_ts - prev_ts

            if delta_ns < 0:
                self.non_monotonic_timestamp_jumps += 1
                print_warn(f"  Frame {i} vs {i-1}: Non-monotonic timestamp! Prev: {prev_ts}, Curr: {curr_ts}, Delta: {delta_ns/1e6:.2f} ms")
            else:
                valid_deltas_ns.append(delta_ns)
                print_info(f"  Frame {i} vs {i-1}: Delta: {delta_ns/1e6:.2f} ms")
                if self.frame_duration_ms > 0: # frame_duration_ms from private header
                    expected_delta_ns = self.frame_duration_ms * 1_000_000
                    # Check if expected_delta_ns is too small to avoid division by zero or tiny numbers
                    if expected_delta_ns > 1000: # e.g. > 1 microsecond
                        diff_from_expected_percentage = abs((delta_ns - expected_delta_ns) / expected_delta_ns)
                        if diff_from_expected_percentage > 0.1: # More than 10% deviation
                            print_warn(f"    Significant deviation from expected frame duration ({self.frame_duration_ms} ms). Actual: {delta_ns/1e6:.2f} ms, Expected: {expected_delta_ns/1e6:.2f} ms")
                    elif delta_ns > (self.frame_duration_ms * 1_000_000 * 1.1) or delta_ns < (self.frame_duration_ms * 1_000_000 * 0.9) : # Fallback for very small expected durations
                         print_warn(f"    Deviation from expected frame duration ({self.frame_duration_ms} ms). Actual: {delta_ns/1e6:.2f} ms")


        print_info(f"Total non-monotonic timestamp jumps: {self.non_monotonic_timestamp_jumps}")
        if valid_deltas_ns:
            min_delta_ms = np.min(valid_deltas_ns) / 1e6
            max_delta_ms = np.max(valid_deltas_ns) / 1e6
            self.avg_delta_ms_from_timestamps = np.mean(valid_deltas_ns) / 1e6 # Store for summary
            std_delta_ms = np.std(valid_deltas_ns) / 1e6
            print_info(f"Inter-frame timestamp deltas (ms): Min={min_delta_ms:.2f}, Max={max_delta_ms:.2f}, Avg={self.avg_delta_ms_from_timestamps:.2f}, StdDev={std_delta_ms:.2f}")

            if len(self.frame_timestamps_ns) > 1 :
                total_duration_s = (self.frame_timestamps_ns[-1] - self.frame_timestamps_ns[0]) / 1e9
                if total_duration_s > 0: # Avoid division by zero if first and last timestamp are the same
                    self.estimated_fps_from_timestamps = (len(self.frame_timestamps_ns) - 1) / total_duration_s # Store for summary
                    print_info(f"Estimated average frame rate from timestamps: {self.estimated_fps_from_timestamps:.2f} Hz ({len(self.frame_timestamps_ns)-1} intervals over {total_duration_s:.2f} s)")
                elif len(self.frame_timestamps_ns) -1 > 0 : # Many frames but zero duration
                     print_warn("Total duration of timestamped frames is zero. Cannot estimate FPS.")
                     self.estimated_fps_from_timestamps = 0 # Explicitly set
                # else only one frame timestamp, no interval
        elif self.non_monotonic_timestamp_jumps > 0 and not valid_deltas_ns:
             print_warn("No valid positive deltas to calculate statistics due to all-or-nothing non-monotonic timestamps.")
        else: # No valid deltas and no non-monotonic jumps (implies < 2 timestamps)
            print_warn("No valid deltas to calculate further statistics (likely due to insufficient number of timestamps).")

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

        print_header("\nTimestamp Analysis Summary:")
        print_info(f"  Non-monotonic Timestamp Jumps: {self.non_monotonic_timestamp_jumps}")
        if len(self.frame_timestamps_ns) >= 2 :
            print_info(f"  Estimated Average Frame Rate: {self.estimated_fps_from_timestamps:.2f} Hz")
            print_info(f"  Average Inter-frame Delta: {self.avg_delta_ms_from_timestamps:.2f} ms")
        else:
            print_warn("  Not enough timestamps for detailed FPS/Delta analysis.")

        total_errors = self.offset_mismatches + self.package_read_errors + self.frame_read_errors + self.non_monotonic_timestamp_jumps
        # Add other critical errors if they set flags or counters

        print_header("\nOverall Status:")
        if total_errors > 0:
            print_warn(f"File check completed. Issues found: YES (Total major issues: {total_errors})")
        else:
            print_ok("File check completed. No major issues detected in parsed sections.")
        print_header("--- End of Summary ---")


def main():
    parser = argparse.ArgumentParser(description="Checks the integrity and analyzes LVX2 files.")
    parser.add_argument("filepath", help="Path to the LVX2 file to check.")
    args = parser.parse_args()

    checker = Lvx2Checker(args.filepath)
    if not checker.check_file():
        sys.exit(1)
    sys.exit(0)

if __name__ == '__main__':
    main()
