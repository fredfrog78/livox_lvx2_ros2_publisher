import unittest
import os
import struct
import shutil
import sys

# Adjust path to import Lvx2Checker from the parent directory
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lvx2_checker import Lvx2Checker, print_fail # Import print_fail for test_truncate_negative_frames_error

class TestLvx2Truncation(unittest.TestCase):

    def setUp(self):
        self.test_input_filename = "test_input.lvx"
        self.output_dir = "test_output_dir"
        os.makedirs(self.output_dir, exist_ok=True)

        # Define LVX2 file structure components
        self.public_header_sig = b"livox_tech\0\0\0\0\0\0"
        self.public_header_ver = b'\x02\x00\x00\x00' # 2,0,0,0
        self.public_header_magic = 0xAC0EA767
        self.public_header = self.public_header_sig + self.public_header_ver + struct.pack('<I', self.public_header_magic)

        self.private_header_frame_dur = 50
        self.private_header_dev_count = 1
        self.private_header = struct.pack('<IB', self.private_header_frame_dur, self.private_header_dev_count)

        self.dev_info_lidar_sn = b'TESTLIDARSN\0\0\0\0\0\0'
        self.dev_info_hub_sn = b'TESTHUBSN\0\0\0\0\0\0\0\0'
        self.dev_info_id = 0
        self.dev_info_type_res = 0
        self.dev_info_type = 3 # AVIA
        self.dev_info_ext_en = 0
        self.dev_info_params = b'\x00' * 24 # Roll, Pitch, Yaw, X, Y, Z = 0.0
        self.device_info_block = (
            self.dev_info_lidar_sn + self.dev_info_hub_sn +
            struct.pack('<IBBB', self.dev_info_id, self.dev_info_type_res, self.dev_info_type, self.dev_info_ext_en) +
            self.dev_info_params
        )
        self.assertEqual(len(self.device_info_block), 63) # Sanity check

        self.header_total_size = len(self.public_header) + len(self.private_header) + len(self.device_info_block)
        self.assertEqual(self.header_total_size, 24 + 5 + 63) # 92

        # Frame 0
        self.frame0_data = b'frame0_data_content' # 19 bytes
        self.frame0_size = 24 + len(self.frame0_data) # 43 bytes
        self.frame0_offset_curr = self.header_total_size
        self.frame0_offset_next = self.frame0_offset_curr + self.frame0_size
        self.frame0_idx = 0
        self.frame0_header = struct.pack('<QQQ', self.frame0_offset_curr, self.frame0_offset_next, self.frame0_idx)

        # Frame 1
        self.frame1_data = b'frame1_data_longer' # 20 bytes
        self.frame1_size = 24 + len(self.frame1_data) # 44 bytes
        self.frame1_offset_curr = self.frame0_offset_next
        self.frame1_offset_next = self.frame1_offset_curr + self.frame1_size
        self.frame1_idx = 1
        self.frame1_header = struct.pack('<QQQ', self.frame1_offset_curr, self.frame1_offset_next, self.frame1_idx)

        # Frame 2
        self.frame2_data = b'frame2_even_longer_data' # 25 bytes
        self.frame2_size = 24 + len(self.frame2_data) # 49 bytes
        self.frame2_offset_curr = self.frame1_offset_next
        self.frame2_offset_next = 0 # Last frame
        self.frame2_idx = 2
        self.frame2_header = struct.pack('<QQQ', self.frame2_offset_curr, self.frame2_offset_next, self.frame2_idx)

        self.total_input_file_size = self.header_total_size + self.frame0_size + self.frame1_size + self.frame2_size
        self.assertEqual(self.total_input_file_size, 92 + 43 + 44 + 49) # 228 bytes

        with open(self.test_input_filename, 'wb') as f:
            f.write(self.public_header)
            f.write(self.private_header)
            f.write(self.device_info_block)
            f.write(self.frame0_header)
            f.write(self.frame0_data)
            f.write(self.frame1_header)
            f.write(self.frame1_data)
            f.write(self.frame2_header)
            f.write(self.frame2_data)

        self.checker = Lvx2Checker(self.test_input_filename)
        # Manually parse headers as truncate_file relies on this, similar to main() logic
        self.checker.file_size = os.path.getsize(self.test_input_filename)
        with open(self.test_input_filename, 'rb') as f_check:
            self.checker.f = f_check
            self.assertTrue(self.checker._parse_public_header(), "Setup: Public header parsing failed")
            self.assertTrue(self.checker._parse_private_header(), "Setup: Private header parsing failed")
            self.assertTrue(self.checker._parse_device_info_block(), "Setup: Device info parsing failed")
        self.checker.f = None


    def tearDown(self):
        if os.path.exists(self.test_input_filename):
            os.remove(self.test_input_filename)
        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)

    def _read_lvx2_file_data(self, filepath):
        """Helper to read and parse an LVX2 file for verification."""
        if not os.path.exists(filepath):
            return None, None, None, []

        file_size = os.path.getsize(filepath)
        frames = []
        with open(filepath, 'rb') as f:
            public_header = f.read(24)
            private_header = f.read(5)

            # Quick parse of device count from private header to read device info
            _, dev_count_check = struct.unpack('<IB', private_header)
            device_info_block_size = dev_count_check * 63
            device_info_block = f.read(device_info_block_size)

            current_pos = 24 + 5 + device_info_block_size
            while current_pos < file_size:
                f.seek(current_pos)
                frame_header_bytes = f.read(24)
                if len(frame_header_bytes) < 24:
                    break

                curr_off, next_off, idx = struct.unpack('<QQQ', frame_header_bytes)

                frame_data_size = 0
                if next_off == 0: # Last frame
                    frame_data_size = file_size - curr_off - 24
                else:
                    frame_data_size = next_off - curr_off - 24

                if frame_data_size < 0 : # Should not happen in valid file
                    # print(f"Warning: Calculated frame_data_size is negative ({frame_data_size}) for frame at offset {curr_off}. File: {filepath}")
                    break

                frame_data = f.read(frame_data_size)
                frames.append({
                    'header_bytes': frame_header_bytes,
                    'current_offset_abs': curr_off,
                    'next_offset_abs': next_off,
                    'frame_idx': idx,
                    'data': frame_data,
                    'calculated_size': 24 + frame_data_size
                })
                if next_off == 0 or next_off >= file_size:
                    break
                current_pos = next_off # This was the error in original thought, should be next_off from header.
                                       # No, current_pos is managed by seek(curr_off) from frame header.
                                       # The loop condition is on current_pos, but the seeking for next frame is based on its own curr_off.
                                       # The structure of this helper is to iterate based on offsets in frame headers, not just linear read.
                                       # Let's simplify: the truncate_file writes frames sequentially based on its logic.
                                       # This reader should also read based on sequential layout as written.
                # The critical part for reading is that `curr_off` from the frame header tells us where it *thinks* it is.
                # And `next_off` tells us where the *next* one *should* be.
                # For a correctly written file, current_pos should align with curr_off after reading the previous frame.
                # Let's re-think: The file is written by truncate_file. We are reading it back.
                # The current_pos for the loop should be the start of the current frame.
                # The next frame starts at 'next_off' if it's not 0.
                if next_off == 0:
                    break
                current_pos = next_off


        return public_header, private_header, device_info_block, frames

    def test_truncate_to_specific_frames(self):
        output_filepath = os.path.join(self.output_dir, "truncated_2_frames.lvx")
        self.assertTrue(self.checker.truncate_file(num_frames_to_keep=2, output_filepath=output_filepath))
        self.assertTrue(os.path.exists(output_filepath))

        ph, priv_h, dib, frames = self._read_lvx2_file_data(output_filepath)

        self.assertEqual(ph, self.public_header)
        self.assertEqual(priv_h, self.private_header)
        self.assertEqual(dib, self.device_info_block)
        self.assertEqual(len(frames), 2)

        # Frame 0 verification
        self.assertEqual(frames[0]['current_offset_abs'], self.frame0_offset_curr)
        self.assertEqual(frames[0]['next_offset_abs'], self.frame0_offset_next) # Next points to Frame 1
        self.assertEqual(frames[0]['frame_idx'], self.frame0_idx)
        self.assertEqual(frames[0]['data'], self.frame0_data)

        # Frame 1 verification (now the last frame)
        self.assertEqual(frames[1]['current_offset_abs'], self.frame1_offset_curr)
        self.assertEqual(frames[1]['next_offset_abs'], 0) # Should be 0 as it's the new last frame
        self.assertEqual(frames[1]['frame_idx'], self.frame1_idx) # Original index maintained by current implementation
                                                                # The prompt for truncate_file implies output frame_idx is sequential (0, 1, ...)
                                                                # The implementation uses 'frames_written' as output frame_idx.
        self.assertEqual(frames[1]['data'], self.frame1_data)


        expected_size = self.header_total_size + self.frame0_size + self.frame1_size
        self.assertEqual(os.path.getsize(output_filepath), expected_size)


    def test_truncate_to_zero_frames(self):
        output_filepath = os.path.join(self.output_dir, "truncated_0_frames.lvx")
        self.assertTrue(self.checker.truncate_file(num_frames_to_keep=0, output_filepath=output_filepath))
        self.assertTrue(os.path.exists(output_filepath))

        ph, priv_h, dib, frames = self._read_lvx2_file_data(output_filepath)

        self.assertEqual(ph, self.public_header)
        self.assertEqual(priv_h, self.private_header)
        self.assertEqual(dib, self.device_info_block)
        self.assertEqual(len(frames), 0)
        self.assertEqual(os.path.getsize(output_filepath), self.header_total_size)

    def test_truncate_to_more_than_available_frames(self):
        output_filepath = os.path.join(self.output_dir, "truncated_5_frames.lvx")
        self.assertTrue(self.checker.truncate_file(num_frames_to_keep=5, output_filepath=output_filepath))
        self.assertTrue(os.path.exists(output_filepath))

        # Output should be identical to input, as input has 3 frames and we asked for 5
        # The truncate_file ensures the last frame's next_offset_abs is 0. In our input, it already is.
        self.assertEqual(os.path.getsize(output_filepath), self.total_input_file_size)
        with open(self.test_input_filename, 'rb') as f_in, open(output_filepath, 'rb') as f_out:
            self.assertEqual(f_in.read(), f_out.read())

    def test_truncate_to_all_frames(self):
        output_filepath = os.path.join(self.output_dir, "truncated_3_frames.lvx")
        self.assertTrue(self.checker.truncate_file(num_frames_to_keep=3, output_filepath=output_filepath))
        self.assertTrue(os.path.exists(output_filepath))

        # Output should be identical to input. Last frame's next_offset_abs is already 0 in input.
        self.assertEqual(os.path.getsize(output_filepath), self.total_input_file_size)
        with open(self.test_input_filename, 'rb') as f_in, open(output_filepath, 'rb') as f_out:
            self.assertEqual(f_in.read(), f_out.read())

    def test_truncate_negative_frames_handled(self):
        # This test checks if the method handles it gracefully (e.g., returns False, prints error)
        # The current implementation of truncate_file prints an error and returns False.
        # We can't easily check print output without redirecting stdout.
        # So, we'll just check the return value.
        output_filepath = os.path.join(self.output_dir, "truncated_neg_frames.lvx")
        # Redirect stdout to capture print_fail
        from io import StringIO
        captured_output = StringIO()
        original_stdout = sys.stdout
        sys.stdout = captured_output # Redirect stdout

        result = self.checker.truncate_file(num_frames_to_keep=-1, output_filepath=output_filepath)

        sys.stdout = original_stdout # Restore stdout
        self.assertFalse(result, "truncate_file should return False for negative frame count.")
        self.assertIn("Number of frames to keep cannot be negative.", captured_output.getvalue())
        self.assertFalse(os.path.exists(output_filepath), "Output file should not be created for negative frame count.")


if __name__ == '__main__':
    unittest.main()
