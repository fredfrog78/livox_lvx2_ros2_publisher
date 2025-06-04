import rclpy
from livox_lvx2_ros2_publisher.livox_lvx2_ros2_publisher import Lvx2ParserNode
import sys
import os

def main_profile():
    # Minimal rclpy init if needed for node functionality like logging
    rclpy.init()

    node = Lvx2ParserNode()

    # Manually set essential parameters that would be set by ROS params or launch file
    # Ensure the dummy.lvx path is correct relative to where this script is run
    # Assuming dummy.lvx is in the parent directory of where the script is run from within /app
    node.lvx_file_path = "./dummy.lvx"
    node.pc_topic_prefix = 'livox/lidar_'
    node.imu_topic_prefix = 'livox/imu_'
    node.base_frame_id = 'livox_base'
    node.lidar_frame_id_prefix = 'livox_lidar_'
    node.use_original_timestamps = True # Or False, depending on what path we want to test
    node.playback_rate_hz = 20.0
    node.loop_playback = False # Ensure no looping

    # Ensure publishers and other ROS-dependent things are initialized if needed by process_lvx_file
    # This might require more setup if process_lvx_file relies on them being active
    # For now, assume internal logic of process_lvx_file can run sufficiently for profiling hotspots

    if not node.lvx_file_path or not os.path.exists(node.lvx_file_path):
        print(f"LVX file path is invalid or file does not exist: {node.lvx_file_path}")
        rclpy.shutdown()
        return

    print(f"Starting profiling of process_lvx_file with {node.lvx_file_path}")
    try:
        node.process_lvx_file()
    except SystemExit:
        print("SystemExit caught, proceeding with shutdown.")
    except Exception as e:
        print(f"Exception during process_lvx_file: {e}")
    finally:
        print("Profiling finished. Shutting down rclpy.")
        # node.destroy_node() # This might be problematic if not fully spun
        if rclpy.ok():
            rclpy.shutdown()
        print("Exiting profile_runner.py")
        sys.exit(0) # Ensure exit

if __name__ == '__main__':
    # We want cProfile to call main_profile()
    # So, when running: python -m cProfile -o ... profile_runner.py
    # cProfile will execute this script, and we want main_profile to run.
    # If cProfile itself calls the function, then we don't need to do anything special here.
    # Let's assume cProfile.runctx will be used on main_profile or similar.
    # For direct execution:
    main_profile()
