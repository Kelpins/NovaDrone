import socket
import time
import csv # Added for CSV logging
from datetime import datetime, timedelta
from collections import deque

class DroneDataLogger: # Renamed class
    def __init__(self, host="192.168.76.215", port=23, update_interval=250):  # Default to 5Hz (200ms)
        """
        Initialize the DroneDataLogger. # Updated docstring
        
        Args:
            host (str): IP address of the drone
            port (int): Port number for the telnet connection
            update_interval (int): Update interval in milliseconds (default: 250ms for 4Hz)
        """
        self.host = host
        self.port = port
        self.update_interval = update_interval
        self.max_data_points = 10000  # Maximum number of data points to keep in memory
        
        # Initialize data storage with deques for efficient appending/popping
        self.timestamps = deque(maxlen=self.max_data_points)
        # self.window_size = 60 # seconds for plot window # Removed

        self.imu_data = {
            'roll': deque(maxlen=self.max_data_points),
            'pitch': deque(maxlen=self.max_data_points),
            'yaw': deque(maxlen=self.max_data_points),
        }
        self.ctrl_data = {
            'roll': deque(maxlen=self.max_data_points),
            'pitch': deque(maxlen=self.max_data_points),
            'throttle': deque(maxlen=self.max_data_points),
            'yaw': deque(maxlen=self.max_data_points),
        }
        self.motor_scalar_data = {
            'v': deque(maxlen=self.max_data_points),
            'f_cw': deque(maxlen=self.max_data_points),
            'b_cw': deque(maxlen=self.max_data_points),
            'f_ccw': deque(maxlen=self.max_data_points),
            'b_ccw': deque(maxlen=self.max_data_points), 
        }
        # self.derived_data = {'yaw_diff': deque(maxlen=self.max_data_points)} # Removed

        # Attributes to store the latest values for UI update and general reference
        self.latest_values = {
            'imu_roll': 0.0,
            'imu_pitch': 0.0,
            'imu_yaw': 0.0,
            'ctrl_roll': 0.0,
            'ctrl_pitch': 0.0,
            'throttle': 1000.0, # Default throttle, matches old self.last_throttle
            'ctrl_yaw': 0.0,
            'v_scalar': 0.0,
            'f_cw_scalar': 0.0,
            'b_cw_scalar': 0.0,
            'f_ccw_scalar': 0.0,
            'b_ccw_scalar': 0.0,
        }

        # Placeholders for UI elements # Removed section
        
        self.start_time = time.time()

        # Test state management attributes
        self.current_test_number = 0 # This will be removed from _start_new_log_file's usage
        self.test_state = "IDLE"  # Possible states: IDLE, PRE_TEST, TESTING, POST_TEST
        self.throttle_deadzone_threshold = 1050.0
        self.post_test_wait_duration = 5.0  # seconds
        self.last_state_transition_time = time.time()

        # Dynamic CSV Logging Initialization
        self.log_filename_base = "live logs/" # Modified
        self.current_log_filename = None
        self.csv_file = None
        self.csv_writer = None

        # Header definition:
        # 'name': Display name for UI if needed.
        # 'csv_col_name': The exact name of the column in the CSV file.
        # 'data_category': Which main dictionary holds this data's deque (e.g., 'imu_data', 'ctrl_data').
        # 'data_key': The key within the data_category dictionary for this specific data's deque.
        # 'latest_val_key': The key used in self.latest_values dictionary.
        # 'csv_index': The expected 0-based index in the incoming CSV data line (after splitting by comma).
        #               'timestamp' is special and not from the drone's CSV line, so its csv_index is None.
        self.header_info = [
            {'name': 'Timestamp', 'csv_col_name': 'timestamp', 'data_category': None, 'data_key': 'timestamps', 'latest_val_key': None, 'csv_index': None}, # Special case for our generated timestamp
            {'name': 'IMU Yaw', 'csv_col_name': 'imu_yaw', 'data_category': 'imu_data', 'data_key': 'yaw', 'latest_val_key': 'imu_yaw', 'csv_index': 0},
            {'name': 'IMU Pitch', 'csv_col_name': 'imu_pitch', 'data_category': 'imu_data', 'data_key': 'pitch', 'latest_val_key': 'imu_pitch', 'csv_index': 1},
            {'name': 'IMU Roll', 'csv_col_name': 'imu_roll', 'data_category': 'imu_data', 'data_key': 'roll', 'latest_val_key': 'imu_roll', 'csv_index': 2},
            {'name': 'Ctrl Roll', 'csv_col_name': 'ctrl_roll', 'data_category': 'ctrl_data', 'data_key': 'roll', 'latest_val_key': 'ctrl_roll', 'csv_index': 3},
            {'name': 'Ctrl Pitch', 'csv_col_name': 'ctrl_pitch', 'data_category': 'ctrl_data', 'data_key': 'pitch', 'latest_val_key': 'ctrl_pitch', 'csv_index': 4},
            {'name': 'Ctrl Throttle', 'csv_col_name': 'ctrl_throttle', 'data_category': 'ctrl_data', 'data_key': 'throttle', 'latest_val_key': 'throttle', 'csv_index': 5},
            {'name': 'Ctrl Yaw', 'csv_col_name': 'ctrl_yaw', 'data_category': 'ctrl_data', 'data_key': 'yaw', 'latest_val_key': 'ctrl_yaw', 'csv_index': 6},
            {'name': 'V Scalar', 'csv_col_name': 'v_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'v', 'latest_val_key': 'v_scalar', 'csv_index': 7},
            {'name': 'F CW Scalar', 'csv_col_name': 'f_cw_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'f_cw', 'latest_val_key': 'f_cw_scalar', 'csv_index': 8},
            {'name': 'B CW Scalar', 'csv_col_name': 'b_cw_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'b_cw', 'latest_val_key': 'b_cw_scalar', 'csv_index': 9},
            {'name': 'F CCW Scalar', 'csv_col_name': 'f_ccw_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'f_ccw', 'latest_val_key': 'f_ccw_scalar', 'csv_index': 10},
            {'name': 'B CCW Scalar', 'csv_col_name': 'b_ccw_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'b_ccw', 'latest_val_key': 'b_ccw_scalar', 'csv_index': 11},
        ]
        # Generate the simple list of CSV column names for writing the CSV header row
        self.header = [info['csv_col_name'] for info in self.header_info]
        # The print statement for logging will be moved to when a test starts.

        # self.setup_plot() # Removed

    def setup_connection(self):
        """Set up the socket connection to the drone."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)  # 5 second timeout
            self.sock.connect((self.host, self.port))
            self.stream = self.sock.makefile("r")
            print(f"Connected to {self.host}:{self.port}")
            return True
        except socket.error as e:
            print(f"Error connecting to {self.host}:{self.port} - {e}")
            return False
    
    def _start_new_log_file(self):
        """Handles the creation of a new log file.""" # Updated docstring
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.close()

        # self.current_test_number += 1 # Removed
        now = datetime.now()
        filename = now.strftime('%A %H:%M:%S.csv')
        self.current_log_filename = self.log_filename_base + filename

        self.csv_file = open(self.current_log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.header)
        print(f"Starting new log. Logging data to {self.current_log_filename}")
    
    def read_data(self):
        """Read and parse data from the socket stream."""
        try:
            if not hasattr(self, 'stream') or not self.stream:
                return None
                
            line = self.stream.readline()
            if not line:
                return None
                
            line = line.strip()
            if not line: # Handle empty lines after strip
                return None

            values_str = line.split(',')
            
            expected_value_count = len(self.header_info)
            if len(values_str) != expected_value_count:
                print(f"Malformed data line: Expected {expected_value_count} values, got {len(values_str)}. Line: '{line}'")
                return None

            log_timestamp = datetime.now().timestamp()
            log_row = [log_timestamp] + values_str
            if self.csv_writer and self.test_state in ["TEST_RUNNING", "WAITING_POST_TEST"]:
                self.csv_writer.writerow(log_row)

            vals = []
            for i, value_str in enumerate(values_str):
                field_info = self.header_info[i+1]
                try:
                    vals.append(float(value_str))
                except ValueError:
                    print(f"Error converting value for field '{field_info['name']}': Could not convert '{value_str}' to float. Line: '{line}'")
                    return None

            current_time = datetime.now()
            self.timestamps.append(current_time)

            for info in self.header_info:
                if info['csv_index'] is None:
                    continue
                raw_value = vals[info['csv_index']]
                if info['latest_val_key']:
                    self.latest_values[info['latest_val_key']] = raw_value
                if info['data_category'] and info['data_key']:
                    data_deque_category = getattr(self, info['data_category'])
                    # Special scaling for controller data for data processing
                    if info['data_category'] == 'ctrl_data':
                        if info['data_key'] == 'throttle':
                            scaled_value = (raw_value - 1000) / 1000.0 # Example scaling
                        else:
                            scaled_value = (raw_value - 1500) / 500.0 # Example scaling
                        data_deque_category[info['data_key']].append(scaled_value)
                    else:
                        data_deque_category[info['data_key']].append(raw_value)

            # Return True on successful data read and processing.
            # self.latest_values is updated, deques are populated.
            return True
                    
        except Exception as e:
            print(f"Error in read_data: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def process_data_and_manage_state(self): # Renamed method, removed frame argument
        """Update the data processing and state machine."""
            
        try:
            # Call read_data and check its success directly.
            # perform_data_update variable removed, logic will run if new_data is not None or state changes imply updates.

            # ----- State Machine -----
            if self.test_state == "IDLE":
                if self.read_data() and self.latest_values['throttle'] > self.throttle_deadzone_threshold: # Modified condition
                    self.test_state = "TEST_RUNNING"
                    self._start_new_log_file()
                    self.last_state_transition_time = time.time()
                    self.timestamps.clear()
                    # Clear previous test data from deques for a fresh log session
                    for category_dict in [self.imu_data, self.ctrl_data, self.motor_scalar_data]: # self.derived_data removed
                        for data_deque in category_dict.values():
                            data_deque.clear()
                    # perform_data_update = True # No longer needed, data processing happens if new_data exists
                else:
                    # Still IDLE, minimal work.
                    pass
            elif self.test_state == "TEST_RUNNING": # State: TEST_RUNNING - Actively logging test data
                if not self.read_data(): # If read_data fails
                    # Data read failed during a test, could log or handle error
                    pass # Continue, hoping connection recovers or relying on timeout.
                # Check throttle state even if read_data failed, as it might be a brief comms issue but state should update.
                if self.latest_values['throttle'] < self.throttle_deadzone_threshold:
                    self.test_state = "WAITING_POST_TEST"
                    self.last_state_transition_time = time.time()

            elif self.test_state == "WAITING_POST_TEST":
                _ = self.read_data() # Call read_data to keep processing incoming data, but ignore its direct result for state change here.
                                     # State change depends on throttle and time.
                if self.latest_values['throttle'] > self.throttle_deadzone_threshold:
                    self.test_state = "TEST_RUNNING"
                    self.last_state_transition_time = time.time()
                elif time.time() - self.last_state_transition_time > self.post_test_wait_duration:
                    self.test_state = "IDLE"
                    self.last_state_transition_time = time.time()
                    # current_test_number is not incremented, so this message might need adjustment or removal of test number.
                    print(f"Logging period complete. Log file: {self.current_log_filename}")

            # ----- Actual Data Processing/Logging -----
            # Data is read via self.read_data() at the beginning of the method.
            # Data is appended to deques in read_data() itself.
            # State transitions are handled above.
            # No specific plotting or data manipulation needed here for now beyond what read_data and state machine do.
            # The main purpose of this block was UI updates which are removed.

        except Exception as e:
            print(f"Error in process_data_and_manage_state: {e}") # Updated method name in print
            import traceback
            traceback.print_exc()

        # No return needed as it's not part of an animation loop callback anymore
    
    def run(self):
        """Start the data collection and logging.""" # Updated docstring
        if not self.setup_connection():
            print("Failed to establish connection. Exiting...")
            return
        
        try:
            print(f"Starting data logging. Press Ctrl+C to stop.") # Updated print statement
            
            # Matplotlib setup and animation related lines are removed.
            
            try:
                # Main loop for data processing
                while True:
                    self.process_data_and_manage_state() # Corrected method call
                    time.sleep(self.update_interval / 1000.0) # Respect update interval

            except Exception as e:
                print(f"Error in main processing loop: {e}") # Updated print statement for error
                raise
                
        except KeyboardInterrupt:
            print("\nShutdown requested...")
        except Exception as e:
            print(f"An error occurred: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
            try:
                pass # plt.close('all')  # Ensure all resources are closed # Removed # Updated comment
            except:
                pass
    
    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, 'sock') and self.sock:
            self.sock.close()
        if hasattr(self, 'stream') and self.stream:
            self.stream.close()
        print("Connection closed.")

        if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            if hasattr(self, 'current_log_filename') and self.current_log_filename:
                print(f"Data logged to {self.current_log_filename}")
            else:
                print("No active log file to save at cleanup, or logging did not start.") # Made message slightly more informative

def main():
    """Main function to run the application."""
    
    print("Starting Drone Data Logger") # Updated print
    print("-" * 40)
    
    plotter = DroneDataLogger() # Updated class instantiation
    
    try:
        plotter.run()
    except KeyboardInterrupt:
        print("\nShutdown requested. Cleaning up...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        plotter.cleanup()
        print("Application terminated.")

if __name__ == "__main__":
    main()
