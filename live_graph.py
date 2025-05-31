import socket
import re
import time
import sys
import argparse
import csv # Added for CSV logging
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import matplotlib.patches as patches # Added for Drone UI
from matplotlib.animation import FuncAnimation
from datetime import datetime, timedelta
from collections import deque

class DroneDataPlotter:
    def __init__(self, host, port, window_size=30, update_interval=200):  # Default to 5Hz (200ms)
        """
        Initialize the DroneDataPlotter.
        
        Args:
            host (str): IP address of the drone
            port (int): Port number for the telnet connection
            window_size (int): Time window to display in seconds
            update_interval (int): Plot update interval in milliseconds (default: 200ms for 5Hz)
        """
        self.host = host
        self.port = port
        self.window_size = window_size
        self.update_interval = update_interval
        self.max_data_points = 10000  # Maximum number of data points to keep in memory
        
        # Initialize data storage with deques for efficient appending/popping
        self.timestamps = deque(maxlen=self.max_data_points)

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
        self.derived_data = {
            'pitch_diff': deque(maxlen=self.max_data_points),
            'roll_diff': deque(maxlen=self.max_data_points),
            'current_yaw_target': deque(maxlen=self.max_data_points),
            'yaw_diff': deque(maxlen=self.max_data_points),
            'data_field_11': deque(maxlen=self.max_data_points), # Placeholder for vals[11]
        }
        self.motor_scalar_data = {
            'v': deque(maxlen=self.max_data_points),      # vals[12]
            'f_cw': deque(maxlen=self.max_data_points),   # vals[13]
            'b_cw': deque(maxlen=self.max_data_points),   # vals[14]
            'f_ccw': deque(maxlen=self.max_data_points),  # vals[15]
            'b_ccw': deque(maxlen=self.max_data_points),  # vals[16]
        }

        # Attributes to store the latest values for UI update and general reference
        self.latest_values = {
            'imu_roll': 0.0,
            'imu_pitch': 0.0,
            'imu_yaw': 0.0,
            'ctrl_roll': 0.0,
            'ctrl_pitch': 0.0,
            'throttle': 1000.0, # Default throttle, matches old self.last_throttle
            'ctrl_yaw': 0.0,
            'pitch_diff': 0.0,
            'roll_diff': 0.0,
            'current_yaw_target': 0.0,
            'yaw_diff': 0.0,
            'data_field_11': 0.0, # Placeholder for vals[11]
            'v_scalar': 0.0,
            'f_cw_scalar': 0.0,
            'b_cw_scalar': 0.0,
            'f_ccw_scalar': 0.0,
            'b_ccw_scalar': 0.0,
        }

        # Placeholders for UI elements
        self.throttle_display_shape = None
        self.throttle_display_text = None
        self.motor_display_shapes = {}  # e.g., {'f_cw': shape_object, ...}
        self.motor_display_texts = {}   # e.g., {'f_cw': text_object, ...}
        self.ln_yaw_diff = None # For Yaw Difference plot
        
        self.start_time = time.time()

        # Test state management attributes
        self.current_test_number = 0
        self.test_state = "IDLE"  # Possible states: IDLE, PRE_TEST, TESTING, POST_TEST
        self.throttle_deadzone_threshold = 1050.0
        self.post_test_wait_duration = 5.0  # seconds
        self.last_state_transition_time = time.time()

        # Dynamic CSV Logging Initialization
        self.log_filename_base = "live_log"
        self.current_log_filename = None
        self.csv_file = None
        self.csv_writer = None

        # Header definition:
        # 'name': Display name for plots/UI if needed.
        # 'csv_col_name': The exact name of the column in the CSV file.
        # 'data_category': Which main dictionary holds this data's deque (e.g., 'imu_data', 'ctrl_data').
        # 'data_key': The key within the data_category dictionary for this specific data's deque.
        # 'latest_val_key': The key used in self.latest_values dictionary.
        # 'csv_index': The expected 0-based index in the incoming CSV data line (after splitting by comma).
        #               'timestamp' is special and not from the drone's CSV line, so its csv_index is None.
        self.header_info = [
            {'name': 'Timestamp', 'csv_col_name': 'timestamp', 'data_category': None, 'data_key': 'timestamps', 'latest_val_key': None, 'csv_index': None}, # Special case for our generated timestamp
            {'name': 'IMU Roll', 'csv_col_name': 'imu_roll', 'data_category': 'imu_data', 'data_key': 'roll', 'latest_val_key': 'imu_roll', 'csv_index': 0},
            {'name': 'IMU Pitch', 'csv_col_name': 'imu_pitch', 'data_category': 'imu_data', 'data_key': 'pitch', 'latest_val_key': 'imu_pitch', 'csv_index': 1},
            {'name': 'IMU Yaw', 'csv_col_name': 'imu_yaw', 'data_category': 'imu_data', 'data_key': 'yaw', 'latest_val_key': 'imu_yaw', 'csv_index': 2},
            {'name': 'Ctrl Roll', 'csv_col_name': 'ctrl_roll', 'data_category': 'ctrl_data', 'data_key': 'roll', 'latest_val_key': 'ctrl_roll', 'csv_index': 3},
            {'name': 'Ctrl Pitch', 'csv_col_name': 'ctrl_pitch', 'data_category': 'ctrl_data', 'data_key': 'pitch', 'latest_val_key': 'ctrl_pitch', 'csv_index': 4},
            {'name': 'Ctrl Throttle', 'csv_col_name': 'ctrl_throttle', 'data_category': 'ctrl_data', 'data_key': 'throttle', 'latest_val_key': 'throttle', 'csv_index': 5},
            {'name': 'Ctrl Yaw', 'csv_col_name': 'ctrl_yaw', 'data_category': 'ctrl_data', 'data_key': 'yaw', 'latest_val_key': 'ctrl_yaw', 'csv_index': 6},
            {'name': 'Pitch Diff', 'csv_col_name': 'pitch_diff', 'data_category': 'derived_data', 'data_key': 'pitch_diff', 'latest_val_key': 'pitch_diff', 'csv_index': 7},
            {'name': 'Roll Diff', 'csv_col_name': 'roll_diff', 'data_category': 'derived_data', 'data_key': 'roll_diff', 'latest_val_key': 'roll_diff', 'csv_index': 8},
            {'name': 'Current Yaw Target', 'csv_col_name': 'current_yaw_target', 'data_category': 'derived_data', 'data_key': 'current_yaw_target', 'latest_val_key': 'current_yaw_target', 'csv_index': 9},
            {'name': 'Yaw Diff', 'csv_col_name': 'yaw_diff', 'data_category': 'derived_data', 'data_key': 'yaw_diff', 'latest_val_key': 'yaw_diff', 'csv_index': 10},
            {'name': 'Data Field 11', 'csv_col_name': 'data_field_11', 'data_category': 'derived_data', 'data_key': 'data_field_11', 'latest_val_key': 'data_field_11', 'csv_index': 11},
            {'name': 'V Scalar', 'csv_col_name': 'v_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'v', 'latest_val_key': 'v_scalar', 'csv_index': 12},
            {'name': 'F CW Scalar', 'csv_col_name': 'f_cw_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'f_cw', 'latest_val_key': 'f_cw_scalar', 'csv_index': 13},
            {'name': 'B CW Scalar', 'csv_col_name': 'b_cw_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'b_cw', 'latest_val_key': 'b_cw_scalar', 'csv_index': 14},
            {'name': 'F CCW Scalar', 'csv_col_name': 'f_ccw_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'f_ccw', 'latest_val_key': 'f_ccw_scalar', 'csv_index': 15},
            {'name': 'B CCW Scalar', 'csv_col_name': 'b_ccw_scalar', 'data_category': 'motor_scalar_data', 'data_key': 'b_ccw', 'latest_val_key': 'b_ccw_scalar', 'csv_index': 16},
        ]
        # Generate the simple list of CSV column names for writing the CSV header row
        self.header = [info['csv_col_name'] for info in self.header_info]
        # The print statement for logging will be moved to when a test starts.

        self.setup_plot()

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
    
    def setup_plot(self):
        """Initialize the matplotlib plot using a configuration-driven approach."""
        plt.style.use('default')
        plt.rcParams.update({
            'axes.grid': True, 'grid.alpha': 0.3, 'axes.edgecolor': '#333333',
            'axes.labelcolor': '#333333', 'text.color': '#333333',
            'xtick.color': '#333333', 'ytick.color': '#333333', 'figure.facecolor': 'white',
        })

        if not hasattr(self, 'fig') or self.fig is None: # Ensure fig is created if not existing (e.g. called standalone)
             self.fig = plt.figure(figsize=(16, 9))
        else:
            self.fig.clf() # Clear figure if it already exists, to avoid overlaying plots on rerun

        gs = self.fig.add_gridspec(3, 2, height_ratios=[1, 1, 1], width_ratios=[3, 1])
        
        self.axes = {}
        self.lines = {}

        plot_configs = [
            {
                'name': 'yaw', 'grid_spec_idx': gs[0, 0], 'sharex_name': None,
                'ylabel': 'Yaw', 'ylim': (-200, 200), 'yticks': range(-180, 181, 45),
                'data_series': [
                    {'line_key': 'yaw_imu', 'data_key': 'yaw', 'plot_args': {'color': 'b', 'linestyle': '-', 'lw': 1.5, 'label': 'Yaw'}},
                    {'line_key': 'yaw_ctrl', 'data_key': 'yaw', 'plot_args': {'color': 'b', 'linestyle': '--', 'lw': 1, 'alpha': 0.7, 'label': 'Yaw (Ctrl)'}}
                ]
            },
            {
                'name': 'pitch', 'grid_spec_idx': gs[1, 0], 'sharex_name': 'yaw',
                'ylabel': 'Pitch', 'ylim': (-90, 90), 'yticks': range(-90, 91, 30),
                'data_series': [
                    {'line_key': 'pitch_imu', 'data_key': 'pitch', 'plot_args': {'color': 'g', 'linestyle': '-', 'lw': 1.5, 'label': 'Pitch'}},
                    {'line_key': 'pitch_ctrl', 'data_key': 'pitch', 'plot_args': {'color': 'g', 'linestyle': '--', 'lw': 1, 'alpha': 0.7, 'label': 'Pitch (Ctrl)'}}
                ]
            },
            {
                'name': 'roll', 'grid_spec_idx': gs[2, 0], 'sharex_name': 'yaw',
                'ylabel': 'Roll', 'ylim': (-60, 60), 'yticks': range(-45, 46, 15), 'xlabel': 'Time',
                'data_series': [
                    {'line_key': 'roll_imu', 'data_key': 'roll', 'plot_args': {'color': 'r', 'linestyle': '-', 'lw': 1.5, 'label': 'Roll'}},
                    {'line_key': 'roll_ctrl', 'data_key': 'roll', 'plot_args': {'color': 'r', 'linestyle': '--', 'lw': 1, 'alpha': 0.7, 'label': 'Roll (Ctrl)'}}
                ]
            },
            {
                'name': 'yaw_diff', 'grid_spec_idx': gs[2, 1], 'sharex_name': 'yaw',
                'ylabel': 'Yaw Diff (deg)', 'ylim': (-45, 45), 'yticks': None, 'xlabel': 'Time', # yticks can be default
                'data_series': [
                    {'line_key': 'yaw_diff_line', 'data_key': 'yaw_diff', 'plot_args': {'color': 'c', 'linestyle': '-', 'lw': 1.5, 'label': 'Yaw Diff'}}
                ]
            }
        ]

        for config in plot_configs:
            ax = self.fig.add_subplot(config['grid_spec_idx'], sharex=self.axes.get(config['sharex_name']))
            self.axes[config['name']] = ax
            ax.set_ylabel(config['ylabel'])
            ax.set_ylim(config['ylim'])
            if config['yticks'] is not None:
                ax.set_yticks(config['yticks'])
            if config.get('xlabel'):
                ax.set_xlabel(config['xlabel'])

            current_lines = []
            for series_conf in config['data_series']:
                # Determine data category based on line_key or other logic if needed
                # For now, imu lines use self.imu_data, ctrl lines use self.ctrl_data, yaw_diff uses self.derived_data
                # This logic might need to be more robust or explicit in config
                data_cat_map = {'imu': self.imu_data, 'ctrl': self.ctrl_data, 'line': self.derived_data} # Simplified
                category_prefix = series_conf['line_key'].split('_')[1] if '_' in series_conf['line_key'] else 'line'

                # Plotting empty data initially, actual data comes from update_plot
                line, = ax.plot([], [], **series_conf['plot_args'])
                self.lines[series_conf['line_key']] = line
                current_lines.append(line)

            ax.axhline(0, color='gray', linestyle='--', alpha=0.5)
            ax.legend(loc='upper right')

        # Drone UI subplot
        self.axes['drone_ui'] = self.fig.add_subplot(gs[0:2, 1])
        ax_drone_ui = self.axes['drone_ui']
        ax_drone_ui.set_title('Drone Status', fontsize=10)
        ax_drone_ui.set_xticks([])
        ax_drone_ui.set_yticks([])
        ax_drone_ui.axis('off')
        ax_drone_ui.set_aspect('equal', adjustable='box')
        ax_drone_ui.set_ylim(-1.3, 1.3)
        ax_drone_ui.set_xlim(-1.3, 1.3)

        self.throttle_display_shape = patches.Rectangle((-0.4, -0.4), 0.8, 0.8, color='lightgray', ec='black', lw=1)
        ax_drone_ui.add_patch(self.throttle_display_shape)
        self.throttle_display_text = ax_drone_ui.text(0, 0, f'Throttle:\n{int(self.latest_values["throttle"])}', ha='center', va='center', fontsize=10)

        motor_radius = 0.35
        motor_config_ui = { # Renamed to avoid conflict if 'motor_config' is used elsewhere
            'f_ccw': {'label': 'F_CCW', 'pos': (-0.8, 0.8)}, 'f_cw':  {'label': 'F_CW',  'pos': (0.8, 0.8)},
            'b_ccw': {'label': 'B_CCW', 'pos': (-0.8, -0.8)}, 'b_cw':  {'label': 'B_CW',  'pos': (0.8, -0.8)}
        }
        self.motor_display_shapes = {} # Ensure these are initialized here
        self.motor_display_texts = {}
        for key, m_config in motor_config_ui.items():
            shape = patches.Circle(m_config['pos'], radius=motor_radius, color='lightgray', ec='black', lw=1)
            self.motor_display_shapes[key] = shape
            ax_drone_ui.add_patch(shape)
            scalar_key = f"{key}_scalar"
            initial_value = self.latest_values.get(scalar_key, 0.0)
            text = ax_drone_ui.text(m_config['pos'][0], m_config['pos'][1], f"{m_config['label']}\n{initial_value:.2f}", ha='center', va='center', fontsize=8)
            self.motor_display_texts[key] = text

        # General plot settings
        time_format = mdates.DateFormatter('%H:%M:%S')
        if 'roll' in self.axes: # Apply to the last main telemetry plot's x-axis
             self.axes['roll'].xaxis.set_major_formatter(time_format)
        if 'yaw_diff' in self.axes: # Apply to yaw_diff plot's x-axis
             self.axes['yaw_diff'].xaxis.set_major_formatter(time_format)

        self.fig.suptitle(f'Drone Telemetry - {datetime.now().strftime("%H:%M:%S")}', fontsize=16)
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.subplots_adjust(hspace=0.25, wspace=0.25)

        self.end_time = datetime.now()
        self.start_time_dt = self.end_time - timedelta(seconds=self.window_size)

    def _start_new_log_file(self):
        """Handles the creation of a new log file for each test."""
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.close()

        self.current_test_number += 1
        now = datetime.now()
        self.current_log_filename = \
            f"{self.log_filename_base}_test{self.current_test_number}_{now.strftime('%Y-%m-%d_%H-%M-%S')}.csv"

        self.csv_file = open(self.current_log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.header)
        print(f"Starting Test {self.current_test_number}. Logging data to {self.current_log_filename}")
    
    def read_data(self):
        """Read and parse data from the socket stream (CSV format)."""
        try:
            line = self.stream.readline()
            if not line:
                return None
                
            line = line.strip()
            if not line: # Handle empty lines after strip
                return None

            values_str = line.split(',')
            
            # The drone sends 16 values. header_info has 17 entries because of our timestamp.
            # So we expect len(values_str) to be len(self.header_info) - 1
            expected_value_count = len(self.header_info) -1 # Exclude our timestamp
            if len(values_str) != expected_value_count:
                print(f"Malformed data line: Expected {expected_value_count} values, got {len(values_str)}. Line: '{line}'")
                return None

            # Log the raw data with a high-resolution timestamp
            log_timestamp = datetime.now().timestamp()
            # The first item in self.header is 'timestamp', so log_row uses its csv_col_name.
            log_row = [log_timestamp] + values_str
            if self.csv_writer and self.test_state in ["TEST_RUNNING", "WAITING_POST_TEST"]:
                self.csv_writer.writerow(log_row) # Use the generated self.header for column names

            vals = []
            for i, value_str in enumerate(values_str):
                # Find the corresponding header_info entry.
                # header_info[0] is 'Timestamp', csv_index starts from 0 for drone data.
                # So, header_info[i+1] corresponds to values_str[i]
                field_info = self.header_info[i+1]
                try:
                    vals.append(float(value_str))
                except ValueError:
                    print(f"Error converting value for field '{field_info['name']}': Could not convert '{value_str}' to float. Line: '{line}'")
                    return None

            current_time = datetime.now()
            self.timestamps.append(current_time)

            # Iterate through header_info to populate latest_values and deques
            # This loop structure is okay, as it uses csv_index to pick from `vals`
            for info in self.header_info:
                if info['csv_index'] is None: # Skip timestamp, already handled
                    continue

                # `vals` now contains successfully converted floats, in the same order as values_str
                raw_value = vals[info['csv_index']]

                # Update latest_values
                if info['latest_val_key']:
                    self.latest_values[info['latest_val_key']] = raw_value

                # Append to appropriate deque
                if info['data_category'] and info['data_key']:
                    data_deque_category = getattr(self, info['data_category']) # e.g., self.imu_data

                    # Special scaling for controller data for plotting compatibility
                    if info['data_category'] == 'ctrl_data':
                        if info['data_key'] == 'throttle':
                            scaled_value = (raw_value - 1000) / 1000.0
                        else: # roll, pitch, yaw
                            scaled_value = (raw_value - 1500) / 500.0
                        data_deque_category[info['data_key']].append(scaled_value)
                    else:
                        data_deque_category[info['data_key']].append(raw_value)

            # Return values for existing plotting (maintaining compatibility for now)
            # These are: current_time, yaw, pitch, roll, scaled_ctrl_roll, scaled_ctrl_pitch, scaled_ctrl_throttle, scaled_ctrl_yaw
            # Access them from the newly populated latest_values or scaled deques
            return (current_time,
                    self.latest_values['imu_yaw'], self.latest_values['imu_pitch'], self.latest_values['imu_roll'],
                    self.ctrl_data['roll'][-1],    # Last appended scaled value
                    self.ctrl_data['pitch'][-1],   # Last appended scaled value
                    self.ctrl_data['throttle'][-1],# Last appended scaled value
                    self.ctrl_data['yaw'][-1])     # Last appended scaled value
                    
        except Exception as e:
            print(f"Error in read_data: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def update_plot(self, frame):
        """Update the plot with new data."""
        # Check if figure is still valid
        if not plt.fignum_exists(self.fig.number):
            self.ani.event_source.stop()
            return []
            
        try:
            new_data = self.read_data() # Fetch new data
            perform_plot_update = False

            # ----- State Machine -----
            # State: IDLE - Waiting for a test to start
            if self.test_state == "IDLE":
                # Transition to TEST_RUNNING if throttle goes above threshold
                if new_data is not None and self.latest_values['throttle'] > self.throttle_deadzone_threshold:
                    self.test_state = "TEST_RUNNING"
                    self._start_new_log_file() # Initialize new log for this test
                    self.last_state_transition_time = time.time()

                    # Clear previous test data from deques for a fresh plot
                    self.timestamps.clear()
                    for category_dict in [self.imu_data, self.ctrl_data, self.derived_data, self.motor_scalar_data]:
                        for data_deque in category_dict.values():
                            data_deque.clear()
                    perform_plot_update = True # Start plotting the new test data
                else:
                    # Still IDLE, minimal work. Plot title might be updated at the end.
                    pass

            # State: TEST_RUNNING - Actively logging and plotting test data
            elif self.test_state == "TEST_RUNNING":
                if new_data is not None:
                    perform_plot_update = True
                    # Transition to WAITING_POST_TEST if throttle drops below threshold
                    if self.latest_values['throttle'] < self.throttle_deadzone_threshold:
                        self.test_state = "WAITING_POST_TEST"
                        self.last_state_transition_time = time.time()
                else:
                    # Data read failed during a test, potentially log this or handle
                    pass

            # State: WAITING_POST_TEST - Throttle has dropped, waiting for test to fully end or resume
            elif self.test_state == "WAITING_POST_TEST":
                if new_data is not None:
                    perform_plot_update = True # Continue plotting data during this phase

                # Transition back to TEST_RUNNING if throttle goes up again (test resumes)
                if self.latest_values['throttle'] > self.throttle_deadzone_threshold:
                    self.test_state = "TEST_RUNNING"
                    self.last_state_transition_time = time.time()
                # Transition to IDLE if post-test duration has elapsed
                elif time.time() - self.last_state_transition_time > self.post_test_wait_duration:
                    self.test_state = "IDLE"
                    self.last_state_transition_time = time.time()
                    print(f"Test {self.current_test_number} complete. Log file: {self.current_log_filename}")
                    # CSV log file is already being written to; it will be closed on cleanup or when a new test starts.

            # ----- Actual Plotting Logic -----
            if perform_plot_update and len(self.timestamps) > 1:
                current_time = self.timestamps[-1]
                start_time = current_time - timedelta(seconds=self.window_size)

                # Update plot data using self.lines and self.axes
                self.lines['yaw_imu'].set_data(self.timestamps, self.imu_data['yaw'])
                self.lines['pitch_imu'].set_data(self.timestamps, self.imu_data['pitch'])
                self.lines['roll_imu'].set_data(self.timestamps, self.imu_data['roll'])

                self.lines['yaw_ctrl'].set_data(self.timestamps, self.ctrl_data['yaw'])
                self.lines['pitch_ctrl'].set_data(self.timestamps, self.ctrl_data['pitch'])
                self.lines['roll_ctrl'].set_data(self.timestamps, self.ctrl_data['roll'])

                if 'yaw_diff_line' in self.lines:
                    self.lines['yaw_diff_line'].set_data(self.timestamps, self.derived_data['yaw_diff'])

                # Update x-axis limits for relevant axes
                for ax_key in ['yaw', 'pitch', 'roll', 'yaw_diff']: # yaw_diff shares x-axis with yaw
                    if ax_key in self.axes:
                        self.axes[ax_key].set_xlim(start_time, current_time)

                # Update Drone UI
                def get_color(value, min_val, max_val, start_color_rgb=(0.85, 0.85, 0.95), end_color_rgb=(0.1, 0.1, 0.6)):
                    if max_val == min_val: norm_value = 0.5
                    else: norm_value = max(0, min(1, (value - min_val) / (max_val - min_val)))
                    color = [start_color_rgb[i] + (end_color_rgb[i] - start_color_rgb[i]) * norm_value for i in range(3)]
                    return tuple(color)

                throttle_value = self.latest_values['throttle']
                self.throttle_display_shape.set_color(get_color(throttle_value, 1000, 2000))
                self.throttle_display_text.set_text(f'Throttle:\n{int(throttle_value)}')

                # Motor keys in motor_config_update should match keys in latest_values for scalars
                motor_config_update = {
                    'f_ccw': {'label': 'F_CCW', 'value': self.latest_values['f_ccw_scalar']},
                    'f_cw':  {'label': 'F_CW',  'value': self.latest_values['f_cw_scalar']},
                    'b_ccw': {'label': 'B_CCW', 'value': self.latest_values['b_ccw_scalar']},
                    'b_cw':  {'label': 'B_CW',  'value': self.latest_values['b_cw_scalar']}
                }
                for key, data in motor_config_update.items():
                    self.motor_display_shapes[key].set_color(get_color(data['value'], 0.0, 1.0))
                    self.motor_display_texts[key].set_text(f"{data['label']}\n{data['value']:.2f}")

            # Update title based on state AFTER state transitions and potential data processing
            if self.test_state == "IDLE":
                if self.current_test_number == 0:
                    self.fig.suptitle(f'Drone Telemetry - Waiting for Test #1 - {datetime.now().strftime("%H:%M:%S")}', fontsize=16)
                else:
                    self.fig.suptitle(f'Drone Telemetry - Waiting for next Test (Test #{self.current_test_number} ended) - {datetime.now().strftime("%H:%M:%S")}', fontsize=16)
            elif self.test_state == "TEST_RUNNING":
                self.fig.suptitle(f'Drone Telemetry - Test #{self.current_test_number} Running - {datetime.now().strftime("%H:%M:%S")}', fontsize=16)
            elif self.test_state == "WAITING_POST_TEST":
                self.fig.suptitle(f'Drone Telemetry - Test #{self.current_test_number} Finishing - {datetime.now().strftime("%H:%M:%S")}', fontsize=16)

        except Exception as e:
            print(f"Error in update_plot: {e}")
            import traceback
            traceback.print_exc()

        # Return all artists that need to be redrawn for blitting
        # Collect all line objects from self.lines
        artists = list(self.lines.values())
        # Add UI elements
        artists.append(self.throttle_display_shape)
        artists.append(self.throttle_display_text)
        artists.extend(self.motor_display_shapes.values())
        artists.extend(self.motor_display_texts.values())

        if self.fig._suptitle is not None:
            artists.append(self.fig._suptitle)

        # Filter out any None values just in case, though all should be initialized
        return [a for a in artists if a is not None]
    
    def init_plot(self):
        """Initialize the plot with empty data and prepare UI elements."""
        if not hasattr(self, 'fig') or not plt.fignum_exists(self.fig.number) or not self.lines:
            # If fig or lines aren't set up (e.g. setup_plot wasn't called or failed), return empty
            return []
            
        try:
            # Set empty data for all lines stored in self.lines
            for line_key in self.lines:
                self.lines[line_key].set_data([], [])

            if hasattr(self, 'fig') and self.fig: # Ensure fig exists before setting suptitle
                 self.fig.suptitle(f'Drone Telemetry - Waiting for Test #1 - {datetime.now().strftime("%H:%M:%S")}')
            
            # Initial state for UI elements is set in setup_plot.
            # For blitting, all animated artists must be returned by init_func.
            artists = list(self.lines.values()) # All plot lines
            
            # Add UI elements if they exist
            if hasattr(self, 'throttle_display_shape') and self.throttle_display_shape:
                artists.append(self.throttle_display_shape)
            if hasattr(self, 'throttle_display_text') and self.throttle_display_text:
                artists.append(self.throttle_display_text)

            if hasattr(self, 'motor_display_shapes'):
                artists.extend(self.motor_display_shapes.values())
            if hasattr(self, 'motor_display_texts'):
                artists.extend(self.motor_display_texts.values())
            
            if hasattr(self, 'fig') and self.fig and self.fig._suptitle is not None:
                artists.append(self.fig._suptitle)
                
            return [a for a in artists if a is not None] # Filter Nones
            
        except Exception as e:
            print(f"Error in init_plot: {e}")
            return []
    
    def run(self):
        """Start the data collection and plotting."""
        if not self.setup_connection():
            print("Failed to establish connection. Exiting...")
            return
        
        try:
            print(f"Starting data collection at {self.update_interval}ms interval. Close the plot window to exit...")
            print("Press Ctrl+C to stop")
            
            # Create a new figure and setup plot
            self.fig = plt.figure(figsize=(16, 9))
            self.setup_plot()
            
            # Set up the animation with a reference to keep it alive
            self.ani = FuncAnimation(
                self.fig, 
                self.update_plot,
                init_func=self.init_plot,
                interval=self.update_interval,
                blit=True,  # Enable blitting for better performance
                cache_frame_data=False
            )
            
            # Show the plot and block until the window is closed
            plt.tight_layout()
            plt.show(block=True)
            
        except KeyboardInterrupt:
            print("\nShutdown requested...")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.cleanup()
            plt.close('all')  # Ensure all plots are closed
    
    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, 'sock') and self.sock:
            self.sock.close()
        if hasattr(self, 'stream') and self.stream:
            self.stream.close()
        print("Connection closed.")

        # Close CSV file
        if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            if self.current_log_filename: # Check if a log file was actually created
                print(f"Data logged to {self.current_log_filename}")
            else:
                print("No active log file to save at cleanup.")


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Plot drone yaw data in real-time.')
    parser.add_argument('--host', type=str, default='192.168.76.215',
                       help='IP address of the drone (default: 192.168.76.215)')
    parser.add_argument('--port', type=int, default=23,
                       help='Port number (default: 23)')
    parser.add_argument('--window', type=int, default=10,
                       help='Time window to display in seconds (default: 10)')
    parser.add_argument('--interval', type=int, default=200,
                       help='Update interval in milliseconds (default: 200 for 5Hz)')
    return parser.parse_args()

def main():
    """Main function to run the application."""
    args = parse_arguments()
    
    print(f"Starting Drone Data Plotter")
    print(f"  Host: {args.host}")
    print(f"  Port: {args.port}")
    print(f"  Time Window: {args.window} seconds")
    print(f"  Update Interval: {args.interval} ms")
    print("-" * 40)
    
    plotter = DroneDataPlotter(
        host=args.host,
        port=args.port,
        window_size=args.window,
        update_interval=args.interval
    )
    
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
