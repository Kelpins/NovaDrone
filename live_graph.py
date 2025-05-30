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
        # IMU data
        self.yaw_data = deque(maxlen=self.max_data_points)
        self.pitch_data = deque(maxlen=self.max_data_points)
        self.roll_data = deque(maxlen=self.max_data_points)
        # Controller data
        self.ctrl_roll_data = deque(maxlen=self.max_data_points)
        self.ctrl_pitch_data = deque(maxlen=self.max_data_points)
        self.ctrl_throttle_data = deque(maxlen=self.max_data_points)
        self.ctrl_yaw_data = deque(maxlen=self.max_data_points)
        
        # Data fields from CSV (ensure all 17 are covered)
        # vals[0-2] are imu_roll, imu_pitch, imu_yaw (already covered by roll_data etc.)
        # vals[3-6] are cout_roll, cout_pitch, cout_throttle, cout_yaw (already covered by ctrl_roll_data etc.)
        self.pitch_diff_data = deque(maxlen=self.max_data_points) # vals[7] (index based on header: imu_roll,imu_pitch,imu_yaw,ctrl_roll,ctrl_pitch,ctrl_throttle,ctrl_yaw, pitch_diff, ...)
        self.roll_diff_data = deque(maxlen=self.max_data_points)  # vals[8]
        self.current_yaw_target_data = deque(maxlen=self.max_data_points) # vals[9]
        self.yaw_diff_data = deque(maxlen=self.max_data_points)    # vals[10]
        # vals[11] will be a new field, named 'data_field_11' for now
        self.data_field_11_data = deque(maxlen=self.max_data_points) # For vals[11]
        self.v_scalar_data = deque(maxlen=self.max_data_points)    # vals[12] (shifted from 11)
        
        # Motor scalars start from vals[13]
        self.f_cw_scalar_data = deque(maxlen=self.max_data_points) # vals[13]
        self.b_cw_scalar_data = deque(maxlen=self.max_data_points) # vals[14]
        self.f_ccw_scalar_data = deque(maxlen=self.max_data_points)# vals[15]
        self.b_ccw_scalar_data = deque(maxlen=self.max_data_points)# vals[16]

        # Attributes to store the latest values for UI update and general reference
        self.last_imu_roll = 0.0
        self.last_imu_pitch = 0.0
        self.last_imu_yaw = 0.0
        self.last_ctrl_roll = 0.0
        self.last_ctrl_pitch = 0.0
        self.last_throttle = 1000.0 # (from vals[5])
        self.last_ctrl_yaw = 0.0
        self.last_pitch_diff = 0.0
        self.last_roll_diff = 0.0
        self.last_current_yaw_target = 0.0
        self.last_yaw_diff = 0.0 # (from vals[10])
        self.last_v_scalar = 0.0 # (from vals[12])
        self.last_f_cw_scalar = 0.0 # (from vals[13])
        self.last_b_cw_scalar = 0.0 # (from vals[14])
        self.last_f_ccw_scalar = 0.0# (from vals[15])
        self.last_b_ccw_scalar = 0.0# (from vals[16])

        # Placeholders for UI elements
        self.throttle_display_shape = None
        self.throttle_display_text = None
        self.motor_display_shapes = {}  # e.g., {'f_cw': shape_object, ...}
        self.motor_display_texts = {}   # e.g., {'f_cw': text_object, ...}
        self.ln_yaw_diff = None # For Yaw Difference plot
        
        self.start_time = time.time()
        self.setup_plot()

        # CSV Logging Initialization
        self.log_filename = f"live_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        self.csv_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Updated header to include data_field_11 and reflect new v_scalar, motor scalar indexing
        self.header = ["timestamp", "imu_roll", "imu_pitch", "imu_yaw", 
                       "ctrl_roll", "ctrl_pitch", "ctrl_throttle", "ctrl_yaw", 
                       "pitch_diff", "roll_diff", "current_yaw_target", "yaw_diff", 
                       "data_field_11", "v_scalar", 
                       "f_cw_scalar", "b_cw_scalar", "f_ccw_scalar", "b_ccw_scalar"]
        self.csv_writer.writerow(self.header)
        print(f"Logging data to {self.log_filename}")

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
        """Initialize the matplotlib plot with three subplots for yaw, pitch, and roll."""
        # Configure plot style
        plt.style.use('default')
        plt.rcParams.update({
            'axes.grid': True,
            'grid.alpha': 0.3,
            'axes.edgecolor': '#333333',
            'axes.labelcolor': '#333333',
            'text.color': '#333333',
            'xtick.color': '#333333',
            'ytick.color': '#333333',
            'figure.facecolor': 'white',
        })
        
        # Create subplots with new layout
        self.fig = plt.figure(figsize=(16, 9)) 
        # Use width_ratios=[3,1] to give more space to data plots as per latest plan
        gs = self.fig.add_gridspec(3, 2, height_ratios=[1, 1, 1], width_ratios=[3, 1]) 

        self.ax_yaw = self.fig.add_subplot(gs[0, 0])
        self.ax_pitch = self.fig.add_subplot(gs[1, 0], sharex=self.ax_yaw)
        self.ax_roll = self.fig.add_subplot(gs[2, 0], sharex=self.ax_yaw)
        # self.ax_throttle (line plot) is removed.

        # Drone UI subplot (top-right, spanning two rows of the right column)
        self.ax_drone_ui = self.fig.add_subplot(gs[0:2, 1]) # Corrected rowspan to gs[0:2, 1] for clarity
        self.ax_drone_ui.set_title('Drone Status', fontsize=10)
        self.ax_drone_ui.set_xticks([])
        self.ax_drone_ui.set_yticks([])
        self.ax_drone_ui.axis('off') 
        self.ax_drone_ui.set_aspect('equal', adjustable='box')
        self.ax_drone_ui.set_ylim(-1.3, 1.3) 
        self.ax_drone_ui.set_xlim(-1.3, 1.3)

        # Draw Drone UI Elements (initial state)
        # Central Square (Throttle)
        # Adjusted size and position for clarity
        self.throttle_display_shape = patches.Rectangle((-0.4, -0.4), 0.8, 0.8, color='lightgray', ec='black', lw=1)
        self.ax_drone_ui.add_patch(self.throttle_display_shape)
        self.throttle_display_text = self.ax_drone_ui.text(0, 0, f'Throttle:\n{int(self.last_throttle)}', ha='center', va='center', fontsize=10)

        # Motor Circles (Propellers)
        motor_radius = 0.35 # Slightly larger radius
        # Adjusted positions for better spacing
        motor_config = {
            'f_ccw': {'label': 'F_CCW', 'pos': (-0.8, 0.8)}, # Top-left
            'f_cw':  {'label': 'F_CW',  'pos': (0.8, 0.8)},  # Top-right
            'b_ccw': {'label': 'B_CCW', 'pos': (-0.8, -0.8)},# Bottom-left
            'b_cw':  {'label': 'B_CW',  'pos': (0.8, -0.8)}  # Bottom-right
        }

        for key, config_item in motor_config.items():
            shape = patches.Circle(config_item['pos'], radius=motor_radius, color='lightgray', ec='black', lw=1)
            self.motor_display_shapes[key] = shape
            self.ax_drone_ui.add_patch(shape)
            initial_value = getattr(self, f"last_{key}_scalar", 0.0)
            text = self.ax_drone_ui.text(config_item['pos'][0], config_item['pos'][1], f"{config_item['label']}\n{initial_value:.2f}", 
                                         ha='center', va='center', fontsize=8)
            self.motor_display_texts[key] = text
        
        # Yaw plot (-180° to 180°)
        self.ln_yaw, = self.ax_yaw.plot([], [], 'b-', lw=1.5, label='Yaw')
        self.ln_yaw_ctrl, = self.ax_yaw.plot([], [], 'b--', lw=1, alpha=0.7, label='Yaw (Ctrl)')
        self.ax_yaw.set_ylabel('Yaw')
        self.ax_yaw.set_ylim(-200, 200)
        self.ax_yaw.set_yticks(range(-180, 181, 45))
        self.ax_yaw.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_yaw.legend(loc='upper right')
        
        # Pitch plot (-90° to 90°)
        self.ln_pitch, = self.ax_pitch.plot([], [], 'g-', lw=1.5, label='Pitch')
        self.ln_pitch_ctrl, = self.ax_pitch.plot([], [], 'g--', lw=1, alpha=0.7, label='Pitch (Ctrl)')
        self.ax_pitch.set_ylabel('Pitch')
        self.ax_pitch.set_ylim(-90, 90)
        self.ax_pitch.set_yticks(range(-90, 91, 30))
        self.ax_pitch.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_pitch.legend(loc='upper right')
        
        # Roll plot (-45° to 45°)
        self.ln_roll, = self.ax_roll.plot([], [], 'r-', lw=1.5, label='Roll')
        self.ln_roll_ctrl, = self.ax_roll.plot([], [], 'r--', lw=1, alpha=0.7, label='Roll (Ctrl)')
        self.ax_roll.set_ylabel('Roll')
        self.ax_roll.set_ylim(-60, 60)
        self.ax_roll.set_yticks(range(-45, 46, 15))
        self.ax_roll.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_roll.legend(loc='upper right')
        self.ax_roll.set_xlabel('Time') # Add X label to the bottom plot of the left column
        
        # Yaw Difference plot (bottom-right)
        self.ax_yaw_diff = self.fig.add_subplot(gs[2,1], sharex=self.ax_yaw)
        self.ln_yaw_diff, = self.ax_yaw_diff.plot([], [], 'c-', lw=1.5, label='Yaw Diff') # Cyan color
        self.ax_yaw_diff.set_ylabel('Yaw Diff (deg)')
        self.ax_yaw_diff.set_ylim(-45, 45) # Example range, adjust as needed
        self.ax_yaw_diff.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_yaw_diff.legend(loc='upper right')
        self.ax_yaw_diff.set_xlabel('Time') # X label for this plot too
        
        # Configure time axis (applies to all plots sharing self.ax_yaw's x-axis)
        self.ax_roll.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
        # Ensure the new ax_yaw_diff also gets the formatter if not automatically shared
        self.ax_yaw_diff.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
        
        # Set window title and adjust layout
        self.fig.suptitle(f'Drone Telemetry - {datetime.now().strftime("%H:%M:%S")}', fontsize=16)
        plt.tight_layout(rect=[0, 0, 1, 0.96]) 
        plt.subplots_adjust(hspace=0.25, wspace=0.25) # Adjusted spacing
        
        # Set initial time window
        self.end_time = datetime.now()
        self.start_time_dt = self.end_time - timedelta(seconds=self.window_size)
    
    def read_data(self):
        """Read and parse data from the socket stream (CSV format)."""
        try:
            line = self.stream.readline()
            if not line:
                # print("Connection closed or no data received.") # Less verbose
                return None
                
            line = line.strip()
            if not line: # Handle empty lines after strip
                return None

            values_str = line.split(',')
            
            if len(values_str) != 16:  # Updated to expect 16 values
                print(f"Malformed data line: Expected 16 values, got {len(values_str)}. Line: '{line}'")
                return None

            # Log the raw data with a high-resolution timestamp
            log_timestamp = datetime.now().timestamp()
            log_row = [log_timestamp] + values_str
            self.csv_writer.writerow(log_row)
                
            try:
                # Convert all values to float
                vals = [float(v) for v in values_str]
            except ValueError as e:
                print(f"Error converting data to float: {e}. Line: '{line}'")
                return None

            current_time = datetime.now()
            
            # Parse all 16 data fields from vals[0] to vals[16]
            # and assign to self.last_... attributes
            self.last_imu_roll = vals[0]
            self.last_imu_pitch = vals[1]
            self.last_imu_yaw = vals[2]
            
            self.last_ctrl_roll = vals[3]
            self.last_ctrl_pitch = vals[4]
            self.last_throttle = vals[5] # Used by UI
            self.last_ctrl_yaw = vals[6]
            
            self.last_pitch_diff = vals[7]
            self.last_roll_diff = vals[8]
            self.last_current_yaw_target = vals[9]
            self.last_yaw_diff = vals[10] # Used by Yaw Diff Plot
            
            self.last_v_scalar = vals[11]    # v_scalar is now vals[11]
            
            self.last_f_cw_scalar = vals[12] # Used by UI
            self.last_b_cw_scalar = vals[13] # Used by UI
            self.last_f_ccw_scalar = vals[14]# Used by UI
            self.last_b_ccw_scalar = vals[15]# Used by UI

            # Scale controller outputs for plotting visualization (using self.last_... attributes)
            scaled_cout_roll = (self.last_ctrl_roll - 1500) / 500
            scaled_cout_pitch = (self.last_ctrl_pitch - 1500) / 500
            scaled_cout_yaw = (self.last_ctrl_yaw - 1500) / 500
            scaled_cout_throttle = (self.last_throttle - 1000) / 1000.0

            # Append all data to deques using self.last_... attributes
            self.timestamps.append(current_time)
            # IMU data
            self.roll_data.append(self.last_imu_roll)
            self.pitch_data.append(self.last_imu_pitch)
            self.yaw_data.append(self.last_imu_yaw)
            # Controller data (scaled for existing plots)
            self.ctrl_roll_data.append(scaled_cout_roll)
            self.ctrl_pitch_data.append(scaled_cout_pitch)
            self.ctrl_throttle_data.append(scaled_cout_throttle)
            self.ctrl_yaw_data.append(scaled_cout_yaw)
            # Additional data
            self.pitch_diff_data.append(self.last_pitch_diff)
            self.roll_diff_data.append(self.last_roll_diff)
            self.current_yaw_target_data.append(self.last_current_yaw_target)
            self.yaw_diff_data.append(self.last_yaw_diff)
            self.v_scalar_data.append(self.last_v_scalar) # Appending self.last_v_scalar (from vals[12])
            self.f_cw_scalar_data.append(self.last_f_cw_scalar)
            self.b_cw_scalar_data.append(self.last_b_cw_scalar)
            self.f_ccw_scalar_data.append(self.last_f_ccw_scalar)
            self.b_ccw_scalar_data.append(self.last_b_ccw_scalar)
            
            # Return values for existing plotting (maintaining compatibility)
            # Ensure these returned values are consistent with what the plot expects (e.g. raw vs scaled)
            return (current_time, self.last_imu_yaw, self.last_imu_pitch, self.last_imu_roll,
                   scaled_cout_roll, scaled_cout_pitch, 
                   scaled_cout_throttle, scaled_cout_yaw)
                    
        except Exception as e:
            print(f"Error in read_data: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def update_plot(self, frame):
        """Update the plot with new data."""
        try:
            # Read and process data
            new_data = self.read_data() # Fetch new data
            if new_data is not None:
                # Only update plot if we have enough data points
                if len(self.timestamps) > 1:
                    current_time = self.timestamps[-1]
                    start_time = current_time - timedelta(seconds=self.window_size)
                    
                    # Update plot data - IMU
                    self.ln_yaw.set_data(self.timestamps, self.yaw_data)
                    self.ln_pitch.set_data(self.timestamps, self.pitch_data)
                    self.ln_roll.set_data(self.timestamps, self.roll_data)
                    
                    # Update plot data - Controller
                    self.ln_yaw_ctrl.set_data(self.timestamps, self.ctrl_yaw_data)
                    self.ln_pitch_ctrl.set_data(self.timestamps, self.ctrl_pitch_data)
                    self.ln_roll_ctrl.set_data(self.timestamps, self.ctrl_roll_data)
                    
                    # Update Yaw Difference plot
                    self.ln_yaw_diff.set_data(self.timestamps, self.yaw_diff_data)
                    
                    # Update time window for data subplots (including new yaw_diff plot due to sharex)
                    # No need to explicitly set_xlim for self.ax_yaw_diff if sharex is working as expected.
                    # The primary axis (self.ax_yaw) along with its shared axes (self.ax_pitch, self.ax_roll, self.ax_yaw_diff)
                    # will have their limits updated.
                    for ax in [self.ax_yaw, self.ax_pitch, self.ax_roll]: # ax_yaw_diff is implicitly handled by sharex=ax_yaw
                        ax.set_xlim(start_time, current_time)
                    
                    # Update Drone UI
                    # Helper function for color interpolation (can be defined inside or outside class method)
                    def get_color(value, min_val, max_val, start_color_rgb=(0.85, 0.85, 0.95), end_color_rgb=(0.1, 0.1, 0.6)):
                        if max_val == min_val: norm_value = 0.5 # Avoid division by zero
                        else: norm_value = max(0, min(1, (value - min_val) / (max_val - min_val)))
                        # Interpolate RGB values
                        color = [start_color_rgb[i] + (end_color_rgb[i] - start_color_rgb[i]) * norm_value for i in range(3)]
                        return tuple(color)

                    # Update Throttle Display
                    throttle_value = self.last_throttle
                    self.throttle_display_shape.set_color(get_color(throttle_value, 1000, 2000))
                    self.throttle_display_text.set_text(f'Throttle:\n{int(throttle_value)}')

                    # Update Motor Scalar Displays
                    # Using the motor_config from setup_plot to get labels
                    motor_config_update = {
                        'f_ccw': {'label': 'F_CCW', 'value': self.last_f_ccw_scalar},
                        'f_cw':  {'label': 'F_CW',  'value': self.last_f_cw_scalar},
                        'b_ccw': {'label': 'B_CCW', 'value': self.last_b_ccw_scalar},
                        'b_cw':  {'label': 'B_CW',  'value': self.last_b_cw_scalar}
                    }
                    for key, data in motor_config_update.items():
                        self.motor_display_shapes[key].set_color(get_color(data['value'], 0.0, 1.0)) # Motor scalars are 0-1
                        self.motor_display_texts[key].set_text(f"{data['label']}\n{data['value']:.2f}")
                    
                    # Update main figure suptitle with current time
                    self.fig.suptitle(f'Drone Telemetry - {current_time.strftime("%H:%M:%S")}', fontsize=16)
                    
        except Exception as e:
            print(f"Error in update_plot: {e}")
            import traceback
            traceback.print_exc()

        # Return all artists that need to be redrawn for blitting
        artists = [self.ln_yaw, self.ln_pitch, self.ln_roll, 
                   self.ln_yaw_ctrl, self.ln_pitch_ctrl, self.ln_roll_ctrl,
                   self.throttle_display_shape, self.throttle_display_text,
                   self.ln_yaw_diff] # Added Yaw Diff line
        artists.extend(self.motor_display_shapes.values())
        artists.extend(self.motor_display_texts.values())
        return artists
    
    def init_plot(self):
        """Initialize the plot with empty data and prepare UI elements."""
        # IMU data lines
        self.ln_yaw.set_data([], [])
        self.ln_pitch.set_data([], [])
        self.ln_roll.set_data([], [])
        
        # Controller data lines
        self.ln_yaw_ctrl.set_data([], [])
        self.ln_pitch_ctrl.set_data([], [])
        self.ln_roll_ctrl.set_data([], [])
        
        # Yaw Difference line
        if self.ln_yaw_diff: # Check if it's initialized
            self.ln_yaw_diff.set_data([], [])
        
        # Initial state for UI elements is set in setup_plot.
        # For blitting, all animated artists must be returned by init_func.
        artists = [self.ln_yaw, self.ln_pitch, self.ln_roll,
                   self.ln_yaw_ctrl, self.ln_pitch_ctrl, self.ln_roll_ctrl,
                   self.throttle_display_shape, self.throttle_display_text,
                   self.ln_yaw_diff] # Added Yaw Diff line
        artists.extend(self.motor_display_shapes.values())
        artists.extend(self.motor_display_texts.values())
        return artists
    
    def run(self):
        """Start the data collection and plotting."""
        if not self.setup_connection():
            print("Failed to establish connection. Exiting...")
            return
        
        try:
            print(f"Starting data collection at {self.update_interval}ms interval. Close the plot window to exit...")
            print("Press Ctrl+C to stop")
            
            # Set up the animation
            self.ani = FuncAnimation(
                self.fig, 
                self.update_plot,
                init_func=self.init_plot,
                interval=self.update_interval,
                blit=True,  # Enable blitting for better performance
                cache_frame_data=False
            )
            
            # Use a more responsive backend if available
            plt.ion()  # Turn on interactive mode
            plt.show(block=True)  # Block until window is closed
            
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
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            print(f"Data logged to {self.log_filename}")

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
