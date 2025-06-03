import csv
import matplotlib.pyplot as plt
from matplotlib import gridspec
from datetime import datetime, timedelta
from collections import deque
import os
import numpy as np

class LogDataPlotter:
    def __init__(self, log_file_path, window_size=30):
        """
        Initialize the LogDataPlotter.
        
        Args:
            log_file_path (str): Path to the log file
            window_size (int): Time window to display in seconds
        """
        self.log_file_path = log_file_path
        self.window_size = window_size
        self.max_data_points = 10000
        
        # Initialize data storage
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
        
        # Motor scalar data
        self.v_scalar_data = deque(maxlen=self.max_data_points)
        self.f_cw_scalar_data = deque(maxlen=self.max_data_points)
        self.b_cw_scalar_data = deque(maxlen=self.max_data_points)
        self.f_ccw_scalar_data = deque(maxlen=self.max_data_points)
        self.b_ccw_scalar_data = deque(maxlen=self.max_data_points)
        
        self.start_time = None
        self.setup_plot()
    
    def parse_log_file(self):
        """Parse the CSV log file and extract data."""
        try:
            with open(self.log_file_path, 'r') as f:
                reader = csv.reader(f)
                headers = next(reader)  # Get header row
                
                for row in reader:
                    if len(row) < len(headers):  # Skip incomplete rows
                        continue
                        
                    try:
                        # Extract timestamp (first value)
                        timestamp = float(row[0])
                        
                        # Extract IMU data (first 3 values after timestamp)
                        imu_yaw = float(row[1])
                        imu_pitch = float(row[2])
                        imu_roll = float(row[3])
                        
                        # Extract controller outputs (next 4 values)
                        cout_roll = float(row[4])
                        cout_pitch = float(row[5])
                        cout_throttle = float(row[6])
                        cout_yaw = float(row[7])
                        
                        # Extract difference data (next 4 values)
                        if len(row) > 8:
                            pitch_diff = float(row[8])
                            roll_diff = float(row[9])
                            
                            # Extract motor scalar data (remaining values)
                            v_scalar = float(row[12]) if len(row) > 12 else 0
                            f_cw_scalar = float(row[13]) if len(row) > 13 else 0
                            b_cw_scalar = float(row[14]) if len(row) > 14 else 0
                            f_ccw_scalar = float(row[15]) if len(row) > 15 else 0
                            b_ccw_scalar = float(row[16]) if len(row) > 16 else 0
                        
                        # Scale controller outputs for better visualization
                        scaled_cout_roll = (cout_roll - 1500) / 500  # Scale to ±1
                        scaled_cout_pitch = (cout_pitch - 1500) / 500
                        scaled_cout_yaw = (cout_yaw - 1500) / 500
                        
                        # Store data
                        self.timestamps.append(timestamp / 1000.0)  # Convert to seconds
                        
                        # IMU data
                        self.yaw_data.append(imu_yaw)
                        self.pitch_data.append(imu_pitch)
                        self.roll_data.append(imu_roll)
                        
                        # Controller data
                        self.ctrl_roll_data.append(scaled_cout_roll)
                        self.ctrl_pitch_data.append(scaled_cout_pitch)
                        self.ctrl_yaw_data.append(scaled_cout_yaw)
                        self.ctrl_throttle_data.append((cout_throttle - 1000) / 1000.0)  # Scale to 0-1
                        
                        # Difference data
                        if len(row) > 8:
                            self.pitch_diff_data.append(pitch_diff)
                            self.roll_diff_data.append(roll_diff)
                            
                            # Motor scalar data
                            self.v_scalar_data.append(v_scalar)
                            self.f_cw_scalar_data.append(f_cw_scalar)
                            self.b_cw_scalar_data.append(b_cw_scalar)
                            self.f_ccw_scalar_data.append(f_ccw_scalar)
                            self.b_ccw_scalar_data.append(b_ccw_scalar)
                        
                    except (ValueError, IndexError) as e:
                        print(f"Warning: Could not parse line: {row}")
                        print(f"Error: {str(e)}")
                        continue
            
            if not self.timestamps:
                print("No valid data points found.")
                return False
                
            # Set start time based on first timestamp
            self.start_time = min(self.timestamps)
            return True
            
        except Exception as e:
            print(f"Error reading log file: {e}")
            return False
    
    def setup_plot(self):
        """Initialize the matplotlib plot with multiple subplots."""
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
            'figure.figsize': (16, 20),
            'font.size': 9,
        })
        
        # Create figure with multiple subplots
        self.fig = plt.figure()
        gs = gridspec.GridSpec(6, 2, height_ratios=[1, 1, 1, 1, 1, 1], width_ratios=[1, 1])
        
        # Orientation plots (left column)
        self.ax_yaw = self.fig.add_subplot(gs[0, 0])
        self.ax_pitch = self.fig.add_subplot(gs[1, 0], sharex=self.ax_yaw)
        self.ax_roll = self.fig.add_subplot(gs[2, 0], sharex=self.ax_yaw)
        self.ax_throttle = self.fig.add_subplot(gs[3, 0], sharex=self.ax_yaw)
        
        # Difference plots (right column, top)
        self.ax_diff = self.fig.add_subplot(gs[0:2, 1], sharex=self.ax_yaw)
        
        # Motor scalar plots (right column, middle)
        self.ax_motors = self.fig.add_subplot(gs[2:4, 1], sharex=self.ax_yaw)
        
        # Scatter plots for controller vs IMU (right column, bottom)
        self.ax_scatter = self.fig.add_subplot(gs[4:6, :])
        
        # Yaw plot
        self.ln_yaw, = self.ax_yaw.plot([], [], 'b-', lw=1, label='Yaw (IMU)')
        self.ln_yaw_ctrl, = self.ax_yaw.plot([], [], 'b--', lw=1, alpha=0.7, label='Yaw (Ctrl)')
        self.ax_yaw.set_ylabel('Yaw (deg)')
        self.ax_yaw.set_ylim(-200, 200)
        self.ax_yaw.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_yaw.legend(loc='upper right', fontsize=8)
        
        # Pitch plot
        self.ln_pitch, = self.ax_pitch.plot([], [], 'g-', lw=1, label='Pitch (IMU)')
        self.ln_pitch_ctrl, = self.ax_pitch.plot([], [], 'g--', lw=1, alpha=0.7, label='Pitch (Ctrl)')
        self.ax_pitch.set_ylabel('Pitch (deg)')
        self.ax_pitch.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_pitch.legend(loc='upper right', fontsize=8)
        
        # Roll plot
        self.ln_roll, = self.ax_roll.plot([], [], 'r-', lw=1, label='Roll (IMU)')
        self.ln_roll_ctrl, = self.ax_roll.plot([], [], 'r--', lw=1, alpha=0.7, label='Roll (Ctrl)')
        self.ax_roll.set_ylabel('Roll (deg)')
        self.ax_roll.axhline(0, color='gray', linestyle='--', alpha=0.5)
        self.ax_roll.legend(loc='upper right', fontsize=8)
        
        # Throttle plot
        self.ln_throttle, = self.ax_throttle.plot([], [], 'm-', lw=1, label='Throttle')
        self.ax_throttle.set_ylabel('Throttle (0-1)')
        self.ax_throttle.set_ylim(0, 1)
        self.ax_throttle.set_yticks(np.linspace(0, 1, 6))
        self.ax_throttle.set_xlabel('Time (s)')
        self.ax_throttle.legend(loc='upper right', fontsize=8)
        
        # Motor scalars plot
        self.ln_v_scalar, = self.ax_motors.plot([], [], 'k-', lw=1, label='V Scalar')
        self.ln_f_cw, = self.ax_motors.plot([], [], 'b-', lw=1, label='Front CW')
        self.ln_b_cw, = self.ax_motors.plot([], [], 'g-', lw=1, label='Back CW')
        self.ln_f_ccw, = self.ax_motors.plot([], [], 'r-', lw=1, label='Front CCW')
        self.ln_b_ccw, = self.ax_motors.plot([], [], 'm-', lw=1, label='Back CCW')
        self.ax_motors.set_title('Motor Scalars')
        self.ax_motors.set_ylabel('Scalar Value')
        self.ax_motors.set_ylim(0, 1.1)
        self.ax_motors.legend(loc='upper right', fontsize=7, ncol=2)
        
        # Adjust layout
        plt.tight_layout()
        plt.subplots_adjust(hspace=0.5, wspace=0.3)
        
        # Set window title and adjust layout
        self.fig.suptitle(f'Drone Orientation - {datetime.now().strftime("%H:%M:%S")}')
        plt.tight_layout()
        plt.subplots_adjust(hspace=0.25)
    
    def update_plot(self):
        """Update the plot with the parsed data."""
        if not self.timestamps:
            return []
        
        # Convert timestamps to relative seconds
        timestamps = [t - self.start_time for t in self.timestamps]
        
        # Update orientation plots
        self.ln_yaw.set_data(timestamps, self.yaw_data)
        self.ln_yaw_ctrl.set_data(timestamps, self.ctrl_yaw_data)
        self.ln_pitch.set_data(timestamps, self.pitch_data)
        self.ln_pitch_ctrl.set_data(timestamps, self.ctrl_pitch_data)
        self.ln_roll.set_data(timestamps, self.roll_data)
        self.ln_roll_ctrl.set_data(timestamps, self.ctrl_roll_data)
        self.ln_throttle.set_data(timestamps, self.ctrl_throttle_data)
        
        # Update motor scalar plots
        self.ln_v_scalar.set_data(timestamps, self.v_scalar_data)
        self.ln_f_cw.set_data(timestamps, self.f_cw_scalar_data)
        self.ln_b_cw.set_data(timestamps, self.b_cw_scalar_data)
        self.ln_f_ccw.set_data(timestamps, self.f_ccw_scalar_data)
        self.ln_b_ccw.set_data(timestamps, self.b_ccw_scalar_data)
        
        
        # Set x-axis limits based on window size
        if timestamps:
            x_min = max(0, timestamps[-1] - self.window_size)
            x_max = max(self.window_size, timestamps[-1] + 1)
            
            # Set xlim for all time-based plots
            for ax in [self.ax_yaw, self.ax_pitch, self.ax_roll, 
                      self.ax_throttle, self.ax_diff, self.ax_motors]:
                ax.set_xlim(x_min, x_max)
            
            # Auto-scale y-axes for orientation plots
            for ax in [self.ax_yaw, self.ax_pitch, self.ax_roll, self.ax_diff]:
                ax.relim()
                ax.autoscale_view()
        
        # Collect all artists to return
        artists = [
            self.ln_yaw, self.ln_yaw_ctrl,
            self.ln_pitch, self.ln_pitch_ctrl,
            self.ln_roll, self.ln_roll_ctrl,
            self.ln_throttle,
            self.ln_v_scalar, self.ln_f_cw, self.ln_b_cw, self.ln_f_ccw, self.ln_b_ccw,
        ]
        
        return artists

def main():
    import argparse
    import os
    
    parser = argparse.ArgumentParser(description='Plot drone data from log file')
    parser.add_argument('log_file', help='Path to the log file')
    parser.add_argument('--window', type=int, default=30, help='Time window to display in seconds (default: 30)')
    args = parser.parse_args()
    
    if not os.path.isfile(args.log_file):
        print(f"Error: File '{args.log_file}' not found.")
        return
    
    plotter = LogDataPlotter(args.log_file, args.window)
    
    if plotter.parse_log_file():
        plotter.update_plot()
        plt.show()
    else:
        print("Failed to parse log file.")

if __name__ == "__main__":
    main()
