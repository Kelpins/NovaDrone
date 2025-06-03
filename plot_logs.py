import csv
import matplotlib.pyplot as plt
from matplotlib import gridspec
from datetime import datetime, timedelta
from collections import deque
import os
import numpy as np
import sys

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
        self.data_queues = {} # Stores deques for each column
        
        self.start_time = None
        # Defer setup_plot call until after parse_log_file in main or when data is loaded
        # self.setup_plot()
    
    def parse_log_file(self):
        """Parse the CSV log file and extract data."""
        try:
            with open(self.log_file_path, 'r') as f:
                reader = csv.reader(f)
                headers = next(reader)  # Get header row

                # Initialize deques for each header
                for header in headers:
                    if header != 'timestamp': # timestamp is handled separately
                        self.data_queues[header] = deque(maxlen=self.max_data_points)

                scaling_params = {
                    'ctrl_roll': (1500, 500),      # (value - 1500) / 500
                    'ctrl_pitch': (1500, 500),     # (value - 1500) / 500
                    'ctrl_yaw': (1500, 500),       # (value - 1500) / 500
                    'ctrl_throttle': (1000, 1000),  # (value - 1000) / 1000 -> 0.0 to 1.0 for 1000-2000 range
                    'current_yaw_target': (0, 1)    # Already in degrees, no scaling needed
                }
                
                for row_idx, row in enumerate(reader):
                    # Warn once if row is short, but proceed with NaN padding
                    if len(row) < len(headers):
                        print(f"Warning: Row {row_idx+2} is short ({len(row)} columns, expected {len(headers)}). Missing values will be NaN.")

                    for i, header in enumerate(headers):
                        if i >= len(row): # Current header is beyond the number of columns in this specific row
                            if header != 'timestamp': # Timestamps cannot be NaN from missing columns
                                self.data_queues[header].append(np.nan)
                                print(f"Warning: Missing data for '{header}' in row {row_idx+2}. Using NaN.")
                            # If timestamp column itself is missing past len(row), it will be handled by critical check if not populated.
                            continue # Move to the next header

                        value_str = row[i].strip()

                        if header == 'timestamp':
                            try:
                                timestamp = float(value_str) / 1000.0  # Convert to seconds
                                self.timestamps.append(timestamp)
                            except ValueError:
                                raise ValueError(f"Error: Invalid or missing timestamp in row {row_idx+2}: '{value_str}'")
                        else: # For other data columns
                            if value_str == "":
                                self.data_queues[header].append(np.nan)
                                print(f"Warning: Empty value for '{header}' in row {row_idx+2}. Using NaN.")
                            else:
                                try:
                                    value = float(value_str)
                                    # Apply scaling if header is in scaling_params
                                    if header in scaling_params:
                                        offset, divisor = scaling_params[header]
                                        value = (value - offset) / divisor
                                    self.data_queues[header].append(value)
                                except ValueError:
                                    self.data_queues[header].append(np.nan)
                                    print(f"Warning: Non-numeric value for '{header}' in row {row_idx+2}: '{value_str}'. Using NaN.")
            
            if not self.timestamps: # If after all rows, no valid timestamps were parsed.
                # This could happen if all timestamp fields were invalid or missing.
                print("Error: No valid timestamps found in the log file. Cannot proceed.")
                return False
                
            # Set start time based on first timestamp
            if self.timestamps: # Ensure timestamps is not empty
                self.start_time = min(self.timestamps)
            return True
            
        except Exception as e:
            print(f"Error reading log file: {e}")
            return False
    
    def setup_plot(self):
        """Initialize the matplotlib plot with multiple subplots dynamically."""
        # Ensure data is parsed, so self.data_queues is populated
        if not self.data_queues and not self.timestamps: # Check if parsing happened
             # Attempt to parse if not already done.
             # This might be problematic if __init__ expects setup_plot to run
             # For now, we assume parse_log_file is called before plotting by the user of the class.
             print("Warning: setup_plot called before parse_log_file or no data loaded. Plot may be empty.")
             # Or, alternatively, call self.parse_log_file() here if appropriate for the class design

        self.plot_lines = {}
        self.active_axes = [] # To keep track of axes that actually get plots

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
        gs = gridspec.GridSpec(4, 2, height_ratios=[1, 1, 1, 1], width_ratios=[1, 1])
        
        # Define base axes, these might not all be used if data is missing
        # We will create them but only configure them if they get plots
        ax_yaw_candidate = self.fig.add_subplot(gs[0, 0])
        ax_pitch_candidate = self.fig.add_subplot(gs[1, 0], sharex=ax_yaw_candidate)
        ax_roll_candidate = self.fig.add_subplot(gs[2, 0], sharex=ax_yaw_candidate)
        ax_throttle_candidate = self.fig.add_subplot(gs[3, 0], sharex=ax_yaw_candidate)
        ax_diff_candidate = self.fig.add_subplot(gs[0:2, 1], sharex=ax_yaw_candidate)
        ax_motors_candidate = self.fig.add_subplot(gs[2:4, 1], sharex=ax_yaw_candidate)

        # Store all candidate axes to manage them later (e.g., for deletion if unused)
        self.candidate_axes_map = {
            'ax_yaw': ax_yaw_candidate, 'ax_pitch': ax_pitch_candidate,
            'ax_roll': ax_roll_candidate, 'ax_throttle': ax_throttle_candidate,
            'ax_diff': ax_diff_candidate, 'ax_motors': ax_motors_candidate
        }
        
        # Define plot configurations
        # Each dict: ax_attr, ax_candidate, lines_def (key, style, label, alpha), ylabel, [ylim], [title], [extra_config_func]
        plot_configs = [
            {
                'ax_attr': 'ax_yaw', 'ax_cand': ax_yaw_candidate,
                'lines_def': [('imu_yaw', 'b-', 'Yaw (IMU)', 1.0), ('ctrl_yaw', 'b--', 'Yaw (Ctrl)', 0.7)],
                'ylabel': 'Yaw (deg)', 'ylim': (-200, 200),
                'extra_ax_config_func': lambda ax: ax.axhline(0, color='gray', linestyle='--', alpha=0.5)
            },
            {
                'ax_attr': 'ax_pitch', 'ax_cand': ax_pitch_candidate,
                'lines_def': [('imu_pitch', 'g-', 'Pitch (IMU)', 1.0), ('ctrl_pitch', 'g--', 'Pitch (Ctrl)', 0.7)],
                'ylabel': 'Pitch (deg)',
                'extra_ax_config_func': lambda ax: ax.axhline(0, color='gray', linestyle='--', alpha=0.5)
            },
            {
                'ax_attr': 'ax_roll', 'ax_cand': ax_roll_candidate,
                'lines_def': [('imu_roll', 'r-', 'Roll (IMU)', 1.0), ('ctrl_roll', 'r--', 'Roll (Ctrl)', 0.7)],
                'ylabel': 'Roll (deg)',
                'extra_ax_config_func': lambda ax: ax.axhline(0, color='gray', linestyle='--', alpha=0.5)
            },
            {
                'ax_attr': 'ax_throttle', 'ax_cand': ax_throttle_candidate,
                'lines_def': [('ctrl_throttle', 'm-', 'Throttle', 1.0)],
                'ylabel': 'Throttle (0-1)', 'ylim': (0, 1),
                'extra_ax_config_func': lambda ax: ax.set_yticks(np.linspace(0, 1, 6))
            },
            {
                'ax_attr': 'ax_diff', 'ax_cand': ax_diff_candidate,
                'lines_def': [('pitch_diff', 'c-', 'Pitch Diff', 1.0), ('roll_diff', 'y-', 'Roll Diff', 1.0)],
                'title': 'Difference Data', 'ylabel': 'Difference',
                'extra_ax_config_func': lambda ax: ax.axhline(0, color='gray', linestyle='--', alpha=0.5)
            },
            {
                'ax_attr': 'ax_motors', 'ax_cand': ax_motors_candidate,
                'lines_def': [
                    ('v_scalar', 'k-', 'V Scalar', 1.0), ('f_cw_scalar', 'b-', 'Front CW', 1.0),
                    ('b_cw_scalar', 'g-', 'Back CW', 1.0), ('f_ccw_scalar', 'r-', 'Front CCW', 1.0),
                    ('b_ccw_scalar', 'm-', 'Back CCW', 1.0)
                ],
                'title': 'Motor Scalars', 'ylabel': 'Scalar Value', 'ylim': (0, 1.1),
                'legend_ncol': 2
            }
        ]

        active_main_plot_axes = [] # For determining which gets the X-axis label

        for config in plot_configs:
            ax_cand = config['ax_cand']
            any_line_plotted = False
            current_ax_lines = []

            for data_key, style, label, alpha in config['lines_def']:
                if data_key in self.data_queues:
                    line, = ax_cand.plot([], [], style, lw=1, label=label, alpha=alpha)
                    self.plot_lines[data_key] = line
                    any_line_plotted = True
                    current_ax_lines.append(line)

            if any_line_plotted:
                setattr(self, config['ax_attr'], ax_cand) # Set self.ax_yaw, self.ax_pitch etc.
                ax = getattr(self, config['ax_attr'])

                if 'ylabel' in config: ax.set_ylabel(config['ylabel'])
                if 'ylim' in config: ax.set_ylim(config['ylim'])
                if 'title' in config: ax.set_title(config['title'])
                if config.get('extra_ax_config_func'): config['extra_ax_config_func'](ax)

                ax.legend(loc='upper right', fontsize=config.get('legend_fontsize', 8), ncol=config.get('legend_ncol', 1))

                if ax not in self.active_axes: # Add to global list of active axes
                    self.active_axes.append(ax)

                # Check if this axis is part of the main left column (sharing x-axis with ax_yaw_candidate)
                if ax_cand in [ax_yaw_candidate, ax_pitch_candidate, ax_roll_candidate, ax_throttle_candidate]:
                    active_main_plot_axes.append(ax)

        # Set xlabel on the last active axis in the first column (main shared X axis)
        if active_main_plot_axes:
            # Find the lowest positioned axis among the active_main_plot_axes
            # Based on GridSpec positions gs[0,0], gs[1,0], gs[2,0], gs[3,0]
            # The one with highest gs-row index is the lowest.
            # ax_throttle_candidate is gs[3,0], ax_roll_candidate gs[2,0] etc.
            lowest_ax = active_main_plot_axes[0] # Default to first one
            if ax_throttle_candidate in active_main_plot_axes: lowest_ax = ax_throttle_candidate
            elif ax_roll_candidate in active_main_plot_axes: lowest_ax = ax_roll_candidate
            elif ax_pitch_candidate in active_main_plot_axes: lowest_ax = ax_pitch_candidate
            # ax_yaw_candidate would be chosen if it's the only one or if others above are not active.

            lowest_ax.set_xlabel('Time (s)')

        # Remove candidate axes that were not used (i.e., no lines plotted on them)
        for ax_attr_name_key, ax_c in self.candidate_axes_map.items():
            if not hasattr(self, ax_attr_name_key): # If self.ax_yaw (etc.) was never set
                 if ax_c in self.fig.axes: # Check if it's still part of the figure
                    self.fig.delaxes(ax_c)
            elif getattr(self, ax_attr_name_key) not in self.active_axes : # If it was set but no lines were drawn
                 if getattr(self, ax_attr_name_key) in self.fig.axes:
                    self.fig.delaxes(getattr(self, ax_attr_name_key))


        plt.tight_layout(rect=[0, 0, 1, 0.96]) # Adjust rect to make space for suptitle
        # plt.subplots_adjust(hspace=0.5, wspace=0.3) # May not be needed if tight_layout is effective
        
        # Set window title
        if self.fig:  # Ensure fig exists
            self.fig.suptitle(f'Drone Log Data - {os.path.basename(self.log_file_path)} - {datetime.now().strftime("%H:%M:%S")}')
        
        # Adjust layout to use the space previously used by the scatter plot
        plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    def plot_data(self):
        """Plot the data on the initialized axes."""
        if not self.timestamps or not self.plot_lines:
            return []
        
        # Convert timestamps to relative seconds
        current_start_time = self.start_time if self.start_time is not None else self.timestamps[0]
        timestamps_rel = [t - current_start_time for t in self.timestamps]
        
        # Update all plot lines
        for key, line_artist in self.plot_lines.items():
            if key in self.data_queues and self.data_queues[key]:
                data = list(self.data_queues[key])
                min_len = min(len(timestamps_rel), len(data))
                if min_len > 0:
                    valid_data = [v for v in data[:min_len] if not np.isnan(v)]
                    if valid_data:
                        if key == 'ctrl_throttle':
                            print(f"Plotting {key} data: min={min(valid_data):.2f}, max={max(valid_data):.2f}, count={len(valid_data)}")
                        line_artist.set_data(timestamps_rel[:min_len], data[:min_len])
        
        # Set axis limits
        if timestamps_rel:
            x_max = max(timestamps_rel) * 1.05
            
            for ax in self.active_axes:
                ax.set_xlim(0, x_max)
                
                if hasattr(self, 'ax_yaw') and ax == self.ax_yaw:
                    ax.set_ylim(-180, 180)
                elif hasattr(self, 'ax_pitch') and ax == self.ax_pitch:
                    ax.set_ylim(-20, 20)
                elif hasattr(self, 'ax_roll') and ax == self.ax_roll:
                    ax.set_ylim(-20, 20)
                elif hasattr(self, 'ax_throttle') and ax == self.ax_throttle:
                    ax.set_ylim(0, 1)
                elif hasattr(self, 'ax_motors') and ax == self.ax_motors:
                    ax.set_ylim(0, 1.1)
        
        return list(self.plot_lines.values())

def main():
    import argparse
    import os
    
    parser = argparse.ArgumentParser(description='Plot drone data from log file')
    parser.add_argument('log_file', help='Path to the log file')
    parser.add_argument('--window', type=int, default=30, help='Time window to display in seconds (default: 30)')
    args = parser.parse_args()
    
    if not os.path.isfile(args.log_file):
        print(f"Error: File '{args.log_file}' not found.", file=sys.stderr)
        sys.exit(1)
    
    plotter = LogDataPlotter(args.log_file, args.window)
    
    if plotter.parse_log_file():
        print("Successfully parsed log file")
        print(f"Found data for: {list(plotter.data_queues.keys())}")
        
        plotter.setup_plot()  # Initialize the figure and axes
        print("Plot setup complete")
        
        plotter.plot_data()  # Plot the data
        print("Data plotted")
        
        plt.tight_layout()  # Adjust layout to prevent overlap
        plt.show()
    else:
        print("Error: Failed to parse log file.", file=sys.stderr)
        sys.exit(1)  # Exit with status 1

if __name__ == "__main__":
    main()
