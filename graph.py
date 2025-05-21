import socket
import re
import time
import sys
import argparse
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.animation import FuncAnimation
from datetime import datetime, timedelta
from collections import deque

class DroneDataPlotter:
    def __init__(self, host, port, window_size=30, update_interval=50):
        """
        Initialize the DroneDataPlotter.
        
        Args:
            host (str): IP address of the drone
            port (int): Port number for the telnet connection
            window_size (int): Time window to display in seconds
            update_interval (int): Plot update interval in milliseconds
        """
        self.host = host
        self.port = port
        self.window_size = window_size
        self.update_interval = update_interval
        self.max_data_points = 1000  # Maximum number of data points to keep in memory
        
        # Initialize data storage with deques for efficient appending/poping
        self.timestamps = deque(maxlen=self.max_data_points)
        self.yaw_data = deque(maxlen=self.max_data_points)
        self.pitch_data = deque(maxlen=self.max_data_points)
        self.roll_data = deque(maxlen=self.max_data_points)
        
        self.start_time = time.time()
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
        
        # Create subplots
        self.fig, (self.ax_yaw, self.ax_pitch, self.ax_roll) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        
        # Yaw plot (-180° to 180°)
        self.ln_yaw, = self.ax_yaw.plot([], [], 'b-', lw=1.5, label='Yaw')
        self.ax_yaw.set_ylabel('Yaw (°)')
        self.ax_yaw.set_ylim(-185, 185)
        self.ax_yaw.set_yticks(range(-180, 181, 45))
        self.ax_yaw.axhline(0, color='gray', linestyle='--', alpha=0.5)
        
        # Pitch plot (-45° to 45°)
        self.ln_pitch, = self.ax_pitch.plot([], [], 'g-', lw=1.5, label='Pitch')
        self.ax_pitch.set_ylabel('Pitch (°)')
        self.ax_pitch.set_ylim(-50, 50)
        self.ax_pitch.set_yticks(range(-45, 46, 15))
        self.ax_pitch.axhline(0, color='gray', linestyle='--', alpha=0.5)
        
        # Roll plot (-45° to 45°)
        self.ln_roll, = self.ax_roll.plot([], [], 'r-', lw=1.5, label='Roll')
        self.ax_roll.set_ylabel('Roll (°)')
        self.ax_roll.set_xlabel('Time')
        self.ax_roll.set_ylim(-50, 50)
        self.ax_roll.set_yticks(range(-45, 46, 15))
        self.ax_roll.axhline(0, color='gray', linestyle='--', alpha=0.5)
        
        # Configure time axis
        self.ax_roll.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
        
        # Set window title and adjust layout
        self.fig.suptitle(f'Drone Orientation - {datetime.now().strftime("%H:%M:%S")}')
        plt.tight_layout()
        plt.subplots_adjust(hspace=0.25)
        
        # Set initial time window
        self.end_time = datetime.now()
        self.start_time_dt = self.end_time - timedelta(seconds=self.window_size)
    
    def read_data(self):
        """Read and parse data from the socket stream."""
        try:
            line = self.stream.readline()
            if not line:
                print("Connection closed by remote host")
                return None
                
            # Look for orientation data in the format "Iy: 123.45 Ip: 123.45 Ir: 123.45"
            yaw_match = re.search(r"Iy:\s*([-+]?\d+\.?\d*)", line)
            pitch_match = re.search(r"Ip:\s*([-+]?\d+\.?\d*)", line)
            roll_match = re.search(r"Ir:\s*([-+]?\d+\.?\d*)", line)
            
            if yaw_match and pitch_match and roll_match:
                current_time = datetime.now()
                yaw = float(yaw_match.group(1))
                pitch = float(pitch_match.group(1))
                roll = float(roll_match.group(1))
                
                # Normalize angles to -180 to 180 range
                yaw = (yaw + 180) % 360 - 180
                pitch = (pitch + 180) % 360 - 180
                roll = (roll + 180) % 360 - 180
                
                return current_time, yaw, pitch, roll
                
            return None
        except Exception as e:
            print(f"Error reading data: {e}")
            return None
    
    def update_plot(self, frame):
        """Update the plot with new data."""
        data = self.read_data()
        
        if data:
            current_time, yaw, pitch, roll = data
            
            # Update data storage
            self.timestamps.append(current_time)
            self.yaw_data.append(yaw)
            self.pitch_data.append(pitch)
            self.roll_data.append(roll)
            
            # Update plot data
            self.ln_yaw.set_data(self.timestamps, self.yaw_data)
            self.ln_pitch.set_data(self.timestamps, self.pitch_data)
            self.ln_roll.set_data(self.timestamps, self.roll_data)
            
            # Update time window
            self.end_time = current_time
            self.start_time_dt = self.end_time - timedelta(seconds=self.window_size)
            
            # Update x-axis limits for all subplots
            for ax in [self.ax_yaw, self.ax_pitch, self.ax_roll]:
                ax.set_xlim(self.start_time_dt, self.end_time)
        
        # Update title with current time
        self.fig.suptitle(f'Drone Orientation Data - {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
        
        return self.ln_yaw, self.ln_pitch, self.ln_roll
    
    def init_plot(self):
        """Initialize the plot with empty data."""
        self.ln_yaw.set_data([], [])
        self.ln_pitch.set_data([], [])
        self.ln_roll.set_data([], [])
        return self.ln_yaw, self.ln_pitch, self.ln_roll
    
    def run(self):
        """Start the data collection and plotting."""
        if not self.setup_connection():
            print("Failed to establish connection. Exiting...")
            return
        
        try:
            print("Starting data collection. Close the plot window to exit...")
            self.ani = FuncAnimation(
                self.fig, 
                self.update_plot, 
                init_func=self.init_plot, 
                blit=True, 
                interval=self.update_interval,
                cache_frame_data=False,
                save_count=1000
            )
            plt.show()
        except KeyboardInterrupt:
            print("\nShutting down...")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, 'sock') and self.sock:
            self.sock.close()
        if hasattr(self, 'stream') and self.stream:
            self.stream.close()
        print("Connection closed.")

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Plot drone yaw data in real-time.')
    parser.add_argument('--host', type=str, default='192.168.76.215',
                       help='IP address of the drone (default: 192.168.76.215)')
    parser.add_argument('--port', type=int, default=23,
                       help='Port number (default: 23)')
    parser.add_argument('--window', type=int, default=10,
                       help='Time window to display in seconds (default: 10)')
    parser.add_argument('--interval', type=int, default=100,
                       help='Update interval in milliseconds (default: 100)')
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
