import socket
import time
import signal
import sys

# Host and port for the server
HOST = 'localhost'
PORT = 12345

# Global flag to control the server loop
running = True

def signal_handler(sig, frame):
    global running
    print("\nSignal received, shutting down server...")
    running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def generate_data_line(counter):
    """Generates a CSV string with 16 cycling/varying float values."""
    imu_yaw = (counter % 360) - 180  # Cycles -180 to 179
    imu_pitch = (counter % 90) - 45    # Cycles -45 to 44
    imu_roll = (counter % 90) - 45     # Cycles -45 to 44
    
    cout_roll = 1500 + (counter % 100) - 50      # 1450 to 1549
    cout_pitch = 1500 - ((counter % 100) - 50)   # 1550 to 1451
    cout_throttle = 1000 + (counter % 1000)    # 1000 to 1999
    cout_yaw = 1500 + (counter % 200) - 100      # 1400 to 1599
    
    pitch_diff = (counter % 10) / 10.0 - 0.5    # -0.5 to 0.4
    roll_diff = ((counter + 5) % 10) / 10.0 - 0.5 # -0.5 to 0.4
    current_yaw_target = (counter % 360)        # 0 to 359
    yaw_diff = (counter % 20) - 10              # -10 to 9
    
    v_scalar = (counter % 10) / 10.0            # 0.0 to 0.9
    f_cw_scalar = ((counter + 1) % 10) / 10.0   # 0.1 to 0.0
    b_cw_scalar = ((counter + 2) % 10) / 10.0
    f_ccw_scalar = ((counter + 3) % 10) / 10.0
    b_ccw_scalar = ((counter + 4) % 10) / 10.0
    dummy_extra_field = (counter % 5) / 5.0 # Cycles 0.0, 0.2, 0.4, 0.6, 0.8
    
    data = [
        imu_yaw, imu_pitch, imu_roll,
        cout_roll, cout_pitch, cout_throttle, cout_yaw,
        pitch_diff, roll_diff, current_yaw_target, yaw_diff,
        v_scalar, f_cw_scalar, b_cw_scalar, f_ccw_scalar, b_ccw_scalar,
        dummy_extra_field # This is the 17th data field (index 16)
    ]
    return ",".join(map(str, data)) + "\n"

def main():
    global running
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.settimeout(1.0) # For server_socket.accept()

    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print(f"Fake IMU sender listening on {HOST}:{PORT}")
        print("Press Ctrl+C to stop.")

        client_socket = None
        counter = 0

        while running:
            if client_socket is None:
                try:
                    print("Waiting for a connection...")
                    client_socket, addr = server_socket.accept()
                    # No client-specific timeout for now, rely on send errors
                    print(f"Accepted connection from {addr}")
                except socket.timeout:
                    # Normal, no connection attempt in the last second
                    pass
                except Exception as e:
                    # Error during accept, client_socket remains None
                    if running: # Only print if not shutting down
                        print(f"Error accepting connection: {e}")
                    pass # Continue to time.sleep
            else: # if client_socket is not None
                try:
                    data_line = generate_data_line(counter) # Assumes 17 fields
                    client_socket.sendall(data_line.encode('utf-8'))
                    # print(f"Sent: {data_line.strip()}") # Uncomment for debugging
                    counter += 1
                except (socket.error, BrokenPipeError, ConnectionResetError) as e:
                    print(f"Socket error during send: {e}. Client disconnected.")
                    try:
                        client_socket.close()
                    except socket.error:
                        pass # Ignore errors during close if already bad
                    client_socket = None # Reset to allow re-accepting
                except Exception as e:
                    # Catch any other error during send
                    if running: # Only print if not shutting down
                        print(f"Unexpected error during send: {e}")
                    try:
                        client_socket.close()
                    except socket.error:
                        pass
                    client_socket = None # Reset
            
            time.sleep(0.1) # Loop pause

    except Exception as e: # Catch errors setting up the server socket itself
        if running: # Only print if not shutting down
            print(f"Server setup error: {e}")
    finally:
        print("Closing server socket.")
        if client_socket:
            try:
                client_socket.close()
            except socket.error:
                pass
        server_socket.close()
        print("Server shut down.")

if __name__ == "__main__":
    main()
