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
    
    data = [
        imu_yaw, imu_pitch, imu_roll,
        cout_roll, cout_pitch, cout_throttle, cout_yaw,
        pitch_diff, roll_diff, current_yaw_target, yaw_diff,
        v_scalar, f_cw_scalar, b_cw_scalar, f_ccw_scalar, b_ccw_scalar
    ]
    return ",".join(map(str, data)) + "\n"

def main():
    global running
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.settimeout(1.0) # Timeout for accept
    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print(f"Fake IMU sender listening on {HOST}:{PORT}")
        print("Press Ctrl+C to stop.")

        counter = 0
        client_socket = None

        while running:
            if client_socket is None:
                try:
                    client_socket, addr = server_socket.accept()
                    client_socket.settimeout(1.0) # Timeout for send/recv
                    print(f"Accepted connection from {addr}")
                except socket.timeout:
                    continue # No connection attempt, continue loop
                except Exception as e:
                    if running: # Avoid error message if shutting down
                        print(f"Error accepting connection: {e}")
                    continue

            if client_socket:
                try:
                    data_line = generate_data_line(counter)
                    client_socket.sendall(data_line.encode('utf-8'))
                    # print(f"Sent: {data_line.strip()}") # Uncomment for debugging
                    counter += 1
                    time.sleep(0.1)  # Send data every 100ms
                except socket.timeout:
                    print("Socket timeout while sending data. Client might have disconnected.")
                    client_socket.close()
                    client_socket = None
                except socket.error as e:
                    print(f"Socket error: {e}. Client disconnected.")
                    client_socket.close()
                    client_socket = None
                except Exception as e:
                    print(f"An unexpected error occurred: {e}")
                    client_socket.close()
                    client_socket = None
        
    except Exception as e:
        if running:
            print(f"Server error: {e}")
    finally:
        print("Closing server socket.")
        if client_socket:
            client_socket.close()
        server_socket.close()
        print("Server shut down.")

if __name__ == "__main__":
    main()
