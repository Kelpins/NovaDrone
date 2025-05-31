import socket
import sys
import os
import json
import time # Added for time.sleep()

# Global Variables
TELNET_HOST = '192.168.76.215'
TELNET_PORT = 23
SOCKET_TIMEOUT = 5  # seconds
PID_COMMAND_PREFIX = "!PID "
AUTH_COMMAND_PREFIX = "!AUTH "

SUPPORTED_PID_VARIABLES = [
    "P_ANGLE_ROLL", "I_ANGLE_ROLL", "D_ANGLE_ROLL",
    "P_ANGLE_PITCH", "I_ANGLE_PITCH", "D_ANGLE_PITCH",
    "KP_PITCH", "KI_PITCH", "KD_PITCH",
    "KP_ROLL", "KI_ROLL", "KD_ROLL",
    "KP_YAW", "KI_YAW", "KD_YAW"
]

def connect_to_drone(host, port):
    """Creates and returns a socket object connected to the drone."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(SOCKET_TIMEOUT)
        print(f"Connecting to {host}:{port}...")
        sock.connect((host, port))
        print("Connected to drone.")
        # Consume initial messages from drone if any
        time.sleep(0.5) # Give some time for the drone to send initial messages
        try:
            while True:
                sock.recv(4096, socket.MSG_DONTWAIT) # Clear buffer
        except BlockingIOError:
            pass # No more data
        return sock
    except socket.error as e:
        print(f"Error connecting to drone: {e}")
        return None

def send_command(sock, command_str, expect_response=False, timeout=2):
    """Sends command_str + "\n" to the socket."""
    try:
        print(f"Sending command: {command_str.strip()}")
        sock.sendall((command_str + "\n").encode())
        if expect_response:
            sock.settimeout(timeout)
            try:
                response_data = b""
                while True:
                    chunk = sock.recv(1024)
                    if not chunk:
                        break # Connection closed
                    response_data += chunk
                    if b"\n" in chunk: # Assuming newline terminated responses
                        break
                response = response_data.decode().strip()
                if response:
                    print(f"Response: {response}")
                return response
            except socket.timeout:
                print("No response received within timeout.")
                return None
            except socket.error as e:
                print(f"Error receiving response: {e}")
                return None
            finally:
                sock.settimeout(SOCKET_TIMEOUT) # Reset to default timeout
        return None # Or an empty string if no response expected/received
    except socket.error as e:
        print(f"Error sending command: {e}")
        return None

def authenticate(sock):
    """Prompts the user for the password and sends the authentication command."""
    password = input("Enter password: ")
    cmd = AUTH_COMMAND_PREFIX + password
    send_command(sock, cmd)
    # For now, we assume authentication is successful if the command is sent.
    print("Authentication command sent. Check Arduino Serial for confirmation.")
    # Consider adding a brief pause to allow Arduino to process
    time.sleep(0.5)
    return True

def set_pid_value(sock, pid_variable, value):
    """Constructs and sends the PID setting command."""
    # Basic validation, though Arduino is the final validator
    # if pid_variable.upper() not in SUPPORTED_PID_VARIABLES:
    #     print(f"Warning: {pid_variable} is not in the predefined list of supported PID variables.")
    try:
        # Attempt to convert value to float to catch errors early
        float_val = float(value)
        cmd = PID_COMMAND_PREFIX + pid_variable.upper() + "=" + str(float_val)
        send_command(sock, cmd)
        print(f"Sent command to set {pid_variable.upper()} to {float_val}")
    except ValueError:
        print(f"Error: Invalid value '{value}' for PID. Must be a number.")
    except Exception as e:
        print(f"An error occurred in set_pid_value: {e}")


def load_pids_from_file(sock, filepath):
    """Reads PID values from a JSON file and sends them to the drone."""
    if not os.path.exists(filepath):
        print(f"Error: File not found at '{filepath}'")
        return

    try:
        with open(filepath, 'r') as f:
            pid_config = json.load(f)

        print(f"Loading PIDs from {filepath}...")
        for key, value in pid_config.items():
            set_pid_value(sock, key, value)
            time.sleep(0.1) # Small delay between commands
        print("Finished loading PIDs from file.")
    except json.JSONDecodeError:
        print(f"Error: Could not parse JSON from '{filepath}'.")
    except Exception as e:
        print(f"An unexpected error occurred while loading PIDs: {e}")

def print_help():
    """Prints available commands."""
    print("\nAvailable commands:")
    print("  set <PID_VARIABLE> <VALUE>  - Set a PID value (e.g., set P_ANGLE_ROLL -2.5)")
    print("  load <FILEPATH>             - Load PID values from a JSON file (e.g., load pids.json)")
    print("  help                        - Show this help message")
    print("  exit                        - Exit the PID tuner\n")

def main():
    """Main CLI loop for the PID tuner."""
    host = TELNET_HOST
    port = TELNET_PORT

    # Parse command-line arguments for host and port
    if "--host" in sys.argv:
        try:
            host_index = sys.argv.index("--host") + 1
            host = sys.argv[host_index]
        except (ValueError, IndexError):
            print("Error: --host requires an argument.")
            sys.exit(1)

    if "--port" in sys.argv:
        try:
            port_index = sys.argv.index("--port") + 1
            port = int(sys.argv[port_index])
        except (ValueError, IndexError):
            print("Error: --port requires an integer argument.")
            sys.exit(1)

    sock = connect_to_drone(host, port)
    if not sock:
        sys.exit(1)

    if not authenticate(sock):
        print("Authentication failed. Exiting.") # Though currently authenticate always returns True
        sock.close()
        sys.exit(1)

    print_help() # Show help on startup

    try:
        while True:
            try:
                user_input = input("pid_tuner> ").strip()
                if not user_input:
                    continue

                parts = user_input.split()
                command = parts[0].lower()

                if command == "exit":
                    print("Exiting PID tuner.")
                    break
                elif command == "help":
                    print_help()
                elif command == "set":
                    if len(parts) == 3:
                        pid_var = parts[1]
                        pid_val = parts[2]
                        set_pid_value(sock, pid_var, pid_val)
                    else:
                        print("Error: 'set' command requires PID_VARIABLE and VALUE.")
                        print("Usage: set <PID_VARIABLE> <VALUE>")
                elif command == "load":
                    if len(parts) == 2:
                        filepath = parts[1]
                        load_pids_from_file(sock, filepath)
                    else:
                        print("Error: 'load' command requires FILEPATH.")
                        print("Usage: load <FILEPATH>")
                else:
                    print(f"Error: Unknown command '{command}'. Type 'help' for available commands.")

            except EOFError: # Handle Ctrl+D
                print("\nExiting PID tuner (EOF).")
                break
            except KeyboardInterrupt: # Handle Ctrl+C
                print("\nExiting PID tuner (Interrupted).")
                break
            except Exception as e:
                print(f"An unexpected error occurred in the main loop: {e}")
                # Consider whether to break or continue based on the error
                # For robustness, it might be better to log and continue if possible
                # For now, we'll print and continue the loop
    finally:
        if sock:
            print("Closing connection.")
            sock.close()

if __name__ == "__main__":
    main()
