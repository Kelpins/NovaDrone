# PID Tuning Guide for ESP32 Drone

This guide explains how to dynamically update PID controller constants on the ESP32 drone without recompiling and re-uploading the entire firmware.

## 1. Overview

The drone firmware now includes a Telnet-based interface that allows for real-time PID tuning. This is protected by a simple password mechanism. A Python script, `pid_tuner.py`, is provided to simplify this process.

## 2. Security

- **Password:** To change PID values, you must first authenticate. The password is hardcoded in the Arduino firmware.
  - The default password is: `dronecontrol`
  - **Important:** Changing this password requires modifying the `SECRET_PASSWORD` variable in the `pid_drone_movements/pid_drone_movements.ino` file and re-uploading the firmware.
- **Network Access:** Ensure your drone's WiFi network is secure, as Telnet is not an encrypted protocol.

## 3. Direct Telnet Access (Advanced / Debugging)

You can interact with the PID tuning system directly using any Telnet client.

1.  **Connect to the Drone:**
    - Find the drone's IP address (usually printed to Serial Monitor on boot, or via mDNS: `esp32-nova-drone.local`).
    - Connect using Telnet: `telnet <DRONE_IP_ADDRESS> 23`
    (e.g., `telnet 192.168.76.215 23`)

2.  **Authentication:**
    - Once connected, send the authentication command followed by a newline:
      `!AUTH dronecontrol`
    - The Arduino will print a confirmation to its *Serial output only* if successful. There's no direct Telnet response for successful authentication.

3.  **Sending PID Commands:**
    - Once authenticated, you can send PID update commands.
    - **Format:** `!PID <VARIABLE_NAME>=<VALUE>` (followed by a newline)
    - **Example:** `!PID P_ANGLE_ROLL=-2.5`
    - **Important:**
        - Commands are case-sensitive.
        - Invalid commands or values might be silently ignored or print an error to the *Arduino's Serial output only*.
        - There is no direct Telnet response for successful PID updates. Observe drone behavior or telemetry for confirmation.

4.  **Supported PID Variables (Case-Sensitive):**
    - `P_ANGLE_ROLL`, `I_ANGLE_ROLL`, `D_ANGLE_ROLL`
    - `P_ANGLE_PITCH`, `I_ANGLE_PITCH`, `D_ANGLE_PITCH`
    - `KP_PITCH`, `KI_PITCH`, `KD_PITCH`
    - `KP_ROLL`, `KI_ROLL`, `KD_ROLL`
    - `KP_YAW`, `KI_YAW`, `KD_YAW`

## 4. Using the `pid_tuner.py` Script (Recommended)

The `pid_tuner.py` script provides a more user-friendly command-line interface.

1.  **Prerequisites:**
    - Python 3 installed on your computer.
    - The `pid_tuner.py` script in your current directory.

2.  **Running the Script:**
    ```bash
    python pid_tuner.py [--host <DRONE_IP_ADDRESS>] [--port <PORT>]
    ```
    - If `--host` is not provided, it defaults to `192.168.76.215`.
    - If `--port` is not provided, it defaults to `23`.

3.  **Authentication:**
    - The script will prompt you to enter the password:
      `Enter password: `
    - Type the password (`dronecontrol`) and press Enter.
    - The script will notify you that the authentication command has been sent.

4.  **Commands:**
    - Once authenticated, you'll see the `pid_tuner> ` prompt.
    - **Set a single PID value:**
      `set <PID_VARIABLE> <VALUE>`
      Example: `set P_ANGLE_ROLL -2.1`
      Example: `set KI_PITCH 0.005`
    - **Load PID values from a JSON file:**
      `load <FILEPATH>`
      Example: `load my_pids.json`
      The script will send each PID setting from the file to the drone.
    - **Help:**
      `help`
      Displays available commands.
    - **Exit:**
      `exit`
      Closes the connection and exits the script.

5.  **JSON Configuration File Format:**
    - The file should be a simple JSON object with PID variable names as keys and their desired numeric values.
    - **Example (`pids.json`):**
      ```json
      {
          "P_ANGLE_ROLL": -2.0,
          "I_ANGLE_ROLL": 0.0,
          "D_ANGLE_ROLL": 0.0,
          "P_ANGLE_PITCH": 2.0,
          "I_ANGLE_PITCH": 0.0,
          "D_ANGLE_PITCH": 0.0,
          "KP_PITCH": 0.025,
          "KI_PITCH": 0.0004,
          "KD_PITCH": 0.0,
          "KP_ROLL": -0.025,
          "KI_ROLL": -0.0004,
          "KD_ROLL": 0.0,
          "KP_YAW": 0.015,
          "KI_YAW": 0.0,
          "KD_YAW": 0.0
      }
      ```

## 5. Troubleshooting

- **No connection:** Verify drone IP address and that it's on the same network. Check firewall settings.
- **Authentication failure:** Double-check the password. Remember it's hardcoded in the firmware.
- **PID values not changing:**
    - Ensure you are authenticated.
    - Check the Arduino's Serial Monitor output for any error messages.
    - Verify the PID variable names are correct (case-sensitive).
    - Ensure the `live_graph.py` or other telemetry tools are not interfering (though the system is designed to prevent this, close them temporarily to isolate issues).

This concludes the PID tuning guide. Happy tuning!
