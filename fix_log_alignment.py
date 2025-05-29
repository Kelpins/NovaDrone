import csv
import os

# List of target log file paths
log_files = [
    "live_log_2025-05-28_14-46-50.csv",
    "live_log_2025-05-28_14-51-47.csv",
    "live_log_2025-05-28_14-55-25.csv",
    "live_log_2025-05-28_14-58-32.csv",
    "live_log_2025-05-28_15-02-37.csv",
    "live_log_2025-05-28_15-05-07.csv",
    "live_log_2025-05-28_15-05-49.csv"
]

# Correct header for the fixed logs (18 columns)
correct_header = [
    "timestamp", "imu_roll", "imu_pitch", "imu_yaw",
    "ctrl_roll", "ctrl_pitch", "ctrl_throttle", "ctrl_yaw",
    "pitch_diff", "roll_diff", "current_yaw_target", "yaw_diff",
    "data_field_11", "v_scalar",
    "f_cw_scalar", "b_cw_scalar", "f_ccw_scalar", "b_ccw_scalar"
]

def fix_log_file(input_filepath):
    """
    Corrects the alignment of a single CSV log file and saves it with a '_fixed' suffix.
    """
    base, ext = os.path.splitext(input_filepath)
    output_filepath = f"{base}_fixed{ext}"

    try:
        with open(input_filepath, 'r', newline='') as infile, \
             open(output_filepath, 'w', newline='') as outfile:
            
            reader = csv.reader(infile)
            writer = csv.writer(outfile)

            # Write the correct header to the output file
            writer.writerow(correct_header)

            # Skip the header row in the input file
            try:
                next(reader)
            except StopIteration:
                print(f"Warning: Input file {input_filepath} is empty or has no header.")
                return # Nothing more to do if the file is empty

            # Process each data row
            for old_row in reader:
                if len(old_row) < 18:  # Check if row has enough columns
                    print(f"Warning: Skipping malformed row in {input_filepath}: {old_row}")
                    continue

                new_row = [""] * 18  # Initialize new_row with 18 empty strings

                # Populate new_row based on the specified mapping
                new_row[0] = old_row[0]  # timestamp

                # IMU data (indices from old_row: yaw=2, pitch=3, roll=4)
                new_row[1] = old_row[4]  # imu_roll
                new_row[2] = old_row[3]  # imu_pitch
                new_row[3] = old_row[2]  # imu_yaw

                # Controller data
                new_row[4] = old_row[5]  # ctrl_roll
                new_row[5] = old_row[6]  # ctrl_pitch
                new_row[6] = old_row[7]  # ctrl_throttle
                new_row[7] = old_row[8]  # ctrl_yaw

                # Difference and target values
                new_row[8] = old_row[9]   # pitch_diff
                new_row[9] = old_row[10]  # roll_diff
                new_row[10] = old_row[11] # current_yaw_target
                new_row[11] = old_row[12] # yaw_diff

                # Newly inserted dummy value
                new_row[12] = "0.0"       # data_field_11

                # Scalar values
                new_row[13] = old_row[13] # v_scalar
                new_row[14] = old_row[14] # f_cw_scalar
                new_row[15] = old_row[15] # b_cw_scalar
                new_row[16] = old_row[16] # f_ccw_scalar
                new_row[17] = old_row[17] # b_ccw_scalar
                
                writer.writerow(new_row)
            
            print(f"Processed '{input_filepath}' -> '{output_filepath}'")

    except FileNotFoundError:
        print(f"Error: Input file '{input_filepath}' not found.")
    except Exception as e:
        print(f"An error occurred while processing '{input_filepath}': {e}")

if __name__ == "__main__":
    if not log_files:
        print("No log files specified for processing.")
    else:
        for log_file_path in log_files:
            # Assuming the script is in the root, and log files are also in the root
            # or their paths are relative to the root.
            fix_log_file(log_file_path)
        print("All specified log files processed.")
