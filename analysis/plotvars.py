import numpy as np
from collections import defaultdict
import sys

def parse_logfile(filepath):
    # Store lists of timestamps and values mapped by variable name
    raw_data = defaultdict(lambda: {"timestamps": [], "values": []})

    with open(filepath, "r", encoding="utf-8") as file:
        for line_num, line in enumerate(file, start=1):
            line = line.strip()
            
            # Only process lines ending with the <<<VAR marker
            if not line.endswith("<<<VAR"):
                continue
            
            # Remove the '<<<VAR' suffix
            clean_line = line[:-6]
            
            # Split by tab (falls back to whitespace if tabs were converted to spaces)
            parts = clean_line.split("\t")
            if len(parts) != 3:
                parts = clean_line.split()
                
            if len(parts) != 3:
                print(f"Skipping malformed line {line_num}: {line}")
                continue
                
            timestamp_str, var_name, val_str = parts
            
            dt = float(timestamp_str)/1000
            val = float(val_str)

            raw_data[var_name]["timestamps"].append(dt)
            raw_data[var_name]["values"].append(val)

    # Convert the collected lists into mapped NumPy arrays
    mapped_arrays = {}
    for var_name, content in raw_data.items():
        mapped_arrays[var_name] = {
            "timestamps": np.array(content["timestamps"], dtype=np.float64),
            "values": np.array(content["values"], dtype=np.float64)
        }
        
    return mapped_arrays


import matplotlib.pyplot as plt
import matplotlib.dates as mdates

def plot_combined(log_data, variables_to_plot=None):
    plt.figure(figsize=(10, 5))

    # Plot all variables unless a specific list is provided
    keys = variables_to_plot if variables_to_plot else log_data.keys()

    for var_name in keys:
        if var_name in log_data:
            plt.plot(
                log_data[var_name]["timestamps"],
                log_data[var_name]["values"],
                marker=".",
                label=var_name
            )

    plt.title("Log Variables Over Time")
    plt.xlabel("Time/sec")
    plt.ylabel("Value")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()

    plt.tight_layout()
    plt.show()

# --- Usage ---


# --- Example Usage ---
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: {} logfile.txt".format(sys.argv[0]))
        exit(1)
        
    log_data = parse_logfile(sys.argv[1])
    
    plot_combined(log_data)
