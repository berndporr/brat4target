import numpy as np
from collections import defaultdict
from datetime import datetime
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
            
            try:
                # Parse 'yymmdd_hhmm' into a standard datetime object
                dt = datetime.strptime(timestamp_str, "%y%m%d_%H%M")
                val = float(val_str)
            except ValueError as e:
                print(f"Skipping line {line_num} due to parse error ({e}): {line}")
                continue
                
            raw_data[var_name]["timestamps"].append(dt)
            raw_data[var_name]["values"].append(val)

    # Convert the collected lists into mapped NumPy arrays
    mapped_arrays = {}
    for var_name, content in raw_data.items():
        mapped_arrays[var_name] = {
            "timestamps": np.array(content["timestamps"], dtype="datetime64[m]"),
            "values": np.array(content["values"], dtype=np.float64)
        }
        
    return mapped_arrays

# --- Example Usage ---
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: {} logfile.txt".format(sys.argv[0]))
        exit(1)
        
    log_data = parse_logfile(sys.argv[1])
    
    # Access a specific variable's arrays
    if "targetAngle" in log_data:
        angles = log_data["targetAngle"]["values"]
        times = log_data["targetAngle"]["timestamps"]
        
        print(f"Extracted {len(angles)} entries for targetAngle:")
        print("Timestamps:", times)
        print("Values:", angles)
