#!/usr/bin/env python3
import os
import sys
import glob
import csv

def compute_average_for_file(filepath):
    """
    Reads a CSV file where the first line is a header and the following lines contain a single numeric value.
    Returns the average and the sample count.
    """
    times = []
    with open(filepath, "r") as f:
        reader = csv.reader(f)
        # Skip header row.
        header = next(reader)
        for row in reader:
            if len(row) < 1:
                continue
            try:
                value = float(row[0])
                times.append(value)
            except Exception:
                continue
    if times:
        avg = sum(times) / len(times)
        return avg, len(times)
    else:
        return None, 0

def main():
    if len(sys.argv) < 2:
        print("Usage: {} <folder_path>".format(sys.argv[0]))
        sys.exit(1)
    
    folder = sys.argv[1]
    pattern = os.path.join(folder, "computation_times_num_*.csv")
    file_list = glob.glob(pattern)
    if not file_list:
        print("No files matching pattern found in folder: {}".format(folder))
        sys.exit(1)
    
    total_sum = 0.0
    total_count = 0
    file_averages = {}
    # Prepare rows for the output CSV.
    rows = []
    rows.append(["file_name", "average_time_microseconds", "sample_count"])

    # Process each CSV file.
    for filepath in sorted(file_list):
        avg, count = compute_average_for_file(filepath)
        filename = os.path.basename(filepath)
        if avg is not None:
            file_averages[filename] = avg
            total_sum += avg * count
            total_count += count
            rows.append([filename, "{:.2f}".format(avg), count])
            print("File {}: Average = {:.2f} microseconds ({} samples)".format(filename, avg, count))
        else:
            rows.append([filename, "N/A", 0])
            print("File {}: No valid samples found.".format(filename))
    
    if total_count > 0:
        overall_avg = total_sum / total_count
        rows.append(["Overall Average", "{:.2f}".format(overall_avg), total_count])
        print("\nTotal average computation time across {} files ({} samples): {:.2f} microseconds".format(len(file_averages), total_count, overall_avg))
    else:
        print("No valid computation times found in any file.")
        rows.append(["Overall Average", "N/A", 0])
    
    # Write results to average_computation_times.csv.
    output_csv = folder + "/average_computation_times.csv"
    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(rows)
    
    print("Averages saved to {}".format(output_csv))

if __name__ == "__main__":
    main()
