#!/usr/bin/env python3
import os
import sys
import glob
import csv
import statistics

def compute_stats_and_raw(filepath):
    """
    Reads a CSV file with rows: success, init_time, opt_time
    Returns (stats_dict, successes, init_times, opt_times).
    """
    successes = []
    init_times = []
    opt_times  = []
    with open(filepath, newline='') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) < 3:
                continue
            try:
                succ = int(row[0])
                init = float(row[1])
                opt  = float(row[2])
            except ValueError:
                continue
            successes.append(succ)
            init_times.append(init)
            opt_times.append(opt)

    n = len(successes)
    if n == 0:
        return None, successes, init_times, opt_times

    success_rate = sum(successes) / n

    def stats(arr):
        avg = statistics.mean(arr)
        mn  = min(arr)
        mx  = max(arr)
        std = statistics.stdev(arr) if len(arr) > 1 else 0.0
        return avg, mn, mx, std

    init_avg, init_min, init_max, init_std = stats(init_times)
    opt_avg,  opt_min,  opt_max,  opt_std  = stats(opt_times)

    stats_dict = {
        'count':        n,
        'success_rate': success_rate,
        'init': {
            'avg': init_avg,
            'min': init_min,
            'max': init_max,
            'std': init_std,
        },
        'opt': {
            'avg': opt_avg,
            'min': opt_min,
            'max': opt_max,
            'std': opt_std,
        }
    }
    return stats_dict, successes, init_times, opt_times

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <folder_path>")
        sys.exit(1)

    folder    = sys.argv[1]
    pattern   = os.path.join(folder, "results_num_*.csv")
    file_list = sorted(glob.glob(pattern))
    if not file_list:
        print(f"No files matching pattern {pattern}")
        sys.exit(1)

    all_successes = []
    all_init      = []
    all_opt       = []

    output_path = os.path.join(folder, "statistics.txt")
    with open(output_path, "w") as out:
        # Per‐file stats
        for filepath in file_list:
            stats, succs, inits, opts = compute_stats_and_raw(filepath)
            filename = os.path.basename(filepath)

            # Accumulate raw data for overall
            all_successes.extend(succs)
            all_init.extend(inits)
            all_opt.extend(opts)

            if stats is None:
                out.write(f"{filename}: no valid data\n\n")
                continue

            out.write(f"File: {filename}\n")
            out.write(f"  Runs: {stats['count']}\n")
            out.write(f"  Success rate: {stats['success_rate']:.3f}\n")
            out.write(
                f"  Init Comp (ms):  avg={stats['init']['avg']:.3f}, "
                f"min={stats['init']['min']:.3f}, max={stats['init']['max']:.3f}, "
                f"std={stats['init']['std']:.3f}\n"
            )
            out.write(
                f"  Opt  Comp (ms):  avg={stats['opt']['avg']:.3f}, "
                f"min={stats['opt']['min']:.3f}, max={stats['opt']['max']:.3f}, "
                f"std={stats['opt']['std']:.3f}\n\n"
            )

        # Overall stats
        total_runs = len(all_successes)
        if total_runs > 0:
            overall_success_rate = sum(all_successes) / total_runs

            def stats(arr):
                avg = statistics.mean(arr)
                mn  = min(arr)
                mx  = max(arr)
                std = statistics.stdev(arr) if len(arr) > 1 else 0.0
                return avg, mn, mx, std

            init_avg, init_min, init_max, init_std = stats(all_init)
            opt_avg,  opt_min,  opt_max,  opt_std  = stats(all_opt)

            out.write("Overall Performance:\n")
            out.write(f"  Total runs: {total_runs}\n")
            out.write(f"  Success rate: {overall_success_rate:.3f}\n")
            out.write(
                f"  Init Comp (ms):  avg={init_avg:.3f}, "
                f"min={init_min:.3f}, max={init_max:.3f}, "
                f"std={init_std:.3f}\n"
            )
            out.write(
                f"  Opt  Comp (ms):  avg={opt_avg:.3f}, "
                f"min={opt_min:.3f}, max={opt_max:.3f}, "
                f"std={opt_std:.3f}\n"
            )
        else:
            out.write("Overall Performance: no valid data\n")

    print(f"Statistics written to {output_path}")

if __name__ == "__main__":
    main()
