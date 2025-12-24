#!/usr/bin/env python3
"""
Plot benchmark results: Success rate progression over time (academic-style).
Shows how quickly each method reaches 100% success rate, with individual run completion times.
Adapted for occupancy grid benchmark results.
"""

import os
import csv
import glob
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from collections import defaultdict

# Use non-interactive backend
matplotlib.use('Agg')

def read_csv_file(filepath):
    """Read a CSV file and return list of dictionaries."""
    data = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(row)
    return data

def parse_success(success_str):
    """Parse success string to boolean."""
    if isinstance(success_str, bool):
        return success_str
    if isinstance(success_str, int):
        return bool(success_str)
    return str(success_str).lower() in ('true', '1', 'yes')

def parse_time(time_str):
    """Parse time string to float."""
    try:
        return float(time_str)
    except (ValueError, TypeError):
        return None

def get_config_label(planner_name):
    """Get a label for the configuration from planner name."""
    # Map planner names to display labels
    if planner_name == 'MAB-RRT' or planner_name == 'MAB-SSRRT':
        return "MAB-RRT"
    elif planner_name == 'RRT':
        return "RRT (uniform)"
    elif planner_name == 'RRT-Gaussian':
        return "RRT (gaussian)"
    elif planner_name == 'RRT-Bridge':
        return "RRT (bridge)"
    else:
        return planner_name

def calculate_success_rate_progression(runs, max_time=None, time_resolution=None):
    """
    Calculate success rate progression over time.
    
    Args:
        runs: List of (success, time) tuples
        max_time: Maximum time to plot (auto-detected if None)
        time_resolution: Time step for interpolation (auto-detected if None)
    
    Returns:
        (time_points, success_rates, individual_times, time_to_100) where:
        - time_points: array of time values
        - success_rates: array of success rates (0-100%)
        - individual_times: list of completion times for successful runs
        - time_to_100: time when 100% success rate is reached (or None)
    """
    total_runs = len(runs)
    if total_runs == 0:
        return np.array([]), np.array([]), [], None
    
    # Get all times (successful and failed) to determine range
    all_times = []
    successful_times = []
    for success, t in runs:
        if t is not None:
            all_times.append(t)
            if success:
                successful_times.append(t)
    successful_times.sort()
    all_times.sort()
    
    # Auto-detect time range if not provided
    if max_time is None:
        if all_times:
            # Use 1.5x the maximum time, with a minimum of 0.001
            max_time = max(all_times[-1] * 1.5, 0.001)
        else:
            max_time = 0.001
    
    # Filter successful times by max_time
    successful_times = [t for t in successful_times if t <= max_time]
    
    # Calculate time to reach 100%
    time_to_100 = None
    if len(successful_times) == total_runs:
        time_to_100 = max(successful_times) if successful_times else None
    
    # Auto-detect time resolution
    if time_resolution is None:
        if all_times:
            # Use 1/1000 of the max time, with reasonable bounds
            time_resolution = max(max_time / 1000.0, min(all_times) / 10.0 if all_times else max_time / 1000.0)
        else:
            time_resolution = max_time / 1000.0
    
    # Create time points (start from a small fraction of min time for log scale)
    if all_times:
        min_time = min(all_times) * 0.5  # Start at half the minimum time
        min_time = max(min_time, max_time / 10000.0)  # But not too small
    else:
        min_time = max_time / 10000.0
    
    time_points = np.arange(min_time, max_time + time_resolution, time_resolution)
    success_rates = np.zeros_like(time_points)
    
    # Build success rate progression
    current_success_count = 0
    run_idx = 0
    
    for i, t in enumerate(time_points):
        # Count how many runs have succeeded by time t
        while run_idx < len(successful_times) and successful_times[run_idx] <= t:
            current_success_count += 1
            run_idx += 1
        # Calculate success rate as percentage
        success_rates[i] = (current_success_count / total_runs) * 100.0
    
    return time_points, success_rates, successful_times, time_to_100

def plot_success_rate_progression(results_file, output_dir=None, max_time=None, env_name="Occupancy Grid"):
    """
    Create success rate progression plots (academic-style).
    
    Args:
        results_file: Path to CSV result file (or directory containing CSV files)
        output_dir: Directory to save plots (default: same directory as results_file)
        max_time: Maximum time to show on x-axis
        env_name: Name of the environment for plot titles
    """
    # Determine if input is a file or directory
    if os.path.isdir(results_file):
        # Find all CSV files in directory
        csv_files = glob.glob(os.path.join(results_file, '*_results.csv'))
        if not csv_files:
            csv_files = glob.glob(os.path.join(results_file, '*.csv'))
        if not csv_files:
            print(f"No CSV files found in {results_file}")
            return
        results_dir = results_file
    else:
        csv_files = [results_file]
        results_dir = os.path.dirname(os.path.abspath(results_file))
    
    if output_dir is None:
        output_dir = os.path.join(results_dir, 'plots')
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Found {len(csv_files)} result file(s)")
    
    # Read all data
    all_data = []
    for csv_file in csv_files:
        data = read_csv_file(csv_file)
        all_data.extend(data)
    
    # Group by configuration (planner)
    config_data = defaultdict(list)
    all_times_list = []  # Collect all times to determine range
    
    for row in all_data:
        planner = row.get('planner', row.get('Planner', ''))
        success = parse_success(row.get('solved', row.get('Success', '0')))
        time = parse_time(row.get('time_sec', row.get('Time', None)))
        
        config_label = get_config_label(planner)
        config_data[config_label].append((success, time))
        if time is not None:
            all_times_list.append(time)
    
    all_configs = sorted(config_data.keys())
    print(f"Found {len(all_configs)} configurations: {all_configs}")
    
    # Auto-detect max_time if not provided
    if max_time is None:
        if all_times_list:
            max_time = max(all_times_list) * 1.5
            # Round to a nice number
            if max_time < 0.001:
                max_time = 0.001
            elif max_time < 0.01:
                max_time = 0.01
            elif max_time < 0.1:
                max_time = 0.1
            elif max_time < 1.0:
                max_time = 1.0
            else:
                max_time = np.ceil(max_time)
        else:
            max_time = 0.001
        print(f"Auto-detected max_time: {max_time} seconds")
    
    # Create color map for configurations (bright, distinctive colors)
    config_colors = {}
    for i, cfg in enumerate(all_configs):
        if cfg == "MAB-RRT" or cfg == "MAB-SSRRT":
            config_colors[cfg] = "#d62728"  # red
        elif cfg == "RRT (bridge)":
            config_colors[cfg] = "#2ca02c"  # green
        elif cfg == "RRT (gaussian)":
            config_colors[cfg] = "#9467bd"  # purple
        elif cfg == "RRT (uniform)":
            config_colors[cfg] = "#1f77b4"  # blue
        else:
            # Fallback: use tab10 palette
            colors = plt.cm.tab10(np.linspace(0, 1, len(all_configs)))
            config_colors[cfg] = colors[i]
    
    # Store statistics for summary
    stats_summary = {}
    
    # Create figure for main plot
    fig, ax = plt.subplots(figsize=(8, 5))
    
    # Plot each configuration
    for config_label in all_configs:
        runs = config_data[config_label]
        color = config_colors[config_label]
        
        # Calculate success rate progression
        time_points, success_rates, individual_times, time_to_100 = \
            calculate_success_rate_progression(runs, max_time=max_time)
        
        if len(time_points) == 0:
            continue
        
        # Store statistics
        total_runs = len(runs)
        successful_runs = len(individual_times)
        final_success_rate = success_rates[-1] if len(success_rates) > 0 else 0.0
        
        stats_summary[config_label] = {
            'time_to_100': time_to_100,
            'total_runs': total_runs,
            'successful_runs': successful_runs,
            'final_success_rate': final_success_rate,
            'avg_time': np.mean(individual_times) if individual_times else None,
            'median_time': np.median(individual_times) if individual_times else None
        }
        
        # Plot the success rate progression line
        ax.plot(time_points, success_rates, 
               label=config_label, color=color, linewidth=2.0, alpha=0.9, zorder=10)
        
        # Mark time to 100% if reached (vertical line)
        if time_to_100 is not None:
            ax.axvline(x=time_to_100, color=color, linestyle='--', 
                      linewidth=1.0, alpha=0.5, zorder=5)
    
    ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Success Rate (%)', fontsize=12, fontweight='bold')
    ax.set_title(f'{env_name} - Success Rate Progression', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Determine appropriate scale and limits
    if all_times_list:
        min_time = min(all_times_list) * 0.5
        min_time = max(min_time, max_time / 10000.0)
    else:
        min_time = max_time / 10000.0
    
    # Use log scale for time if range spans orders of magnitude, otherwise linear
    if max_time / min_time > 10:
        ax.set_xscale('log')
        ax.set_xlim([min_time, max_time])
    else:
        ax.set_xscale('linear')
        ax.set_xlim([0, max_time])
    
    ax.set_ylim([0, 105])
    ax.axhline(y=100, color='gray', linestyle=':', linewidth=1, alpha=0.5, zorder=1)
    ax.legend(loc='lower right', fontsize=10, framealpha=0.9)
    
    plt.tight_layout()
    
    # Save main plot
    output_file = os.path.join(output_dir, 'success_rate_progression.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Main plot saved to: {output_file}")
    
    output_file_pdf = os.path.join(output_dir, 'success_rate_progression.pdf')
    plt.savefig(output_file_pdf, bbox_inches='tight')
    print(f"✓ Main plot (PDF) saved to: {output_file_pdf}")
    
    plt.close()
    
    # Create academic paper style plot (single column width)
    print("\nCreating academic paper style plot...")
    
    # Set scientific font family (Times New Roman style)
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman', 'Times', 'DejaVu Serif']
    plt.rcParams['mathtext.fontset'] = 'stix'
    
    fig, ax = plt.subplots(figsize=(3.5, 2.5))
    
    # Plot each configuration
    for config_label in all_configs:
        runs = config_data[config_label]
        color = config_colors[config_label]
        
        # Calculate success rate progression
        time_points, success_rates, individual_times, time_to_100 = \
            calculate_success_rate_progression(runs, max_time=max_time)
        
        if len(time_points) == 0:
            continue
        
        # Plot the success rate progression line
        ax.plot(time_points, success_rates, 
               label=config_label, color=color, linewidth=1.8, alpha=0.95, zorder=10)
        
        # Mark time to 100% if reached
        if time_to_100 is not None:
            ax.axvline(x=time_to_100, color=color, linestyle='--', 
                      linewidth=1.0, alpha=0.5, zorder=5)
    
    # Academic paper styling
    ax.set_xlabel('Time (seconds)', fontsize=11, fontfamily='serif')
    ax.set_ylabel('Success Rate (%)', fontsize=11, fontfamily='serif')
    ax.set_title(f'{env_name}', fontsize=11, fontweight='normal', fontfamily='serif')
    ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    
    # Determine appropriate scale and limits (same as main plot)
    if all_times_list:
        min_time = min(all_times_list) * 0.5
        min_time = max(min_time, max_time / 10000.0)
    else:
        min_time = max_time / 10000.0
    
    # Use log scale for time if range spans orders of magnitude, otherwise linear
    if max_time / min_time > 10:
        ax.set_xscale('log')
        ax.set_xlim([min_time, max_time])
    else:
        ax.set_xscale('linear')
        ax.set_xlim([0, max_time])
    
    ax.set_ylim([0, 105])
    ax.axhline(y=100, color='gray', linestyle=':', linewidth=1.0, alpha=0.5, zorder=1)
    
    # Increase tick label sizes
    ax.tick_params(axis='both', which='major', labelsize=9, width=0.5, length=3)
    ax.tick_params(axis='both', which='minor', labelsize=8, width=0.5, length=2)
    
    # Adjust spines for cleaner look
    for spine in ax.spines.values():
        spine.set_linewidth(0.5)
    
    # Legend
    ax.legend(loc='lower right', fontsize=9, framealpha=0.9, frameon=True)
    
    plt.tight_layout(pad=0.3)
    
    output_file_png = os.path.join(output_dir, f'{env_name.replace(" ", "_")}_success_rate_progression.png')
    output_file_pdf = os.path.join(output_dir, f'{env_name.replace(" ", "_")}_success_rate_progression.pdf')
    plt.savefig(output_file_png, dpi=300, bbox_inches='tight', pad_inches=0.05)
    plt.savefig(output_file_pdf, bbox_inches='tight', pad_inches=0.05)
    print(f"✓ Academic plot saved: {output_file_png} and {output_file_pdf}")
    
    plt.close()
    
    # Reset font settings
    plt.rcParams['font.family'] = plt.rcParamsDefault['font.family']
    plt.rcParams['font.serif'] = plt.rcParamsDefault['font.serif']
    plt.rcParams['mathtext.fontset'] = plt.rcParamsDefault['mathtext.fontset']
    
    # Print summary statistics
    print("\n" + "="*80)
    print("SUMMARY STATISTICS: Success Rate and Planning Time")
    print("="*80)
    for config_label in sorted(all_configs):
        stats = stats_summary[config_label]
        print(f"\n{config_label}:")
        print(f"  Success Rate: {stats['final_success_rate']:5.1f}% ({stats['successful_runs']}/{stats['total_runs']} runs)")
        if stats['time_to_100'] is not None:
            print(f"  Time to 100%: {stats['time_to_100']:6.4f} seconds")
        if stats['avg_time'] is not None:
            print(f"  Avg Planning Time: {stats['avg_time']:6.4f} seconds")
            print(f"  Median Planning Time: {stats['median_time']:6.4f} seconds")
    
    print(f"\n✓ All plots saved to: {output_dir}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Plot success rate progression over time from benchmark results'
    )
    parser.add_argument(
        'results_file',
        type=str,
        nargs='?',
        default='benchmark_results.csv',
        help='Path to CSV result file (default: benchmark_results.csv)'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Directory to save plots (default: plots/ in same directory as results)'
    )
    parser.add_argument(
        '--max-time',
        type=float,
        default=None,
        help='Maximum time to show on x-axis in seconds (default: auto-detect from data)'
    )
    parser.add_argument(
        '--env-name',
        type=str,
        default='Occupancy Grid',
        help='Name of the environment for plot titles (default: Occupancy Grid)'
    )
    
    args = parser.parse_args()
    
    if not os.path.exists(args.results_file):
        print(f"Error: Results file not found: {args.results_file}")
        return 1
    
    plot_success_rate_progression(args.results_file, args.output_dir, args.max_time, args.env_name)
    return 0

if __name__ == '__main__':
    exit(main())

