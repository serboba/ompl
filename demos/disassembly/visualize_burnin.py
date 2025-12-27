#!/usr/bin/env python3
"""
Visualization script for MAB-RRT burn-in phase radius search.

This script creates an academic-style visualization showing:
- Burn-in phase radius search steps (grow/shrink adaptive)
- Circles colored by step number (gradient from blue to purple)
- Final valid radius highlighted in green
- Step numbers and radius values annotated
"""

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to prevent hanging

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import csv
import sys
import os
from collections import defaultdict

# Try to import yaml for config file reading
try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

# Set matplotlib style for academic publications (matching visualize_bugtrap.py exactly)
try:
    plt.style.use('seaborn-v0_8-paper')
except:
    try:
        plt.style.use('seaborn-paper')
    except:
        pass

plt.rcParams.update({
    'font.size': 11,  # Match visualize_bugtrap.py exactly
    'font.family': 'sans-serif',
    'font.sans-serif': ['DejaVu Sans', 'Arial', 'Helvetica'],
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.titlesize': 16,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1
})


def load_occupancy_grid(filename):
    """Load occupancy grid from CSV file."""
    grid = []
    start_x, start_y = 0.0, 0.0
    goal_x, goal_y = 0.0, 0.0
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                
                # Parse header lines
                if line.startswith('#'):
                    parts = line[1:].strip().split(',')
                    if len(parts) >= 3:
                        keyword = parts[0].strip()
                        if keyword == 'start':
                            start_x = float(parts[1].strip())
                            start_y = float(parts[2].strip())
                        elif keyword == 'goal':
                            goal_x = float(parts[1].strip())
                            goal_y = float(parts[2].strip())
                    continue
                
                # Parse grid row
                row = [int(x.strip()) for x in line.split(',') if x.strip()]
                if row:
                    grid.append(row)
    except Exception as e:
        print(f"Error loading occupancy grid: {e}")
        return None, None, None, None, None
    
    if not grid:
        return None, None, None, None, None
    
    grid_array = np.array(grid)
    height, width = grid_array.shape
    
    return grid_array, width, height, (start_x, start_y), (goal_x, goal_y)


def draw_occupancy_grid(ax, grid, width, height):
    """Draw the occupancy grid on the given axes.
    
    Grid is centered at (0, 0), with Y-axis matching validity checker:
    - Grid is centered at (0, 0) in world coordinates
    - Y increases upward (mathematical coordinates)
    - CSV row 0 is at TOP (image coordinates)
    - Coordinate mapping matches validity checker:
      - World (-W/2, -H/2) → Grid[0][0] (top-left of CSV)
      - World (0, 0) → Grid[H/2][W/2] (center)
      - World (+W/2, +H/2) → Grid[H-1][W-1] (bottom-right of CSV)
    
    With origin='upper' and extent=[x_min, x_max, y_min, y_max]:
    - y_max (top) = -H/2 (negative Y, above center)
    - y_min (bottom) = H/2 (positive Y, below center)
    """
    if grid is None:
        return
    
    # Grid is centered at (0, 0)
    half_width = width / 2.0
    half_height = height / 2.0
    
    # Create extent for imshow: [left, right, bottom, top]
    # Y-axis: positive Y is below center, negative Y is above center
    # With origin='upper': top-left = (x_min, y_max), bottom-right = (x_max, y_min)
    # So: y_max (top) = -H/2, y_min (bottom) = H/2
    extent = [-half_width, half_width, half_height, -half_height]
    
    # Display grid (0 = free, 100 = obstacle)
    # Use grayscale colormap: white for free, dark gray for obstacles
    ax.imshow(grid, extent=extent, origin='upper', cmap='gray_r', 
              vmin=0, vmax=100, alpha=0.3, interpolation='nearest')


def load_config_thresholds(config_file=None):
    """
    Load validity rate thresholds from YAML config file.
    
    Parameters:
    - config_file: Path to YAML config file (default: benchmark_baseline.yaml in same directory)
    
    Returns:
    - (min_threshold, max_threshold) tuple, or (0.10, 0.50) as defaults if not found
    """
    min_threshold = 0.10  # Default
    max_threshold = 0.50  # Default
    
    if not HAS_YAML:
        return min_threshold, max_threshold
    
    # Try to find config file
    if config_file is None:
        # Look for benchmark_baseline.yaml in the same directory as the script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_file = os.path.join(script_dir, 'benchmark_baseline.yaml')
    
    if not os.path.exists(config_file):
        # Try relative to current working directory
        alt_config = os.path.join(os.getcwd(), 'benchmark_baseline.yaml')
        if os.path.exists(alt_config):
            config_file = alt_config
        else:
            return min_threshold, max_threshold
    
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        if 'adaptive_min_expected_validity_rate' in config:
            min_threshold = float(config['adaptive_min_expected_validity_rate'])
        if 'adaptive_max_expected_validity_rate' in config:
            max_threshold = float(config['adaptive_max_expected_validity_rate'])
    except Exception as e:
        print(f"Warning: Could not load thresholds from config file: {e}")
        print(f"Using default thresholds: min={min_threshold}, max={max_threshold}")
    
    return min_threshold, max_threshold


def compute_wilson_ci(k, n, z=1.96):
    """Compute Wilson confidence interval for binomial proportion.
    
    Parameters:
    - k: Number of successes (valid samples)
    - n: Total number of samples
    - z: Z-score for confidence level (default 1.96 for 95% CI)
    
    Returns:
    - (ci_low, ci_high): Confidence interval bounds
    """
    if n == 0:
        return (0.0, 1.0)
    
    p_hat = k / n
    
    # Wilson CI formula
    denom = 1.0 + (z * z) / n
    center = (p_hat + (z * z) / (2.0 * n)) / denom
    half = (z / denom) * np.sqrt((p_hat * (1.0 - p_hat) / n) + (z * z) / (4.0 * n * n))
    
    ci_low = max(0.0, center - half)
    ci_high = min(1.0, center + half)
    
    return (ci_low, ci_high)


def load_burnin_samples(filename, config_file=None):
    """Load burn-in samples from CSV and organize by burn-in step numbers.
    
    Parameters:
    - filename: Path to CSV file with burn-in samples
    - config_file: Optional path to YAML config file for thresholds
    """
    # Load thresholds from config for step type determination (not used here but available)
    min_threshold, max_threshold = load_config_thresholds(config_file)
    
    # Load Wilson CI parameters from config if available
    wilson_z_score = 1.96  # Default 95% CI
    if config_file and HAS_YAML:
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                if config and 'adaptive_wilson_ci_z_score' in config:
                    wilson_z_score = config['adaptive_wilson_ci_z_score']
        except:
            pass
    
    # Group samples by burn-in step number (from CSV)
    all_samples_by_step = defaultdict(lambda: {'valid': [], 'invalid': [], 'radius': None})
    
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                phase = row.get('phase', 'planning')
                if phase != 'burnin':
                    continue
                
                x = float(row['x'])
                y = float(row['y'])
                is_valid = int(row['is_valid']) == 1
                radius = float(row.get('radius', '0'))
                burnin_step = int(row.get('burnin_step', '-1'))
                
                # Skip if no valid burn-in step number
                if burnin_step < 0:
                    continue
                
                # Store radius for this step (should be same for all samples in step)
                if all_samples_by_step[burnin_step]['radius'] is None:
                    all_samples_by_step[burnin_step]['radius'] = radius
                
                if radius > 0:
                    if is_valid:
                        all_samples_by_step[burnin_step]['valid'].append((x, y))
                    else:
                        all_samples_by_step[burnin_step]['invalid'].append((x, y))
    except FileNotFoundError:
        print(f"Error: Samples file not found: {filename}")
        return []
    except Exception as e:
        print(f"Error reading samples file: {e}")
        return []
    
    if not all_samples_by_step:
        return []
    
    # Sort by step number to get actual burn-in sequence
    sorted_steps = sorted(all_samples_by_step.keys())
    
    burnin_steps = []
    for step_num in sorted_steps:
        step_data = all_samples_by_step[step_num]
        radius = step_data['radius'] if step_data['radius'] is not None else 0.0
        valid_samples = step_data['valid']
        invalid_samples = step_data['invalid']
        total_count = len(valid_samples) + len(invalid_samples)
        valid_count = len(valid_samples)
        
        # For grow/shrink approach, all steps are adaptive adjustments
        step_type = 'grow_shrink'
        validity_rate = valid_count / total_count if total_count > 0 else 0.0
        
        # Get thresholds from config
        min_threshold, max_threshold = load_config_thresholds(config_file)
        
        # Determine decision based on validity rate and radius change
        # For grow/shrink: shrink if validity too low, grow if validity too high
        if validity_rate < min_threshold:
            # Validity too low → shrink radius
            decision = 'shrink'
        elif validity_rate > max_threshold:
            # Validity too high → grow radius
            decision = 'grow'
        else:
            # In target range → accept (if final) or continue
            decision = 'accept'
        
        # Final step is the last one
        is_final = (step_num == sorted_steps[-1])
        
        # For grow/shrink, we don't use Wilson CI, but we can compute it for display if needed
        ci_low, ci_high = compute_wilson_ci(valid_count, total_count, wilson_z_score)
        is_inconclusive = False  # Not used in grow/shrink approach
        
        burnin_steps.append({
            'radius': radius,
            'step_num': step_num,
            'valid_samples': valid_samples,
            'invalid_samples': invalid_samples,
            'valid_count': valid_count,
            'total_count': total_count,
            'validity_rate': validity_rate,
            'ci_low': ci_low,  # Keep for optional display
            'ci_high': ci_high,  # Keep for optional display
            'is_inconclusive': is_inconclusive,
            'step_type': step_type,
            'decision': decision,  # 'shrink', 'grow', or 'accept'
            'is_final': is_final
        })
    
    # Second pass: refine decisions by checking actual radius changes
    # This helps identify the actual action taken (grow/shrink) even if validity was in range
    for i, step in enumerate(burnin_steps):
        if step.get('decision') == 'accept' and not step['is_final'] and i + 1 < len(burnin_steps):
            next_step = burnin_steps[i + 1]
            # If next radius is smaller, we shrunk
            # If next radius is larger, we grew
            if next_step['radius'] < step['radius']:
                step['decision'] = 'shrink'
            elif next_step['radius'] > step['radius']:
                step['decision'] = 'grow'
            # If same (shouldn't happen), keep 'accept'
    
    return burnin_steps


def plot_convergence_evolution(burnin_steps, output_file=None, config_file=None):
    """
    Create convergence plot showing radius and validity rate evolution over burn-in steps.
    
    Parameters:
    - burnin_steps: List of burn-in step dictionaries
    - output_file: Optional output file path
    - config_file: Optional path to YAML config file for thresholds
    """
    if not burnin_steps:
        return False
    
    # Extract data for plotting
    step_nums = [step['step_num'] for step in burnin_steps]
    radii = [step['radius'] for step in burnin_steps]
    validity_rates_raw = [step['validity_rate'] for step in burnin_steps]
    # Convert validity rates to percentage if they're in decimal (0-1 range)
    # Check if max value is <= 1.0 to determine if conversion is needed
    if validity_rates_raw and max(validity_rates_raw) <= 1.0:
        validity_rates = [v * 100.0 for v in validity_rates_raw]
    else:
        validity_rates = validity_rates_raw
    ci_lows = [step.get('ci_low', 0.0) for step in burnin_steps]
    ci_highs = [step.get('ci_high', 1.0) for step in burnin_steps]
    is_inconclusive_list = [step.get('is_inconclusive', False) for step in burnin_steps]
    
    # Load thresholds from config file
    min_threshold, max_threshold = load_config_thresholds(config_file)
    # Convert thresholds to percentage if validity rates were converted
    if validity_rates_raw and max(validity_rates_raw) <= 1.0:
        min_threshold = min_threshold * 100.0
        max_threshold = max_threshold * 100.0
    
    # Create separate figure for graph with constrained layout to prevent clipping
    # Make it much bigger for readability without zooming
    fig1, ax1 = plt.subplots(1, 1, figsize=(20, 10), constrained_layout=True)
    # No title - removed as requested
    
    # Plot: Validity rate and radius evolution
    ax1_twin = ax1.twinx()
    
    line1 = ax1.plot(step_nums, validity_rates, 'o-', color='blue', linewidth=2, 
                     markersize=10, label='Validity Rate', zorder=3)
    line2 = ax1_twin.plot(step_nums, radii, 's--', color='red', linewidth=2, 
                          markersize=10, label='Radius', zorder=3)
    
    # Threshold lines (now in percentage)
    ax1.axhline(y=min_threshold, color='red', linestyle='--', linewidth=1.5, 
                alpha=0.7, label=f'Min Threshold ({min_threshold:.1f}%)', zorder=1)
    ax1.axhline(y=max_threshold, color='green', linestyle='--', linewidth=1.5, 
                alpha=0.7, label=f'Max Threshold ({max_threshold:.1f}%)', zorder=1)
    ax1.axhspan(min_threshold, max_threshold, alpha=0.1, color='green', 
                label='Target Range', zorder=0)
    
    # Remove all step info boxes/annotations as requested
    
    # Set axis labels with much larger font sizes for readability without zooming
    ax1.set_xlabel('Iteration Number', fontsize=24, color='black')
    ax1.set_ylabel('Validity Rate (%)', fontsize=24, color='black')
    # Ensure secondary y-axis label is clearly visible on the right - also black
    ax1_twin.set_ylabel('Radius Value', fontsize=24, color='black', 
                        rotation=270, labelpad=30)
    # No title - removed as requested
    # Set y-axis limits with proper padding for percentage values
    max_validity = max(validity_rates) if validity_rates else 100.0
    ax1.set_ylim(0, max(100.0, max_validity * 1.1))
    if radii:
        ax1_twin.set_ylim(0, max(radii) * 1.2)
    ax1.grid(True, linestyle='--', alpha=0.3, linewidth=0.5, zorder=0)
    # Set tick labels to serif font with much larger font size
    ax1.tick_params(axis='both', which='major', labelsize=20)
    ax1_twin.tick_params(axis='y', which='major', labelsize=20)
    # Set serif font for tick labels (numbers) - use DejaVu Serif as it's commonly available
    # This provides a serif font similar to Times New Roman
    import matplotlib.font_manager as fm
    # Try to find a serif font (prefer DejaVu Serif, Times, or any serif)
    serif_font = None
    for font_name in ['DejaVu Serif', 'Times', 'Times New Roman', 'Liberation Serif']:
        try:
            if font_name in [f.name for f in fm.fontManager.ttflist]:
                serif_font = font_name
                break
        except:
            pass
    
    # If no specific serif found, just use serif family (will pick default)
    for label in ax1.get_xticklabels() + ax1.get_yticklabels():
        label.set_fontfamily('serif')
        if serif_font:
            try:
                label.set_fontname(serif_font)
            except:
                pass
    for label in ax1_twin.get_yticklabels():
        label.set_fontfamily('serif')
        if serif_font:
            try:
                label.set_fontname(serif_font)
            except:
                pass
    
    # Combine all legends properly to avoid overlap - matching right plot style
    # Get all lines and labels
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_twin.get_legend_handles_labels()
    
    # Combine data lines (from both axes)
    all_lines = line1 + line2
    all_labels = [l.get_label() for l in all_lines]
    
    # Add threshold lines
    threshold_lines = [
        plt.Line2D([0], [0], color='red', linestyle='--', linewidth=1.5),
        plt.Line2D([0], [0], color='green', linestyle='--', linewidth=1.5)
    ]
    threshold_labels = [
        f'Min Threshold ({min_threshold:.1f}%)', 
        f'Max Threshold ({max_threshold:.1f}%)'
    ]
    
    # Place main legend outside plot area on the left with larger font
    legend1 = ax1.legend(all_lines, all_labels, loc='upper left', 
                        bbox_to_anchor=(0.0, 1.0), framealpha=0.9, fontsize=18)
    # Place threshold legend outside plot area on the right with larger font
    legend2 = ax1.legend(threshold_lines, threshold_labels, loc='upper right',
                        bbox_to_anchor=(1.0, 1.0), framealpha=0.9, fontsize=18)
    # Add first legend back (matplotlib only keeps the last one)
    ax1.add_artist(legend1)
    
    # Save graph separately with DPI 300 (matching right plot)
    if output_file:
        graph_file = output_file.replace('.pdf', '_convergence_graph.pdf').replace('.png', '_convergence_graph.pdf')
        plt.savefig(graph_file, dpi=300, bbox_inches='tight')
        print(f"Convergence graph saved to: {graph_file}")
        if graph_file.endswith('.pdf'):
            graph_png = graph_file.replace('.pdf', '.png')
            plt.savefig(graph_png, dpi=300, bbox_inches='tight')
            print(f"Convergence graph (PNG) saved to: {graph_png}")
    
    plt.close(fig1)
    
    # Create separate figure for table
    fig2, ax2 = plt.subplots(1, 1, figsize=(14, max(8, len(burnin_steps) * 0.3)))
    ax2.axis('off')
    table_data = []
    headers = ['Step', 'Radius', 'Valid', 'Total', 'Validity', 'Decision', 'Action']
    for step in burnin_steps:
        # Format decision based on grow/shrink approach
        decision_field = step.get('decision', '')
        
        if step.get('is_final', False):
            decision = 'ACCEPT'
            action = '[OK] Final'
        elif decision_field == 'shrink':
            decision = 'SHRINK'
            action = '↓ Reduce Radius'
        elif decision_field == 'grow':
            decision = 'GROW'
            action = '↑ Increase Radius'
        elif decision_field == 'accept':
            decision = 'IN RANGE'
            action = '→ Continue'
        else:
            # Fallback: determine from validity rate
            if step['validity_rate'] < min_threshold:
                decision = 'SHRINK'
                action = '↓ Reduce Radius'
            elif step['validity_rate'] > max_threshold:
                decision = 'GROW'
                action = '↑ Increase Radius'
            else:
                decision = 'IN RANGE'
                action = '→ Continue'
        
        table_data.append([
            step['step_num'],
            f"{step['radius']:.6f}",
            step['valid_count'],
            step['total_count'],
            f"{step['validity_rate']:.3f}",
            decision,
            action
        ])
    
    table = ax2.table(cellText=table_data, colLabels=headers, 
                     cellLoc='center', loc='center',
                     colWidths=[0.08, 0.15, 0.08, 0.08, 0.10, 0.12, 0.20])
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.8)
    
    # Color code rows based on decision type
    for i, step in enumerate(burnin_steps):
        decision = step.get('decision', '')
        if step.get('is_final', False):
            color = 'lightgreen'  # Green for final accepted step
        elif decision == 'shrink' or step['validity_rate'] < min_threshold:
            color = 'lightcoral'  # Red for shrink (validity too low)
        elif decision == 'grow' or step['validity_rate'] > max_threshold:
            color = 'wheat'  # Light orange for grow (validity too high)
        else:
            color = 'lightblue'  # Blue for in range
        for j in range(len(headers)):
            table[(i+1, j)].set_facecolor(color)
    
    # Header styling
    for j in range(len(headers)):
        table[(0, j)].set_facecolor('lightblue')
        table[(0, j)].set_text_props(weight='bold')
    
    plt.tight_layout()
    
    # Save table separately
    if output_file:
        table_file = output_file.replace('.pdf', '_convergence_table.pdf').replace('.png', '_convergence_table.pdf')
        plt.savefig(table_file, dpi=300, bbox_inches='tight')
        print(f"Convergence table saved to: {table_file}")
        if table_file.endswith('.pdf'):
            table_png = table_file.replace('.pdf', '.png')
            plt.savefig(table_png, dpi=300, bbox_inches='tight')
            print(f"Convergence table (PNG) saved to: {table_png}")
    
    plt.close(fig2)
    return True


def visualize_burnin_phase(samples_file, grid_file=None, output_file=None, config_file=None):
    """
    Create academic-style visualization of burn-in phase radius search.
    
    Parameters:
    - samples_file: Path to debug samples CSV file
    - grid_file: Optional path to occupancy grid CSV file
    - output_file: Output file path (PDF/PNG)
    - config_file: Optional path to YAML config file for thresholds
    """
    # Load burn-in data
    burnin_steps = load_burnin_samples(samples_file, config_file)
    
    if not burnin_steps:
        print("Error: No burn-in samples found in the file.")
        return False
    
    # Generate convergence plot first
    print("Generating convergence plot...")
    plot_convergence_evolution(burnin_steps, output_file, config_file)
    
    # Load grid if provided
    grid = None
    grid_width = None
    grid_height = None
    if grid_file:
        grid, grid_width, grid_height, start_pos, goal_pos = load_occupancy_grid(grid_file)
    
    # Create figure with extra space for legend on the right
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    # Draw occupancy grid if available
    if grid is not None:
        draw_occupancy_grid(ax, grid, grid_width, grid_height)
    
    # Color scheme
    # Bisection steps: gradient from blue to purple based on step number
    # Final accepted radius: green
    
    # Draw circles and samples for each burn-in step
    circles = []
    circle_colors = []
    circle_alphas = []
    circle_labels = []
    
    # Determine color for each step
    n_steps = len(burnin_steps)
    # Get max radius for minimum visible radius calculation
    max_radius = max(step['radius'] for step in burnin_steps) if burnin_steps else 1.0
    
    for step in burnin_steps:
        radius = step['radius']
        step_num = step['step_num']
        step_type = step['step_type']
        is_final = step['is_final']
        
        # Create circle at origin (0, 0)
        circle = plt.Circle((0, 0), radius, fill=False)
        circles.append(circle)
        
        # Determine color based on step number (gradient from blue to purple)
        # Final accepted radius gets a distinct green color
        if is_final:
            # Final accepted radius: green for high visibility
            color = '#2ecc71'  # Green
            alpha = 0.9
            linewidth = 3.5
            label = f"Step {step_num}: Final (r={radius:.6f})"
        elif step_type == 'initial_lower':
            # Initial lower bound: red
            color = '#e74c3c'  # Red
            alpha = 0.8
            linewidth = 2.5
            label = f"Step {step_num}: Lower Bound (r={radius:.6f})"
        elif step_type == 'initial_upper':
            # Initial upper bound: purple
            color = '#9b59b6'  # Purple
            alpha = 0.8
            linewidth = 2.5
            label = f"Step {step_num}: Upper Bound (r={radius:.6f})"
        else:
            # Bisection steps: gradient of orange based on step number
            # Normalize step number to [0, 1] for gradient
            if n_steps > 1:
                # Only consider bisection steps (skip initial lower/upper)
                bisection_steps = [s for s in burnin_steps if s['step_type'] == 'bisection']
                if bisection_steps:
                    bisection_indices = [i for i, s in enumerate(burnin_steps) if s['step_type'] == 'bisection']
                    if step_num in bisection_indices:
                        bisection_pos = bisection_indices.index(step_num)
                        gradient_pos = bisection_pos / (len(bisection_steps) - 1) if len(bisection_steps) > 1 else 0.0
                    else:
                        gradient_pos = 0.0
                else:
                    gradient_pos = 0.0
            else:
                gradient_pos = 0.0
            
            # Use orange gradient: light orange to dark orange
            # Use Oranges colormap: 0.3 (light) to 0.9 (dark)
            color = plt.cm.Oranges(0.3 + 0.6 * gradient_pos)  # Light orange to dark orange gradient
            alpha = 0.7
            linewidth = 2.0
            label = f"Step {step_num}: Bisection (r={radius:.6f})"
        
        circle_colors.append(color)
        circle_alphas.append(alpha)
        circle_labels.append(label)
        
        # Draw circle with appropriate style
        # All circles are solid, but final step has thicker line
        # For very small radii (like initial lower bound), use a minimum visible radius
        display_radius = max(radius, max_radius * 0.01) if radius > 0 else max_radius * 0.01
        circle_patch = patches.Circle((0, 0), display_radius, fill=False, 
                                     edgecolor=color, linewidth=linewidth, 
                                     alpha=alpha, linestyle='-')
        ax.add_patch(circle_patch)
        
        # For very small radii, add a marker at the origin to make it visible
        if radius < max_radius * 0.05:
            ax.plot(0, 0, marker='o', color=color, markersize=8, 
                   alpha=alpha, markeredgewidth=linewidth, zorder=10)
        
        # Add step number and validity annotation
        # Place annotation at top-right of circle
        angle = np.pi / 4  # 45 degrees
        annot_x = radius * np.cos(angle) * 1.1
        annot_y = radius * np.sin(angle) * 1.1
        
        # Format validity information
        total_count = step['total_count']
        validity_rate = step['validity_rate']
        decision = step.get('decision', None)
        
        if total_count > 0:
            # Show step number and percentage: "0\n62.5%"
            validity_text = f"{step_num}\n{validity_rate:.1%}"
        else:
            # No samples (step that discarded samples)
            validity_text = f"{step_num}\n(no samples)"
        
        # Add decision annotation for bisection steps
        decision_text = ""
        decision_color = color
        if decision == 'left' or decision == 'down':
            decision_text = "↓\nradius↓"  # Going left/down (reducing radius)
            decision_color = '#3498db'  # Blue for going down
        elif decision == 'right' or decision == 'up':
            decision_text = "↑\nradius↑"  # Going right/up (increasing radius)
            decision_color = '#e67e22'  # Orange for going up
        elif decision == 'accept':
            decision_text = "✓\naccept"  # Accepted (in target range)
            decision_color = '#2ecc71'  # Green for accept
        
        # Use smaller font for step numbers and validity
        # All steps (including step 0) get annotation on the circle line
        ax.annotate(validity_text, 
                   xy=(annot_x, annot_y),
                   fontsize=8,
                   ha='center',
                   va='center',
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='white', 
                            edgecolor=color, alpha=0.9, linewidth=1.5),
                   color=color,
                   weight='bold')
        
        # Add decision annotation for bisection steps (place at bottom-left of circle)
        if decision_text and step_type == 'bisection':
            # Place decision annotation at bottom-left of circle (opposite side from validity)
            decision_angle = -np.pi / 4  # -45 degrees (bottom-left)
            decision_annot_x = radius * np.cos(decision_angle) * 1.1
            decision_annot_y = radius * np.sin(decision_angle) * 1.1
            
            ax.annotate(decision_text,
                       xy=(decision_annot_x, decision_annot_y),
                       fontsize=9,
                       ha='center',
                       va='center',
                       bbox=dict(boxstyle='round,pad=0.5', facecolor='white', 
                                edgecolor=decision_color, alpha=0.95, linewidth=2),
                       color=decision_color,
                       weight='bold')
    
    # Add annotation for final accepted radius (if different from last step annotation)
    final_step = burnin_steps[-1] if burnin_steps else None
    if final_step and final_step['total_count'] > 0:
        final_radius = final_step['radius']
        final_validity = final_step['validity_rate']
        # Place annotation at a different angle to avoid overlap
        final_angle = -np.pi / 4  # -45 degrees (bottom-left)
        final_annot_x = final_radius * np.cos(final_angle) * 1.1
        final_annot_y = final_radius * np.sin(final_angle) * 1.1
        
        final_text = f"Final\n{final_validity:.1%}"
        ax.annotate(final_text,
                   xy=(final_annot_x, final_annot_y),
                   fontsize=9,
                   ha='center',
                   va='center',
                   bbox=dict(boxstyle='round,pad=0.5', facecolor='#d5f4e6', 
                            edgecolor='#2ecc71', alpha=0.95, linewidth=2.5),
                   color='#1e8449',
                   weight='bold')
    
    # Draw samples for ALL burn-in steps to show the search process
    # Use different colors/intensities for different steps
    all_valid_x = []
    all_valid_y = []
    all_invalid_x = []
    all_invalid_y = []
    
    for step in burnin_steps:
        if step['valid_samples']:
            valid_x, valid_y = zip(*step['valid_samples'])
            all_valid_x.extend(valid_x)
            all_valid_y.extend(valid_y)
        if step['invalid_samples']:
            invalid_x, invalid_y = zip(*step['invalid_samples'])
            all_invalid_x.extend(invalid_x)
            all_invalid_y.extend(invalid_y)
    
    # Draw all valid samples with full opacity
    if all_valid_x:
        ax.scatter(all_valid_x, all_valid_y, c='#27ae60', s=30, alpha=1.0, 
                  marker='o', edgecolors='#1e8449', linewidths=0.8,
                  label=f"Valid samples (all steps, {len(all_valid_x)})", zorder=5)
    
    # Draw all invalid samples with full opacity
    if all_invalid_x:
        ax.scatter(all_invalid_x, all_invalid_y, c='#e74c3c', s=25, alpha=1.0,
                  marker='x', linewidths=1.5,
                  label=f"Invalid samples (all steps, {len(all_invalid_x)})", zorder=4)
    
    # Draw start position (origin)
    ax.plot(0, 0, 'ko', markersize=10, label='Start', zorder=6)
    ax.plot(0, 0, 'wo', markersize=6, zorder=7)
    
    # Set equal aspect ratio and limits
    max_radius = max(step['radius'] for step in burnin_steps)
    margin = max_radius * 0.2
    ax.set_xlim(-max_radius - margin, max_radius + margin)
    ax.set_ylim(-max_radius - margin, max_radius + margin)
    ax.set_aspect('equal')
    
    # Labels and title
    ax.set_xlabel('X (world coordinates)', fontweight='bold')
    ax.set_ylabel('Y (world coordinates)', fontweight='bold')
    ax.set_title('MAB-RRT Burn-In Phase: Adaptive Radius Search', 
                fontweight='bold', fontsize=16, pad=20)
    
    # Create legend with step information
    legend_elements = []
    
    # Add step type colors
    has_initial_lower = any(s['step_type'] == 'initial_lower' for s in burnin_steps)
    has_initial_upper = any(s['step_type'] == 'initial_upper' for s in burnin_steps)
    
    if has_initial_lower:
        legend_elements.append(plt.Line2D([0], [0], color='#e74c3c', lw=2.5, 
                                          label='Initial Lower Bound', linestyle='-'))
    if has_initial_upper:
        legend_elements.append(plt.Line2D([0], [0], color='#9b59b6', lw=2.5, 
                                          label='Initial Upper Bound', linestyle='-'))
    # Show gradient range for bisection steps
    legend_elements.append(plt.Line2D([0], [0], color=plt.cm.viridis(0.3), lw=2, 
                                      label='Bisection Steps (gradient)', linestyle='-', alpha=0.7))
    legend_elements.append(plt.Line2D([0], [0], color='#2ecc71', lw=3, 
                                      label='Final Accepted Radius', linestyle='-'))
    
    # Add sample markers
    if all_valid_x:
        legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', 
                                         markerfacecolor='#27ae60', markersize=8,
                                         label=f"Valid samples (all steps)"))
    if all_invalid_x:
        legend_elements.append(plt.Line2D([0], [0], marker='x', color='#e74c3c', 
                                         markersize=8, label=f"Invalid samples (all steps)"))
    
    legend_elements.append(plt.Line2D([0], [0], marker='o', color='k', 
                                     markersize=8, label='Start position'))
    
    # Don't add legend to main plot - will be saved separately
    
    # Grid
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Save figure (without legend and stats)
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Burn-in visualization saved to: {output_file}")
        
        # Also save as PNG
        png_file = output_file.replace('.pdf', '.png')
        if png_file != output_file:
            plt.savefig(png_file, dpi=300, bbox_inches='tight')
            print(f"Burn-in visualization (PNG) saved to: {png_file}")
        
        # Save legend separately
        if legend_elements:
            # Calculate figure height based on number of legend elements
            num_elements = len(legend_elements)
            fig_height = max(3, num_elements * 0.4)  # Dynamic height based on elements
            legend_fig, legend_ax = plt.subplots(figsize=(3.5, fig_height))
            legend_ax.axis('off')
            legend = legend_ax.legend(handles=legend_elements, loc='center', framealpha=0.95, 
                           fancybox=True, shadow=True, fontsize=11, ncol=1,
                           borderpad=0.5, columnspacing=0.5)
            legend_file = output_file.replace('.pdf', '_legend.pdf').replace('.png', '_legend.pdf')
            legend_fig.tight_layout(pad=0.0)
            legend_fig.savefig(legend_file, format='pdf', bbox_inches='tight', dpi=300, pad_inches=0.0)
            legend_png = legend_file.replace('.pdf', '.png')
            legend_fig.savefig(legend_png, format='png', bbox_inches='tight', dpi=300, pad_inches=0.0)
            plt.close(legend_fig)
            print(f"Legend saved to: {legend_file}")
            print(f"Legend saved to: {legend_png}")
        
        # Save stats box separately
        final_step = burnin_steps[-1] if burnin_steps else None
        stats_text = "Burn-In Statistics:\n"
        stats_text += f"Total Steps: {len(burnin_steps)}\n"
        if final_step:
            stats_text += f"Final Radius: {final_step['radius']:.4f}\n"
            stats_text += f"Final Validity Rate: {final_step['validity_rate']:.2%}\n"
            stats_text += f"Final Valid Samples: {final_step['valid_count']}/{final_step['total_count']}\n"
        
        # Count bisection steps
        bisection_count = sum(1 for s in burnin_steps if s['step_type'] == 'bisection')
        initial_lower_count = sum(1 for s in burnin_steps if s['step_type'] == 'initial_lower')
        initial_upper_count = sum(1 for s in burnin_steps if s['step_type'] == 'initial_upper')
        initial_count = initial_lower_count + initial_upper_count
        stats_text += f"Initial Lower: {initial_lower_count}, Initial Upper: {initial_upper_count}, Bisection: {bisection_count}"
        
        stats_fig, stats_ax = plt.subplots(figsize=(5, 3))
        stats_ax.axis('off')
        stats_ax.text(0.1, 0.5, stats_text, transform=stats_ax.transAxes,
                     fontsize=11, verticalalignment='center', horizontalalignment='left',
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                     family='monospace')
        stats_file = output_file.replace('.pdf', '_stats.pdf').replace('.png', '_stats.pdf')
        stats_fig.tight_layout(pad=0.1)
        stats_fig.savefig(stats_file, format='pdf', bbox_inches='tight', dpi=300, pad_inches=0.05)
        stats_png = stats_file.replace('.pdf', '.png')
        stats_fig.savefig(stats_png, format='png', bbox_inches='tight', dpi=300, pad_inches=0.05)
        plt.close(stats_fig)
        print(f"Stats box saved to: {stats_file}")
        print(f"Stats box saved to: {stats_png}")
    else:
        plt.show()
    
    plt.close()
    return True


def main():
    """Main function to handle command line arguments."""
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_burnin.py <samples_file> [grid_file] [output_file] [config_file]")
        print("  samples_file: Path to debug samples CSV file")
        print("  grid_file: Optional path to occupancy grid CSV file")
        print("  output_file: Optional output file path (default: burnin_visualization.pdf)")
        print("  config_file: Optional path to YAML config file (default: benchmark_baseline.yaml)")
        sys.exit(1)
    
    samples_file = sys.argv[1]
    grid_file = sys.argv[2] if len(sys.argv) > 2 else None
    output_file = sys.argv[3] if len(sys.argv) > 3 else None
    config_file = sys.argv[4] if len(sys.argv) > 4 else None
    
    if not output_file:
        # Generate output filename from samples filename
        base_name = os.path.splitext(os.path.basename(samples_file))[0]
        # Remove _samples_debug suffix if present
        if base_name.endswith('_samples_debug'):
            base_name = base_name[:-14]
        output_file = f"{base_name}_burnin_visualization.pdf"
    
    # Try to find config file if not provided
    if config_file is None:
        # Look for benchmark_baseline.yaml in the same directory as samples file
        samples_dir = os.path.dirname(os.path.abspath(samples_file))
        default_config = os.path.join(samples_dir, 'benchmark_baseline.yaml')
        if os.path.exists(default_config):
            config_file = default_config
        else:
            # Try script directory
            script_dir = os.path.dirname(os.path.abspath(__file__))
            default_config = os.path.join(script_dir, 'benchmark_baseline.yaml')
            if os.path.exists(default_config):
                config_file = default_config
    
    success = visualize_burnin_phase(samples_file, grid_file, output_file, config_file)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

