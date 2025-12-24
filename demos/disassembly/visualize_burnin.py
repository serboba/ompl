#!/usr/bin/env python3
"""
Visualization script for MAB-RRT burn-in phase radius search.

This script creates an academic-style visualization showing:
- Burn-in phase radius search steps (bisection search)
- Circles colored by step number (gradient from blue to purple)
- Final valid radius highlighted in green
- Step numbers and radius values annotated
"""

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

# Set matplotlib style for academic publications
try:
    plt.style.use('seaborn-v0_8-paper')
except:
    try:
        plt.style.use('seaborn-paper')
    except:
        pass

plt.rcParams.update({
    'font.size': 11,
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


def load_burnin_samples(filename, config_file=None):
    """Load burn-in samples from CSV and organize by burn-in step numbers.
    
    Parameters:
    - filename: Path to CSV file with burn-in samples
    - config_file: Optional path to YAML config file for thresholds
    """
    # Load thresholds from config for step type determination (not used here but available)
    min_threshold, max_threshold = load_config_thresholds(config_file)
    
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
        
        # For bisection search, identify step types:
        # Step 0 = initial lower bound
        # Step 1 = initial upper bound (if exists)
        # Step 2+ = bisection steps
        if step_num == 0:
            step_type = 'initial_lower'
            decision = None  # No decision for initial bounds
        elif step_num == 1 and len(sorted_steps) > 1:
            # Check if this is actually the upper bound by comparing radii
            # Upper bound should be larger than lower bound
            if len(burnin_steps) > 0 and radius > burnin_steps[0]['radius']:
                step_type = 'initial_upper'
                decision = None  # No decision for initial bounds
            else:
                step_type = 'bisection'
                decision = None  # Will determine from validity rate
        else:
            step_type = 'bisection'
            decision = None  # Will determine from validity rate
        
        validity_rate = valid_count / total_count if total_count > 0 else 0.0
        
        # Determine decision direction for bisection steps
        if step_type == 'bisection' and total_count > 0:
            # Get thresholds from config
            min_threshold, max_threshold = load_config_thresholds(config_file)
            
            # Determine decision based on validity rate
            # Note: Even if validity is in target range, if there are more steps,
            # the algorithm continued, so we show what direction it would go
            # (or if it's the last step and in range, it's accepted)
            if validity_rate < min_threshold:
                # Validity too low → need smaller radius → reduce upper bound (go left/down)
                decision = 'left'  # or 'down' - reducing radius
            elif validity_rate > max_threshold:
                # Validity too high → need larger radius → increase lower bound (go right/up)
                decision = 'right'  # or 'up' - increasing radius
            else:
                # In target range
                # If this is the final step, it's accepted
                # Otherwise, the algorithm might continue to refine
                # For visualization, we'll show 'accept' only if it's the final step
                # For intermediate steps in range, we need to infer from next step's radius
                decision = 'accept'  # Will be refined below if not final
        
        # Final step is the last one
        is_final = (step_num == sorted_steps[-1])
        
        burnin_steps.append({
            'radius': radius,
            'step_num': step_num,
            'valid_samples': valid_samples,
            'invalid_samples': invalid_samples,
            'valid_count': valid_count,
            'total_count': total_count,
            'validity_rate': validity_rate,
            'step_type': step_type,
            'decision': decision,  # 'left'/'down', 'right'/'up', 'accept', or None
            'is_final': is_final
        })
    
    # Second pass: refine decisions for bisection steps that are in target range
    # by checking if next step's radius is smaller or larger
    for i, step in enumerate(burnin_steps):
        if (step['step_type'] == 'bisection' and 
            step.get('decision') == 'accept' and 
            not step['is_final'] and
            i + 1 < len(burnin_steps)):
            next_step = burnin_steps[i + 1]
            if next_step['step_type'] == 'bisection':
                # If next radius is smaller, we went left/down
                # If next radius is larger, we went right/up
                if next_step['radius'] < step['radius']:
                    step['decision'] = 'left'
                elif next_step['radius'] > step['radius']:
                    step['decision'] = 'right'
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
    validity_rates = [step['validity_rate'] for step in burnin_steps]
    
    # Load thresholds from config file
    min_threshold, max_threshold = load_config_thresholds(config_file)
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    fig.suptitle('Burn-In Phase: Convergence Analysis (Bisection Search)', 
                 fontsize=16, fontweight='bold')
    
    # Top plot: Validity rate and radius evolution
    ax1_twin = ax1.twinx()
    line1 = ax1.plot(step_nums, validity_rates, 'o-', color='blue', linewidth=2, 
                     markersize=10, label='Validity Rate', zorder=3)
    line2 = ax1_twin.plot(step_nums, radii, 's--', color='red', linewidth=2, 
                          markersize=10, label='Radius', zorder=3)
    
    # Threshold lines
    ax1.axhline(y=min_threshold, color='red', linestyle='--', linewidth=1.5, 
                alpha=0.7, label=f'Min Threshold ({min_threshold})', zorder=1)
    ax1.axhline(y=max_threshold, color='green', linestyle='--', linewidth=1.5, 
                alpha=0.7, label=f'Max Threshold ({max_threshold})', zorder=1)
    ax1.axhspan(min_threshold, max_threshold, alpha=0.1, color='green', 
                label='Target Range', zorder=0)
    
    # Annotate each step
    for i, step in enumerate(burnin_steps):
        x = step['step_num']
        y = step['validity_rate']
        r = step['radius']
        step_type = step['step_type']
        
        # Determine annotation based on step type and validity
        if step.get('is_final', False):
            arrow_style = '✓'
            color = 'green'
            text = f'Step {x}\nValidity: {y:.3f}\nRadius: {r:.6f}\n{arrow_style} ACCEPT'
        elif y < min_threshold:
            arrow_style = '↓'
            color = 'red'
            text = f'Step {x}\nValidity: {y:.3f}\nRadius: {r:.6f}\n{arrow_style} Reduce'
        elif y > max_threshold:
            arrow_style = '↑'
            color = 'orange'
            text = f'Step {x}\nValidity: {y:.3f}\nRadius: {r:.6f}\n{arrow_style} Increase'
        else:
            arrow_style = '→'
            color = 'blue'
            text = f'Step {x}\nValidity: {y:.3f}\nRadius: {r:.6f}'
        
        # Only annotate every few steps to avoid clutter
        if i % max(1, len(burnin_steps) // 8) == 0 or step.get('is_final', False):
            ax1.annotate(text, xy=(x, y), xytext=(x + 0.3, y + 0.1),
                        bbox=dict(boxstyle='round,pad=0.5', facecolor=color, alpha=0.3),
                        arrowprops=dict(arrowstyle='->', color=color, lw=1.5),
                        fontsize=9, zorder=4)
    
    ax1.set_xlabel('Burn-In Step Number', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Validity Rate', fontsize=12, color='blue', fontweight='bold')
    ax1_twin.set_ylabel('Radius Value', fontsize=12, color='red', fontweight='bold')
    ax1.set_title('Validity Rate and Radius Evolution (Bisection Search)', 
                  fontweight='bold', fontsize=14)
    ax1.set_ylim(0, max(1.0, max(validity_rates) * 1.1))
    if radii:
        ax1_twin.set_ylim(0, max(radii) * 1.2)
    ax1.grid(True, alpha=0.3, zorder=0)
    
    # Combine legends
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper left')
    ax1.legend([plt.Line2D([0], [0], color='red', linestyle='--'), 
                plt.Line2D([0], [0], color='green', linestyle='--')],
               [f'Min Threshold ({min_threshold})', f'Max Threshold ({max_threshold})'], 
               loc='upper right')
    
    # Bottom plot: Convergence table
    ax2.axis('off')
    table_data = []
    headers = ['Step', 'Radius', 'Valid', 'Total', 'Validity Rate', 'Type', 'Decision']
    for step in burnin_steps:
        # Format decision based on step type and decision field
        step_type = step['step_type']
        decision_field = step.get('decision', None)
        
        if step.get('is_final', False):
            decision = 'ACCEPT'
        elif step_type == 'initial_lower':
            decision = 'INITIAL_LOWER'
        elif step_type == 'initial_upper':
            decision = 'INITIAL_UPPER'
        elif step_type == 'bisection':
            # Use the actual decision direction
            if decision_field == 'left':
                decision = '↓ REDUCE (left)'
            elif decision_field == 'right':
                decision = '↑ INCREASE (right)'
            elif decision_field == 'accept':
                decision = '✓ ACCEPT'
            else:
                # Fallback: determine from validity rate
                if step['validity_rate'] < min_threshold:
                    decision = '↓ REDUCE (left)'
                elif step['validity_rate'] > max_threshold:
                    decision = '↑ INCREASE (right)'
                else:
                    decision = '→ IN RANGE'
        else:
            decision = step_type.upper()
        
        table_data.append([
            step['step_num'],
            f"{step['radius']:.6f}",
            step['valid_count'],
            step['total_count'],
            f"{step['validity_rate']:.3f}",
            step['step_type'],
            decision
        ])
    
    table = ax2.table(cellText=table_data, colLabels=headers, 
                     cellLoc='center', loc='center',
                     colWidths=[0.08, 0.18, 0.1, 0.1, 0.15, 0.15, 0.15])
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.8)
    
    # Color code rows with gradient based on step number
    for i, step in enumerate(burnin_steps):
        if step.get('is_final', False):
            color = 'lightgreen'
        elif step['validity_rate'] < min_threshold:
            color = 'lightcoral'  # Red for too low validity
        elif step['validity_rate'] > max_threshold:
            color = 'lightyellow'  # Yellow for too high validity
        else:
            # Use orange gradient for bisection steps
            step_type = step.get('step_type', 'bisection')
            if step_type == 'bisection':
                bisection_steps = [s for s in burnin_steps if s.get('step_type') == 'bisection']
                if bisection_steps:
                    bisection_indices = [j for j, s in enumerate(burnin_steps) if s.get('step_type') == 'bisection']
                    if i in bisection_indices:
                        bisection_pos = bisection_indices.index(i)
                        gradient_pos = bisection_pos / (len(bisection_steps) - 1) if len(bisection_steps) > 1 else 0.0
                    else:
                        gradient_pos = 0.0
                    # Interpolate between light orange and dark orange
                    color = plt.cm.Oranges(0.3 + 0.6 * gradient_pos)
                else:
                    color = plt.cm.Oranges(0.5)  # Default orange
            elif step_type == 'initial_lower':
                color = 'mistyrose'  # Light red
            elif step_type == 'initial_upper':
                color = 'lavender'  # Light purple
            else:
                # Fallback: orange gradient
                if len(burnin_steps) > 1:
                    gradient_pos = i / (len(burnin_steps) - 1)
                else:
                    gradient_pos = 0.0
                color = plt.cm.Oranges(0.3 + 0.6 * gradient_pos)
        for j in range(len(headers)):
            table[(i+1, j)].set_facecolor(color)
    
    # Header styling
    for j in range(len(headers)):
        table[(0, j)].set_facecolor('lightblue')
        table[(0, j)].set_text_props(weight='bold')
    
    plt.tight_layout()
    
    if output_file:
        conv_file = output_file.replace('.pdf', '_convergence.pdf').replace('.png', '_convergence.png')
        plt.savefig(conv_file, dpi=300, bbox_inches='tight')
        print(f"Convergence plot saved to: {conv_file}")
        # Also save as PNG
        if conv_file.endswith('.pdf'):
            png_file = conv_file.replace('.pdf', '.png')
            plt.savefig(png_file, dpi=300, bbox_inches='tight')
            print(f"Convergence plot (PNG) saved to: {png_file}")
    else:
        plt.show()
    
    plt.close()
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
    
    # Move legend outside the plot area (top-right)
    legend = ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1), 
                       framealpha=0.9, borderaxespad=0)
    
    # Add text box with burn-in statistics (below the legend, outside plot)
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
    
    # Get legend position to place stats box below it
    legend_bbox = legend.get_window_extent(fig.canvas.get_renderer())
    legend_bbox_fig = legend_bbox.transformed(fig.transFigure.inverted())
    
    # Place stats box below legend, aligned to left edge
    stats_y = legend_bbox_fig.y0 - 0.15  # Below legend with some spacing
    stats_x = legend_bbox_fig.x0  # Aligned with legend left edge
    
    fig.text(stats_x, stats_y, stats_text, transform=fig.transFigure,
             fontsize=9, verticalalignment='top', horizontalalignment='left',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
             family='monospace')
    
    # Grid
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Save figure
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Burn-in visualization saved to: {output_file}")
        
        # Also save as PNG
        png_file = output_file.replace('.pdf', '.png')
        if png_file != output_file:
            plt.savefig(png_file, dpi=300, bbox_inches='tight')
            print(f"Burn-in visualization (PNG) saved to: {png_file}")
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

