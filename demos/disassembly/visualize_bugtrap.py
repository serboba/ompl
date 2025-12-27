#!/usr/bin/env python3
"""
Visualization script for Bug Trap path planning results.

This script creates an academic-style visualization showing:
- The bug trap obstacle configuration
- The planned path from start to goal
- Start and goal positions
"""

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to prevent hanging

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv
import sys
import os
import time

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
    
    Grid is centered at (0, 0), with Y-axis INVERTED:
    - Positive Y is BELOW the X-axis (downward)
    - Negative Y is ABOVE the X-axis (upward)
    - X-axis: positive to the right, negative to the left (unchanged)
    
    Coordinate mapping:
    - grid[0][0] maps to world (-width/2, +height/2) [top-left]
    - grid[height/2][width/2] maps to world (0, 0) - the center
    - grid[height-1][width-1] maps to world (width/2 - 1, -height/2 + 1) [bottom-right]
    
    Note: CSV row 0 is at the TOP (image coordinates), which matches our display.
    """
    # Grid is centered at (0, 0)
    # extent defines [x_min, x_max, y_min, y_max] in world coordinates
    # Y-axis is INVERTED: positive Y is below, negative Y is above
    half_width = width / 2.0
    half_height = height / 2.0
    # Y-axis: positive Y is below X-axis, negative Y is above X-axis
    # extent = [x_min, x_max, y_min, y_max] where y_min is bottom, y_max is top
    # With origin='upper', top-left = (x_min, y_max), bottom-right = (x_max, y_min)
    # So: y_max (top) = -half_height (negative Y), y_min (bottom) = half_height (positive Y)
    extent = [-half_width, half_width, half_height, -half_height]
    
    # No need to flip: CSV row 0 is at TOP, which maps to negative Y (above X-axis)
    # Display with origin at top-left (upper) to match image coordinates
    im = ax.imshow(grid, extent=extent, origin='upper', 
                   cmap='gray_r', vmin=0, vmax=100, 
                   interpolation='nearest', aspect='equal')
    
    # Add grid lines for better visibility (in world coordinates)
    tick_spacing = max(1, width // 10)
    ax.set_xticks(np.arange(-half_width, half_width + 1, tick_spacing))
    ax.set_yticks(np.arange(-half_height, half_height + 1, tick_spacing))
    ax.grid(True, color='lightgray', linestyle='-', linewidth=0.5, alpha=0.3)
    
    return im


def draw_bug_trap(ax):
    """
    Draw the bug trap obstacles on the given axes.
    
    The bug trap consists of:
    - Top wall: x in [-2, 5], y in [1, 2]
    - Bottom wall: x in [-2, 5], y in [-2, -1]
    - Back wall: x in [5, 6], y in [-2, 2]
    """
    # Top wall
    top_wall = patches.Rectangle(
        (-2.0, 1.0), 7.0, 1.0,
        linewidth=1.5, edgecolor='black', facecolor='#2c3e50', alpha=0.8
    )
    ax.add_patch(top_wall)
    
    # Bottom wall
    bottom_wall = patches.Rectangle(
        (-2.0, -2.0), 7.0, 1.0,
        linewidth=1.5, edgecolor='black', facecolor='#2c3e50', alpha=0.8
    )
    ax.add_patch(bottom_wall)
    
    # Back wall (blocking goal)
    back_wall = patches.Rectangle(
        (5.0, -2.0), 1.0, 4.0,
        linewidth=1.5, edgecolor='black', facecolor='#2c3e50', alpha=0.8
    )
    ax.add_patch(back_wall)


def load_path(filename):
    """Load path from CSV file, optionally with sampler information."""
    x_coords = []
    y_coords = []
    samplers = []  # List of sampler names for each path segment
    
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x_coords.append(float(row['x']))
                y_coords.append(float(row['y']))
                # Get sampler if available, default to 'unknown'
                samplers.append(row.get('sampler', 'unknown'))
    except FileNotFoundError:
        print(f"Error: Path file '{filename}' not found.")
        print("Please run the bug trap demo first to generate the path file.")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading path file: {e}")
        sys.exit(1)
    
    # If no samplers found, create list of 'unknown'
    if len(samplers) == 0:
        samplers = ['unknown'] * len(x_coords)
    
    return np.array(x_coords), np.array(y_coords), samplers


def load_samples(filename):
    """Load debug samples from CSV file."""
    valid_samples_x = []
    valid_samples_y = []
    invalid_samples_x = []
    invalid_samples_y = []
    burnin_valid_x = []
    burnin_valid_y = []
    burnin_invalid_x = []
    burnin_invalid_y = []
    edges = []  # List of dicts with (x1, y1, x2, y2, sampler) for edges
    samples_data = []  # List of dicts with all sample info
    burnin_radii = set()  # Track unique burn-in radii
    
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row['x'])
                y = float(row['y'])
                is_valid = int(row['is_valid']) == 1
                was_connected = int(row.get('was_connected', '0')) == 1
                nearest_x = float(row.get('nearest_x', '0'))
                nearest_y = float(row.get('nearest_y', '0'))
                sampler = row.get('sampler', 'unknown')
                phase = row.get('phase', 'planning')  # Get phase information
                
                radius = float(row.get('radius', '0'))
                
                # Get burnin_step if available
                burnin_step = row.get('burnin_step', '-1')
                
                sample_info = {
                    'x': x, 'y': y, 'is_valid': is_valid,
                    'sampler': sampler, 'was_connected': was_connected, 'phase': phase,
                    'radius': radius, 'burnin_step': burnin_step
                }
                samples_data.append(sample_info)
                
                # Separate burn-in and planning samples
                if phase == 'burnin':
                    # Track unique radii for burn-in samples
                    if radius > 0:
                        burnin_radii.add(radius)
                    if is_valid:
                        burnin_valid_x.append(x)
                        burnin_valid_y.append(y)
                    else:
                        burnin_invalid_x.append(x)
                        burnin_invalid_y.append(y)
                else:  # planning phase
                    if is_valid:
                        valid_samples_x.append(x)
                        valid_samples_y.append(y)
                    else:
                        invalid_samples_x.append(x)
                        invalid_samples_y.append(y)
                
                # Add edge if connected (only for planning phase)
                # Always add edge if was_connected is true, even if nearest is (0,0) (start state)
                # Include sampler info so edges can be colored by sampler
                if was_connected and phase == 'planning':
                    edges.append({
                        'x1': nearest_x, 'y1': nearest_y, 
                        'x2': x, 'y2': y, 
                        'sampler': sampler
                    })
                    
    except FileNotFoundError:
        return None, None, None, None, None, None, None, None, [], [], set()
    except Exception as e:
        print(f"Error reading samples file: {e}")
        return None, None, None, None, None, None, None, None, [], [], set()
    
    return valid_samples_x, valid_samples_y, invalid_samples_x, invalid_samples_y, \
           burnin_valid_x, burnin_valid_y, burnin_invalid_x, burnin_invalid_y, edges, samples_data, burnin_radii


def visualize_bug_trap_path(path_file=None, output_file=None, samples_file=None, grid_file=None, show_invalid=False):
    """
    Create academic-style visualization of bug trap path planning.
    
    Parameters:
    -----------
    path_file : str
        CSV file containing the planned path (x, y coordinates)
    output_file : str
        Output filename for the visualization
    samples_file : str
        Optional CSV file containing debug samples (valid/invalid)
    grid_file : str
        Optional CSV file containing occupancy grid (if provided, uses grid instead of bug trap)
    show_invalid : bool
        If True, show invalid samples (invalid uniform and invalid cylinder). Default: False
    
    Note: Y-axis is INVERTED in the visualization:
    - Positive Y is BELOW the X-axis (downward)
    - Negative Y is ABOVE the X-axis (upward)
    - This matches image/screen coordinates where Y increases downward
    """
    # Create figure with two subplots - increased size for better resolution
    fig = plt.figure(figsize=(20, 8), dpi=150)
    ax1 = plt.subplot(1, 2, 1)  # Main plot with tree
    ax2 = plt.subplot(1, 2, 2)  # Secondary plot with sampler colors
    
    # Track if we're using grid (for Y-axis inversion)
    use_grid = False
    grid_data = None
    grid_width = None
    grid_height = None
    
    if grid_file and os.path.exists(grid_file):
        grid_data, grid_width, grid_height, start_pos, goal_pos = load_occupancy_grid(grid_file)
        if grid_data is not None:
            use_grid = True
            # Draw occupancy grid on both plots
            draw_occupancy_grid(ax1, grid_data, grid_width, grid_height)
            draw_occupancy_grid(ax2, grid_data, grid_width, grid_height)
        else:
            print(f"Warning: Could not load grid from {grid_file}, using default bug trap")
            draw_bug_trap(ax1)
            draw_bug_trap(ax2)
    else:
        # Draw bug trap obstacles on both plots
        draw_bug_trap(ax1)
        draw_bug_trap(ax2)
    
    # Helper function - NO LONGER INVERTING Y coordinates
    # CSV coordinates are already in world coordinates (center-based)
    # The grid extent already handles the display orientation correctly
    # Define this function outside the conditional block so it's always available
    def invert_y_if_needed(y_coords):
        """Return Y coordinates as-is (no inversion needed)"""
        return y_coords
    
    # Load and plot debug samples if available
    edges = []
    samples_data = []
    burnin_radii = set()
    valid_x = None
    valid_y = None
    invalid_x = None
    invalid_y = None
    burnin_valid_x = []
    burnin_valid_y = []
    burnin_invalid_x = []
    burnin_invalid_y = []
    final_burnin_radius = None
    
    if samples_file and os.path.exists(samples_file):
        valid_x, valid_y, invalid_x, invalid_y, burnin_valid_x, burnin_valid_y, \
        burnin_invalid_x, burnin_invalid_y, edges, samples_data, burnin_radii = load_samples(samples_file)
        
        # Draw burn-in radius circles on both plots
        if burnin_radii:
            # Find the final radius from the last burn-in step
            # The radius from the last burn-in step is the actual final radius (bestRadius)
            final_burnin_radius = None
            max_burnin_step = -1
            for sample in samples_data:
                if sample.get('phase') == 'burnin':
                    try:
                        step_str = sample.get('burnin_step', '-1')
                        if step_str and step_str != '-1':
                            step = int(step_str)
                            radius = float(sample.get('radius', 0))
                            if step > max_burnin_step and radius > 0:
                                max_burnin_step = step
                                final_burnin_radius = radius
                    except (ValueError, TypeError):
                        pass
            
            # Fallback: if we couldn't find by step, use the largest radius (last step typically has largest)
            if final_burnin_radius is None:
                sorted_radii = sorted(burnin_radii, reverse=True)  # Largest to smallest
                final_burnin_radius = sorted_radii[0] if sorted_radii else None
            
            sorted_radii = sorted(burnin_radii, reverse=True)  # For drawing other circles
            
            # Start position (origin for burn-in circles)
            start_x, start_y = 0.0, 0.0
            
            # Draw final radius first (so it's on top and visible)
            for ax in [ax1, ax2]:
                # Draw final radius with orange color - visible but not blocking samples
                if final_burnin_radius is not None:
                    final_circle = plt.Circle((start_x, start_y), final_burnin_radius, 
                                            fill=False, edgecolor='#FF6600', 
                                            linewidth=2.5, alpha=0.7, zorder=9,
                                            label='Final Burn-in Radius')
                    ax.add_patch(final_circle)
                
                # Draw other radii as thin grey circles
                for radius in sorted_radii:
                    if radius != final_burnin_radius:
                        circle = plt.Circle((start_x, start_y), radius, 
                                          fill=False, color='#888888', 
                                          linewidth=0.8, alpha=0.5, zorder=0)
                        ax.add_patch(circle)
        
        # Plot 1: Main plot with tree edges only (no samples - samples go on right plot)
        if edges:
            # Draw RRT tree edges - all same color (light gray)
            # Invert Y coordinates for edges if using grid
            for edge in edges:
                y1_inv = invert_y_if_needed(edge['y1'])
                y2_inv = invert_y_if_needed(edge['y2'])
                ax1.plot([edge['x1'], edge['x2']], [y1_inv, y2_inv], 
                        '-', color='#888888', linewidth=0.8, alpha=0.4, zorder=1)
        
        # Plot 2: Secondary plot with sampler arm colors
        # Also draw edges on second plot (all same color)
        if edges:
            # Draw RRT tree edges - all same color (light gray)
            # Invert Y coordinates for edges if using grid
            for edge in edges:
                y1_inv = invert_y_if_needed(edge['y1'])
                y2_inv = invert_y_if_needed(edge['y2'])
                ax2.plot([edge['x1'], edge['x2']], [y1_inv, y2_inv], 
                        '-', color='#888888', linewidth=0.8, alpha=0.4, zorder=1)
        
        if samples_data:
            # Plot burn-in samples on second plot (ax2 only)
            # All burn-in samples use full opacity (1.0) with gray border
            # Valid = green, Invalid = red
            burnin_alpha = 1.0
            burnin_edge_color = '#808080'  # Gray border
            burnin_size = 25  # Increased size for visibility
            burnin_valid_x_list = []
            burnin_valid_y_list = []
            burnin_invalid_x_list = []
            burnin_invalid_y_list = []
            
            for sample in samples_data:
                if sample.get('phase') == 'burnin':
                    x, y = sample['x'], sample['y']
                    # Invert Y coordinate if using grid
                    y_inv = invert_y_if_needed(y)
                    is_valid = sample['is_valid']
                    
                    # Skip invalid burn-in samples if show_invalid is False
                    if not show_invalid and not is_valid:
                        continue
                    
                    if is_valid:
                        burnin_valid_x_list.append(x)
                        burnin_valid_y_list.append(y_inv)
                    else:
                        burnin_invalid_x_list.append(x)
                        burnin_invalid_y_list.append(y_inv)
            
            # Plot burn-in valid samples
            if burnin_valid_x_list:
                ax2.scatter(burnin_valid_x_list, burnin_valid_y_list, 
                          c='#2ecc71', s=burnin_size, alpha=burnin_alpha, 
                          edgecolors=burnin_edge_color, linewidths=0.5, zorder=4, marker='o',
                          label='Burn-in Valid')
            
            # Plot burn-in invalid samples (if show_invalid is True)
            if show_invalid and burnin_invalid_x_list:
                ax2.scatter(burnin_invalid_x_list, burnin_invalid_y_list, 
                          c='#e74c3c', s=burnin_size, alpha=burnin_alpha, 
                          edgecolors=burnin_edge_color, linewidths=0.5, zorder=4, marker='o',
                          label='Burn-in Invalid')
            
            # Plot planning phase samples on ax2
            # Simplified: single color per sample based on sampler + validity
            planning_valid_uniform_x = []
            planning_valid_uniform_y = []
            planning_valid_cylinder_up_x = []
            planning_valid_cylinder_up_y = []
            planning_valid_cylinder_down_x = []
            planning_valid_cylinder_down_y = []
            planning_invalid_x = []
            planning_invalid_y = []
            
            for sample in samples_data:
                # Skip burn-in samples (already plotted above)
                if sample.get('phase') == 'burnin':
                    continue
                    
                x, y = sample['x'], sample['y']
                # Invert Y coordinate if using grid
                y_inv = invert_y_if_needed(y)
                is_valid = sample['is_valid']
                sampler = sample['sampler']
                
                # Skip invalid samples if show_invalid is False
                if not is_valid:
                    if not show_invalid:
                        continue
                    planning_invalid_x.append(x)
                    planning_invalid_y.append(y_inv)
                else:
                    # Valid samples - group by sampler for efficiency
                    if sampler == 'uniform':
                        planning_valid_uniform_x.append(x)
                        planning_valid_uniform_y.append(y_inv)
                    elif sampler == 'cylinder_up':
                        planning_valid_cylinder_up_x.append(x)
                        planning_valid_cylinder_up_y.append(y_inv)
                    elif sampler == 'cylinder_down':
                        planning_valid_cylinder_down_x.append(x)
                        planning_valid_cylinder_down_y.append(y_inv)
                    else:
                        # Other samplers (start, etc.) - plot individually
                        if sampler == 'start':
                            color = '#FFFF00'  # Yellow for start
                            edge_color = '#CCCC00'
                        else:
                            color = '#808080'  # Gray for unknown
                            edge_color = 'none'
                        ax2.scatter(x, y_inv, c=color, s=50, alpha=0.95, 
                                  edgecolors=edge_color, linewidths=0.5, zorder=8, marker='o')
            
            # Plot grouped valid samples for efficiency - use higher zorder so they're visible above path
            if planning_valid_uniform_x:
                ax2.scatter(planning_valid_uniform_x, planning_valid_uniform_y, 
                          c='#0066FF', s=50, alpha=0.95, edgecolors='#0033CC', linewidths=0.5, zorder=8, marker='o',
                          label='Valid Uniform')
            if planning_valid_cylinder_up_x:
                ax2.scatter(planning_valid_cylinder_up_x, planning_valid_cylinder_up_y, 
                          c='#006400', s=50, alpha=0.95, edgecolors='#004400', linewidths=0.5, zorder=8, marker='o',
                          label='Valid Cylinder Up')
            if planning_valid_cylinder_down_x:
                ax2.scatter(planning_valid_cylinder_down_x, planning_valid_cylinder_down_y, 
                          c='#FF1493', s=50, alpha=0.95, edgecolors='#CC0066', linewidths=0.5, zorder=8, marker='o',
                          label='Valid Cylinder Down')
            
            # Plot invalid planning samples
            if show_invalid and planning_invalid_x:
                ax2.scatter(planning_invalid_x, planning_invalid_y, 
                          c='#FF0000', s=35, alpha=0.8, edgecolors='none', zorder=7, marker='o',
                          label='Invalid Samples')
    
    # Load and plot path on both plots
    path_plotted = False
    if os.path.exists(path_file):
        x_path, y_path, path_samplers = load_path(path_file)
        
        # Define sampler colors for path - match the edge colors
        sampler_path_colors = {
            'uniform': '#0066FF',      # Bright Blue
            'cylinder_up': '#006400',  # Dark Green
            'cylinder_down': '#FF1493', # Deep Pink (to match samples)
            'cylinder': '#006400',      # Dark Green (default when UP/DOWN unknown)
            'connected': '#00FF00',     # Bright Green (for connected samples)
            'start': '#FFFF00',         # Yellow (for start state)
            'root': '#FFFF00',          # Yellow (for root/start)
            'unknown': '#FF0000'        # Red (to make it obvious when sampler info is missing)
        }
        
        # Plot path on both subplots with sampler colors
        # Invert Y coordinates for path if using grid
        y_path_inv = invert_y_if_needed(y_path) if use_grid else y_path
        
        for ax in [ax1, ax2]:
            # Plot path segments with different colors based on sampler
            if len(path_samplers) > 0 and len(path_samplers) == len(x_path):
                # Plot each segment with its sampler color (use sampler of the target waypoint to match edge coloring)
                for i in range(len(x_path) - 1):
                    # Use the sampler of the target waypoint (i+1) to match edge coloring logic
                    sampler = path_samplers[i+1] if (i+1) < len(path_samplers) else path_samplers[i] if i < len(path_samplers) else 'unknown'
                    color = sampler_path_colors.get(sampler, '#808080')
                    ax.plot([x_path[i], x_path[i+1]], [y_path_inv[i], y_path_inv[i+1]], 
                           '-', color=color, linewidth=2.5, zorder=5, alpha=0.8)
                
                # Plot markers at each waypoint with sampler color
                for i in range(len(x_path)):
                    sampler = path_samplers[i] if i < len(path_samplers) else 'unknown'
                    color = sampler_path_colors.get(sampler, '#FF0000')  # Red if unknown
                    ax.plot(x_path[i], y_path_inv[i], 'o', color=color, markersize=8,
                           markerfacecolor=color, markeredgecolor='white',
                           markeredgewidth=1.5, zorder=6)
                
                # Add legend entries for path segments by sampler type
                # Only add if we have sampler info
                if len(path_samplers) > 0:
                    # Create a combined legend entry showing path with sampler colors
                    ax.plot([], [], '-', color='#0066FF', linewidth=2.5, label='Path (Uniform)', zorder=5)
                    ax.plot([], [], '-', color='#006400', linewidth=2.5, label='Path (Cylinder Up)', zorder=5)
                    ax.plot([], [], '-', color='#FF1493', linewidth=2.5, label='Path (Cylinder Down)', zorder=5)
                else:
                    ax.plot([], [], '-', color='#808080', linewidth=2.5, label='Planned Path', zorder=5)
            else:
                # Fallback: plot as single line if no sampler info
                ax.plot(x_path, y_path_inv, 'o-', color='#808080', linewidth=2.5, 
                        markersize=6, markerfacecolor='#808080', markeredgecolor='white',
                        markeredgewidth=1, label='Planned Path', zorder=5)
            
            # Highlight start and goal (only if we plotted the full path above)
            if len(x_path) > 0:
                ax.plot(x_path[0], y_path_inv[0], 's', color='#27ae60', markersize=12,
                        markerfacecolor='#2ecc71', markeredgecolor='white', markeredgewidth=2,
                        label='Start', zorder=7)
                ax.plot(x_path[-1], y_path_inv[-1], '^', color='#3498db', markersize=12,
                        markerfacecolor='#5dade2', markeredgecolor='white', markeredgewidth=2,
                        label='Goal', zorder=7)
        path_plotted = True
    else:
        print(f"Warning: Path file '{path_file}' not found. Showing only obstacles.")
    
    # Configure both subplots
    for ax, title in [(ax1, 'RRT Tree with Valid/Invalid Samples'), 
                       (ax2, 'Samples by Sampler Arm (Border Color)')]:
        if use_grid and grid_width is not None and grid_height is not None:
            # Grid is centered at (0, 0), with Y-axis INVERTED
            # Positive Y is BELOW X-axis, negative Y is ABOVE X-axis
            half_width = grid_width / 2.0
            half_height = grid_height / 2.0
            ax.set_xlim(-half_width - 0.5, half_width + 0.5)
            # Y-axis: positive Y is below (bottom), negative Y is above (top)
            # Set ylim with y_max (top) first, then y_min (bottom) to match extent
            ax.set_ylim(half_height + 0.5, -half_height - 0.5)
            ax.set_xlabel('X Position')
            ax.set_ylabel('Y Position')
        else:
            ax.set_xlim(-5, 15)
            ax.set_ylim(-5, 5)
            ax.set_xlabel('X Position (m)')
            ax.set_ylabel('Y Position (m)')
        
        ax.set_aspect('equal')
        if not use_grid:
            ax.grid(True, linestyle='--', alpha=0.3, linewidth=0.5)
        ax.set_title(title, pad=15)
        
        # Don't show legend on plot - will be saved separately
    
    # Create legend separately (will be saved to separate file)
    from matplotlib.patches import Patch, Circle
    legend_elements = []
    
    # Add final burn-in radius to legend first (so it appears early)
    if final_burnin_radius is not None:
        legend_elements.append(
            Circle((0, 0), 0.1, fill=False, color='#FF6600', linewidth=3.0, 
                  label=f'Final Burn-in Radius ({final_burnin_radius:.4f})')
        )
    
    # Add path elements
    if path_plotted:
        legend_elements.extend([
            plt.Line2D([0], [0], color='#0066FF', linewidth=2.5, label='Path (Uniform)'),
            plt.Line2D([0], [0], color='#006400', linewidth=2.5, label='Path (Cylinder Up)'),
            plt.Line2D([0], [0], color='#FF1493', linewidth=2.5, label='Path (Cylinder Down)'),
            plt.plot([], [], 's', color='#27ae60', markersize=12, markerfacecolor='#2ecc71', 
                    markeredgecolor='white', markeredgewidth=2, label='Start')[0],
            plt.plot([], [], '^', color='#3498db', markersize=12, markerfacecolor='#5dade2', 
                    markeredgecolor='white', markeredgewidth=2, label='Goal')[0]
        ])
    
    # Add sample elements
    if samples_data:
        legend_elements.extend([
            Patch(facecolor='#0066FF', edgecolor='none', 
                  label='Valid Uniform (Blue)'),
            Patch(facecolor='#006400', edgecolor='none', 
                  label='Valid Cylinder Up (Dark Green)'),
            Patch(facecolor='#FF1493', edgecolor='none', 
                  label='Valid Cylinder Down (Deep Pink)'),
            Patch(facecolor='#2ecc71', edgecolor='none', 
                  label='Burn-in Valid (Green)')
        ])
        
        # Add invalid samples to legend only if show_invalid is True
        if show_invalid:
            legend_elements.extend([
                Patch(facecolor='#FF6600', edgecolor='none', 
                      label='Invalid Cylinder (Orange)'),
                Patch(facecolor='#FF0000', edgecolor='none', 
                      label='Invalid Uniform (Red)'),
                Patch(facecolor='#e74c3c', edgecolor='none', 
                      label='Burn-in Invalid (Red)')
            ])
    
    # Save legend to separate file
    if legend_elements:
        legend_fig, legend_ax = plt.subplots(figsize=(6, 8))
        legend_ax.axis('off')
        legend_ax.legend(handles=legend_elements, loc='center', framealpha=0.95, 
                        fancybox=True, shadow=True, fontsize=11, ncol=1)
        legend_output_file = output_file.replace('.pdf', '_legend.pdf').replace('.png', '_legend.pdf')
        legend_fig.savefig(legend_output_file, format='pdf', bbox_inches='tight', dpi=300)
        legend_png_file = legend_output_file.replace('.pdf', '.png')
        legend_fig.savefig(legend_png_file, format='png', bbox_inches='tight', dpi=300)
        plt.close(legend_fig)
        print(f"Legend saved to: {legend_output_file}")
        print(f"Legend saved to: {legend_png_file}")
    
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', bbox_inches='tight', dpi=300)
    print(f"Visualization saved to: {output_file}")
    png_file = output_file.replace('.pdf', '.png')
    plt.savefig(png_file, format='png', bbox_inches='tight', dpi=300)
    print(f"Visualization saved to: {png_file}")
    plt.close()  # Close figure instead of showing
    
    return final_burnin_radius


if __name__ == '__main__':
    # All files should be in demos/disassembly/ directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    demos_disassembly_dir = script_dir  # This script is in demos/disassembly/
    
    # Parse command line arguments
    show_invalid = False
    args = []
    i = 1
    while i < len(sys.argv):
        if sys.argv[i] == '--show-invalid':
            show_invalid = True
            i += 1
        else:
            args.append(sys.argv[i])
            i += 1
    
    # Get files from command line or use defaults
    path_file = args[0] if len(args) > 0 else os.path.join(demos_disassembly_dir, 'bugtrap_path.csv')
    output_file = args[1] if len(args) > 1 else os.path.join(demos_disassembly_dir, 'bugtrap_visualization.pdf')
    samples_file = args[2] if len(args) > 2 else os.path.join(demos_disassembly_dir, 'bugtrap_samples_debug.csv')
    grid_file = args[3] if len(args) > 3 else None
    
    # Convert all paths to absolute paths
    # If already absolute, use as-is; otherwise resolve relative to current working directory
    if path_file and not os.path.isabs(path_file):
        path_file = os.path.abspath(path_file)
    if output_file and not os.path.isabs(output_file):
        output_file = os.path.abspath(output_file)
    if samples_file and not os.path.isabs(samples_file):
        samples_file = os.path.abspath(samples_file)
    if grid_file and not os.path.isabs(grid_file):
        grid_file = os.path.abspath(grid_file)
    
    # Debug: Print the actual files being used
    print(f"[DEBUG] Visualization script using files:")
    if path_file:
        if os.path.exists(path_file):
            mtime = os.path.getmtime(path_file)
            print(f"  Path: {path_file} (modified: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))})")
        else:
            print(f"  Path: {path_file} (NOT FOUND)")
    if samples_file:
        if os.path.exists(samples_file):
            mtime = os.path.getmtime(samples_file)
            print(f"  Samples: {samples_file} (modified: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))})")
        else:
            print(f"  Samples: {samples_file} (NOT FOUND)")
    if grid_file:
        print(f"  Grid: {grid_file}")
    
    # Check if files exist
    if path_file and not os.path.exists(path_file):
        print(f"Warning: Path file '{path_file}' not found. Showing only obstacles.")
        path_file = None
    
    if samples_file and not os.path.exists(samples_file):
        print(f"Warning: Samples file '{samples_file}' not found. Skipping sample visualization.")
        samples_file = None
    
    visualize_bug_trap_path(path_file, output_file, samples_file, grid_file, show_invalid)
