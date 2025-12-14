#!/usr/bin/env python3
"""
Visualization script for Bug Trap path planning results.

This script creates an academic-style visualization showing:
- The bug trap obstacle configuration
- The planned path from start to goal
- Start and goal positions
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv
import sys
import os

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
                
                sample_info = {
                    'x': x, 'y': y, 'is_valid': is_valid,
                    'sampler': sampler, 'was_connected': was_connected, 'phase': phase,
                    'radius': radius
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


def visualize_bug_trap_path(path_file=None, output_file=None, samples_file=None):
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
    """
    # Create figure with two subplots - increased size for better resolution
    fig = plt.figure(figsize=(20, 8), dpi=150)
    ax1 = plt.subplot(1, 2, 1)  # Main plot with tree
    ax2 = plt.subplot(1, 2, 2)  # Secondary plot with sampler colors
    
    # Draw bug trap obstacles on both plots
    draw_bug_trap(ax1)
    draw_bug_trap(ax2)
    
    # Load and plot debug samples if available
    edges = []
    samples_data = []
    burnin_radii = set()
    if samples_file and os.path.exists(samples_file):
        valid_x, valid_y, invalid_x, invalid_y, burnin_valid_x, burnin_valid_y, \
        burnin_invalid_x, burnin_invalid_y, edges, samples_data, burnin_radii = load_samples(samples_file)
        
        # Draw burn-in radius circles on both plots
        if burnin_radii:
            # Sort radii to find the exit radius (smallest/last radius)
            sorted_radii = sorted(burnin_radii, reverse=True)  # Largest to smallest
            exit_radius = sorted_radii[-1] if sorted_radii else None
            
            # Start position (origin for burn-in circles)
            start_x, start_y = 0.0, 0.0
            
            for ax in [ax1, ax2]:
                for radius in sorted_radii:
                    if radius == exit_radius:
                        # Exit radius: thick, visible color
                        circle = plt.Circle((start_x, start_y), radius, 
                                          fill=False, color='#FF6600', 
                                          linewidth=2.5, alpha=0.9, zorder=0)
                    else:
                        # Other radii: thin grey circles
                        circle = plt.Circle((start_x, start_y), radius, 
                                          fill=False, color='#888888', 
                                          linewidth=0.8, alpha=0.5, zorder=0)
                    ax.add_patch(circle)
        
        # Plot 1: Main plot with tree edges and samples
        if edges:
            # Draw RRT tree edges - all same color (light gray)
            for edge in edges:
                ax1.plot([edge['x1'], edge['x2']], [edge['y1'], edge['y2']], 
                        '-', color='#888888', linewidth=0.8, alpha=0.4, zorder=1)
        
        if valid_x is not None:
            # Plot burn-in samples first (so they're in the background)
            # All burn-in samples use full opacity (1.0) with gray border
            # Valid = green, Invalid = red
            burnin_alpha = 1.0
            burnin_edge_color = '#808080'  # Gray border
            burnin_size = 25  # Increased size
            if burnin_valid_x:
                ax1.scatter(burnin_valid_x, burnin_valid_y, c='#2ecc71', s=burnin_size, alpha=burnin_alpha, 
                          label='Burn-in Valid', zorder=1, edgecolors=burnin_edge_color, 
                          linewidths=0.5, marker='o')
            if burnin_invalid_x:
                ax1.scatter(burnin_invalid_x, burnin_invalid_y, c='#e74c3c', s=burnin_size, alpha=burnin_alpha,
                          label='Burn-in Invalid', zorder=1, edgecolors=burnin_edge_color,
                          linewidths=0.5, marker='o')
            
            # Plot planning phase valid samples in green
            if valid_x:
                ax1.scatter(valid_x, valid_y, c='#2ecc71', s=15, alpha=0.7, 
                          label='Valid Samples', zorder=3, edgecolors='none')
            # Plot planning phase invalid samples in red
            if invalid_x:
                ax1.scatter(invalid_x, invalid_y, c='#e74c3c', s=15, alpha=0.6,
                          label='Invalid Samples', zorder=2, edgecolors='none')
        
        # Plot 2: Secondary plot with sampler arm colors
        # Also draw edges on second plot (all same color)
        if edges:
            # Draw RRT tree edges - all same color (light gray)
            for edge in edges:
                ax2.plot([edge['x1'], edge['x2']], [edge['y1'], edge['y2']], 
                        '-', color='#888888', linewidth=0.8, alpha=0.4, zorder=1)
        
        if samples_data:
            # Plot burn-in samples on second plot too (with appropriate colors)
            # All burn-in samples use full opacity (1.0) with gray border
            # Valid = green, Invalid = red
            burnin_alpha = 1.0
            burnin_edge_color = '#808080'  # Gray border
            burnin_size = 18  # Increased size
            for sample in samples_data:
                if sample.get('phase') == 'burnin':
                    x, y = sample['x'], sample['y']
                    is_valid = sample['is_valid']
                    sampler = sample['sampler']
                    
                    # Burn-in samples: green for valid, red for invalid
                    face_color = '#2ecc71' if is_valid else '#e74c3c'
                    
                    ax2.scatter(x, y, c=face_color, s=burnin_size, alpha=burnin_alpha, 
                              edgecolors=burnin_edge_color, linewidths=0.5, zorder=2, marker='o')
            
            # Plot planning phase samples
            # Simplified: single color per sample based on sampler + validity
            for sample in samples_data:
                # Skip burn-in samples (already plotted above)
                if sample.get('phase') == 'burnin':
                    continue
                    
                x, y = sample['x'], sample['y']
                is_valid = sample['is_valid']
                sampler = sample['sampler']
                
                # Determine color based on sampler + validity combination
                if not is_valid:
                    # Invalid samples
                    if sampler == 'uniform':
                        color = '#FF0000'  # Red for invalid uniform
                    elif sampler in ['cylinder_up', 'cylinder_down', 'cylinder']:
                        color = '#FF6600'  # Orange for invalid cylinder
                    else:
                        color = '#FF0000'  # Default red
                else:
                    # Valid samples
                    if sampler == 'uniform':
                        color = '#0066FF'  # Blue for valid uniform
                    elif sampler == 'cylinder_up':
                        color = '#FF00FF'  # Magenta for valid cylinder_up
                    elif sampler == 'cylinder_down':
                        color = '#FF1493'  # Deep pink for valid cylinder_down
                    elif sampler == 'cylinder':
                        color = '#FF00FF'  # Magenta (default cylinder)
                    elif sampler == 'start':
                        color = '#FFFF00'  # Yellow for start
                    else:
                        color = '#808080'  # Gray for unknown
                
                # Draw sample with single color (no border)
                ax2.scatter(x, y, c=color, s=40, alpha=0.9, 
                          edgecolors='none', zorder=3, marker='o')
    
    # Load and plot path on both plots
    path_plotted = False
    if os.path.exists(path_file):
        x_path, y_path, path_samplers = load_path(path_file)
        
        # Define sampler colors for path - match the edge colors
        sampler_path_colors = {
            'uniform': '#0066FF',      # Bright Blue
            'cylinder_up': '#FF00FF',  # Magenta
            'cylinder_down': '#FF6600', # Bright Orange
            'connected': '#00FF00',     # Bright Green (for connected samples)
            'start': '#FFFF00',         # Yellow (for start state)
            'unknown': '#808080'        # Gray (default, not red)
        }
        
        # Plot path on both subplots with sampler colors
        for ax in [ax1, ax2]:
            # Plot path segments with different colors based on sampler
            if len(path_samplers) > 0 and len(path_samplers) == len(x_path):
                # Plot each segment with its sampler color (use sampler of the target waypoint to match edge coloring)
                for i in range(len(x_path) - 1):
                    # Use the sampler of the target waypoint (i+1) to match edge coloring logic
                    sampler = path_samplers[i+1] if (i+1) < len(path_samplers) else path_samplers[i] if i < len(path_samplers) else 'unknown'
                    color = sampler_path_colors.get(sampler, '#808080')
                    ax.plot([x_path[i], x_path[i+1]], [y_path[i], y_path[i+1]], 
                           '-', color=color, linewidth=2.5, zorder=5, alpha=0.8)
                
                # Plot markers at each waypoint
                for i in range(len(x_path)):
                    sampler = path_samplers[i] if i < len(path_samplers) else 'unknown'
                    color = sampler_path_colors.get(sampler, '#e74c3c')
                    ax.plot(x_path[i], y_path[i], 'o', color=color, markersize=6,
                           markerfacecolor=color, markeredgecolor='white',
                           markeredgewidth=1, zorder=6)
                
                # Add a single label for the path (use a neutral color for the legend entry)
                ax.plot([], [], '-', color='#808080', linewidth=2.5, label='Planned Path', zorder=5)
            else:
                # Fallback: plot as single line if no sampler info
                ax.plot(x_path, y_path, 'o-', color='#808080', linewidth=2.5, 
                        markersize=6, markerfacecolor='#808080', markeredgecolor='white',
                        markeredgewidth=1, label='Planned Path', zorder=5)
            
            # Highlight start and goal (only if we plotted the full path above)
            if len(x_path) > 0:
                ax.plot(x_path[0], y_path[0], 's', color='#27ae60', markersize=12,
                        markerfacecolor='#2ecc71', markeredgecolor='white', markeredgewidth=2,
                        label='Start', zorder=7)
                ax.plot(x_path[-1], y_path[-1], '^', color='#3498db', markersize=12,
                        markerfacecolor='#5dade2', markeredgecolor='white', markeredgewidth=2,
                        label='Goal', zorder=7)
        path_plotted = True
    else:
        print(f"Warning: Path file '{path_file}' not found. Showing only obstacles.")
    
    # Configure both subplots
    for ax, title in [(ax1, 'RRT Tree with Valid/Invalid Samples'), 
                       (ax2, 'Samples by Sampler Arm (Border Color)')]:
        ax.set_xlim(-5, 15)
        ax.set_ylim(-5, 5)
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.3, linewidth=0.5)
        ax.set_xlabel('X Position (m)', fontweight='bold')
        ax.set_ylabel('Y Position (m)', fontweight='bold')
        ax.set_title(title, fontweight='bold', pad=15)
        
        if path_plotted:
            ax.legend(loc='upper right', framealpha=0.95, fancybox=True, shadow=True)
        
        scenario_text = ("Scenario: Start inside U-shaped trap.\\n" "Planner must explore backwards to escape.")
        ax.text(0.02, 0.98, scenario_text, transform=ax.transAxes,
                fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Add legend for simplified sampler colors on second plot
    if samples_data:
        from matplotlib.patches import Patch
        
        # Create legend patches with simplified color scheme (no borders)
        legend_elements = [
            Patch(facecolor='#0066FF', edgecolor='none', 
                  label='Valid Uniform (Blue)'),
            Patch(facecolor='#FF00FF', edgecolor='none', 
                  label='Valid Cylinder Up (Magenta)'),
            Patch(facecolor='#FF1493', edgecolor='none', 
                  label='Valid Cylinder Down (Deep Pink)'),
            Patch(facecolor='#FF6600', edgecolor='none', 
                  label='Invalid Cylinder (Orange)'),
            Patch(facecolor='#FF0000', edgecolor='none', 
                  label='Invalid Uniform (Red)'),
            Patch(facecolor='#2ecc71', edgecolor='none', 
                  label='Burn-in Valid (Green)'),
            Patch(facecolor='#e74c3c', edgecolor='none', 
                  label='Burn-in Invalid (Red)')
        ]
        ax2.legend(handles=legend_elements, loc='lower right', framealpha=0.95, fancybox=True, shadow=True, fontsize=9)
    
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', bbox_inches='tight', dpi=300)
    print(f"Visualization saved to: {output_file}")
    png_file = output_file.replace('.pdf', '.png')
    plt.savefig(png_file, format='png', bbox_inches='tight', dpi=300)
    print(f"Visualization saved to: {png_file}")
    plt.close()  # Close figure instead of showing


if __name__ == '__main__':
    # All files should be in demos/disassembly/ directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    demos_disassembly_dir = script_dir  # This script is in demos/disassembly/
    
    # Get path file from command line or use default
    path_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(demos_disassembly_dir, 'bugtrap_path.csv')
    output_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(demos_disassembly_dir, 'bugtrap_visualization.pdf')
    samples_file = sys.argv[3] if len(sys.argv) > 3 else os.path.join(demos_disassembly_dir, 'bugtrap_samples_debug.csv')
    
    # If relative paths provided, make them absolute relative to demos/disassembly
    if not os.path.isabs(path_file):
        path_file = os.path.join(demos_disassembly_dir, path_file)
    if not os.path.isabs(output_file):
        output_file = os.path.join(demos_disassembly_dir, output_file)
    if not os.path.isabs(samples_file):
        samples_file = os.path.join(demos_disassembly_dir, samples_file)
    
    # Check if files exist
    if not os.path.exists(path_file):
        print(f"Warning: Path file '{path_file}' not found. Showing only obstacles.")
        path_file = None
    
    if not os.path.exists(samples_file):
        print(f"Warning: Samples file '{samples_file}' not found. Skipping sample visualization.")
        samples_file = None
    
    visualize_bug_trap_path(path_file, output_file, samples_file)
