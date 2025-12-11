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
    edges = []  # List of (x1, y1, x2, y2) tuples for edges
    samples_data = []  # List of dicts with all sample info
    
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
                
                sample_info = {
                    'x': x, 'y': y, 'is_valid': is_valid,
                    'sampler': sampler, 'was_connected': was_connected
                }
                samples_data.append(sample_info)
                
                if is_valid:
                    valid_samples_x.append(x)
                    valid_samples_y.append(y)
                else:
                    invalid_samples_x.append(x)
                    invalid_samples_y.append(y)
                
                # Add edge if connected
                # Always add edge if was_connected is true, even if nearest is (0,0) (start state)
                if was_connected:
                    edges.append((nearest_x, nearest_y, x, y))
                    
    except FileNotFoundError:
        return None, None, None, None, [], []
    except Exception as e:
        print(f"Error reading samples file: {e}")
        return None, None, None, None, [], []
    
    return valid_samples_x, valid_samples_y, invalid_samples_x, invalid_samples_y, edges, samples_data


def visualize_bug_trap_path(path_file='bugtrap_path.csv', output_file='bugtrap_visualization.pdf', 
                            samples_file=None):
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
    if samples_file and os.path.exists(samples_file):
        valid_x, valid_y, invalid_x, invalid_y, edges, samples_data = load_samples(samples_file)
        
        # Plot 1: Main plot with tree edges and samples
        if edges:
            # Draw RRT tree edges - draw all edges (including from origin 0,0 which is valid)
            for edge in edges:
                ax1.plot([edge[0], edge[2]], [edge[1], edge[3]], 
                        'b-', linewidth=0.8, alpha=0.4, zorder=1)
        
        if valid_x is not None:
            # Plot valid samples in green
            if valid_x:
                ax1.scatter(valid_x, valid_y, c='#2ecc71', s=15, alpha=0.7, 
                          label='Valid Samples', zorder=3, edgecolors='none')
            # Plot invalid samples in red
            if invalid_x:
                ax1.scatter(invalid_x, invalid_y, c='#e74c3c', s=15, alpha=0.6,
                          label='Invalid Samples', zorder=2, edgecolors='none')
        
        # Plot 2: Secondary plot with sampler arm colors as borders
        # Also draw edges on second plot
        if edges:
            # Draw RRT tree edges on second plot too
            for edge in edges:
                ax2.plot([edge[0], edge[2]], [edge[1], edge[3]], 
                        'b-', linewidth=0.8, alpha=0.4, zorder=1)
        
        if samples_data:
            # Define sampler colors - more distinctive colors
            # Map all possible sampler types to colors
            sampler_colors = {
                'uniform': '#0066FF',      # Bright Blue
                'cylinder_up': '#FF00FF',  # Magenta
                'cylinder_down': '#FF6600', # Bright Orange
                'cylinder': '#FF00FF',      # Magenta (default cylinder to up color)
                'connected': '#00FF00',     # Bright Green (for connected samples)
                'start': '#FFFF00',         # Yellow (for start state)
                'root': '#FFFF00',          # Yellow (for root/start)
                'burnin': '#00FFFF',        # Cyan (for burn-in samples)
                'unknown': '#FF0000'         # Red (for truly unknown)
            }
            
            for sample in samples_data:
                x, y = sample['x'], sample['y']
                is_valid = sample['is_valid']
                sampler = sample['sampler']
                
                # SWAPPED: Inner (face) color based on sampler, outer (border) color based on validity
                face_color = sampler_colors.get(sampler, '#FF0000')  # Sampler color for inner
                edge_color = '#2ecc71' if is_valid else '#e74c3c'  # Validity color for border (green=valid, red=invalid)
                
                ax2.scatter(x, y, c=face_color, s=25, alpha=0.8, 
                          edgecolors=edge_color, linewidths=2.5, zorder=3)
    
    # Load and plot path on both plots
    path_plotted = False
    if os.path.exists(path_file):
        x_path, y_path, path_samplers = load_path(path_file)
        
        # Define sampler colors for path - match the border colors
        sampler_path_colors = {
            'uniform': '#0066FF',      # Bright Blue
            'cylinder_up': '#FF00FF',  # Magenta
            'cylinder_down': '#FF6600', # Bright Orange
            'unknown': '#e74c3c'        # Red (default)
        }
        
        # Plot path on both subplots with sampler colors
        for ax in [ax1, ax2]:
            # Plot path segments with different colors based on sampler
            if len(path_samplers) > 0 and len(path_samplers) == len(x_path):
                # Plot each segment with its sampler color
                for i in range(len(x_path) - 1):
                    sampler = path_samplers[i] if i < len(path_samplers) else 'unknown'
                    color = sampler_path_colors.get(sampler, '#e74c3c')
                    ax.plot([x_path[i], x_path[i+1]], [y_path[i], y_path[i+1]], 
                           '-', color=color, linewidth=2.5, zorder=5, alpha=0.8)
                
                # Plot markers at each waypoint
                for i in range(len(x_path)):
                    sampler = path_samplers[i] if i < len(path_samplers) else 'unknown'
                    color = sampler_path_colors.get(sampler, '#e74c3c')
                    ax.plot(x_path[i], y_path[i], 'o', color=color, markersize=6,
                           markerfacecolor=color, markeredgecolor='white',
                           markeredgewidth=1, zorder=6)
                
                # Add a single label for the path
                ax.plot([], [], '-', color='#e74c3c', linewidth=2.5, label='Planned Path', zorder=5)
            else:
                # Fallback: plot as single line if no sampler info
                ax.plot(x_path, y_path, 'o-', color='#e74c3c', linewidth=2.5, 
                        markersize=6, markerfacecolor='#c0392b', markeredgecolor='white',
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
    
    # Add legend for sampler colors on second plot
    if samples_data:
        from matplotlib.patches import Patch
        sampler_colors = {
            'uniform': '#0066FF',      # Bright Blue
            'cylinder_up': '#FF00FF',  # Magenta
            'cylinder_down': '#FF6600', # Bright Orange
            'cylinder': '#FF00FF',      # Magenta (default cylinder)
            'connected': '#00FF00',     # Bright Green
            'start': '#FFFF00',         # Yellow
            'root': '#FFFF00',          # Yellow
            'burnin': '#00FFFF',        # Cyan
            'unknown': '#FF0000'         # Red
        }
        # Create legend patches with swapped colors: inner=sampler, outer=validity
        legend_elements = [
            Patch(facecolor=sampler_colors['uniform'], edgecolor='#2ecc71', 
                  linewidth=2.5, label='Uniform (Blue inner, Green border=valid)'),
            Patch(facecolor=sampler_colors['cylinder_up'], edgecolor='#2ecc71', 
                  linewidth=2.5, label='Cylinder Up (Magenta inner, Green border=valid)'),
            Patch(facecolor=sampler_colors['cylinder_down'], edgecolor='#2ecc71', 
                  linewidth=2.5, label='Cylinder Down (Orange inner, Green border=valid)'),
            Patch(facecolor=sampler_colors['cylinder'], edgecolor='#2ecc71', 
                  linewidth=2.5, label='Cylinder (Magenta inner, Green border=valid)'),
            Patch(facecolor=sampler_colors['burnin'], edgecolor='#2ecc71', 
                  linewidth=2.5, label='Burn-in (Cyan inner, Green border=valid)'),
            Patch(facecolor=sampler_colors['uniform'], edgecolor='#e74c3c', 
                  linewidth=2.5, label='Invalid Sample (Red border)')
        ]
        ax2.legend(handles=legend_elements, loc='lower right', framealpha=0.95, fancybox=True, shadow=True)
    
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', bbox_inches='tight', dpi=300)
    print(f"Visualization saved to: {output_file}")
    png_file = output_file.replace('.pdf', '.png')
    plt.savefig(png_file, format='png', bbox_inches='tight', dpi=300)
    print(f"Visualization saved to: {png_file}")
    plt.close()  # Close figure instead of showing


if __name__ == '__main__':
    # Get path file from command line or use default
    path_file = sys.argv[1] if len(sys.argv) > 1 else 'bugtrap_path.csv'
    output_file = sys.argv[2] if len(sys.argv) > 2 else 'bugtrap_visualization.pdf'
    samples_file = sys.argv[3] if len(sys.argv) > 3 else 'bugtrap_samples_debug.csv'
    
    # Check if we're in the right directory
    if not os.path.exists(path_file):
        # Try looking in build directory
        build_path = os.path.join('build', path_file)
        if os.path.exists(build_path):
            path_file = build_path
        else:
            # Try parent directory
            parent_path = os.path.join('..', path_file)
            if os.path.exists(parent_path):
                path_file = parent_path
    
    # Check samples file
    if not os.path.exists(samples_file):
        samples_file = None  # Don't plot samples if file doesn't exist
    
    visualize_bug_trap_path(path_file, output_file, samples_file)
