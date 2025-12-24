#!/usr/bin/env python3
"""
Visualization script for occupancy grids.

This script visualizes all occupancy grid CSV files in the current directory.
Each grid is displayed with:
- Free space (0) in white
- Occupied space (100) in black/red
- Start position marked in green
- Goal position marked in red
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import glob

# Set matplotlib style
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'sans-serif',
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1
})


def load_occupancy_grid(filename):
    """Load occupancy grid from CSV file.
    
    Returns:
        grid_array: numpy array of the grid (0=free, 100=occupied)
        width: width of the grid
        height: height of the grid
        start: (x, y) tuple of start position
        goal: (x, y) tuple of goal position
    """
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
        print(f"Error loading occupancy grid from {filename}: {e}")
        return None, None, None, None, None
    
    if not grid:
        print(f"Warning: Empty grid in {filename}")
        return None, None, None, None, None
    
    grid_array = np.array(grid)
    height, width = grid_array.shape
    
    return grid_array, width, height, (start_x, start_y), (goal_x, goal_y)


def visualize_grid(grid_array, width, height, start, goal, filename, output_dir=None):
    """Visualize a single occupancy grid.
    
    Args:
        grid_array: numpy array of the grid
        width: width of the grid
        height: height of the grid
        start: (x, y) tuple of start position
        goal: (x, y) tuple of goal position
        filename: name of the source file (for title)
        output_dir: directory to save the visualization (None to show interactively)
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Create a display grid where:
    # - 0 (free) -> 1.0 (white)
    # - 100 (occupied) -> 0.0 (black)
    display_grid = 1.0 - (grid_array / 100.0)
    
    # Display the grid
    # Note: origin='upper' means row 0 is at the top (like image coordinates)
    im = ax.imshow(display_grid, cmap='gray', origin='upper', 
                   extent=[-width/2, width/2, -height/2, height/2],
                   interpolation='nearest')
    
    # Mark start position (green circle)
    if start[0] is not None and start[1] is not None:
        circle_start = patches.Circle((start[0], start[1]), radius=0.5, 
                                      color='green', fill=True, 
                                      label='Start', zorder=10)
        ax.add_patch(circle_start)
        # Add a small marker for better visibility
        ax.plot(start[0], start[1], 'go', markersize=12, markeredgecolor='darkgreen', 
                markeredgewidth=2, zorder=11)
    
    # Mark goal position (red circle)
    if goal[0] is not None and goal[1] is not None:
        circle_goal = patches.Circle((goal[0], goal[1]), radius=0.5, 
                                     color='red', fill=True, 
                                     label='Goal', zorder=10)
        ax.add_patch(circle_goal)
        # Add a small marker for better visibility
        ax.plot(goal[0], goal[1], 'ro', markersize=12, markeredgecolor='darkred', 
                markeredgewidth=2, zorder=11)
    
    # Set labels and title
    ax.set_xlabel('X coordinate', fontsize=12)
    ax.set_ylabel('Y coordinate', fontsize=12)
    ax.set_title(f'Occupancy Grid: {os.path.basename(filename)}', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='upper right')
    ax.set_aspect('equal')
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('Occupancy (0=free, 100=occupied)', rotation=270, labelpad=20)
    
    plt.tight_layout()
    
    # Save or show
    if output_dir:
        output_filename = os.path.join(output_dir, 
                                      os.path.splitext(os.path.basename(filename))[0] + '_visualization.png')
        plt.savefig(output_filename)
        print(f"Saved visualization to: {output_filename}")
        plt.close()
    else:
        plt.show()


def main():
    """Main function to visualize all occupancy grids in the current directory."""
    # Get current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Find all CSV files in the current directory
    grid_files = glob.glob(os.path.join(current_dir, 'occupancy_grid_*.csv'))
    
    if not grid_files:
        print(f"No occupancy grid files found in {current_dir}")
        print("Looking for files matching pattern: occupancy_grid_*.csv")
        return
    
    # Sort files for consistent ordering
    grid_files.sort()
    
    print(f"Found {len(grid_files)} occupancy grid file(s):")
    for f in grid_files:
        print(f"  - {os.path.basename(f)}")
    
    # Create output directory for visualizations
    output_dir = os.path.join(current_dir, 'visualizations')
    os.makedirs(output_dir, exist_ok=True)
    
    # Visualize each grid
    for grid_file in grid_files:
        print(f"\nProcessing: {os.path.basename(grid_file)}")
        
        grid_array, width, height, start, goal = load_occupancy_grid(grid_file)
        
        if grid_array is None:
            print(f"  Failed to load grid from {grid_file}")
            continue
        
        print(f"  Grid size: {width} x {height}")
        print(f"  Start: ({start[0]}, {start[1]})")
        print(f"  Goal: ({goal[0]}, {goal[1]})")
        print(f"  Occupied cells: {np.sum(grid_array == 100)}")
        print(f"  Free cells: {np.sum(grid_array == 0)}")
        
        visualize_grid(grid_array, width, height, start, goal, grid_file, output_dir)
    
    print(f"\nAll visualizations saved to: {output_dir}")


if __name__ == '__main__':
    main()


