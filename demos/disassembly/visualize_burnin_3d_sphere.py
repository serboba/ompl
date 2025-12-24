#!/usr/bin/env python3
"""
3D Sphere Visualization of Bisection Search for Burn-In Phase

This script creates a 3D visualization showing:
- Multiple spheres at different radii (bisection search steps)
- Samples distributed on sphere surfaces
- Valid/invalid samples colored differently
- Gradient coloring based on bisection step number
- Final accepted radius highlighted in green
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as mpatches

# Set matplotlib style
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'sans-serif',
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1,
    'axes.grid': True,
    'grid.alpha': 0.3
})


def fibonacci_sphere_samples(n_samples, radius, jitter_radius=0.39269908169872414, rng=None):
    """
    Generate Fibonacci sphere samples matching the actual implementation.
    
    Uses the exact algorithm from FibonacciSphereSampler.cpp:
    - latitude = acos(1 - 2*t) - π/2, with jitter
    - longitude = golden_angle * i mod 2π, with jitter
    """
    if rng is None:
        rng = np.random
    
    points = []
    golden_angle = np.pi * (3.0 - np.sqrt(5.0))  # Golden angle (~2.399963)
    
    for i in range(n_samples):
        # Compute latitude (matching C++ implementation)
        t = i / (n_samples - 1) if n_samples > 1 else 0
        latitude = np.arccos(1 - 2 * t) - np.pi / 2  # Latitude in [-π/2, π/2]
        
        # Add jitter
        latitude += (rng.random() - 0.5) * jitter_radius
        
        # Compute longitude using golden angle
        longitude = (golden_angle * i) % (2 * np.pi)
        longitude += (rng.random() - 0.5) * jitter_radius
        
        # Convert to Cartesian (matching C++ implementation)
        x = radius * np.cos(latitude) * np.cos(longitude)
        y = radius * np.cos(latitude) * np.sin(longitude)
        z = radius * np.sin(latitude)
        
        points.append([x, y, z])
    
    return np.array(points)


def uniform_sphere_samples(n_samples, radius, rng=None):
    """
    Generate uniform random sphere samples matching the actual implementation.
    
    Uses the exact algorithm from UniformSphereSampler.cpp:
    - z = uniform[-1, 1]
    - theta = uniform[0, 2π]
    - x = r * sqrt(1-z²) * cos(theta)
    - y = r * sqrt(1-z²) * sin(theta)
    - z = r * z
    """
    if rng is None:
        rng = np.random
    
    points = []
    for _ in range(n_samples):
        # Uniform sampling (matching C++ implementation)
        z = 2.0 * rng.random() - 1.0  # Uniform in [-1, 1]
        theta = 2.0 * np.pi * rng.random()  # Uniform in [0, 2π]
        
        # Compute radius in x-y plane
        r_xy = np.sqrt(1.0 - z * z)
        
        # Convert to Cartesian
        x = radius * r_xy * np.cos(theta)
        y = radius * r_xy * np.sin(theta)
        z_scaled = radius * z
        
        points.append([x, y, z_scaled])
    
    return np.array(points)


def simulate_bisection_steps():
    """Simulate realistic bisection search steps using actual sampling algorithms"""
    steps = []
    
    # Bisection search parameters (matching config)
    lower_bound = 0.0000001
    upper_bound = 20.0
    tolerance = 0.0001
    min_threshold = 0.45
    max_threshold = 0.50
    n_samples_per_step = 8  # adaptive_quasirandom_sample_size (use 8 for better visualization)
    jitter_radius = 0.39269908169872414  # M_PI/8 (from config)
    
    current_step = 0
    lower = lower_bound
    upper = upper_bound
    best_radius = lower
    best_validity = 0.0
    
    # Use a seeded RNG for reproducibility
    rng = np.random.RandomState(42)
    
    while (upper - lower) > tolerance and current_step < 20:
        mid = (lower + upper) / 2.0
        
        # Simulate sampling at this radius (matching actual implementation)
        # Generate samples: sampleSize_ Fibonacci + sampleSize_ Uniform = 2*sampleSize_ total
        fib_samples = fibonacci_sphere_samples(n_samples_per_step, mid, 
                                               jitter_radius=jitter_radius, rng=rng)
        uni_samples = uniform_sphere_samples(n_samples_per_step, mid, rng=rng)
        all_samples = np.vstack([fib_samples, uni_samples])
        
        # Simulate validity (realistic: smaller radius = higher validity, but with noise)
        # Assume there's a constraint direction along [0.3, 0.5, 0.8]
        constraint_dir = np.array([0.3, 0.5, 0.8])
        constraint_dir = constraint_dir / np.linalg.norm(constraint_dir)
        
        valid_samples = []
        invalid_samples = []
        
        for sample in all_samples:
            # Samples closer to constraint direction are more likely to be valid
            # Also, smaller radii have higher validity rates
            sample_dir = sample / (np.linalg.norm(sample) + 1e-10)
            alignment = np.abs(np.dot(sample_dir, constraint_dir))
            
            # Validity probability increases with alignment and decreases with radius
            # Make it more realistic: very small radii have high validity, larger ones have lower
            radius_factor = 1.0 - (mid / upper_bound) * 0.3  # Small radii favor validity
            validity_prob = 0.2 + 0.4 * alignment + 0.3 * radius_factor
            validity_prob = np.clip(validity_prob, 0.0, 1.0)
            
            is_valid = rng.random() < validity_prob
            
            if is_valid:
                valid_samples.append(sample)
            else:
                invalid_samples.append(sample)
        
        validity_rate = len(valid_samples) / len(all_samples) if len(all_samples) > 0 else 0.0
        
        # Track best radius
        if min_threshold <= validity_rate <= max_threshold:
            best_radius = mid
            best_validity = validity_rate
            steps.append({
                'step': current_step,
                'radius': mid,
                'lower': lower,
                'upper': upper,
                'valid_samples': np.array(valid_samples),
                'invalid_samples': np.array(invalid_samples),
                'validity_rate': validity_rate,
                'is_final': True,
                'decision': 'ACCEPT'
            })
            break
        elif abs(validity_rate - (min_threshold + max_threshold) / 2.0) < \
             abs(best_validity - (min_threshold + max_threshold) / 2.0):
            best_radius = mid
            best_validity = validity_rate
        
        # Bisection decision
        if validity_rate < min_threshold:
            decision = 'REDUCE'
            upper = mid
        elif validity_rate > max_threshold:
            decision = 'INCREASE'
            lower = mid
        else:
            decision = 'ACCEPT'
            best_radius = mid
            best_validity = validity_rate
            steps.append({
                'step': current_step,
                'radius': mid,
                'lower': lower,
                'upper': upper,
                'valid_samples': np.array(valid_samples),
                'invalid_samples': np.array(invalid_samples),
                'validity_rate': validity_rate,
                'is_final': True,
                'decision': decision
            })
            break
        
        steps.append({
            'step': current_step,
            'radius': mid,
            'lower': lower,
            'upper': upper,
            'valid_samples': np.array(valid_samples),
            'invalid_samples': np.array(invalid_samples),
            'validity_rate': validity_rate,
            'is_final': False,
            'decision': decision
        })
        
        current_step += 1
    
    # If we didn't find a perfect match, use the best one
    if not any(s.get('is_final', False) for s in steps) and steps:
        steps[-1]['is_final'] = True
        steps[-1]['decision'] = 'ACCEPT'
    
    return steps


def visualize_bisection_3d_sphere(output_file=None):
    """
    Create 3D visualization of bisection search on spheres.
    
    Parameters:
    - output_file: Optional output file path (PNG/PDF)
    """
    # Simulate bisection steps
    steps = simulate_bisection_steps()
    
    if not steps:
        print("Error: No bisection steps generated")
        return False
    
    # Create figure with subplots showing different views
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle('Bisection Search: Adaptive Radius Discovery on 3D Sphere', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    n_steps = len(steps)
    
    # Create a grid of subplots: show key steps
    # Show first step, middle steps, and final step
    key_steps = []
    if n_steps > 0:
        key_steps.append(0)  # First step
    if n_steps > 2:
        key_steps.append(n_steps // 2)  # Middle step
    if n_steps > 1:
        key_steps.append(n_steps - 1)  # Final step
    
    # Limit to 6 subplots max
    if len(key_steps) > 6:
        key_steps = [0, n_steps // 4, n_steps // 2, 3 * n_steps // 4, n_steps - 1]
    
    n_plots = len(key_steps)
    cols = min(3, n_plots)
    rows = (n_plots + cols - 1) // cols
    
    for idx, step_idx in enumerate(key_steps):
        step = steps[step_idx]
        ax = fig.add_subplot(rows, cols, idx + 1, projection='3d')
        
        radius = step['radius']
        step_num = step['step']
        validity_rate = step['validity_rate']
        is_final = step.get('is_final', False)
        decision = step['decision']
        
        # Draw sphere wireframe
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 30)
        x_sphere = radius * np.outer(np.cos(u), np.sin(v))
        y_sphere = radius * np.outer(np.sin(u), np.sin(v))
        z_sphere = radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        # Determine color based on step number and final status
        if is_final:
            sphere_color = '#2ecc71'  # Green for final
            sphere_alpha = 0.4
            linewidth = 3.0
        else:
            # Use distinct colors for better visibility of evolution
            # Red -> Orange -> Yellow -> Cyan -> Blue
            gradient_pos = step_num / max(n_steps - 1, 1)
            if gradient_pos < 0.2:
                sphere_color = '#e74c3c'  # Red
            elif gradient_pos < 0.4:
                sphere_color = '#f39c12'  # Orange
            elif gradient_pos < 0.6:
                sphere_color = '#f1c40f'  # Yellow
            elif gradient_pos < 0.8:
                sphere_color = '#1abc9c'  # Cyan
            else:
                sphere_color = '#3498db'  # Blue
            sphere_alpha = 0.25
            linewidth = 1.8
        
        ax.plot_wireframe(x_sphere, y_sphere, z_sphere, 
                         color=sphere_color, alpha=sphere_alpha, linewidth=linewidth)
        
        # Draw valid samples
        if len(step['valid_samples']) > 0:
            valid = step['valid_samples']
            ax.scatter(valid[:, 0], valid[:, 1], valid[:, 2],
                      c='#27ae60', s=50, alpha=0.9, marker='o', 
                      edgecolors='#1e8449', linewidths=1.0, label='Valid', zorder=6)
        
        # Draw invalid samples
        if len(step['invalid_samples']) > 0:
            invalid = step['invalid_samples']
            ax.scatter(invalid[:, 0], invalid[:, 1], invalid[:, 2],
                      c='#e74c3c', s=35, alpha=0.7, marker='x', 
                      linewidths=2.0, label='Invalid', zorder=5)
        
        # Draw origin
        ax.scatter([0], [0], [0], color='green', s=250, marker='*', 
                  edgecolors='darkgreen', linewidths=1.5, zorder=15)
        
        # Title with step info
        title = f"Step {step_num}: r={radius:.4f}\n"
        title += f"Validity: {validity_rate:.1%} | {decision}"
        if is_final:
            title += " (FINAL)"
        ax.set_title(title, fontweight='bold', fontsize=10)
        
        ax.set_xlabel('X', fontsize=9)
        ax.set_ylabel('Y', fontsize=9)
        ax.set_zlabel('Z', fontsize=9)
        ax.set_box_aspect([1, 1, 1])
        
        # Set view angle for better visualization
        ax.view_init(elev=20, azim=45)
        
        # Set equal limits
        max_radius = max(s['radius'] for s in steps)
        limit = max_radius * 1.2
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
    
    # Add legend to the last subplot
    if key_steps:
        ax.legend(loc='upper left', fontsize=8)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"3D sphere visualization saved to: {output_file}")
    else:
        plt.show()
    
    plt.close()
    return True


def visualize_bisection_evolution_3d(output_file=None):
    """
    Create a single 3D plot showing all bisection steps as nested spheres.
    
    Parameters:
    - output_file: Optional output file path (PNG/PDF)
    """
    steps = simulate_bisection_steps()
    
    if not steps:
        print("Error: No bisection steps generated")
        return False
    
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(111, projection='3d')
    
    fig.suptitle('Bisection Search: Radius Evolution on 3D Sphere', 
                 fontsize=16, fontweight='bold')
    
    n_steps = len(steps)
    max_radius = max(s['radius'] for s in steps)
    
    # Draw all spheres (nested)
    for step in steps:
        radius = step['radius']
        step_num = step['step']
        is_final = step.get('is_final', False)
        
        # Draw sphere surface
        u = np.linspace(0, 2 * np.pi, 40)
        v = np.linspace(0, np.pi, 40)
        x_sphere = radius * np.outer(np.cos(u), np.sin(v))
        y_sphere = radius * np.outer(np.sin(u), np.sin(v))
        z_sphere = radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        # Color based on step - use distinct colors for evolution visibility
        if is_final:
            color = '#2ecc71'  # Green
            alpha = 0.5
            linewidth = 3.5
        else:
            # Use distinct colors: red -> orange -> yellow -> cyan -> blue
            gradient_pos = step_num / max(n_steps - 1, 1)
            if gradient_pos < 0.2:
                color = '#e74c3c'  # Red
            elif gradient_pos < 0.4:
                color = '#f39c12'  # Orange
            elif gradient_pos < 0.6:
                color = '#f1c40f'  # Yellow
            elif gradient_pos < 0.8:
                color = '#1abc9c'  # Cyan
            else:
                color = '#3498db'  # Blue
            alpha = 0.2
            linewidth = 1.5
        
        ax.plot_surface(x_sphere, y_sphere, z_sphere, 
                       color=color, alpha=alpha, linewidth=0, 
                       edgecolor=color, linewidths=linewidth)
        
        # Draw samples for key steps to show evolution (not all to avoid clutter)
        # Show samples for first, middle, and final steps
        show_samples = (step_num == 0 or step_num == n_steps // 2 or is_final)
        
        if show_samples and len(step['valid_samples']) > 0:
            valid = step['valid_samples']
            # Use alpha based on step: final is most visible
            if is_final:
                sample_alpha = 0.8
                sample_size = 50
            else:
                sample_alpha = 0.5
                sample_size = 35
            ax.scatter(valid[:, 0], valid[:, 1], valid[:, 2],
                      c='#27ae60', s=sample_size, alpha=sample_alpha, marker='o',
                      edgecolors='#1e8449', linewidths=0.8, zorder=10)
        
        # Show invalid samples only for final step
        if is_final and len(step['invalid_samples']) > 0:
            invalid = step['invalid_samples']
            ax.scatter(invalid[:, 0], invalid[:, 1], invalid[:, 2],
                      c='#e74c3c', s=30, alpha=0.5, marker='x',
                      linewidths=1.5, zorder=9)
    
    # Draw origin
    ax.scatter([0], [0], [0], color='green', s=300, marker='*',
              edgecolors='darkgreen', linewidths=2, zorder=20)
    
    # Add text annotations for key steps
    for step in [steps[0], steps[-1]]:
        radius = step['radius']
        step_num = step['step']
        is_final = step.get('is_final', False)
        
        # Place annotation at a point on the sphere
        angle = np.pi / 4
        annot_x = radius * np.cos(angle) * np.cos(angle) * 1.1
        annot_y = radius * np.cos(angle) * np.sin(angle) * 1.1
        annot_z = radius * np.sin(angle) * 1.1
        
        label = f"Step {step_num}\nr={radius:.4f}"
        if is_final:
            label += "\n(FINAL)"
        
        ax.text(annot_x, annot_y, annot_z, label, fontsize=9, 
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
               color='black' if is_final else 'blue')
    
    ax.set_xlabel('X', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z', fontsize=12, fontweight='bold')
    ax.set_box_aspect([1, 1, 1])
    
    # Set view
    ax.view_init(elev=25, azim=45)
    
    # Set limits
    limit = max_radius * 1.3
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    
    # Add legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='#e74c3c', lw=2, label='Step 0 (Red)'),
        Line2D([0], [0], color='#f39c12', lw=2, label='Early Steps (Orange)'),
        Line2D([0], [0], color='#3498db', lw=2, label='Later Steps (Blue)'),
        Line2D([0], [0], color='#2ecc71', lw=3, label='Final Accepted Radius (Green)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#27ae60', 
               markersize=10, label='Valid Samples'),
        Line2D([0], [0], marker='x', color='#e74c3c', markersize=10, label='Invalid Samples'),
        Line2D([0], [0], marker='*', color='green', markersize=15, label='Origin')
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=10)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"3D evolution visualization saved to: {output_file}")
    else:
        plt.show()
    
    plt.close()
    return True


def main():
    """Generate 3D sphere visualizations"""
    import sys
    import os
    
    output_dir = '.'
    
    # Generate both visualizations
    print("Generating 3D sphere visualizations for bisection search...")
    print("=" * 60)
    
    print("\n1. Generating multi-view 3D sphere visualization...")
    output1 = os.path.join(output_dir, 'bisection_search_3d_spheres.png')
    visualize_bisection_3d_sphere(output1)
    
    print("\n2. Generating nested spheres evolution visualization...")
    output2 = os.path.join(output_dir, 'bisection_search_3d_evolution.png')
    visualize_bisection_evolution_3d(output2)
    
    print("\n" + "=" * 60)
    print("All 3D visualizations generated successfully!")
    print(f"Output directory: {os.path.abspath(output_dir)}")


if __name__ == '__main__':
    main()

