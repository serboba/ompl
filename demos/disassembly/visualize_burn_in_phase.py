#!/usr/bin/env python3
"""
Visualization script for MAB-RRT Burn-In Phase

This script generates visualizations of the burn-in phase radius search mechanism,
including:
- Iterative radius search process
- Validity rate evolution
- Sample distribution on sphere surface
- Radius adjustment mechanism
- Constraint direction discovery
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform

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


class Arrow3D(FancyArrowPatch):
    """3D arrow for matplotlib"""
    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)


def fibonacci_sphere_samples(n_samples, radius, jitter=0.0):
    """Generate Fibonacci sphere samples (evenly distributed)"""
    points = []
    golden_angle = np.pi * (3.0 - np.sqrt(5.0))  # Golden angle
    
    for i in range(n_samples):
        t = i / (n_samples - 1) if n_samples > 1 else 0
        latitude = np.arccos(1 - 2 * t) - np.pi / 2
        
        # Add jitter
        if jitter > 0:
            latitude += (np.random.random() - 0.5) * jitter
        
        longitude = (golden_angle * i) % (2 * np.pi)
        if jitter > 0:
            longitude += (np.random.random() - 0.5) * jitter
        
        x = radius * np.cos(latitude) * np.cos(longitude)
        y = radius * np.cos(latitude) * np.sin(longitude)
        z = radius * np.sin(latitude)
        
        points.append([x, y, z])
    
    return np.array(points)


def uniform_sphere_samples(n_samples, radius):
    """Generate uniform random sphere samples"""
    points = []
    for _ in range(n_samples):
        # Uniform sampling on sphere
        theta = 2 * np.pi * np.random.random()
        phi = np.arccos(1 - 2 * np.random.random())
        
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi)
        
        points.append([x, y, z])
    
    return np.array(points)


def simulate_burn_in_iterations():
    """Simulate burn-in iterations with realistic data"""
    iterations = []
    
    # Iteration 0
    radius = 0.5
    total_samples = 256
    valid_samples = 45
    validity_rate = valid_samples / total_samples
    iterations.append({
        'iteration': 0,
        'radius': radius,
        'valid_samples': valid_samples,
        'total_samples': total_samples,
        'validity_rate': validity_rate,
        'decision': 'SHRINK',
        'new_radius': radius * np.exp(-0.1)
    })
    
    # Iteration 1
    radius = iterations[-1]['new_radius']
    valid_samples = 60
    validity_rate = valid_samples / total_samples
    iterations.append({
        'iteration': 1,
        'radius': radius,
        'valid_samples': valid_samples,
        'total_samples': total_samples,
        'validity_rate': validity_rate,
        'decision': 'SHRINK',
        'new_radius': radius * np.exp(-0.1)
    })
    
    # Iteration 2
    radius = iterations[-1]['new_radius']
    valid_samples = 80
    validity_rate = valid_samples / total_samples
    iterations.append({
        'iteration': 2,
        'radius': radius,
        'valid_samples': valid_samples,
        'total_samples': total_samples,
        'validity_rate': validity_rate,
        'decision': 'ACCEPT',
        'new_radius': radius
    })
    
    return iterations


def figure1_iterative_radius_search(output_dir='.'):
    """Figure 1: Iterative Radius Search Process (6-panel visualization)"""
    iterations = simulate_burn_in_iterations()
    
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle('Burn-In Phase: Iterative Radius Search Process', fontsize=16, fontweight='bold')
    
    # Panel 1: Initial State
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    radius = iterations[0]['radius']
    u = np.linspace(0, 2 * np.pi, 50)
    v = np.linspace(0, np.pi, 50)
    x_sphere = radius * np.outer(np.cos(u), np.sin(v))
    y_sphere = radius * np.outer(np.sin(u), np.sin(v))
    z_sphere = radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax1.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.2, color='red')
    ax1.scatter([0], [0], [0], color='green', s=200, marker='*', label='Origin')
    ax1.set_title(f'Step 0: Initial Radius = {radius:.3f}', fontweight='bold')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.legend()
    ax1.set_box_aspect([1,1,1])
    
    # Panel 2: Sample Generation
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    fib_samples = fibonacci_sphere_samples(64, radius, jitter=0.1)
    uni_samples = uniform_sphere_samples(64, radius)
    ax2.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.1, color='orange')
    ax2.scatter(fib_samples[:, 0], fib_samples[:, 1], fib_samples[:, 2], 
                c='blue', s=10, alpha=0.6, label='Fibonacci')
    ax2.scatter(uni_samples[:, 0], uni_samples[:, 1], uni_samples[:, 2], 
                c='orange', s=10, alpha=0.6, label='Uniform')
    ax2.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax2.set_title(f'Step 1: Generate Quasi-Random Samples\n(256 total: 128 Fibonacci + 128 Uniform)', fontweight='bold')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.legend()
    ax2.set_box_aspect([1,1,1])
    
    # Panel 3: Validity Testing
    ax3 = fig.add_subplot(2, 3, 3, projection='3d')
    # Simulate valid/invalid samples
    np.random.seed(42)
    all_samples = np.vstack([fib_samples, uni_samples])
    n_valid = iterations[0]['valid_samples']
    valid_indices = np.random.choice(len(all_samples), n_valid, replace=False)
    valid_samples = all_samples[valid_indices]
    invalid_samples = np.delete(all_samples, valid_indices, axis=0)
    
    ax3.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.1, color='gray')
    ax3.scatter(valid_samples[:, 0], valid_samples[:, 1], valid_samples[:, 2], 
                c='green', s=20, alpha=0.8, label='Valid')
    ax3.scatter(invalid_samples[:, 0], invalid_samples[:, 1], invalid_samples[:, 2], 
                c='red', s=20, alpha=0.8, marker='x', label='Invalid')
    # Draw lines from origin
    for vs in valid_samples[:20]:  # Show subset for clarity
        ax3.plot([0, vs[0]], [0, vs[1]], [0, vs[2]], 'g-', alpha=0.3, linewidth=0.5)
    for ivs in invalid_samples[:30]:
        ax3.plot([0, ivs[0]], [0, ivs[1]], [0, ivs[2]], 'r--', alpha=0.2, linewidth=0.5)
    ax3.scatter([0], [0], [0], color='green', s=200, marker='*')
    validity_rate = iterations[0]['validity_rate']
    ax3.set_title(f'Step 2: Test Motion Validity\nValid: {n_valid}/256 ({validity_rate*100:.1f}%)', fontweight='bold')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.legend()
    ax3.set_box_aspect([1,1,1])
    
    # Panel 4: Radius Adjustment Decision
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.axis('off')
    validity_rate = iterations[0]['validity_rate']
    min_threshold = 0.3
    max_threshold = 0.7
    
    # Decision flowchart
    y_pos = 0.8
    box_height = 0.15
    
    # Validity rate box
    rect1 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightblue', edgecolor='black', linewidth=2)
    ax4.add_patch(rect1)
    ax4.text(0.5, y_pos + box_height/2, f'Validity Rate = {validity_rate:.3f}', 
             ha='center', va='center', fontsize=12, fontweight='bold')
    
    y_pos -= 0.25
    # Decision box
    if validity_rate < min_threshold:
        color = 'lightcoral'
        decision = 'SHRINK'
        action = f'New Radius = {radius:.3f} * exp(-0.1) = {radius * np.exp(-0.1):.3f}'
    elif validity_rate > max_threshold:
        color = 'lightyellow'
        decision = 'GROW'
        action = f'New Radius = {radius:.3f} * exp(0.1) = {radius * np.exp(0.1):.3f}'
    else:
        color = 'lightgreen'
        decision = 'ACCEPT'
        action = f'Radius = {radius:.3f} (Optimal)'
    
    rect2 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor=color, edgecolor='black', linewidth=2)
    ax4.add_patch(rect2)
    ax4.text(0.5, y_pos + box_height/2, f'Decision: {decision}', 
             ha='center', va='center', fontsize=12, fontweight='bold')
    
    y_pos -= 0.25
    rect3 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightgray', edgecolor='black', linewidth=2)
    ax4.add_patch(rect3)
    ax4.text(0.5, y_pos + box_height/2, action, 
             ha='center', va='center', fontsize=11)
    
    ax4.set_xlim(0, 1)
    ax4.set_ylim(0, 1)
    ax4.set_title('Step 3: Radius Adjustment Decision', fontweight='bold', pad=20)
    
    # Panel 5: Next Iteration
    ax5 = fig.add_subplot(2, 3, 5, projection='3d')
    new_radius = iterations[1]['radius']
    u = np.linspace(0, 2 * np.pi, 50)
    v = np.linspace(0, np.pi, 50)
    x_sphere_new = new_radius * np.outer(np.cos(u), np.sin(v))
    y_sphere_new = new_radius * np.outer(np.sin(u), np.sin(v))
    z_sphere_new = new_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax5.plot_surface(x_sphere_new, y_sphere_new, z_sphere_new, alpha=0.2, color='orange')
    ax5.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax5.set_title(f'Step 4: Iterate with New Radius = {new_radius:.3f}', fontweight='bold')
    ax5.set_xlabel('X')
    ax5.set_ylabel('Y')
    ax5.set_zlabel('Z')
    ax5.set_box_aspect([1,1,1])
    
    # Panel 6: Convergence
    ax6 = fig.add_subplot(2, 3, 6, projection='3d')
    final_radius = iterations[-1]['radius']
    final_validity = iterations[-1]['validity_rate']
    u = np.linspace(0, 2 * np.pi, 50)
    v = np.linspace(0, np.pi, 50)
    x_sphere_final = final_radius * np.outer(np.cos(u), np.sin(v))
    y_sphere_final = final_radius * np.outer(np.sin(u), np.sin(v))
    z_sphere_final = final_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax6.plot_surface(x_sphere_final, y_sphere_final, z_sphere_final, alpha=0.3, color='green')
    
    # Show valid samples clustered along constraint direction (simulate)
    constraint_dir = np.array([0.3, 0.5, 0.8])
    constraint_dir = constraint_dir / np.linalg.norm(constraint_dir)
    valid_points_final = []
    for i in range(67):
        # Sample along constraint direction with some perpendicular jitter
        t = (i - 33) / 33 * final_radius
        perp = np.random.randn(3)
        perp = perp - np.dot(perp, constraint_dir) * constraint_dir
        perp = perp / np.linalg.norm(perp) * final_radius * 0.3
        point = t * constraint_dir + perp
        valid_points_final.append(point)
    valid_points_final = np.array(valid_points_final)
    
    ax6.scatter(valid_points_final[:, 0], valid_points_final[:, 1], valid_points_final[:, 2],
                c='green', s=30, alpha=0.8)
    ax6.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax6.set_title(f'Step N: Optimal Radius Found = {final_radius:.3f}\n'
                  f'Validity: {final_validity*100:.1f}%, Valid Samples: 67', fontweight='bold')
    ax6.set_xlabel('X')
    ax6.set_ylabel('Y')
    ax6.set_zlabel('Z')
    ax6.set_box_aspect([1,1,1])
    
    plt.tight_layout()
    output_path = f'{output_dir}/burn_in_figure1_iterative_radius_search.png'
    plt.savefig(output_path)
    print(f"Saved: {output_path}")
    plt.close()


def figure2_validity_rate_evolution(output_dir='.'):
    """Figure 2: Validity Rate Evolution with decision points"""
    iterations = simulate_burn_in_iterations()
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    fig.suptitle('Burn-In Phase: Validity Rate Evolution', fontsize=16, fontweight='bold')
    
    # Main graph
    iter_nums = [it['iteration'] for it in iterations]
    validity_rates = [it['validity_rate'] for it in iterations]
    radii = [it['radius'] for it in iterations]
    
    # Plot validity rate
    ax1_twin = ax1.twinx()
    line1 = ax1.plot(iter_nums, validity_rates, 'o-', color='blue', linewidth=2, 
                     markersize=10, label='Validity Rate')
    line2 = ax1_twin.plot(iter_nums, radii, 's--', color='red', linewidth=2, 
                          markersize=10, label='Radius')
    
    # Threshold lines
    ax1.axhline(y=0.3, color='red', linestyle='--', linewidth=1.5, alpha=0.7, label='Min Threshold (0.3)')
    ax1.axhline(y=0.7, color='green', linestyle='--', linewidth=1.5, alpha=0.7, label='Max Threshold (0.7)')
    ax1.axhspan(0.3, 0.7, alpha=0.1, color='green', label='Target Range')
    
    # Annotations
    for i, it in enumerate(iterations):
        x = it['iteration']
        y = it['validity_rate']
        r = it['radius']
        decision = it['decision']
        
        # Annotation box
        if decision == 'SHRINK':
            arrow_style = '↓'
            color = 'red'
            text = f'Iter {x}\nValidity: {y:.3f}\nRadius: {r:.3f}\n{arrow_style} SHRINK'
        elif decision == 'GROW':
            arrow_style = '↑'
            color = 'yellow'
            text = f'Iter {x}\nValidity: {y:.3f}\nRadius: {r:.3f}\n{arrow_style} GROW'
        else:
            arrow_style = '✓'
            color = 'green'
            text = f'Iter {x}\nValidity: {y:.3f}\nRadius: {r:.3f}\n{arrow_style} ACCEPT'
        
        ax1.annotate(text, xy=(x, y), xytext=(x + 0.3, y + 0.1),
                    bbox=dict(boxstyle='round,pad=0.5', facecolor=color, alpha=0.3),
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.5),
                    fontsize=9)
    
    ax1.set_xlabel('Iteration Number', fontsize=12)
    ax1.set_ylabel('Validity Rate', fontsize=12, color='blue')
    ax1_twin.set_ylabel('Radius Value', fontsize=12, color='red')
    ax1.set_title('Validity Rate and Radius Evolution', fontweight='bold', fontsize=14)
    ax1.set_ylim(0, 1.0)
    ax1_twin.set_ylim(0, 0.6)
    ax1.grid(True, alpha=0.3)
    
    # Combine legends
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper left')
    ax1.legend([ax1.axhline(y=0.3, color='red', linestyle='--'), 
                ax1.axhline(y=0.7, color='green', linestyle='--')],
               ['Min Threshold (0.3)', 'Max Threshold (0.7)'], loc='upper right')
    
    # Table
    ax2.axis('off')
    table_data = []
    headers = ['Iter', 'Radius', 'Valid', 'Total', 'Validity Rate', 'Decision', 'New Radius']
    for it in iterations:
        table_data.append([
            it['iteration'],
            f"{it['radius']:.3f}",
            it['valid_samples'],
            it['total_samples'],
            f"{it['validity_rate']:.3f}",
            it['decision'],
            f"{it['new_radius']:.3f}"
        ])
    
    table = ax2.table(cellText=table_data, colLabels=headers, 
                     cellLoc='center', loc='center',
                     colWidths=[0.1, 0.15, 0.1, 0.1, 0.15, 0.15, 0.15])
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2)
    
    # Color code rows
    for i, it in enumerate(iterations):
        if it['decision'] == 'SHRINK':
            color = 'lightcoral'
        elif it['decision'] == 'GROW':
            color = 'lightyellow'
        else:
            color = 'lightgreen'
        for j in range(len(headers)):
            table[(i+1, j)].set_facecolor(color)
    
    # Header styling
    for j in range(len(headers)):
        table[(0, j)].set_facecolor('lightblue')
        table[(0, j)].set_text_props(weight='bold')
    
    plt.tight_layout()
    output_path = f'{output_dir}/burn_in_figure2_validity_rate_evolution.png'
    plt.savefig(output_path)
    print(f"Saved: {output_path}")
    plt.close()


def figure3_sample_distribution(output_dir='.'):
    """Figure 3: Sample Distribution on Sphere Surface"""
    radius = 0.5
    n_samples = 128
    
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle('Burn-In Phase: Sample Distribution on Sphere Surface', fontsize=16, fontweight='bold')
    
    # View 1: Full Sphere with all samples
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    fib_samples = fibonacci_sphere_samples(n_samples, radius, jitter=0.1)
    uni_samples = uniform_sphere_samples(n_samples, radius)
    all_samples = np.vstack([fib_samples, uni_samples])
    
    u = np.linspace(0, 2 * np.pi, 50)
    v = np.linspace(0, np.pi, 50)
    x_sphere = radius * np.outer(np.cos(u), np.sin(v))
    y_sphere = radius * np.outer(np.sin(u), np.sin(v))
    z_sphere = radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax1.plot_wireframe(x_sphere, y_sphere, z_sphere, alpha=0.2, color='gray')
    ax1.scatter(fib_samples[:, 0], fib_samples[:, 1], fib_samples[:, 2], 
                c='blue', s=15, alpha=0.7, label='Fibonacci')
    ax1.scatter(uni_samples[:, 0], uni_samples[:, 1], uni_samples[:, 2], 
                c='orange', s=15, alpha=0.7, label='Uniform')
    ax1.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax1.set_title('View 1: Full Sphere (256 samples)', fontweight='bold')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.legend()
    ax1.view_init(elev=30, azim=45)
    ax1.set_box_aspect([1,1,1])
    
    # View 2: Valid samples only
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    np.random.seed(42)
    n_valid = 45
    valid_indices = np.random.choice(len(all_samples), n_valid, replace=False)
    valid_samples = all_samples[valid_indices]
    
    ax2.plot_wireframe(x_sphere, y_sphere, z_sphere, alpha=0.1, color='gray')
    ax2.scatter(valid_samples[:, 0], valid_samples[:, 1], valid_samples[:, 2],
                c='green', s=30, alpha=0.8, label='Valid')
    # Draw lines from origin
    for vs in valid_samples[:15]:
        ax2.plot([0, vs[0]], [0, vs[1]], [0, vs[2]], 'g-', alpha=0.4, linewidth=1)
    ax2.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax2.set_title(f'View 2: Valid Samples Only ({n_valid} samples)', fontweight='bold')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.legend()
    ax2.view_init(elev=30, azim=45)
    ax2.set_box_aspect([1,1,1])
    
    # View 3: Invalid samples only
    ax3 = fig.add_subplot(2, 3, 3, projection='3d')
    invalid_samples = np.delete(all_samples, valid_indices, axis=0)
    ax3.plot_wireframe(x_sphere, y_sphere, z_sphere, alpha=0.1, color='gray')
    ax3.scatter(invalid_samples[:, 0], invalid_samples[:, 1], invalid_samples[:, 2],
                c='red', s=20, alpha=0.6, marker='x', label='Invalid')
    # Draw lines from origin
    for ivs in invalid_samples[:30]:
        ax3.plot([0, ivs[0]], [0, ivs[1]], [0, ivs[2]], 'r--', alpha=0.2, linewidth=0.5)
    ax3.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax3.set_title(f'View 3: Invalid Samples Only ({len(invalid_samples)} samples)', fontweight='bold')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.legend()
    ax3.view_init(elev=30, azim=45)
    ax3.set_box_aspect([1,1,1])
    
    # View 4: Multiple radii comparison (create separate figure for clarity)
    # We'll show 2 radii in remaining subplots
    radii_list = [0.5, 0.3]
    colors_list = ['red', 'green']
    validity_rates = [0.176, 0.312]
    
    for idx, (r, color, vr) in enumerate(zip(radii_list, colors_list, validity_rates)):
        ax = fig.add_subplot(2, 3, 4 + idx, projection='3d')
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 30)
        x_s = r * np.outer(np.cos(u), np.sin(v))
        y_s = r * np.outer(np.sin(u), np.sin(v))
        z_s = r * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_wireframe(x_s, y_s, z_s, alpha=0.3, color=color, linewidth=0.5)
        
        # Simulate valid/invalid samples
        n_total = 256
        n_valid = int(n_total * vr)
        fib_s = fibonacci_sphere_samples(64, r, jitter=0.1)
        uni_s = uniform_sphere_samples(64, r)
        all_s = np.vstack([fib_s, uni_s])
        np.random.seed(42 + idx)
        valid_idx = np.random.choice(len(all_s), n_valid, replace=False)
        valid_s = all_s[valid_idx]
        invalid_s = np.delete(all_s, valid_idx, axis=0)
        
        ax.scatter(valid_s[:, 0], valid_s[:, 1], valid_s[:, 2],
                   c='green', s=15, alpha=0.8)
        ax.scatter(invalid_s[:, 0], invalid_s[:, 1], invalid_s[:, 2],
                   c='red', s=10, alpha=0.5, marker='x')
        ax.scatter([0], [0], [0], color='green', s=150, marker='*')
        ax.set_title(f'Radius = {r:.1f}\nValidity: {vr*100:.1f}%', fontweight='bold', fontsize=10)
        ax.set_xlabel('X', fontsize=8)
        ax.set_ylabel('Y', fontsize=8)
        ax.set_zlabel('Z', fontsize=8)
        ax.view_init(elev=30, azim=45)
        ax.set_box_aspect([1,1,1])
    
    plt.tight_layout()
    output_path = f'{output_dir}/burn_in_figure3_sample_distribution.png'
    plt.savefig(output_path)
    print(f"Saved: {output_path}")
    plt.close()


def figure6_radius_adjustment(output_dir='.'):
    """Figure 6: Radius Adjustment Mechanism"""
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('Burn-In Phase: Radius Adjustment Mechanism', fontsize=16, fontweight='bold')
    
    # Panel 1: Shrink Mechanism
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.axis('off')
    
    # Flowchart for shrink
    y_pos = 0.9
    box_height = 0.12
    
    # Validity too low
    rect1 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightcoral', edgecolor='black', linewidth=2)
    ax1.add_patch(rect1)
    ax1.text(0.5, y_pos + box_height/2, 'Validity Rate < 0.3\n(Too Low)', 
             ha='center', va='center', fontsize=11, fontweight='bold')
    
    y_pos -= 0.2
    # Arrow
    ax1.arrow(0.5, y_pos + 0.05, 0, -0.05, head_width=0.05, head_length=0.03, 
              fc='black', ec='black', linewidth=2)
    
    y_pos -= 0.15
    # Shrink formula
    rect2 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightblue', edgecolor='black', linewidth=2)
    ax1.add_patch(rect2)
    ax1.text(0.5, y_pos + box_height/2, 
             r'$r_{new} = r_{old} \times e^{-\alpha}$' + '\n' + 
             r'$\alpha = 0.1$ (shrinkStep)', 
             ha='center', va='center', fontsize=11)
    
    y_pos -= 0.2
    ax1.arrow(0.5, y_pos + 0.05, 0, -0.05, head_width=0.05, head_length=0.03, 
              fc='black', ec='black', linewidth=2)
    
    y_pos -= 0.15
    # Keep valid samples
    rect3 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightgreen', edgecolor='black', linewidth=2)
    ax1.add_patch(rect3)
    ax1.text(0.5, y_pos + box_height/2, 'Keep Valid Samples\n(Append to cache)', 
             ha='center', va='center', fontsize=11)
    
    # Example
    ax1.text(0.5, 0.15, 'Example: r = 0.5 → r_new = 0.5 × e^(-0.1) = 0.452', 
             ha='center', va='center', fontsize=10, style='italic')
    
    ax1.set_xlim(0, 1)
    ax1.set_ylim(0, 1)
    ax1.set_title('Shrink Mechanism', fontweight='bold', fontsize=14, pad=20)
    
    # Panel 2: Grow Mechanism
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.axis('off')
    
    y_pos = 0.9
    # Validity too high
    rect1 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightyellow', edgecolor='black', linewidth=2)
    ax2.add_patch(rect1)
    ax2.text(0.5, y_pos + box_height/2, 'Validity Rate > 0.7\n(Too High)', 
             ha='center', va='center', fontsize=11, fontweight='bold')
    
    y_pos -= 0.2
    ax2.arrow(0.5, y_pos + 0.05, 0, -0.05, head_width=0.05, head_length=0.03, 
              fc='black', ec='black', linewidth=2)
    
    y_pos -= 0.15
    # Grow formula
    rect2 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightblue', edgecolor='black', linewidth=2)
    ax2.add_patch(rect2)
    ax2.text(0.5, y_pos + box_height/2, 
             r'$r_{new} = r_{old} \times e^{\beta}$' + '\n' + 
             r'$\beta = 0.1$ (growStep)', 
             ha='center', va='center', fontsize=11)
    
    y_pos -= 0.2
    ax2.arrow(0.5, y_pos + 0.05, 0, -0.05, head_width=0.05, head_length=0.03, 
              fc='black', ec='black', linewidth=2)
    
    y_pos -= 0.15
    # Discard samples
    rect3 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightcoral', edgecolor='black', linewidth=2)
    ax2.add_patch(rect3)
    ax2.text(0.5, y_pos + box_height/2, 'Discard All Samples\n(Clear cache)', 
             ha='center', va='center', fontsize=11)
    
    # Example
    ax2.text(0.5, 0.15, 'Example: r = 0.2 → r_new = 0.2 × e^(0.1) = 0.221', 
             ha='center', va='center', fontsize=10, style='italic')
    
    ax2.set_xlim(0, 1)
    ax2.set_ylim(0, 1)
    ax2.set_title('Grow Mechanism', fontweight='bold', fontsize=14, pad=20)
    
    # Panel 3: Accept Mechanism
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.axis('off')
    
    y_pos = 0.9
    # Validity in range
    rect1 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightgreen', edgecolor='black', linewidth=2)
    ax3.add_patch(rect1)
    ax3.text(0.5, y_pos + box_height/2, '0.3 < Validity Rate ≤ 0.7\n(In Target Range)', 
             ha='center', va='center', fontsize=11, fontweight='bold')
    
    y_pos -= 0.2
    ax3.arrow(0.5, y_pos + 0.05, 0, -0.05, head_width=0.05, head_length=0.03, 
              fc='black', ec='black', linewidth=2)
    
    y_pos -= 0.15
    # Accept
    rect2 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightgreen', edgecolor='black', linewidth=2)
    ax3.add_patch(rect2)
    ax3.text(0.5, y_pos + box_height/2, 'Accept Current Radius\n(Store as bestRadius)', 
             ha='center', va='center', fontsize=11, fontweight='bold')
    
    y_pos -= 0.2
    ax3.arrow(0.5, y_pos + 0.05, 0, -0.05, head_width=0.05, head_length=0.03, 
              fc='black', ec='black', linewidth=2)
    
    y_pos -= 0.15
    # Finalize
    rect3 = mpatches.FancyBboxPatch((0.1, y_pos), 0.8, box_height,
                                     boxstyle="round,pad=0.02", 
                                     facecolor='lightblue', edgecolor='black', linewidth=2)
    ax3.add_patch(rect3)
    ax3.text(0.5, y_pos + box_height/2, 'Append All Valid Samples\n(Finalize collection)', 
             ha='center', va='center', fontsize=11)
    
    ax3.set_xlim(0, 1)
    ax3.set_ylim(0, 1)
    ax3.set_title('Accept Mechanism', fontweight='bold', fontsize=14, pad=20)
    
    # Panel 4: Exponential Adjustment Visualization
    ax4 = fig.add_subplot(2, 2, 4)
    iterations = simulate_burn_in_iterations()
    iter_nums = [it['iteration'] for it in iterations]
    radii = [it['radius'] for it in iterations]
    
    # Plot radius evolution
    ax4.semilogy(iter_nums, radii, 'o-', color='blue', linewidth=2, markersize=10, label='Radius')
    ax4.axhline(y=0.3, color='green', linestyle='--', alpha=0.5, label='Target Range (0.3-0.7)')
    ax4.axhline(y=0.7, color='green', linestyle='--', alpha=0.5)
    ax4.fill_between([-0.5, 3.5], 0.3, 0.7, alpha=0.1, color='green')
    
    # Annotate shrink/grow
    for i, it in enumerate(iterations[:-1]):
        if it['decision'] == 'SHRINK':
            ax4.annotate('SHRINK', xy=(it['iteration'], it['radius']), 
                        xytext=(it['iteration'] + 0.2, it['radius'] * 1.3),
                        arrowprops=dict(arrowstyle='->', color='red', lw=1.5),
                        fontsize=10, color='red', fontweight='bold')
    
    ax4.set_xlabel('Iteration', fontsize=12)
    ax4.set_ylabel('Radius (log scale)', fontsize=12)
    ax4.set_title('Exponential Radius Adjustment', fontweight='bold', fontsize=14)
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    ax4.set_xlim(-0.5, 2.5)
    
    plt.tight_layout()
    output_path = f'{output_dir}/burn_in_figure6_radius_adjustment.png'
    plt.savefig(output_path)
    print(f"Saved: {output_path}")
    plt.close()


def figure7_constraint_discovery(output_dir='.'):
    """Figure 7: Constraint Direction Discovery"""
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle('Burn-In Phase: Constraint Direction Discovery', fontsize=16, fontweight='bold')
    
    # Stage 1: Initial Sampling
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    radius = 0.5
    fib_samples = fibonacci_sphere_samples(64, radius, jitter=0.1)
    uni_samples = uniform_sphere_samples(64, radius)
    all_samples = np.vstack([fib_samples, uni_samples])
    ax1.scatter(all_samples[:, 0], all_samples[:, 1], all_samples[:, 2],
                c='gray', s=10, alpha=0.6)
    ax1.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax1.set_title('Stage 1: Initial Sampling\n(Random Distribution)', fontweight='bold')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.view_init(elev=30, azim=45)
    ax1.set_box_aspect([1,1,1])
    
    # Stage 2: Validity Testing
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    np.random.seed(42)
    n_valid = 45
    valid_indices = np.random.choice(len(all_samples), n_valid, replace=False)
    valid_samples = all_samples[valid_indices]
    invalid_samples = np.delete(all_samples, valid_indices, axis=0)
    ax2.scatter(valid_samples[:, 0], valid_samples[:, 1], valid_samples[:, 2],
                c='green', s=20, alpha=0.8, label='Valid')
    ax2.scatter(invalid_samples[:, 0], invalid_samples[:, 1], invalid_samples[:, 2],
                c='red', s=10, alpha=0.3, marker='x', label='Invalid')
    ax2.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax2.set_title(f'Stage 2: Validity Reveals Pattern\n(Validity: 17.6%)', fontweight='bold')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.legend()
    ax2.view_init(elev=30, azim=45)
    ax2.set_box_aspect([1,1,1])
    
    # Stage 3: Radius Refinement
    ax3 = fig.add_subplot(2, 3, 3, projection='3d')
    new_radius = 0.409
    fib_samples2 = fibonacci_sphere_samples(64, new_radius, jitter=0.1)
    uni_samples2 = uniform_sphere_samples(64, new_radius)
    all_samples2 = np.vstack([fib_samples2, uni_samples2])
    np.random.seed(43)
    n_valid2 = 80
    valid_indices2 = np.random.choice(len(all_samples2), n_valid2, replace=False)
    valid_samples2 = all_samples2[valid_indices2]
    ax3.scatter(valid_samples2[:, 0], valid_samples2[:, 1], valid_samples2[:, 2],
                c='green', s=25, alpha=0.8)
    ax3.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax3.set_title(f'Stage 3: Refinement\n(Validity: 31.2%)', fontweight='bold')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.view_init(elev=30, azim=45)
    ax3.set_box_aspect([1,1,1])
    
    # Stage 4: Valid Sample Collection
    ax4 = fig.add_subplot(2, 3, 4, projection='3d')
    # Combine valid samples from all iterations (simulate clustering)
    constraint_dir = np.array([0.3, 0.5, 0.8])
    constraint_dir = constraint_dir / np.linalg.norm(constraint_dir)
    valid_points_all = []
    for i in range(67):
        t = (i - 33) / 33 * 0.4
        perp = np.random.randn(3)
        perp = perp - np.dot(perp, constraint_dir) * constraint_dir
        perp = perp / (np.linalg.norm(perp) + 1e-10) * 0.4 * 0.3
        point = t * constraint_dir + perp
        valid_points_all.append(point)
    valid_points_all = np.array(valid_points_all)
    ax4.scatter(valid_points_all[:, 0], valid_points_all[:, 1], valid_points_all[:, 2],
                c='green', s=30, alpha=0.8)
    ax4.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax4.set_title('Stage 4: Valid Samples Collected\n(67 total, clustered)', fontweight='bold')
    ax4.set_xlabel('X')
    ax4.set_ylabel('Y')
    ax4.set_zlabel('Z')
    ax4.view_init(elev=30, azim=45)
    ax4.set_box_aspect([1,1,1])
    
    # Stage 5: PCA Analysis
    ax5 = fig.add_subplot(2, 3, 5, projection='3d')
    # Compute PCA (simplified)
    centroid = np.mean(valid_points_all, axis=0)
    centered = valid_points_all - centroid
    cov = np.cov(centered.T)
    eigenvals, eigenvecs = np.linalg.eigh(cov)
    principal_axis = eigenvecs[:, np.argmax(eigenvals)]
    # Ensure consistent direction
    if principal_axis[2] < 0:
        principal_axis = -principal_axis
    
    ax5.scatter(valid_points_all[:, 0], valid_points_all[:, 1], valid_points_all[:, 2],
                c='green', s=30, alpha=0.6)
    # Draw principal axis
    axis_length = 0.6
    ax5.plot([centroid[0] - axis_length*principal_axis[0], 
              centroid[0] + axis_length*principal_axis[0]],
             [centroid[1] - axis_length*principal_axis[1], 
              centroid[1] + axis_length*principal_axis[1]],
             [centroid[2] - axis_length*principal_axis[2], 
              centroid[2] + axis_length*principal_axis[2]],
             'r-', linewidth=4, label='PCA Axis')
    ax5.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax5.set_title('Stage 5: Principal Component Analysis\n(Axis = eigenvector)', fontweight='bold')
    ax5.set_xlabel('X')
    ax5.set_ylabel('Y')
    ax5.set_zlabel('Z')
    ax5.legend()
    ax5.view_init(elev=30, azim=45)
    ax5.set_box_aspect([1,1,1])
    
    # Stage 6: Cylinder Fitting
    ax6 = fig.add_subplot(2, 3, 6, projection='3d')
    # Draw cylinder
    cylinder_radius = 0.15
    cylinder_height = 0.8
    u_cyl = np.linspace(0, 2 * np.pi, 30)
    v_cyl = np.linspace(-cylinder_height/2, cylinder_height/2, 20)
    u_cyl, v_cyl = np.meshgrid(u_cyl, v_cyl)
    
    # Create perpendicular basis
    if abs(principal_axis[2]) < 0.9:
        perp1 = np.array([0, 0, 1])
    else:
        perp1 = np.array([1, 0, 0])
    perp1 = perp1 - np.dot(perp1, principal_axis) * principal_axis
    perp1 = perp1 / (np.linalg.norm(perp1) + 1e-10)
    perp2 = np.cross(principal_axis, perp1)
    perp2 = perp2 / (np.linalg.norm(perp2) + 1e-10)
    
    x_cyl = centroid[0] + v_cyl * principal_axis[0] + cylinder_radius * (np.cos(u_cyl) * perp1[0] + np.sin(u_cyl) * perp2[0])
    y_cyl = centroid[1] + v_cyl * principal_axis[1] + cylinder_radius * (np.cos(u_cyl) * perp1[1] + np.sin(u_cyl) * perp2[1])
    z_cyl = centroid[2] + v_cyl * principal_axis[2] + cylinder_radius * (np.cos(u_cyl) * perp1[2] + np.sin(u_cyl) * perp2[2])
    
    ax6.plot_surface(x_cyl, y_cyl, z_cyl, alpha=0.3, color='blue', linewidth=0)
    ax6.scatter(valid_points_all[:, 0], valid_points_all[:, 1], valid_points_all[:, 2],
                c='green', s=30, alpha=0.8)
    # Draw axis
    ax6.plot([centroid[0] - axis_length*principal_axis[0], 
              centroid[0] + axis_length*principal_axis[0]],
             [centroid[1] - axis_length*principal_axis[1], 
              centroid[1] + axis_length*principal_axis[1]],
             [centroid[2] - axis_length*principal_axis[2], 
              centroid[2] + axis_length*principal_axis[2]],
             'r-', linewidth=4, label='PCA Axis')
    ax6.scatter([0], [0], [0], color='green', s=200, marker='*')
    ax6.set_title('Stage 6: Cylinder Fitted\n(Ready for directed sampling)', fontweight='bold')
    ax6.set_xlabel('X')
    ax6.set_ylabel('Y')
    ax6.set_zlabel('Z')
    ax6.legend()
    ax6.view_init(elev=30, azim=45)
    ax6.set_box_aspect([1,1,1])
    
    plt.tight_layout()
    output_path = f'{output_dir}/burn_in_figure7_constraint_discovery.png'
    plt.savefig(output_path)
    print(f"Saved: {output_path}")
    plt.close()


def main():
    """Generate all burn-in phase visualizations"""
    import os
    
    output_dir = '.'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    print("Generating burn-in phase visualizations...")
    print("=" * 60)
    
    print("\n1. Generating Figure 1: Iterative Radius Search Process...")
    figure1_iterative_radius_search(output_dir)
    
    print("\n2. Generating Figure 2: Validity Rate Evolution...")
    figure2_validity_rate_evolution(output_dir)
    
    print("\n3. Generating Figure 3: Sample Distribution on Sphere Surface...")
    figure3_sample_distribution(output_dir)
    
    print("\n4. Generating Figure 6: Radius Adjustment Mechanism...")
    figure6_radius_adjustment(output_dir)
    
    print("\n5. Generating Figure 7: Constraint Direction Discovery...")
    figure7_constraint_discovery(output_dir)
    
    print("\n" + "=" * 60)
    print("All visualizations generated successfully!")
    print(f"Output directory: {os.path.abspath(output_dir)}")


if __name__ == '__main__':
    main()

