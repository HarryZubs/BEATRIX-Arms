#!/usr/bin/env python3

import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# --- 1. Configuration & Setup ---

# Define your 4 URDF files and their corresponding end-effector frame names.
robots_config = [
    {"urdf": "dummy_path/human.urdf", "ee_frame": "joint7", "name": "Human"},
    {"urdf": "dummy_path/7dof.urdf", "ee_frame": "joint7", "name": "7DoF"},
    {"urdf": "dummy_path/6dof.urdf", "ee_frame": "joint6", "name": "6 DoF"},
    {"urdf": "dummy_path/5dof.urdf", "ee_frame": "joint5", "name": "5DoF"}
]

num_samples = 50000  # Number of random poses to test per robot

# Voxel sizes to test (in cm). range(1, 20, 2) gives: 1, 3, 5, 7, 9, 11, 13, 15, 17, 19
voxel_sizes_cm = list(range(1, 20, 2))

# Dictionary to store the generated point clouds so we only simulate once
point_clouds = {}

# --- 2. Simulation Loop (Generate Point Clouds) ---
print("Generating point clouds. This may take a moment...")

for config in robots_config:
    urdf_path = config["urdf"]
    ee_name = config["ee_frame"]
    
    try:
        model = pin.buildModelFromUrdf(urdf_path)
        robot_name = model.name
    except ValueError:
        print(f"  -> WARNING: Could not load URDF at {urdf_path}.")
        continue

    data = model.createData()
    
    if model.existFrame(ee_name):
        ee_frame_id = model.getFrameId(ee_name)
    else:
        print(f"  -> WARNING: Frame '{ee_name}' not found in {robot_name}.")
        continue

    pts_kinematic = []

    # Monte Carlo simulation
    for _ in range(num_samples):
        q = pin.randomConfiguration(model)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        
        pos = data.oMf[ee_frame_id].translation.copy()
        pts_kinematic.append(pos)

    point_clouds[robot_name] = np.array(pts_kinematic)
    print(f"  -> {robot_name}: Generated {len(pts_kinematic)} reachable points.")


# --- 3. Visualization 1: 3D Reachability Subplots ---
print("\nGenerating 3D workspace plots...")
fig1 = plt.figure(figsize=(16, 12))
fig1.canvas.manager.set_window_title('Kinematic Workspaces')

for i, (robot_name, pts) in enumerate(point_clouds.items()):
    ax = fig1.add_subplot(2, 2, i + 1, projection='3d')
    
    ax.scatter(pts[:,0], pts[:,1], pts[:,2], c='blue', s=1, alpha=0.1)

    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title(f'Kinematic Workspace: {robot_name}')

    # Equalize axis scaling for a proportional 3D view
    max_range = np.array([pts[:,0].max()-pts[:,0].min(), 
                          pts[:,1].max()-pts[:,1].min(), 
                          pts[:,2].max()-pts[:,2].min()]).max() / 2.0
    mid_x = (pts[:,0].max()+pts[:,0].min()) * 0.5
    mid_y = (pts[:,1].max()+pts[:,1].min()) * 0.5
    mid_z = (pts[:,2].max()+pts[:,2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

fig1.tight_layout()


# --- 4. Voxelization & Analysis Loop ---
print("\nCalculating voxel overlaps across varying resolutions...")

# We assume the first robot loaded successfully is our baseline (Human)
baseline_name = list(point_clouds.keys())[0]
baseline_pts = point_clouds[baseline_name]

# Prepare a dictionary to store coverage results for the other robots
overlap_results = {name: [] for name in point_clouds.keys() if name != baseline_name}

for vs_cm in voxel_sizes_cm:
    vs_m = vs_cm / 100.0  # Convert cm to meters for the math
    
    # Discretize the baseline (Human) point cloud at this voxel size
    baseline_voxels = set(tuple(v) for v in np.floor(baseline_pts / vs_m).astype(int))
    
    # Compare against every other robot
    for robot_name, pts in point_clouds.items():
        if robot_name == baseline_name:
            continue
            
        robot_voxels = set(tuple(v) for v in np.floor(pts / vs_m).astype(int))
        intersection = baseline_voxels.intersection(robot_voxels)
        
        if len(baseline_voxels) > 0:
            coverage = (len(intersection) / len(baseline_voxels)) * 100
        else:
            coverage = 0.0
            
        overlap_results[robot_name].append(coverage)


# --- 5. Visualization 2: 2D Line Graph ---
print("Generating resolution line graph...")
fig2 = plt.figure(figsize=(10, 6))
fig2.canvas.manager.set_window_title('Workspace Overlap vs Resolution')

# Plot a line for each robot being compared
markers = ['o', 's', '^', 'D', 'v']
for i, (robot_name, coverages) in enumerate(overlap_results.items()):
    plt.plot(voxel_sizes_cm, coverages, marker=markers[i % len(markers)], 
             linewidth=2, markersize=8, label=f"{robot_name}")

plt.title(f"Workspace Overlap vs. Voxel Resolution\n(Baseline: {baseline_name})", fontsize=14, fontweight='bold')
plt.xlabel("Voxel Size (cm)", fontsize=12)
plt.ylabel(f"Percentage of {baseline_name} Workspace Reached (%)", fontsize=12)
plt.xticks(voxel_sizes_cm) 
plt.ylim(0, 105) 
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=12)

fig2.tight_layout()

# Show both figures
plt.show()