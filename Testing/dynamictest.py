#!/usr/bin/env python3

import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# --- 1. Configuration & Setup ---
package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
urdf_folder = os.path.join(package_root, "urdf")
os.makedirs(urdf_folder, exist_ok=True)
urdf_path = os.path.join(urdf_folder, "static.urdf")

model = pin.buildModelFromUrdf(urdf_path)

data = model.createData()

# Define your motor torque limits (Nm) for joints 1 through 7
torque_limits = np.array([11.34, 34.02, 11.34, 34.02, 0.47, 1.41, 0.47])

# Define dynamic movement targets
target_dq = np.ones(model.nq) * 5.0   # Velocity vector 
target_ddq = np.ones(model.nv) * 10.0  # Acceleration vector

num_samples = 10000 # Number of random poses to test
ee_frame_id = model.getFrameId("joint7") # The frame to track

# Mutually exclusive lists to prevent overlapping scatter plots
pts_kinematic_only = []
pts_static_only = []
pts_dynamic = []
pts_all = [] 

# Array to track the absolute maximum dynamic torque required for each joint
max_seen_torques = np.zeros(model.nv)

print("Simulating workspace...")

# --- 2. Monte Carlo Simulation Loop ---
for _ in range(num_samples):
    # Generate random joint angles within the URDF limits
    q = pin.randomConfiguration(model)
    
    # Calculate Forward Kinematics to find the End-Effector XYZ
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pos = data.oMf[ee_frame_id].translation.copy()
    pts_all.append(pos)
    
    # Calculate tau required just to hold against gravity
    tau_static = pin.computeGeneralizedGravity(model, data, q)
    
    # Calculate full Inverse Dynamics (M*ddq + C*dq + g) under our target velocity/acceleration
    tau_dynamic = pin.rnea(model, data, q, target_dq, target_ddq)
    
    # Update our maximum torque tracker with the highest absolute value seen so far
    max_seen_torques = np.maximum(max_seen_torques, np.abs(tau_dynamic))
    
    # Sort the point into mutually exclusive categories
    if np.all(np.abs(tau_static) <= torque_limits):
        if np.all(np.abs(tau_dynamic) <= torque_limits):
            # Passes both static and dynamic checks
            pts_dynamic.append(pos)
        else:
            # Passes static, but fails dynamic
            pts_static_only.append(pos)
    else:
        # Fails static check (kinematic only)
        pts_kinematic_only.append(pos)

# Convert lists to numpy arrays for plotting
pts_kinematic_only = np.array(pts_kinematic_only)
pts_static_only = np.array(pts_static_only)
pts_dynamic = np.array(pts_dynamic)
pts_all = np.array(pts_all)

# --- 3. Console Output ---
print(f"Total Poses Evaluated: {num_samples}")
print(f"Kinematic Only:        {len(pts_kinematic_only)}")
print(f"Static Only:           {len(pts_static_only)}")
print(f"Dynamically Reachable: {len(pts_dynamic)}")

print("\n" + "="*50)
print("MAXIMUM DYNAMIC TORQUES OBSERVED (Nm)")
print("="*50)
for i in range(model.nv):
    joint_name = model.names[i+1] # +1 because universe/root is index 0
    print(f"{joint_name:>10}: {max_seen_torques[i]:>8.3f} Nm")
print("="*50 + "\n")

# --- 4. Visualization ---
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot Dynamic 
if len(pts_dynamic) > 0:
    ax.scatter(pts_dynamic[:,0], pts_dynamic[:,1], pts_dynamic[:,2], 
               c='green', s=15, alpha=0.8, label='Dynamically Reachable (Green)')

# Plot Static Only
if len(pts_static_only) > 0:
    ax.scatter(pts_static_only[:,0], pts_static_only[:,1], pts_static_only[:,2], 
               c='yellow', s=5, alpha=0.3, label='Statically Reachable Only (Yellow)')

# Plot Kinematic Only 
if len(pts_kinematic_only) > 0:
    ax.scatter(pts_kinematic_only[:,0], pts_kinematic_only[:,1], pts_kinematic_only[:,2], 
               c='red', s=1, alpha=0.1, label='Kinematic Only - Too Weak (Red)')

ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_zlabel('Z (meters)')
ax.set_title('7-DOF Torque-Constrained Workspace')
ax.legend()

# Equalize axis scaling for a proportional 3D view using all generated points
if len(pts_all) > 0:
    max_range = np.array([pts_all[:,0].max()-pts_all[:,0].min(), 
                          pts_all[:,1].max()-pts_all[:,1].min(), 
                          pts_all[:,2].max()-pts_all[:,2].min()]).max() / 2.0
    mid_x = (pts_all[:,0].max()+pts_all[:,0].min()) * 0.5
    mid_y = (pts_all[:,1].max()+pts_all[:,1].min()) * 0.5
    mid_z = (pts_all[:,2].max()+pts_all[:,2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()