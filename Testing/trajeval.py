#!/usr/bin/env python3

import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# --- 1. Configuration ---
robots_config = [
    {"urdf": "dummy_path/human.urdf", "ee_frame": "joint7", "name": "Human", "color": "blue"},
    {"urdf": "dummy_path/7dof.urdf", "ee_frame": "joint7", "name": "7DoF", "color": "green"},
    {"urdf": "dummy_path/6dof.urdf", "ee_frame": "joint6", "name": "6 DoF", "color": "orange"},
    {"urdf": "dummy_path/5dof.urdf", "ee_frame": "joint5", "name": "5DoF", "color": "red"}
]

# The XYZ coordinate all robots will try to reach
target_pos = np.array([0.4, 0.2, 0.3]) 
max_steps = 200     # Maximum IK iterations
step_size = 0.1     # How aggressively the algorithm moves per step
tolerance = 0.01    # Stop if within 1 cm of the target

# --- 2. Setup Visualization ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(*target_pos, c='black', s=100, marker='*', label='Target')

print("Starting quick trajectory investigation...\n" + "-"*40)

# --- 3. Simulation Loop ---
for config in robots_config:
    urdf_path = config["urdf"]
    
    try:
        model = pin.buildModelFromUrdf(urdf_path)
    except ValueError:
        print(f"WARNING: Could not load {config['name']}")
        continue

    data = model.createData()
    if not model.existFrame(config["ee_frame"]):
        continue
        
    ee_id = model.getFrameId(config["ee_frame"])
    
    # Start all robots at their "zero" configuration
    q = pin.neutral(model)
    
    trajectory = []
    
    # Greedy Jacobian IK Loop
    for step in range(max_steps):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        current_pos = data.oMf[ee_id].translation.copy()
        
        trajectory.append(current_pos)
        
        # Calculate error (distance to target)
        err = target_pos - current_pos
        if np.linalg.norm(err) < tolerance:
            break # Reached the target!
            
        # Get the Jacobian for the end-effector
        J = pin.computeFrameJacobian(model, data, q, ee_id, pin.LOCAL_WORLD_ALIGNED)[:3, :]
        
        # Calculate joint velocities to move towards the target
        dq = np.linalg.pinv(J) @ (err * step_size)
        
        # Integrate to get new joint angles
        q = pin.integrate(model, q, dq)
        
    trajectory = np.array(trajectory)
    
    # Calculate total path length
    path_length = np.sum(np.linalg.norm(np.diff(trajectory, axis=0), axis=1))
    
    print(f"{config['name']:>10}: Reached in {len(trajectory)} steps | Path Length: {path_length:.3f} m")
    
    # Plot the trajectory
    ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2], 
            linewidth=3, label=config["name"], color=config["color"])
    
    # Plot the start point
    ax.scatter(*trajectory[0], c=config["color"], s=30, marker='o')

# --- 4. Final Graph Formatting ---
ax.set_title('End-Effector Trajectories (Zero-Pose to Target)')
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_zlabel('Z (meters)')
ax.legend()
plt.show()