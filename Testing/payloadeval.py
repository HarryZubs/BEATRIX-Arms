#!/usr/bin/env python3

import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import os

# --- 1. Configuration & Setup ---
package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
urdf_folder = os.path.join(package_root, "urdf")
urdf_path = os.path.join(urdf_folder, "static.urdf")

# Load the base model once
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# Define your motor torque limits (Nm) for joints 1 through 7
torque_limits = np.array([11.34, 34.02, 11.34, 34.02, 0.47, 1.41, 0.47])

# Define dynamic movement targets
target_dq = np.ones(model.nq) * 5.0    # Velocity vector 
target_ddq = np.ones(model.nv) * 10.0  # Acceleration vector
num_samples = 10000

# Get the joint ID for the final link (adjust name if it differs in your URDF)
# Note: In Pinocchio, spatial inertias are tied to the joint index.
ee_joint_name = "joint7"
if model.existJointName(ee_joint_name):
    ee_joint_id = model.getJointId(ee_joint_name)
else:
    raise ValueError(f"Joint '{ee_joint_name}' not found in the model.")

# Generate a single static batch of random configurations.
# Testing the exact SAME poses against different masses eliminates random noise 
# and gives a perfect apples-to-apples comparison for the plot.
print(f"Generating {num_samples} test poses...")
q_samples = [pin.randomConfiguration(model) for _ in range(num_samples)]

# --- 2. Evaluate Workspace Across Different Masses ---
masses_g = np.arange(0, 550, 50)  # [0, 50, 100, ..., 500]
reachable_counts = []

print("\nSimulating reachability for varying payloads...")
print("-" * 50)

for mass_g in masses_g:
    # Convert grams to kg
    mass_kg = mass_g / 1000.0
    
    # Update the mass of the final link directly in memory
    model.inertias[ee_joint_id].mass = mass_kg
    
    valid_poses = 0
    
    for q in q_samples:
        # Calculate full Inverse Dynamics (M*ddq + C*dq + g) under our target kinematics
        tau_dynamic = pin.rnea(model, data, q, target_dq, target_ddq)
        
        # Check if the required torque is within limits for ALL joints
        if np.all(np.abs(tau_dynamic) <= torque_limits):
            valid_poses += 1
            
    reachable_counts.append(valid_poses)
    print(f"Payload: {mass_g:>3}g -> Reachable Poses: {valid_poses}/{num_samples}")

print("-" * 50)

# --- 3. Calculate Relative Percentages ---
# Treat 0g payload as the 100% baseline
baseline_count = reachable_counts[0]

if baseline_count == 0:
    print("Warning: Baseline (0g) has 0 reachable poses. Check your torque limits.")
    relative_percentages = [0.0] * len(reachable_counts)
else:
    relative_percentages = [(count / baseline_count) * 100.0 for count in reachable_counts]

# --- 4. Visualization ---
plt.figure(figsize=(10, 6))

# Plot the line with markers
plt.plot(masses_g, relative_percentages, marker='o', linestyle='-', color='#1f77b4', linewidth=2, markersize=8)

# Fill the area beneath for visual readability
plt.fill_between(masses_g, relative_percentages, alpha=0.2, color='#1f77b4')

# Formatting the plot
plt.title('Impact of Payload Mass on Dynamically Reachable Workspace', fontsize=14, fontweight='bold')
plt.xlabel('Payload Mass (g)', fontsize=12)
plt.ylabel('% of Reachable Poses ', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)
plt.xticks(masses_g)

# Cap the Y-axis logically
plt.ylim(0, max(105, max(relative_percentages) + 5)) 

# Add data labels slightly above each point
for x, y in zip(masses_g, relative_percentages):
    plt.text(x, y + 2.5, f"{y:.1f}%", ha='center', va='bottom', fontsize=9)

plt.tight_layout()
plt.show()