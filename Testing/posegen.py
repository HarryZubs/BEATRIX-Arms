#!/usr/bin/env python3

import pinocchio as pin
import numpy as np
import csv
import os

# --- 1. Configuration & Setup ---
urdf_path = "dummy_path/7dof.urdf"
ee_frame_name = "joint7"       # Make sure this matches your URDF's end-effector frame
output_csv = "7dof_poses.csv"

# Number of random poses to generate. 
# 1,000,000 will give a very dense map but will result in a ~150MB CSV file.
num_samples = 100000 

def generate_poses():
    print(f"Loading URDF from: {urdf_path}")
    try:
        model = pin.buildModelFromUrdf(urdf_path)
    except ValueError:
        print(f"ERROR: Could not load URDF at {urdf_path}. Please check the path.")
        return

    data = model.createData()
    
    # Verify the end-effector frame exists
    if model.existFrame(ee_frame_name):
        ee_frame_id = model.getFrameId(ee_frame_name)
    else:
        print(f"ERROR: Frame '{ee_frame_name}' not found in the model.")
        return

    print(f"Generating {num_samples} poses...")
    
    # Open CSV file for writing
    with open(output_csv, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Determine the number of joints dynamically based on the URDF
        num_joints = model.nq
        joint_headers = [f"q{i+1}" for i in range(num_joints)]
        
        # Write the CSV Header
        header = joint_headers + ["x", "y", "z", "qx", "qy", "qz", "qw"]
        writer.writerow(header)
        
        # --- 2. Simulation Loop ---
        for i in range(num_samples):
            # Generate a random valid configuration respecting joint limits
            q = pin.randomConfiguration(model)
            
            # Compute Forward Kinematics
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            
            # Extract position (translation)
            pos = data.oMf[ee_frame_id].translation
            
            # Extract orientation (rotation matrix to quaternion)
            rot_matrix = data.oMf[ee_frame_id].rotation
            quat = pin.Quaternion(rot_matrix)
            
            # Compile row data
            row_data = np.concatenate((q, pos, [quat.x, quat.y, quat.z, quat.w]))
            
            # Write to CSV
            writer.writerow(row_data)
            
            # Optional: Print progress every 10% to keep track of large generations
            if (i + 1) % (num_samples // 10) == 0:
                print(f"  -> Progress: {((i + 1) / num_samples) * 100:.0f}%")

    print(f"\nSuccess! Saved {num_samples} poses to {output_csv}")

if __name__ == "__main__":
    generate_poses()