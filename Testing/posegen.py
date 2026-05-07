#!/usr/bin/env python3

import pinocchio as pin
import numpy as np
import csv
import os


urdf_path = "dummy_path/7dof.urdf"
ee_frame_name = "joint7"       
output_csv = "7dof_poses.csv"

#
num_samples = 100000 

def generate_poses():
    print(f"Loading URDF from: {urdf_path}")
    try:
        model = pin.buildModelFromUrdf(urdf_path)
    except ValueError:
        print(f"ERROR: Could not load URDF at {urdf_path}. Please check the path.")
        return

    data = model.createData()
    

    if model.existFrame(ee_frame_name):
        ee_frame_id = model.getFrameId(ee_frame_name)
    else:
        print(f"ERROR: Frame '{ee_frame_name}' not found in the model.")
        return

    print(f"Generating {num_samples} poses...")
    
 
    with open(output_csv, mode='w', newline='') as file:
        writer = csv.writer(file)
        
       
        num_joints = model.nq
        joint_headers = [f"q{i+1}" for i in range(num_joints)]
        
       
        header = joint_headers + ["x", "y", "z", "qx", "qy", "qz", "qw"]
        writer.writerow(header)
        
       
        for i in range(num_samples):
            
            q = pin.randomConfiguration(model)
            
           
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            
            
            pos = data.oMf[ee_frame_id].translation
            
            
            rot_matrix = data.oMf[ee_frame_id].rotation
            quat = pin.Quaternion(rot_matrix)
            
           
            row_data = np.concatenate((q, pos, [quat.x, quat.y, quat.z, quat.w]))
            
          
            writer.writerow(row_data)
            
            
            if (i + 1) % (num_samples // 10) == 0:
                print(f"  -> Progress: {((i + 1) / num_samples) * 100:.0f}%")

    print(f"\nSuccess! Saved {num_samples} poses to {output_csv}")

if __name__ == "__main__":
    generate_poses()