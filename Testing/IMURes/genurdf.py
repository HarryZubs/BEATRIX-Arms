#!/usr/bin/env python3

import os
import math

def generate_dh_urdf(robot_name, dh_table, joint_limits, output_path):
    """
    Generates a URDF file from a Standard DH table using dummy links 
    to separate the Z-axis (theta, d) and X-axis (a, alpha) transformations.
    """
    urdf = f'<?xml version="1.0"?>\n<robot name="{robot_name}">\n'
    
    # Create the base link
    urdf += '  <link name="base_link"/>\n'
    parent_link = "base_link"
    
    for i, (dh, limits) in enumerate(zip(dh_table, joint_limits)):
        joint_num = i + 1
        a, alpha, d, theta_offset = dh
        lower_lim, upper_lim = limits
        
        dummy_link = f"dummy_link_{joint_num}"
        child_link = f"link_{joint_num}"
        joint_name = f"joint{joint_num}" # Matches the naming in your simulation script
        fixed_joint = f"fixed_{joint_num}"
        
        # 1. Revolute Joint: Handles 'd' (translation along Z) and 'theta' (rotation around Z)
        urdf += f'''
  <joint name="{joint_name}" type="revolute">
    <parent link="{parent_link}"/>
    <child link="{dummy_link}"/>
    <origin xyz="0 0 {d}" rpy="0 0 {theta_offset}"/>
    <axis xyz="0 0 1"/>
    <limit lower="{lower_lim}" upper="{upper_lim}" effort="100" velocity="100"/>
  </joint>
  
  <link name="{dummy_link}"/>
'''
        # 2. Fixed Joint: Handles 'a' (translation along X) and 'alpha' (rotation around X)
        urdf += f'''
  <joint name="{fixed_joint}" type="fixed">
    <parent link="{dummy_link}"/>
    <child link="{child_link}"/>
    <origin xyz="{a} 0 0" rpy="{alpha} 0 0"/>
  </joint>
  
  <link name="{child_link}"/>
'''
        parent_link = child_link
        
    urdf += '</robot>\n'
    
    # Write to file
    with open(output_path, 'w') as f:
        f.write(urdf)
    print(f"Generated URDF for {robot_name} at: {output_path}")


# --- Configuration ---
output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "dummy_path"))
os.makedirs(output_dir, exist_ok=True)

# Standard DH Representation:
# a (meters): distance along X_i from Z_{i-1} to Z_i
# alpha (rad): angle around X_i from Z_{i-1} to Z_i
# d (meters): distance along Z_{i-1} from X_{i-1} to X_i
# theta_offset (rad): constant offset angle around Z_{i-1} 

# Note: Math.pi can be used for exact radians (e.g., math.pi/2)

# --- DUMMY DATA FOR 4 ROBOTS ---
# Replace the tables below with your actual DH parameters and limits.
# Ensure both tables for a robot have the same number of rows (DOFs).

robots = [
    {
        "name": "Human",
        "filename": "human.urdf",
        "dh_table": [
            # [a, alpha, d, theta_offset]
            [0.0,  -math.pi/2, 0.1, 0.0],  # Joint 1
            [0.0, math.pi/2, 0.0,  math.pi/2],  # Joint 2
            [0.0,  -math.pi/2, 0.3, 0.0],  # Joint 3
            [-0.05, math.pi/2, 0.0,  0.0],  # Joint 4
            [0.0,  -math.pi/2, 0.2, 0.0],  # Joint 5
            [0.0, math.pi/2, 0.0,  0.0],  # Joint 6
            [0.0,  0.0,       0.10, -math.pi/2]   # Joint 7
        ],
        "joint_limits": [
            # [lower_limit (rad), upper_limit (rad)]
            [-math.pi/2, math.pi/2],
            [-math.pi/6, math.pi],
            [-math.pi/2, math.pi/2],
            [0, 5*math.pi/6],
            [-math.pi/2, math.pi/2],
            [-math.pi/2, math.pi/2],
            [-math.pi/3, math.pi/3]
        ]
    },
    {
        "name": "7DoF",
        "filename": "7dof.urdf",
        "dh_table": [
            # [a, alpha, d, theta_offset]
            [0.0,  -math.pi/2, 0.0, 0.0],  # Joint 1
            [0.0, math.pi/2, 0.0,  math.pi/2],  # Joint 2
            [0.0,  -math.pi/2, 0.3, 0.0],  # Joint 3
            [0.0, math.pi/2, 0.0,  0.0],  # Joint 4
            [0.0,  -math.pi/2, 0.2, 0.0],  # Joint 5
            [0.0, math.pi/2, 0.0,  0.0],  # Joint 6
            [0.0,  0.0,       0.10, -math.pi/2]   # Joint 7
        ],
        "joint_limits": [
            [-math.pi/2, math.pi/2],
            [-math.pi/6, math.pi],
            [-math.pi/2, math.pi/2],
            [0, 5*math.pi/6],
            [-math.pi/2, math.pi/2],
            [-math.pi/2, math.pi/2],
            [-math.pi/3, math.pi/3]
        ]
    },
    {
        "name": "5DoF",
        "filename": "5dof.urdf",
        "dh_table": [
            [0.0, 0, 0.1, 0.0],
            [0.0, math.pi/2,       0.0,  0.0],
            [0.3, 0.0,       0.0,  0.0],
            [0.0, -math.pi/2,       0.10, 0.0],
            [0.2,0,0,0] 
        ],
        "joint_limits": [
            [-math.pi, math.pi],
             [-math.pi/12,3.4],
               [-math.pi/4, math.pi/4],
                 [-math.pi/3, math.pi/3],
                 [-math.pi,math.pi]
        ]
    },
    {
        "name": "6 DoF",
        "filename": "6dof.urdf",
        "dh_table": [
            [0.0, math.pi/2, 0.0, -math.pi/2],
            [0.0,  math.pi/2, 0.0,  math.pi/2],
            [0.0, math.pi/2, 0.3, math.pi/2],
            [0.0,  -math.pi/2, 0.0,  0.0],
            [0.0,  math.pi/2,       0.20, 0.0],
            [0.0,  math.pi/2,       0.0, math.pi/2],

        ],
        "joint_limits": [
            [-math.pi/3, 2.35],
              [0, 2.26], 
              [-math.pi/2, math.pi/2], 
              [-2.5, 0], 
              [-math.pi/2, math.pi/2],
              [-math.pi/4,math.pi/4]
        ]
    }
]

# --- Execution ---
if __name__ == "__main__":
    print(f"Generating URDF files in: {output_dir}\n" + "-"*40)
    for robot in robots:
        file_path = os.path.join(output_dir, robot["filename"])
        generate_dh_urdf(robot["name"], robot["dh_table"], robot["joint_limits"], file_path)