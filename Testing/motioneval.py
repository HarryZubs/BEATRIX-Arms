#!/usr/bin/env python3

import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os


robots_config = [
    {"urdf": "dummy_path/human.urdf", "ee_frame": "joint7", "name": "Human"},
    {"urdf": "dummy_path/7dof.urdf", "ee_frame": "joint7", "name": "7DoF"},
    {"urdf": "dummy_path/6dof.urdf", "ee_frame": "joint6", "name": "6 DoF"},
    {"urdf": "dummy_path/5dof.urdf", "ee_frame": "joint5", "name": "5DoF"}
]

num_valid_paths_needed = 100
max_ik_steps = 300
step_size = 0.05
tolerance = 0.05 # 1 cm

#
y_range = [-0.3, 0.3]
z_range = [0.1, 0.6]


def run_ik(model, data, ee_id, q_init, target_pos):
    """Runs greedy Jacobian IK and returns the trajectory and success status."""
    q = q_init.copy()
    trajectory = []
    success = False
    
    for _ in range(max_ik_steps):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        current_pos = data.oMf[ee_id].translation.copy()
        trajectory.append(current_pos)
        
        err = target_pos - current_pos
        if np.linalg.norm(err) < tolerance:
            success = True
            break
            
        J = pin.computeFrameJacobian(model, data, q, ee_id, pin.LOCAL_WORLD_ALIGNED)[:3, :]
        dq = np.linalg.pinv(J) @ (err * step_size)
        q = pin.integrate(model, q, dq)
        
    return np.array(trajectory), q, success

def normalize_trajectory(traj, num_points=100):
    """Interpolates a 3D trajectory to have exactly 'num_points' for 1-to-1 comparison."""
    if len(traj) < 2:
        return np.tile(traj[0], (num_points, 1))
    
    
    distances = np.cumsum(np.insert(np.linalg.norm(np.diff(traj, axis=0), axis=1), 0, 0))
    distances /= distances[-1] 
    
   
    interpolator = interp1d(distances, traj, axis=0, kind='linear')
    
    
    return interpolator(np.linspace(0, 1, num_points))

models = {}
for config in robots_config:
    try:
        model = pin.buildModelFromUrdf(config["urdf"])
        data = model.createData()
        if model.existFrame(config["ee_frame"]):
            ee_id = model.getFrameId(config["ee_frame"])
            models[config["name"]] = {"model": model, "data": data, "ee_id": ee_id}
    except ValueError:
        print(f"Skipping {config['name']} (URDF not found)")

human_name = robots_config[0]["name"]
errors_accumulated = {name: [] for name in models.keys() if name != human_name}


print(f"Starting analysis. Looking for {num_valid_paths_needed} valid paths shared by all arms...")
valid_paths_found = 0
attempts = 0

while valid_paths_found < num_valid_paths_needed:
    attempts += 1
    
    
    start_pos = np.array([np.random.uniform(*x_range), np.random.uniform(*y_range), np.random.uniform(*z_range)])
    end_pos = np.array([np.random.uniform(*x_range), np.random.uniform(*y_range), np.random.uniform(*z_range)])
    
    all_successful = True
    trajectories = {}
    
   
    for name, m_dict in models.items():
        q_neutral = pin.neutral(m_dict["model"])
        
        
        _, q_start, start_success = run_ik(m_dict["model"], m_dict["data"], m_dict["ee_id"], q_neutral, start_pos)
        
        if not start_success:
            all_successful = False
            break
            
      
        traj, _, end_success = run_ik(m_dict["model"], m_dict["data"], m_dict["ee_id"], q_start, end_pos)
        
        if not end_success:
            all_successful = False
            break
            
        trajectories[name] = traj

   
    if not all_successful:
        continue
        
    valid_paths_found += 1
    print(f"\rProgress: {valid_paths_found}/{num_valid_paths_needed} paths calculated...", end="")
    
   
    human_traj = normalize_trajectory(trajectories[human_name])
    
    for name in errors_accumulated.keys():
        robot_traj = normalize_trajectory(trajectories[name])
       
        path_error = np.mean(np.linalg.norm(robot_traj - human_traj, axis=1))
        errors_accumulated[name].append(path_error)

print(f"\nAnalysis complete! Took {attempts} attempts to find {num_valid_paths_needed} valid shared paths.")


print("\n" + "="*45)
print(f"AVERAGE TRAJECTORY ERROR VS. HUMAN PATH")
print(f"(Lower is better/more human-like)")
print("="*45)

names = []
avg_errors = []

for name, err_list in errors_accumulated.items():
    avg_error_m = np.mean(err_list)
    avg_error_cm = avg_error_m * 100
    
    names.append(name)
    avg_errors.append(avg_error_cm)
    
    print(f"{name:>10}: {avg_error_cm:>5.2f} cm average deviation")
print("="*45)

# Generate a quick bar chart
plt.figure(figsize=(8, 6))
plt.bar(names, avg_errors, color=['green', 'orange', 'red'])
plt.title("Average Path Deviation from Human Trajectory")
plt.ylabel("Average Error (cm)")
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.show()