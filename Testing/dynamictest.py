#!/usr/bin/env python3

import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os


package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
urdf_folder = os.path.join(package_root, "urdf")
os.makedirs(urdf_folder, exist_ok=True)
urdf_path = os.path.join(urdf_folder, "static.urdf")

model = pin.buildModelFromUrdf(urdf_path)

data = model.createData()

torque_limits = np.array([11.34, 34.02, 11.34, 34.02, 0.47, 1.41, 0.47])


target_dq = np.ones(model.nq) * 5.0
target_ddq = np.ones(model.nv) * 10.0  

num_samples = 10000 
ee_frame_id = model.getFrameId("joint7") 


pts_kinematic_only = []
pts_static_only = []
pts_dynamic = []
pts_all = [] 


max_seen_torques = np.zeros(model.nv)

print("Simulating workspace...")


for _ in range(num_samples):

    q = pin.randomConfiguration(model)
    

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pos = data.oMf[ee_frame_id].translation.copy()
    pts_all.append(pos)
    
   
    tau_static = pin.computeGeneralizedGravity(model, data, q)
    
   
    tau_dynamic = pin.rnea(model, data, q, target_dq, target_ddq)
    
    
    max_seen_torques = np.maximum(max_seen_torques, np.abs(tau_dynamic))

    if np.all(np.abs(tau_static) <= torque_limits):
        if np.all(np.abs(tau_dynamic) <= torque_limits):
            
            pts_dynamic.append(pos)
        else:
       
            pts_static_only.append(pos)
    else:
      
        pts_kinematic_only.append(pos)


pts_kinematic_only = np.array(pts_kinematic_only)
pts_static_only = np.array(pts_static_only)
pts_dynamic = np.array(pts_dynamic)
pts_all = np.array(pts_all)


print(f"Total Poses Evaluated: {num_samples}")
print(f"Kinematic Only:        {len(pts_kinematic_only)}")
print(f"Static Only:           {len(pts_static_only)}")
print(f"Dynamically Reachable: {len(pts_dynamic)}")

print("\n" + "="*50)
print("MAXIMUM DYNAMIC TORQUES OBSERVED (Nm)")
print("="*50)
for i in range(model.nv):
    joint_name = model.names[i+1] 
    print(f"{joint_name:>10}: {max_seen_torques[i]:>8.3f} Nm")
print("="*50 + "\n")


fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')


if len(pts_dynamic) > 0:
    ax.scatter(pts_dynamic[:,0], pts_dynamic[:,1], pts_dynamic[:,2], 
               c='green', s=15, alpha=0.8, label='Dynamically Reachable (Green)')


if len(pts_static_only) > 0:
    ax.scatter(pts_static_only[:,0], pts_static_only[:,1], pts_static_only[:,2], 
               c='yellow', s=5, alpha=0.3, label='Statically Reachable Only (Yellow)')


if len(pts_kinematic_only) > 0:
    ax.scatter(pts_kinematic_only[:,0], pts_kinematic_only[:,1], pts_kinematic_only[:,2], 
               c='red', s=1, alpha=0.1, label='Kinematic Only - Too Weak (Red)')

ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_zlabel('Z (meters)')
ax.set_title('7-DOF Torque-Constrained Workspace')
ax.legend()

#
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