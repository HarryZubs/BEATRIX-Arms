#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64  # <-- Changed to Float64
import numpy as np

# ---------------- Robot DH table ----------------
dh = np.array([
    [np.pi/2, -0.22, 0, -np.pi/2],
    [-np.pi, 0, 0, -np.pi/2],
    [-np.pi, 0.23, 0, -np.pi/2],
    [0, 0, 0, -np.pi/2],
    [-np.pi/2, 0.1925, 0, -np.pi/2],
    [np.pi/2, 0, 0.074, -np.pi/2],
    [0, 0, 0.045, 0]
])
joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6","joint7"]
num_joints = len(dh)

# ---------------- Joint limits in degrees (converted to radians) ----------------
deg2rad = np.pi / 180.0
joint_min_deg = np.array([-90, -45, -90, 0, -90, -30, -30])
joint_max_deg = np.array([ 90,  90,  90, 130,  90,  30,  30])
joint_min = joint_min_deg * deg2rad
joint_max = joint_max_deg * deg2rad
q_center = (joint_min + joint_max)/2  # center for limit bias

# ---------------- FK ----------------
def fk(thetas):
    T = np.eye(4)
    for i, (theta_offset,d,a,alpha) in enumerate(dh):
        theta = thetas[i] + theta_offset
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        Ti = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0,0,0,1]
        ])
        T = T @ Ti
    return T

# ---------------- Numerical Jacobian for position ----------------
def jacobian(thetas, delta=1e-6):
    J = np.zeros((3,num_joints))
    f0 = fk(thetas)[:3,3]
    for i in range(num_joints):
        dtheta = np.zeros(num_joints)
        dtheta[i] = delta
        f1 = fk(thetas + dtheta)[:3,3]
        J[:,i] = (f1 - f0)/delta
    return J

# ---------------- Orientation error ----------------
def orientation_error(R_current, R_desired):
    R_err = R_desired @ R_current.T
    angle = np.arccos(np.clip((np.trace(R_err)-1)/2, -1.0, 1.0))
    if np.isclose(angle, 0):
        return np.zeros(3)
    
    s = np.sin(angle)
    # Singularity protection: if angle is near pi, sin(angle) approaches 0
    if abs(s) < 1e-5: 
        s = 1e-5 if s >= 0 else -1e-5
        
    rx = (R_err[2,1] - R_err[1,2])/(2*s)
    ry = (R_err[0,2] - R_err[2,0])/(2*s)
    rz = (R_err[1,0] - R_err[0,1])/(2*s)
    return angle * np.array([rx, ry, rz])

# ---------------- Quaternion to rotation matrix ----------------
def quat_to_rotmat(qx,qy,qz,qw):
    R = np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx**2 + qy**2)]
    ])
    return R

# ---------------- IK solver with joint limits & multiple seeds ----------------
def ik_solver(target_pose, max_iter=200, tol=1e-4, alpha=0.15):
    target_pos = np.array([target_pose.position.x,
                           target_pose.position.y,
                           target_pose.position.z])
    target_rot = quat_to_rotmat(target_pose.orientation.x,
                                target_pose.orientation.y,
                                target_pose.orientation.z,
                                target_pose.orientation.w)
    
    initial_guesses = [
        np.zeros(num_joints),
        q_center,
        joint_min + 0.8*(joint_max-joint_min),
        joint_max - 0.8*(joint_max-joint_min)
    ]
    
    best_thetas = None
    best_pos_error = np.inf
    
    weight_pos = 1.0
    weight_ori = 0.1 

    for guess in initial_guesses:
        thetas = guess.copy()
        
        for _ in range(max_iter):
            T_current = fk(thetas)
            pos_current = T_current[:3,3]
            rot_current = T_current[:3,:3]
            
            e_pos = target_pos - pos_current
            e_ori = orientation_error(rot_current, target_rot)
            
            e = np.concatenate((e_pos * weight_pos, e_ori * weight_ori))
            
            # Early exit if we hit tolerance!
            if np.linalg.norm(e_pos) < tol and np.linalg.norm(e_ori) < tol:
                # Return tuple of (thetas, error) so callback can use both
                return thetas, np.linalg.norm(e_pos)
            
            J_pos = jacobian(thetas)
            
            J_ori = np.zeros((3,num_joints))
            delta = 1e-6
            for i in range(num_joints):
                dtheta = np.zeros(num_joints)
                dtheta[i] = delta
                R_new = fk(thetas + dtheta)[:3,:3]
                w_delta = orientation_error(rot_current, R_new)
                J_ori[:,i] = w_delta / delta
            
            J_full = np.vstack((J_pos * weight_pos, J_ori * weight_ori))
            
            lambda2 = 1e-3 
            J_pinv = J_full.T @ np.linalg.inv(J_full @ J_full.T + lambda2*np.eye(6))
            
            delta_theta_primary = J_pinv @ e
            
            null_space_proj = np.eye(num_joints) - J_pinv @ J_full
            k_limit = 0.1 
            delta_theta_secondary = null_space_proj @ (k_limit * (q_center - thetas))
            
            thetas += alpha * (delta_theta_primary + delta_theta_secondary)
            thetas = np.minimum(np.maximum(thetas, joint_min), joint_max)
        
        T_final = fk(thetas)
        final_pos_error = np.linalg.norm(target_pos - T_final[:3,3])
        
        if final_pos_error < best_pos_error:
            best_pos_error = final_pos_error
            best_thetas = thetas.copy()
            
    # <-- NEW: We now return BOTH the joint states and the final calculated error 
    return best_thetas, best_pos_error

# ---------------- ROS callback ----------------
def pose_callback(msg):
    # Unpack the tuple returned by the updated ik_solver
    thetas, pos_error = ik_solver(msg.pose)
    
    if pos_error < 0.1: 
        # Pose is reachable: Publish Joint States
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = joint_names
        js.position = thetas.tolist()
        pub.publish(js)
    else:
        # Pose is unreachable: Publish the position error
        rospy.logwarn(f"IK Failed! Unreachable pose. Pos Error: {pos_error:.4f}m")
        error_pub.publish(pos_error)

# ---------------- Main ----------------
if __name__ == "__main__":
    rospy.init_node("ik_solver_node_full")
    
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    
    # <-- NEW: Publisher for the positional error float
    error_pub = rospy.Publisher("/unreachable_error", Float64, queue_size=10)
    
    rospy.Subscriber("/target_pose", PoseStamped, pose_callback)
    
    rospy.loginfo("IK solver node (pos+orientation + limits + multi-seed) started")
    rospy.spin()