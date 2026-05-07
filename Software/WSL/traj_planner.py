#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
import time


latest_real = None       
latest_desired = None    
replan_requested = False 


ERROR_THRESHOLD = 0.2  


history_t = []
history_pos = []
history_vel = []
history_acc = []


def real_callback(msg):
    global latest_real
    if len(msg.position) >= 7:
        latest_real = np.array(msg.position[:7])

def desired_callback(msg):
    global latest_desired
    if len(msg.position) >= 7:
        latest_desired = np.array(msg.position[:7])

def error_callback(msg):
    global replan_requested
    if len(msg.position) >= 7:
        error_arr = np.array(msg.position[:7])
        max_err = np.max(np.abs(error_arr))
        
        
        if max_err > ERROR_THRESHOLD and not replan_requested:
            rospy.logwarn(f"Error Threshold Exceeded! Max Error: {max_err:.3f} rad. Requesting Replan...")
            replan_requested = True

def cubic_coefficients(q0, qf, v0, vf, T):
    a0 = q0
    a1 = v0
    a2 = (3*(qf - q0)/T**2) - (2*v0 + vf)/T
    a3 = (-2*(qf - q0)/T**3) + (v0 + vf)/T**2
    return np.array([a0, a1, a2, a3])

def cubic_eval(coeffs, t):
    a0, a1, a2, a3 = coeffs
    pos = a0 + a1*t + a2*t**2 + a3*t**3
    vel = a1 + 2*a2*t + 3*a3*t**2
    acc = 2*a2 + 6*a3*t
    return pos, vel, acc

def plot_trajectory():
    rospy.loginfo("Generating trajectory plots...")
    if not history_t:
        rospy.logwarn("No data collected. Exiting without plotting.")
        return

    t = np.array(history_t)
    p = np.array(history_pos)
    v = np.array(history_vel)
    a = np.array(history_acc)

    fig, axes = plt.subplots(7, 1, figsize=(12, 16), sharex=True)
    fig.suptitle('Planned Trajectory with Active Error Replanning', fontsize=16)

    for i in range(7):
        ax = axes[i]
        ax.plot(t, p[:, i], label='Position (rad)', color='blue', linewidth=2)
        ax.plot(t, v[:, i], label='Velocity (rad/s)', color='orange', linestyle='--')
        ax.plot(t, a[:, i], label='Acceleration (rad/s²)', color='green', linestyle=':', alpha=0.8)
        
        ax.set_ylabel(f'Joint {i+1}')
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(loc='upper right')

    axes[-1].set_xlabel('Global Node Time (seconds)')
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


def main():
    global latest_real, latest_desired, replan_requested
    rospy.init_node("dynamic_cubic_path_planner")

    rospy.on_shutdown(plot_trajectory)

 
    rospy.Subscriber("/joint_states_real", JointState, real_callback, queue_size=1)
    rospy.Subscriber("/joint_states", JointState, desired_callback, queue_size=1)
    rospy.Subscriber("/tracking_error", JointState, error_callback, queue_size=1) # NEW SUBSCRIBER
    pub = rospy.Publisher("/planned_joint_states", JointState, queue_size=1)

    rate = rospy.Rate(50)  

    duration = 5.0  
    t0 = rospy.get_time()
    node_start_time = rospy.get_time()

    prev_desired = None
    current_planned_pos = None
    current_planned_vel = None
    coeffs = None

    rospy.loginfo("Trajectory planner initialized. Press Ctrl+C to stop and view graphs.")

    while not rospy.is_shutdown():
        if latest_real is None or latest_desired is None:
            rospy.loginfo_throttle(2.0, f"Waiting for data... Real: {latest_real is not None} | Desired: {latest_desired is not None}")
            if latest_real is not None and latest_desired is None:
                latest_desired = latest_real.copy()
            else:
                rate.sleep()
                continue

       
        target_changed = prev_desired is None or not np.allclose(latest_desired, prev_desired, atol=1e-5)
        
        if target_changed or replan_requested:
            
            if replan_requested:
                rospy.logwarn("Executing Recovery: Replanning from CURRENT REAL position.")
                q0 = latest_real.copy()
                
                v0 = np.zeros(7) 
            elif current_planned_pos is None:
                
                q0 = latest_real.copy()
                v0 = np.zeros(7)
            else:
               
                q0 = current_planned_pos.copy()
                v0 = current_planned_vel.copy() 

            qf = latest_desired.copy()
            vf = np.zeros(7) 
            
            coeffs = np.array([cubic_coefficients(q0[i], qf[i], v0[i], vf[i], duration) for i in range(7)])
            t0 = rospy.get_time()
            prev_desired = latest_desired.copy()
            
           
            replan_requested = False

        if coeffs is None:
            rate.sleep()
            continue

     
        t_now = rospy.get_time()
        t_elapsed = t_now - t0
        t_clamped = min(t_elapsed, duration)
        t_global = t_now - node_start_time 

        pos, vel, acc = [], [], []
        for i in range(7):
            p, v, a = cubic_eval(coeffs[i], t_clamped)
            pos.append(p)
            vel.append(v)
            acc.append(a)

        current_planned_pos = np.array(pos)
        current_planned_vel = np.array(vel)

        history_t.append(t_global)
        history_pos.append(pos)
        history_vel.append(vel)
        history_acc.append(acc)

        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        js_msg.name = [f"joint{i+1}" for i in range(7)]
        js_msg.position = pos
        js_msg.velocity = vel
        js_msg.effort = acc  
        pub.publish(js_msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass