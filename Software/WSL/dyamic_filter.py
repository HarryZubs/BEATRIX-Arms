#!/usr/bin/env python3

import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
import os

class AggressiveDynamicFilter:
    def __init__(self):
        rospy.init_node('aggressive_dynamic_filter', anonymous=True)

 
        package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        urdf_path = os.path.join(package_root, "urdf", "static.urdf")
        
        try:
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            rospy.loginfo(f"Successfully loaded URDF for {self.model.name}")
        except Exception as e:
            rospy.logerr(f"Failed to load URDF: {e}")
            rospy.signal_shutdown("URDF Load Failure")


        self.torque_limits = np.array([11.34, 34.02, 11.34, 34.02, 0.47, 1.41, 0.47])
        
       
        self.threshold = 0.33
        
      
        self.k_aggression = 15.0 
        
        self.last_time = rospy.Time.now()
        self.last_dq = np.zeros(self.model.nv)

        self.time_history = []
        self.tau_planned_history = []
        self.tau_filtered_history = []
        self.start_time = None
        
        rospy.on_shutdown(self.plot_results)
        self.safe_state_pub = rospy.Publisher('/safe_joint_states', JointState, queue_size=10)
        self.sub = rospy.Subscriber('/planned_joint_states', JointState, self.joint_state_callback)

        rospy.loginfo(f"Aggressive Filter (SF=4, k={self.k_aggression}) Active.")

    def joint_state_callback(self, msg):
        current_time = rospy.Time.now()
        if self.start_time is None:
            self.start_time = current_time
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).to_sec()
        if dt < 1e-4: return

        q = np.array(msg.position)
        dq_planned = np.array(msg.velocity)
        ddq_planned = (dq_planned - self.last_dq) / dt

        # Calculate planned torques
        tau_planned = pin.rnea(self.model, self.data, q, dq_planned, ddq_planned)
        torque_ratios = np.abs(tau_planned) / self.torque_limits
        max_ratio = np.max(torque_ratios)

        safe_msg = JointState()
        safe_msg.header.stamp = current_time
        safe_msg.name = msg.name
        safe_msg.position = msg.position

      
        if max_ratio > self.threshold:
       
            target_ratio = self.threshold + (1.0 - self.threshold) * \
                           (1.0 - np.exp(-self.k_aggression * (max_ratio - self.threshold)))
            
            scale_factor_accel = target_ratio / max_ratio
            scale_factor_vel = np.sqrt(scale_factor_accel)

            dq_safe = dq_planned * scale_factor_vel
            ddq_safe = ddq_planned * scale_factor_accel
            
            tau_safe = pin.rnea(self.model, self.data, q, dq_safe, ddq_safe)
            safe_msg.velocity = dq_safe.tolist()
            self.last_dq = dq_safe 
        else:
            tau_safe = tau_planned.copy()
            safe_msg.velocity = msg.velocity
            self.last_dq = dq_planned

        self.safe_state_pub.publish(safe_msg)
        self.last_time = current_time

        # Logging
        elapsed_time = (current_time - self.start_time).to_sec()
        self.time_history.append(elapsed_time)
        self.tau_planned_history.append(tau_planned)
        self.tau_filtered_history.append(tau_safe)

    def plot_results(self):
        rospy.loginfo("Generating plots...")
        if not self.time_history: return
        times = np.array(self.time_history)
        tau_p = np.array(self.tau_planned_history)
        tau_f = np.array(self.tau_filtered_history)

        fig, axes = plt.subplots(7, 1, figsize=(12, 16), sharex=True)
        fig.suptitle(f'Filtered Joint Torque', fontsize=16)

        for i in range(7):
            ax = axes[i]
            limit = self.torque_limits[i]
            # Red = Original, Blue = Filtered, Green = SF4 threshold
            ax.plot(times, tau_p[:, i], 'r--', alpha=0.5, label='Generated')
            ax.plot(times, tau_f[:, i], 'b-', linewidth=2, label='Filtered')
            ax.axhline(limit , color='green', linestyle=':', label='Torque Limit')
            ax.axhline(-limit, color='green', linestyle=':')
            ax.set_ylabel(f'J{i+1} (Nm)')
            if i == 0: ax.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()

if __name__ == '__main__':
    try:
        AggressiveDynamicFilter()
        rospy.spin()
    except rospy.ROSInterruptException: pass