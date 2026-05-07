#!/usr/bin/env python3

import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np
import scipy.signal
from scipy.interpolate import interp1d
from collections import deque
from sensor_msgs.msg import JointState

class AdaptiveErrorPublisher:
    def __init__(self):
        rospy.init_node('adaptive_error_publisher', anonymous=True)

        
        self.buffer_time = 1.5  
        self.dynamic_lag_sec = 0.050 
        self.lag_smoothing = 0.2 
  
        self.safe_buffer = deque(maxlen=300)
        self.real_buffer = deque(maxlen=300)

        self.error_pub = rospy.Publisher('/tracking_error', JointState, queue_size=10)
        
        rospy.Subscriber('/safe_joint_states', JointState, self.safe_callback)
        rospy.Subscriber('/joint_states_real', JointState, self.real_callback)


        rospy.Timer(rospy.Duration(0.1), self.calculate_dynamic_lag)

        rospy.loginfo("Adaptive Error Publisher active.")
        rospy.loginfo("Continuously correlating the last 1.5s of movement to adapt to jitter.")

    def safe_callback(self, msg):
        self.safe_buffer.append(msg)

    def real_callback(self, real_msg):
        self.real_buffer.append(real_msg)

        if not self.safe_buffer:
            return

        current_time = real_msg.header.stamp.to_sec()
        
       
        target_time = current_time - self.dynamic_lag_sec

    
        closest_safe_msg = min(self.safe_buffer, key=lambda m: abs(m.header.stamp.to_sec() - target_time))

        time_diff = abs(closest_safe_msg.header.stamp.to_sec() - target_time)
        if time_diff > 0.1:
    
            return

        if len(real_msg.position) >= 7 and len(closest_safe_msg.position) >= 7:
            real_pos = np.array(real_msg.position[:7])
            safe_pos = np.array(closest_safe_msg.position[:7])

      
            error_pos = real_pos - safe_pos

            err_msg = JointState()
            err_msg.header.stamp = real_msg.header.stamp 
            err_msg.name = real_msg.name[:7]
            err_msg.position = error_pos.tolist()
            self.error_pub.publish(err_msg)

    def calculate_dynamic_lag(self, event):
        """Runs at 10Hz to update the phase lag using a sliding window."""
        if len(self.real_buffer) < 50 or len(self.safe_buffer) < 50:
            return

  
        t_real = np.array([m.header.stamp.to_sec() for m in self.real_buffer])
        v_real = np.array([m.velocity[:7] for m in self.real_buffer])

        t_safe = np.array([m.header.stamp.to_sec() for m in self.safe_buffer])
        v_safe = np.array([m.velocity[:7] for m in self.safe_buffer])

       
        v_real_mag = np.linalg.norm(v_real, axis=1)
        v_safe_mag = np.linalg.norm(v_safe, axis=1)

       
        if np.max(v_real_mag) < 0.05 or np.max(v_safe_mag) < 0.05:
            return


        dt = 0.001
        t_min = max(t_real[0], t_safe[0])
        t_max = min(t_real[-1], t_safe[-1])
        
        if t_max <= t_min + 0.5:
            return 
            
        t_common = np.arange(t_min, t_max, dt)

        try:
            interp_real = interp1d(t_real, v_real_mag, kind='linear')
            interp_safe = interp1d(t_safe, v_safe_mag, kind='linear')

            v_real_resampled = interp_real(t_common)
            v_safe_resampled = interp_safe(t_common)

            # Normalize to remove baseline offsets
            v_real_norm = v_real_resampled - np.mean(v_real_resampled)
            v_safe_norm = v_safe_resampled - np.mean(v_safe_resampled)

           
            correlation = scipy.signal.correlate(v_real_norm, v_safe_norm, mode='full')
            lags = scipy.signal.correlation_lags(len(v_real_norm), len(v_safe_norm), mode='full')
            
            best_lag_idx = np.argmax(correlation)
            raw_calculated_lag = lags[best_lag_idx] * dt

            
            if 0.0 <= raw_calculated_lag <= 0.200:
         
                self.dynamic_lag_sec = (self.lag_smoothing * raw_calculated_lag) + ((1.0 - self.lag_smoothing) * self.dynamic_lag_sec)
                
        except ValueError:
            
            pass

if __name__ == '__main__':
    
    AdaptiveErrorPublisher()
    rospy.spin()
