#!/usr/bin/env python3

import rospy
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# --- 1. Configuration & Setup ---
package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
urdf_folder = os.path.join(package_root, "urdf")
os.makedirs(urdf_folder, exist_ok=True)
urdf_path = os.path.join(urdf_folder, "static.urdf")

model = pin.buildModelFromUrdf(urdf_path)

data = model.createData()

# Define your motor torque limits (Nm) for joints 1 through 7
torque_limits = np.array([11.34, 34.02, 11.34, 34.02, 0.47, 1.41, 0.47])

# Define dynamic movement targets
target_dq = np.ones(model.nq) * 5.0   # Velocity vector 
target_ddq = np.ones(model.nv) * 10.0  # Acceleration vector

num_samples = 10000 # Number of random poses to test
ee_frame_id = model.getFrameId("joint7") # The frame to track

# Mutually exclusive lists to prevent overlapping scatter plots
pts_kinematic_only = []
pts_static_only = []
pts_dynamic = []
pts_all = [] 

# Array to track the absolute maximum dynamic torque required for each joint
max_seen_torques = np.zeros(model.nv)

print("Simulating workspace...")

# --- 2. Monte Carlo Simulation Loop ---
for _ in range(num_samples):
    # Generate random joint angles within the URDF limits
    q = pin.randomConfiguration(model)
    
    # Calculate Forward Kinematics to find the End-Effector XYZ
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pos = data.oMf[ee_frame_id].translation.copy()
    pts_all.append(pos)
    
    # Calculate tau required just to hold against gravity
    tau_static = pin.computeGeneralizedGravity(model, data, q)
    
    # Calculate full Inverse Dynamics (M*ddq + C*dq + g) under our target velocity/acceleration
    tau_dynamic = pin.rnea(model, data, q, target_dq, target_ddq)
    
    # Update our maximum torque tracker with the highest absolute value seen so far
    max_seen_torques = np.maximum(max_seen_torques, np.abs(tau_dynamic))
    
    # Sort the point into mutually exclusive categories
    if np.all(np.abs(tau_static) <= torque_limits):
        if np.all(np.abs(tau_dynamic) <= torque_limits):
            # Passes both static and dynamic checks
            pts_dynamic.append(pos)
        else:
            # Passes static, but fails dynamic
            pts_static_only.append(pos)
    else:
        # Fails static check (kinematic only)
        pts_kinematic_only.append(pos)

# Convert lists to numpy arrays for plotting
pts_kinematic_only = np.array(pts_kinematic_only)
pts_static_only = np.array(pts_static_only)
pts_dynamic = np.array(pts_dynamic)
pts_all = np.array(pts_all)

# --- 3. Console Output ---
print(f"Total Poses Evaluated: {num_samples}")
print(f"Kinematic Only:        {len(pts_kinematic_only)}")
print(f"Static Only:           {len(pts_static_only)}")
print(f"Dynamically Reachable: {len(pts_dynamic)}")

print("\n" + "="*50)
print("MAXIMUM DYNAMIC TORQUES OBSERVED (Nm)")
print("="*50)
for i in range(model.nv):
    joint_name = model.names[i+1] # +1 because universe/root is index 0
    print(f"{joint_name:>10}: {max_seen_torques[i]:>8.3f} Nm")
print("="*50 + "\n")

# --- 4. Visualization ---
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot Dynamic 
if len(pts_dynamic) > 0:
    ax.scatter(pts_dynamic[:,0], pts_dynamic[:,1], pts_dynamic[:,2], 
               c='green', s=15, alpha=0.8, label='Dynamically Reachable (Green)')

# Plot Static Only
if len(pts_static_only) > 0:
    ax.scatter(pts_static_only[:,0], pts_static_only[:,1], pts_static_only[:,2], 
               c='yellow', s=5, alpha=0.3, label='Statically Reachable Only (Yellow)')

# Plot Kinematic Only 
if len(pts_kinematic_only) > 0:
    ax.scatter(pts_kinematic_only[:,0], pts_kinematic_only[:,1], pts_kinematic_only[:,2], 
               c='red', s=1, alpha=0.1, label='Kinematic Only - Too Weak (Red)')

ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_zlabel('Z (meters)')
ax.set_title('7-DOF Torque-Constrained Workspace')
ax.legend()

# Equalize axis scaling for a proportional 3D view using all generated points
if len(pts_all) > 0:
    max_range = np.array([pts_all[:,0].max()-pts_all[:,0].min(), 
                     #!/usr/bin/env python3

import rospy
import numpy as np
import scipy.signal
from scipy.interpolate import interp1d
from collections import deque
from sensor_msgs.msg import JointState

class AdaptiveErrorPublisher:
    def __init__(self):
        rospy.init_node('adaptive_error_publisher', anonymous=True)

        # --- Adaptive Parameters ---
        self.buffer_time = 1.5  # Seconds of history to keep for the sliding window
        self.dynamic_lag_sec = 0.050  # Seed value (50ms)
        self.lag_smoothing = 0.2  # Exponential Moving Average (EMA) alpha. Lower = smoother.
        
        # Max lengths assume roughly 100Hz publish rate. 1.5s * 100Hz = 150 points. 
        # We use slightly larger deques to be safe.
        self.safe_buffer = deque(maxlen=300)
        self.real_buffer = deque(maxlen=300)

        # --- Publishers & Subscribers ---
        self.error_pub = rospy.Publisher('/tracking_error', JointState, queue_size=10)
        
        rospy.Subscriber('/safe_joint_states', JointState, self.safe_callback)
        rospy.Subscriber('/joint_states_real', JointState, self.real_callback)

        # --- Background Lag Calculator ---
        # Run the heavy cross-correlation math in the background at 10Hz 
        # so it doesn't interrupt the high-speed real_callback loop.
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
        
        # Look back in time using our constantly updating dynamic lag
        target_time = current_time - self.dynamic_lag_sec

        # Find the safe command that was sent at exactly target_time
        closest_safe_msg = min(self.safe_buffer, key=lambda m: abs(m.header.stamp.to_sec() - target_time))

        time_diff = abs(closest_safe_msg.header.stamp.to_sec() - target_time)
        if time_diff > 0.1:
            # If we are missing data, don't publish a garbage error
            return

        if len(real_msg.position) >= 7 and len(closest_safe_msg.position) >= 7:
            real_pos = np.array(real_msg.position[:7])
            safe_pos = np.array(closest_safe_msg.position[:7])

            # Calculate True Error
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

        # Extract timestamps and velocities from the deques
        t_real = np.array([m.header.stamp.to_sec() for m in self.real_buffer])
        v_real = np.array([m.velocity[:7] for m in self.real_buffer])

        t_safe = np.array([m.header.stamp.to_sec() for m in self.safe_buffer])
        v_safe = np.array([m.velocity[:7] for m in self.safe_buffer])

        # Convert multidimensional joint velocities into a single magnitude representing overall robot speed
        v_real_mag = np.linalg.norm(v_real, axis=1)
        v_safe_mag = np.linalg.norm(v_safe, axis=1)

        # --- THE MOVEMENT GATE ---
        # If the max velocity in the window is less than 0.05 rad/s, the robot is functionally still.
        # Do not calculate lag. Keep the last known good lag.
        if np.max(v_real_mag) < 0.05 or np.max(v_safe_mag) < 0.05:
            return

        # --- Interpolation onto a uniform 1000Hz local timeline ---
        dt = 0.001
        t_min = max(t_real[0], t_safe[0])
        t_max = min(t_real[-1], t_safe[-1])
        
        if t_max <= t_min + 0.5:
            return # Window overlap too small
            
        t_common = np.arange(t_min, t_max, dt)

        try:
            interp_real = interp1d(t_real, v_real_mag, kind='linear')
            interp_safe = interp1d(t_safe, v_safe_mag, kind='linear')

            v_real_resampled = interp_real(t_common)
            v_safe_resampled = interp_safe(t_common)

            # Normalize to remove baseline offsets
            v_real_norm = v_real_resampled - np.mean(v_real_resampled)
            v_safe_norm = v_safe_resampled - np.mean(v_safe_resampled)

            # --- Cross-Correlation ---
            correlation = scipy.signal.correlate(v_real_norm, v_safe_norm, mode='full')
            lags = scipy.signal.correlation_lags(len(v_real_norm), len(v_safe_norm), mode='full')
            
            best_lag_idx = np.argmax(correlation)
            raw_calculated_lag = lags[best_lag_idx] * dt

            # Sanity Check: If the correlation says the lag is > 200ms or < 0ms, it's likely a false match. Ignore it.
            if 0.0 <= raw_calculated_lag <= 0.200:
                # Apply Exponential Moving Average to smooth out jitter
                self.dynamic_lag_sec = (self.lag_smoothing * raw_calculated_lag) + ((1.0 - self.lag_smoothing) * self.dynamic_lag_sec)
                
        except ValueError:
            # Catch interpolation bounds errors caused by thread timing offsets
            pass

if __name__ == '__main__':
    try:
        AdaptiveErrorPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass     pts_all[:,1].max()-pts_all[:,1].min(), 
                          pts_all[:,2].max()-pts_all[:,2].min()]).max() / 2.0
    mid_x = (pts_all[:,0].max()+pts_all[:,0].min()) * 0.5
    mid_y = (pts_all[:,1].max()+pts_all[:,1].min()) * 0.5
    mid_z = (pts_all[:,2].max()+pts_all[:,2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()