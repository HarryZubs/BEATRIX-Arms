#!/usr/bin/env python3

import rospy
import math
import random
from sensor_msgs.msg import JointState

def dummy_data_publisher():
    rospy.init_node('dummy_trajectory_target_generator', anonymous=True)
    
    # Publishers
    pub_real = rospy.Publisher('/joint_states_real', JointState, queue_size=10)
    pub_desired = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    # Run at 50Hz to match your planner's loop rate
    rate = rospy.Rate(50) 
    
    # Your specified joint limits
    limits = [
        [-math.pi/2, math.pi/2],
        [-math.pi/6, math.pi],
        [-math.pi/2, math.pi/2],
        [0, 5*math.pi/6],
        [-math.pi/2, math.pi/2],
        [-math.pi/2, math.pi/2],
        [-math.pi/3, math.pi/3]
    ]
    
    joint_names = [f"joint{i+1}" for i in range(7)]
    
    # Timing variables for generating new targets
    target_interval = 6.0  # seconds (longer than your 5s trajectory)
    last_target_time = 0.0
    current_target = [0.0] * 7
    
    rospy.loginfo("Dummy publisher running. Publishing real=0 constantly.")
    rospy.loginfo(f"Generating new random target every {target_interval} seconds...")

    # Wait for ROS time to initialize (important if using use_sim_time)
    while rospy.get_time() == 0 and not rospy.is_shutdown():
        rate.sleep()

    while not rospy.is_shutdown():
        now = rospy.get_time()
        
        # --- 1. Publish Real State (Constantly 0.0) ---
        msg_real = JointState()
        msg_real.header.stamp = rospy.Time.now()
        msg_real.name = joint_names
        msg_real.position = [0.0] * 7 
        pub_real.publish(msg_real)
        
        # --- 2. Generate New Target (Every 6 seconds) ---
        if now - last_target_time > target_interval:
            # Generate a new random configuration within the limits
            current_target = [random.uniform(limit[0], limit[1]) for limit in limits]
            last_target_time = now
            
            # Print a nicely formatted string of the new target so you can track it in the terminal
            formatted_target = [f"{val:+.2f}" for val in current_target]
            rospy.loginfo(f"New target generated: {formatted_target}")
            
        # --- 3. Publish Desired State (Constantly publishing the current target) ---
        msg_desired = JointState()
        msg_desired.header.stamp = rospy.Time.now()
        msg_desired.name = joint_names
        msg_desired.position = current_target
        pub_desired.publish(msg_desired)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        dummy_data_publisher()
    except rospy.ROSInterruptException:
        pass