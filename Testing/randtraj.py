#!/usr/bin/env python3

import rospy
import math
import random
from sensor_msgs.msg import JointState

def dummy_data_publisher():
    rospy.init_node('dummy_trajectory_target_generator', anonymous=True)
    
   
    pub_real = rospy.Publisher('/joint_states_real', JointState, queue_size=10)
    pub_desired = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
   
    rate = rospy.Rate(50) 
    

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
    
    
    target_interval = 6.0  
    last_target_time = 0.0
    current_target = [0.0] * 7
    
    rospy.loginfo("Dummy publisher running. Publishing real=0 constantly.")
    rospy.loginfo(f"Generating new random target every {target_interval} seconds...")

   
    while rospy.get_time() == 0 and not rospy.is_shutdown():
        rate.sleep()

    while not rospy.is_shutdown():
        now = rospy.get_time()
        
       
        msg_real = JointState()
        msg_real.header.stamp = rospy.Time.now()
        msg_real.name = joint_names
        msg_real.position = [0.0] * 7 
        pub_real.publish(msg_real)
        
       
        if now - last_target_time > target_interval:
           
            current_target = [random.uniform(limit[0], limit[1]) for limit in limits]
            last_target_time = now
            
           
            formatted_target = [f"{val:+.2f}" for val in current_target]
            rospy.loginfo(f"New target generated: {formatted_target}")
            
        
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