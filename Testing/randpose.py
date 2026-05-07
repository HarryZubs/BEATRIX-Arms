#!/usr/bin/env python3

import rospy
import csv
import random
import os
from geometry_msgs.msg import PoseStamped

def publish_random_pose():
    
    rospy.init_node('random_target_publisher', anonymous=True)
    pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(50) # Publish continuously at 10 Hz

    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_filename = "7dof_poses.csv"
    csv_path = os.path.join(script_dir, csv_filename)

   
    try:
        with open(csv_path, mode='r') as file:
            reader = csv.reader(file)
            header = next(reader) 
            rows = list(reader)
    except FileNotFoundError:
        rospy.logerr(f"Failed to find {csv_filename} at {csv_path}")
        return

    if not rows:
        rospy.logerr("The CSV file is empty!")
        return

    
    target_msg = PoseStamped()
    target_msg.header.frame_id = "world" 
    
    update_interval = rospy.Duration(15.0) 
    last_update_time = rospy.Time.now()
    
   
    select_new_pose = True 

   
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

       
        if select_new_pose is True or (current_time - last_update_time) >= update_interval:
            random_row = random.choice(rows)
            
           
            x, y, z = float(random_row[7]), float(random_row[8]), float(random_row[9])
            qx, qy, qz, qw = float(random_row[10]), float(random_row[11]), float(random_row[12]), float(random_row[13])

           
            target_msg.pose.position.x = x
            target_msg.pose.position.y = y
            target_msg.pose.position.z = z
            target_msg.pose.orientation.x = qx
            target_msg.pose.orientation.y = qy
            target_msg.pose.orientation.z = qz
            target_msg.pose.orientation.w = qw

            rospy.loginfo(f"--- NEW TARGET (15s Elapsed) ---\n Pos: ({x:.3f}, {y:.3f}, {z:.3f})\n Ori: ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
            
        
            last_update_time = current_time
            select_new_pose = False

       
        target_msg.header.stamp = current_time
        pub.publish(target_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_random_pose()
    except rospy.ROSInterruptException:
        pass