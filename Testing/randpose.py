#!/usr/bin/env python3

import rospy
import csv
import random
import os
from geometry_msgs.msg import PoseStamped

def publish_random_pose():
    # 1. Initialize the ROS node and Publisher
    rospy.init_node('random_target_publisher', anonymous=True)
    pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(50) # Publish continuously at 10 Hz

    # 2. Locate the CSV in the SAME folder as this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_filename = "7dof_poses.csv"
    csv_path = os.path.join(script_dir, csv_filename)

    # 3. Load the CSV Data
    try:
        with open(csv_path, mode='r') as file:
            reader = csv.reader(file)
            header = next(reader) # Skip the header row
            rows = list(reader)
    except FileNotFoundError:
        rospy.logerr(f"Failed to find {csv_filename} at {csv_path}")
        return

    if not rows:
        rospy.logerr("The CSV file is empty!")
        return

    # 4. Setup Target Message and Timers
    target_msg = PoseStamped()
    target_msg.header.frame_id = "world" # Update to match your robot's base frame if needed
    
    update_interval = rospy.Duration(15.0) # 15 seconds
    last_update_time = rospy.Time.now()
    
    # Force the first selection immediately upon starting
    select_new_pose = True 

    # 5. Publishing Loop
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Check if 15 seconds have passed (or if it's the very first run)
        if select_new_pose is True or (current_time - last_update_time) >= update_interval:
            random_row = random.choice(rows)
            
            # Extract Cartesian data (columns 7-13 based on our generation script)
            x, y, z = float(random_row[7]), float(random_row[8]), float(random_row[9])
            qx, qy, qz, qw = float(random_row[10]), float(random_row[11]), float(random_row[12]), float(random_row[13])

            # Update the message data
            target_msg.pose.position.x = x
            target_msg.pose.position.y = y
            target_msg.pose.position.z = z
            target_msg.pose.orientation.x = qx
            target_msg.pose.orientation.y = qy
            target_msg.pose.orientation.z = qz
            target_msg.pose.orientation.w = qw

            rospy.loginfo(f"--- NEW TARGET (15s Elapsed) ---\n Pos: ({x:.3f}, {y:.3f}, {z:.3f})\n Ori: ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
            
            # Reset timer
            last_update_time = current_time
            select_new_pose = False

        # Update timestamp and publish the current target
        target_msg.header.stamp = current_time
        pub.publish(target_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_random_pose()
    except rospy.ROSInterruptException:
        pass