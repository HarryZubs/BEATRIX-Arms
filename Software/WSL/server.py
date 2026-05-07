#!/usr/bin/env python3
import socket
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler
import time


latest_positions = [0.0] * 7  
recv_buffer = ""             
latest_unreachable_error = None 


latest_target_mode = "SIM"     
current_cartesian_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 


def joint_state_callback(msg):
    global latest_positions
    latest_positions = list(msg.position)[:7]  

def unreachable_error_callback(msg):
    global latest_unreachable_error
    latest_unreachable_error = msg.data


rospy.init_node("tcp_joint_server", anonymous=True)

rospy.Subscriber("/planned_joint_states", JointState, joint_state_callback, queue_size=1)
rospy.Subscriber("/unreachable_error", Float64, unreachable_error_callback, queue_size=1)

real_pub = rospy.Publisher("/joint_states_real", JointState, queue_size=1)
target_pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=1)


HOST = "0.0.0.0"
PORT = 5000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

server.bind((HOST, PORT))
server.listen(1)
print(f"TCP Server listening on port {PORT}")

def wait_for_connection():
    print("Waiting for connection...")
    conn, addr = server.accept()
    print(f"Connected by {addr}")
    conn.setblocking(False) 
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return conn

conn = wait_for_connection()


try:
    rate = rospy.Rate(50) 

    while not rospy.is_shutdown():
      
        if latest_unreachable_error is not None:
            err_msg = f"ERR,{latest_unreachable_error:.4f}\n"
            try:
                conn.sendall(err_msg.encode())
                latest_unreachable_error = None
            except BlockingIOError:
                pass 
            except (BrokenPipeError, ConnectionResetError, OSError):
                print("Connection lost. Waiting for new connection...")
                conn.close()
                conn = wait_for_connection()
                continue

       
        msg_out = ','.join(f"{p:.4f}" for p in latest_positions) + '\n'
        try:
            conn.sendall(msg_out.encode())
        except BlockingIOError:
            pass
        except (BrokenPipeError, ConnectionResetError, OSError):
            print("Connection lost. Waiting for new connection...")
            conn.close()
            conn = wait_for_connection()
            continue

       
        try:
            data = conn.recv(4096).decode()
            if data:
                recv_buffer += data
                
                while "\n" in recv_buffer:
                    line, recv_buffer = recv_buffer.split("\n", 1)
                    valid_line = line.strip()
                    
                    if valid_line:
                        parts = valid_line.split(',')
                        
                        if len(parts) > 0:
                            header = parts[0]

                           
                            if header == "FBK" and len(parts) == 9:
                                try:
                                    target_flag = parts[1] 
                                    real_positions = [float(p) for p in parts[2:]]
                                    
                                    js_msg = JointState()
                                    js_msg.header.stamp = rospy.Time.now()
                                    js_msg.header.frame_id = target_flag
                                    js_msg.name = [f"joint{i+1}" for i in range(7)]
                                    js_msg.position = real_positions
                                    real_pub.publish(js_msg)
                                except ValueError:
                                    pass

                          
                            elif header == "CMD" and len(parts) == 9:
                                try:
                                    target_flag = parts[1] 
                                    
                                  
                                    latest_target_mode = target_flag 
                                    
                                    mode_flag = parts[2]  
                                    coords = [float(p) for p in parts[3:]]
                                    
                                  
                                    if mode_flag == "ABS":
                                     
                                        current_cartesian_pose = coords
                                    elif mode_flag == "REL":
                                        
                                        current_cartesian_pose = [curr + delta for curr, delta in zip(current_cartesian_pose, coords)]
                                    
                                    
                                    pose_msg = PoseStamped()
                                    pose_msg.header.stamp = rospy.Time.now()
                                    pose_msg.header.frame_id = f"{target_flag}_{mode_flag}"
                                    
                                    pose_msg.pose.position.x = current_cartesian_pose[0]
                                    pose_msg.pose.position.y = current_cartesian_pose[1]
                                    pose_msg.pose.position.z = current_cartesian_pose[2]
                                    
                                    q = quaternion_from_euler(current_cartesian_pose[3], 
                                                              current_cartesian_pose[4], 
                                                              current_cartesian_pose[5])
                                    
                                    pose_msg.pose.orientation.x = q[0]
                                    pose_msg.pose.orientation.y = q[1]
                                    pose_msg.pose.orientation.z = q[2]
                                    pose_msg.pose.orientation.w = q[3]
                                    
                                    target_pub.publish(pose_msg)
                                    print(f"Command published: {target_flag} {mode_flag} Updated XYZ: {current_cartesian_pose[:3]}")
                                except ValueError:
                                    print("Invalid numbers in CMD data:", valid_line)
                            
        except BlockingIOError:
            pass 
        except (BrokenPipeError, ConnectionResetError, OSError):
            print("Connection lost during receive. Waiting for new connection...")
            conn.close()
            conn = wait_for_connection()
            continue

        rate.sleep()

except KeyboardInterrupt:
    print("\nShutting down server...")

finally:
    try:
        conn.close()
        server.close()
    except Exception:
        pass
    print("Server shut down.")