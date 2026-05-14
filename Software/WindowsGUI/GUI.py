#!/usr/bin/env python3
import socket
import time
import threading
import queue
import tkinter as tk
import serial 
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Socket setup
HOST = "172.28.203.158"
PORT = 5000

# Arduino setup
SERIAL_PORT = "COM3" 
BAUD_RATE = 115200


last_received_positions = []
recv_buffer = ""
command_queue = queue.Queue()
trajectory_queue = queue.Queue() 


current_move_target = "SIM" 


tcp = None
sim = None
arduino = None

ros_connected = False
sim_connected = False
arduino_connected = False
latest_ik_error = None 


def send_pos(pos_list):
    if not pos_list or sim is None: return
    for i, val in enumerate(pos_list, start=1):
        sim.setStringSignal(f"pos{i}", str(val))

def read_accPos():
    if sim is None: return [0.0] * 7
    acc = []
    for i in range(1, 8):
        try:
            val_str = sim.getStringSignal(f"accPos{i}")
            val = float(val_str) if val_str else 0.0
        except Exception:
            val = 0.0
        acc.append(val)
    return acc

# Arduno Thread
def serial_loop():
    global arduino, arduino_connected
    
    while True:
        if arduino is None:
            try:
                temp_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                time.sleep(2) 
                arduino = temp_serial
                arduino_connected = True
            except Exception:
                arduino_connected = False
                time.sleep(1)
                continue

        next_step_time = time.time()
        
        while arduino_connected:
            if not trajectory_queue.empty():
                waypoint = trajectory_queue.get()
                
                msg_str = ",".join(f"{x:.4f}" for x in waypoint) + "\n"
                
                try:
                    arduino.write(msg_str.encode())
                except Exception:
                    arduino.close()
                    arduino = None
                    arduino_connected = False
                    break
                
                next_step_time += 0.100 
                sleep_duration = next_step_time - time.time()
                
                if sleep_duration > 0:
                    time.sleep(sleep_duration)
            else:
                next_step_time = time.time()
                time.sleep(0.01)

# ROS Comms Thread
def network_loop():
    global tcp, sim, last_received_positions, recv_buffer
    global ros_connected, sim_connected, latest_ik_error, current_move_target

    while tcp is None:
        try:
            temp_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            temp_tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            temp_tcp.connect((HOST, PORT))
            temp_tcp.settimeout(0.01)
            tcp = temp_tcp
            ros_connected = True
        except Exception:
            ros_connected = False
            time.sleep(1)

    while sim is None:
        try:
            client = RemoteAPIClient()
            sim = client.getObject('sim')
            sim_connected = True
        except Exception:
            sim_connected = False
            time.sleep(1)

    while not last_received_positions:
        last_received_positions = read_accPos()
        time.sleep(0.1)

    try:
        while True:
       
            while not command_queue.empty():
              
                target_val, cmd_str = command_queue.get()
                current_move_target = target_val # Update the global state
                
                try:
                    tcp.sendall(cmd_str.encode())
                    time.sleep(0.005) 
                except Exception:
                    ros_connected = False
                    return

            try:
                data = tcp.recv(4096).decode()
                if data:
                    recv_buffer += data
                    
                    while "\n" in recv_buffer:
                        line, recv_buffer = recv_buffer.split("\n", 1)
                        valid_line = line.strip()
                        
                        if valid_line:
                            parts = valid_line.split(',')
                            
                            if parts[0] == "ERR" and len(parts) == 2:
                                latest_ik_error = parts[1]
                                
                            elif parts[0] == "TRAJ" and len(parts) == 8:
                            
                                if current_move_target == "REAL":
                                    try:
                                        waypoint = [float(p) for p in parts[1:8]]
                                        trajectory_queue.put(waypoint)
                                    except ValueError:
                                        pass 

                            elif len(parts) == 7:
                               
                                try:
                                    last_received_positions = [float(p) for p in parts]
                                except ValueError:
                                    pass 
            except socket.timeout:
                pass
            except Exception:
                ros_connected = False
                break

            # --- BRIDGE DATA ---
            send_pos(last_received_positions)
            accPos = read_accPos()

            # --- SEND FEEDBACK ---
            try:
                target_flag = "SIM" 
                msg_out = f"FBK,{target_flag}," + ','.join(f"{x:.4f}" for x in accPos) + "\n"
                tcp.sendall(msg_out.encode())
            except Exception:
                ros_connected = False
                break

            time.sleep(0.02)

    except Exception:
        ros_connected = False
    finally:
        if tcp: tcp.close()
        ros_connected = False

# GUI Thread
def start_gui():
    root = tk.Tk()
    root.title("Robot Cartesian Controller")
    root.geometry("380x550") 
    root.resizable(False, False)

    target_var = tk.StringVar(value="SIM")
    mode_var = tk.StringVar(value="ABS")
    feedback_msg = tk.StringVar(value="Ready.")

    status_frame = tk.LabelFrame(root, text="System Status", font=("Arial", 10, "bold"), padx=10, pady=5)
    status_frame.pack(fill="x", padx=15, pady=10)

    tk.Label(status_frame, text="WSL ROS Server:").grid(row=0, column=0, sticky="w")
    ros_lbl = tk.Label(status_frame, text="🔴 Disconnected", fg="red")
    ros_lbl.grid(row=0, column=1, sticky="w", padx=10)

    tk.Label(status_frame, text="CoppeliaSim:").grid(row=1, column=0, sticky="w")
    sim_lbl = tk.Label(status_frame, text="🔴 Disconnected", fg="red")
    sim_lbl.grid(row=1, column=1, sticky="w", padx=10)

    tk.Label(status_frame, text="Real Robot (Arduino):").grid(row=2, column=0, sticky="w")
    real_lbl = tk.Label(status_frame, text="🔴 Disconnected ", fg="red")
    real_lbl.grid(row=2, column=1, sticky="w", padx=10)

    tk.Label(root, text="Target Robot", font=("Arial", 10, "bold")).pack(pady=(5,0))
    frame_target = tk.Frame(root)
    frame_target.pack()
    tk.Radiobutton(frame_target, text="Simulator", variable=target_var, value="SIM").pack(side=tk.LEFT)
    tk.Radiobutton(frame_target, text="Real Robot", variable=target_var, value="REAL").pack(side=tk.LEFT)

    tk.Label(root, text="Move Type", font=("Arial", 10, "bold")).pack(pady=(10,0))
    frame_mode = tk.Frame(root)
    frame_mode.pack()
    tk.Radiobutton(frame_mode, text="Absolute", variable=mode_var, value="ABS").pack(side=tk.LEFT)
    tk.Radiobutton(frame_mode, text="Relative", variable=mode_var, value="REL").pack(side=tk.LEFT)

    tk.Label(root, text="Cartesian Pose", font=("Arial", 10, "bold")).pack(pady=(10,0))
    frame_pose = tk.Frame(root)
    frame_pose.pack()

    entries = {}
    labels = ["X", "Y", "Z", "Roll (Rx)", "Pitch (Ry)", "Yaw (Rz)"]
    for i, axis in enumerate(labels):
        tk.Label(frame_pose, text=f"{axis}:").grid(row=i, column=0, sticky="e", padx=5, pady=2)
        e = tk.Entry(frame_pose, width=15)
        e.insert(0, "0.0")
        e.grid(row=i, column=1, pady=2)
        entries[axis] = e

    def submit_command():
        if not ros_connected:
            feedback_msg.set(" Error: Not connected to WSL Server!")
            return
        
        if target_var.get() == "REAL" and not arduino_connected:
            feedback_msg.set(" Error: Arduino not connected on Serial!")
            return

        if target_var.get() == "SIM" and not sim_connected:
            feedback_msg.set(" Error: Not connected to Simulator!")
            return

        try:
            x = float(entries["X"].get())
            y = float(entries["Y"].get())
            z = float(entries["Z"].get())
            rx = float(entries["Roll (Rx)"].get())
            ry = float(entries["Pitch (Ry)"].get())
            rz = float(entries["Yaw (Rz)"].get())
            
            cmd_str = f"CMD,{target_var.get()},{mode_var.get()},{x},{y},{z},{rx},{ry},{rz}\n"
            
           
            command_queue.put((target_var.get(), cmd_str))
            
            feedback_msg.set(f" Sending command to {target_var.get()}...")

        except ValueError:
            feedback_msg.set(" Error: All pose inputs must be numbers.")

    tk.Button(root, text="SEND COMMAND", bg="#4CAF50", fg="white", font=("Arial", 12, "bold"), command=submit_command).pack(pady=15)

    tk.Label(root, textvariable=feedback_msg, font=("Arial", 9, "bold"), fg="red").pack(pady=5)

    def poll_connections():
        global latest_ik_error 
        
        ros_lbl.config(text="🟢 Connected", fg="green") if ros_connected else ros_lbl.config(text="🔴 Disconnected", fg="red")
        sim_lbl.config(text="🟢 Connected", fg="green") if sim_connected else sim_lbl.config(text="🔴 Disconnected", fg="red")
        real_lbl.config(text="🟢 Connected", fg="green") if arduino_connected else real_lbl.config(text="🔴 Disconnected", fg="red")
            
        if latest_ik_error is not None:
            feedback_msg.set(f" IK Error: Unreachable pose (Missed by {latest_ik_error}m)")
            latest_ik_error = None 
            
        root.after(200, poll_connections)

    poll_connections()
    root.mainloop()

if __name__ == "__main__":
    net_thread = threading.Thread(target=network_loop, daemon=True)
    net_thread.start()
    
    ser_thread = threading.Thread(target=serial_loop, daemon=True)
    ser_thread.start()
    
    start_gui()
