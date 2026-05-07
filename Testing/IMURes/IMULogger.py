import serial
import time


SERIAL_PORT = 'COM10' 
BAUD_RATE = 9600
OUTPUT_FILE = 'bno055_data_or4.csv'

try:
  
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
   
    time.sleep(2) 

   
    with open(OUTPUT_FILE, 'a', encoding='utf-8') as file:
        print(f"Connected! Logging data to '{OUTPUT_FILE}'")
        print("Press Ctrl+C in this terminal to stop recording.\n")
        
        while True:
           
            if ser.in_waiting > 0:
                
                line = ser.readline().decode('utf-8').strip()
                
                
                if line:
                    file.write(line + '\n')
                    
                    print(line) 


except KeyboardInterrupt:
    print("\n[STOPPED] Data logging stopped by user.")
finally:

    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed safely.")