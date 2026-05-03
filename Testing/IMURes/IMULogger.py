import serial
import time

# --- CONFIGURATION ---
# Replace with your actual port. 
# Find this in the Arduino IDE under Tools > Port
SERIAL_PORT = 'COM10' 
BAUD_RATE = 9600
OUTPUT_FILE = 'bno055_data_or4.csv'

try:
    # Open the serial connection
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    # Opening the serial port often resets the Arduino. 
    # Pause for 2 seconds to let it boot up before we start reading.
    time.sleep(2) 

    # Open the CSV file in 'append' mode ('a') so we don't overwrite older data
    # If the file doesn't exist, Python will create it.
    with open(OUTPUT_FILE, 'a', encoding='utf-8') as file:
        print(f"Connected! Logging data to '{OUTPUT_FILE}'")
        print("Press Ctrl+C in this terminal to stop recording.\n")
        
        while True:
            # Check if there is data waiting in the buffer
            if ser.in_waiting > 0:
                # Read the line, decode the bytes to text, and strip trailing whitespace/newlines
                line = ser.readline().decode('utf-8').strip()
                
                # Only save if the line isn't blank
                if line:
                    file.write(line + '\n')
                    # Print to the terminal so you can monitor it live
                    print(line) 


except KeyboardInterrupt:
    print("\n[STOPPED] Data logging stopped by user.")
finally:
    # This block ensures the port is safely closed even if you crash the script
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed safely.")