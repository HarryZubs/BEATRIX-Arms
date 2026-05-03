import pandas as pd
import matplotlib.pyplot as plt
import re
from scipy.signal import butter, filtfilt

# --- 1. Configuration & Settings ---
SAMPLE_RATE = 27.0  
CUTOFF_FREQ = 2.0   
START_ROW = 300
END_ROW = 500

# Define your moves here! Format is: (start_time_in_seconds, end_time_in_seconds)
# Time 0.0 is exactly at START_ROW.
MOVES = [
    (0, 1.6*5),  # Move 1: between 1 second and 2.5 seconds
    (1.6*5,3.25*5),  # Move 2: between 4 seconds and 5.5 seconds
    (3.25*5,5.0*5),
    (5.0*5,6.7*5)# You can add as many as you want here...
]

# --- 2. Helper Functions ---
def lowpass_filter(data, cutoff, fs, order=4):
    nyquist = 0.5 * fs 
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def get_closest_row(df, target_time):
    """Finds the row in the dataframe closest to the requested time in seconds."""
    # Calculates the absolute difference between all times and the target time, grabs the smallest one
    closest_index = (df['Time'] - target_time).abs().idxmin()
    return df.loc[closest_index]

# --- 3. Data Extraction & Filtering ---
clean_data = []
with open('bno055_data_2.csv', 'r') as file:
    for line in file:
        numbers = re.findall(r'[-+]?\d*\.\d+|\d+', line)
        if len(numbers) >= 3:
            clean_data.append([float(numbers[0]), float(numbers[1]), float(numbers[2])])

df = pd.DataFrame(clean_data, columns=['X', 'Y', 'Z'])

df['X'] = lowpass_filter(df['X'], CUTOFF_FREQ, SAMPLE_RATE)
df['Y'] = lowpass_filter(df['Y'], CUTOFF_FREQ, SAMPLE_RATE)
df['Z'] = lowpass_filter(df['Z'], CUTOFF_FREQ, SAMPLE_RATE)

# --- 4. Slice Data & Reset Time to Zero ---
# We use .copy() here to tell pandas we are making a distinct new table, which prevents warnings
df_subset = df.iloc[START_ROW:END_ROW].copy()

# Subtract the starting row from the current index, then divide by sample rate
df_subset['Time'] = (df_subset.index - START_ROW) / SAMPLE_RATE*5

# --- 5. Analyze Consistency Between Moves ---
move_deltas = {'X': [], 'Y': [], 'Z': []}

print("\n--- Move Analysis ---")
for i, (start_t, end_t) in enumerate(MOVES):
    start_row = get_closest_row(df_subset, start_t)
    end_row = get_closest_row(df_subset, end_t)
    
    # Calculate the change (Delta) for this move
    delta_x = end_row['X'] - start_row['X']
    delta_y = end_row['Y'] - start_row['Y']
    delta_z = end_row['Z'] - start_row['Z']
    
    # Save the deltas for later
    move_deltas['X'].append(delta_x)
    move_deltas['Y'].append(delta_y)
    move_deltas['Z'].append(delta_z)
    
    print(f"Move {i+1} ({start_t}s to {end_t}s):")
    print(f"  Change in X: {delta_x:.2f} | Y: {delta_y:.2f} | Z: {delta_z:.2f}")

print("\n--- Consistency (Variance as % of Overall Sliced Range) ---")
for axis in ['X', 'Y', 'Z']:
    if len(move_deltas[axis]) > 1:
        # 1. Calculate the raw range of the movement differences (Max - Min)
        delta_range = max(move_deltas[axis]) - min(move_deltas[axis])
        
        # 2. Find the absolute highest and lowest recorded points in the chopped series
        global_max = df_subset[axis].max()
        global_min = df_subset[axis].min()
        global_range = global_max - global_min
        
        # 3. Calculate percentage against the total series range
        if global_range > 0.001: # Safety check to prevent dividing by zero
            percent_variance = (delta_range / global_range) * 100
            print(f"{axis} Axis Range: {delta_range:.4f} degrees ({percent_variance:.2f}% of overall series range)")
        else:
            print(f"{axis} Axis Range: {delta_range:.4f} degrees (Overall range too small to calculate %)")
    else:
        print("Add more than one move to calculate a consistency range!")
print("--------------------------\n")

# --- 6. Plotting ---
plt.figure(figsize=(10, 6))

plt.plot(df_subset['Time'], df_subset['X'], label='X', color='red', alpha=0.8)
plt.plot(df_subset['Time'], df_subset['Y'], label='Y', color='green', alpha=0.8)
plt.plot(df_subset['Time'], df_subset['Z'], label='Z', color='blue', alpha=0.8)

# Overlay dots at the exact points we measured
for start_t, end_t in MOVES:
    start_row = get_closest_row(df_subset, start_t)
    end_row = get_closest_row(df_subset, end_t)
    
    # Plot a black dot at the start and end of each move for visual confirmation
    plt.scatter([start_row['Time'], end_row['Time']], [start_row['X'], end_row['X']], color='black', zorder=5)
    plt.scatter([start_row['Time'], end_row['Time']], [start_row['Y'], end_row['Y']], color='black', zorder=5)
    plt.scatter([start_row['Time'], end_row['Time']], [start_row['Z'], end_row['Z']], color='black', zorder=5)

plt.title(f"IMU reading Position Test 1", fontsize=14, fontweight='bold')
plt.xlabel("Time (s)")
plt.ylabel("Orientation (deg)")
plt.legend()
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()
plt.show()