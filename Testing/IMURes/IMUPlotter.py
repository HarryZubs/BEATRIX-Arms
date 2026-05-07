import pandas as pd
import matplotlib.pyplot as plt
import re
from scipy.signal import butter, filtfilt


SAMPLE_RATE = 27.0  
CUTOFF_FREQ = 2.0   
START_ROW = 300
END_ROW = 500


MOVES = [
    (0, 1.6*5),  
    (1.6*5,3.25*5),  
    (3.25*5,5.0*5),
    (5.0*5,6.7*5)
]


def lowpass_filter(data, cutoff, fs, order=4):
    nyquist = 0.5 * fs 
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def get_closest_row(df, target_time):
    """Finds the row in the dataframe closest to the requested time in seconds."""
    
    closest_index = (df['Time'] - target_time).abs().idxmin()
    return df.loc[closest_index]


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


df_subset = df.iloc[START_ROW:END_ROW].copy()


df_subset['Time'] = (df_subset.index - START_ROW) / SAMPLE_RATE*5

move_deltas = {'X': [], 'Y': [], 'Z': []}

print("\n--- Move Analysis ---")
for i, (start_t, end_t) in enumerate(MOVES):
    start_row = get_closest_row(df_subset, start_t)
    end_row = get_closest_row(df_subset, end_t)
    
   
    delta_x = end_row['X'] - start_row['X']
    delta_y = end_row['Y'] - start_row['Y']
    delta_z = end_row['Z'] - start_row['Z']
    

    move_deltas['X'].append(delta_x)
    move_deltas['Y'].append(delta_y)
    move_deltas['Z'].append(delta_z)
    
    print(f"Move {i+1} ({start_t}s to {end_t}s):")
    print(f"  Change in X: {delta_x:.2f} | Y: {delta_y:.2f} | Z: {delta_z:.2f}")

print("\n--- Consistency (Variance as % of Overall Sliced Range) ---")
for axis in ['X', 'Y', 'Z']:
    if len(move_deltas[axis]) > 1:
       
        delta_range = max(move_deltas[axis]) - min(move_deltas[axis])
        
       
        global_max = df_subset[axis].max()
        global_min = df_subset[axis].min()
        global_range = global_max - global_min
        
        if global_range > 0.001: 
            percent_variance = (delta_range / global_range) * 100
            print(f"{axis} Axis Range: {delta_range:.4f} degrees ({percent_variance:.2f}% of overall series range)")
        else:
            print(f"{axis} Axis Range: {delta_range:.4f} degrees (Overall range too small to calculate %)")
    else:
        print("Add more than one move to calculate a consistency range!")
print("--------------------------\n")


plt.figure(figsize=(10, 6))

plt.plot(df_subset['Time'], df_subset['X'], label='X', color='red', alpha=0.8)
plt.plot(df_subset['Time'], df_subset['Y'], label='Y', color='green', alpha=0.8)
plt.plot(df_subset['Time'], df_subset['Z'], label='Z', color='blue', alpha=0.8)

for start_t, end_t in MOVES:
    start_row = get_closest_row(df_subset, start_t)
    end_row = get_closest_row(df_subset, end_t)
    
    
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