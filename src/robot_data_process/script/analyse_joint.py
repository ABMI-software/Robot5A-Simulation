import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

# Define paths
input_csv_path = '/home/chipmunk-151/Robot5A/src/robot_data_process/data_analysis/logs/joint_sync_log.csv'
output_pdf_path = '/home/chipmunk-151/Robot5A/src/robot_data_process/data_analysis/output/joint_sync_log_analysis.pdf'

# Load the CSV file
df = pd.read_csv(input_csv_path)

# Ensure time starts at 0 for better readability
df['Time (s)'] -= df['Time (s)'].min()

# Function to parse space-separated lists in CSV
def parse_space_separated(column):
    return column.apply(lambda x: [float(val) if val != 'nan' else np.nan for val in str(x).split()] if pd.notna(x) and x != 'N/A' else [])

# Parse joint positions
df['True Positions'] = parse_space_separated(df['True Positions'])
df['Combined Positions'] = parse_space_separated(df['Combined Positions'])

# Extract joint names from the first non-NaN row
def get_first_valid_names(column):
    for entry in df[column]:
        if pd.notna(entry) and entry != 'N/A':
            return entry.split()
    return []

true_joint_names = get_first_valid_names('True Joints')
combined_joint_names = get_first_valid_names('Combined Joints')

if not true_joint_names or not combined_joint_names:
    raise ValueError("No valid joint names found in 'True Joints' or 'Combined Joints' columns.")

# Calculate position errors for matching joints
error_columns = {}
for i, joint in enumerate(true_joint_names):
    if joint in combined_joint_names:
        j_idx = combined_joint_names.index(joint)
        df[f'Error {joint} (rad)'] = df['True Positions'].apply(lambda x: x[i] if x and len(x) > i else np.nan) - \
                                     df['Combined Positions'].apply(lambda x: x[j_idx] if x and len(x) > j_idx else np.nan)
        error_columns[joint] = f'Error {joint} (rad)'

# Compute statistics for errors
stats = {}
for joint, col in error_columns.items():
    stats[joint] = {
        'Mean Error (rad)': np.abs(df[col]).mean(),  # Absolute mean error
        'Std Dev (rad)': np.abs(df[col]).std()       # Std Dev of absolute errors
    }

# Create PDF report
with PdfPages(output_pdf_path) as pdf:
    # Page 1: Arm Joint Positions Plot
    plt.figure(figsize=(12, 8))
    arm_joints = ["R0_Yaw", "R1_Pitch", "R2_Pitch", "R3_Yaw", "R4_Pitch"]
    colors = ['blue', 'orange', 'green', 'red', 'purple']
    
    for i, joint in enumerate(arm_joints):
        if joint in true_joint_names:
            j_idx = true_joint_names.index(joint)
            plt.plot(df['Time (s)'].to_numpy(), 
                     df['True Positions'].apply(lambda x: x[j_idx] if x and len(x) > j_idx else np.nan).to_numpy(), 
                     label=f'{joint} True', color=colors[i % len(colors)], linestyle='-')
        if joint in combined_joint_names:
            j_idx = combined_joint_names.index(joint)
            plt.plot(df['Time (s)'].to_numpy(), 
                     df['Combined Positions'].apply(lambda x: x[j_idx] if x and len(x) > j_idx else np.nan).to_numpy(), 
                     label=f'{joint} Combined', color=colors[i % len(colors)], linestyle='--')
    
    # Highlight command execution periods
    command_colors = ['lime', 'cyan', 'magenta', 'yellow']
    color_idx = 0
    start_time = None
    current_cmd = None
    
    for i in range(len(df)):
        cmd = df['Command'].iloc[i]
        if cmd != 'Idle':
            if current_cmd is None or cmd != current_cmd:
                if start_time is not None:
                    end_time = df['Time (s)'].iloc[i-1]
                    color = command_colors[color_idx % len(command_colors)]
                    plt.axvspan(start_time, end_time, alpha=0.2, color=color, 
                               label=f'Command: {current_cmd}' if color_idx == 0 else "")
                    color_idx += 1
                start_time = df['Time (s)'].iloc[i]
                current_cmd = cmd
            if i == len(df) - 1:
                end_time = df['Time (s)'].iloc[i]
                color = command_colors[color_idx % len(command_colors)]
                plt.axvspan(start_time, end_time, alpha=0.2, color=color,
                           label=f'Command: {current_cmd}' if color_idx == 0 else "")
    
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.title('True vs Combined Arm Joint Positions')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True)
    plt.tight_layout()
    pdf.savefig()
    plt.close()

    # Page 2: Absolute Error Plot for Arm Joints
    plt.figure(figsize=(12, 8))
    for i, joint in enumerate(arm_joints):
        if joint in error_columns:
            plt.plot(df['Time (s)'].to_numpy(), 
                     np.abs(df[error_columns[joint]]).to_numpy(), 
                     label=f'|{joint} Error|', color=colors[i % len(colors)])
            plt.axhline(stats[joint]['Mean Error (rad)'], color=colors[i % len(colors)], linestyle='--',
                       label=f'{joint} Mean: {stats[joint]["Mean Error (rad)"]:.4f} rad')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Absolute Error (rad)')
    plt.title('Absolute Errors Between True and Combined Arm Joint Positions')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True)
    plt.tight_layout()
    pdf.savefig()
    plt.close()

    # Page 3: Statistics Table
    plt.figure(figsize=(10, 6))
    plt.axis('off')
    table_data = [[joint, f"{stats[joint]['Mean Error (rad)']:.4f}", f"{stats[joint]['Std Dev (rad)']:.4f}"] 
                  for joint in arm_joints if joint in stats]
    table = plt.table(cellText=table_data,
                     colLabels=['Joint', 'Mean Error (rad)', 'Standard Deviation (rad)'],
                     loc='center',
                     cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.2)
    plt.title('Statistical Analysis of Arm Joint Errors', pad=20)
    pdf.savefig()
    plt.close()

print(f"PDF report generated: {output_pdf_path}")