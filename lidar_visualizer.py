#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from lidar_driver import LidarDriver
import time

# Configuration
LIDAR_PORT = '/dev/ttyUSB0'
MAX_DISTANCE_M = 5.0  # Meters
DANGER_ZONE_M = 0.5   # Meters
WARNING_ZONE_M = 1.5  # Meters

class LidarVisualizer:
    def __init__(self):
        print("Initializing Lidar...")
        self.lidar = LidarDriver(port=LIDAR_PORT)
        self.lidar.start()
        
        self.start_time = time.time()
        self.is_calibrated = False
        self.calibration_duration = 3.0 # seconds
        
        # Setup the plot
        self.fig = plt.figure(figsize=(10, 10))
        self.fig.patch.set_facecolor('#1e1e1e') # Dark background
        
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.ax.set_facecolor('#2d2d2d')
        
        # Configure plot
        self.ax.set_ylim(0, MAX_DISTANCE_M)
        self.ax.set_theta_zero_location('N')  # 0 degrees at top (FRONT)
        self.ax.set_theta_direction(-1)  # Clockwise
        
        # Set tick labels color
        self.ax.tick_params(colors='white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        
        # Grid and Labels
        self.ax.set_title('ROBOT LIDAR SYSTEM', pad=30, fontsize=16, fontweight='bold', color='cyan')
        self.ax.grid(True, color='gray', linestyle='--', alpha=0.5)
        
        # Add compass-like labels
        self.ax.set_xticklabels(['FRONT', '45°', 'RIGHT', '135°', 'BACK', '225°', 'LEFT', '315°'])
        
        # Zone Shading
        # Danger zone (Red)
        self.ax.fill_between(np.linspace(0, 2*np.pi, 100), 0, DANGER_ZONE_M, color='red', alpha=0.1)
        # Warning zone (Yellow)
        self.ax.fill_between(np.linspace(0, 2*np.pi, 100), DANGER_ZONE_M, WARNING_ZONE_M, color='orange', alpha=0.05)
        
        # Initial empty scatter plot
        self.scatter = self.ax.scatter([], [], c='cyan', s=8, alpha=0.8, edgecolors='none')
        
        # Status Text (Large and centered during calibration)
        self.status_text = self.fig.text(0.5, 0.5, 'INITIALIZING...', 
                                         color='yellow', fontsize=30, fontweight='bold',
                                         ha='center', va='center', alpha=0.8)
        
        # Side Info Panel
        self.info_text = self.fig.text(0.02, 0.98, '', verticalalignment='top', 
                                       fontsize=12, family='monospace', color='white',
                                       bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
        
    def update_plot(self, frame):
        elapsed = time.time() - self.start_time
        
        # Get latest scan
        scan = self.lidar.get_scan()
        
        # Handle Calibration Phase
        if not self.is_calibrated:
            if elapsed < self.calibration_duration:
                progress = (elapsed / self.calibration_duration) * 100
                self.status_text.set_text(f"CALIBRATING... {progress:.0f}%")
                return self.scatter,
            else:
                self.is_calibrated = True
                self.status_text.set_alpha(0) # Hide status text
        
        if len(scan) == 0:
            return self.scatter,
        
        # Data processing
        angles = []
        distances = []
        nearest_m = float('inf')
        points_in_front = []
        
        for angle, dist_mm in scan:
            dist_m = dist_mm / 1000.0
            if 0.1 < dist_m < MAX_DISTANCE_M:  # Filter noise and max range
                angles.append(np.deg2rad(angle))
                distances.append(dist_m)
                
                if dist_m < nearest_m:
                    nearest_m = dist_m
                
                # Front cone ±20 degrees
                if abs(angle) < 20:
                    points_in_front.append(dist_m)
        
        if len(angles) > 0:
            # Update data
            self.scatter.set_offsets(np.column_stack([angles, distances]))
            
            # Dynamic coloring based on distance
            colors = []
            for d in distances:
                if d < DANGER_ZONE_M: colors.append('#ff0000') # Bright Red
                elif d < WARNING_ZONE_M: colors.append('#ffa500') # Orange
                else: colors.append('#00ffff') # Cyan
            self.scatter.set_color(colors)
        
        # Human Readable Logic with Sensitivity Filtering
        status = "SYSTEM ACTIVE"
        status_color = "lightgreen"
        
        # Count points in zones for filtering out noise
        # Sensitivity: Require at least 5 points to confirm an obstacle
        danger_points = [d for d in points_in_front if d < DANGER_ZONE_M]
        warning_points = [d for d in points_in_front if d < WARNING_ZONE_M]
        
        if len(danger_points) >= 5: # Obstacle at 50cm detected by >5 sensors/samples
            status = "!!! CRITICAL OBSTACLE !!!"
            status_color = "red"
            front_dist = min(danger_points)
        elif len(warning_points) >= 5: # Warning zone detection
            status = "WARNING: OBJECT AHEAD"
            status_color = "orange"
            front_dist = min(warning_points)
        else:
            front_dist = MAX_DISTANCE_M

        # Update Info Panel
        info = f"--- STATUS: {status} ---\n\n"
        info += f"Detection: {len(angles)} points\n"
        info += f"Range    : {MAX_DISTANCE_M}m\n"
        info += f"Nearest  : {nearest_m:.2f}m\n"
        if front_dist < MAX_DISTANCE_M:
            info += f"Front Dist: {front_dist:.2f}m\n"
        else:
            info += "Front Dist: CLEAR\n"
            
        self.info_text.set_text(info)
        self.info_text.get_bbox_patch().set_edgecolor(status_color)
        
        return self.scatter,
    
    def run(self):
        ani = animation.FuncAnimation(
            self.fig, self.update_plot, 
            interval=100,
            blit=False, # Set to False for dynamic updates like color
            cache_frame_data=False
        )
        plt.show()
        
    def cleanup(self):
        self.lidar.stop()

if __name__ == '__main__':
    viz = LidarVisualizer()
    try:
        viz.run()
    except KeyboardInterrupt:
        pass
    finally:
        viz.cleanup()
