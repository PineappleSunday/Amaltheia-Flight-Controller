
import numpy as np
from amaltheia_mission_components import WaypointAction, Waypoint, MissionLoader

class MissionManager:
    def __init__(self, mission_plan):
        self.mission = mission_plan
        self.current_index = 0
        self.hover_start_time = None
        self.prev_dist = float('inf')
        self.wp_start_time = 0.0 
        # Initialize previous position (start at 0,0,0,0)
        self.prev_wp_position = np.array([0.0, 0.0, 0.0, 0.0]) 
        
    def get_target(self, t, current_state):
        # 1. Check if mission is done
        if self.current_index >= len(self.mission):
            return self.mission[-1].position, np.zeros(3), 0.0, len(self.mission)-1

        # Get Current Waypoint
        wp = self.mission[self.current_index]
        
        # Calculate Duration of this leg
        # ToA is absolute time. wp_start_time is when we started THIS leg.
        # Ideally, leg duration is (wp.toa - previous_wp.toa), but simplified:
        leg_duration = wp.toa - self.wp_start_time
        
        # --- INTERPOLATION LOGIC ---
        if leg_duration > 0:
            # Calculate Percentage (0.0 to 1.0)
            pct = (t - self.wp_start_time) / leg_duration
            pct = np.clip(pct, 0.0, 1.0) 
            
            start_pos = self.prev_wp_position
            end_pos   = wp.position
            
            # Interpolate Position (includes Yaw!)
            target_pos = start_pos + (end_pos - start_pos) * pct
            
            # Feedforward Velocity (Only XYZ needed)
            if pct >= 1.0:
                target_velocity = np.zeros(3)
            else:
                target_velocity = (end_pos[:3] - start_pos[:3]) / leg_duration
        else:
            # Instant teleport if duration is 0 or negative
            target_pos = wp.position
            target_velocity = np.zeros(3)
        
        # Distance check (XYZ only)
        remaining_dist_to_wp = np.linalg.norm(wp.position[:3] - current_state[:3])
        
        # --- SUCCESS CHECK ---
        if remaining_dist_to_wp < wp.tolerance:
            self._handle_arrival(t, wp)
            
        # --- OVERSHOOT PROTECTION ---
        elif (remaining_dist_to_wp - self.prev_dist) > 0.1 and self.hover_start_time is None:
            if remaining_dist_to_wp < 5.0: # Only if reasonably close
                print(f"[t={t:.2f}] Overshoot detected. Advancing.")
                self.advance(t)

        self.prev_dist = remaining_dist_to_wp 
        
        # Return 4 values
        return target_pos, target_velocity, remaining_dist_to_wp, self.current_index

    def _handle_arrival(self, t, wp):
        if wp.action == WaypointAction.HOVER:
            if self.hover_start_time is None:
                self.hover_start_time = t
                print(f"[t={t:.2f}] Arrived at WP{self.current_index}. Holding...")
            
            if t - self.hover_start_time >= wp.duration:
                print(f"[t={t:.2f}] Hover complete. Advancing.")
                self.advance(t)
                
        elif wp.action == WaypointAction.LAND:
            if self.hover_start_time is None:
                print(f"[t={t:.2f}] Landing sequence initiated...")
                self.hover_start_time = t
            self.advance(t)
            
        else: # MOVE or TAKEOFF
            print(f"[t={t:.2f}] WP{self.current_index} reached. Advancing.")
            self.advance(t)
            
    def advance(self, current_time):
        # Store the position of the waypoint we just finished as the start for the next leg
        self.prev_wp_position = self.mission[self.current_index].position
        
        self.current_index += 1
        self.hover_start_time = None
        self.prev_dist = float('inf')
        self.wp_start_time = current_time
