import numpy as np
from dataclasses import dataclass
from enum import Enum, auto
import json
import os

class WaypointAction(Enum):
    TAKEOFF = auto()
    HOVER = auto()
    LAND = auto()
    MOVE = auto()
      
@dataclass
class Waypoint:
    position: np.ndarray  # [x, y, z, yaw]
    action: WaypointAction = WaypointAction.HOVER
    tolerance: float = 2.0 #meters tolerance to waypoint 
    duration: float = 0.0
    toa: float = 0.0 # Time of Arrival (Absolute Simulation Time)
    
    def __str__(self):
        pos_str = f"[{self.position[0]:.1f}, {self.position[1]:.1f}, {self.position[2]:.1f}, {self.position[3]:.2f}]"
        return f"{self.action.name:<8} | Pos: {pos_str} | Tol: {self.tolerance}m | ToA: {self.toa}s"

class MissionLoader:
    @staticmethod
    def save_to_json(filename, mission_plan):
        data = []
        for wp in mission_plan:
            item = {
                "position": wp.position.tolist(),
                "action": wp.action.name,
                "tolerance": wp.tolerance,
                "duration": wp.duration,
                "toa": wp.toa
            }
            data.append(item)
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=4)
            print(f"Mission saved to {filename}")
        except Exception as e:
            print(f"Error saving mission: {e}")

    @staticmethod
    def load_from_json(filename):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(script_dir, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Mission file {filename} not found at {filepath}.")
            
        with open(filepath, 'r') as f:
            data = json.load(f)
            
        mission_plan = []
        for item in data:
            wp = Waypoint(
                position=np.array(item["position"]),
                action=WaypointAction[item["action"]],
                tolerance=item["tolerance"],
                duration=item["duration"],
                toa=item["toa"]
            )
            mission_plan.append(wp)
            
        print(f"Mission loaded from {filename} ({len(mission_plan)} waypoints)")
        return mission_plan
