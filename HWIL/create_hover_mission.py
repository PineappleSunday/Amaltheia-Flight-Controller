from amaltheia_mission_components import Waypoint, WaypointAction, MissionLoader
import numpy as np


def create_hover_mission(filename="PIL_Hover_1.json"):
    """
    Creates a simple mission: Takeoff to 1m, then hold position forever.
    """
    plan = []
    
    # 1. Takeoff to 1.0 meter
    # ToA = 5.0s (Give it 5 seconds to spool up and get there)
    plan.append(Waypoint(
        position=np.array([0.0, 0.0, 1.0, 0.0]), 
        action=WaypointAction.TAKEOFF, 
        tolerance=0.2, 
        toa=5.0
    ))
    
    # 2. Indefinite Hover
    # We set a massive duration so the Mission Manager never "advances" past this.
    # Even if it did advance, the logic holds the last position, so it's safe either way.
    plan.append(Waypoint(
        position=np.array([0.0, 0.0, 1.0, 0.0]), 
        action=WaypointAction.HOVER, 
        tolerance=0.2, 
        duration=9999.0, 
        toa=10.0
    ))
    
    MissionLoader.save_to_json(filename, plan)
    print(f"Hover Mission generated: {filename}")

if __name__ == "__main__":
    create_hover_mission()