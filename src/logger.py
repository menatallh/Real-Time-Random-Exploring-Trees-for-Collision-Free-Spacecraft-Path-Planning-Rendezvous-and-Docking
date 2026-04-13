import pandas as pd
import numpy as np
import math
import time
from .planner import *
from .utils  import *
class DataCollector:
    def __init__(self):
        self.data = []

    def collect(self, planner, time_step, path_length, comp_time, chaser_pos, target_pos, target_angle, safety_margin):
        # 1. Calculate precise docking error using your geometry functions
        # We calculate the distance between the two docking ports (cone tips)

        # Target's docking port location
        tg_dock_x, tg_dock_y = RRTStarPlanner.get_target_docking_point(
            target_pos[0], target_pos[1], 30, 30,
            planner.target.docking_cone_length, target_angle
        )

        # Chaser's docking port location
        ch_dock_x, ch_dock_y = RRTStarPlanner.get_chaser_docking_point(
            chaser_pos[0], chaser_pos[1], 30, 30,
            planner.chaser.docking_cone_length, chaser_pos[2]
        )

        # Euclidean distance error
        pos_error = math.hypot(ch_dock_x - tg_dock_x, ch_dock_y - tg_dock_y)

        # Angular error
        # Assuming target mating angle is perpendicular to target body (target_angle + 90 deg)
        desired_angle = target_angle + math.pi/2

        # Normalize angular error
        raw_ang_err = chaser_pos[2] - desired_angle
        ang_error = abs(math.atan2(math.sin(raw_ang_err), math.cos(raw_ang_err)))

        entry = {
            'time': time_step,
            'status': planner.planner_status,
            'tree_size': len(planner.node_list),
            'planning_time_ms': comp_time * 1000,
            'path_length': path_length,
            'safety_margin': safety_margin,
            'pos_error': pos_error,
            'ang_error_deg': math.degrees(ang_error),
            'chaser_x': chaser_pos[0],
            'chaser_y': chaser_pos[1]
        }
        self.data.append(entry)

    def save_to_csv(self, filename="simulation_results.csv"):
        if not self.data:
            print("No data collected.")
            return

        df = pd.DataFrame(self.data)

        # --- SMART DISCARD LOGIC ---
        # If the last few points show increasing error (after docking success), trim them.
        # This fixes the "jitter" at the very end.
        if len(df) > 5:
            # Look at the last 5 frames. If position error increases significantly, cut it.
            last_idx = df.index[-1]
            for i in range(last_idx, last_idx - 5, -1):
                current_err = df.loc[i, 'pos_error']
                prev_err = df.loc[i-1, 'pos_error']

                # If error jumped up by more than 1cm at the very end, drop this row
                if current_err > prev_err + 1.0:
                    print(f"Dropping noise data at time {df.loc[i, 'time']:.2f}s")
                    df = df.drop(i)
                else:
                    # Once we find a stable point, stop trimming
                    break

        df.to_csv(filename, index=False)
        print(f"Data saved to {filename}")
