
from src.planner import RRTStarPlanner
from src.logger import DataCollector
import matplotlib.pyplot as plt
from src.controller import  *
from src.visualizer import *
from src.utils import *

MOVE_Distance=5
connect_circle_dist=30

def main():
    # --- 1. SETUP DATA COLLECTOR ---
    data_collector = DataCollector()

    docked = False
    BOARD_WIDTH_CM, BOARD_HEIGHT_CM = 12 * 30.48, 8 * 30.48
    world_map = [0, BOARD_WIDTH_CM, 0, BOARD_HEIGHT_CM]
    square_obstacle_list = [Obstacle(world_map[1] * 0.5, world_map[3] * 0.5, 30, 30, 10)]
    target_pos = [world_map[1] - 50.0, world_map[3] / 2]

    chaser_pos = [30.0, world_map[3] / 2, 0]
    chaser_history = [list(chaser_pos)] # Fixed: Ensure it's a list copy
    move_distance = MOVE_Distance
    rrt_steps_per_frame = 30
    total_sim_time = 600.0

    start_time = time.time()
    target_angle_rad_first = math.radians((2.0 * 0) % 360)
    square_obstacle_list[-1].angle = (2.0 * 0) % 360

    planner = RRTStarPlanner(
        chaser_pos=chaser_pos,
        target_pos=target_pos,
        target_angle=target_angle_rad_first,
        world_map=world_map,
        square_obstacles=square_obstacle_list
    )

    visualizer = Visualizer(world_map)

    # --- MAIN LOOP ---
    while time.time() - start_time < total_sim_time and not docked:

        cycle_start_time = time.time()
        elapsed_time = time.time() - start_time

        # 1. Plan
        for _ in range(rrt_steps_per_frame):
            planner.run_planning_step()

        planned_path = planner.find_best_path()
        cycle_planning_time = time.time() - cycle_start_time

        # 2. Movement Logic
        if planner.planner_status == "DOCKED":
            docked = True
            if planned_path:
                final_point = planned_path[-1]
                chaser_pos[0] = final_point[0]
                chaser_pos[1] = final_point[1]
            print('Docked')
        else:
            if planned_path and len(planned_path) > 1:
                avg_spacing = np.mean([
                    math.hypot(planned_path[i + 1][0] - planned_path[i][0],
                               planned_path[i + 1][1] - planned_path[i][1])
                    for i in range(len(planned_path) - 1)
                ])
                samples_per_segment = max(4, int(12 * (avg_spacing / move_distance)))

                smoothed = planner.bspline_smooth_path(
                    planned_path, degree=3, samples_per_segment=samples_per_segment
                )

                if smoothed and len(smoothed) > 1:
                    next_waypoint = smoothed[min(1, len(smoothed) - 1)]
                    dx = next_waypoint[0] - chaser_pos[0]
                    dy = next_waypoint[1] - chaser_pos[1]
                    dist = math.hypot(dx, dy)

                    move_step = min(move_distance, dist)
                    if dist > 0:
                        chaser_pos[0] += move_step * (dx / dist)
                        chaser_pos[1] += move_step * (dy / dist)

        # 3. Dynamic Updates
        target_angle_rad = math.radians((2.0 * elapsed_time) % 360)
        square_obstacle_list[-1].angle = (2.0 * elapsed_time) % 360
        dist_to_obstacle = compute_min_distance(chaser_pos, square_obstacle_list[-1])

        # --- 4. COLLECT DATA HERE ---
        data_collector.collect(
            planner=planner,
            time_step=elapsed_time,
            path_length=len(planned_path) if planned_path else 0,
            comp_time=cycle_planning_time,
            chaser_pos=list(chaser_pos), # Pass copy
            target_pos=target_pos,
            target_angle=target_angle_rad,
            safety_margin=dist_to_obstacle
        )

        # 5. Re-init Planner & Draw
        planner = RRTStarPlanner(
            chaser_pos=chaser_pos,
            target_pos=target_pos,
            target_angle=target_angle_rad,
            world_map=world_map,
            square_obstacles=square_obstacle_list
        )

        chaser_history.append(list(chaser_pos))
        visualizer.draw(planner, chaser_history, planned_path)

        print(f"Time: {elapsed_time:.1f}s | Status: {planner.planner_status} | "
              f"Tree: {len(planner.node_list)} | "
              f"Plan: {cycle_planning_time*1000:.1f}ms", end='\r')

        if docked:
            break

    # --- 6. SAVE DATA ON EXIT ---
    plt.ioff()
    plt.close()
    data_collector.save_to_csv()
    print("\nSimulation Finished. Results saved to 'simulation_results.csv'.")

if __name__ == "__main__":
    main()
