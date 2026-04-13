import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt
from .utils import *
from .planner import *


# --- Visualization Class ---
class Visualizer:
    def __init__(self, world_map):
        self.min_randx, self.max_randx, self.min_randy, self.max_randy = world_map
        plt.figure(figsize=(12, 8))
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])

    def draw(self, planner, history_path, planned_path):
        plt.clf()
        for node in planner.node_list:
            if node.parent: plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color="cyan", alpha=0.5)



        if len(history_path) > 1:
            hx, hy ,_= zip(*history_path)
            plt.plot(hx, hy, '-g', linewidth=2.5, label="Actual Path Taken")
        x_cone_target,y_cone_target=RRTStarPlanner.get_rotated_target_cone(planner.target.x,planner.target.y,planner.target.width, planner.target.height,planner.target.angle)
        chaser_cone_end = RRTStarPlanner.get_chaser_docking_point(planner.chaser.x, planner.chaser.y,
                                                  planner.chaser.width, planner.chaser.height,
                                                  planner.chaser.docking_cone_length, planner.chaser.angle)
        target_cone_end = RRTStarPlanner.get_target_docking_point(planner.target.x, planner.target.y,30, 30,
                                                  planner.target.docking_cone_length, planner.target.angle)

        rendevous_point = RRTStarPlanner.get_rendevous_point(planner.target.x, planner.target.y,30, 30,
                                                  planner.target.docking_cone_length, planner.target.angle)
        rendevous_point2 = RRTStarPlanner.get_rendevous_point(planner.target.x, planner.target.y,30, 30,
                                                  planner.target.docking_cone_length,angle=np.deg2rad(90))
        self.draw_chaser_assembly(planner.chaser)
        self.draw_target_assembly(planner.target)
        self.draw_ellipse(planner.target, planner.target_ellipse_a, planner.target_ellipse_b)


        self.draw_osbtacle(planner.square_obstacles)

        chaser_cone_end = RRTStarPlanner.get_chaser_docking_point(planner.chaser.x, planner.chaser.y,
                                                  planner.chaser.width, planner.chaser.height,
                                                  planner.chaser.docking_cone_length, planner.chaser.angle)
        target_cone_end = RRTStarPlanner.get_target_docking_point(planner.target.x, planner.target.y,30, 30,
                                                  planner.target.docking_cone_length, planner.target.angle) # Target dimensions
        plt.scatter(*chaser_cone_end, color='blue', label='Chaser Docking Point')
        plt.scatter(*target_cone_end, color='red', label='Target Docking Point')
        plt.scatter(*rendevous_point, color='purple', label='Rendovous Point')
        plt.scatter(*rendevous_point2, color='green', label='Rendovous Point2')
        if planned_path:
            px, py,_ = zip(*planned_path)
            plt.plot(px, py, '-m', linewidth=2, label="Planned Path")

        title = f"Status: {planner.planner_status} | Target: {math.degrees(planner.target.angle):.1f}°"
        plt.title(title)
        plt.axis("equal")
        plt.axis([self.min_randx, self.max_randx, self.min_randy, self.max_randy])
        plt.grid(True)
        plt.legend(loc='upper left', fontsize='small')
        plt.pause(0.01)

    def draw_body(self, x, y, w, h, angle, color):
        corners = RRTStarPlanner._get_rotated_corners(x, y, w, h, angle)
        plt.fill([c[0] for c in corners], [c[1] for c in corners], color, alpha=0.7)


    def draw_osbtacle(self,square_obstacle_list):
        for obs in square_obstacle_list:
             self.draw_body(obs.x, obs.y, obs.width, obs.height, math.radians(obs.angle), 'gray')
    def draw_chaser_assembly(self, chaser_node):
        # Core body
        self.draw_body(chaser_node.x, chaser_node.y, 30, 30, chaser_node.angle, 'b')

        # Get cone base location
        x_cone, y_cone = RRTStarPlanner.get_rotated_chaser_cone(
            chaser_node.x, chaser_node.y,
            chaser_node.width, chaser_node.height,
            chaser_node.angle
        )

        # Male trapezoid docking cone (on chaser)
        self.draw_male_docking_cone(
            x_cone, y_cone,
            chaser_node.angle,
            base_width=12, top_width=6, length=5,
            color='brown', alpha=0.3
        )

    def draw_target_assembly(self, target_node):
        # Core body
        self.draw_body(target_node.x, target_node.y, 30, 30, target_node.angle, 'b')

        # Get cone base location
        x_cone_tip, y_cone_tip = RRTStarPlanner.get_target_docking_point(
            target_node.x, target_node.y,
            30, 30,
            target_node.docking_cone_length,
            target_node.angle
        )
        x_cone_base, y_cone_base = RRTStarPlanner.get_rotated_target_cone(target_node.x, target_node.y,
            30, 30,target_node.angle)

        # Docking sides highlight
        core_corners = RRTStarPlanner._get_rotated_corners(
            target_node.x, target_node.y, 30, 30, target_node.angle
        )
        plt.plot([core_corners[0][0], core_corners[3][0]],
                 [core_corners[0][1], core_corners[3][1]],
                 'lime', linewidth=3)
        plt.plot([core_corners[1][0], core_corners[2][0]],
                 [core_corners[1][1], core_corners[2][1]],
                 'lime', linewidth=3)

        # Female inverted trapezoid docking cone (on target)
        self.draw_female_docking_cone(
            x_cone_base, y_cone_base,
            target_node.angle + math.pi/2,
            square_size=6, base_width=12, cone_length=5.08,
            color='lime', alpha=0.3
        )




        # Solar panels
        panel_w, panel_h = 48, 4
        c, s = math.cos(target_node.angle), math.sin(target_node.angle)
        dx_panel_center, dy_panel_center = 0.5 * 30 * c, 0.5 * 30 * s
        offset_x, offset_y = 0.5 * panel_w * c, 0.5 * panel_w * s

        self.draw_body(
            (target_node.x + dx_panel_center) + offset_x,
            (target_node.y + dy_panel_center) + offset_y,
            panel_w, panel_h, target_node.angle, 'purple'
        )
        self.draw_body(
            (target_node.x - dx_panel_center) - offset_x,
            (target_node.y - dy_panel_center) - offset_y,
            panel_w, panel_h, target_node.angle, 'purple'
        )



    def draw_ellipse(self, node, semi_a, semi_b):
        angles = np.linspace(0, 2 * np.pi, 100)
        cos_rot, sin_rot = math.cos(node.angle), math.sin(node.angle)
        ex = node.x + semi_a * np.cos(angles) * cos_rot - semi_b * np.sin(angles) * sin_rot
        ey = node.y + semi_a * np.cos(angles) * sin_rot + semi_b * np.sin(angles) * cos_rot
        plt.plot(ex, ey, 'r--', label="Keep-out Zone")





    def draw_male_docking_cone(self, x, y, angle, base_width=12, top_width=6, length=6.7, color='brown', alpha=0.3):
        """
        Male docking cone: trapezoid with base (12 cm) at spacecraft body,
        tapering to top width (6 cm) at 12.7 cm forward.
        """
        half_base = base_width / 2
        half_top = top_width / 2

        dx, dy = math.cos(angle), math.sin(angle)
        px, py = -dy, dx  # perpendicular vector

        # Base (near chaser body)
        base_left = (x - half_base * px, y - half_base * py)
        base_right = (x + half_base * px, y + half_base * py)
        #print(x,y)
        #print(length , dx ,half_top , px,  dy , py)
        # Tip (farther away along docking direction)
        top_left = (x + length * dx - half_top * px, y + length * dy - half_top * py)
        top_right = (x + length * dx + half_top * px, y + length * dy + half_top * py)

        trap_x = [base_left[0], base_right[0], top_right[0], top_left[0]]
        trap_y = [base_left[1], base_right[1], top_right[1], top_left[1]]

        plt.fill(trap_x, trap_y, color=color, alpha=alpha)
        plt.plot(trap_x + [trap_x[0]], trap_y + [trap_y[0]], color=color, linestyle='--')

    def draw_female_docking_cone(self, x, y, angle, square_size=6.75, square_height=6.75,
                                 base_width=12, cone_length=5.08, color='lime', alpha=0.3):
        """
        Female docking structure:
        - A protruding square (6x6, height 6 cm)
        - An inverted trapezoid (6 -> 12 cm) extending 7.62 cm beyond the square
        Total length = square_height + cone_length
        """
        # Ensure x, y are scalars
        x = float(np.array(x).flatten()[0])
        y = float(np.array(y).flatten()[0])

        half_square = square_size / 2
        half_base = base_width / 2

        dx, dy = math.cos(angle), math.sin(angle)
        px, py = -dy, dx

        # --- Square block protruding outward ---
        sq_top_left = (x + square_height * dx - half_square * px,
                       y + square_height * dy - half_square * py)
        sq_top_right = (x + square_height * dx + half_square * px,
                        y + square_height * dy + half_square * py)
        sq_base_left = (x - half_square * px, y - half_square * py)
        sq_base_right = (x + half_square * px, y + half_square * py)

        sq_x = [sq_base_left[0], sq_base_right[0], sq_top_right[0], sq_top_left[0]]
        sq_y = [sq_base_left[1], sq_base_right[1], sq_top_right[1], sq_top_left[1]]
        plt.fill(sq_x, sq_y, color=color, alpha=alpha * 1.2)  # slightly stronger fill

        # --- Inverted trapezoid funnel ---
        top_left = sq_top_left
        top_right = sq_top_right
        base_left = (x + (square_height + cone_length) * dx - half_base * px,
                     y + (square_height + cone_length) * dy - half_base * py)
        base_right = (x + (square_height + cone_length) * dx + half_base * px,
                      y + (square_height + cone_length) * dy + half_base * py)

        trap_x = [top_left[0], top_right[0], base_right[0], base_left[0]]
        trap_y = [top_left[1], top_right[1], base_right[1], base_left[1]]
        plt.fill(trap_x, trap_y, color=color, alpha=alpha)


def compute_min_distance(chaser_pos, obstacle):
    dx = chaser_pos[0] - obstacle.x
    dy = chaser_pos[1] - obstacle.y
    center_dist = math.hypot(dx, dy)

    safety_dist = center_dist - (obstacle.width / 2)
    return safety_dist
