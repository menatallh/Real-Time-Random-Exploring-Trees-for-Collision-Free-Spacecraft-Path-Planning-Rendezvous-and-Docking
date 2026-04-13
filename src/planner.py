import random
import math
from .utils import  *
import time 
from .controller  import *


class Node:
    def __init__(self, x, y, width=30, height=30, angle=0.0, docking_cone_length=15):
        self.x, self.y = x, y
        self.width, self.height = width, height
        self.angle = angle
        self.parent = None
        self.cost = 0.0
        self.docking_cone_length = docking_cone_length
        self.docking_cone_angle = 30


MOVE_Distance=5
connect_circle_dist=30
t=time.time()
class Node:
    """
    Represents a state in the RRT* tree.
    """
    def __init__(self, x, y, width=0, height=0, angle=0.0,docking_cone_length=15):
        self.x = x
        self.y = y
        self.angle = angle  # Orientation in radians
        self.width = width
        self.height = height
        self.parent = None
        self.cost = 0.0
        self.docking_cone_length = docking_cone_length
        self.docking_cone_angle = 30  # Default value in degrees


# --- RRT* Planner Class ---
class RRTStarPlanner:
    def __init__(self, chaser_pos, target_pos, target_angle, world_map, square_obstacles=[]):
        self.node_class = Node
        self.chaser = self.node_class(chaser_pos[0], chaser_pos[1], 30, 30, chaser_pos[2],docking_cone_length=5)
        self.target = self.node_class(target_pos[0], target_pos[1], 126, 30, target_angle,docking_cone_length=12)
        self.square_obstacles = square_obstacles

        self.target_ellipse_a = self.target.width / 2
        self.target_ellipse_b = self.target.height / 2

        self.min_randx, self.max_randx, self.min_randy, self.max_randy = world_map
        self.move_step=[]
        # The tree is now initialized with the chaser's current position as the root
        self.node_list = [self.chaser]
        self.planner_status = "Initializing"
        self.docking_tolerance=0.3
        self.rewire_count = 0
        if self.node_list[0].parent:
            self.node_list[0].parent = None

    # ---------------- Planning Step ---------------- #
    def run_planning_step(self, expand_radius=MOVE_Distance ,connect_circle_dist=connect_circle_dist):
        start_t = time.perf_counter()

        rnd_node = self.get_random_node()
        nearest_node = self.get_nearest_node(self.node_list, rnd_node)
        new_node = self.steer(nearest_node, rnd_node,extend_length=expand_radius)  # <-- no expand_radius

        if self.is_path_valid_for_tree(nearest_node, new_node):
            near_inds = self.find_near_nodes(new_node, connect_circle_dist)
            new_node = self.choose_parent(new_node, near_inds)
            if new_node:
                self.node_list.append(new_node)
                self.rewire(new_node, near_inds, connect_circle_dist)
            # ---- TIME LOGGING ----
        elapsed = time.perf_counter() - start_t
        self.last_planning_time = elapsed

    # ---------------- Dynamic Step & Steering ---------------- #
    def dynamic_step_size(self, from_node, to_node, max_step=10.0, min_step=4.0):
        d, theta_desired = self.get_distance_and_angle(from_node, to_node)

        # Clearance check
        clearance = float("inf")
        for obs in self.square_obstacles:
            dist = self.point_to_rotated_rect_distance(
                from_node.x, from_node.y,
                obs.x, obs.y, obs.width, obs.height, obs.angle
            )
            clearance = min(clearance, dist)

        # Adaptive step scaling
        if clearance > 50:
            step = max_step
        elif clearance < 10:
            step = min_step
        else:
            step = min_step + (max_step - min_step) * (clearance - 10) / (50 - 10)

        return min(step, d), theta_desired

    def steer(self, from_node, to_node, extend_length=float("inf")):
        # Distance and desired heading
        d, theta_desired = self.get_distance_and_angle(from_node, to_node)
        extend_length = min(d, extend_length)
        new_node = self.node_class(from_node.x, from_node.y)
        new_node.x += extend_length * math.cos(theta_desired)
        new_node.y += extend_length * math.sin(theta_desired)
        new_node.parent = from_node
        #self.move_step.append(extend_length)
        return new_node

    # ---------------- Path Extraction ---------------- #
    def find_best_path(self):
        best_tree_node_idx, final_node, docking_possible = self.search_best_final_node()
        print(docking_possible)
        if docking_possible :
            final_node.parent = self.node_list[best_tree_node_idx]
            end_node=self.final_approach(final_node)
            end_node.parent = final_node
            planned_path=self.backtrack_from_node(end_node)
            print(planned_path[-1],"find best path")
            self.planner_status = "DOCKED"
            return planned_path
        elif best_tree_node_idx is not None:
            final_node.parent = self.node_list[best_tree_node_idx]

            return self.backtrack_from_node(final_node)
        else :
          self.planner_status = "No Path to Target"
          return None



    ######################## smoothing helpers ######################################

    def _uniform_clamped_knots(self, n_ctrl, degree):
        """
        Returns a clamped uniform knot vector for n_ctrl control points and given degree.
        Length = n_ctrl + degree + 1
        """
        m = n_ctrl + degree + 1
        knots = [0.0] * (degree + 1)
        inner = m - 2*(degree + 1)
        if inner > 0:
            step = 1.0 / (inner + 1)
            knots += [i * step for i in range(1, inner + 1)]
        knots += [1.0] * (degree + 1)
        return knots

    def _cox_de_boor(self, i, k, u, knots, degree):
        """Cox–de Boor recursion for B-spline basis N_{i,k}(u)."""
        # Base case: check if indices are valid first
        if i + k + 1 >= len(knots):
            return 0.0

        if k == 0:
            # Check if u is in the interval [knots[i], knots[i+1])
            if knots[i] <= u < knots[i+1]:
                return 1.0
            # Special case for the very end of the knot vector
            if u == knots[-1] and i == len(knots) - degree - 2 and knots[i] <= u <= knots[i+1]:
                return 1.0
            return 0.0

        # Check if indices are valid before accessing
        if i + k >= len(knots) or i + k + 1 >= len(knots):
            return 0.0

        denom1 = knots[i+k] - knots[i]
        denom2 = knots[i+k+1] - knots[i+1]

        term1 = 0.0
        term2 = 0.0

        # Only calculate term1 if denominator is not zero and indices are valid
        if denom1 != 0 and i < len(knots) - k:
            term1 = ((u - knots[i]) / denom1) * self._cox_de_boor(i, k-1, u, knots, degree)

        # Only calculate term2 if denominator is not zero and indices are valid
        if denom2 != 0 and i + 1 < len(knots) - k:
            term2 = ((knots[i+k+1] - u) / denom2) * self._cox_de_boor(i+1, k-1, u, knots, degree)

        return term1 + term2

    def _bspline_curve(self, control_pts, degree=3, samples=100):
        """
        Evaluate clamped uniform B-spline for control_pts: [(x,y,angle_optional_or_None), ...]
        Returns list of (x, y) points.
        """
        n_ctrl = len(control_pts)
        if n_ctrl < degree + 1:
            # not enough points; return original
            return [(p[0], p[1]) for p in control_pts]

        knots = self._uniform_clamped_knots(n_ctrl, degree)
        curve = []
        for s in range(samples + 1):
            u = s / samples
            # Ensure u is within valid range [0, 1] with slight tolerance for floating point
            u = max(0.0, min(u, 1.0))
            x = 0.0
            y = 0.0
            for i in range(n_ctrl):
                Ni = self._cox_de_boor(i, degree, u, knots, degree)
                x += Ni * control_pts[i][0]
                y += Ni * control_pts[i][1]
            curve.append((x, y))
        return curve

    def _compute_headings_from_polyline(self, pts):
        """Return heading angles from tangent of consecutive points."""
        import math
        headings = []
        for i in range(len(pts)):
            if i == len(pts) - 1:
                # Use backward difference for the last point
                dx = pts[i][0] - pts[i-1][0]
                dy = pts[i][1] - pts[i-1][1]
            else:
                # Use forward difference for all other points
                dx = pts[i+1][0] - pts[i][0]
                dy = pts[i+1][1] - pts[i][1]
            # Calculate heading angle (in radians)
            heading = math.atan2(dy, dx) if (dx != 0 or dy != 0) else 0.0
            headings.append(heading)
        return headings

    def bspline_smooth_path(self, path_xytheta, degree=3, samples_per_segment=10):
        """
        Smooth a discrete RRT path with a clamped cubic B-spline.
        Input path: [(x,y,theta), ...] (theta optional; ignored for smoothing)
        Output: [(x,y,theta_smooth), ...] dense & smooth.
        """
        print(len(path_xytheta))
        if not path_xytheta or len(path_xytheta) < 2:
            print('path is too short')
            return path_xytheta[:]  # nothing to smooth

        if len(path_xytheta) == 2:
           (x1, y1, *rest1) = path_xytheta[0]
           (x2, y2, *rest2) = path_xytheta[1]
           xm, ym = (x1 + x2) / 2.0, (y1 + y2) / 2.0

           # build 3-point path: start → midpoint → end
           midpoint = (xm, ym, 0.0)
           new_path = [path_xytheta[0], midpoint, path_xytheta[1]]
           return new_path


        controls = [(p[0], p[1]) for p in path_xytheta]
        samples = max(samples_per_segment * (len(controls) - 1), 50)  # ensure reasonable density
        curve_xy = self._bspline_curve(controls, degree=degree, samples=samples)
        curve_head = self._compute_headings_from_polyline(curve_xy)

        # Pack as (x, y, theta) using computed headings
        smoothed = [(curve_xy[i][0], curve_xy[i][1], curve_head[i]) for i in range(len(curve_xy))]
        return smoothed






    def point_to_segment_distance(self, px, py, x1, y1, x2, y2):
        """Shortest distance from point (px, py) to segment (x1,y1)-(x2,y2)."""
        dx, dy = x2 - x1, y2 - y1
        if dx == dy == 0:
            return math.hypot(px - x1, py - y1)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        proj_x, proj_y = x1 + t * dx, y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    def point_to_rotated_rect_distance(self, px, py, ox, oy, w, h, angle):
        corners = self._get_rotated_corners(ox, oy, w, h, angle)
        min_dist = float("inf")
        for i in range(len(corners)):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % len(corners)]
            d = self.point_to_segment_distance(px, py, x1, y1, x2, y2)
            min_dist = min(min_dist, d)
        return min_dist



    @staticmethod
    def get_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    @staticmethod
    def _get_rotated_corners(cx, cy, w, h, angle):
        hw, hh = w / 2, h / 2
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        return [(cx + p[0]*cos_a - p[1]*sin_a, cy + p[0]*sin_a + p[1]*cos_a) for p in corners]
    @staticmethod
    def get_rotated_target_cone( cx, cy, w,h,angle):
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        p = -h/4 ,h/2
        return (cx + p[0]*cos_a - p[1]*sin_a, cy + p[0]*sin_a + p[1]*cos_a)
    @staticmethod
    def get_rotated_chaser_cone( cx, cy, w,h,angle):
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        p = w/2 ,h/4
        return (cx + p[0]*cos_a - p[1]*sin_a, cy + p[0]*sin_a + p[1]*cos_a)

    @staticmethod
    def get_target_docking_point( cx, cy, w,h,cone_lenght,angle):

        cos_a, sin_a = math.cos(angle), math.sin(angle)
        p = -h/4 ,h/2+cone_lenght
        return (cx + p[0]*cos_a - p[1]*sin_a, cy + p[0]*sin_a + p[1]*cos_a)

    @staticmethod
    def get_rendevous_point( cx, cy, w,h,cone_lenght,angle):
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        threshold=40
        p = -h/4 ,h/2+cone_lenght+threshold
        return (cx + p[0]*cos_a - p[1]*sin_a, cy + p[0]*sin_a + p[1]*cos_a)
    @staticmethod
    def get_chaser_docking_point( cx, cy, w,h,cone_lenght,angle):


        cos_a, sin_a = math.cos(angle), math.sin(angle)
        p = w/2+cone_lenght ,h/4
        return (cx + p[0]*cos_a - p[1]*sin_a, cy + p[0]*sin_a + p[1]*cos_a)
    @staticmethod
    def get_inv_chaser_docking_point( px, py, w,h,cone_lenght,angle):
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        p = w/2+cone_lenght ,h/4
        return (px - p[0]*cos_a + p[1]*sin_a, py - p[0]*sin_a - p[1]*cos_a)
    @staticmethod
    def _check_rect_collision(rect1_node, rect2_node):
        rects = [rect1_node, rect2_node]
        for i in range(2):
            corners = RRTStarPlanner._get_rotated_corners(rects[i].x, rects[i].y, rects[i].width, rects[i].height, rects[i].angle)
            axes = [(corners[1][0] - corners[0][0], corners[1][1] - corners[0][1]),
                    (corners[3][0] - corners[0][0], corners[3][1] - corners[0][1])]
            for ax_x, ax_y in axes:
                mag = math.hypot(ax_x, ax_y)
                if mag == 0: continue
                axis = (ax_x / mag, ax_y / mag)
                projections = []
                for j in range(2):
                    other_corners = RRTStarPlanner._get_rotated_corners(rects[j].x, rects[j].y, rects[j].width, rects[j].height, rects[j].angle)
                    min_p, max_p = other_corners[0][0] * axis[0] + other_corners[0][1] * axis[1], other_corners[0][0] * axis[0] + other_corners[0][1] * axis[1]
                    for k in range(1, 4):
                        p = other_corners[k][0] * axis[0] + other_corners[k][1] * axis[1]
                        min_p, max_p = min(min_p, p), max(max_p, p)
                    projections.append((min_p, max_p))
                if projections[0][1] < projections[1][0] or projections[1][1] < projections[0][0]:
                    return False
        return True

    def _is_path_clear_of_static_obstacles(self, from_node, to_node):
        dist, _ = self.get_distance_and_angle(from_node, to_node)
        n_steps = int(dist / (self.chaser.width / 4)) + 1
        for i in range(n_steps + 1):
            ratio = i / n_steps
            px, py = from_node.x * (1 - ratio) + to_node.x * ratio, from_node.y * (1 - ratio) + to_node.y * ratio
            robot_at_p = self.node_class(px, py, self.chaser.width, self.chaser.height, 0)
            for obs in self.square_obstacles:
                obs_node = self.node_class(obs.x, obs.y, obs.width, obs.height, math.radians(obs.angle))
                if self._check_rect_collision(robot_at_p, obs_node):
                    return False

        return True

 # ---------------- GEOMETRY HELPERS ---------------- #
    @staticmethod
    def dist_point_to_segment(px, py, x1, y1, x2, y2):
        """Shortest distance from point (px, py) to line segment (x1,y1)-(x2,y2)."""
        dx, dy = x2 - x1, y2 - y1
        if dx == dy == 0:  # degenerate case
            return math.hypot(px - x1, py - y1)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        proj_x, proj_y = x1 + t * dx, y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    @staticmethod
    def segments_intersect(p1, p2, q1, q2):
        """Check if two line segments (p1-p2 and q1-q2) intersect."""
        def ccw(a, b, c):
            return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
        return (ccw(p1, q1, q2) != ccw(p2, q1, q2)) and (ccw(p1, p2, q1) != ccw(p1, p2, q2))

    # ---------------- COLLISION CHECK ---------------- #
    @staticmethod
    def check_collision(from_node, to_node,obstacles, safe_margin=5):
        chaser_base=RRTStarPlanner.get_rotated_chaser_cone(from_node.x,from_node.y,from_node.width,from_node.height,from_node.angle)
        chaser_tip=RRTStarPlanner.get_chaser_docking_point(from_node.x,from_node.y,from_node.width,from_node.height,from_node.docking_cone_length,from_node.angle)
        # 1) Check intersection between cones (segments)
        target_base=RRTStarPlanner.get_rotated_target_cone(to_node.x,to_node.y,to_node.width,to_node.height,to_node.angle)
        target_tip=RRTStarPlanner.get_target_docking_point(to_node.x,to_node.y,to_node.width,to_node.height,to_node.docking_cone_length,to_node.angle)
        #if RRTStarPlanner.segments_intersect(chaser_base, chaser_tip, target_base, target_tip):
        #    return True

        # 2) Check distance between cones (safe margin)
        dist_cones = RRTStarPlanner.dist_point_to_segment(
            chaser_tip[0], chaser_tip[1],
            target_base[0], target_base[1],
            target_tip[0], target_tip[1]
        )
        if dist_cones < safe_margin:
            return True

        # 3) Check collisions with obstacles
        for o in obstacles:
            dist = RRTStarPlanner.dist_point_to_segment(o.x, o.y,
                                                      chaser_base[0], chaser_base[1],
                                                      chaser_tip[0], chaser_tip[1])
            if dist < 15 + safe_margin:
                return True

        return False
    @staticmethod
    def normalize_angle(angle):
        """
        Wraps the angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle




    def is_path_valid_for_tree(self, from_node, to_node):
        if not self._is_path_clear_of_static_obstacles(from_node, to_node):
           return False

        # Check docking cone alignment along the path
        dist, _ = self.get_distance_and_angle(from_node, to_node)
        n_steps = int(dist / (self.chaser.width / 4)) + 1

        for i in range(n_steps + 1):
            ratio = i / n_steps
            px = from_node.x * (1 - ratio) + to_node.x * ratio
            py = from_node.y * (1 - ratio) + to_node.y * ratio

            # Create temporary node for alignment check
            temp_node = self.node_class(px, py)
            temp_node.angle = from_node.angle * (1 - ratio) + to_node.angle * ratio

            #if self._get_facing_side_type(temp_node) != 'DOCKING':
            #   return False
            if self._inside_keep_out_zone(px, py):
                return False

            if self.check_collision(temp_node,to_node,self.square_obstacles):
                 return False


            return True

    def _inside_keep_out_zone(self, x, y):
        # Parameters for the keep-out zone
        semi_a = self.target_ellipse_a   # ellipse semi-major axis
        semi_b = self.target_ellipse_b   # ellipse semi-minor axis

        # Translate to target center
        dx = x - self.target.x
        dy = y - self.target.y

        # Rotate into target frame
        cos_rot = math.cos(-self.target.angle)
        sin_rot = math.sin(-self.target.angle)
        xr = dx * cos_rot - dy * sin_rot
        yr = dx * sin_rot + dy * cos_rot

        # Ellipse equation: (x/a)^2 + (y/b)^2 <= 1 → inside ellipse
        val = (xr / semi_a) ** 2 + (yr / semi_b) ** 2
        return val <= 1.0


    def get_random_node(self):
       if random.random() > 0.75:
           desired_theta = 0 + math.pi
           tg_dp = self.get_target_docking_point(self.target.x, self.target.y,
                                          30, 30, self.target.docking_cone_length,desired_theta)

           tg_db=self.get_rendevous_point(self.target.x, self.target.y,
                                          30, 30, self.target.docking_cone_length,desired_theta)

           desired_cx,desired_cy=self.get_inv_chaser_docking_point( tg_db[0], tg_db[1], self.chaser.width,self.chaser.height,self.chaser.docking_cone_length,self.chaser.angle)





           return self.node_class(desired_cx, desired_cy, angle=0)  # Face target
       else:
        return self.node_class(
            random.uniform(self.min_randx, self.max_randx),
            random.uniform(self.min_randy, self.max_randy),
            angle=0
        )


    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            new_node.cost = new_node.parent.cost + self.get_distance_and_angle(new_node.parent, new_node)[0]
            return new_node
        min_cost, best_parent = float('inf'), None
        for i in near_inds:
            near_node = self.node_list[i]
            dist, _ = self.get_distance_and_angle(near_node, new_node)
            if near_node.cost + dist < min_cost and self.is_path_valid_for_tree(near_node, new_node):
                min_cost, best_parent = near_node.cost + dist, near_node
        if best_parent:
            new_node.parent, new_node.cost = best_parent, min_cost
            return new_node
        return None

    def rewire(self, new_node, near_inds, connect_circle_dist):
        self.rewire_count = 0

        for i in near_inds:
            near_node = self.node_list[i]
            dist, _ = self.get_distance_and_angle(new_node, near_node)
            if new_node.cost + dist < near_node.cost and self.is_path_valid_for_tree(new_node, near_node):
                near_node.parent = new_node
                near_node.cost = new_node.cost + dist
                self.rewire_count += 1

    def _get_facing_side_type(self, from_node):
       # Get docking cone endpoints for both spacecraft
       chaser_cone_end = self.get_chaser_docking_point(from_node.x, from_node.y,
                                           self.chaser.width, self.chaser.height,
                                           self.chaser.docking_cone_length, from_node.angle)
       target_cone_end = self.get_target_docking_point(self.target.x, self.target.y,
                                          30, 30,  # Target dimensions
                                          self.target.docking_cone_length, self.target.angle)

       # Vector from chaser cone to target cone
       path_vec = np.array([target_cone_end[0] - chaser_cone_end[0],
                        target_cone_end[1] - chaser_cone_end[1]])



       path_vec_norm = path_vec / np.linalg.norm(path_vec)

       # Calculate alignment between cones
       chaser_dock_dir = np.array([math.cos(from_node.angle), math.sin(from_node.angle)])
       target_dock_dir = np.array([math.cos(self.target.angle + math.pi),  # Opposite direction
                              math.sin(self.target.angle + math.pi)])

       chaser_dot = np.dot(chaser_dock_dir, path_vec_norm)
       target_dot = np.dot(target_dock_dir, -path_vec_norm)  # Facing each other

       # Check if both cones are properly aligned
       if (chaser_dot > math.cos(self.chaser.docking_cone_angle) and
        target_dot > math.cos(self.target.docking_cone_angle)):
        return 'DOCKING'
       return 'NONE'

    def search_best_final_node(self):
        best_cost = float('inf')
        best_tree_node_idx = None
        best_final_node = None
        docking_possible = False  # Flag to indicate successful docking


        desired_theta = np.deg2rad(90)

        # Target docking point (world coords)
        tg_dp = self.get_target_docking_point(self.target.x, self.target.y,
                                          30, 30, self.target.docking_cone_length,desired_theta)





        print(abs(np.rad2deg(self.target.angle) - np.rad2deg(desired_theta)))
        if abs(np.rad2deg(self.target.angle) - np.rad2deg(desired_theta)) < 3:
                print('target in place')
                docking_possible = True  # Set the flag to True
        for i, node in enumerate(self.node_list):



            angle_diff = node.angle - self.target.angle


            # Fall back to normal approach if not perfectly aligned yet
            facing_type = self._get_facing_side_type(node)

            is_docking_intent = (facing_type == 'DOCKING')
            self.planner_status = "Docking Approach" if is_docking_intent else "Waiting"

            target_point,at_target_point = self.calculate_target_point(node, is_docking_intent)
            if target_point and self._is_path_clear_of_static_obstacles(node, target_point):
                dist, _ = self.get_distance_and_angle(node, target_point)
                total_cost = node.cost + dist
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_tree_node_idx = i
                    best_final_node = target_point
        #print(best_tree_node_idx,best_final_node)
        return best_tree_node_idx, best_final_node, docking_possible  # Always return docked flag

    def final_approach(self,best_final_node,at_target_point=True):
                desired_theta = np.deg2rad(90)

              # Target docking point (world coords)
                tg_dp = self.get_target_docking_point(self.target.x, self.target.y,
                                          30, 30, self.target.docking_cone_length,desired_theta)


                print(abs(np.rad2deg(self.target.angle) - np.rad2deg(desired_theta)))
              #if abs(np.rad2deg(self.target.angle) - np.rad2deg(desired_theta)) < 3:
                print('target in place')
                    # --- Align with PID refinement ---
                final_coordiantes,trajectory,final_node= run_to_target(
                best_final_node, tg_dp,
                tau_p=0.2, tau_d=0.01, tau_i=0.001,

                n=300,
                tolerance=self.docking_tolerance,

                get_chaser_dp=self.get_chaser_docking_point,
                get_inv_chaser_dp=self.get_inv_chaser_docking_point
            )
                x_final_node,y_final_node=final_coordiantes
                #x_traj, y_traj=trajectory
                final_node=self.node_class(x_final_node,y_final_node,self.chaser.width,self.chaser.height,self.chaser.angle)
                final_node.parent=best_final_node
                # Check final alignment
                ch_dp = self.get_chaser_docking_point(
                final_node.x, final_node.y,
                final_node.width, final_node.height,
                final_node.docking_cone_length, final_node.angle
                )
                print(tg_dp,ch_dp,'-----------------------------------------------------------',abs(normalize_angle(final_node.angle - desired_theta)))
                position_aligned = math.hypot(ch_dp[0] - tg_dp[0], ch_dp[1] - tg_dp[1]) < self.docking_tolerance
                orientation_aligned = abs(normalize_angle(final_node.angle - desired_theta)) < math.radians(5)

                if position_aligned :
                  print('Docked')
                  print("total time ",time.time()-t)
                  self.planner_status = "DOCKED"
                return  final_node  # return docked immediately


    def calculate_target_point(self, from_node, is_docking_intent):

       # Desired chaser angle: face *into* the target docking direction
       desired_theta = np.deg2rad(90)

       # Target docking point (world coords)
       tg_dp = self.get_target_docking_point(self.target.x, self.target.y,
                                          30, 30, self.target.docking_cone_length,desired_theta)
       tg_db=self.get_rendevous_point(self.target.x, self.target.y,
                                          30, 30, self.target.docking_cone_length,desired_theta)

       desired_cx,desired_cy=self.get_inv_chaser_docking_point( tg_db[0], tg_db[1], self.chaser.width,self.chaser.height,self.chaser.docking_cone_length,self.chaser.angle)


       # Vector from current node center to desired center
       dx = desired_cx - from_node.x
       dy = desired_cy - from_node.y
       dist = math.hypot(dx, dy)
       at_target_point = False
       if dist < 1e-6:
           print('Reached the target')
           at_target_point=True
           # Already at desired center; ensure angle matches
           docking_point = self.get_chaser_docking_point(from_node.x, from_node.y,
                                                      self.chaser.width, self.chaser.height,
                                                      self.chaser.docking_cone_length, 0)
           n = self.node_class(from_node.x, from_node.y, self.chaser.width, self.chaser.height, 0)
           n.parent = from_node
           return n,at_target_point


       step = max(self.chaser.width * 0.8, 1.0)
       r = min(1.0, step / dist)

       nx = from_node.x + r * dx
       ny = from_node.y + r * dy

       ang_diff = (desired_theta - from_node.angle + math.pi) % (2 * math.pi) - math.pi
       ang_step = math.radians(10)  # limit heading change per step (tunable)
       if abs(ang_diff) < ang_step:
           ntheta = desired_theta
       else:
           ntheta = (from_node.angle + math.copysign(ang_step, ang_diff)) % (2 * math.pi)

       docking_point = self.get_chaser_docking_point(nx, ny,
                                                  self.chaser.width, self.chaser.height,
                                                  self.chaser.docking_cone_length, 0)
       n = self.node_class(nx, ny, self.chaser.width, self.chaser.height)
       n.parent = from_node


       return n,at_target_point

    def find_near_nodes(self, new_node, connect_circle_dist):
        return [i for i, n in enumerate(self.node_list) if (n.x-new_node.x)**2 + (n.y-new_node.y)**2 <= connect_circle_dist**2]
    def get_nearest_node(self, nodes, rnd_node):
        return min(nodes, key=lambda n: (n.x - rnd_node.x)**2 + (n.y - rnd_node.y)**2)
    def backtrack_from_node(self, node):
        path = []
        while node: path.append([node.x, node.y,node.angle]); node = node.parent
        return path[::-1]
