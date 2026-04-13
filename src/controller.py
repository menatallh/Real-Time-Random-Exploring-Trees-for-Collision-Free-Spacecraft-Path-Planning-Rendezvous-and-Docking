import math
from .utils import normalize_angle
from .utils import *
from .planner import *
def run_to_target(node, target_dp, tau_p=0.2, tau_d=0.01, tau_i=0.001, 
                  n=300, tolerance=0.5, get_chaser_dp=None, get_inv_chaser_dp=None):
    """PID controller to align chaser docking point with target."""
    x_traj, y_traj = [], []
    prev_err_x = prev_err_y = sum_err_x = sum_err_y = 0.0

    for i in range(n):
        ch_dp = get_chaser_dp(node.x, node.y, node.width, node.height, node.docking_cone_length, node.angle)
        err_x, err_y = target_dp[0] - ch_dp[0], target_dp[1] - ch_dp[1]
        
        if math.hypot(err_x, err_y) < tolerance:
            break

        # PID Logic
        control_x = tau_p * err_x + tau_d * (err_x - prev_err_x) + tau_i * sum_err_x
        control_y = tau_p * err_y + tau_d * (err_y - prev_err_y) + tau_i * sum_err_y
        
        # Update State
        new_x, new_y = get_inv_chaser_dp(ch_dp[0] + control_x, ch_dp[1] + control_y, 
                                        node.width, node.height, node.docking_cone_length, node.angle)
        node.x, node.y = new_x, new_y
        
        prev_err_x, prev_err_y = err_x, err_y
        sum_err_x += err_x; sum_err_y += err_y
        x_traj.append(node.x); y_traj.append(node.y)

    return (node.x, node.y), (x_traj, y_traj), node
