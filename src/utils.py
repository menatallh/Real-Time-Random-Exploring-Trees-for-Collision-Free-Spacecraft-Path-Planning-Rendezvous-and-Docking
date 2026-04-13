import math
import numpy as np

class Obstacle:
    def __init__(self, x, y, width=0, height=0, angle=0.0):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.angle = angle

def normalize_angle(angle):
    """Keep angle in [-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def get_rotated_corners(cx, cy, w, h, angle):
    hw, hh = w / 2, h / 2
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    return [(cx + p[0]*cos_a - p[1]*sin_a, cy + p[0]*sin_a + p[1]*cos_a) for p in corners]
