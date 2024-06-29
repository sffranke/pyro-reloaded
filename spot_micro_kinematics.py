from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from math import pi
import numpy as np


class SpotMicroKinematics:
    def __init__(self):
        self.d2r = pi / 180
        self.r2d = 180 / pi
        self.height = 0.18
        self.sm = SpotMicroStickFigure(x=0, y=self.height, z=0, theta=5 * self.d2r)
        self.desired_p4_points = np.array([
            [-self.sm.body_length / 2, 0, self.sm.body_width / 2 + self.sm.hip_length],
            [self.sm.body_length / 2, 0, self.sm.body_width / 2 + self.sm.hip_length],
            [self.sm.body_length / 2, 0, -self.sm.body_width / 2 - self.sm.hip_length],
            [-self.sm.body_length / 2, 0, -self.sm.body_width / 2 - self.sm.hip_length]
        ])
        self.sm.set_absolute_foot_coordinates(self.desired_p4_points)