import sympy as sp
from typing import Self

import kinematics_computation.kinematics.two.vector as vector


class Rotation:
    def __init__(self, angle):
        self.angle = angle

    @staticmethod
    def zero() -> Self:
        return Rotation(0)

    def __add__(self, other: Self) -> Self:
        return Rotation(self.angle + other.angle)

    def rotate(self, other: vector.Vector2) -> vector.Vector2:
        c = sp.cos(self.angle)
        s = sp.sin(self.angle)
        return vector.Vector2(c * other.x - s * other.y, s * other.x + c * other.y)
