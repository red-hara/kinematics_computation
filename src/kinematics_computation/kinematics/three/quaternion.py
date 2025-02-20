from typing import Self, Any
import sympy as sp

import kinematics_computation.kinematics.three.vector as vector


class Quaternion:
    def __init__(self, w: Any, i: Any, j: Any, k: Any):
        self.w = w
        self.i = i
        self.j = j
        self.k = k

    @staticmethod
    def identity() -> Self:
        return Quaternion(1, 0, 0, 0)

    @staticmethod
    def from_axis_angle(axis: vector.Vector3, angle: Any) -> Self:
        cos = sp.cos(angle / 2)
        sin = sp.sin(angle / 2)
        return Quaternion(cos, sin * axis.x, sin * axis.y, sin * axis.z)

    def __mul__(self, other: Self) -> Self:
        return Quaternion(
            self.w * other.w - self.i * other.i - self.j * other.j - self.k * other.k,
            self.w * other.i + self.i * other.w + self.j * other.k - self.k * other.j,
            self.w * other.j - self.i * other.k + self.j * other.w + self.k * other.i,
            self.w * other.k + self.i * other.j - self.j * other.i + self.k * other.w,
        )

    def conjugate(self) -> Self:
        return Quaternion(self.w, -self.i, -self.j, -self.k)

    def rotate(self, other: vector.Vector3) -> vector.Vector3:
        pure = self * Quaternion(0, other.x, other.y, other.z) * self.conjugate()
        return vector.Vector3(pure.i, pure.j, pure.k)

    def into_velocity(self, parameter: Any) -> vector.Vector3:
        pure = (
            Quaternion(
                sp.diff(self.w, parameter),
                sp.diff(self.i, parameter),
                sp.diff(self.j, parameter),
                sp.diff(self.k, parameter),
            )
            * self.conjugate()
        )
        return vector.Vector3(pure.i, pure.j, pure.k) * 2
