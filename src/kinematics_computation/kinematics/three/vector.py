from typing import Self, Any


class Vector3:
    def __init__(self, x: Any, y: Any, z: Any):
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def zero() -> Self:
        return Vector3(0, 0, 0)

    @staticmethod
    def unit_x() -> Self:
        return Vector3(1, 0, 0)

    @staticmethod
    def unit_y() -> Self:
        return Vector3(0, 1, 0)

    @staticmethod
    def unit_z() -> Self:
        return Vector3(0, 0, 1)

    def __add__(self, other: Self) -> Self:
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Self) -> Self:
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other: Any) -> Self:
        return Vector3(self.x * other, self.y * other, self.z * other)

    def cross(self, other: Self) -> Self:
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
