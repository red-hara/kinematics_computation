from typing import Self, Any


class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @staticmethod
    def zero() -> Self:
        return Vector2(0, 0)

    def ortho_cross(self, other: Any) -> Self:
        return Vector2(-self.y * other, self.x * other)

    def __add__(self, other: Self) -> Self:
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Self) -> Self:
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, other: Any) -> Self:
        return Vector2(self.x * other, self.y * other)
