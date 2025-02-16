from typing import Self, Any

import kinematics_computation.kinematics.two.rotation as rotation
import kinematics_computation.kinematics.two.vector as vector


class FramePos:
    def __init__(self, translation: vector.Vector2, rotation: rotation.Rotation):
        self.translation = translation
        self.rotation = rotation

    def __mul__(self, other: Self) -> Self:
        translation = self.translation + self.rotation.rotate(other.translation)
        rotation = self.rotation + other.rotation
        return FramePos(translation, rotation)


class FrameVel:
    def __init__(
        self,
        translation: vector.Vector2,
        rotation: rotation.Rotation,
        translation_vel: vector.Vector2,
        rotation_vel: Any,
    ):
        self.translation = translation
        self.rotation = rotation
        self.translation_vel = translation_vel
        self.rotation_vel = rotation_vel

    def __mul__(self, other: Self) -> Self:
        translation = self.translation + self.rotation.rotate(other.translation)
        rotation = self.rotation + other.rotation
        translation_vel = (
            self.translation_vel
            + self.rotation.rotate(other.translation_vel)
            + self.rotation.rotate(other.translation).ortho_cross(self.rotation_vel)
        )
        rotation_vel = self.rotation_vel + other.rotation_vel
        return FrameVel(translation, rotation, translation_vel, rotation_vel)
