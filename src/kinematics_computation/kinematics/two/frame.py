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


class FrameAcc:
    def __init__(
        self,
        translation: vector.Vector2,
        rotation: rotation.Rotation,
        translation_vel: vector.Vector2,
        rotation_vel: Any,
        translation_acc: vector.Vector2,
        rotation_acc: Any,
    ):
        self.translation = translation
        self.rotation = rotation
        self.translation_vel = translation_vel
        self.rotation_vel = rotation_vel
        self.translation_acc = translation_acc
        self.rotation_acc = rotation_acc

    def __mul__(self, other: Self) -> Self:
        translation = self.translation + self.rotation.rotate(other.translation)
        rotation = self.rotation + other.rotation
        translation_vel = (
            self.translation_vel
            + self.rotation.rotate(other.translation_vel)
            + self.rotation.rotate(other.translation).ortho_cross(self.rotation_vel)
        )
        rotation_vel = self.rotation_vel + other.rotation_vel
        translation_acc = (
            self.translation_acc
            + self.rotation.rotate(other.translation).ortho_cross(self.rotation_acc)
            + self.rotation.rotate(other.translation)
            .ortho_cross(self.rotation_vel)
            .ortho_cross(self.rotation_vel)
            + self.rotation.rotate(other.translation_vel).ortho_cross(self.rotation_vel)
            * 2
            + self.rotation.rotate(other.translation_acc)
        )
        rotation_acc = self.rotation_acc + other.rotation_acc
        return FrameAcc(
            translation,
            rotation,
            translation_vel,
            rotation_vel,
            translation_acc,
            rotation_acc,
        )
