from typing import Self, Any

from kinematics_computation.kinematics.three import vector, quaternion


class FramePos:
    def __init__(self, translation: vector.Vector3, rotation: quaternion.Quaternion):
        self.translation = translation
        self.rotation = rotation

    def __mul__(self, other: Self) -> Self:
        translation = self.translation + self.rotation.rotate(other.translation)
        rotation = self.rotation * other.rotation
        return FramePos(translation, rotation)


class FrameVel:
    def __init__(
        self,
        translation: vector.Vector3,
        rotation: quaternion.Quaternion,
        translation_vel: vector.Vector3,
        rotation_vel: vector.Vector3,
    ):
        self.translation = translation
        self.rotation = rotation
        self.translation_vel = translation_vel
        self.rotation_vel = rotation_vel

    def __mul__(self, other: Self) -> Self:
        translation = self.translation + self.rotation.rotate(other.translation)
        rotation = self.rotation * other.rotation
        translation_vel = (
            self.translation_vel
            + self.rotation.rotate(other.translation_vel)
            + self.rotation_vel.cross(self.rotation.rotate(other.translation))
        )
        rotation_vel = self.rotation_vel + self.rotation.rotate(other.rotation_vel)
        return FrameVel(translation, rotation, translation_vel, rotation_vel)


class FrameAcc:
    def __init__(
        self,
        translation: vector.Vector3,
        rotation: quaternion.Quaternion,
        translation_vel: vector.Vector3,
        rotation_vel: vector.Vector3,
        translation_acc: vector.Vector3,
        rotation_acc: vector.Vector3,
    ):
        self.translation = translation
        self.rotation = rotation
        self.translation_vel = translation_vel
        self.rotation_vel = rotation_vel
        self.translation_acc = translation_acc
        self.rotation_acc = rotation_acc

    def __mul__(self, other: Self) -> Self:
        translation = self.translation + self.rotation.rotate(other.translation)
        rotation = self.rotation * other.rotation
        translation_vel = (
            self.translation_vel
            + self.rotation.rotate(other.translation_vel)
            + self.rotation_vel.cross(self.rotation.rotate(other.translation))
        )
        rotation_vel = self.rotation_vel + self.rotation.rotate(other.rotation_vel)
        translation_acc = (
            self.translation_acc
            + self.rotation_acc.cross(self.rotation.rotate(other.translation))
            + self.rotation_vel.cross(
                self.rotation_vel.cross(self.rotation.rotate(other.translation))
            )
            + self.rotation_vel.cross(self.rotation.rotate(other.translation_vel)) * 2
            + self.rotation.rotate(other.translation_acc)
        )
        rotation_acc = (
            self.rotation_acc
            + self.rotation.rotate(other.rotation_acc)
            + self.rotation_vel.cross(self.rotation.rotate(other.rotation_vel))
        )
        return FrameAcc(
            translation,
            rotation,
            translation_vel,
            rotation_vel,
            translation_acc,
            rotation_acc,
        )
