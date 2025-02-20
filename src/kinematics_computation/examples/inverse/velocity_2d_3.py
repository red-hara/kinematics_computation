import numpy as np
from typing import Any

from kinematics_computation.kinematics.two import vector, rotation, frame


class Robot:
    def __init__(self, segment_length):
        self.segment_length = segment_length

    def forward_vel(self, g, dg) -> frame.FrameVel:
        return (
            frame.FrameVel(
                vector.Vector2.zero(),
                rotation.Rotation(g[0]),
                vector.Vector2.zero(),
                dg[0],
            )
            * frame.FrameVel(
                vector.Vector2(self.segment_length[0], 0),
                rotation.Rotation(g[1]),
                vector.Vector2.zero(),
                dg[1],
            )
            * frame.FrameVel(
                vector.Vector2(self.segment_length[1], 0),
                rotation.Rotation(g[2]),
                vector.Vector2.zero(),
                dg[2],
            )
            * frame.FrameVel(
                vector.Vector2(self.segment_length[2], 0),
                rotation.Rotation.zero(),
                vector.Vector2.zero(),
                0,
            )
        )

    def inverse_vel(self, g, translation_vel: vector.Vector2, rotation_vel: Any):
        j0 = self.forward_vel(g, [1, 0, 0])
        j1 = self.forward_vel(g, [0, 1, 0])
        j2 = self.forward_vel(g, [0, 0, 1])
        jacobi = np.transpose(
            np.array(
                [
                    [j0.translation_vel.x, j0.translation_vel.y, j0.rotation_vel],
                    [j1.translation_vel.x, j1.translation_vel.y, j1.rotation_vel],
                    [j2.translation_vel.x, j2.translation_vel.y, j2.rotation_vel],
                ],
                dtype="float",
            )
        )
        dg = np.matmul(
            np.linalg.inv(jacobi),
            np.array(
                [[translation_vel.x], [translation_vel.y], [rotation_vel]],
                dtype="float",
            ),
        )
        dg = [dg[0, 0], dg[1, 0], dg[2, 0]]
        return np.array([float(d) for d in dg])


if __name__ == "__main__":
    l = [3, 2, 1.5]
    g = [0.5, 1.2, 1.8]
    dg = np.array([1.0, 2.0, 3.0])
    robot = Robot(l)
    forward = robot.forward_vel(g, dg)
    print("Expected: ")
    print(dg)

    print()

    solution = robot.inverse_vel(g, forward.translation_vel, forward.rotation_vel)
    print("Solution: ")
    print(solution)
