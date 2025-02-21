import numpy as np
from typing import Any

from kinematics_computation.kinematics.two import vector, rotation, frame


class Robot:
    def __init__(self, segment_length):
        self.segment_length = segment_length

    def forward_acc(self, g, dg, ddg) -> frame.FrameAcc:
        return (
            frame.FrameAcc(
                vector.Vector2.zero(),
                rotation.Rotation(g[0]),
                vector.Vector2.zero(),
                dg[0],
                vector.Vector2.zero(),
                ddg[0],
            )
            * frame.FrameAcc(
                vector.Vector2(self.segment_length[0], 0),
                rotation.Rotation(g[1]),
                vector.Vector2.zero(),
                dg[1],
                vector.Vector2.zero(),
                ddg[1],
            )
            * frame.FrameAcc(
                vector.Vector2(self.segment_length[1], 0),
                rotation.Rotation(g[2]),
                vector.Vector2.zero(),
                dg[2],
                vector.Vector2.zero(),
                ddg[2],
            )
            * frame.FrameAcc(
                vector.Vector2(self.segment_length[2], 0),
                rotation.Rotation.zero(),
                vector.Vector2.zero(),
                0,
                vector.Vector2.zero(),
                0,
            )
        )

    def inverse_acc(self, g, dg, translation_acc: vector.Vector2, rotation_acc: Any):
        vi = self.forward_acc(g, dg, [0, 0, 0])
        j0 = self.forward_acc(g, [0, 0, 0], [1, 0, 0])
        j1 = self.forward_acc(g, [0, 0, 0], [0, 1, 0])
        j2 = self.forward_acc(g, [0, 0, 0], [0, 0, 1])
        matrix = np.transpose(
            np.array(
                [
                    [j0.translation_acc.x, j0.translation_acc.y, j0.rotation_acc],
                    [j1.translation_acc.x, j1.translation_acc.y, j1.rotation_acc],
                    [j2.translation_acc.x, j2.translation_acc.y, j2.rotation_acc],
                ],
                dtype="float",
            )
        )
        ddg = np.matmul(
            np.linalg.inv(matrix),
            np.array(
                [[translation_acc.x], [translation_acc.y], [rotation_acc]],
                dtype="float",
            )
            - np.array(
                [[vi.translation_acc.x], [vi.translation_acc.y], [vi.rotation_acc]],
                dtype="float",
            ),
        )
        ddg = [ddg[0, 0], ddg[1, 0], ddg[2, 0]]
        return np.array([float(d) for d in ddg])


if __name__ == "__main__":
    l = [3, 2, 1.5]
    g = [0.5, 1.2, 1.8]
    dg = np.array([1.0, 2.0, 3.0])
    ddg = np.array([1.2, 2.3, 3.4])
    robot = Robot(l)
    forward = robot.forward_acc(g, dg, ddg)
    print("Expected: ")
    print(ddg)

    print()

    solution = robot.inverse_acc(g, dg, forward.translation_acc, forward.rotation_acc)
    print("Solution: ")
    print(solution)
