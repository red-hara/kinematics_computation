import numpy as np

from kinematics_computation.kinematics.three import vector, quaternion, frame


class Robot:
    def __init__(self, segment_length):
        self.segment_length = segment_length

    def forward_acc(self, g, dg, ddg) -> frame.FrameAcc:
        return (
            frame.FrameAcc(
                vector.Vector3.zero(),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_z(), g[0]),
                vector.Vector3.zero(),
                vector.Vector3(0, 0, dg[0]),
                vector.Vector3.zero(),
                vector.Vector3(0, 0, ddg[0]),
            )
            * frame.FrameAcc(
                vector.Vector3(0, 0, self.segment_length[0]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_y(), g[1]),
                vector.Vector3.zero(),
                vector.Vector3(0, dg[1], 0),
                vector.Vector3.zero(),
                vector.Vector3(0, ddg[1], 0),
            )
            * frame.FrameAcc(
                vector.Vector3(0, 0, self.segment_length[1]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_y(), g[2]),
                vector.Vector3.zero(),
                vector.Vector3(0, dg[2], 0),
                vector.Vector3.zero(),
                vector.Vector3(0, ddg[2], 0),
            )
            * frame.FrameAcc(
                vector.Vector3(0, 0, self.segment_length[2]),
                quaternion.Quaternion.identity(),
                vector.Vector3.zero(),
                vector.Vector3.zero(),
                vector.Vector3.zero(),
                vector.Vector3.zero(),
            )
        )

    def inverse_acc(self, g, dg, translation_acc: vector.Vector3):
        vi = self.forward_acc(g, dg, [0, 0, 0])
        j0 = self.forward_acc(g, [0, 0, 0], [1, 0, 0])
        j1 = self.forward_acc(g, [0, 0, 0], [0, 1, 0])
        j2 = self.forward_acc(g, [0, 0, 0], [0, 0, 1])
        matrix = np.transpose(
            np.array(
                [
                    [j0.translation_acc.x, j0.translation_acc.y, j0.translation_acc.z],
                    [j1.translation_acc.x, j1.translation_acc.y, j1.translation_acc.z],
                    [j2.translation_acc.x, j2.translation_acc.y, j2.translation_acc.z],
                ],
                dtype="float",
            )
        )
        ddg = np.matmul(
            np.linalg.inv(matrix),
            np.array(
                [[translation_acc.x], [translation_acc.y], [translation_acc.z]],
                dtype="float",
            )
            - np.array(
                [
                    [vi.translation_acc.x],
                    [vi.translation_acc.y],
                    [vi.translation_acc.z],
                ],
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

    solution = robot.inverse_acc(g, dg, forward.translation_acc)
    print("Solution: ")
    print(solution)
