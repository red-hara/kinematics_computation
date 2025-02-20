import numpy as np

from kinematics_computation.kinematics.three import vector, quaternion, frame


class Robot:
    def __init__(self, segment_length):
        self.segment_length = segment_length

    def forward_vel(self, g, dg) -> frame.FrameVel:
        return (
            frame.FrameVel(
                vector.Vector3.zero(),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_z(), g[0]),
                vector.Vector3.zero(),
                vector.Vector3(0, 0, dg[0]),
            )
            * frame.FrameVel(
                vector.Vector3(0, 0, self.segment_length[0]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_y(), g[1]),
                vector.Vector3.zero(),
                vector.Vector3(0, dg[1], 0),
            )
            * frame.FrameVel(
                vector.Vector3(0, 0, self.segment_length[1]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_y(), g[2]),
                vector.Vector3.zero(),
                vector.Vector3(0, dg[2], 0),
            )
            * frame.FrameVel(
                vector.Vector3(0, 0, self.segment_length[2]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_z(), g[3]),
                vector.Vector3.zero(),
                vector.Vector3(0, 0, dg[3]),
            )
            * frame.FrameVel(
                vector.Vector3.zero(),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_y(), g[4]),
                vector.Vector3.zero(),
                vector.Vector3(0, dg[4], 0),
            )
            * frame.FrameVel(
                vector.Vector3(0, 0, self.segment_length[3]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_z(), g[5]),
                vector.Vector3.zero(),
                vector.Vector3(0, 0, dg[5]),
            )
        )

    def inverse_vel(
        self, g, translation_vel: vector.Vector3, rotation_vel: vector.Vector3
    ):
        j0 = self.forward_vel(g, [1, 0, 0, 0, 0, 0])
        j1 = self.forward_vel(g, [0, 1, 0, 0, 0, 0])
        j2 = self.forward_vel(g, [0, 0, 1, 0, 0, 0])
        j3 = self.forward_vel(g, [0, 0, 0, 1, 0, 0])
        j4 = self.forward_vel(g, [0, 0, 0, 0, 1, 0])
        j5 = self.forward_vel(g, [0, 0, 0, 0, 0, 1])
        jacobi = np.transpose(
            np.array(
                [
                    [
                        j0.translation_vel.x,
                        j0.translation_vel.y,
                        j0.translation_vel.z,
                        j0.rotation_vel.x,
                        j0.rotation_vel.y,
                        j0.rotation_vel.z,
                    ],
                    [
                        j1.translation_vel.x,
                        j1.translation_vel.y,
                        j1.translation_vel.z,
                        j1.rotation_vel.x,
                        j1.rotation_vel.y,
                        j1.rotation_vel.z,
                    ],
                    [
                        j2.translation_vel.x,
                        j2.translation_vel.y,
                        j2.translation_vel.z,
                        j2.rotation_vel.x,
                        j2.rotation_vel.y,
                        j2.rotation_vel.z,
                    ],
                    [
                        j3.translation_vel.x,
                        j3.translation_vel.y,
                        j3.translation_vel.z,
                        j3.rotation_vel.x,
                        j3.rotation_vel.y,
                        j3.rotation_vel.z,
                    ],
                    [
                        j4.translation_vel.x,
                        j4.translation_vel.y,
                        j4.translation_vel.z,
                        j4.rotation_vel.x,
                        j4.rotation_vel.y,
                        j4.rotation_vel.z,
                    ],
                    [
                        j5.translation_vel.x,
                        j5.translation_vel.y,
                        j5.translation_vel.z,
                        j5.rotation_vel.x,
                        j5.rotation_vel.y,
                        j5.rotation_vel.z,
                    ],
                ],
                dtype="float",
            )
        )
        dg = np.matmul(
            np.linalg.inv(jacobi),
            np.array(
                [
                    [translation_vel.x],
                    [translation_vel.y],
                    [translation_vel.z],
                    [rotation_vel.x],
                    [rotation_vel.y],
                    [rotation_vel.z],
                ],
                dtype="float",
            ),
        )
        dg = [dg[0, 0], dg[1, 0], dg[2, 0], dg[3, 0], dg[4, 0], dg[5, 0]]
        return np.array([float(d) for d in dg])


if __name__ == "__main__":
    l = [3, 2, 1.5, 0.5]
    g = [0.5, 1.2, 1.8, -1.2, 0.25, 1.25]
    dg = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    robot = Robot(l)
    forward = robot.forward_vel(g, dg)
    print("Expected: ")
    print(dg)

    print()

    solution = robot.inverse_vel(g, forward.translation_vel, forward.rotation_vel)
    print("Solution: ")
    print(solution)
