import sympy as sp

from kinematics_computation.kinematics.three import vector, quaternion, frame


class Robot:
    def __init__(self, segment_length):
        self.segment_length = segment_length

    def forward_pos(self, g) -> frame.FramePos:
        return (
            frame.FramePos(
                vector.Vector3.zero(),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_z(), g[0]),
            )
            * frame.FramePos(
                vector.Vector3(0, 0, self.segment_length[0]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_y(), g[1]),
            )
            * frame.FramePos(
                vector.Vector3(0, 0, self.segment_length[1]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_y(), g[2]),
            )
            * frame.FramePos(
                vector.Vector3(0, 0, self.segment_length[2]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_z(), g[3]),
            )
            * frame.FramePos(
                vector.Vector3.zero(),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_y(), g[4]),
            )
            * frame.FramePos(
                vector.Vector3(0, 0, self.segment_length[3]),
                quaternion.Quaternion.from_axis_angle(vector.Vector3.unit_z(), g[5]),
            )
        )


if __name__ == "__main__":
    l = sp.symbols("l_1:5")
    g = sp.symbols("g_1:7", cls=sp.Function)
    t = sp.symbols("t")
    robot = Robot(l)
    solution = robot.forward_pos([g[0](t), g[1](t), g[2](t), g[3](t), g[4](t), g[5](t)])
    sp.pprint(solution.translation.x.expand().collect(l).trigsimp())
    sp.pprint(solution.translation.y.expand().collect(l).trigsimp())
    sp.pprint(solution.translation.z.expand().collect(l).trigsimp())
