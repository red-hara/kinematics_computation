import sympy as sp

from kinematics_computation.kinematics.two import vector, rotation, frame


class Robot:
    def __init__(self, segment_length):
        self.segment_length = segment_length

    def forward_pos(self, g) -> frame.FramePos:
        return (
            frame.FramePos(vector.Vector2.zero(), rotation.Rotation(g[0]))
            * frame.FramePos(
                vector.Vector2(self.segment_length[0], 0), rotation.Rotation(g[1])
            )
            * frame.FramePos(
                vector.Vector2(self.segment_length[1], 0), rotation.Rotation(g[2])
            )
            * frame.FramePos(
                vector.Vector2(self.segment_length[2], 0), rotation.Rotation.zero()
            )
        )


if __name__ == "__main__":
    l = sp.symbols("l_1:4")
    g = sp.symbols("g_1:4", cls=sp.Function)
    t = sp.symbols("t")
    robot = Robot(l)
    solution = robot.forward_pos([g[0](t), g[1](t), g[2](t)])
    sp.pprint(solution.translation.x)
    sp.pprint(solution.translation.y)
    sp.pprint(solution.rotation.angle)
