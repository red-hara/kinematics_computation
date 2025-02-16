import sympy as sp

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


if __name__ == "__main__":
    l = sp.symbols("l_1:4")
    g = sp.symbols("g_1:4", cls=sp.Function)
    t = sp.symbols("t")
    robot = Robot(l)
    solution = robot.forward_vel(
        [g[0](t), g[1](t), g[2](t)], [g[0](t).diff(t), g[1](t).diff(t), g[2](t).diff(t)]
    )

    print("Frame-based solutions")
    sp.pprint(solution.translation_vel.x)
    sp.pprint(solution.translation_vel.y)
    sp.pprint(solution.rotation_vel)

    print()

    print("Differentiation-based solutions")
    sp.pprint(solution.translation.x.diff(t))
    sp.pprint(solution.translation.y.diff(t))
    sp.pprint(solution.rotation.angle.diff(t))
