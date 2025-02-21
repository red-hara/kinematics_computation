import sympy as sp

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


if __name__ == "__main__":
    l = sp.symbols("l_1:4")
    g = sp.symbols("g_1:4", cls=sp.Function)
    t = sp.symbols("t")
    robot = Robot(l)
    solution = robot.forward_acc(
        [g[0](t), g[1](t), g[2](t)],
        [g[0](t).diff(t), g[1](t).diff(t), g[2](t).diff(t)],
        [g[0](t).diff(t, t), g[1](t).diff(t, t), g[2](t).diff(t, t)],
    )

    print("Frame-based solutions")
    sp.pprint(solution.translation_acc.x)
    sp.pprint(solution.translation_acc.y)
    sp.pprint(solution.rotation_acc)

    print()

    print("Differentiation-based solutions")
    sp.pprint(solution.translation.x.diff(t, t))
    sp.pprint(solution.translation.y.diff(t, t))
    sp.pprint(solution.rotation.angle.diff(t, t))

    print()

    print("Difference between solutions")
    sp.pprint(
        sp.simplify(solution.translation_acc.x - solution.translation.x.diff(t, t))
    )
    sp.pprint(
        sp.simplify(solution.translation_acc.y - solution.translation.y.diff(t, t))
    )
    sp.pprint(sp.simplify(solution.rotation_acc - solution.rotation.angle.diff(t, t)))
