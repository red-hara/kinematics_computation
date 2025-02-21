import sympy as sp

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
        )


if __name__ == "__main__":
    l = list(sp.symbols("l_1:3"))
    gf = sp.symbols("g_1:3", cls=sp.Function)
    t = sp.symbols("t")
    g = [gf[0](t), gf[1](t), gf[1](t)]
    dg = [g[0].diff(t), g[1].diff(t), g[2].diff(t)]
    ddg = [g[0].diff(t, t), g[1].diff(t, t), g[2].diff(t, t)]

    robot = Robot(l)
    solution = robot.forward_acc(g, dg, ddg)

    print("Frame-based solutions")
    sp.pprint(solution.translation_acc.x.expand())
    sp.pprint(solution.translation_acc.y.expand())
    sp.pprint(solution.translation_acc.z.expand())

    sp.pprint(solution.rotation_acc.x.expand())
    sp.pprint(solution.rotation_acc.y.expand())
    sp.pprint(solution.rotation_acc.z.expand())

    print("Differentiation-based solutions")

    sp.pprint(solution.translation.x.diff(t, t).expand())
    sp.pprint(solution.translation.y.diff(t, t).expand())
    sp.pprint(solution.translation.z.diff(t, t).expand())
    sp.pprint(solution.rotation.into_velocity(t).x.diff(t).expand())
    sp.pprint(solution.rotation.into_velocity(t).y.diff(t).expand())
    sp.pprint(solution.rotation.into_velocity(t).z.diff(t).expand())

    print("Difference between solutions")
    sp.pprint(
        (solution.translation_acc.x - solution.translation.x.diff(t, t))
        .expand()
        .collect(dg + ddg)
        .simplify()
        .expand()
    )
    sp.pprint(
        (solution.translation_acc.y - solution.translation.y.diff(t, t))
        .expand()
        .collect(dg + ddg)
        .simplify()
        .expand()
    )
    sp.pprint(
        (solution.translation_acc.z - solution.translation.z.diff(t, t))
        .expand()
        .collect(dg + ddg)
        .simplify()
        .expand()
    )

    sp.pprint(
        (solution.rotation_acc.x - solution.rotation.into_velocity(t).x.diff(t))
        .expand()
        .simplify()
    )
    sp.pprint(
        (solution.rotation_acc.y - solution.rotation.into_velocity(t).y.diff(t))
        .expand()
        .simplify()
    )
    sp.pprint(
        (solution.rotation_acc.z - solution.rotation.into_velocity(t).z.diff(t))
        .expand()
        .simplify()
    )
