import sympy as sp

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
                quaternion.Quaternion.identity(),
                vector.Vector3.zero(),
                vector.Vector3.zero(),
            )
        )


if __name__ == "__main__":
    l = sp.symbols("l_1:4")
    g = sp.symbols("g_1:4", cls=sp.Function)
    t = sp.symbols("t")
    robot = Robot(l)
    solution = robot.forward_vel(
        [g[0](t), g[1](t), g[2](t)],
        [
            g[0](t).diff(t),
            g[1](t).diff(t),
            g[2](t).diff(t),
        ],
    )

    print("Frame-based solutions")
    sp.pprint(solution.translation_vel.x.expand())
    sp.pprint(solution.translation_vel.y.expand())
    sp.pprint(solution.translation_vel.z.expand())

    sp.pprint(solution.rotation_vel.x.expand())
    sp.pprint(solution.rotation_vel.y.expand())
    sp.pprint(solution.rotation_vel.z.expand())

    print("Differentiation-based solutions")

    sp.pprint(solution.translation.x.diff(t).expand())
    sp.pprint(solution.translation.y.diff(t).expand())
    sp.pprint(solution.translation.z.diff(t).expand())
    sp.pprint(solution.rotation.into_velocity(t).x.expand())
    sp.pprint(solution.rotation.into_velocity(t).y.expand())
    sp.pprint(solution.rotation.into_velocity(t).z.expand())

    print("Difference between solutions")
    sp.pprint(
        sp.simplify(
            (solution.translation_vel.x - solution.translation.x.diff(t)).expand()
        )
    )
    sp.pprint(
        sp.simplify(
            (solution.translation_vel.y - solution.translation.y.diff(t)).expand()
        )
    )
    sp.pprint(
        sp.simplify(
            (solution.translation_vel.z - solution.translation.z.diff(t)).expand()
        )
    )

    sp.pprint(
        sp.simplify(
            (solution.rotation_vel.x - solution.rotation.into_velocity(t).x).expand()
        )
    )
    sp.pprint(
        sp.simplify(
            (solution.rotation_vel.y - solution.rotation.into_velocity(t).y).expand()
        )
    )
    sp.pprint(
        sp.simplify(
            (solution.rotation_vel.z - solution.rotation.into_velocity(t).z).expand()
        )
    )
