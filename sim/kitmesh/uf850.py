"""UFactory UF850 for the packing cell: spec, Isaac Lab config, and chain IK.

The arm USD is NVIDIA's published Isaac Sim 5.1 asset (Robots/Ufactory/uf850),
used without its finger-gripper variant. A suction tool is added on link6 by
sim_worker.py. Kinematics are read from the joint frames of the *loaded* stage
and checked against the physics body poses at startup, so no hand-typed DH
table is trusted.
"""

from __future__ import annotations

import numpy as np

USD_PATH = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Robots/Ufactory/uf850/uf850.usd"
)
JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
FLANGE_LINK = "link6"
# Suction tool: flange to cup lip along link6 +Z (which points down in the
# working poses).
TOOL_LENGTH = 0.10
# Tool down, tool +X along world +X. Solved offline from the USD frames.
HOME_DEG = (-90.0, 19.3, -33.0, -180.0, 52.3, 90.0)
HOME = tuple(float(np.radians(v)) for v in HOME_DEG)


def articulation_cfg(prim_path: str, base_pos, yaw_deg: float = 0.0):
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg

    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=USD_PATH,
            variants={"gripper": "None"},
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, max_depenetration_velocity=5.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=16, solver_velocity_iteration_count=1
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(base_pos),
            rot=(float(np.cos(np.radians(yaw_deg) / 2)), 0.0, 0.0, float(np.sin(np.radians(yaw_deg) / 2))),
            joint_pos=dict(zip(JOINT_NAMES, HOME)),
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-6]"],
                stiffness=20000.0,
                damping=800.0,
                effort_limit_sim=1000.0,
                velocity_limit_sim=3.0,
            )
        },
    )


def _rotvec_matrix(axis, angle):
    """Rotation about a unit X, Y or Z axis, in closed form (FK runs thousands of times per move)."""
    c, s = np.cos(angle), np.sin(angle)
    if axis[2] == 1.0:
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    if axis[1] == 1.0:
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _transform(position, quaternion_wxyz):
    from scipy.spatial.transform import Rotation

    matrix = np.eye(4)
    matrix[:3, 3] = np.asarray(position, dtype=float)
    q = np.asarray(quaternion_wxyz, dtype=float)
    matrix[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return matrix


class ChainKinematics:
    """Serial chain from exported USD joint records to the flange, plus a tool offset.

    ``records`` hold name, type, body0, body1, local_pos0/1, local_rot0/1 (wxyz)
    and axis for each joint, as read from the loaded stage.
    """

    def __init__(self, records, limits, base_transform=None, tool_length=TOOL_LENGTH, flange=FLANGE_LINK):
        by_child = {}
        for record in records:
            if len(record["body1"]) != 1:
                continue
            child = record["body1"][0].rsplit("/", 1)[-1]
            parent = record["body0"][0].rsplit("/", 1)[-1] if record["body0"] else None
            by_child[child] = (record, parent)
        chain = []
        link = flange
        while link in by_child:
            chain.append(link)
            link = by_child[link][1]
        chain.reverse()
        self.steps = []
        names = []
        for link in chain:
            record = by_child[link][0]
            axis = None
            if record["type"] == "PhysicsRevoluteJoint":
                axis = np.zeros(3)
                axis["XYZ".index(record["axis"])] = 1.0
                names.append(record["name"])
            self.steps.append(
                (
                    link,
                    axis,
                    _transform(record["local_pos0"], record["local_rot0"]),
                    np.linalg.inv(_transform(record["local_pos1"], record["local_rot1"])),
                )
            )
        if tuple(names) != JOINT_NAMES:
            raise RuntimeError(f"UF850 chain joints {names} do not match {JOINT_NAMES}")
        self.lower = np.asarray(limits, dtype=float)[:6, 0]
        self.upper = np.asarray(limits, dtype=float)[:6, 1]
        self.base = np.eye(4) if base_transform is None else np.asarray(base_transform, dtype=float)
        self.tool_length = tool_length

    def frames(self, q) -> dict:
        q = np.asarray(q, dtype=float)
        matrix = self.base.copy()
        out = {}
        index = 0
        for link, axis, local0, inverse1 in self.steps:
            motion = np.eye(4)
            if axis is not None:
                motion[:3, :3] = _rotvec_matrix(axis, q[index])
                index += 1
            matrix = matrix @ local0 @ motion @ inverse1
            out[link] = matrix.copy()
        tool = out[FLANGE_LINK].copy()
        tool[:3, 3] = tool[:3, 3] + tool[:3, 2] * self.tool_length
        out["tool"] = tool
        return out

    def tool(self, q) -> np.ndarray:
        return self.frames(q)["tool"]

    def solve(self, position, seed, yaw_axis=(1.0, 0.0, 0.0)):
        """Tool tip at ``position``, tool +Z straight down, tool +X along ``yaw_axis``.

        Returns the successful solution closest to ``seed``, or raises.
        """
        from scipy.optimize import least_squares

        target = np.asarray(position, dtype=float)
        down = np.array([0.0, 0.0, -1.0])
        yaw = np.asarray(yaw_axis, dtype=float)
        yaw = yaw / np.linalg.norm(yaw)
        lower, upper = self.lower + 1e-6, self.upper - 1e-6

        def residual(q):
            tool = self.tool(q)
            return np.r_[tool[:3, 3] - target, 0.2 * (tool[:3, 2] - down), 0.2 * (tool[:3, 0] - yaw)]

        seed = np.clip(np.asarray(seed, dtype=float), lower, upper)
        starts = [seed, np.clip(np.asarray(HOME), lower, upper)]
        rng = np.random.default_rng(2026)
        starts.extend(rng.uniform(lower, upper) for _ in range(8))
        best = None
        for start in starts:
            fit = least_squares(residual, start, bounds=(lower, upper), max_nfev=400)
            tool = self.tool(fit.x)
            distance = float(np.linalg.norm(tool[:3, 3] - target))
            tilt = float(np.arccos(np.clip(tool[:3, 2] @ down, -1.0, 1.0)))
            twist = float(np.arccos(np.clip(tool[:3, 0] @ yaw, -1.0, 1.0)))
            if distance <= 0.001 and tilt <= 0.02 and twist <= 0.05:
                cost = float(np.linalg.norm(fit.x - seed))
                if best is None or cost < best[0]:
                    best = (cost, fit.x.copy())
                if cost < 0.5:
                    break
        if best is None:
            raise RuntimeError(f"UF850 IK has no tool-down solution at {np.round(target, 3).tolist()}")
        return best[1]
