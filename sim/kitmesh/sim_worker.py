"""Isaac Sim packing cell: two UFactory UF850 arms with suction, working at the same time.

Launch with the Isaac Lab Python. The arms face each other across one table.
Each robot agent asks for its own picks; this process runs both arms'
motions concurrently: every control step advances each busy arm by one step,
then physics steps once for the whole scene.

Suction is modelled with the Isaac Sim surface gripper: a D6 joint holds the
parcel only while suction is on. Parcels are never teleported during a pick.
"Reset scene" puts parcels back on their marks between demo runs, and says so
in the log.

Commands on the local socket (one JSON line in, one out):
  observe  {robot}           parcel poses and the OTHER arm's link positions,
                             the same things a camera over the cell would see.
                             Agents use only this and their own pick replies.
  pick     {arm, stock, carton, task_id, cube_id}   blocks until done.
  status   full state for the demo page (includes faults; agents never call it).
  fault / restore {arm}      operator-injected arm fault: the arm stops where
                             it is and suction turns off.
  reset    parcels back on marks, arms home, faults cleared.
  camera   {eye, target}     move the viewport camera.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
from pathlib import Path

PHYSICS_DT = 1 / 120
DECIMATION = 4
SUCTION_DESCRIPTION = (
    "UF850 suction cup, modelled by the Isaac Sim surface gripper: a D6 joint holds the parcel "
    "only while suction is on. The parcel is never teleported."
)
CAMERA_EYE = (1.55, -1.75, 1.25)
CAMERA_TARGET = (0.0, -0.45, 0.10)
HOME_TIP_LOCAL = (0.0, -0.25, 0.45)
TRAVEL_Z = 0.30


def main() -> None:
    parser = argparse.ArgumentParser(description="KitMesh Isaac Sim two-robot packing cell")
    parser.add_argument("--port", type=int, default=9401)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--no-snapshots", action="store_true")
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if not args.gui:
        args.headless = True
    # The surface gripper only runs on the CPU physics backend.
    args.device = "cpu"
    app = None
    code = 1
    try:
        app = AppLauncher(args).app
        code = run(app, args)
    except KeyboardInterrupt:
        code = 130
    except Exception as exc:  # noqa: BLE001 — report startup failures in the log the launcher watches
        import traceback

        traceback.print_exc()
        print(f"KITMESH_SIM_FAILED {exc}", flush=True)
        code = 2
    finally:
        if app is not None:
            app.close()
    sys.exit(code)


def run(app, args) -> int:
    import numpy as np
    import torch
    from isaacsim.core.utils.extensions import enable_extension
    from scipy.spatial.transform import Rotation

    enable_extension("isaacsim.robot.surface_gripper")
    app.update()

    import isaaclab.sim as sim_utils
    import usd.schema.isaac.robot_schema as robot_schema
    from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg, SurfaceGripper, SurfaceGripperCfg
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    from kitmesh import uf850
    from kitmesh.cell_layout import (
        CARTON_HEIGHT,
        CARTON_WALL,
        CARTONS,
        PARCEL_MASS_KG,
        PARCEL_REST_Z,
        PARCEL_SIZE,
        PARCELS,
        ROBOT_INFO,
        SLOT_OFFSETS,
        TABLE_CENTER,
        TABLE_SIZE,
        TABLE_TOP_Z,
        in_carton,
        mark_xyz,
    )

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=PHYSICS_DT, device="cpu", render_interval=DECIMATION))
    sim.set_camera_view(eye=list(CAMERA_EYE), target=list(CAMERA_TARGET))
    stage = sim.stage

    _static_box("/World/Table", TABLE_SIZE, TABLE_CENTER, (0.55, 0.55, 0.57), collide=True)
    _static_box("/World/Ground", (5.0, 5.0, 0.04), (0.0, -0.45, -0.80), (0.22, 0.22, 0.22), collide=True)
    light = sim_utils.DomeLightCfg(intensity=2000.0)
    light.func("/World/Light", light)
    for name, info in ROBOT_INFO.items():
        bx, by, _ = info["base"]
        tag = "RobotA" if name == "station-a" else "RobotB"
        _static_box(f"/World/Markers/{tag}_base", (0.30, 0.30, 0.002), (bx, by, TABLE_TOP_Z + 0.001), info["colour"], False)
    for pid, spec in PARCELS.items():
        x, y = spec["xy"]
        _static_box(
            f"/World/Markers/Mark_{pid}",
            (PARCEL_SIZE[0] + 0.012, PARCEL_SIZE[1] + 0.03, 0.002),
            (x, y, TABLE_TOP_Z + 0.001),
            spec["color"],
            collide=False,
        )
    for carton, spec in CARTONS.items():
        cx, cy = spec["center"]
        hx, hy = spec["inner"][0] / 2, spec["inner"][1] / 2
        t, height, colour = CARTON_WALL, CARTON_HEIGHT, spec["color"]
        _static_box(f"/World/Cartons/{carton}/Floor", (2 * hx, 2 * hy, 0.002), (cx, cy, TABLE_TOP_Z + 0.001), colour, False)
        for wall, size, (wx, wy) in (
            ("PlusX", (t, 2 * hy + 2 * t, height), (cx + hx + t / 2, cy)),
            ("MinusX", (t, 2 * hy + 2 * t, height), (cx - hx - t / 2, cy)),
            ("PlusY", (2 * hx, t, height), (cx, cy + hy + t / 2)),
            ("MinusY", (2 * hx, t, height), (cx, cy - hy - t / 2)),
        ):
            _static_box(f"/World/Cartons/{carton}/{wall}", size, (wx, wy, TABLE_TOP_Z + height / 2), colour, True)

    paths = {"station-a": "/World/RobotA/Robot", "station-b": "/World/RobotB/Robot"}
    robots = {
        name: Articulation(uf850.articulation_cfg(paths[name], ROBOT_INFO[name]["base"], ROBOT_INFO[name]["yaw_deg"]))
        for name in ROBOT_INFO
    }

    grippers = {}
    for name, robot_path in paths.items():
        suction_root = robot_path.rsplit("/", 1)[0] + "/Suction"
        flange = f"{robot_path}/{uf850.FLANGE_LINK}"
        cup = UsdGeom.Cylinder.Define(stage, f"{flange}/SuctionCup")
        cup.CreateRadiusAttr(0.025)
        cup.CreateHeightAttr(uf850.TOOL_LENGTH)
        cup.CreateAxisAttr("Z")
        cup.CreateDisplayColorAttr([(0.12, 0.12, 0.12)])
        UsdGeom.XformCommonAPI(cup).SetTranslate(Gf.Vec3d(0, 0, uf850.TOOL_LENGTH / 2))
        joint = UsdPhysics.Joint.Define(stage, f"{suction_root}/AttachmentPoint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(flange)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, uf850.TOOL_LENGTH))
        joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateJointEnabledAttr(True)
        jprim = joint.GetPrim()
        for dof, low, high in (("transX", 1.0, -1.0), ("transY", 1.0, -1.0), ("transZ", 0.0, 0.005)):
            limit = UsdPhysics.LimitAPI.Apply(jprim, dof)
            limit.CreateLowAttr(low)
            limit.CreateHighAttr(high)
        for dof in ("rotX", "rotY", "rotZ"):
            limit = UsdPhysics.LimitAPI.Apply(jprim, dof)
            limit.CreateLowAttr(-2.0)
            limit.CreateHighAttr(2.0)
        robot_schema.ApplyAttachmentPointAPI(jprim)
        jprim.GetAttribute(robot_schema.Attributes.FORWARD_AXIS.name).Set("Z")
        jprim.GetAttribute(robot_schema.Attributes.CLEARANCE_OFFSET.name).Set(0.002)
        gripper_path = f"{suction_root}/SurfaceGripper"
        robot_schema.CreateSurfaceGripper(stage, gripper_path)
        gprim = stage.GetPrimAtPath(gripper_path)
        gprim.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name).SetTargets([jprim.GetPath()])
        grippers[name] = SurfaceGripper(
            SurfaceGripperCfg(
                prim_path=gripper_path,
                max_grip_distance=0.02,
                coaxial_force_limit=200.0,
                shear_force_limit=200.0,
                retry_interval=0.5,
            )
        )

    parcels = {}
    for pid, spec in PARCELS.items():
        x, y, _ = mark_xyz(pid)
        parcels[pid] = RigidObject(
            RigidObjectCfg(
                prim_path=f"/World/Parcels/{pid}",
                spawn=sim_utils.CuboidCfg(
                    size=PARCEL_SIZE,
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.0),
                    mass_props=sim_utils.MassPropertiesCfg(mass=PARCEL_MASS_KG),
                    collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.6, restitution=0.0),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=spec["color"]),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, PARCEL_REST_Z + 0.003)),
            )
        )

    sim.reset()
    home = np.array(uf850.HOME, dtype=np.float32)
    arms = {}
    for name, robot in robots.items():
        joint_ids, _ = robot.find_joints(list(uf850.JOINT_NAMES), preserve_order=True)
        limits = robot.data.joint_pos_limits[0, joint_ids].detach().cpu().numpy()
        home_t = torch.tensor(home[None], device=sim.device)
        robot.write_joint_state_to_sim(home_t, torch.zeros_like(home_t), joint_ids=joint_ids)
        robot.set_joint_position_target(home_t, joint_ids=joint_ids)
        robot.reset()
        arms[name] = {
            "name": name,
            "robot": robot,
            "joint_ids": joint_ids,
            "limits": limits,
            "hold": home.copy(),
            "kin": None,
            "home_tip": None,
            "gen": None,
            "job": None,
            "queue": [],
            "faulted": False,
        }

    camera_lock = threading.Lock()
    pending_camera = {"value": None}

    def joints(arm) -> np.ndarray:
        return arm["robot"].data.joint_pos[0, arm["joint_ids"]].detach().cpu().numpy().copy()

    def suction_state(arm) -> int:
        return int(round(float(grippers[arm["name"]].state[0])))

    def suction(arm, on: bool) -> None:
        gripper = grippers[arm["name"]]
        gripper.set_grippers_command(torch.tensor([1.0 if on else -1.0], device=sim.device))
        gripper.write_data_to_sim()
        gripper.set_grippers_command(torch.tensor([0.0], device=sim.device))

    def step_all() -> None:
        with camera_lock:
            camera, pending_camera["value"] = pending_camera["value"], None
        if camera is not None:
            sim.set_camera_view(eye=camera[0], target=camera[1])
        for arm in arms.values():
            target = np.clip(arm["hold"], arm["limits"][:, 0], arm["limits"][:, 1])
            arm["robot"].set_joint_position_target(torch.as_tensor(target, device=sim.device)[None], joint_ids=arm["joint_ids"])
        for substep in range(DECIMATION):
            for arm in arms.values():
                arm["robot"].write_data_to_sim()
            # Render once per control step (30 frames per simulated second).
            # Rendering every physics substep capped the sim near 0.25x real time.
            sim.step(render=args.gui and substep == DECIMATION - 1)
            for arm in arms.values():
                arm["robot"].update(PHYSICS_DT)
            for gripper in grippers.values():
                gripper.update(PHYSICS_DT)
            for parcel in parcels.values():
                parcel.update(PHYSICS_DT)

    dt = PHYSICS_DT * DECIMATION
    for _ in range(int(1.0 / dt)):
        step_all()

    # Kinematics from the loaded stage; base pose from the measured root body,
    # so the rotated arm B is handled the same way as arm A.
    def joint_records(prefix: str) -> list[dict]:
        found = []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path.startswith(prefix) and prim.IsA(UsdPhysics.Joint):
                joint = UsdPhysics.Joint(prim)

                def quat(value):
                    return [float(value.GetReal()), *[float(part) for part in value.GetImaginary()]]

                found.append(
                    {
                        "name": prim.GetName(),
                        "type": prim.GetTypeName(),
                        "body0": [str(item) for item in joint.GetBody0Rel().GetTargets()],
                        "body1": [str(item) for item in joint.GetBody1Rel().GetTargets()],
                        "local_pos0": list(joint.GetLocalPos0Attr().Get()),
                        "local_pos1": list(joint.GetLocalPos1Attr().Get()),
                        "local_rot0": quat(joint.GetLocalRot0Attr().Get()),
                        "local_rot1": quat(joint.GetLocalRot1Attr().Get()),
                        "axis": prim.GetAttribute("physics:axis").Get(),
                    }
                )
        return found

    for name, arm in arms.items():
        robot = arm["robot"]
        records = joint_records(paths[name] + "/")
        q = joints(arm)
        model0 = uf850.ChainKinematics(records, arm["limits"])
        frames0 = model0.frames(q)
        root = "world" if "world" in robot.body_names else robot.body_names[0]
        index = robot.body_names.index(root)
        measured = np.eye(4)
        measured[:3, 3] = robot.data.body_pos_w[0, index].detach().cpu().numpy()
        wxyz = robot.data.body_quat_w[0, index].detach().cpu().numpy()
        measured[:3, :3] = Rotation.from_quat([wxyz[1], wxyz[2], wxyz[3], wxyz[0]]).as_matrix()
        base = measured @ np.linalg.inv(frames0[root])
        model = uf850.ChainKinematics(records, arm["limits"], base_transform=base)
        frames = model.frames(q)
        worst = 0.0
        for i, body in enumerate(robot.body_names):
            if body in frames:
                delta = robot.data.body_pos_w[0, i].detach().cpu().numpy() - frames[body][:3, 3]
                worst = max(worst, float(np.max(np.abs(delta))))
        if worst > 2e-3:
            print(f"KITMESH_SIM_FAILED {name} FK mismatch, worst {worst:.4f} m", flush=True)
            return 2
        arm["kin"] = model
        yaw = np.radians(ROBOT_INFO[name]["yaw_deg"])
        # Each arm keeps its cup yaw along its own mounting direction, so arm B
        # (rotated 180 degrees) never needs a half-turn of the wrist.
        arm["yaw_axis"] = (float(np.cos(yaw)), float(np.sin(yaw)), 0.0)
        local = np.array(HOME_TIP_LOCAL)
        rotated = np.array([local[0] * np.cos(yaw) - local[1] * np.sin(yaw), local[0] * np.sin(yaw) + local[1] * np.cos(yaw), local[2]])
        arm["home_tip"] = np.array(ROBOT_INFO[name]["base"], dtype=float) + rotated
        print(f"KITMESH_FK_OK {name} worst={worst * 1000:.2f}mm home_tip={np.round(arm['home_tip'], 3).tolist()}", flush=True)

    # ---- motion as generators: one yield per control step -------------------
    def g_move(arm, goal, seconds):
        """Straight-line tool move. IK keyframes every 2 cm are solved as the motion
        reaches them, so one arm's IK never stalls the other arm for long."""
        model = arm["kin"]
        start_q = arm["hold"].astype(float)
        start = model.tool(start_q)[:3, 3]
        goal = np.asarray(goal, dtype=float)
        pieces = max(1, int(np.ceil(np.linalg.norm(goal - start) / 0.02)))
        keyframes = [start_q]

        def keyframe(k):
            while len(keyframes) <= k:
                n = len(keyframes)
                q = model.solve(start + (goal - start) * n / pieces, seed=keyframes[-1], yaw_axis=arm["yaw_axis"])
                if np.max(np.abs(q - keyframes[-1])) > 0.6:
                    raise RuntimeError(f"UF850 IK jumped branches near {np.round(goal, 3).tolist()}")
                keyframes.append(q)
            return keyframes[k]

        keyframe(1)
        count = max(pieces, int(np.ceil(seconds / dt)))
        for step in range(1, count + 1):
            s = step / count
            s = s * s * (3 - 2 * s)
            u = s * pieces
            i = min(int(u), pieces - 1)
            f = u - i
            arm["hold"] = (keyframe(i) + f * (keyframe(i + 1) - keyframe(i))).astype(np.float32)
            yield

    def g_hold(seconds):
        for _ in range(max(1, int(np.ceil(seconds / dt)))):
            yield

    def place_xy(carton: str, pid: str):
        centre = np.array(CARTONS[carton]["center"], dtype=float)
        for offset in SLOT_OFFSETS:
            slot = centre + np.array(offset)
            if not any(
                float(np.linalg.norm(_position(p)[:2] - slot)) < 0.07 for other, p in parcels.items() if other != pid
            ):
                return slot
        raise RuntimeError(f"Carton {carton} has no free place slot")

    def trace(arm, pid, label, start):
        record = {
            "arm": arm["name"],
            "stock": pid,
            "stage": label,
            "tip": [round(float(v), 4) for v in arm["kin"].tool(joints(arm))[:3, 3]],
            "suction": suction_state(arm),
            "parcel": [round(float(v), 4) for v in _position(parcels[pid])],
            "target_moved_m": round(float(np.linalg.norm(_position(parcels[pid])[:2] - start[:2])), 4),
        }
        print(f"KITMESH_GRASP_POSE {json.dumps(record)}", flush=True)

    def g_pick(arm, command):
        pid = command["stock"]
        carton = command["carton"]
        item = command.get("cube_id") or f"parcel-{pid}"
        parcel = parcels[pid]
        start = _position(parcel)
        mark = np.array(mark_xyz(pid))
        if float(np.linalg.norm(start[:2] - mark[:2])) > 0.03 or abs(start[2] - mark[2]) > 0.02:
            return _result(False, f"{item} is not at its mark. Nothing moved.", parcel, carton)
        print(f"KITMESH_PICK_START {arm['name']} {command.get('task_id')} {pid} -> {carton}", flush=True)
        top = start[2] + PARCEL_SIZE[2] / 2
        try:
            slot = place_xy(carton, pid)
            yield from g_move(arm, [start[0], start[1], top + 0.15], 1.5)
            yield from g_move(arm, [start[0], start[1], top + 0.002], 1.0)
            yield from g_hold(0.2)
            suction(arm, True)
            for _ in range(int(1.0 / dt)):
                yield
                if suction_state(arm) == 1:
                    break
            trace(arm, pid, "suction_on", start)
            if suction_state(arm) != 1:
                suction(arm, False)
                yield from g_hold(0.3)
                return _result(False, f"Suction did not seal on {item}", parcel, carton)
            yield from g_move(arm, [start[0], start[1], TRAVEL_Z], 1.2)
            yield from g_hold(0.2)
            if _position(parcel)[2] < start[2] + 0.10 or suction_state(arm) != 1:
                suction(arm, False)
                yield from g_hold(0.5)
                return _result(False, f"{item} was not lifted by the suction cup", parcel, carton)
            yield from g_move(arm, [slot[0], slot[1], TRAVEL_Z], 1.8)
            yield from g_move(arm, [slot[0], slot[1], TABLE_TOP_Z + PARCEL_SIZE[2] + 0.015], 1.2)
            yield from g_hold(0.2)
            suction(arm, False)
            yield from g_hold(0.5)
            trace(arm, pid, "released", start)
            yield from g_move(arm, [slot[0], slot[1], TRAVEL_Z], 0.8)
            yield from g_move(arm, arm["home_tip"], 1.2)
            yield from g_hold(0.6)
        except RuntimeError as exc:
            suction(arm, False)
            return _result(False, str(exc), parcel, carton)
        result = _judge(item, parcel, carton, start, in_carton, PARCEL_REST_Z)
        state = suction_state(arm)
        result["evidence"]["suction_state"] = state
        result["evidence"]["released"] = state == -1
        if result["verified"] and state != -1:
            result["verified"] = False
            result["reason"] = f"Suction was still on when {item} was checked"
        return result

    # Bring both arms to their home pose together.
    def run_together(gens) -> None:
        live = list(gens)
        while live:
            for gen in live[:]:
                try:
                    next(gen)
                except StopIteration:
                    live.remove(gen)
            step_all()

    run_together([g_move(arm, arm["home_tip"], 2.0) for arm in arms.values()])

    published: dict = {"parcels": {}, "arms": {}, "sim_time": 0.0}
    clock = {"t": 0.0}
    state_lock = threading.Lock()
    control: list[tuple[dict, dict, threading.Event]] = []

    def publish() -> None:
        snapshot = {
            pid: {"position": [float(v) for v in _position(p)], "speed": float(np.linalg.norm(_velocity(p)))}
            for pid, p in parcels.items()
        }
        arm_view = {}
        for name, arm in arms.items():
            links = arm["robot"].data.body_pos_w[0].detach().cpu().numpy()
            arm_view[name] = {
                "links": [[round(float(v), 3) for v in row] for row in links],
                "tip": [round(float(v), 3) for v in arm["kin"].tool(joints(arm))[:3, 3]],
                "suction": suction_state(arm),
                "faulted": arm["faulted"],
                "busy": arm["gen"] is not None,
                "stock": arm["job"][0].get("stock") if arm["job"] else None,
            }
        with state_lock:
            published["parcels"] = snapshot
            published["arms"] = arm_view
            published["sim_time"] = round(clock["t"], 3)

    def finish(arm, result: dict) -> None:
        command, box, done = arm["job"]
        result.setdefault("evidence", {})
        result["evidence"].update({"arm": arm["name"], "cube_id": command.get("cube_id"), "task_id": command.get("task_id")})
        print(f"KITMESH_PICK_RESULT {json.dumps(result)}", flush=True)
        box["result"] = result
        arm["gen"] = None
        arm["job"] = None
        done.set()

    def handle_control(command: dict) -> dict:
        kind = command["cmd"]
        if kind in ("fault", "restore"):
            arm = arms.get(command.get("arm"))
            if arm is None:
                return {"ok": False, "reason": "unknown arm"}
            if kind == "fault":
                arm["faulted"] = True
                arm["hold"] = joints(arm).astype(np.float32)
                suction(arm, False)
                if arm["gen"] is not None:
                    arm["gen"].close()
                    parcel = parcels[arm["job"][0]["stock"]]
                    finish(arm, _result(False, "Robot arm faulted mid-pick (operator-injected fault): motion stopped, suction off", parcel, arm["job"][0]["carton"]))
                print(f"KITMESH_FAULT {arm['name']} injected by operator: arm stopped in place, suction off", flush=True)
            else:
                arm["faulted"] = False
                print(f"KITMESH_RESTORE {arm['name']} fault cleared by operator", flush=True)
            return {"ok": True}
        if kind == "reset":
            if any(arm["gen"] is not None or arm["queue"] for arm in arms.values()):
                return {"ok": False, "reason": "A pick is running or queued. Reset refused."}
            for arm in arms.values():
                suction(arm, False)
                arm["faulted"] = False
            for _ in range(int(0.3 / dt)):
                step_all()
            for pid, parcel in parcels.items():
                x, y, _ = mark_xyz(pid)
                state = parcel.data.default_root_state.clone()
                state[0, 0:3] = torch.tensor([x, y, PARCEL_REST_Z + 0.003], device=sim.device)
                state[0, 3:7] = torch.tensor([1.0, 0.0, 0.0, 0.0], device=sim.device)
                state[0, 7:] = 0.0
                parcel.write_root_state_to_sim(state)
            run_together([g_move(arm, arm["home_tip"], 1.5) for arm in arms.values()])
            print("KITMESH_SCENE_RESET parcels on their marks, arms home, suction off, faults cleared", flush=True)
            return {"ok": True}
        return {"ok": False, "reason": "unknown control command"}

    # Reachability check for every arm, parcel and own-carton slot.
    for name, arm in arms.items():
        carton = ROBOT_INFO[name]["carton"]
        failures = []
        for pid in PARCELS:
            x, y, z = mark_xyz(pid)
            for point in ((x, y, z + PARCEL_SIZE[2] / 2 + 0.15), (x, y, z + PARCEL_SIZE[2] / 2 + 0.002)):
                try:
                    arm["kin"].solve(point, seed=arm["hold"], yaw_axis=arm["yaw_axis"])
                except RuntimeError:
                    failures.append(f"{pid}@{point[2]:.2f}")
        for offset in SLOT_OFFSETS:
            cx, cy = CARTONS[carton]["center"]
            for z in (TRAVEL_Z, TABLE_TOP_Z + PARCEL_SIZE[2] + 0.015):
                try:
                    arm["kin"].solve((cx + offset[0], cy + offset[1], z), seed=arm["hold"], yaw_axis=arm["yaw_axis"])
                except RuntimeError:
                    failures.append(f"{carton}{offset}@{z:.2f}")
        print(f"KITMESH_REACH {name} {'all parcels and own carton slots reachable' if not failures else 'UNREACHABLE ' + ', '.join(failures)}", flush=True)

    publish()
    if args.gui:
        try:
            import omni.ui as ui

            for _ in range(10):
                app.update()
            hidden = []
            for title in (
                "Simulation Settings", "Stage", "Layer", "Render Settings", "Property",
                "Semantics Schema Editor", "Content", "Console",
            ):
                window = ui.Workspace.get_window(title)
                if window is not None:
                    window.visible = False
                    hidden.append(title)
            print(f"KITMESH_UI_LAYOUT hid {hidden}; the Window menu brings any of them back", flush=True)
        except Exception as exc:  # noqa: BLE001 — cosmetic only
            print(f"KITMESH_UI_NOTE {exc}", flush=True)

    def request_camera(eye, target) -> None:
        eye = [float(v) for v in eye][:3]
        target = [float(v) for v in target][:3]
        if len(eye) != 3 or len(target) != 3 or not all(np.isfinite(eye + target)):
            raise ValueError("eye and target need three finite numbers each")
        with camera_lock:
            pending_camera["value"] = (eye, target)

    def enqueue_pick(command: dict, box: dict, done: threading.Event) -> None:
        with state_lock:
            arms[command["arm"]]["queue"].append((command, box, done))

    def enqueue_control(command: dict, box: dict, done: threading.Event) -> None:
        with state_lock:
            control.append((command, box, done))

    _serve(args.port, published, state_lock, arms, enqueue_pick, enqueue_control, request_camera)
    print(f"KITMESH_SIM_READY port={args.port} arms=2 robot=UF850 gripper=suction physics=cpu concurrent=yes", flush=True)

    tick = 0
    while app.is_running():
        with state_lock:
            controls, control[:] = list(control), []
            starts = []
            for arm in arms.values():
                if arm["gen"] is None and arm["queue"]:
                    starts.append((arm, arm["queue"].pop(0)))
        for command, box, done in controls:
            try:
                box["result"] = handle_control(command)
            except Exception as exc:  # noqa: BLE001
                box["result"] = {"ok": False, "reason": f"{command['cmd']} failed: {exc}"}
            done.set()
        for arm, job in starts:
            arm["job"] = job
            if arm["faulted"]:
                finish(arm, _result(False, "Robot arm is faulted. Nothing moved.", parcels[job[0]["stock"]], job[0]["carton"]))
                continue
            arm["gen"] = g_pick(arm, job[0])
        active = False
        for arm in arms.values():
            if arm["gen"] is None:
                continue
            active = True
            try:
                next(arm["gen"])
            except StopIteration as stop:
                finish(arm, stop.value)
            except Exception as exc:  # noqa: BLE001 — report instead of killing the cell
                suction(arm, False)
                parcel = parcels[arm["job"][0]["stock"]]
                finish(arm, _result(False, f"Simulator error: {exc}", parcel, arm["job"][0]["carton"]))
        step_all()
        clock["t"] += dt
        tick += 1
        if tick % 3 == 0 or not active:
            publish()
        if not active:
            time.sleep(0.01)
    return 0


def _result(verified: bool, reason: str, parcel, carton: str) -> dict:
    return {
        "ok": True,
        "verified": verified,
        "reason": reason,
        "evidence": {
            "simulator": "isaac-sim",
            "robot": "UFactory UF850",
            "grasp": SUCTION_DESCRIPTION,
            "zone_id": carton,
            "position": [float(value) for value in _position(parcel)],
            "speed": float(_speed(parcel)),
            "released": None,
            "inside_zone": None,
        },
    }


def _judge(item: str, parcel, carton: str, start, in_carton, rest_z: float) -> dict:
    import numpy as np

    position = _position(parcel)
    speed = _speed(parcel)
    inside = in_carton(carton, position)
    away = float(np.linalg.norm(position[:2] - start[:2])) > 0.05
    settled = abs(position[2] - rest_z) <= 0.02 and speed <= 0.05
    result = _result(False, "", parcel, carton)
    result["evidence"]["inside_zone"] = bool(inside)
    if inside and away and settled:
        result["verified"] = True
        result["reason"] = f"{item} released and settled inside carton {carton}"
    else:
        result["reason"] = (
            f"{item} did not settle inside carton {carton} "
            f"(inside={inside}, moved={away}, speed={speed:.3f}, z={position[2]:.3f})"
        )
    return result


def _speed(parcel) -> float:
    import numpy as np

    return float(np.linalg.norm(_velocity(parcel)))


def _position(parcel):
    import numpy as np

    return np.asarray(parcel.data.root_pos_w[0].detach().cpu().numpy(), dtype=float)


def _velocity(parcel):
    import numpy as np

    return np.asarray(parcel.data.root_lin_vel_w[0].detach().cpu().numpy(), dtype=float)


def _static_box(path: str, size, translation, color, collide: bool) -> None:
    import isaaclab.sim as sim_utils

    cfg = sim_utils.CuboidCfg(
        size=tuple(size),
        collision_props=sim_utils.CollisionPropertiesCfg() if collide else None,
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.6, restitution=0.0)
        if collide
        else None,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(color)),
    )
    cfg.func(path, cfg, translation=tuple(translation))


def _serve(port, published, state_lock, arms, enqueue_pick, enqueue_control, request_camera) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen(16)

    def wait(enqueue, command, timeout):
        box: dict = {}
        done = threading.Event()
        enqueue(command, box, done)
        if not done.wait(timeout):
            return {"ok": False, "verified": False, "reason": f"{command['cmd']} timed out"}
        return box.get("result") or {"ok": False, "reason": "no result"}

    def client(conn: socket.socket) -> None:
        try:
            buffer = b""
            while b"\n" not in buffer:
                chunk = conn.recv(65536)
                if not chunk:
                    return
                buffer += chunk
            command = json.loads(buffer.split(b"\n", 1)[0].decode())
            kind = command.get("cmd")
            if kind == "observe":
                me = command.get("robot")
                with state_lock:
                    parcels = {pid: list(v["position"]) for pid, v in published["parcels"].items()}
                    others = [arm["links"] for name, arm in published["arms"].items() if name != me]
                    sim_time = published["sim_time"]
                reply = {"parcels": parcels, "other_arm_links": others[0] if others else [], "sim_time": sim_time}
            elif kind == "status":
                with state_lock:
                    reply = {
                        "sim_time": published["sim_time"],
                        "name": "isaac-sim",
                        "ready": True,
                        "arm": "UFactory UF850 x2",
                        "detail": "Two simulated UF850 arms with suction, running at the same time. " + SUCTION_DESCRIPTION,
                        "parcels": json.loads(json.dumps(published["parcels"])),
                        "arms": {
                            name: {k: v for k, v in arm.items() if k != "links"} for name, arm in published["arms"].items()
                        },
                    }
            elif kind == "pick":
                if command.get("arm") not in arms or command.get("stock") is None or command.get("carton") is None:
                    reply = {"ok": False, "verified": False, "reason": "pick needs arm, stock and carton"}
                else:
                    reply = wait(enqueue_pick, command, 180)
            elif kind in ("fault", "restore", "reset"):
                reply = wait(enqueue_control, command, 60)
            elif kind == "camera":
                try:
                    request_camera(command["eye"], command["target"])
                    reply = {"ok": True}
                except (KeyError, TypeError, ValueError) as exc:
                    reply = {"ok": False, "reason": f"Bad camera request: {exc}"}
            else:
                reply = {"ok": False, "reason": "unknown command"}
            conn.sendall((json.dumps(reply) + "\n").encode())
        except (OSError, json.JSONDecodeError):
            return
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def loop() -> None:
        while True:
            try:
                conn, _ = server.accept()
            except OSError:
                return
            threading.Thread(target=client, args=(conn,), daemon=True).start()

    threading.Thread(target=loop, name="sim-socket", daemon=True).start()


if __name__ == "__main__":
    main()
