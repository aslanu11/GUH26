"""Two-robot packing cell: shared plan and geometry. Pure Python, no simulator.

Two UF850 arms face each other across one table. A row of six parcels runs
down the middle. Before the shift each robot is assigned a lane of three
parcels and its own carton. That plan is fixed and known to both robots in
advance, the way a warehouse system assigns lanes. While the job runs the
robots exchange nothing: each one only observes the table (where the parcels
are and where the other arm physically is).

Both arms can reach every parcel in the row, so either one can finish the
other's lane. A robot always packs into its own carton.
"""

from __future__ import annotations

import math

ROBOT_A = "station-a"
ROBOT_B = "station-b"
ROBOTS = (ROBOT_A, ROBOT_B)

TABLE_TOP_Z = 0.0
TABLE_CENTER = (0.0, -0.425, -0.02)
TABLE_SIZE = (1.50, 1.45, 0.04)

PARCEL_SIZE = (0.12, 0.09, 0.07)
PARCEL_MASS_KG = 1.0
PARCEL_REST_Z = TABLE_TOP_Z + PARCEL_SIZE[2] / 2
ROW_Y = -0.45

# Arm B sits across the table, rotated 180 degrees, so the cell is symmetric.
ROBOT_INFO = {
    ROBOT_A: {
        "label": "Dum-E",
        "base": (0.0, 0.0, 0.0),
        "yaw_deg": 0.0,
        "carton": "CA",
        "colour": (0.75, 0.18, 0.14),
        # Pick order: outermost first.
        "lane": ("P1", "P2", "P3"),
    },
    ROBOT_B: {
        "label": "U",
        "base": (0.0, -0.90, 0.0),
        "yaw_deg": 180.0,
        "carton": "CB",
        "colour": (0.16, 0.32, 0.70),
        "lane": ("P6", "P5", "P4"),
    },
}

PARCELS = {
    "P1": {"xy": (-0.40, ROW_Y), "color": (0.86, 0.36, 0.26)},
    "P2": {"xy": (-0.26, ROW_Y), "color": (0.90, 0.56, 0.24)},
    "P3": {"xy": (-0.12, ROW_Y), "color": (0.86, 0.72, 0.30)},
    "P4": {"xy": (0.12, ROW_Y), "color": (0.36, 0.70, 0.46)},
    "P5": {"xy": (0.26, ROW_Y), "color": (0.26, 0.62, 0.78)},
    "P6": {"xy": (0.40, ROW_Y), "color": (0.28, 0.42, 0.86)},
}

CARTON_WALL = 0.01
CARTON_HEIGHT = 0.12
CARTONS = {
    "CA": {"center": (-0.40, -0.10), "inner": (0.30, 0.40), "color": (0.70, 0.52, 0.34)},
    "CB": {"center": (0.40, -0.80), "inner": (0.30, 0.40), "color": (0.70, 0.52, 0.34)},
}
# Six place slots per carton (2 x 3), as offsets from the carton centre.
SLOT_OFFSETS = tuple((dx, dy) for dy in (-0.13, 0.0, 0.13) for dx in (-0.075, 0.075))

MARK_TOLERANCE_M = 0.03
# Two normal pick cycles (about 13 s each) with no parcel leaving the other
# robot's lane, while that lane still has parcels, means it has stopped.
STALL_SECONDS = 25.0
# Do not reach into the other robot's lane if any part of its arm is this close.
ARM_CLEARANCE_M = 0.22


def other(robot: str) -> str:
    return ROBOT_B if robot == ROBOT_A else ROBOT_A


def owner_of(parcel: str) -> str:
    return next(robot for robot, info in ROBOT_INFO.items() if parcel in info["lane"])


def mark_xyz(parcel: str) -> tuple[float, float, float]:
    x, y = PARCELS[parcel]["xy"]
    return (x, y, PARCEL_REST_Z)


def in_carton(carton: str, position) -> bool:
    spec = CARTONS[carton]
    return (
        abs(position[0] - spec["center"][0]) <= spec["inner"][0] / 2
        and abs(position[1] - spec["center"][1]) <= spec["inner"][1] / 2
    )


def where(parcel: str, position) -> str:
    """Classify an observed parcel position: 'mark', 'carton:CA', 'carton:CB' or 'elsewhere'."""
    mx, my, mz = mark_xyz(parcel)
    if math.hypot(position[0] - mx, position[1] - my) <= MARK_TOLERANCE_M and abs(position[2] - mz) <= 0.02:
        return "mark"
    for carton in CARTONS:
        if in_carton(carton, position) and position[2] < CARTON_HEIGHT:
            return f"carton:{carton}"
    return "elsewhere"
