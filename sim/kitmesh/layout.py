"""Shared packing-cell layout: two UF850 stations, parcel stocks, shipping cartons.

Both agents import this module and apply the same ownership rule. Reachability
is a property of the station, not something the dashboard assigns.

Coordinates are metres in the world frame. The table top is z = 0. Arm A's
base is at the origin; arm B's base is ARM_B_OFFSET along +X, far enough apart
that the two 850 mm reach envelopes do not overlap. Each station picks parcels
from two stock marks and places them in an open shipping carton. Every
waypoint was checked against the UF850 USD kinematics with the suction tool
pointing straight down.
"""

from __future__ import annotations

STATION_A = "station-a"
STATION_B = "station-b"
STATIONS = (STATION_A, STATION_B)

STATION_INFO = {
    STATION_A: {
        "label": "Station A — simulated UF850, suction",
        "short": "Station A",
        "role": "robot",
    },
    STATION_B: {
        "label": "Station B — simulated UF850, suction",
        "short": "Station B",
        "role": "robot",
    },
}

ARM_B_OFFSET = (1.80, 0.0, 0.0)

TABLE_TOP_Z = 0.0
TABLE_SIZE = (3.10, 1.20, 0.04)
TABLE_CENTER = (0.90, -0.30, TABLE_TOP_Z - TABLE_SIZE[2] / 2)

# A small mailer parcel: 120 x 90 x 70 mm, 1.0 kg. The UF850 is rated for 5 kg.
PARCEL_SIZE = (0.12, 0.09, 0.07)
PARCEL_MASS_KG = 1.0
PARCEL_REST_Z = TABLE_TOP_Z + PARCEL_SIZE[2] / 2

# Open shipping carton, resting on the table: inner 260 x 260 mm, walls 120 mm.
CARTON_INNER = 0.26
CARTON_WALL = 0.01
CARTON_HEIGHT = 0.12

STOCKS = {
    "S1": {
        "xyz": (-0.25, -0.40, PARCEL_REST_Z),
        "reach": (STATION_A,),
        "color": (0.80, 0.30, 0.22),
        "label": "Stock S1",
    },
    "S2": {
        "xyz": (-0.05, -0.45, PARCEL_REST_Z),
        "reach": (STATION_A, STATION_B),
        "color": (0.25, 0.42, 0.78),
        "label": "Stock S2",
    },
    "S3": {
        "xyz": (ARM_B_OFFSET[0] + 0.05, -0.45, PARCEL_REST_Z),
        "reach": (STATION_A, STATION_B),
        "color": (0.26, 0.62, 0.34),
        "label": "Stock S3",
    },
    "S4": {
        "xyz": (ARM_B_OFFSET[0] + 0.25, -0.40, PARCEL_REST_Z),
        "reach": (STATION_B,),
        "color": (0.86, 0.66, 0.20),
        "label": "Stock S4",
    },
}

# half_extent is the carton's inner half-width: a parcel centre inside it,
# at rest height, is inside the carton walls.
ZONES = {
    "ZA": {
        "xy": (0.30, -0.40),
        "half_extent": CARTON_INNER / 2,
        "reach": (STATION_A, STATION_B),
        "color": (0.62, 0.48, 0.32),
        "label": "Carton ZA",
    },
    "ZB": {
        "xy": (ARM_B_OFFSET[0] - 0.30, -0.40),
        "half_extent": CARTON_INNER / 2,
        "reach": (STATION_A, STATION_B),
        "color": (0.62, 0.48, 0.32),
        "label": "Carton ZB",
    },
}

DEMO_ORDER = (
    {"cube_id": "parcel-S1", "stock_id": "S1", "zone_id": "ZA"},
    {"cube_id": "parcel-S2", "stock_id": "S2", "zone_id": "ZA"},
    {"cube_id": "parcel-S4", "stock_id": "S4", "zone_id": "ZB"},
)

# Submitted during the outage. S3 is free in the demo order, and both stations
# can perform it, so the smaller station id owns it once the order is shared.
WAITING_ORDER = (
    {"cube_id": "parcel-S3", "stock_id": "S3", "zone_id": "ZB"},
)


def stations_that_can(stock_id: str, zone_id: str) -> tuple[str, ...]:
    if stock_id not in STOCKS:
        raise ValueError(f"Unknown stock {stock_id}")
    if zone_id not in ZONES:
        raise ValueError(f"Unknown zone {zone_id}")
    stock_reach = set(STOCKS[stock_id]["reach"])
    zone_reach = set(ZONES[zone_id]["reach"])
    return tuple(station for station in STATIONS if station in stock_reach and station in zone_reach)


def default_owner(stock_id: str, zone_id: str) -> str:
    capable = stations_that_can(stock_id, zone_id)
    if not capable:
        raise ValueError(f"No station can take {stock_id} to {zone_id}")
    return capable[0]


def can_perform(station_id: str, stock_id: str, zone_id: str) -> bool:
    return station_id in stations_that_can(stock_id, zone_id)
