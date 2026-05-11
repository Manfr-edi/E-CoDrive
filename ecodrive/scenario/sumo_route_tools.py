from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version as package_version
from pathlib import Path
import json
import math
import os
import random
import shlex
import shutil
import socket
import subprocess
import sys
import time
from typing import Optional
import xml.etree.ElementTree as ET

try:
    import psutil
except ImportError:  # pragma: no cover - optional runtime dependency
    psutil = None


PROJECT_ROOT = Path(__file__).resolve().parents[2]
EGO_SUMO_VTYPE = "ego_vehicle_type"
SUPPORTED_CARLA_VERSIONS = ("0.9.13", "0.9.15")
DEFAULT_CARLA_VERSION = "0.9.13"
ACTIVE_CARLA_VERSION = None

CARLA_DIR = None
CARLA_SCRIPT = None
CARLA_CONFIG_SCRIPT = None
SUMO_DIR = None
EXAMPLES_DIR = None
NET_DIR = None
ROUTE_DIR = None
OUTPUT_DIR = None
SUMO_TOOLS_DIR = None
CARLA_VTYPES_JSON = None
CARLA_VTYPE_FILE = None
EGO_VTYPE_FILE = None
CARLA_DIST_DIR = None
VTYPE_FILES = ()

DEFAULT_MAP = "Town04"
DEFAULT_VEHICLE_TYPE = "vehicle.bmw.grandtourer"
DEFAULT_EGO_BLUEPRINT = "vehicle.tesla.model3"
AUTOWARE_EGO_VTYPE = "vehicle.lexus.utlexus"
DEFAULT_EGO_BATTERY_CAPACITY = 75000.0
AUTOWARE_CARLA_SPAWN_Z = 2.0
DEFAULT_SUMO_HOME = "/usr/share/sumo"
BUNDLED_SUMO_HOME = PROJECT_ROOT / "sumo"
DEFAULT_CARLA_HOST = "127.0.0.1"
DEFAULT_CARLA_PORT = 2000
DEFAULT_AUTOWARE_DOCKER_FILTER = "autoware_mini"

XSI_NS = "http://www.w3.org/2001/XMLSchema-instance"

ENERGY_EMISSION_CLASS = "Energy"
MMPEVEM_EMISSION_CLASS = "MMPEVEM"

ENERGY_ATTRIBUTE_DEFAULTS = {
    "minGap": "2.50",
    "maxSpeed": "13.88",
    "color": "white",
    "accel": "1.0",
    "decel": "1.0",
    "sigma": "0.0",
    "mass": "1919",
}

ENERGY_PARAM_DEFAULTS = {
    "airDragCoefficient": "0.23",
    "constantPowerIntake": "100",
    "frontSurfaceArea": "2.2",
    "rotatingMass": "80",
    "maximumPower": "350000",
    "propulsionEfficiency": ".98",
    "radialDragCoefficient": "0.1",
    "recuperationEfficiency": ".96",
    "rollDragCoefficient": "0.01",
    "stoppingThreshold": "0.1",
}

MMPEVEM_ATTRIBUTE_DEFAULTS = {
    "color": "white",
    "actionStepLength": "1.0",
}

MMPEVEM_PARAM_DEFAULTS = {
    "mass": "1999",
    "wheelRadius": "0.33",
    "internalMomentOfInertia": "16",
    "rollDragCoefficient": "0.01",
    "airDragCoefficient": "0.23",
    "frontSurfaceArea": "2.22",
    "gearRatio": "9.325",
    "gearEfficiency": "0.96",
    "maximumTorque": "494",
    "maximumPower": "350000",
    "maximumRecuperationTorque": "140",
    "maximumRecuperationPower": "75000",
    "internalBatteryResistance": "0.1575",
    "nominalBatteryVoltage": "357",
    "constantPowerIntake": "360",
    # "powerLossMap": "2,1|0,413.7931,827.5862,1241.3793,1655.1724,2068.9655,2482.7586,2896.5517,3310.3448,3724.1379,4137.931,4551.7241,4965.5172,5379.3103,5793.1034,6206.8966,6620.6897,7034.4828,7448.2759,7862.069,8275.8621,8689.6552,9103.4483,9517.2414,9931.0345,10344.8276,10758.6207,11172.4138,11586.2069,12000;-192.8157,-176.7477,-160.6797,-144.6118,-128.5438,-112.4758,-96.4078,-80.3399,-64.2719,-48.2039,-32.1359,-16.068,0,16.068,32.1359,48.2039,64.2719,80.3399,96.4078,112.4758,128.5438,144.6118,160.6797,176.7477,192.8157,208.8836,224.9516,241.0196,257.0876,273.1555,289.2235,305.2915,321.3594,337.4274,353.4954,369.5634,385.6313|3059.92,3292.2,3684.4,4125.3,4607.96,5128.38,5683.86,6272.42,6892.5,7542.89,8222.55,8930.61,9666.35,10429.1,10310.8,11089.67,10785.82,10390.27,10810.04,10481.93,10979.63,11522.42,11203.92,11766.01,12353.56,12963.17,12593.66,13195.25,13812.23,14443.49,2571.18,2794.54,3160.93,3574.57,4028.8,4519.78,5044.9,5602.28,6190.42,6808.15,7454.48,8128.58,8829.74,9557.33,10310.8,11089.67,10785.82,10390.27,10810.04,10481.93,10979.63,11522.42,11203.92,11766.01,12353.56,12963.17,12593.66,13195.25,13812.23,14443.49,2124.95,2338.35,2679.36,3066.29,3492.74,3955.01,4450.61,4977.7,5534.87,6120.96,6735.04,7376.31,8044.08,8737.75,9456.79,10200.74,10785.82,10390.27,10810.04,10481.93,10979.63,11522.42,11203.92,11766.01,12353.56,12963.17,12593.66,13195.25,13812.23,14443.49,1721.21,1923.61,2239.66,2600.44,2999.77,3434.09,3900.98,4398.68,4925.83,5481.31,6064.23,6673.8,7309.37,7970.36,8656.27,9366.63,10064.34,10390.27,10810.04,10481.93,10979.63,11522.42,11203.92,11766.01,12353.56,12963.17,12593.66,13195.25,13812.23,14443.49,1359.97,1550.33,1841.86,2177.04,2549.9,2957,3396.01,3865.22,4363.31,4889.2,5442.04,6021.05,6625.62,7255.17,7909.23,8587.34,9289.11,9670.59,10040.46,10481.93,10979.63,11522.42,11203.92,11766.01,12353.56,12963.17,12593.66,13195.25,13812.23,14443.49,1041.22,1218.5,1485.94,1796.07,2143.13,2523.76,2935.71,3377.32,3847.31,4344.64,4868.47,5418.07,5992.82,6592.18,7215.67,7862.87,8533.39,9003.21,9328.15,9724.18,10175.82,10671.82,11203.92,11766.01,12353.56,12963.17,12593.66,13195.25,13812.23,14443.49,764.98,928.14,1171.9,1457.55,1779.45,2134.36,2520.07,2934.97,3377.82,3847.62,4343.53,4864.85,5410.97,5981.38,6575.6,7193.22,7833.87,8384.9,8669.14,9024.21,9434.59,9888.97,10379.02,10898.58,11443.04,12008.93,12593.66,13195.25,13812.23,14443.49,531.24,679.23,899.76,1161.47,1458.87,1788.79,2149.09,2538.18,2954.86,3398.14,3867.21,4361.39,4880.08,5422.77,5989,6578.39,7190.55,7813.13,8060.28,8378.21,8751.34,9168.33,9620.81,10102.57,10608.98,11136.54,11682.6,12245.16,12822.71,13414.09,339.99,471.78,669.49,907.82,1181.39,1487.07,1822.77,2186.96,2578.42,2996.21,3439.52,3907.69,4400.14,4916.35,5455.9,6018.38,6603.43,7210.74,7499.12,7783.23,8122.55,8505.71,8924.32,9372.16,9844.56,10337.99,10849.81,11377.99,11920.98,12477.63,191.25,305.79,481.12,696.62,947,1229.19,1541.13,1881.29,2248.5,2641.82,3060.46,3503.76,3971.15,4462.13,4976.27,5513.19,6072.52,6653.95,6983.77,7237,7545.53,7897.97,8285.9,8703.07,9144.81,9607.58,10088.72,10586.18,11098.42,11624.26,85,181.26,334.63,527.86,755.71,1015.15,1304.14,1621.18,1965.1,2334.97,2730.02,3149.58,3593.11,4060.1,4550.14,5062.82,5597.81,6154.78,6512.75,6737.81,7018.29,7342.77,7702.83,8092.19,8506.18,8941.24,9394.7,9864.51,10349.11,10847.33,21.25,98.19,230.02,401.53,607.51,844.95,1111.82,1406.62,1728.22,2075.66,2448.2,2845.17,3266.03,3710.27,4177.48,4667.27,5179.3,5713.25,6085.03,6284.42,6539.38,6838.46,7173.21,7537.36,7926.2,8336.18,8764.61,9209.44,9669.11,10142.42,0,58.32,172.48,327.47,517.95,740.81,993.98,1275.91,1585.42,1921.55,2283.51,2670.64,3082.37,3518.18,3977.64,4460.35,4965.97,5494.17,5876.18,6057.77,6296.78,6581.41,6902.92,7254.84,7632.29,8031.6,8450,8885.36,9336.06,9800.88,21.25,98.19,230.02,401.53,607.51,844.95,1111.82,1406.62,1728.22,2075.66,2448.2,2845.17,3266.03,3710.27,4177.48,4667.27,5179.3,5713.25,6085.03,6284.42,6539.38,6838.46,7173.21,7537.36,7926.2,8336.18,8764.61,9209.44,9669.11,10142.42,85,181.26,334.63,527.86,755.71,1015.15,1304.14,1621.18,1965.1,2334.97,2730.02,3149.58,3593.11,4060.1,4550.14,5062.82,5597.81,6154.78,6512.75,6737.81,7018.29,7342.77,7702.83,8092.19,8506.18,8941.24,9394.7,9864.51,10349.11,10847.33,191.25,305.79,481.12,696.62,947,1229.19,1541.13,1881.29,2248.5,2641.82,3060.46,3503.76,3971.15,4462.13,4976.27,5513.19,6072.52,6653.95,6983.77,7237,7545.53,7897.97,8285.9,8703.07,9144.81,9607.58,10088.72,10586.18,11098.42,11624.26,339.99,471.78,669.49,907.82,1181.39,1487.07,1822.77,2186.96,2578.42,2996.21,3439.52,3907.69,4400.14,4916.35,5455.9,6018.38,6603.43,7210.74,7499.12,7783.23,8122.55,8505.71,8924.32,9372.16,9844.56,10337.99,10849.81,11377.99,11920.98,12477.63,531.24,679.23,899.76,1161.47,1458.87,1788.79,2149.09,2538.18,2954.86,3398.14,3867.21,4361.39,4880.08,5422.77,5989,6578.39,7190.55,7813.13,8060.28,8378.21,8751.34,9168.33,9620.81,10102.57,10608.98,11136.54,11682.6,12245.16,12822.71,13414.09,764.98,928.14,1171.9,1457.55,1779.45,2134.36,2520.07,2934.97,3377.82,3847.62,4343.53,4864.85,5410.97,5981.38,6575.6,7193.22,7833.87,8384.9,8669.14,9024.21,9434.59,9888.97,10379.02,10898.58,11443.04,12008.93,12593.66,13195.25,13812.23,14443.49,1041.22,1218.5,1485.94,1796.07,2143.13,2523.76,2935.71,3377.32,3847.31,4344.64,4868.47,5418.07,5992.82,6592.18,7215.67,7862.87,8533.39,9003.21,9328.15,9724.18,10175.82,10671.82,11203.92,11766.01,12353.56,12963.17,13592.32,14239.12,14902.19,15580.51,1359.97,1550.33,1841.86,2177.04,2549.9,2957,3396.01,3865.22,4363.31,4889.2,5442.04,6021.05,6625.62,7255.17,7909.23,8587.34,9289.11,9670.59,10040.46,10481.93,10979.63,11522.42,12102.14,12712.82,13350.05,14010.59,14692.09,15392.85,16111.74,16848.03,1721.21,1923.61,2239.66,2600.44,2999.77,3434.09,3900.98,4398.68,4925.83,5481.31,6064.23,6673.8,7309.37,7970.36,8656.27,9366.63,10064.34,10390.27,10810.04,11302.37,11852.06,12448.17,13082.76,13750.09,14446.05,15167.75,15913.26,16681.4,17471.69,18284.23,2124.95,2338.35,2679.36,3066.29,3492.74,3955.01,4450.61,4977.7,5534.87,6120.96,6735.04,7376.31,8044.08,8737.75,9456.79,10200.74,10785.82,11166.28,11641.97,12191.89,12801.15,13459.15,14158.41,14893.73,15661.64,16460.13,17288.37,18146.71,19036.72,18284.23,2571.18,2794.54,3160.93,3574.57,4028.8,4519.78,5044.9,5602.28,6190.42,6808.15,7454.48,8128.58,8829.74,9557.33,10310.8,11089.67,11562.13,12003.73,12542.85,13158.98,13837.82,14569.51,15347.47,16167.76,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,3059.92,3292.2,3684.4,4125.3,4607.96,5128.38,5683.86,6272.42,6892.5,7542.89,8222.55,8930.61,9666.35,10429.1,11218.29,12017.36,12398.26,12909.18,13521.37,14215.18,14977.47,15799.94,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,3591.16,3831.31,4249.75,4718.46,5230.21,5780.83,6367.49,6988.11,7641.1,8325.17,9039.24,9782.41,10553.91,11353.06,12179.27,12844.59,13300.53,13891.26,14589.29,15376.72,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,4164.89,4411.88,4856.98,5354.06,5895.56,6477.12,7095.77,7749.36,8436.22,9154.99,9904.55,10683.97,11492.43,12329.22,13193.73,13735.1,14277.15,14961.52,15763.13,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,4781.13,5033.91,5506.1,6032.11,6604.01,7217.24,7868.72,8556.18,9277.86,10032.35,10818.49,11635.29,12481.9,13357.58,14244.44,14696.41,15339.05,16136.09,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,5439.86,5697.4,6197.11,6752.59,7355.56,8001.21,8686.33,9408.55,10166.02,10957.26,11781.06,12636.37,13522.32,14438.12,15180.17,15738.39,16501.23,17438.49,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,6141.09,6402.34,6930,7515.52,8150.2,8829.02,9548.61,10306.48,11100.69,11929.71,12792.25,13687.22,14613.7,15570.87,16190.45,16874.33,17785.24,17438.49,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,6884.82,7148.74,7704.78,8320.88,8987.93,9700.67,10455.55,11249.96,12081.89,12949.7,13852.06,14787.83,15756.02,16747.9,17286.49,18122.91,17785.24,17438.49,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,7671.05,7936.6,8521.45,9168.69,9868.76,10616.16,11407.15,12239.01,13109.61,14017.24,14960.5,15938.2,16949.3,17791.47,18483.6,18122.91,17785.24,17438.49,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,8499.78,8765.92,9380,10058.93,10792.69,11575.49,12403.42,13273.61,14183.84,15132.32,16117.57,17138.33,18193.54,18923.23,18483.6,18122.91,17785.24,17438.49,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,9371.01,9636.7,10280.44,10991.61,11759.72,12578.66,13444.35,14353.78,15304.6,16294.94,17323.25,18388.23,19488.72,18923.23,18483.6,18122.91,17785.24,17438.49,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,10284.74,10548.93,11222.76,11966.74,12769.84,13625.68,14529.95,15479.5,16471.87,17505.11,18577.57,19687.88,20674.02,18923.23,18483.6,18122.91,17785.24,17438.49,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,11240.96,11502.63,12206.97,12984.3,13823.05,14716.53,15660.21,16650.78,17685.67,18762.81,19880.51,21037.3,20674.02,18923.23,18483.6,18122.91,17785.24,17438.49,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23,12239.69,12497.78,13233.06,14044.31,14919.37,15851.22,16835.13,17867.61,18945.98,20068.07,21232.07,22436.48,20674.02,18923.23,18483.6,18122.91,17785.24,17438.49,17067.26,16667.61,16242.8,17182.8,16678.16,17611.3,17028.57,17930.23,17288.37,18146.71,19036.72,18284.23",
}

BASE_COLOR_VALUES = {
    "white": "255,255,255",
    "black": "0,0,0",
    "gray": "128,128,128",
    "silver": "192,192,192",
    "red": "255,0,0",
    "green": "0,128,0",
    "blue": "0,0,255",
    "yellow": "255,255,0",
    "orange": "255,165,0",
    "cyan": "0,255,255",
    "magenta": "255,0,255",
}
BASE_COLOR_NAMES = {
    value: key
    for key, value in BASE_COLOR_VALUES.items()
}


def _normalize_carla_version(version):
    """Normalize a CARLA version string to the repository naming convention."""
    if version is None:
        return DEFAULT_CARLA_VERSION

    normalized = str(version).strip()
    if normalized.startswith("CARLA_"):
        normalized = normalized.replace("CARLA_", "", 1)

    return normalized


def _extract_supported_carla_version(text):
    """Extract the first supported CARLA version mentioned in free-form text."""
    if text is None:
        return None

    normalized = str(text).strip()
    for version in SUPPORTED_CARLA_VERSIONS:
        if version in normalized:
            return version
    return None


def _infer_carla_version_from_dir(carla_dir):
    """Infer the CARLA version from an installation directory."""
    version_file = carla_dir / "VERSION"
    if version_file.exists():
        try:
            detected = _extract_supported_carla_version(
                version_file.read_text(encoding="utf-8", errors="ignore")
            )
        except OSError:
            detected = None
        if detected:
            return detected

    detected = _extract_supported_carla_version(carla_dir.name)
    if detected:
        return detected

    dist_dir = carla_dir / "PythonAPI" / "carla" / "dist"
    if dist_dir.exists():
        for archive in sorted(dist_dir.iterdir()):
            detected = _extract_supported_carla_version(archive.name)
            if detected:
                return detected

    return None


def _carla_installation_priority(carla_dir):
    """Return the sort key used to prioritize discovered CARLA installations."""
    name = carla_dir.name
    return (
        name.lower() != "carla",
        not name.startswith("CARLA_"),
        name.lower(),
    )


def _discover_carla_installations():
    """Discover supported CARLA installations in the repository."""
    installations = {}
    candidates = []

    search_roots = [PROJECT_ROOT]
    nested_root = PROJECT_ROOT / "carla"
    if nested_root.is_dir():
        search_roots.append(nested_root)

    for root_dir in search_roots:
        for child in root_dir.iterdir():
            if not child.is_dir():
                continue
            if not (child / "CarlaUE4.sh").exists():
                continue
            if not (child / "Co-Simulation" / "Sumo").exists():
                continue
            candidates.append(child)

    for child in PROJECT_ROOT.iterdir():
        if not child.is_dir():
            continue
        if not (child / "CarlaUE4.sh").exists():
            continue
        if not (child / "Co-Simulation" / "Sumo").exists():
            continue
        candidates.append(child)

    seen = set()
    unique_candidates = []
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        unique_candidates.append(candidate)

    for carla_dir in sorted(unique_candidates, key=_carla_installation_priority):
        detected_version = _infer_carla_version_from_dir(carla_dir)
        if detected_version not in SUPPORTED_CARLA_VERSIONS:
            continue
        installations.setdefault(detected_version, carla_dir)

    return installations


def available_carla_versions():
    """Return the supported CARLA versions available in the repository."""
    installations = _discover_carla_installations()
    return [
        version
        for version in SUPPORTED_CARLA_VERSIONS
        if version in installations
    ]


def carla_paths(version=None):
    """Resolve the main filesystem paths for a CARLA installation."""
    normalized = _normalize_carla_version(version)
    installations = _discover_carla_installations()
    carla_dir = installations.get(normalized)
    if carla_dir is None:
        available = ", ".join(
            f"{detected_version} ({path.name})"
            for detected_version, path in sorted(installations.items())
        )
        raise FileNotFoundError(
            f"CARLA installation for version {normalized} not found in the project root. "
            f"Available installations: {available or 'none'}."
        )

    sumo_dir = carla_dir / "Co-Simulation" / "Sumo"
    examples_dir = sumo_dir / "examples"
    return {
        "version": normalized,
        "carla_dir": carla_dir,
        "carla_script": carla_dir / "CarlaUE4.sh",
        "carla_config_script": carla_dir / "PythonAPI" / "util" / "config.py",
        "carla_dist_dir": carla_dir / "PythonAPI" / "carla" / "dist",
        "sumo_dir": sumo_dir,
        "examples_dir": examples_dir,
        "net_dir": examples_dir / "net",
        "route_dir": examples_dir / "rou",
        "output_dir": examples_dir / "output",
        "sumo_tools_dir": examples_dir / "tools",
        "carla_vtypes_json": sumo_dir / "data" / "vtypes.json",
        "carla_vtype_file": examples_dir / "carlavtypes.rou.xml",
        "ego_vtype_file": examples_dir / "egovtype.xml",
    }


def set_active_carla_version(version=None):
    """Set CARLA version string."""
    global ACTIVE_CARLA_VERSION
    global CARLA_DIR, CARLA_SCRIPT, CARLA_CONFIG_SCRIPT, CARLA_DIST_DIR
    global SUMO_DIR, EXAMPLES_DIR, NET_DIR, ROUTE_DIR, OUTPUT_DIR, SUMO_TOOLS_DIR
    global CARLA_VTYPES_JSON, CARLA_VTYPE_FILE, EGO_VTYPE_FILE
    global VTYPE_FILES

    paths = carla_paths(version)
    ACTIVE_CARLA_VERSION = paths["version"]
    CARLA_DIR = paths["carla_dir"]
    CARLA_SCRIPT = paths["carla_script"]
    CARLA_CONFIG_SCRIPT = paths["carla_config_script"]
    CARLA_DIST_DIR = paths["carla_dist_dir"]
    SUMO_DIR = paths["sumo_dir"]
    EXAMPLES_DIR = paths["examples_dir"]
    NET_DIR = paths["net_dir"]
    ROUTE_DIR = paths["route_dir"]
    OUTPUT_DIR = paths["output_dir"]
    SUMO_TOOLS_DIR = paths["sumo_tools_dir"]
    CARLA_VTYPES_JSON = paths["carla_vtypes_json"]
    CARLA_VTYPE_FILE = paths["carla_vtype_file"]
    EGO_VTYPE_FILE = paths["ego_vtype_file"]
    VTYPE_FILES = (
        CARLA_VTYPE_FILE,
        EGO_VTYPE_FILE,
    )
    return ACTIVE_CARLA_VERSION


def active_carla_version():
    """Return the active CARLA version string."""
    return ACTIVE_CARLA_VERSION


def current_sumo_dir():
    """Return the current a path relative to the active SUMO co-simulation directory."""
    return SUMO_DIR


def installed_carla_python_api_version():
    """Return the installed carla python api version."""
    try:
        return package_version("carla")
    except PackageNotFoundError:
        return None


def _carla_python_env_var_name(version=None):
    """Return the environment variable name for a version-specific CARLA interpreter."""
    normalized = _normalize_carla_version(version or active_carla_version())
    return f"CARLA_PYTHON_{normalized.replace('.', '_')}"


def resolve_carla_python_executable(version=None):
    """Resolve Python executable used for CARLA helpers."""
    candidates = []

    version_specific = os.environ.get(_carla_python_env_var_name(version))
    if version_specific:
        candidates.append(version_specific)

    generic = os.environ.get("CARLA_PYTHON")
    if generic:
        candidates.append(generic)

    candidates.append(sys.executable)

    seen = set()
    for candidate in candidates:
        resolved = Path(candidate).expanduser().resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if resolved.exists():
            return resolved

    raise FileNotFoundError(
        "No usable Python interpreter found for CARLA. "
        f"Set {_carla_python_env_var_name(version)} or CARLA_PYTHON."
    )


def selected_carla_python_api_archive():
    """Return the selected CARLA Python API archive for the active setup."""
    if CARLA_DIST_DIR is None or not CARLA_DIST_DIR.exists():
        return None

    active_version = active_carla_version()
    if installed_carla_python_api_version() == active_version:
        return None

    candidates = [
        path
        for path in sorted(CARLA_DIST_DIR.iterdir())
        if path.suffix in {".egg", ".whl"}
        and "py2.7" not in path.name
        and "cp27" not in path.name
    ]
    if not candidates:
        return None

    py_major = sys.version_info.major
    py_minor = sys.version_info.minor

    def score(path):
        """Rank CARLA Python API archives by compatibility with the active interpreter."""
        name = path.name
        return (
            f"cp{py_major}{py_minor}" in name or f"py{py_major}.{py_minor}" in name,
            "py3" in name or "cp3" in name,
            path.suffix == ".egg",
        )

    candidates.sort(key=score, reverse=True)
    return candidates[0]


def selected_carla_runtime_library_dirs(python_executable=None):
    """Return the selected runtime library directories needed by CARLA."""
    candidates = []
    executable = Path(python_executable or sys.executable).resolve()
    candidates.append(executable.parent.parent / "lib")

    conda_exe = os.environ.get("CONDA_EXE")
    if conda_exe:
        candidates.append(Path(conda_exe).resolve().parent.parent / "lib")

    home_dir = Path.home()
    for dirname in ("anaconda3", "miniconda3", "miniforge3", "mambaforge"):
        candidates.append(home_dir / dirname / "lib")

    library_dirs = []
    seen = set()
    for path in candidates:
        if path in seen or not path.exists():
            continue
        seen.add(path)
        runtime_markers = (
            path / "libtiff.so.5",
            path / "libomp.so.5",
            path / "libomp.so",
        )
        if any(marker.exists() for marker in runtime_markers):
            library_dirs.append(path)

    return library_dirs


def ensure_carla_python_api_ready():
    """Ensure that the selected CARLA Python API can be imported."""
    python_executable = resolve_carla_python_executable()
    command = [str(python_executable), "-c", "import carla"]
    process = subprocess.run(
        command,
        env=_build_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode == 0:
        return

    details = (process.stderr or process.stdout or "").strip()
    if details:
        details = details.splitlines()[-1]

    raise RuntimeError(
        "The selected CARLA Python API is not usable in the current environment "
        f"(version {active_carla_version()}, interpreter {python_executable}). "
        f"{details or 'import carla failed.'}"
    )


def ensure_carla_runner_dependencies_ready():
    """Ensure that the CARLA runner dependencies are available."""
    python_executable = resolve_carla_python_executable()
    command = [
        str(python_executable),
        "-c",
        (
            "import carla; "
            "import flask; "
            "import lxml.etree; "
            "import traci; "
            "import sumolib"
        ),
    ]
    process = subprocess.run(
        command,
        env=_build_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode == 0:
        return

    details = (process.stderr or process.stdout or "").strip()
    if details:
        details = details.splitlines()[-1]

    raise RuntimeError(
        "The selected CARLA runner Python environment is missing required modules "
        f"(version {active_carla_version()}, interpreter {python_executable}). "
        "Install at least `setuptools`, `flask`, `lxml`, and the SUMO Python tools "
        "(`traci`, `sumolib`) in that interpreter. "
        f"{details or 'dependency import failed.'}"
    )


def _docker_exec_env():
    """Handle a Docker-friendly copy of the current environment."""
    env = os.environ.copy()
    return env


def _prepare_autoware_x11_access():
    """Prepare host X11 access before launching Autoware in Docker."""
    display = str(os.environ.get("DISPLAY", "")).strip()
    if not display:
        raise RuntimeError(
            "DISPLAY is not set on the host, so the Autoware Docker launch "
            "cannot prepare X11 access automatically."
        )

    xhost_binary = shutil.which("xhost")
    if xhost_binary is None:
        raise FileNotFoundError("xhost executable not found in PATH.")

    process = subprocess.run(
        [xhost_binary, "+local:"],
        env=os.environ.copy(),
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        raise RuntimeError(
            "Could not authorize local X11 clients with `xhost +local:`: "
            f"{process.stderr.strip() or process.stdout.strip()}"
        )

    return {
        "display": display,
        "host_command": "xhost +local:",
    }


def find_running_autoware_container(name_filter=DEFAULT_AUTOWARE_DOCKER_FILTER):
    """Find the active Autoware container."""
    docker_binary = shutil.which("docker")
    if docker_binary is None:
        raise FileNotFoundError("Docker executable not found in PATH.")

    process = subprocess.run(
        [
            docker_binary,
            "ps",
            "-a",
            "--format",
            "{{json .}}",
        ],
        env=_docker_exec_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        raise RuntimeError(
            f"Could not inspect Docker containers: {process.stderr.strip() or process.stdout.strip()}"
        )

    filter_text = (name_filter or DEFAULT_AUTOWARE_DOCKER_FILTER).strip().lower()
    candidates = []
    for line in process.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            container = json.loads(line)
        except json.JSONDecodeError:
            continue

        name = str(container.get("Names", ""))
        image = str(container.get("Image", ""))
        search_blob = f"{name} {image}".lower()
        if filter_text and filter_text not in search_blob:
            continue

        score = (
            name.lower() == "autoware_mini",
            search_blob.startswith("autoware_mini "),
            "autoware_mini" in search_blob,
            "autoware" in name.lower(),
            "autoware" in image.lower(),
        )
        candidates.append((score, container))

    if not candidates:
        raise RuntimeError(
            f"No Docker container matching '{filter_text}' was found. "
            "Create/start it first, for example with `docker compose up -d autoware_mini`."
        )

    candidates.sort(key=lambda item: item[0], reverse=True)
    container = candidates[0][1]
    state = str(container.get("State", "")).strip().lower()
    if state != "running":
        container_name = str(container.get("Names") or container.get("ID"))
        status = str(container.get("Status", "")).strip()
        raise RuntimeError(
            f"Docker container '{container_name}' exists but is not active "
            f"(state: {state or 'unknown'}{', ' + status if status else ''}). "
            "Start it before launching Autoware."
        )

    return container


def ensure_autoware_blueprint_available(
    container_name,
    blueprint_id=AUTOWARE_EGO_VTYPE,
    host=DEFAULT_CARLA_HOST,
    port=DEFAULT_CARLA_PORT,
):
    """Ensure that the required Autoware CARLA blueprint exists."""
    docker_binary = shutil.which("docker")
    check_script = (
        "import carla; "
        f"client = carla.Client({host!r}, {int(port)}); "
        "client.set_timeout(5.0); "
        "blueprint_library = client.get_world().get_blueprint_library(); "
        f"blueprint_library.find({blueprint_id!r}); "
        "print('OK')"
    )
    process = subprocess.run(
        [
            docker_binary,
            "exec",
            str(container_name),
            "sh",
            "-lc",
            f"python3 -c {shlex.quote(check_script)}",
        ],
        env=_docker_exec_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode == 0:
        return

    details = (process.stderr.strip() or process.stdout.strip())
    if "blueprint" in details and "not found" in details:
        raise RuntimeError(
            f"CARLA does not provide the required Autoware blueprint '{blueprint_id}'. "
            "Import the UT Lexus asset into CARLA before launching Autoware."
        )

    raise RuntimeError(
        "Could not verify the Autoware CARLA blueprint inside the container: "
        f"{details or 'unknown error'}"
    )


def _configure_autoware_planning_speed_limit_in_container(
    container_name,
    planner_speed_limit_kmh,
):
    """Handle the planner speed limit inside the Autoware container."""
    if planner_speed_limit_kmh is None:
        return None

    docker_binary = shutil.which("docker")
    speed_limit_value = float(planner_speed_limit_kmh)
    update_script = """
import json
import os
import re
from pathlib import Path

path = Path("/opt/catkin_ws/src/autoware_mini/config/planning.yaml")
speed_limit_value = float(os.environ["AUTOWARE_PLANNER_SPEED_LIMIT_KMH"])
text = path.read_text()
updated_text, replacements = re.subn(
    r"^(speed_limit:\\s*)([0-9.]+)(\\s*(?:#.*)?)$",
    lambda match: f"{match.group(1)}{speed_limit_value:.3f}{match.group(3)}",
    text,
    count=1,
    flags=re.MULTILINE,
)
if replacements != 1:
    raise RuntimeError("Could not locate 'speed_limit' inside planning.yaml")
if updated_text != text:
    path.write_text(updated_text)
print(json.dumps({
    "planning_yaml": str(path),
    "planner_speed_limit_kmh": speed_limit_value,
    "updated": updated_text != text,
}))
""".strip()
    process = subprocess.run(
        [
            docker_binary,
            "exec",
            "-e",
            f"AUTOWARE_PLANNER_SPEED_LIMIT_KMH={speed_limit_value:.3f}",
            str(container_name),
            "bash",
            "-lc",
            (
                "source /root/.bashrc && "
                "source /opt/ros/noetic/setup.bash && "
                "source /opt/catkin_ws/devel/setup.bash && "
                f"python3 -c {shlex.quote(update_script)}"
            ),
        ],
        env=_docker_exec_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        details = " | ".join(
            part
            for part in (process.stderr.strip(), process.stdout.strip())
            if part
        )
        raise RuntimeError(
            "Could not configure the Autoware planner speed limit inside the container: "
            f"{details or 'unknown error'}"
        )

    try:
        return json.loads(process.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            "Autoware planner speed-limit setup completed but returned an invalid payload."
        ) from exc


def _ensure_autoware_dynamic_speed_limit(container_name):
    """Ensure Autoware Mini reads the planner speed cap when a route is requested."""
    docker_binary = shutil.which("docker")
    patch_script = r"""
import json
from pathlib import Path

patches = []

lanelet_path = Path("/opt/catkin_ws/src/autoware_mini/nodes/planning/global/lanelet2/lanelet2_global_planner.py")
lanelet_text = lanelet_path.read_text()
lanelet_original = lanelet_text
lanelet_marker = "        rospy.loginfo(\"%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame\", rospy.get_name(),"
lanelet_refresh = "        self.speed_limit = rospy.get_param(\"speed_limit\", self.speed_limit)\n"
if lanelet_refresh not in lanelet_text:
    if lanelet_marker not in lanelet_text:
        raise RuntimeError("Could not locate lanelet2_global_planner goal callback")
    lanelet_text = lanelet_text.replace(lanelet_marker, lanelet_refresh + lanelet_marker, 1)
    lanelet_path.write_text(lanelet_text)
patches.append({
    "path": str(lanelet_path),
    "updated": lanelet_text != lanelet_original,
})

carla_path = Path("/opt/catkin_ws/src/autoware_mini/nodes/platform/carla/carla_waypoints_publisher.py")
carla_text = carla_path.read_text()
carla_original = carla_text
carla_marker = "        msg = Path()\n        msg.header = data.header"
carla_refresh = "        self.speed_limit = rospy.get_param(\"speed_limit\", self.speed_limit)\n"
if carla_refresh not in carla_text:
    if carla_marker not in carla_text:
        raise RuntimeError("Could not locate carla_waypoints_publisher path callback")
    carla_text = carla_text.replace(carla_marker, carla_refresh + carla_marker, 1)
    carla_path.write_text(carla_text)
patches.append({
    "path": str(carla_path),
    "updated": carla_text != carla_original,
})

print(json.dumps({
    "patches": patches,
    "updated": any(patch["updated"] for patch in patches),
}))
""".strip()
    process = subprocess.run(
        [
            docker_binary,
            "exec",
            str(container_name),
            "bash",
            "-lc",
            f"python3 -c {shlex.quote(patch_script)}",
        ],
        env=_docker_exec_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        details = " | ".join(
            part
            for part in (process.stderr.strip(), process.stdout.strip())
            if part
        )
        raise RuntimeError(
            "Could not prepare Autoware dynamic speed-limit handling inside the container: "
            f"{details or 'unknown error'}"
        )

    try:
        return json.loads(process.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            "Autoware dynamic speed-limit setup completed but returned an invalid payload."
        ) from exc


def _ensure_autoware_spawn_point_passthrough(container_name):
    """Ensure start_carla.launch forwards the dashboard spawn_point argument unchanged."""
    docker_binary = shutil.which("docker")
    patch_script = r"""
from pathlib import Path

path = Path("/opt/catkin_ws/src/autoware_mini/launch/start_carla.launch")
text = path.read_text()
original_text = text

before_carla_block = text.split("<!-- Carla specific -->", 1)[0]
if 'name="spawn_point"' not in before_carla_block:
    marker = "    <!-- Carla specific -->"
    if marker not in text:
        raise RuntimeError("Could not locate CARLA argument block in start_carla.launch")
    text = text.replace(
        marker,
        '    <arg name="spawn_point"            default=""                             doc="Autoware ego spawn point: x,y,z,roll,pitch,yaw"/>\n\n' + marker,
        1,
    )

map_arg = "        <arg name='map_name'                            value='$(arg map_name)'/>"
spawn_arg = '        <arg name="spawn_point"                         value="$(arg spawn_point)" />'
if spawn_arg not in text:
    if map_arg not in text:
        raise RuntimeError("Could not locate platform/carla.launch map_name arg")
    text = text.replace(map_arg, map_arg + "\n" + spawn_arg, 1)

if text != original_text:
    path.write_text(text)

print("patched" if text != original_text else "unchanged")
""".strip()
    process = subprocess.run(
        [
            docker_binary,
            "exec",
            str(container_name),
            "python3",
            "-c",
            patch_script,
        ],
        env=_docker_exec_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        details = " | ".join(
            part
            for part in (process.stderr.strip(), process.stdout.strip())
            if part
        )
        raise RuntimeError(
            "Could not patch Autoware start_carla.launch to pass spawn_point unchanged: "
            f"{details or 'unknown error'}"
        )

    return {
        "launch_file": "/opt/catkin_ws/src/autoware_mini/launch/start_carla.launch",
        "status": process.stdout.strip().splitlines()[-1] if process.stdout.strip() else "unknown",
    }


def _publish_autoware_route_in_container(
    container_name,
    initial_pose,
    goal_pose,
    planner_speed_limit_kmh=None,
    ros_timeout_seconds=75,
    publish_initial_pose=True,
):
    """Handle the initial pose and goal inside the Autoware container."""
    docker_binary = shutil.which("docker")
    payload = json.dumps(
        {
            "initial_pose": initial_pose,
            "goal_pose": goal_pose,
            "planner_speed_limit_kmh": planner_speed_limit_kmh,
            "publish_initial_pose": bool(publish_initial_pose),
        },
        separators=(",", ":"),
    )
    publisher_script = """
import json
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


# def build_initial_pose_message(data):
#     message = PoseWithCovarianceStamped()
#     message.header.frame_id = data.get("frame_id", "map")
#     message.pose.pose.position.x = float(data["x"])
#     message.pose.pose.position.y = float(data["y"])
#     message.pose.pose.position.z = float(data.get("z", 0.0))
#     message.pose.pose.orientation.x = float(data["qx"])
#     message.pose.pose.orientation.y = float(data["qy"])
#     message.pose.pose.orientation.z = float(data["qz"])
#     message.pose.pose.orientation.w = float(data["qw"])
#     return message


def build_goal_message(data):
    message = PoseStamped()
    message.header.frame_id = data.get("frame_id", "map")
    message.pose.position.x = float(data["x"])
    message.pose.position.y = float(data["y"])
    message.pose.position.z = float(data.get("z", 0.0))
    message.pose.orientation.x = float(data["qx"])
    message.pose.orientation.y = float(data["qy"])
    message.pose.orientation.z = float(data["qz"])
    message.pose.orientation.w = float(data["qw"])
    return message


def wait_for_connections(publisher, label, timeout_seconds):
    deadline = time.time() + timeout_seconds
    while publisher.get_num_connections() <= 0 and time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.2)
    if publisher.get_num_connections() <= 0:
        raise RuntimeError(f"Timed out waiting for ROS subscribers on {label}")


payload = json.loads(os.environ["AUTOWARE_ROUTE_PAYLOAD"])
timeout_seconds = float(os.environ.get("AUTOWARE_ROUTE_TIMEOUT_SECONDS", "75"))
initial_pose = payload["initial_pose"]
goal_pose = payload["goal_pose"]
planner_speed_limit_kmh = payload.get("planner_speed_limit_kmh")
publish_initial_pose = bool(payload.get("publish_initial_pose", True))

rospy.init_node("dashboard_autoware_route_publisher", anonymous=True, disable_signals=True)
if planner_speed_limit_kmh is not None:
    rospy.set_param("/planning/speed_limit", float(planner_speed_limit_kmh))
rospy.wait_for_message("/carla/ego_vehicle/odometry", Odometry, timeout=timeout_seconds)

goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
# initial_published = False

# initial_message = build_initial_pose_message(initial_pose)
# if publish_initial_pose:
#     initial_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
#     wait_for_connections(initial_publisher, "/initialpose", timeout_seconds)
#     for _ in range(12):
#         initial_message.header.stamp = rospy.Time.now()
#         initial_publisher.publish(initial_message)
#         time.sleep(0.2)
#     initial_published = True
# 
# time.sleep(1.0)

wait_for_connections(goal_publisher, "/move_base_simple/goal", timeout_seconds)
goal_message = build_goal_message(goal_pose)
for _ in range(12):
    goal_message.header.stamp = rospy.Time.now()
    goal_publisher.publish(goal_message)
    time.sleep(0.2)

print(json.dumps({
    # "initial_frame": initial_message.header.frame_id,
    "goal_frame": goal_message.header.frame_id,
    # "initial_xy": [initial_message.pose.pose.position.x, initial_message.pose.pose.position.y],
    "goal_xy": [goal_message.pose.position.x, goal_message.pose.position.y],
    # "initial_pose_published": initial_published,
    "planner_speed_limit_kmh": planner_speed_limit_kmh
}))
""".strip()
    process = subprocess.run(
        [
            docker_binary,
            "exec",
            "-e",
            f"AUTOWARE_ROUTE_PAYLOAD={payload}",
            "-e",
            f"AUTOWARE_ROUTE_TIMEOUT_SECONDS={int(ros_timeout_seconds)}",
            str(container_name),
            "bash",
            "-lc",
            (
                "source /root/.bashrc && "
                "source /opt/ros/noetic/setup.bash && "
                "source /opt/catkin_ws/devel/setup.bash && "
                f"python3 -c {shlex.quote(publisher_script)}"
            ),
        ],
        env=_docker_exec_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        action_label = "Autoware initial pose and goal" if publish_initial_pose else "Autoware goal"
        details = " | ".join(
            part
            for part in (process.stderr.strip(), process.stdout.strip())
            if part
        )
        raise RuntimeError(
            f"Could not publish the {action_label} via ROS 1: "
            f"{details or 'unknown error'}"
        )

    try:
        return json.loads(process.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            "Autoware route publication completed but returned an invalid ROS payload."
        ) from exc


def publish_autoware_route_in_container(
    map_name,
    name_filter=DEFAULT_AUTOWARE_DOCKER_FILTER,
    container_name=None,
    start_edge=None,
    goal_edge=None,
    speed_limit_kmh=None,
    ros_timeout_seconds=75,
    publish_initial_pose=False,
):
    """Publish the Autoware route goal for an already running launch."""
    map_name = str(map_name).strip()
    if not map_name:
        raise ValueError("A valid map name is required to publish an Autoware route.")

    start_edge = (str(start_edge).strip() if start_edge is not None else "")
    goal_edge = (str(goal_edge).strip() if goal_edge is not None else "")
    if not start_edge or not goal_edge:
        raise ValueError("Both a start edge and a goal edge are required to start the Autoware route.")

    initial_pose = autoware_pose_from_edge(
        start_edge,
        map_name=map_name,
        pose_role="initial",
        edge_position="start",
    )
    goal_pose = autoware_pose_from_edge(
        goal_edge,
        map_name=map_name,
        pose_role="goal",
        edge_position="end",
    )

    if container_name is None:
        container = find_running_autoware_container(name_filter=name_filter)
        container_name = container.get("Names") or container.get("ID")

    dynamic_speed_limit_setup = _ensure_autoware_dynamic_speed_limit(container_name)
    route_publication = _publish_autoware_route_in_container(
        container_name,
        initial_pose["pose"],
        goal_pose["pose"],
        planner_speed_limit_kmh=speed_limit_kmh,
        ros_timeout_seconds=ros_timeout_seconds,
        publish_initial_pose=publish_initial_pose,
    )

    return {
        "container_name": str(container_name),
        "start_edge": start_edge,
        "goal_edge": goal_edge,
        "initial_pose": initial_pose,
        "goal_pose": goal_pose,
        "planner_speed_limit_kmh": (
            float(speed_limit_kmh)
            if speed_limit_kmh is not None
            else None
        ),
        "initial_pose_published": bool(
            route_publication.get("initial_pose_published", publish_initial_pose)
            if isinstance(route_publication, dict)
            else publish_initial_pose
        ),
        "dynamic_speed_limit_setup": dynamic_speed_limit_setup,
        "route_publication": route_publication,
    }


def request_autoware_battery_stop(
    name_filter=DEFAULT_AUTOWARE_DOCKER_FILTER,
    ros_timeout_seconds=15,
):
    """Request an emergency stop to the Autoware ego vehicle."""
    container = find_running_autoware_container(name_filter=name_filter)
    container_name = container.get("Names") or container.get("ID")
    docker_binary = shutil.which("docker")
    stop_script = """
import json
import time

import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from std_srvs.srv import Empty


rospy.init_node("dashboard_autoware_battery_stop", anonymous=True, disable_signals=True)
service_called = False
service_error = None

try:
    rospy.wait_for_service("/planning/cancel_route", timeout=float(__import__("os").environ.get("AUTOWARE_STOP_TIMEOUT_SECONDS", "15")))
    cancel_route = rospy.ServiceProxy("/planning/cancel_route", Empty)
    cancel_route()
    service_called = True
except Exception as exc:
    service_error = str(exc)

ackermann_pub = rospy.Publisher("/carla/ego_vehicle/ackermann_cmd", AckermannDrive, queue_size=1)
target_speed_pub = rospy.Publisher("/carla/ego_vehicle/target_speed", Float64, queue_size=1)
time.sleep(0.5)

stop_cmd = AckermannDrive()
stop_cmd.speed = 0.0
stop_cmd.acceleration = -5.0
for _ in range(10):
    ackermann_pub.publish(stop_cmd)
    target_speed_pub.publish(Float64(0.0))
    time.sleep(0.1)

print(json.dumps({
    "container_name": __import__("os").environ.get("AUTOWARE_STOP_CONTAINER"),
    "service_called": service_called,
    "service_error": service_error,
}))
""".strip()
    process = subprocess.run(
        [
            docker_binary,
            "exec",
            "-e",
            f"AUTOWARE_STOP_TIMEOUT_SECONDS={int(ros_timeout_seconds)}",
            "-e",
            f"AUTOWARE_STOP_CONTAINER={container_name}",
            str(container_name),
            "bash",
            "-lc",
            (
                "source /root/.bashrc && "
                "source /opt/ros/noetic/setup.bash && "
                "source /opt/catkin_ws/devel/setup.bash && "
                f"python3 -c {shlex.quote(stop_script)}"
            ),
        ],
        env=_docker_exec_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        details = " | ".join(
            part
            for part in (process.stderr.strip(), process.stdout.strip())
            if part
        )
        raise RuntimeError(
            "Could not request the Autoware emergency stop via ROS 1: "
            f"{details or 'unknown error'}"
        )

    try:
        return json.loads(process.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            "Autoware battery-stop request completed but returned an invalid payload."
        ) from exc


def launch_autoware_carla_in_container(
    map_name,
    name_filter=DEFAULT_AUTOWARE_DOCKER_FILTER,
    spawn_edge=None,
    start_edge=None,
    goal_edge=None,
    speed_limit_kmh=None,
    carla_bridge_passive=False,
    publish_route=True,
):
    """Launch Autoware against the selected CARLA map inside Docker."""
    map_name = str(map_name).strip()
    if not map_name:
        raise ValueError("A valid map name is required to start Autoware.")

    spawn_edge = (str(spawn_edge).strip() if spawn_edge is not None else "")
    start_edge = (str(start_edge).strip() if start_edge is not None else "")
    goal_edge = (str(goal_edge).strip() if goal_edge is not None else "")
    route_requested = bool(start_edge or goal_edge)
    if route_requested and (not start_edge or not goal_edge):
        raise ValueError(
            "Both a start edge and a goal edge are required for the Autoware edge-based launch."
        )

    initial_pose = None
    goal_pose = None
    spawn_point_data = None
    resolved_spawn_edge = start_edge or spawn_edge
    if resolved_spawn_edge:
        spawn_point_data = autoware_spawn_point_from_edge(
            resolved_spawn_edge,
            map_name=map_name,
        )
    if route_requested:
        initial_pose = autoware_pose_from_edge(
            start_edge,
            map_name=map_name,
            pose_role="initial",
            edge_position="start",
        )
        goal_pose = autoware_pose_from_edge(
            goal_edge,
            map_name=map_name,
            pose_role="goal",
            edge_position="end",
        )

    container = find_running_autoware_container(name_filter=name_filter)
    x11_setup = _prepare_autoware_x11_access()
    container_name = container.get("Names") or container.get("ID")
    docker_binary = shutil.which("docker")
    ensure_autoware_blueprint_available(container_name)
    dynamic_speed_limit_setup = _ensure_autoware_dynamic_speed_limit(container_name)
    spawn_point_passthrough = None
    if spawn_point_data is not None:
        spawn_point_passthrough = _ensure_autoware_spawn_point_passthrough(container_name)
    speed_limit_value = None
    planner_speed_limit_setup = None
    if speed_limit_kmh is not None:
        speed_limit_value = float(speed_limit_kmh)
        planner_speed_limit_setup = _configure_autoware_planning_speed_limit_in_container(
            container_name,
            speed_limit_value,
        )
    command = (
        "source /root/.bashrc && "
        "source /opt/ros/noetic/setup.bash && "
        "source /opt/catkin_ws/devel/setup.bash && "
        + f"exec roslaunch autoware_mini start_carla.launch "
        f"map_name:={shlex.quote(map_name)} generate_traffic:=false"
    )
    if spawn_point_data is not None:
        command += f" spawn_point:={shlex.quote(spawn_point_data['spawn_point'])}"
    if carla_bridge_passive:
        command += " passive:=true"
    if route_requested:
        command += " load_goals:=false"
    process = subprocess.run(
        [
            docker_binary,
            "exec",
            "-d",
            "-e",
            f"DISPLAY={x11_setup['display']}",
            "-e",
            "QT_X11_NO_MITSHM=1",
            "-e",
            "XAUTHORITY=/root/.Xauthority",
            "-w",
            "/opt/catkin_ws",
            str(container_name),
            "bash",
            "-lc",
            command,
        ],
        env=_docker_exec_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        raise RuntimeError(
            f"Could not start Autoware in container '{container_name}': "
            f"{process.stderr.strip() or process.stdout.strip()}"
        )

    route_publication = None
    route_publication_error = None
    if route_requested and publish_route:
        try:
            route_publication = _publish_autoware_route_in_container(
                container_name,
                initial_pose["pose"],
                goal_pose["pose"],
                planner_speed_limit_kmh=speed_limit_kmh,
                publish_initial_pose=True,
            )
        except Exception as exc:  # pragma: no cover - depends on live ROS runtime
            route_publication_error = str(exc)

    return {
        "container_name": str(container_name),
        "container_image": str(container.get("Image", "")),
        "display": x11_setup["display"],
        "host_command": x11_setup["host_command"],
        "command": command,
        "carla_bridge_passive": bool(carla_bridge_passive),
        "route_deferred": bool(route_requested and not publish_route),
        "spawn_edge": resolved_spawn_edge or None,
        "spawn_point": (
            spawn_point_data.get("spawn_point")
            if spawn_point_data is not None
            else None
        ),
        "carla_map": (
            initial_pose.get("carla_map")
            if initial_pose is not None
            else (
                spawn_point_data.get("carla_map")
                if spawn_point_data is not None
                else None
            )
        ),
        "spawn_projection_mode": (
            spawn_point_data.get("projection_mode")
            if spawn_point_data is not None
            else None
        ),
        "spawn_projection_error": (
            spawn_point_data.get("projection_error")
            if spawn_point_data is not None
            else None
        ),
        "spawn_point_passthrough": spawn_point_passthrough,
        "start_edge": start_edge or None,
        "goal_edge": goal_edge or None,
        "planner_speed_limit_kmh": (
            speed_limit_value
        ),
        "planner_speed_limit_setup": planner_speed_limit_setup,
        "dynamic_speed_limit_setup": dynamic_speed_limit_setup,
        "initial_pose": initial_pose,
        "goal_pose": goal_pose,
        "route_publication": route_publication,
        "route_publication_error": route_publication_error,
    }


set_active_carla_version(
    DEFAULT_CARLA_VERSION
    if DEFAULT_CARLA_VERSION in available_carla_versions()
    else (available_carla_versions()[0] if available_carla_versions() else DEFAULT_CARLA_VERSION)
)


@dataclass(frozen=True)
class SumoEdge:
    """Store the metadata needed to render and select a SUMO edge."""
    edge_id: str
    from_node: str
    to_node: str
    length: float
    lane_count: int
    shape: tuple


@dataclass(frozen=True)
class ScenarioResult:
    """Describe the artifacts and command produced by scenario generation."""
    map_name: str
    target_edge: str
    route_file: Path
    trip_file: Path
    sumocfg_file: Path
    command: list
    generated_count: int
    requested_count: int
    target_count: int
    mode: str
    stdout: str
    stderr: str
    spawn_begin: float = 0.0
    spawn_end: float = 0.0
    simulation_end: float = 0.0


@dataclass(frozen=True)
class SynchronizationLaunch:
    """Describe a launched co-simulation process and its related files."""
    sync_process: object
    carla_process: object
    sync_log_file: Path
    carla_log_file: Path
    carla_started: bool
    map_loaded: bool
    map_stdout: str
    map_stderr: str
    start_gate_file: Optional[Path]


def available_maps():
    """Return available maps discovered in the active SUMO installation."""
    return sorted(path.stem.replace(".net", "") for path in NET_DIR.glob("*.net.xml"))


def available_vehicle_types():
    """Return available vehicle types for the active workflow."""
    vehicle_types = []
    seen = set()

    for path in VTYPE_FILES:
        if not path.exists():
            continue

        root = ET.parse(path).getroot()
        for element in root.iter("vType"):
            type_id = element.get("id")
            if not type_id or type_id in seen:
                continue
            if type_id in {EGO_SUMO_VTYPE, AUTOWARE_EGO_VTYPE}:
                continue

            vehicle_types.append(type_id)
            seen.add(type_id)

    if DEFAULT_VEHICLE_TYPE in seen:
        vehicle_types.remove(DEFAULT_VEHICLE_TYPE)
        vehicle_types.insert(0, DEFAULT_VEHICLE_TYPE)

    return vehicle_types


def _read_carla_blueprints_json():
    """Read the CARLA blueprint metadata JSON file."""
    if not CARLA_VTYPES_JSON.exists():
        return {}

    with CARLA_VTYPES_JSON.open(encoding="utf-8") as handle:
        return json.load(handle).get("carla_blueprints", {})


def _read_carla_vtypes_data():
    """Read the CARLA vTypes JSON payload."""
    if not CARLA_VTYPES_JSON.exists():
        return {
            "DEFAULT_2_WHEELED_VEHICLE": {"vClass": "motorcycle"},
            "DEFAULT_WHEELED_VEHICLE": {"vClass": "passenger"},
            "carla_blueprints": {},
        }

    with CARLA_VTYPES_JSON.open(encoding="utf-8") as handle:
        return json.load(handle)


def _write_carla_vtypes_data(data):
    """Write the CARLA vTypes JSON payload."""
    CARLA_VTYPES_JSON.parent.mkdir(parents=True, exist_ok=True)
    with CARLA_VTYPES_JSON.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=4)
        handle.write("\n")


def _normalize_color_for_storage(color_value):
    """Normalize a color value for JSON/XML storage."""
    if color_value is None:
        return None

    color_text = str(color_value).strip()
    if not color_text:
        return None

    return BASE_COLOR_VALUES.get(color_text.lower(), color_text)


def _normalize_color_for_form(color_value):
    """Normalize a color value for dashboard form fields."""
    if color_value is None:
        return "white"

    color_text = str(color_value).strip()
    if not color_text:
        return "white"

    if color_text.lower() in BASE_COLOR_VALUES:
        return color_text.lower()

    normalized = color_text.replace(" ", "")
    return BASE_COLOR_NAMES.get(normalized, "white")


def _merge_vtype_xml_specs(specs, path):
    """Merge vehicle-type specs gathered from the XML definitions."""
    if not path.exists():
        return

    root = ET.parse(path).getroot()
    for element in root.iter("vType"):
        type_id = element.get("id")
        if not type_id:
            continue

        target = specs.setdefault(type_id, {})
        for key, value in element.attrib.items():
            if key != "id" and value is not None:
                target[key] = value


def carla_vehicle_type_specs():
    """Collect vehicle-type specs from the configured CARLA XML sources."""
    specs = {
        type_id: dict(values)
        for type_id, values in _read_carla_blueprints_json().items()
    }
    _merge_vtype_xml_specs(specs, CARLA_VTYPE_FILE)

    return {
        type_id: values
        for type_id, values in specs.items()
        if type_id.startswith("vehicle.")
    }


def available_carla_vehicle_types():
    """Return available vehicle types for the active workflow."""
    vehicle_types = sorted(carla_vehicle_type_specs())

    if DEFAULT_EGO_BLUEPRINT in vehicle_types:
        vehicle_types.remove(DEFAULT_EGO_BLUEPRINT)
        vehicle_types.insert(0, DEFAULT_EGO_BLUEPRINT)

    return vehicle_types


def _vtype_params(vtype):
    """Handle the XML parameter dictionary for a vehicle type node."""
    return {
        param.get("key"): param.get("value", "")
        for param in vtype.findall("param")
        if param.get("key")
    }


def _parameter_payload(params):
    """Handle the parameter payload expected by SUMO XML writers."""
    return {
        key: str(value)
        for key, value in (params or {}).items()
        if value is not None and str(value).strip() != ""
    }


def ego_emission_class_value(emission_model):
    """Return the SUMO emission-class value for the selected ego model."""
    if emission_model == MMPEVEM_EMISSION_CLASS:
        return MMPEVEM_EMISSION_CLASS
    return "Energy/unknown"


def ego_model_defaults(emission_model):
    """Return default attribute and parameter dictionaries for an ego emission model."""
    if emission_model == MMPEVEM_EMISSION_CLASS:
        return dict(MMPEVEM_ATTRIBUTE_DEFAULTS), dict(MMPEVEM_PARAM_DEFAULTS)
    return dict(ENERGY_ATTRIBUTE_DEFAULTS), dict(ENERGY_PARAM_DEFAULTS)


def read_ego_vtype_config():
    """Read the stored SUMO ego vehicle-type configuration."""
    config = {
        "sumo_vtype": EGO_SUMO_VTYPE,
        "carla_blueprint": DEFAULT_EGO_BLUEPRINT,
        "emission_model": ENERGY_EMISSION_CLASS,
        "battery_capacity": DEFAULT_EGO_BATTERY_CAPACITY,
        "attributes": dict(ENERGY_ATTRIBUTE_DEFAULTS),
        "parameters": dict(ENERGY_PARAM_DEFAULTS),
    }

    if not EGO_VTYPE_FILE.exists():
        return config

    root = ET.parse(EGO_VTYPE_FILE).getroot()
    vtype = root.find("vType")
    if vtype is None:
        return config

    params = _vtype_params(vtype)
    vtype_id = vtype.get("id") or EGO_SUMO_VTYPE
    carla_blueprint = params.get("carla.blueprint")
    if not carla_blueprint and vtype_id.startswith("vehicle."):
        carla_blueprint = vtype_id

    emission_class = vtype.get("emissionClass", "")
    emission_model = (
        MMPEVEM_EMISSION_CLASS
        if emission_class == MMPEVEM_EMISSION_CLASS
        else ENERGY_EMISSION_CLASS
    )
    attributes, default_params = ego_model_defaults(emission_model)
    attributes.update(
        {
            key: value
            for key, value in vtype.attrib.items()
            if key not in {"id", "emissionClass", "vClass", "length", "width", "height"}
        }
    )
    default_params.update(
        {
            key: value
            for key, value in params.items()
            if key
            not in {
                "has.battery.device",
                "carla.blueprint",
                "device.battery.capacity",
                "device.battery.maximumBatteryCapacity",
            }
        }
    )

    try:
        battery_capacity = float(
            params.get(
                "device.battery.maximumBatteryCapacity",
                params.get("device.battery.capacity", DEFAULT_EGO_BATTERY_CAPACITY),
            )
        )
    except ValueError:
        battery_capacity = DEFAULT_EGO_BATTERY_CAPACITY

    config.update(
        {
            "sumo_vtype": vtype_id,
            "carla_blueprint": carla_blueprint or DEFAULT_EGO_BLUEPRINT,
            "emission_model": emission_model,
            "battery_capacity": battery_capacity,
            "attributes": attributes,
            "parameters": default_params,
        }
    )
    return config


def read_autoware_ego_vtype_config():
    """Read the stored Autoware ego vehicle-type configuration."""
    config = {
        "sumo_vtype": AUTOWARE_EGO_VTYPE,
        "carla_blueprint": AUTOWARE_EGO_VTYPE,
        "emission_model": ENERGY_EMISSION_CLASS,
        "battery_capacity": DEFAULT_EGO_BATTERY_CAPACITY,
        "battery_charge_level": min(5000.0, DEFAULT_EGO_BATTERY_CAPACITY),
        "battery_failure_threshold": 0.0,
        "attributes": dict(ENERGY_ATTRIBUTE_DEFAULTS),
        "parameters": dict(ENERGY_PARAM_DEFAULTS),
    }

    vtype = _read_carla_blueprints_json().get(AUTOWARE_EGO_VTYPE, {})
    if not vtype:
        config["attributes"]["color"] = _normalize_color_for_form(
            config["attributes"].get("color", "white")
        )
        return config

    emission_class = vtype.get("emissionClass", "")
    emission_model = (
        MMPEVEM_EMISSION_CLASS
        if emission_class == MMPEVEM_EMISSION_CLASS
        else ENERGY_EMISSION_CLASS
    )
    attributes, default_params = ego_model_defaults(emission_model)
    attributes.update(
        {
            key: value
            for key, value in vtype.items()
            if key not in {"vClass", "guiShape", "emissionClass", "params"}
        }
    )
    attributes["color"] = _normalize_color_for_form(attributes.get("color", "white"))

    params = vtype.get("params", {}) if isinstance(vtype.get("params"), dict) else {}
    default_params.update(
        {
            key: value
            for key, value in params.items()
            if key not in {
                "has.battery.device",
                "carla.blueprint",
                "device.battery.capacity",
                "device.battery.maximumBatteryCapacity",
                "device.battery.chargeLevel",
                "device.battery.actualBatteryCapacity",
                "dashboard.battery.failureThreshold",
            }
        }
    )

    try:
        battery_capacity = float(
            params.get(
                "device.battery.maximumBatteryCapacity",
                params.get("device.battery.capacity", DEFAULT_EGO_BATTERY_CAPACITY),
            )
        )
    except ValueError:
        battery_capacity = DEFAULT_EGO_BATTERY_CAPACITY

    try:
        battery_charge_level = float(
            params.get(
                "device.battery.actualBatteryCapacity",
                params.get("device.battery.chargeLevel", min(5000.0, battery_capacity)),
            )
        )
    except ValueError:
        battery_charge_level = min(5000.0, battery_capacity)
    battery_charge_level = min(max(0.0, battery_charge_level), battery_capacity)

    try:
        battery_failure_threshold = float(
            params.get("dashboard.battery.failureThreshold", 0.0)
        )
    except ValueError:
        battery_failure_threshold = 0.0
    battery_failure_threshold = min(max(0.0, battery_failure_threshold), battery_capacity)

    config.update(
        {
            "emission_model": emission_model,
            "battery_capacity": battery_capacity,
            "battery_charge_level": battery_charge_level,
            "battery_failure_threshold": battery_failure_threshold,
            "attributes": attributes,
            "parameters": default_params,
        }
    )
    return config


def write_ego_vtype_config(
    carla_blueprint,
    emission_model,
    battery_capacity,
    attributes=None,
    parameters=None,
):
    """Write the stored SUMO ego vehicle-type configuration."""
    specs = carla_vehicle_type_specs().get(carla_blueprint, {})
    attributes = _parameter_payload(attributes)
    parameters = _parameter_payload(parameters)

    vtype_attributes = {
        "id": EGO_SUMO_VTYPE,
        "vClass": specs.get("vClass", "evehicle"),
        "emissionClass": ego_emission_class_value(emission_model),
    }
    for key in ("length", "width", "height", "guiShape"):
        if specs.get(key):
            vtype_attributes[key] = specs[key]

    if emission_model == MMPEVEM_EMISSION_CLASS:
        for key, value in attributes.items():
            if key in MMPEVEM_ATTRIBUTE_DEFAULTS:
                vtype_attributes[key] = value
    else:
        for key, value in attributes.items():
            if key in ENERGY_ATTRIBUTE_DEFAULTS:
                vtype_attributes[key] = value

    root = ET.Element("routes")
    vtype = ET.SubElement(root, "vType", vtype_attributes)

    base_params = {
        "has.battery.device": "true",
        "carla.blueprint": carla_blueprint,
        "device.battery.capacity": str(float(battery_capacity)),
        "device.battery.maximumBatteryCapacity": str(float(battery_capacity)),
    }
    base_params.update(parameters)

    for key, value in base_params.items():
        if value is None or str(value).strip() == "":
            continue
        ET.SubElement(vtype, "param", {"key": key, "value": str(value)})

    _write_xml(EGO_VTYPE_FILE, root)
    return EGO_VTYPE_FILE


def write_autoware_ego_vtype_config(
    emission_model,
    battery_capacity,
    battery_charge_level,
    attributes=None,
    parameters=None,
):
    """Write the stored Autoware ego vehicle-type configuration."""
    data = _read_carla_vtypes_data()
    carla_blueprints = data.setdefault("carla_blueprints", {})
    current_spec = dict(carla_blueprints.get(AUTOWARE_EGO_VTYPE, {}))
    attributes = _parameter_payload(attributes)
    parameters = _parameter_payload(parameters)

    vtype_spec = {}
    if current_spec.get("guiShape"):
        vtype_spec["guiShape"] = current_spec["guiShape"]

    vtype_spec["vClass"] = "evehicle"
    vtype_spec["emissionClass"] = ego_emission_class_value(emission_model)

    if emission_model == MMPEVEM_EMISSION_CLASS:
        allowed_attribute_keys = MMPEVEM_ATTRIBUTE_DEFAULTS
    else:
        allowed_attribute_keys = ENERGY_ATTRIBUTE_DEFAULTS

    for key, value in attributes.items():
        if key not in allowed_attribute_keys:
            continue
        if key == "color":
            normalized_color = _normalize_color_for_storage(value)
            if normalized_color:
                vtype_spec[key] = normalized_color
            continue
        vtype_spec[key] = value

    vtype_params = {
        "has.battery.device": "true",
        "device.battery.capacity": str(float(battery_capacity)),
        "device.battery.maximumBatteryCapacity": str(float(battery_capacity)),
        "device.battery.chargeLevel": str(
            min(max(0.0, float(battery_charge_level)), float(battery_capacity))
        ),
        "device.battery.actualBatteryCapacity": str(
            min(max(0.0, float(battery_charge_level)), float(battery_capacity))
        ),
    }
    vtype_params.update(parameters)
    vtype_spec["params"] = vtype_params

    carla_blueprints[AUTOWARE_EGO_VTYPE] = vtype_spec
    _write_carla_vtypes_data(data)
    return CARLA_VTYPES_JSON


def map_net_file(map_name=DEFAULT_MAP):
    """Return the the `.net.xml` file for the selected map."""
    return NET_DIR / f"{map_name}.net.xml"


def map_route_file(map_name=DEFAULT_MAP):
    """Return the the `.rou.xml` file for the selected map."""
    return ROUTE_DIR / f"custom_{map_name}_traffic.rou.xml"


def map_trip_file(map_name=DEFAULT_MAP):
    """Return the the `.trips.xml` file for the selected map."""
    return ROUTE_DIR / f"custom_{map_name}_traffic.trips.xml"


def map_sumocfg_file(map_name=DEFAULT_MAP):
    """Return the the `.sumocfg` file for the selected map."""
    return EXAMPLES_DIR / f"custom_{map_name}.sumocfg"


def relative_to_examples(path):
    """Return the relative to examples."""
    return path.relative_to(EXAMPLES_DIR).as_posix()


def _net_location_offset(map_name=DEFAULT_MAP):
    """Read the coordinate offset stored in a SUMO net file."""
    root = ET.parse(map_net_file(map_name)).getroot()
    location = root.find("location")
    net_offset_text = (
        location.get("netOffset", "0.0,0.0")
        if location is not None
        else "0.0,0.0"
    )
    try:
        offset_x, offset_y = net_offset_text.split(",", 1)
        return float(offset_x), float(offset_y)
    except (TypeError, ValueError):
        return 0.0, 0.0


def _parse_shape(shape_text):
    """Parse shape."""
    points = []
    for token in (shape_text or "").split():
        values = token.split(",")
        if len(values) < 2:
            continue
        points.append((float(values[0]), float(values[1])))
    return tuple(points)


def _lane_allows_road_vehicle(lane):
    """Return whether a SUMO lane can be used by road vehicles."""
    lane_type = lane.get("type")
    if lane_type == "driving":
        return True

    allow = set((lane.get("allow") or "").split())
    disallow = set((lane.get("disallow") or "").split())
    road_classes = {
        "passenger",
        "private",
        "evehicle",
        "authority",
        "emergency",
        "truck",
        "motorcycle",
        "taxi",
        "bus",
        "coach",
        "delivery",
    }

    if "all" in disallow:
        return False
    if allow:
        return bool(allow & road_classes)
    return not bool(disallow & {"passenger", "private", "evehicle"})


def read_sumo_edges(map_name=DEFAULT_MAP):
    """Read sumo edges."""
    net_file = map_net_file(map_name)
    root = ET.parse(net_file).getroot()
    edges = []

    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        if not edge_id or edge_id.startswith(":"):
            continue
        if edge.get("function") in {"internal", "walkingarea", "crossing"}:
            continue

        road_lanes = [lane for lane in edge.findall("lane") if _lane_allows_road_vehicle(lane)]
        if not road_lanes:
            continue

        shape = _parse_shape(road_lanes[0].get("shape")) or _parse_shape(edge.get("shape"))
        if len(shape) < 2:
            continue

        length = max(float(lane.get("length", "0") or 0) for lane in road_lanes)
        edges.append(
            SumoEdge(
                edge_id=edge_id,
                from_node=edge.get("from", ""),
                to_node=edge.get("to", ""),
                length=length,
                lane_count=len(road_lanes),
                shape=shape,
            )
        )

    return sorted(edges, key=lambda item: item.edge_id)


def edge_label(edge):
    """Build a readable label for a SUMO edge selection."""
    suffix = "lane" if edge.lane_count == 1 else "lanes"
    direction = edge_direction_label(edge)
    return (
        f"{edge.edge_id} | {direction} "
        f"({edge.length:.0f} m, {edge.lane_count} {suffix})"
    )


def edge_direction_label(edge):
    """Build a direction label for a SUMO edge selection."""
    if len(edge.shape) < 2:
        return f"{edge.from_node}->{edge.to_node}"

    start = edge.shape[0]
    end = edge.shape[-1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]

    if abs(dx) >= abs(dy):
        cardinal = "east" if dx >= 0 else "west"
    else:
        cardinal = "north" if dy >= 0 else "south"

    nodes = f"{edge.from_node}->{edge.to_node}" if edge.from_node and edge.to_node else ""
    coords = f"({start[0]:.0f},{start[1]:.0f}) -> ({end[0]:.0f},{end[1]:.0f})"
    return f"{nodes} toward {cardinal} {coords}".strip()


def opposite_edge_id(edge_id, edge_ids):
    """Return the opposite-direction edge id when it exists."""
    candidate = edge_id[1:] if edge_id.startswith("-") else f"-{edge_id}"
    return candidate if candidate in edge_ids else None


def edge_direction_options(edges, edge_id):
    """Return the selectable direction variants for an edge id."""
    edge_ids = {edge.edge_id for edge in edges}
    options = [edge_id]
    opposite = opposite_edge_id(edge_id, edge_ids)
    if opposite:
        options.append(opposite)
    return options


def _point_along_shape(shape, distance_from_start):
    """Interpolate a point along a polyline at the requested distance."""
    if len(shape) < 2:
        raise ValueError("At least two points are required to compute a spawn direction.")

    remaining = max(float(distance_from_start), 0.0)
    fallback_start = shape[0]
    fallback_end = shape[1]

    for start, end in zip(shape, shape[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        if segment_length <= 1e-6:
            continue

        fallback_start = start
        fallback_end = end

        if remaining <= segment_length:
            ratio = remaining / segment_length
            return (
                start[0] + dx * ratio,
                start[1] + dy * ratio,
                dx,
                dy,
            )

        remaining -= segment_length

    return (
        fallback_end[0],
        fallback_end[1],
        fallback_end[0] - fallback_start[0],
        fallback_end[1] - fallback_start[1],
    )


def _shape_length(shape):
    """Return the total length of a polyline."""
    total_length = 0.0
    for start, end in zip(shape, shape[1:]):
        total_length += math.hypot(end[0] - start[0], end[1] - start[1])
    return total_length


def _sumo_heading_from_vector(dx, dy):
    """Convert a 2D vector into a SUMO heading angle."""
    if abs(dx) <= 1e-6 and abs(dy) <= 1e-6:
        raise ValueError("Could not determine a valid heading from the selected edge.")
    return (90.0 - math.degrees(math.atan2(dy, dx))) % 360.0


def _autoware_map_xy_from_sumo_point(map_name, sumo_x, sumo_y):
    """Convert a SUMO point into the Autoware map coordinate frame."""
    offset_x, offset_y = _net_location_offset(map_name)
    return sumo_x - offset_x, sumo_y - offset_y


def _fallback_autoware_pose_from_edge(edge, map_name, edge_position="start", z_value=0.0):
    """Build an approximate Autoware pose directly from a SUMO edge shape."""
    reference_shape = edge.shape
    reference_length = _shape_length(reference_shape)
    probe_distance = min(max(reference_length * 0.02, 1.0), 4.0)
    if reference_length <= probe_distance * 2.0:
        sample_distance = max(reference_length * 0.25, 0.0)
    elif edge_position == "end":
        sample_distance = max(reference_length - probe_distance, 0.0)
    else:
        sample_distance = probe_distance

    sample_x, sample_y, dx, dy = _point_along_shape(reference_shape, sample_distance)
    fallback_yaw = math.degrees(math.atan2(dy, dx))
    map_x, map_y = _autoware_map_xy_from_sumo_point(map_name, sample_x, sample_y)

    pose = {
        "frame_id": "map",
        "x": round(map_x, 3),
        "y": round(map_y, 3),
        "z": float(z_value),
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": round(fallback_yaw, 3),
    }
    pose.update(_quaternion_from_euler_deg(0.0, 0.0, fallback_yaw))
    return pose


def _quaternion_from_euler_deg(roll_deg, pitch_deg, yaw_deg):
    """Convert Euler angles in degrees into a quaternion payload."""
    roll = math.radians(float(roll_deg))
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))

    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return {
        "qx": sr * cp * cy - cr * sp * sy,
        "qy": cr * sp * cy + sr * cp * sy,
        "qz": cr * cp * sy - sr * sp * cy,
        "qw": cr * cp * cy + sr * sp * sy,
    }


def autoware_pose_from_edge(
    edge_id,
    map_name=DEFAULT_MAP,
    host=DEFAULT_CARLA_HOST,
    port=DEFAULT_CARLA_PORT,
    pose_role="goal",
    edge_position="start",
):
    """Build an Autoware pose payload from a SUMO edge."""
    edge_id = str(edge_id).strip()
    if not edge_id:
        raise ValueError("A valid SUMO edge is required to build an Autoware pose.")

    edge = next(
        (candidate for candidate in read_sumo_edges(map_name) if candidate.edge_id == edge_id),
        None,
    )
    if edge is None:
        raise ValueError(f"SUMO edge '{edge_id}' is not present in map '{map_name}'.")

    fallback_pose = _fallback_autoware_pose_from_edge(
        edge,
        map_name,
        edge_position=edge_position,
        z_value=0.0,
    )

    return {
        "edge_id": edge.edge_id,
        "pose_role": str(pose_role),
        "carla_map": str(map_name),
        "pose": fallback_pose,
        "projection_mode": "sumo_map_frame",
        "projection_error": None,
    }


def autoware_spawn_point_from_edge(
    edge_id,
    map_name=DEFAULT_MAP,
    host=DEFAULT_CARLA_HOST,
    port=DEFAULT_CARLA_PORT,
):
    """Build a CARLA spawn-point string from a SUMO edge."""
    edge_id = str(edge_id).strip()
    if not edge_id:
        raise ValueError("A valid SUMO edge is required to build an Autoware spawn point.")

    edge = next(
        (candidate for candidate in read_sumo_edges(map_name) if candidate.edge_id == edge_id),
        None,
    )
    if edge is None:
        raise ValueError(f"SUMO edge '{edge_id}' is not present in map '{map_name}'.")

    pose_data = autoware_pose_from_edge(
        edge_id,
        map_name=map_name,
        host=host,
        port=port,
        pose_role="spawn",
        edge_position="start",
    )
    pose = dict(pose_data["pose"])
    pose["z"] = AUTOWARE_CARLA_SPAWN_Z
    spawn_point = ",".join(
        f"{float(pose[key]):.3f}"
        for key in ("x", "y", "z", "roll", "pitch", "yaw")
    )
    fallback_pose_values = _fallback_autoware_pose_from_edge(
        edge,
        map_name,
        edge_position="start",
        z_value=AUTOWARE_CARLA_SPAWN_Z,
    )
    if pose_data["projection_mode"] == "sumo_fallback":
        fallback_spawn_point = spawn_point
    else:
        fallback_spawn_point = ",".join(
            f"{float(fallback_pose_values[key]):.3f}"
            for key in ("x", "y", "z", "roll", "pitch", "yaw")
        )

    return {
        "edge_id": edge.edge_id,
        "carla_map": str(pose_data.get("carla_map", map_name)),
        "spawn_point": spawn_point,
        "fallback_spawn_point": fallback_spawn_point,
        "projection_mode": pose_data.get("projection_mode"),
        "projection_error": pose_data.get("projection_error"),
    }


def _point_segment_distance(px, py, ax, ay, bx, by):
    """Return the shortest distance between a point and a line segment."""
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    denom = abx * abx + aby * aby

    if denom == 0:
        return math.hypot(px - ax, py - ay)

    t = max(0.0, min(1.0, (apx * abx + apy * aby) / denom))
    closest_x = ax + t * abx
    closest_y = ay + t * aby
    return math.hypot(px - closest_x, py - closest_y)


def nearest_edge(edges, x, y):
    """Return the SUMO edge closest to the given map coordinates."""
    best_edge = None
    best_distance = float("inf")

    for edge in edges:
        for start, end in zip(edge.shape, edge.shape[1:]):
            distance = _point_segment_distance(x, y, start[0], start[1], end[0], end[1])
            if distance < best_distance:
                best_edge = edge
                best_distance = distance

    return best_edge, best_distance


def _indent_xml(element, level=0):
    """Indent an XML tree in place for readable output."""
    indent = "\n" + level * "  "
    child_indent = "\n" + (level + 1) * "  "

    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indent
        for child in element:
            _indent_xml(child, level + 1)
        if not element.tail or not element.tail.strip():
            element.tail = indent
    else:
        if level and (not element.tail or not element.tail.strip()):
            element.tail = indent


def _write_xml(path, root):
    """Write an XML tree to disk with stable indentation."""
    path.parent.mkdir(parents=True, exist_ok=True)
    _indent_xml(root)
    tree = ET.ElementTree(root)
    tree.write(path, encoding="UTF-8", xml_declaration=True)


def _build_env():
    """Build the subprocess environment used by SUMO and CARLA helpers."""
    env = os.environ.copy()
    resolved_sumo_home = _resolve_sumo_home(env)
    if resolved_sumo_home is not None:
        env["SUMO_HOME"] = str(resolved_sumo_home)

    api_archive = selected_carla_python_api_archive()
    if api_archive is not None:
        existing_pythonpath = [
            item for item in env.get("PYTHONPATH", "").split(os.pathsep) if item
        ]
        filtered_pythonpath = [
            item
            for item in existing_pythonpath
            if "/PythonAPI/carla/dist/" not in item and "site-packages/carla" not in item
        ]
        filtered_pythonpath.insert(0, str(api_archive))
        env["PYTHONPATH"] = os.pathsep.join(filtered_pythonpath)

    library_dirs = (
        selected_carla_runtime_library_dirs(resolve_carla_python_executable())
        if api_archive is not None
        else []
    )
    if library_dirs:
        existing_ld_library_path = [
            item for item in env.get("LD_LIBRARY_PATH", "").split(os.pathsep) if item
        ]
        for library_dir in reversed(library_dirs):
            if str(library_dir) in existing_ld_library_path:
                existing_ld_library_path.remove(str(library_dir))
            existing_ld_library_path.insert(0, str(library_dir))
        env["LD_LIBRARY_PATH"] = os.pathsep.join(existing_ld_library_path)

    return env


def _resolve_sumo_home(env=None):
    """Resolve the SUMO home directory to use for subprocesses."""
    if env is None:
        env = os.environ

    configured_sumo_home = env.get("SUMO_HOME")
    if configured_sumo_home:
        configured_path = Path(configured_sumo_home).expanduser()
        if configured_path.exists():
            return configured_path

    default_path = Path(DEFAULT_SUMO_HOME)
    if default_path.exists():
        return default_path

    if BUNDLED_SUMO_HOME.exists():
        return BUNDLED_SUMO_HOME

    return None


def _resolve_sumo_tools_dir(env=None):
    """Resolve the SUMO tools directory for the active environment."""
    sumo_home = _resolve_sumo_home(env)
    if sumo_home is not None:
        tools_dir = sumo_home / "tools"
        if tools_dir.exists():
            return tools_dir

    if SUMO_TOOLS_DIR is not None and SUMO_TOOLS_DIR.exists():
        return SUMO_TOOLS_DIR

    raise FileNotFoundError(
        "SUMO tools directory not found. Set SUMO_HOME to the SUMO root "
        "(the directory that contains 'tools' and 'bin')."
    )


def _run_command(cmd, cwd=None):
    """Run a helper command in the SUMO environment and return its captured output."""
    if cwd is None:
        cwd = SUMO_DIR

    process = subprocess.run(
        cmd,
        cwd=str(cwd),
        env=_build_env(),
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        raise RuntimeError(
            "Command failed with exit code "
            f"{process.returncode}: {' '.join(cmd)}\n{process.stderr}"
        )
    return process.stdout, process.stderr


def _depart_times(count, begin, end, pattern, rng):
    """Generate departure times for scenario traffic according to the selected pattern."""
    if count <= 0:
        return []
    if end < begin:
        end = begin

    normalized_pattern = (pattern or "").strip().lower()
    if normalized_pattern in {"all together", "tutti subito"}:
        return [float(begin)] * count
    if normalized_pattern in {"randomly", "random"}:
        return sorted(rng.uniform(begin, end) for _ in range(count))

    if count == 1 or end == begin:
        return [float(begin)] * count

    step = (end - begin) / float(count - 1)
    return [begin + index * step for index in range(count)]


def _vehicle_type_for_trip(rng, vehicle_type, random_vehicle_type, vehicle_types):
    """Choose the vehicle type to use for a generated trip."""
    if random_vehicle_type:
        choices = vehicle_types or available_vehicle_types()
        if not choices:
            raise ValueError("No SUMO vType is available for random vType.")

        return rng.choice(choices)

    return vehicle_type


def _write_congestion_trips(
    trip_file,
    edges,
    target_edge,
    vehicle_count,
    begin,
    end,
    spawn_pattern,
    destination_edge=None,
    source_edge=None,
    seed=42,
    vehicle_type=DEFAULT_VEHICLE_TYPE,
    random_vehicle_type=False,
    vehicle_types=None,
    candidate_count=None,
):
    """Write the trip file for a congestion-driven scenario."""
    rng = random.Random(seed)
    if candidate_count is None:
        candidate_count = max(vehicle_count * 4, vehicle_count + 20)
    departures = _depart_times(candidate_count, begin, end, spawn_pattern, rng)
    edge_ids = [edge.edge_id for edge in edges]
    source_pool = [
        edge_id for edge_id in edge_ids if edge_id not in {target_edge, destination_edge}
    ] or edge_ids
    destination_pool = [
        edge_id for edge_id in edge_ids if edge_id not in {target_edge, source_edge}
    ] or edge_ids

    ET.register_namespace("xsi", XSI_NS)
    root = ET.Element(
        "routes",
        {f"{{{XSI_NS}}}noNamespaceSchemaLocation": "http://sumo.dlr.de/xsd/routes_file.xsd"},
    )

    for index, depart in enumerate(departures):
        origin = source_edge or rng.choice(source_pool)
        destination = destination_edge or rng.choice(
            [edge_id for edge_id in destination_pool if edge_id != origin] or destination_pool
        )
        trip_vehicle_type = _vehicle_type_for_trip(
            rng,
            vehicle_type,
            random_vehicle_type,
            vehicle_types,
        )
        trip = ET.SubElement(
            root,
            "trip",
            {
                "id": f"congestion_{index}",
                "depart": f"{depart:.2f}",
                "from": origin,
                "to": destination,
                "type": trip_vehicle_type,
            },
        )
        if target_edge != origin and target_edge != destination:
            trip.set("via", target_edge)

    _write_xml(trip_file, root)


def _write_empty_routes_file(path):
    """Write a valid SUMO routes file with no vehicles."""
    ET.register_namespace("xsi", XSI_NS)
    root = ET.Element(
        "routes",
        {f"{{{XSI_NS}}}noNamespaceSchemaLocation": "http://sumo.dlr.de/xsd/routes_file.xsd"},
    )
    _write_xml(path, root)


def _count_route_vehicles(route_file, target_edge=None):
    """Count vehicles stored in a route file."""
    if not route_file.exists():
        return 0, 0

    root = ET.parse(route_file).getroot()
    vehicles = root.findall("vehicle")
    target_count = 0

    if target_edge:
        for vehicle in vehicles:
            route = vehicle.find("route")
            if route is None:
                continue
            if target_edge in (route.get("edges") or "").split():
                target_count += 1

    return len(vehicles), target_count


def _trim_route_file(route_file, max_vehicles):
    """Trim a route file down to the earliest `max_vehicles` departures."""
    tree = ET.parse(route_file)
    root = tree.getroot()
    vehicles = root.findall("vehicle")

    def depart_time(vehicle):
        """Extract a sortable departure time from a route entry."""
        try:
            return float(vehicle.get("depart", "0"))
        except ValueError:
            return 0.0

    keep = set(sorted(vehicles, key=depart_time)[:max_vehicles])
    for vehicle in vehicles:
        if vehicle not in keep:
            root.remove(vehicle)

    _write_xml(route_file, root)


def _filter_route_file_by_target(route_file, target_edge, max_vehicles=None):
    """Keep only the routes that traverse a target edge."""
    tree = ET.parse(route_file)
    root = tree.getroot()
    vehicles = root.findall("vehicle")
    kept = []

    def depart_time(vehicle):
        """Extract a sortable departure time from a route entry."""
        try:
            return float(vehicle.get("depart", "0"))
        except ValueError:
            return 0.0

    for vehicle in vehicles:
        route = vehicle.find("route")
        route_edges = (route.get("edges") or "").split() if route is not None else []
        if target_edge in route_edges:
            kept.append(vehicle)

    kept = sorted(kept, key=depart_time)
    if max_vehicles is not None:
        kept = kept[:max_vehicles]

    kept_ids = {id(vehicle) for vehicle in kept}
    for vehicle in vehicles:
        if id(vehicle) not in kept_ids:
            root.remove(vehicle)

    _write_xml(route_file, root)


def _assign_random_vehicle_types(route_file, seed=42, vehicle_types=None):
    """Assign vehicle types for the active workflow."""
    choices = vehicle_types or available_vehicle_types()
    if not choices:
        raise ValueError("No SUMO vType is available for random vType.")

    rng = random.Random(seed)
    tree = ET.parse(route_file)
    root = tree.getroot()

    for vehicle in root.findall("vehicle"):
        vehicle.set("type", rng.choice(choices))

    _write_xml(route_file, root)


def _write_sumocfg(map_name, route_file, sumocfg_file, simulation_end=None):
    """Write the SUMO configuration file for a generated scenario."""
    ET.register_namespace("xsi", XSI_NS)
    root = ET.Element(
        "configuration",
        {f"{{{XSI_NS}}}noNamespaceSchemaLocation": "http://sumo.dlr.de/xsd/sumoConfiguration.xsd"},
    )

    input_element = ET.SubElement(root, "input")
    ET.SubElement(input_element, "net-file", {"value": f"net/{map_name}.net.xml"})
    ET.SubElement(
        input_element,
        "route-files",
        {
            "value": (
                "carlavtypes.rou.xml, "
                "egovtype.xml, "
                f"{relative_to_examples(route_file)}"
            )
        },
    )

    processing = ET.SubElement(root, "processing")
    ET.SubElement(processing, "time-to-teleport", {"value": "-1"})

    gui_only = ET.SubElement(root, "gui_only")
    ET.SubElement(gui_only, "gui-settings-file", {"value": "viewsettings.xml"})

    if simulation_end is not None:
        time_element = ET.SubElement(root, "time")
        ET.SubElement(time_element, "begin", {"value": "0"})
        ET.SubElement(time_element, "end", {"value": str(float(simulation_end))})

    output = ET.SubElement(root, "output")
    ET.SubElement(output, "battery-output", {"value": "output/battery.out.xml"})
    ET.SubElement(output, "tripinfo-output", {"value": "output/tripinfos.xml"})
    ET.SubElement(output, "vehroute-output", {"value": "output/vehroute.xml"})
    ET.SubElement(output, "summary-output", {"value": "output/summary.xml"})
    ET.SubElement(output, "edgedata-output", {"value": "output/edgedata-output.xml"})
    ET.SubElement(output, "emission-output", {"value": "output/emission-output.xml"})

    _write_xml(sumocfg_file, root)


def generate_congestion_scenario(
    map_name=DEFAULT_MAP,
    target_edge=None,
    destination_edge=None,
    vehicle_count=100,
    begin=0,
    end=120,
    simulation_end=None,
    spawn_pattern="Equidistant",
    source_edge=None,
    seed=42,
    vehicle_type=DEFAULT_VEHICLE_TYPE,
    random_vehicle_type=False,
    vehicle_types=None,
):
    """Generate a congestion-focused SUMO scenario and its artifacts."""
    vehicle_count = int(vehicle_count)
    if vehicle_count < 0:
        raise ValueError("The number of vehicles cannot be negative.")

    if not target_edge and vehicle_count > 0:
        raise ValueError("Select an edge to congest.")

    edges = read_sumo_edges(map_name)
    edge_ids = {edge.edge_id for edge in edges}
    for label, edge_id in {
        "edge da congestionare": target_edge,
        "destination edge": destination_edge,
        "edge sorgente": source_edge,
    }.items():
        if edge_id and edge_id not in edge_ids:
            raise ValueError(f"{label} not present in the map: {edge_id}")

    trip_file = map_trip_file(map_name)
    route_file = map_route_file(map_name)
    sumocfg_file = map_sumocfg_file(map_name)
    simulation_end = max(
        float(end),
        float(simulation_end) if simulation_end is not None else float(end),
    )

    if vehicle_count == 0:
        _write_empty_routes_file(trip_file)
        _write_empty_routes_file(route_file)
        _write_sumocfg(map_name, route_file, sumocfg_file, simulation_end=simulation_end)
        return ScenarioResult(
            map_name=map_name,
            target_edge=target_edge or "",
            route_file=route_file,
            trip_file=trip_file,
            sumocfg_file=sumocfg_file,
            command=build_run_command(sumocfg_file),
            generated_count=0,
            requested_count=0,
            target_count=0,
            mode="empty traffic",
            stdout="",
            stderr="",
            spawn_begin=float(begin),
            spawn_end=float(end),
            simulation_end=simulation_end,
        )

    duarouter = shutil.which("duarouter") or "duarouter"
    stdout_parts = []
    stderr_parts = []
    generated_count = 0
    target_count = 0

    for attempt, multiplier in enumerate((4, 8, 16, 32), start=1):
        candidate_count = max(vehicle_count * multiplier, vehicle_count + 20)
        _write_congestion_trips(
            trip_file=trip_file,
            edges=edges,
            target_edge=target_edge,
            vehicle_count=vehicle_count,
            begin=float(begin),
            end=float(end),
            spawn_pattern=spawn_pattern,
            destination_edge=destination_edge or None,
            source_edge=source_edge or None,
            seed=int(seed) + attempt - 1,
            vehicle_type=vehicle_type,
            random_vehicle_type=random_vehicle_type,
            vehicle_types=vehicle_types,
            candidate_count=candidate_count,
        )

        stdout, stderr = _run_command(
            [
                duarouter,
                "-n",
                str(map_net_file(map_name)),
                "-r",
                str(trip_file),
                "-o",
                str(route_file),
                "--ignore-errors",
                "--no-step-log",
                "--no-warnings",
            ]
        )
        stdout_parts.append(stdout)
        stderr_parts.append(stderr)

        _filter_route_file_by_target(route_file, target_edge, vehicle_count)
        generated_count, target_count = _count_route_vehicles(route_file, target_edge)
        if generated_count >= vehicle_count:
            break

    _write_sumocfg(map_name, route_file, sumocfg_file, simulation_end=simulation_end)

    return ScenarioResult(
        map_name=map_name,
        target_edge=target_edge,
        route_file=route_file,
        trip_file=trip_file,
        sumocfg_file=sumocfg_file,
        command=build_run_command(sumocfg_file),
        generated_count=generated_count,
        requested_count=vehicle_count,
        target_count=target_count,
        mode="duarouter via edge",
        stdout="\n".join(stdout_parts),
        stderr="\n".join(stderr_parts),
        spawn_begin=float(begin),
        spawn_end=float(end),
        simulation_end=simulation_end,
    )


def generate_random_trips_scenario(
    map_name=DEFAULT_MAP,
    vehicle_count=100,
    begin=0,
    end=120,
    simulation_end=None,
    seed=42,
    vehicle_type=DEFAULT_VEHICLE_TYPE,
    random_vehicle_type=False,
    vehicle_types=None,
):
    """Generate a random-traffic SUMO scenario and its artifacts."""
    vehicle_count = int(vehicle_count)
    if vehicle_count < 0:
        raise ValueError("The number of vehicles cannot be negative.")

    trip_file = map_trip_file(map_name)
    route_file = map_route_file(map_name)
    sumocfg_file = map_sumocfg_file(map_name)
    duration = max(float(end) - float(begin), 1.0)
    simulation_end = max(
        float(end),
        float(simulation_end) if simulation_end is not None else float(end),
    )

    if vehicle_count == 0:
        _write_empty_routes_file(trip_file)
        _write_empty_routes_file(route_file)
        _write_sumocfg(map_name, route_file, sumocfg_file, simulation_end=simulation_end)
        return ScenarioResult(
            map_name=map_name,
            target_edge="",
            route_file=route_file,
            trip_file=trip_file,
            sumocfg_file=sumocfg_file,
            command=build_run_command(sumocfg_file),
            generated_count=0,
            requested_count=0,
            target_count=0,
            mode="empty traffic",
            stdout="",
            stderr="",
            spawn_begin=float(begin),
            spawn_end=float(end),
            simulation_end=simulation_end,
        )

    period = max(duration / float(vehicle_count), 0.01)
    random_trips = _resolve_sumo_tools_dir() / "randomTrips.py"

    stdout, stderr = _run_command(
        [
            sys.executable,
            str(random_trips),
            "-n",
            str(map_net_file(map_name)),
            "-o",
            str(trip_file),
            "-r",
            str(route_file),
            "-b",
            str(float(begin)),
            "-e",
            str(float(end)),
            "-p",
            f"{period:.6f}",
            "--seed",
            str(int(seed)),
            "--edge-permission",
            "passenger",
            "--validate",
            "--trip-attributes",
            f'type="{vehicle_type}"',
            "--prefix",
            "traffic_",
        ]
    )

    generated_count, _ = _count_route_vehicles(route_file)
    if generated_count > vehicle_count:
        _trim_route_file(route_file, vehicle_count)
        generated_count, _ = _count_route_vehicles(route_file)

    if random_vehicle_type:
        _assign_random_vehicle_types(route_file, int(seed), vehicle_types)

    _write_sumocfg(map_name, route_file, sumocfg_file, simulation_end=simulation_end)

    return ScenarioResult(
        map_name=map_name,
        target_edge="",
        route_file=route_file,
        trip_file=trip_file,
        sumocfg_file=sumocfg_file,
        command=build_run_command(sumocfg_file),
        generated_count=generated_count,
        requested_count=vehicle_count,
        target_count=0,
        mode="randomTrips",
        stdout=stdout,
        stderr=stderr,
        spawn_begin=float(begin),
        spawn_end=float(end),
        simulation_end=simulation_end,
    )


def build_run_command(sumocfg_file, sumo_gui=True, wait_start_file=None):
    """Build the command used to launch the synchronization runner."""
    python_executable = resolve_carla_python_executable()
    command = [
        str(python_executable),
        str(PROJECT_ROOT / "ecodrive" / "cosimulation" / "run_dashboard_synchronization.py"),
        "--carla-version",
        active_carla_version(),
        relative_to_sumo_dir(sumocfg_file),
    ]
    if sumo_gui:
        command.append("--sumo-gui")
    if wait_start_file is not None:
        command.extend(["--wait-start-file", str(wait_start_file)])
    return command


def relative_to_sumo_dir(path):
    """Return the relative a path relative to the active SUMO co-simulation directory."""
    return path.relative_to(SUMO_DIR).as_posix()


def is_carla_server_ready(host=DEFAULT_CARLA_HOST, port=DEFAULT_CARLA_PORT, timeout=1.0):
    """Handle whether the CARLA server is reachable."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _running_carla_process_entries():
    """Handle running CARLA processes visible on the host."""
    if psutil is None:
        return []

    installations = {
        version: path.resolve()
        for version, path in _discover_carla_installations().items()
    }
    entries = []

    for process in psutil.process_iter(
        ["pid", "ppid", "name", "cmdline", "cwd", "exe", "status"]
    ):
        try:
            info = process.info
            name = info.get("name") or ""
            cmdline_parts = info.get("cmdline") or []
            cmdline = " ".join(cmdline_parts)
            if "CarlaUE4" not in name and "CarlaUE4" not in cmdline:
                continue
            cwd = info.get("cwd") or ""
            exe = info.get("exe") or ""
        except (psutil.Error, OSError):
            continue

        matched_version = None
        matched_dir = None
        for version, carla_dir in installations.items():
            carla_dir_text = str(carla_dir)
            shipping_binary = str(
                carla_dir / "CarlaUE4" / "Binaries" / "Linux" / "CarlaUE4-Linux-Shipping"
            )
            if (
                cwd == carla_dir_text
                or cwd.startswith(f"{carla_dir_text}{os.sep}")
                or shipping_binary in cmdline
                or shipping_binary == exe
                or carla_dir_text in cmdline
            ):
                matched_version = version
                matched_dir = carla_dir_text
                break

        entries.append(
            {
                "pid": int(info["pid"]),
                "ppid": int(info.get("ppid") or 0),
                "name": name,
                "cmdline": cmdline,
                "cwd": cwd,
                "exe": exe,
                "status": info.get("status") or "",
                "version": matched_version,
                "carla_dir": matched_dir,
            }
        )

    return entries


def carla_server_status(version=None):
    """Handle the current CARLA server status."""
    selected_version = _normalize_carla_version(version or active_carla_version())
    processes = _running_carla_process_entries()
    matched_processes = [item for item in processes if item.get("version")]
    selected_processes = [
        item for item in matched_processes if item["version"] == selected_version
    ]
    running_versions = sorted({item["version"] for item in matched_processes})
    ready = is_carla_server_ready()
    detected_version = None

    if selected_processes:
        detected_version = selected_version
    elif len(running_versions) == 1:
        detected_version = running_versions[0]

    return {
        "selected_version": selected_version,
        "running": ready or bool(processes),
        "ready": ready,
        "external": ready and not matched_processes,
        "detected_version": detected_version,
        "running_versions": running_versions,
        "processes": processes,
        "matched_processes": matched_processes,
        "selected_processes": selected_processes,
    }


def _infer_map_name_from_sumocfg_path(sumocfg_path):
    """Infer the map name encoded in a `.sumocfg` path."""
    if not sumocfg_path:
        return None

    file_name = Path(str(sumocfg_path)).name
    if not (file_name.startswith("custom_") and file_name.endswith(".sumocfg")):
        return None

    candidate = file_name[len("custom_") : -len(".sumocfg")]
    return candidate if candidate in available_maps() else None


def dashboard_synchronization_status():
    """Handle the status of the dashboard synchronization process."""
    if psutil is None:
        return {
            "running": False,
            "map_name": None,
            "ambiguous": False,
            "processes": [],
        }

    entries = []
    for process in psutil.process_iter(["pid", "name", "cmdline", "status"]):
        try:
            cmdline_parts = [str(part) for part in (process.info.get("cmdline") or [])]
            joined_cmdline = " ".join(cmdline_parts)
            if "run_dashboard_synchronization.py" not in joined_cmdline:
                continue
        except (psutil.Error, OSError):
            continue

        sumocfg_file = next(
            (part for part in cmdline_parts if str(part).endswith(".sumocfg")),
            "",
        )
        entries.append(
            {
                "pid": int(process.info["pid"]),
                "name": process.info.get("name") or "",
                "status": process.info.get("status") or "",
                "cmdline": cmdline_parts,
                "sumocfg_file": sumocfg_file,
                "map_name": _infer_map_name_from_sumocfg_path(sumocfg_file),
            }
        )

    unique_maps = sorted({entry["map_name"] for entry in entries if entry["map_name"]})
    return {
        "running": bool(entries),
        "map_name": unique_maps[0] if len(unique_maps) == 1 else None,
        "ambiguous": len(unique_maps) > 1,
        "processes": entries,
    }


def start_carla(log_file=None):
    """Start CARLA using the active repository installation."""
    if log_file is None:
        log_file = OUTPUT_DIR / "carla_server.log"
    if not CARLA_SCRIPT.exists():
        raise FileNotFoundError(f"CARLA launcher not found: {CARLA_SCRIPT}")

    log_file.parent.mkdir(parents=True, exist_ok=True)
    with open(log_file, "a", encoding="utf-8") as log_handle:
        log_handle.write(f"\n\n=== CarlaUE4 {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        log_handle.flush()
        process = subprocess.Popen(
            ["./CarlaUE4.sh"],
            cwd=str(CARLA_DIR),
            env=_build_env(),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
        )

    return process, log_file


def start_carla_server(version=None, timeout=120, log_file=None, map_name=None):
    """Start the CARLA server and wait until it is ready."""
    selected_version = set_active_carla_version(version)
    status = carla_server_status(selected_version)
    if status["running"]:
        running_version = status.get("detected_version")
        if running_version:
            raise RuntimeError(
                f"CARLA {running_version} is already running on "
                f"{DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}."
            )
        raise RuntimeError(
            f"CARLA is already running on {DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}."
        )

    process, resolved_log_file = start_carla(log_file=log_file)
    wait_for_carla_server(process=process, timeout=timeout)
    if map_name:
        load_carla_map(map_name)
    return {
        "version": selected_version,
        "process": process,
        "log_file": resolved_log_file,
        "map_name": map_name,
    }


def stop_carla_server(version=None, timeout=15.0):
    """Stop the CARLA server and wait until it is ready."""
    if psutil is None:
        raise RuntimeError("psutil is required to stop CARLA from the dashboard.")

    status = carla_server_status(version)
    selected_version = (
        _normalize_carla_version(version)
        if version is not None
        else status.get("detected_version")
    )

    if version is None:
        target_entries = status["matched_processes"]
    else:
        target_entries = status["selected_processes"]

    if not target_entries:
        if status["external"]:
            raise RuntimeError(
                "CARLA is listening on port 2000, but the process is not tied to a supported "
                "local installation. Stop it manually."
            )
        if status["running"]:
            raise RuntimeError(
                "CARLA appears to be running, but no matching local CARLA process was found."
            )
        return {
            "version": selected_version,
            "stopped_pids": [],
            "port_closed": True,
        }

    processes = []
    stopped_pids = []
    for pid in sorted({item["pid"] for item in target_entries}, reverse=True):
        try:
            process = psutil.Process(pid)
        except (psutil.Error, OSError):
            continue
        processes.append(process)
        stopped_pids.append(pid)

    for process in processes:
        try:
            process.terminate()
        except (psutil.Error, OSError):
            continue

    _, alive = psutil.wait_procs(processes, timeout=max(timeout / 2.0, 1.0))
    for process in alive:
        try:
            process.kill()
        except (psutil.Error, OSError):
            continue
    psutil.wait_procs(alive, timeout=max(timeout / 2.0, 1.0))

    deadline = time.time() + timeout
    while time.time() < deadline:
        if not is_carla_server_ready():
            break
        time.sleep(0.5)

    return {
        "version": selected_version,
        "stopped_pids": stopped_pids,
        "port_closed": not is_carla_server_ready(),
    }


def wait_for_carla_server(
    process=None,
    host=DEFAULT_CARLA_HOST,
    port=DEFAULT_CARLA_PORT,
    timeout=120,
):
    """Wait for the CARLA server and wait until it is ready."""
    deadline = time.time() + timeout

    while time.time() < deadline:
        if is_carla_server_ready(host, port):
            time.sleep(3)
            return

        if process is not None and process.poll() is not None:
            raise RuntimeError(
                "CARLA closed before opening port "
                f"{host}:{port}. Exit code: {process.returncode}"
            )

        time.sleep(1)

    raise RuntimeError(
        "Timed out waiting for CARLA on port "
        f"{host}:{port} after {timeout} seconds."
    )


def load_carla_map(map_name, log_file=None):
    """Load the selected CARLA map on the running server."""
    if log_file is None:
        log_file = OUTPUT_DIR / "carla_map.log"
    if not CARLA_CONFIG_SCRIPT.exists():
        raise FileNotFoundError(f"CARLA map config not found: {CARLA_CONFIG_SCRIPT}")

    python_executable = resolve_carla_python_executable()
    cmd = [str(python_executable), "PythonAPI/util/config.py", "--map", map_name]
    process = subprocess.run(
        cmd,
        cwd=str(CARLA_DIR),
        env=_build_env(),
        capture_output=True,
        text=True,
    )

    log_file.parent.mkdir(parents=True, exist_ok=True)
    with open(log_file, "a", encoding="utf-8") as log_handle:
        log_handle.write(f"\n\n=== load map {map_name} {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        log_handle.write(" ".join(cmd) + "\n")
        if process.stdout:
            log_handle.write(process.stdout)
        if process.stderr:
            log_handle.write(process.stderr)

    if process.returncode != 0:
        raise RuntimeError(
            "CARLA map loading failed with exit code "
            f"{process.returncode}: {' '.join(cmd)}\n"
            f"{process.stderr or process.stdout}"
        )

    return process.stdout, process.stderr


def prepare_carla(map_name, carla_process=None, timeout=120, load_map=True):
    """Prepare CARLA using the active repository installation."""
    ensure_carla_python_api_ready()

    process = carla_process
    carla_started = False
    carla_log_file = OUTPUT_DIR / "carla_server.log"

    if process is not None and process.poll() is not None:
        process = None

    if not is_carla_server_ready():
        if process is None:
            process, carla_log_file = start_carla()
            carla_started = True
        wait_for_carla_server(process=process, timeout=timeout)

    map_stdout = ""
    map_stderr = ""
    if load_map:
        map_stdout, map_stderr = load_carla_map(map_name)
    return process, carla_log_file, carla_started, map_stdout, map_stderr


def start_synchronization(
    sumocfg_file,
    map_name=DEFAULT_MAP,
    ensure_carla=True,
    carla_process=None,
    carla_timeout=120,
    sumo_gui=True,
    wait_for_start=False,
    load_map=True,
    log_file=None,
):
    """Start the SUMO-CARLA synchronization runner."""
    carla_log_file = OUTPUT_DIR / "carla_server.log"
    carla_started = False
    map_loaded = False
    map_stdout = ""
    map_stderr = ""

    ensure_carla_runner_dependencies_ready()

    if ensure_carla:
        (
            carla_process,
            carla_log_file,
            carla_started,
            map_stdout,
            map_stderr,
        ) = prepare_carla(
            map_name,
            carla_process=carla_process,
            timeout=carla_timeout,
            load_map=load_map,
        )
        map_loaded = bool(load_map)
    else:
        if not is_carla_server_ready():
            raise RuntimeError(
                f"CARLA is not reachable on {DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}."
            )
        if load_map:
            map_stdout, map_stderr = load_carla_map(map_name)
            map_loaded = True

    if log_file is None:
        log_file = OUTPUT_DIR / "run_dashboard_synchronization.log"
    start_gate_file = None
    if wait_for_start:
        start_gate_file = OUTPUT_DIR / "run_dashboard_synchronization.start"
        try:
            start_gate_file.unlink(missing_ok=True)
        except OSError:
            pass

    log_file.parent.mkdir(parents=True, exist_ok=True)
    with open(log_file, "a", encoding="utf-8") as log_handle:
        log_handle.write(
            f"\n\n=== run_dashboard_synchronization {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n"
        )
        command = build_run_command(
            sumocfg_file,
            sumo_gui=sumo_gui,
            wait_start_file=start_gate_file,
        )
        log_handle.write(" ".join(command) + "\n")
        log_handle.flush()
        sync_process = subprocess.Popen(
            command,
            cwd=str(SUMO_DIR),
            env=_build_env(),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
        )

    return SynchronizationLaunch(
        sync_process=sync_process,
        carla_process=carla_process,
        sync_log_file=log_file,
        carla_log_file=carla_log_file,
        carla_started=carla_started,
        map_loaded=map_loaded,
        map_stdout=map_stdout,
        map_stderr=map_stderr,
        start_gate_file=start_gate_file,
    )
