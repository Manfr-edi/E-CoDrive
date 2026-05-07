from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _detect_carla_folder():
    """Detect the default CARLA installation folder used by the legacy helpers."""
    candidates = (
        PROJECT_ROOT / "carla" / "CARLA_0.9.15",
        PROJECT_ROOT / "carla" / "CARLA_0.9.13",
        PROJECT_ROOT / "carla",
        PROJECT_ROOT / "CARLA_0.9.15",
        PROJECT_ROOT / "CARLA_0.9.13",
    )
    for candidate in candidates:
        if (candidate / "CarlaUE4.sh").exists():
            return candidate
    return PROJECT_ROOT / "carla" / "CARLA_0.9.13"


CARLA_FOLDER = str(_detect_carla_folder())
SUMO_BINARY = "/usr/share/sumo/sumo-gui"  # oppure sumo
SUMO_CONFIG = str(Path(CARLA_FOLDER) / "Co-Simulation" / "Sumo" / "examples" / "custom_Town04.sumocfg")

EGO_ID = "ego_vehicle"
EGO_TYPE = "test_ev"
