"""Programmatic CARLA 0.9.13 Autoware simulation runner.

The public entry point is :func:`simulate`, which mirrors the dashboard
workflow without opening the CARLA window or SUMO GUI.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json
import math
import os
import re
import shutil
import signal
import subprocess
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

import pandas as pd

try:
    import psutil
except ImportError:  # pragma: no cover - optional process cleanup dependency.
    psutil = None

from ecodrive.analysis.battery_plots import generate_battery_plots, load_energy_data
from ecodrive.scenario import sumo_route_tools as route_tools


CARLA_VERSION = "0.9.13"
DEFAULT_AUTOWARE_STARTUP_WAIT = 10.0
DEFAULT_AUTOWARE_SPEED_LIMIT_KMH = 50.0
DEFAULT_EXTRA_SIMULATION_TIME = 300.0
DEFAULT_DASHBOARD_API_URL = "http://127.0.0.1:5000"
OUTPUT_XML_FILES = {
    "battery": "battery.out.xml",
    "emission": "emission-output.xml",
    "tripinfo": "tripinfos.xml",
    "vehroute": "vehroute.xml",
    "summary": "summary.xml",
    "edgedata": "edgedata-output.xml",
}


@dataclass(frozen=True)
class SimulationResult:
    """Artifacts and parsed outputs produced by :func:`simulate`."""

    town: str
    carla_version: str
    traffic: Dict[str, Any]
    ego: Dict[str, Any]
    artifacts: Dict[str, str]
    output_paths: Dict[str, str]
    plot_paths: List[str]
    energy_data: pd.DataFrame
    energy_records: List[Dict[str, Any]]
    tripinfos: List[Dict[str, str]]
    ego_tripinfo: Optional[Dict[str, str]]
    summary_records: List[Dict[str, str]]
    sync_returncode: Optional[int]
    completion_reason: str
    progress_log_path: str


@dataclass(frozen=True)
class AutomatedSynchronizationLaunch:
    """Describe the isolated runner used by :func:`simulate`."""

    sync_process: subprocess.Popen
    carla_process: Optional[subprocess.Popen]
    sync_log_file: Path
    carla_log_file: Path
    start_gate_file: Path
    ready_file: Path


class _ProgressLogger:
    """Append a compact timeline for automated simulation runs."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(
            f"=== automated simulation {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n",
            encoding="utf-8",
        )

    def log(self, stage: str, message: str) -> None:
        line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} [{stage}] {message}\n"
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(line)


def simulate(
    *,
    town: str,
    traffic_congestion_edge: Optional[str],
    traffic_source_edge: Optional[str] = None,
    traffic_destination_edge: Optional[str] = None,
    traffic_vehicle_count: int = 10,
    traffic_spawn_time: float = 0.0,
    traffic_stop_spawn_time: float = 120.0,
    traffic_vehicle_type: Optional[str] = None,
    traffic_random_vehicle_type: bool = False,
    ego_source_edge: str,
    ego_destination_edge: str,
    ego_energy_model: str = route_tools.ENERGY_EMISSION_CLASS,
    ego_model_parameters: Optional[Dict[str, Any]] = None,
    ego_model_attributes: Optional[Dict[str, Any]] = None,
    ego_max_battery_capacity: float = route_tools.DEFAULT_EGO_BATTERY_CAPACITY,
    ego_current_battery_charge: Optional[float] = None,
    ego_critical_battery_threshold: float = 0.0,
    simulation_end: Optional[float] = None,
    traffic_seed: int = 42,
    traffic_spawn_pattern: str = "Equidistant",
    autoware_startup_wait: float = DEFAULT_AUTOWARE_STARTUP_WAIT,
    autoware_speed_limit_kmh: Optional[float] = DEFAULT_AUTOWARE_SPEED_LIMIT_KMH,
    carla_timeout: float = 180.0,
    autoware_spawn_timeout: float = 60.0,
    autoware_carla_rpc_timeout: Optional[float] = None,
    autoware_sumo_mirror_timeout: float = 60.0,
    autoware_route_timeout: float = 75.0,
    wall_timeout: Optional[float] = None,
    stop_on_ego_arrival: bool = True,
    generate_plots: bool = True,
    cleanup_existing: bool = True,
    carla_extra_args: Optional[Sequence[str]] = None,
    progress_log_file: Optional[Path] = None,
) -> SimulationResult:
    """Run the complete headless dashboard workflow for CARLA 0.9.13.

    Parameters are intentionally close to the dashboard fields. ``ego_model_parameters``
    may contain both SUMO vType parameters and dashboard vType attributes; keys that
    match the selected model attributes are routed to the XML attributes automatically.
    """

    _activate_carla()
    output_dir = route_tools.OUTPUT_DIR
    progress_log = _ProgressLogger(
        Path(progress_log_file)
        if progress_log_file is not None
        else output_dir / "automated_simulation_progress.log"
    )
    progress_log.log("start", f"simulate(town={town}, carla_version={CARLA_VERSION})")

    _validate_town_and_edges(
        town,
        traffic_congestion_edge,
        traffic_source_edge,
        traffic_destination_edge,
        ego_source_edge,
        ego_destination_edge,
    )
    _validate_battery(
        ego_max_battery_capacity,
        ego_current_battery_charge,
        ego_critical_battery_threshold,
    )
    progress_log.log("inputs", "Town, edge and battery inputs validated.")

    if cleanup_existing:
        progress_log.log("cleanup", "Stopping previous CARLA, sync and Autoware runtime state.")
        _cleanup_existing_runtime(CARLA_VERSION, progress_log=progress_log)

    _clear_previous_outputs(output_dir)
    progress_log.log("outputs", f"Cleared previous SUMO XML outputs in {output_dir}.")

    energy_model = _normalize_energy_model(ego_energy_model)
    ego_attributes, ego_parameters = _ego_payload(
        energy_model,
        ego_model_attributes,
        ego_model_parameters,
    )
    battery_charge = (
        float(ego_current_battery_charge)
        if ego_current_battery_charge is not None
        else float(ego_max_battery_capacity)
    )
    ego_parameters["dashboard.battery.failureThreshold"] = str(
        float(ego_critical_battery_threshold)
    )
    route_tools.write_autoware_ego_vtype_config(
        energy_model,
        float(ego_max_battery_capacity),
        battery_charge,
        attributes=ego_attributes,
        parameters=ego_parameters,
    )
    progress_log.log(
        "ego-config",
        (
            f"Saved Autoware ego vType model={energy_model}, "
            f"capacity={float(ego_max_battery_capacity):.1f}Wh, "
            f"charge={battery_charge:.1f}Wh."
        ),
    )

    vehicle_types = route_tools.available_vehicle_types()
    random_vehicle_type = _as_random_vehicle_type(
        traffic_vehicle_type,
        traffic_random_vehicle_type,
    )
    selected_vehicle_type = (
        route_tools.DEFAULT_VEHICLE_TYPE
        if random_vehicle_type or not traffic_vehicle_type
        else str(traffic_vehicle_type)
    )
    resolved_simulation_end = _simulation_end(
        traffic_stop_spawn_time,
        simulation_end,
    )
    scenario = route_tools.generate_congestion_scenario(
        map_name=town,
        target_edge=traffic_congestion_edge,
        destination_edge=traffic_destination_edge,
        vehicle_count=int(traffic_vehicle_count),
        begin=float(traffic_spawn_time),
        end=float(traffic_stop_spawn_time),
        simulation_end=resolved_simulation_end,
        spawn_pattern=traffic_spawn_pattern,
        source_edge=traffic_source_edge,
        seed=int(traffic_seed),
        vehicle_type=selected_vehicle_type,
        random_vehicle_type=random_vehicle_type,
        vehicle_types=vehicle_types,
    )
    progress_log.log(
        "scenario",
        (
            f"Generated route scenario {scenario.generated_count}/"
            f"{scenario.requested_count} vehicles; sumocfg={scenario.sumocfg_file}."
        ),
    )

    carla_process = None
    sync_launch = None
    autoware_container = None
    autoware_spawn = None
    autoware_route_start = None
    completion_reason = "unknown"

    try:
        carla_process = _start_carla(
            town,
            timeout=float(carla_timeout),
            extra_args=carla_extra_args,
            progress_log=progress_log,
        )
        sync_launch = _start_automated_synchronization(
            scenario.sumocfg_file,
            carla_process=carla_process,
            carla_timeout=float(carla_timeout),
            sumo_gui=False,
            output_dir=output_dir,
            progress_log=progress_log,
        )
        _wait_for_sync_ready(sync_launch, timeout=float(carla_timeout))
        progress_log.log(
            "sync",
            (
                "SUMO/TraCI is initialized and gated; simulation time has not "
                f"advanced. ready_file={getattr(sync_launch, 'ready_file', None)}"
            ),
        )

        progress_log.log("autoware", "Launching Autoware CARLA stack.")
        autoware_launch = route_tools.launch_autoware_carla_in_container(
            town,
            spawn_edge=ego_source_edge,
            start_edge=ego_source_edge,
            goal_edge=ego_destination_edge,
            speed_limit_kmh=autoware_speed_limit_kmh,
            carla_bridge_passive=False,
            publish_route=False,
        )
        autoware_container = autoware_launch.get("container_name")
        progress_log.log(
            "autoware",
            f"Autoware launch requested in container {autoware_container}.",
        )

        if autoware_startup_wait > 0:
            progress_log.log(
                "autoware",
                f"Waiting {float(autoware_startup_wait):.1f}s before checking ego actor.",
            )
            time.sleep(float(autoware_startup_wait))
        autoware_spawn = _wait_for_autoware_spawn(
            timeout=float(autoware_spawn_timeout),
            carla_rpc_timeout=_resolve_autoware_carla_rpc_timeout(
                explicit_timeout=autoware_carla_rpc_timeout,
                carla_timeout=float(carla_timeout),
                spawn_timeout=float(autoware_spawn_timeout),
            ),
            progress_log=progress_log,
        )

        autoware_route_start = start_sumo_and_publish_autoware_route(
            sync_launch=sync_launch,
            town=town,
            container_name=autoware_container,
            start_edge=ego_source_edge,
            goal_edge=ego_destination_edge,
            speed_limit_kmh=autoware_speed_limit_kmh,
            mirror_timeout=float(autoware_sumo_mirror_timeout),
            route_timeout=float(autoware_route_timeout),
            progress_log=progress_log,
        )

        progress_log.log("run", "Waiting for completion outputs.")
        completion_reason = _wait_for_completion(
            sync_launch.sync_process,
            output_dir,
            simulation_end=resolved_simulation_end,
            wall_timeout=wall_timeout,
            stop_on_ego_arrival=stop_on_ego_arrival,
            progress_log=progress_log,
        )
        progress_log.log("run", f"Completion reason: {completion_reason}.")
    finally:
        progress_log.log("cleanup", "Stopping synchronization, Autoware and CARLA.")
        if sync_launch is not None:
            _stop_process(sync_launch.sync_process, interrupt=True)
            _safe_unlink(sync_launch.start_gate_file)
            _safe_unlink(getattr(sync_launch, "ready_file", None))
        if autoware_container:
            _stop_autoware_processes(autoware_container)
        _stop_carla(carla_process, CARLA_VERSION)
        progress_log.log("cleanup", "Runtime cleanup requested.")

    output_paths = _output_paths(output_dir)
    energy_data = _load_energy_output(
        output_paths,
        prefer_mmpevem=energy_model == route_tools.MMPEVEM_EMISSION_CLASS,
    )
    progress_log.log("outputs", f"Loaded {len(energy_data)} energy records.")
    tripinfos = _read_tripinfos(Path(output_paths["tripinfo"]))
    ego_tripinfo = _select_tripinfo(
        tripinfos,
        fallback_vtypes=(route_tools.AUTOWARE_EGO_VTYPE,),
    )
    summary_records = _read_summary_records(Path(output_paths["summary"]))
    plot_paths = _generate_plots(
        output_paths,
        energy_model,
        enabled=generate_plots,
    )
    progress_log.log("outputs", f"Generated {len(plot_paths)} plot files.")

    return SimulationResult(
        town=town,
        carla_version=CARLA_VERSION,
        traffic={
            "source_edge": traffic_source_edge,
            "destination_edge": traffic_destination_edge,
            "congestion_edge": traffic_congestion_edge,
            "vehicle_count": int(traffic_vehicle_count),
            "generated_count": scenario.generated_count,
            "target_count": scenario.target_count,
            "spawn_time": float(traffic_spawn_time),
            "stop_spawn_time": float(traffic_stop_spawn_time),
            "vehicle_type": "random" if random_vehicle_type else selected_vehicle_type,
            "seed": int(traffic_seed),
        },
        ego={
            "source_edge": ego_source_edge,
            "destination_edge": ego_destination_edge,
            "sumo_vtype": route_tools.AUTOWARE_EGO_VTYPE,
            "energy_model": energy_model,
            "max_battery_capacity": float(ego_max_battery_capacity),
            "current_battery_charge": battery_charge,
            "critical_battery_threshold": float(ego_critical_battery_threshold),
            "attributes": ego_attributes,
            "parameters": ego_parameters,
            "autoware_spawn": autoware_spawn,
            "autoware_route_start": autoware_route_start,
        },
        artifacts={
            "route_file": str(scenario.route_file),
            "trip_file": str(scenario.trip_file),
            "sumocfg_file": str(scenario.sumocfg_file),
        },
        output_paths=output_paths,
        plot_paths=plot_paths,
        energy_data=energy_data,
        energy_records=energy_data.to_dict(orient="records"),
        tripinfos=tripinfos,
        ego_tripinfo=ego_tripinfo,
        summary_records=summary_records,
        sync_returncode=(
            sync_launch.sync_process.poll()
            if sync_launch is not None and sync_launch.sync_process is not None
            else None
        ),
        completion_reason=completion_reason,
        progress_log_path=str(progress_log.path),
    )


def start_sumo_and_publish_autoware_route(
    *,
    sync_launch: Any,
    town: str,
    container_name: str,
    start_edge: str,
    goal_edge: str,
    speed_limit_kmh: Optional[float] = DEFAULT_AUTOWARE_SPEED_LIMIT_KMH,
    mirror_timeout: float = 60.0,
    route_timeout: float = 75.0,
    dashboard_api_url: str = DEFAULT_DASHBOARD_API_URL,
    progress_log: Optional[_ProgressLogger] = None,
) -> Dict[str, Any]:
    """Release a gated SUMO run, wait for the Autoware mirror, then publish route."""
    if progress_log is not None:
        progress_log.log("sync", "Releasing gated SUMO/CARLA simulation.")
    _release_start_gate(sync_launch.start_gate_file)
    if progress_log is not None:
        progress_log.log("sync", "Waiting for Autoware ego mirror in SUMO.")
    mirror_vehicle = _wait_for_autoware_sumo_mirror(
        sync_launch,
        timeout=float(mirror_timeout),
        dashboard_api_url=dashboard_api_url,
    )
    if progress_log is not None:
        progress_log.log(
            "sync",
            f"Autoware ego mirror visible in SUMO: {mirror_vehicle}.",
        )
        progress_log.log("autoware", "Publishing Autoware route after SUMO run started.")
    route_publication = route_tools.publish_autoware_route_in_container(
        town,
        container_name=container_name,
        start_edge=start_edge,
        goal_edge=goal_edge,
        speed_limit_kmh=speed_limit_kmh,
        ros_timeout_seconds=int(route_timeout),
        publish_initial_pose=False,
    )
    if progress_log is not None:
        progress_log.log("autoware", f"Route publication completed: {route_publication}.")
    return {
        "mirror_vehicle": mirror_vehicle,
        "route_publication": route_publication,
    }


def _activate_carla() -> None:
    """Configure repository helpers for the only supported automated version."""
    available = route_tools.available_carla_versions()
    if CARLA_VERSION not in available:
        raise RuntimeError(
            f"CARLA {CARLA_VERSION} is required for automated Autoware simulation. "
            f"Available versions: {', '.join(available) or 'none'}."
        )
    route_tools.set_active_carla_version(CARLA_VERSION)
    route_tools.ensure_carla_runner_dependencies_ready()


def _validate_town_and_edges(
    town: str,
    *edges: Optional[str],
) -> None:
    available_maps = route_tools.available_maps()
    if town not in available_maps:
        raise ValueError(
            f"Town '{town}' is not available for CARLA {CARLA_VERSION}. "
            f"Available towns: {', '.join(available_maps)}."
        )

    edge_ids = {edge.edge_id for edge in route_tools.read_sumo_edges(town)}
    for edge in edges:
        if edge and edge not in edge_ids:
            raise ValueError(f"Edge '{edge}' is not present in {town}.")


def _validate_battery(
    capacity: float,
    charge: Optional[float],
    threshold: float,
) -> None:
    capacity = float(capacity)
    threshold = float(threshold)
    effective_charge = capacity if charge is None else float(charge)
    if capacity <= 0:
        raise ValueError("ego_max_battery_capacity must be greater than zero.")
    if not 0 <= effective_charge <= capacity:
        raise ValueError("ego_current_battery_charge must be within [0, capacity].")
    if not 0 <= threshold <= capacity:
        raise ValueError("ego_critical_battery_threshold must be within [0, capacity].")
    if effective_charge <= threshold:
        raise ValueError(
            "ego_current_battery_charge must be greater than the critical threshold."
        )


def _normalize_energy_model(energy_model: str) -> str:
    value = str(energy_model or "").strip().lower()
    if value in {"mmpevem", route_tools.MMPEVEM_EMISSION_CLASS.lower()}:
        return route_tools.MMPEVEM_EMISSION_CLASS
    if value in {"energy", "energy/unknown", route_tools.ENERGY_EMISSION_CLASS.lower()}:
        return route_tools.ENERGY_EMISSION_CLASS
    raise ValueError("ego_energy_model must be either 'energy' or 'mmpevem'.")


def _ego_payload(
    energy_model: str,
    explicit_attributes: Optional[Dict[str, Any]],
    mixed_parameters: Optional[Dict[str, Any]],
) -> Tuple[Dict[str, str], Dict[str, str]]:
    attributes, parameters = route_tools.ego_model_defaults(energy_model)
    attribute_keys = set(attributes)

    for key, value in (mixed_parameters or {}).items():
        if key in attribute_keys:
            attributes[key] = value
        else:
            parameters[key] = value
    for key, value in (explicit_attributes or {}).items():
        attributes[key] = value

    return _string_payload(attributes), _string_payload(parameters)


def _string_payload(values: Dict[str, Any]) -> Dict[str, str]:
    return {
        str(key): str(value)
        for key, value in values.items()
        if value is not None and str(value).strip() != ""
    }


def _as_random_vehicle_type(
    vehicle_type: Optional[str],
    random_vehicle_type: bool,
) -> bool:
    return bool(random_vehicle_type) or str(vehicle_type or "").strip().lower() == "random"


def _simulation_end(stop_spawn_time: float, simulation_end: Optional[float]) -> float:
    if simulation_end is not None:
        return max(float(stop_spawn_time), float(simulation_end))
    return float(stop_spawn_time) + DEFAULT_EXTRA_SIMULATION_TIME


def _resolve_autoware_carla_rpc_timeout(
    *,
    explicit_timeout: Optional[float],
    carla_timeout: float,
    spawn_timeout: float,
) -> float:
    """Resolve the per-call CARLA RPC timeout used while polling Autoware spawn."""
    if explicit_timeout is not None:
        value = float(explicit_timeout)
        if value <= 0:
            raise ValueError("autoware_carla_rpc_timeout must be greater than zero.")
        return value

    return max(
        10.0,
        min(
            float(carla_timeout),
            max(float(spawn_timeout), 10.0),
            60.0,
        ),
    )


def _cleanup_existing_runtime(
    version: str,
    progress_log: Optional[_ProgressLogger] = None,
) -> None:
    sync_pids = _kill_existing_synchronizers()
    sumo_pids = _kill_existing_sumo_processes()
    if progress_log is not None:
        progress_log.log(
            "cleanup",
            (
                "Pre-run process cleanup: "
                f"sync_pids={sync_pids or []}, sumo_pids={sumo_pids or []}."
            ),
        )

    try:
        container = route_tools.find_running_autoware_container()
        container_name = container.get("Names") or container.get("ID")
        if container_name:
            _stop_autoware_processes(str(container_name))
    except Exception:
        pass
    carla_pids = []
    carla_stop_error = None
    try:
        carla_stop = route_tools.stop_carla_server(version, timeout=15.0)
        carla_pids.extend(carla_stop.get("stopped_pids") or [])
    except Exception as exc:
        carla_stop_error = exc

    carla_pids.extend(_kill_existing_carla_processes())
    _wait_for_carla_port_closed(timeout=5.0)
    if progress_log is not None:
        progress_log.log(
            "cleanup",
            f"Pre-run CARLA cleanup: carla_pids={sorted(set(carla_pids)) or []}.",
        )

    if route_tools.is_carla_server_ready():
        if carla_stop_error is not None:
            raise carla_stop_error
        raise RuntimeError(
            "CARLA is still reachable after pre-run cleanup on "
            f"{route_tools.DEFAULT_CARLA_HOST}:{route_tools.DEFAULT_CARLA_PORT}."
        )


def _kill_existing_synchronizers() -> List[int]:
    if psutil is None:
        return []

    current_pid = os.getpid()
    processes = []
    for process in psutil.process_iter(["pid", "cmdline"]):
        try:
            if int(process.info["pid"]) == current_pid:
                continue
            cmdline = " ".join(str(part) for part in (process.info.get("cmdline") or []))
            if not any(
                script_name in cmdline
                for script_name in (
                    "run_dashboard_synchronization.py",
                    "run_automated_synchronization.py",
                )
            ):
                continue
            processes.append(process)
        except (psutil.Error, OSError):
            continue

    return _terminate_psutil_processes(processes, interrupt=True)


def _kill_existing_sumo_processes() -> List[int]:
    """Stop leftover SUMO processes before launching a new automated run."""
    if psutil is None:
        return []

    current_pid = os.getpid()
    processes = []
    sumo_names = {"sumo", "sumo-gui"}
    for process in psutil.process_iter(["pid", "name", "cmdline", "exe"]):
        try:
            if int(process.info["pid"]) == current_pid:
                continue
            name = Path(str(process.info.get("name") or "")).name.lower()
            exe_name = Path(str(process.info.get("exe") or "")).name.lower()
            cmdline = [str(part) for part in (process.info.get("cmdline") or [])]
            first_arg = Path(cmdline[0]).name.lower() if cmdline else ""
            if name not in sumo_names and exe_name not in sumo_names and first_arg not in sumo_names:
                continue
            processes.append(process)
        except (psutil.Error, OSError):
            continue

    return _terminate_psutil_processes(processes, interrupt=False)


def _kill_existing_carla_processes() -> List[int]:
    """Stop leftover CARLA server processes before launching a new automated run."""
    if psutil is None:
        return []

    current_pid = os.getpid()
    processes = []
    for process in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            if int(process.info["pid"]) == current_pid:
                continue
            name = str(process.info.get("name") or "").lower()
            cmdline = " ".join(str(part) for part in (process.info.get("cmdline") or []))
            if "carlaue4" not in name and "carlaue4" not in cmdline.lower():
                continue
            processes.append(process)
        except (psutil.Error, OSError):
            continue

    return _terminate_psutil_processes(processes, interrupt=False)


def _terminate_psutil_processes(
    processes: Sequence[Any],
    *,
    interrupt: bool,
) -> List[int]:
    """Terminate a set of psutil processes and return the PIDs selected."""
    if psutil is None:
        return []

    selected = []
    unique_processes = []
    seen_pids = set()
    for process in processes:
        try:
            pid = int(process.pid)
        except (psutil.Error, OSError, TypeError, ValueError):
            continue
        if pid in seen_pids:
            continue
        seen_pids.add(pid)
        selected.append(pid)
        unique_processes.append(process)

    for process in unique_processes:
        try:
            if interrupt:
                process.send_signal(signal.SIGINT)
            else:
                process.terminate()
        except (psutil.Error, OSError):
            pass

    _, alive = psutil.wait_procs(unique_processes, timeout=5)
    for process in alive:
        try:
            process.terminate()
        except (psutil.Error, OSError):
            pass
    _, alive = psutil.wait_procs(alive, timeout=5)
    for process in alive:
        try:
            process.kill()
        except (psutil.Error, OSError):
            pass

    return selected


def _wait_for_carla_port_closed(timeout: float) -> bool:
    deadline = time.time() + float(timeout)
    while time.time() < deadline:
        if not route_tools.is_carla_server_ready():
            return True
        time.sleep(0.2)
    return not route_tools.is_carla_server_ready()


def _clear_previous_outputs(output_dir: Path) -> None:
    for filename in OUTPUT_XML_FILES.values():
        _safe_unlink(output_dir / filename)


def _start_carla(
    town: str,
    timeout: float,
    extra_args: Optional[Sequence[str]],
    progress_log: Optional[_ProgressLogger] = None,
) -> subprocess.Popen:
    if route_tools.is_carla_server_ready():
        raise RuntimeError(
            "CARLA is already reachable on "
            f"{route_tools.DEFAULT_CARLA_HOST}:{route_tools.DEFAULT_CARLA_PORT}."
        )

    log_file = route_tools.OUTPUT_DIR / "carla_server_headless.log"
    log_file.parent.mkdir(parents=True, exist_ok=True)
    args = ["./CarlaUE4.sh", "-RenderOffScreen", "-quality-level=Low", "-nosound"]
    if extra_args:
        args.extend(str(item) for item in extra_args)
    if progress_log is not None:
        progress_log.log("carla", f"Launching CARLA headless: {' '.join(args)}")

    log_handle = log_file.open("a", encoding="utf-8")
    log_handle.write(f"\n\n=== CarlaUE4 headless {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
    log_handle.write(" ".join(args) + "\n")
    log_handle.flush()

    try:
        process = subprocess.Popen(
            args,
            cwd=str(route_tools.CARLA_DIR),
            env=route_tools._build_env(),  # pylint: disable=protected-access
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    finally:
        log_handle.close()

    try:
        route_tools.wait_for_carla_server(process=process, timeout=timeout)
        if progress_log is not None:
            progress_log.log("carla", "CARLA server port is reachable.")
        route_tools.load_carla_map(
            town,
            log_file=route_tools.OUTPUT_DIR / "carla_map_headless.log",
        )
        if progress_log is not None:
            progress_log.log("carla", f"CARLA map loaded: {town}.")
    except Exception:
        if progress_log is not None:
            progress_log.log("carla", "CARLA startup failed; stopping process.")
        _stop_process(process, interrupt=False)
        raise
    return process


def _start_automated_synchronization(
    sumocfg_file: Path,
    *,
    carla_process: Optional[subprocess.Popen],
    carla_timeout: float,
    sumo_gui: bool,
    output_dir: Path,
    progress_log: Optional[_ProgressLogger] = None,
) -> AutomatedSynchronizationLaunch:
    """Start the automated-only synchronization runner in gated mode."""
    route_tools.ensure_carla_runner_dependencies_ready()

    start_gate_file = output_dir / "run_automated_synchronization.start"
    ready_file = output_dir / "run_automated_synchronization.ready"
    sync_log_file = output_dir / "automated_run_synchronization.log"
    carla_log_file = output_dir / "carla_server_headless.log"

    _safe_unlink(start_gate_file)
    _safe_unlink(ready_file)
    sync_log_file.parent.mkdir(parents=True, exist_ok=True)

    command = [
        str(route_tools.resolve_carla_python_executable()),
        str(
            route_tools.PROJECT_ROOT
            / "ecodrive"
            / "cosimulation"
            / "run_automated_synchronization.py"
        ),
        "--carla-version",
        CARLA_VERSION,
        "--carla-client-timeout",
        str(float(carla_timeout)),
        "--wait-start-file",
        str(start_gate_file),
        "--wait-ready-file",
        str(ready_file),
        route_tools.relative_to_sumo_dir(Path(sumocfg_file)),
    ]
    if sumo_gui:
        command.append("--sumo-gui")

    if progress_log is not None:
        progress_log.log(
            "sync",
            (
                "Starting automated SUMO/TraCI runner in gated mode with "
                f"CARLA client timeout {float(carla_timeout):.1f}s."
            ),
        )

    with sync_log_file.open("a", encoding="utf-8") as log_handle:
        log_handle.write(
            f"\n\n=== run_automated_synchronization {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n"
        )
        log_handle.write(" ".join(command) + "\n")
        log_handle.flush()
        sync_process = subprocess.Popen(
            command,
            cwd=str(route_tools.SUMO_DIR),
            env=route_tools._build_env(),  # pylint: disable=protected-access
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    return AutomatedSynchronizationLaunch(
        sync_process=sync_process,
        carla_process=carla_process,
        sync_log_file=sync_log_file,
        carla_log_file=carla_log_file,
        start_gate_file=start_gate_file,
        ready_file=ready_file,
    )


def _wait_for_sync_ready(sync_launch: Any, timeout: float) -> None:
    """Wait until the sync runner has SUMO ready and is blocked on the start gate."""
    ready_file = getattr(sync_launch, "ready_file", None)
    if ready_file is None:
        return

    ready_path = Path(ready_file)
    deadline = time.time() + timeout
    process = getattr(sync_launch, "sync_process", None)
    while time.time() < deadline:
        if ready_path.exists():
            return
        if process is not None and process.poll() is not None:
            sync_log_tail = _tail_text(getattr(sync_launch, "sync_log_file", None))
            raise RuntimeError(
                "SUMO/CARLA synchronization stopped before reaching the gated-ready state. "
                f"Exit code: {process.returncode}."
                f"{sync_log_tail}"
            )
        time.sleep(0.2)

    raise TimeoutError(
        "Timed out waiting for SUMO/CARLA to become ready before Autoware spawn."
        f"{_tail_text(getattr(sync_launch, 'sync_log_file', None))}"
    )


def _tail_text(path: Optional[Path], lines: int = 30) -> str:
    """Return a compact tail of a text log for exception messages."""
    if path is None:
        return ""
    try:
        text_lines = Path(path).read_text(encoding="utf-8", errors="ignore").splitlines()
    except OSError:
        return ""
    tail = "\n".join(text_lines[-lines:])
    return f"\nLast {min(lines, len(text_lines))} sync log lines:\n{tail}" if tail else ""


def _release_start_gate(start_gate_file: Optional[Path]) -> None:
    """Allow the gated SUMO/CARLA runner to start advancing simulation time."""
    if start_gate_file is None:
        return
    start_gate_file.parent.mkdir(parents=True, exist_ok=True)
    start_gate_file.write_text("start\n", encoding="utf-8")


def _wait_for_autoware_sumo_mirror(
    sync_launch: Any,
    *,
    timeout: float,
    dashboard_api_url: str,
) -> Dict[str, Any]:
    """Wait until the running bridge exposes the Autoware ego mirror in SUMO."""
    deadline = time.time() + timeout
    process = getattr(sync_launch, "sync_process", None)
    last_error = None

    while time.time() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError(
                "SUMO/CARLA synchronization stopped before mirroring the Autoware ego "
                f"in SUMO. Exit code: {process.returncode}."
            )

        try:
            payload = _read_json_url(
                f"{dashboard_api_url.rstrip('/')}/vehicles",
                timeout=2.0,
            )
            for vehicle in payload.get("vehicles", []):
                if _is_autoware_sumo_vehicle(vehicle):
                    return vehicle
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            last_error = exc

        time.sleep(0.5)

    detail = f" Last API error: {last_error}" if last_error else ""
    raise TimeoutError(
        "Timed out waiting for SUMO to mirror the Autoware ego after starting "
        f"the gated simulation.{detail}"
    )


def _read_json_url(url: str, timeout: float) -> Dict[str, Any]:
    with urlopen(url, timeout=timeout) as response:  # nosec - local dashboard API.
        return json.loads(response.read().decode("utf-8"))


def _is_autoware_sumo_vehicle(vehicle: Dict[str, Any]) -> bool:
    type_id = str(vehicle.get("type_id", ""))
    vehicle_id = str(vehicle.get("id", ""))
    return (
        type_id == route_tools.AUTOWARE_EGO_VTYPE
        or vehicle_id == route_tools.AUTOWARE_EGO_VTYPE
        or vehicle_id.startswith("carla")
        and type_id == route_tools.AUTOWARE_EGO_VTYPE
    )


def _wait_for_autoware_spawn(
    timeout: float,
    carla_rpc_timeout: float,
    progress_log: Optional[_ProgressLogger] = None,
) -> Dict[str, Any]:
    """Wait until Autoware has spawned its ego actor in CARLA."""
    if progress_log is not None:
        progress_log.log(
            "autoware",
            (
                "Waiting for Autoware ego actor in CARLA "
                f"(spawn_timeout={float(timeout):.1f}s, "
                f"carla_rpc_timeout={float(carla_rpc_timeout):.1f}s)."
            ),
        )
    script = """
import json
import os
import sys
import time

import carla


host = os.environ.get("ECODRIVE_CARLA_HOST", "127.0.0.1")
port = int(os.environ.get("ECODRIVE_CARLA_PORT", "2000"))
timeout = float(os.environ.get("ECODRIVE_AUTOWARE_SPAWN_TIMEOUT", "120"))
rpc_timeout = float(os.environ.get("ECODRIVE_CARLA_RPC_TIMEOUT", "30"))
type_id = os.environ.get("ECODRIVE_AUTOWARE_TYPE_ID", "vehicle.lexus.utlexus")
role_names = {"ego_vehicle", "hero", "autoware_ego", "aev_ego"}

client = carla.Client(host, port)
client.set_timeout(rpc_timeout)
deadline = time.time() + timeout
last_rpc_error = None

while time.time() < deadline:
    try:
        world = client.get_world()
        for actor in world.get_actors().filter("vehicle.*"):
            attributes = dict(actor.attributes)
            role_name = str(attributes.get("role_name", "")).strip()
            if actor.type_id != type_id and role_name not in role_names:
                continue
            transform = actor.get_transform()
            print(json.dumps({
                "actor_id": actor.id,
                "type_id": actor.type_id,
                "role_name": role_name,
                "location": [
                    transform.location.x,
                    transform.location.y,
                    transform.location.z,
                ],
            }))
            sys.exit(0)
    except RuntimeError as exc:
        last_rpc_error = str(exc)
    time.sleep(0.5)

detail = f" Last CARLA RPC error: {last_rpc_error}" if last_rpc_error else ""
raise RuntimeError(
    f"Timed out waiting for Autoware ego actor {type_id!r} after {timeout:.1f}s."
    f"{detail}"
)
""".strip()
    env = route_tools._build_env()  # pylint: disable=protected-access
    env.update(
        {
            "ECODRIVE_CARLA_HOST": route_tools.DEFAULT_CARLA_HOST,
            "ECODRIVE_CARLA_PORT": str(route_tools.DEFAULT_CARLA_PORT),
            "ECODRIVE_AUTOWARE_SPAWN_TIMEOUT": str(float(timeout)),
            "ECODRIVE_CARLA_RPC_TIMEOUT": str(float(carla_rpc_timeout)),
            "ECODRIVE_AUTOWARE_TYPE_ID": route_tools.AUTOWARE_EGO_VTYPE,
        }
    )
    process = subprocess.run(
        [str(route_tools.resolve_carla_python_executable()), "-c", script],
        env=env,
        capture_output=True,
        text=True,
        timeout=max(float(timeout) + float(carla_rpc_timeout) + 10.0, 10.0),
        check=False,
    )
    if process.returncode != 0:
        details = " | ".join(
            part.strip()
            for part in (process.stderr, process.stdout)
            if part and part.strip()
        )
        raise RuntimeError(
            "Autoware did not spawn the ego vehicle before SUMO startup: "
            f"{details or 'unknown error'}"
        )

    try:
        payload = json.loads(process.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            "Autoware ego spawn check completed but returned an invalid payload."
        ) from exc
    if progress_log is not None:
        progress_log.log("autoware", f"Autoware ego actor detected: {payload}.")
    return payload


def _wait_for_completion(
    process: subprocess.Popen,
    output_dir: Path,
    *,
    simulation_end: float,
    wall_timeout: Optional[float],
    stop_on_ego_arrival: bool,
    progress_log: Optional[_ProgressLogger] = None,
) -> str:
    timeout = (
        float(wall_timeout)
        if wall_timeout is not None
        else max(float(simulation_end) * 2.0, float(simulation_end) + 180.0)
    )
    deadline = time.time() + timeout
    tripinfo_path = output_dir / OUTPUT_XML_FILES["tripinfo"]

    while time.time() < deadline:
        if process.poll() is not None:
            if progress_log is not None:
                progress_log.log(
                    "run",
                    f"Synchronization process exited with code {process.returncode}.",
                )
            return "sync_process_finished"
        if stop_on_ego_arrival:
            tripinfo = _select_tripinfo(
                _read_tripinfos(tripinfo_path),
                fallback_vtypes=(route_tools.AUTOWARE_EGO_VTYPE,),
            )
            if tripinfo is not None:
                if progress_log is not None:
                    progress_log.log("run", f"Ego tripinfo detected: {tripinfo}.")
                _stop_process(process, interrupt=True)
                return "ego_arrived"
        time.sleep(1.0)

    _stop_process(process, interrupt=True)
    raise TimeoutError(
        f"Timed out waiting for the automated co-simulation after {timeout:.1f}s."
    )


def _stop_process(process: Optional[subprocess.Popen], *, interrupt: bool) -> None:
    if process is None or process.poll() is not None:
        return

    try:
        if interrupt:
            process.send_signal(signal.SIGINT)
        else:
            process.terminate()
        process.wait(timeout=10)
        return
    except (OSError, subprocess.TimeoutExpired):
        pass

    _kill_process_tree(process)


def _kill_process_tree(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return

    if psutil is not None:
        try:
            root = psutil.Process(process.pid)
            children = root.children(recursive=True)
            for child in children:
                child.terminate()
            root.terminate()
            _, alive = psutil.wait_procs(children + [root], timeout=5)
            for alive_process in alive:
                alive_process.kill()
            return
        except (psutil.Error, OSError):
            pass

    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        try:
            process.kill()
        except OSError:
            pass


def _stop_autoware_processes(container_name: str) -> None:
    docker = shutil.which("docker")
    if docker is None:
        return

    patterns = [
        "roslaunch autoware_mini start_carla.launch",
        "start_carla.launch",
        "carla_ros_bridge",
        "carla_waypoints_publisher",
        "lanelet2_global_planner",
        "rosmaster",
        "roscore",
        "rosout",
        "rviz",
    ]
    pattern = "|".join(re.escape(item) for item in patterns)
    command = (
        f"pkill -TERM -f {json.dumps(pattern)} || true; "
        "sleep 2; "
        f"pkill -KILL -f {json.dumps(pattern)} || true"
    )
    subprocess.run(
        [docker, "exec", str(container_name), "bash", "-lc", command],
        env=os.environ.copy(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )


def _stop_carla(process: Optional[subprocess.Popen], version: str) -> None:
    try:
        route_tools.stop_carla_server(version, timeout=15.0)
    except Exception:
        pass
    _stop_process(process, interrupt=False)


def _safe_unlink(path: Optional[Path]) -> None:
    if path is None:
        return
    try:
        Path(path).unlink(missing_ok=True)
    except OSError:
        pass


def _output_paths(output_dir: Path) -> Dict[str, str]:
    return {
        key: str(output_dir / filename)
        for key, filename in OUTPUT_XML_FILES.items()
    }


def _load_energy_output(
    output_paths: Dict[str, str],
    *,
    prefer_mmpevem: bool,
) -> pd.DataFrame:
    battery_path = Path(output_paths["battery"])
    emission_path = Path(output_paths["emission"])
    if not battery_path.exists() and not emission_path.exists():
        raise FileNotFoundError(
            "No SUMO energy output was produced. Expected "
            f"{battery_path} or {emission_path}."
        )
    return load_energy_data(
        battery_path,
        emission_xml_file=emission_path,
        prefer_mmpevem=prefer_mmpevem,
    )


def _generate_plots(
    output_paths: Dict[str, str],
    energy_model: str,
    *,
    enabled: bool,
) -> List[str]:
    if not enabled:
        return []
    plot_dir = Path(output_paths["battery"]).parent / "plots"
    paths = generate_battery_plots(
        Path(output_paths["battery"]),
        plot_dir,
        prefix="battery",
        emission_xml_file=Path(output_paths["emission"]),
        prefer_mmpevem=energy_model == route_tools.MMPEVEM_EMISSION_CLASS,
    )
    return [str(path) for path in paths]


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def _iter_xml_fragments(path: Path, tag_name: str) -> Iterable[ET.Element]:
    text = _read_text(path)
    if not text:
        return []

    open_tag = rf"<{tag_name}(?=[\s>/])[^>]*"
    pattern = re.compile(
        rf"{open_tag}/>|{open_tag}>.*?</{tag_name}>",
        re.DOTALL,
    )
    elements = []
    for match in pattern.finditer(text):
        try:
            elements.append(ET.fromstring(match.group(0)))
        except ET.ParseError:
            continue
    return elements


def _read_tripinfos(path: Path) -> List[Dict[str, str]]:
    tripinfos = []
    for tripinfo in _iter_xml_fragments(path, "tripinfo"):
        data = dict(tripinfo.attrib)
        emissions = tripinfo.find("emissions")
        if emissions is not None:
            for key, value in emissions.attrib.items():
                data[f"emissions_{key}"] = value
        tripinfos.append(data)
    return tripinfos


def _select_tripinfo(
    tripinfos: Sequence[Dict[str, str]],
    *,
    vehicle_id: Optional[str] = None,
    fallback_vtypes: Sequence[str] = (),
) -> Optional[Dict[str, str]]:
    if vehicle_id:
        for tripinfo in tripinfos:
            if tripinfo.get("id") == str(vehicle_id):
                return dict(tripinfo)

    matching = [
        tripinfo
        for tripinfo in tripinfos
        if tripinfo.get("vType") in set(fallback_vtypes)
    ]
    if not matching:
        return None

    return dict(max(matching, key=lambda item: _finite_float(item.get("arrival")) or -1.0))


def _read_summary_records(path: Path) -> List[Dict[str, str]]:
    return [dict(step.attrib) for step in _iter_xml_fragments(path, "step")]


def _finite_float(value: Optional[str]) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number
