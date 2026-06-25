"""Programmatic CARLA 0.9.13 Autoware simulation runner.

The public entry point is :func:`simulate`, which runs the automated workflow
without opening the CARLA window or SUMO GUI.
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
from urllib.parse import urlencode
from urllib.request import urlopen

import pandas as pd

try:
    import psutil
except ImportError:  # pragma: no cover - optional process cleanup dependency.
    psutil = None

from ecodrive.analysis.battery_plots import generate_battery_plots, load_energy_data
from ecodrive.scenario import sumo_route_tools as route_tools


CARLA_VERSION = "0.9.13"
DEFAULT_AUTOWARE_STARTUP_WAIT = 0.0
DEFAULT_AUTOWARE_SPEED_LIMIT_KMH = 50.0
DEFAULT_EXTRA_SIMULATION_TIME = 300.0
DEFAULT_AUTOMATED_API_URL = "http://127.0.0.1:5000"
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
    csv_paths: Dict[str, str]
    plot_paths: List[str]
    energy_data: pd.DataFrame
    energy_records: List[Dict[str, Any]]
    tripinfos: List[Dict[str, str]]
    ego_tripinfo: Optional[Dict[str, str]]
    summary_records: List[Dict[str, str]]
    sync_returncode: Optional[int]
    completion_reason: str
    last_ego_state: Dict[str, Any]
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
    headless: bool = True,
    town: str,
    traffic_light_handling_mode: Optional[str] = None,
    force_carla_traffic_lights_green: bool = True,
    disable_autoware_traffic_light_handling: bool = True,
    traffic_congestion_edge: Optional[str] = None,
    traffic_source_edge: Optional[str] = None,
    traffic_destination_edge: Optional[str] = None,
    traffic_vehicle_count: int = 10,
    traffic_spawn_time: float = 0.0,
    traffic_stop_spawn_time: float = 120.0,
    traffic_vehicle_type: Optional[str] = None,
    traffic_random_vehicle_type: bool = False,
    traffic_random_vehicle_cars_only: bool = False,
    traffic_generation_mode: str = "congestion",
    ego_starting_delay: float = 0.0,
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
    traffic_vehicle_type_seed: Optional[int] = None,
    traffic_spawn_pattern: str = "Equidistant",
    autoware_startup_wait: float = DEFAULT_AUTOWARE_STARTUP_WAIT,
    autoware_speed_limit_kmh: Optional[float] = DEFAULT_AUTOWARE_SPEED_LIMIT_KMH,
    carla_timeout: float = 300.0,
    autoware_spawn_timeout: float = 300.0,
    autoware_carla_rpc_timeout: Optional[float] = None,
    autoware_sumo_mirror_timeout: float = 120.0,
    autoware_route_timeout: float = 120.0,
    wall_timeout: Optional[float] = None,
    stop_on_ego_arrival: bool = True,
    completion_grace_period: float = 3.0,
    destination_edge_end_tolerance: float = 22.0,
    destination_stall_timeout: Optional[float] = 45.0,
    ego_stall_timeout: Optional[float] = 240.0,
    ego_stall_speed_threshold: float = 0.05,
    ego_stall_movement_tolerance: float = 1.0,
    runtime_retries: int = 2,
    generate_plots: bool = True,
    cleanup_existing: bool = True,
    carla_extra_args: Optional[Sequence[str]] = None,
    progress_log_file: Optional[Path] = None,
) -> SimulationResult:
    """Run the complete headless automated workflow for CARLA 0.9.13.

    Parameters are intentionally close to the automated simulation fields. ``ego_model_parameters``
    may contain both SUMO vType parameters and vType attributes; keys that
    match the selected model attributes are routed to the XML attributes automatically.
    """

    _activate_carla()
    output_dir = route_tools.OUTPUT_DIR
    progress_log = _ProgressLogger(
        Path(progress_log_file)
        if progress_log_file is not None
        else output_dir / "automated_simulation_progress.log"
    )
    generation_mode = _normalize_traffic_generation_mode(traffic_generation_mode)
    progress_log.log(
        "start",
        (
            f"simulate(town={town}, carla_version={CARLA_VERSION}, "
            f"traffic_generation_mode={generation_mode})"
        ),
    )

    _validate_town_and_edges(
        town,
        *_traffic_edges_to_validate(
            generation_mode,
            traffic_congestion_edge,
            traffic_source_edge,
            traffic_destination_edge,
        ),
        ego_source_edge,
        ego_destination_edge,
    )
    _validate_battery(
        ego_max_battery_capacity,
        ego_current_battery_charge,
        ego_critical_battery_threshold,
    )
    progress_log.log("inputs", "Town, edge and battery inputs validated.")
    resolved_traffic_light_mode = _normalize_traffic_light_handling_mode(
        traffic_light_handling_mode,
        force_carla_traffic_lights_green=force_carla_traffic_lights_green,
        disable_autoware_traffic_light_handling=disable_autoware_traffic_light_handling,
    )

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
    if autoware_speed_limit_kmh is not None:
        ego_attributes["maxSpeed"] = str(max(0.0, float(autoware_speed_limit_kmh)) / 3.6)
    battery_charge = (
        float(ego_current_battery_charge)
        if ego_current_battery_charge is not None
        else float(ego_max_battery_capacity)
    )
    ego_parameters["automated.battery.failureThreshold"] = str(
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
    if random_vehicle_type and traffic_random_vehicle_cars_only:
        vehicle_types = _filter_vehicle_types_by_vclass(vehicle_types, {"passenger"})
        progress_log.log(
            "traffic",
            f"Filtered random vehicle pool to {len(vehicle_types)} passenger car types.",
        )
    selected_vehicle_type = (
        route_tools.DEFAULT_VEHICLE_TYPE
        if random_vehicle_type or not traffic_vehicle_type
        else str(traffic_vehicle_type)
    )
    resolved_vehicle_type_seed = (
        int(traffic_seed)
        if traffic_vehicle_type_seed is None
        else int(traffic_vehicle_type_seed)
    )
    resolved_simulation_end = _simulation_end(
        traffic_stop_spawn_time,
        simulation_end,
    )
    scenario, traffic_metadata = _generate_traffic_scenario(
        generation_mode=generation_mode,
        town=town,
        congestion_edge=traffic_congestion_edge,
        source_edge=traffic_source_edge,
        destination_edge=traffic_destination_edge,
        vehicle_count=int(traffic_vehicle_count),
        spawn_time=float(traffic_spawn_time),
        stop_spawn_time=float(traffic_stop_spawn_time),
        simulation_end=resolved_simulation_end,
        spawn_pattern=traffic_spawn_pattern,
        seed=int(traffic_seed),
        vehicle_type_seed=resolved_vehicle_type_seed,
        vehicle_type=selected_vehicle_type,
        random_vehicle_type=random_vehicle_type,
        vehicle_types=vehicle_types,
    )
    progress_log.log(
        "scenario",
        (
            f"Generated {generation_mode} route scenario {scenario.generated_count}/"
            f"{scenario.requested_count} vehicles; mode={scenario.mode}; "
            f"sumocfg={scenario.sumocfg_file}."
        ),
    )

    autoware_spawn = None
    autoware_route_start = None
    completion_reason = "unknown"
    last_ego_state: Dict[str, Any] = {}
    max_runtime_attempts = max(1, int(runtime_retries) + 1)

    for runtime_attempt in range(max_runtime_attempts):
        last_ego_state.clear()
        carla_process = None
        sync_launch = None
        autoware_container = None
        runtime_succeeded = False

        if runtime_attempt > 0:
            progress_log.log(
                "retry",
                (
                    f"Starting runtime attempt {runtime_attempt + 1}/"
                    f"{max_runtime_attempts} after a transient launch failure."
                ),
            )
            _cleanup_existing_runtime(CARLA_VERSION, progress_log=progress_log)
            _clear_previous_outputs(output_dir)

        try:
            carla_process = _start_carla(
                town,
                timeout=float(carla_timeout),
                extra_args=carla_extra_args,
                headless=headless,
                progress_log=progress_log,
            )
            sync_launch = _start_automated_synchronization(
                scenario.sumocfg_file,
                carla_process=carla_process,
                carla_timeout=float(carla_timeout),
                sumo_gui=not headless,
                output_dir=output_dir,
                traffic_light_handling_mode=resolved_traffic_light_mode,
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
                headless=headless,
                spawn_edge=ego_source_edge,
                start_edge=ego_source_edge,
                goal_edge=ego_destination_edge,
                speed_limit_kmh=autoware_speed_limit_kmh,
                carla_bridge_passive=False,
                publish_route=False,
                traffic_light_handling_mode=resolved_traffic_light_mode,
            )
            autoware_container = autoware_launch.get("container_name")
            progress_log.log(
                "autoware",
                f"Autoware launch requested in container {autoware_container}.",
            )
            if autoware_launch.get("command"):
                progress_log.log("autoware", f"Autoware command: {autoware_launch['command']}")
            if autoware_launch.get("launch_probe"):
                progress_log.log(
                    "autoware",
                    f"Autoware launch probe:\n{autoware_launch['launch_probe']}",
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
                ego_vehicle_delay=ego_starting_delay,
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
                critical_battery_threshold=float(ego_critical_battery_threshold),
                ego_vehicle_id=_completion_ego_vehicle_id(autoware_route_start),
                destination_edge=ego_destination_edge,
                destination_edge_end_tolerance=float(destination_edge_end_tolerance),
                destination_stall_timeout=destination_stall_timeout,
                ego_stall_timeout=ego_stall_timeout,
                ego_stall_speed_threshold=float(ego_stall_speed_threshold),
                ego_stall_movement_tolerance=float(ego_stall_movement_tolerance),
                completion_grace_period=float(completion_grace_period),
                state_sink=last_ego_state,
                progress_log=progress_log,
            )
            progress_log.log("run", f"Completion reason: {completion_reason}.")
            runtime_succeeded = True
        except Exception as exc:
            attempts_left = max_runtime_attempts - runtime_attempt - 1
            progress_log.log(
                "retry",
                (
                    f"Runtime attempt {runtime_attempt + 1}/{max_runtime_attempts} "
                    f"failed with {type(exc).__name__}: {exc}. "
                    f"Retries left: {attempts_left}."
                ),
            )
            if attempts_left <= 0:
                setattr(exc, "last_ego_state", dict(last_ego_state))
                raise
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

        if runtime_succeeded:
            break

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
    csv_paths = _export_output_csvs(
        output_paths,
        energy_data=energy_data,
        tripinfos=tripinfos,
        summary_records=summary_records,
    )
    progress_log.log("outputs", f"Saved {len(csv_paths)} CSV output files.")
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
            **traffic_metadata,
            "requested_source_edge": traffic_source_edge,
            "requested_destination_edge": traffic_destination_edge,
            "requested_congestion_edge": traffic_congestion_edge,
            "vehicle_count": int(traffic_vehicle_count),
            "generated_count": scenario.generated_count,
            "target_count": scenario.target_count,
            "spawn_time": float(traffic_spawn_time),
            "stop_spawn_time": float(traffic_stop_spawn_time),
            "vehicle_type": "random" if random_vehicle_type else selected_vehicle_type,
            "random_vehicle_type": random_vehicle_type,
            "random_vehicle_cars_only": bool(
                random_vehicle_type and traffic_random_vehicle_cars_only
            ),
            "random_vehicle_type_count": len(vehicle_types) if random_vehicle_type else None,
            "seed": int(traffic_seed),
            "vehicle_type_seed": resolved_vehicle_type_seed,
            "scenario_mode": scenario.mode,
            "traffic_light_handling_mode": resolved_traffic_light_mode,
            "force_carla_traffic_lights_green": resolved_traffic_light_mode == "disabled",
            "disable_autoware_traffic_light_handling": resolved_traffic_light_mode == "disabled",
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
        csv_paths=csv_paths,
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
        last_ego_state=dict(last_ego_state),
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
    ego_vehicle_delay: float = 10.0,
    mirror_timeout: float = 60.0,
    route_timeout: float = 75.0,
    automated_api_url: str = DEFAULT_AUTOMATED_API_URL,
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
        automated_api_url=automated_api_url,
    )
    if progress_log is not None:
        progress_log.log(
            "sync",
            f"Autoware ego mirror visible in SUMO: {mirror_vehicle}.",
        )
        progress_log.log("autoware", "Publishing Autoware route after SUMO run started.")
    if ego_vehicle_delay > 0:
        if progress_log is not None:
            progress_log.log("sync", "Waiting configured delay for ego vehicle to start.")
        time.sleep(ego_vehicle_delay)
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


def _completion_ego_vehicle_id(autoware_route_start: Optional[Dict[str, Any]]) -> Optional[str]:
    mirror_vehicle = (autoware_route_start or {}).get("mirror_vehicle") or {}
    vehicle_id = mirror_vehicle.get("id")
    return str(vehicle_id) if vehicle_id not in (None, "") else None


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


def _normalize_traffic_light_handling_mode(
    mode: Optional[str],
    *,
    force_carla_traffic_lights_green: bool,
    disable_autoware_traffic_light_handling: bool,
) -> str:
    value = str(mode or "").strip().lower().replace("-", "_").replace(" ", "_")
    if value in {"disabled", "off", "force_green", "green", "none"}:
        return "disabled"
    if value in {"normal", "active", "carla"}:
        return "normal"
    if value in {"yellow_as_yield", "yellow_yield", "flashing_yellow", "yellow_flash"}:
        return "yellow_as_yield"
    if value:
        raise ValueError(
            "traffic_light_handling_mode must be one of: disabled, normal, "
            "yellow_as_yield."
        )
    if force_carla_traffic_lights_green or disable_autoware_traffic_light_handling:
        return "disabled"
    return "normal"


def _filter_vehicle_types_by_vclass(
    vehicle_types: Sequence[str],
    allowed_vclasses: set[str],
) -> Sequence[str]:
    specs = route_tools.carla_vehicle_type_specs()
    allowed = {str(value).strip().lower() for value in allowed_vclasses}
    filtered = [
        vehicle_type
        for vehicle_type in vehicle_types
        if str(specs.get(vehicle_type, {}).get("vClass", "")).strip().lower()
        in allowed
    ]
    if not filtered:
        raise ValueError(
            "traffic_random_vehicle_cars_only=True did not match any passenger "
            "vehicle type in the active CARLA/SUMO vType configuration."
        )
    return filtered


def _normalize_traffic_generation_mode(mode: str) -> str:
    value = str(mode or "").strip().lower().replace("-", "_").replace(" ", "_")
    if value in {"", "congestion", "congestion_edge", "manual", "via_edge"}:
        return "congestion"
    if value in {
        "random",
        "random_congestion",
        "congestion_random",
        "random_edge",
        "random_via_edge",
        "target_random",
        "target_edge_random",
    }:
        return "random_congestion"
    if value in {
        "random_traffic",
        "automated_random",
        "random_trips",
        "randomtrips",
        "map_random",
        "whole_map_random",
    }:
        return "random_traffic"
    raise ValueError(
        "traffic_generation_mode must be one of: 'congestion', "
        "'random_congestion'/'random', or 'random_traffic'."
    )


def _traffic_edges_to_validate(
    generation_mode: str,
    congestion_edge: Optional[str],
    source_edge: Optional[str],
    destination_edge: Optional[str],
) -> Tuple[Optional[str], ...]:
    if generation_mode == "random_traffic":
        return ()
    if generation_mode == "random_congestion":
        return (congestion_edge,)
    return (congestion_edge, source_edge, destination_edge)


def _generate_traffic_scenario(
    *,
    generation_mode: str,
    town: str,
    congestion_edge: Optional[str],
    source_edge: Optional[str],
    destination_edge: Optional[str],
    vehicle_count: int,
    spawn_time: float,
    stop_spawn_time: float,
    simulation_end: float,
    spawn_pattern: str,
    seed: int,
    vehicle_type_seed: int,
    vehicle_type: str,
    random_vehicle_type: bool,
    vehicle_types: Sequence[str],
) -> Tuple[Any, Dict[str, Any]]:
    if generation_mode == "random_traffic":
        scenario = route_tools.generate_random_trips_scenario(
            map_name=town,
            vehicle_count=vehicle_count,
            begin=spawn_time,
            end=stop_spawn_time,
            simulation_end=simulation_end,
            seed=seed,
            vehicle_type_seed=vehicle_type_seed,
            vehicle_type=vehicle_type,
            random_vehicle_type=random_vehicle_type,
            vehicle_types=vehicle_types,
        )
        return scenario, {
            "generation_mode": generation_mode,
            "source_edge": None,
            "destination_edge": None,
            "congestion_edge": None,
            "spawn_pattern": None,
        }

    effective_source_edge = source_edge
    effective_destination_edge = destination_edge
    if generation_mode == "random_congestion":
        effective_source_edge = None
        effective_destination_edge = None

    scenario = route_tools.generate_congestion_scenario(
        map_name=town,
        target_edge=congestion_edge,
        destination_edge=effective_destination_edge,
        vehicle_count=vehicle_count,
        begin=spawn_time,
        end=stop_spawn_time,
        simulation_end=simulation_end,
        spawn_pattern=spawn_pattern,
        source_edge=effective_source_edge,
        seed=seed,
        vehicle_type_seed=vehicle_type_seed,
        vehicle_type=vehicle_type,
        random_vehicle_type=random_vehicle_type,
        vehicle_types=vehicle_types,
    )
    return scenario, {
        "generation_mode": generation_mode,
        "source_edge": effective_source_edge,
        "destination_edge": effective_destination_edge,
        "congestion_edge": congestion_edge,
        "spawn_pattern": spawn_pattern,
    }


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


def _resolve_carla_rpc_timeout(process_timeout: float) -> float:
    """Resolve a per-RPC CARLA guard used by readiness probes."""
    return max(10.0, min(float(process_timeout), 60.0))


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
    headless: bool = True,
    progress_log: Optional[_ProgressLogger] = None,
) -> subprocess.Popen:
    if route_tools.is_carla_server_ready():
        raise RuntimeError(
            "CARLA is already reachable on "
            f"{route_tools.DEFAULT_CARLA_HOST}:{route_tools.DEFAULT_CARLA_PORT}."
        )

    log_file = route_tools.OUTPUT_DIR / "carla_server_automated.log"
    log_file.parent.mkdir(parents=True, exist_ok=True)
    args = ["./CarlaUE4.sh", "-RenderOffScreen"] if headless else ["./CarlaUE4.sh"]
    if extra_args:
        args.extend(str(item) for item in extra_args)
    if progress_log is not None:
        progress_log.log("carla", f"Launching CARLA: {' '.join(args)}")

    log_handle = log_file.open("a", encoding="utf-8")
    log_handle.write(f"\n\n=== CarlaUE4 automated {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
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
            no_rendering=headless,
        )
        world_ready = _wait_for_carla_world_ready(
            town,
            timeout=float(timeout),
            rpc_timeout=_resolve_carla_rpc_timeout(timeout),
        )
        if progress_log is not None:
            progress_log.log("carla", f"CARLA world ready: {world_ready}.")
    except Exception:
        if progress_log is not None:
            progress_log.log("carla", "CARLA startup failed; stopping process.")
        _stop_process(process, interrupt=False)
        raise
    return process


def _wait_for_carla_world_ready(
    town: str,
    *,
    timeout: float,
    rpc_timeout: float,
) -> Dict[str, Any]:
    """Wait until CARLA answers RPC calls and exposes the requested world."""
    script = """
import json
import os
import sys
import time

import carla


host = os.environ.get("ECODRIVE_CARLA_HOST", "127.0.0.1")
port = int(os.environ.get("ECODRIVE_CARLA_PORT", "2000"))
town = os.environ["ECODRIVE_CARLA_TOWN"]
timeout = float(os.environ.get("ECODRIVE_CARLA_READY_TIMEOUT", "300"))
rpc_timeout = float(os.environ.get("ECODRIVE_CARLA_RPC_TIMEOUT", "30"))

client = carla.Client(host, port)
client.set_timeout(rpc_timeout)
deadline = time.time() + timeout
last_error = None

while time.time() < deadline:
    try:
        world = client.get_world()
        carla_map = world.get_map().name
        snapshot = world.get_snapshot()
        if town in carla_map:
            print(json.dumps({
                "map": carla_map,
                "frame": snapshot.frame,
                "elapsed_seconds": snapshot.timestamp.elapsed_seconds,
            }))
            sys.exit(0)
        last_error = f"loaded map is {carla_map!r}, expected {town!r}"
    except RuntimeError as exc:
        last_error = str(exc)
    time.sleep(0.5)

raise RuntimeError(
    f"Timed out waiting for CARLA world {town!r} after {timeout:.1f}s."
    + (f" Last CARLA RPC error: {last_error}" if last_error else "")
)
""".strip()
    env = route_tools._build_env()  # pylint: disable=protected-access
    env.update(
        {
            "ECODRIVE_CARLA_HOST": route_tools.DEFAULT_CARLA_HOST,
            "ECODRIVE_CARLA_PORT": str(route_tools.DEFAULT_CARLA_PORT),
            "ECODRIVE_CARLA_TOWN": str(town),
            "ECODRIVE_CARLA_READY_TIMEOUT": str(float(timeout)),
            "ECODRIVE_CARLA_RPC_TIMEOUT": str(float(rpc_timeout)),
        }
    )
    process = subprocess.run(
        [str(route_tools.resolve_carla_python_executable()), "-c", script],
        env=env,
        capture_output=True,
        text=True,
        timeout=max(float(timeout) + float(rpc_timeout) + 10.0, 10.0),
        check=False,
    )
    if process.returncode != 0:
        details = " | ".join(
            part.strip()
            for part in (process.stderr, process.stdout)
            if part and part.strip()
        )
        raise RuntimeError(
            "CARLA did not become ready after map load: "
            f"{details or 'unknown error'}"
        )

    try:
        return json.loads(process.stdout.strip().splitlines()[-1])
    except (IndexError, json.JSONDecodeError) as exc:
        raise RuntimeError("CARLA readiness check returned an invalid payload.") from exc


def _start_automated_synchronization(
    sumocfg_file: Path,
    *,
    carla_process: Optional[subprocess.Popen],
    carla_timeout: float,
    sumo_gui: bool,
    output_dir: Path,
    traffic_light_handling_mode: str,
    progress_log: Optional[_ProgressLogger] = None,
) -> AutomatedSynchronizationLaunch:
    """Start the automated-only synchronization runner in gated mode."""
    route_tools.ensure_carla_runner_dependencies_ready()

    start_gate_file = output_dir / "run_automated_synchronization.start"
    ready_file = output_dir / "run_automated_synchronization.ready"
    sync_log_file = output_dir / "automated_run_synchronization.log"
    carla_log_file = output_dir / "carla_server_automated.log"

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
        "--tls-manager",
        "none",
        route_tools.relative_to_sumo_dir(Path(sumocfg_file)),
    ]
    if traffic_light_handling_mode == "disabled":
        command.append("--force-traffic-lights-green")
    elif traffic_light_handling_mode == "yellow_as_yield":
        command.append("--force-traffic-lights-yellow")
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
    automated_api_url: str,
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
                f"{automated_api_url.rstrip('/')}/vehicles",
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
    with urlopen(url, timeout=timeout) as response:  # nosec - local automated API.
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


def _read_vehicle_state(
    automated_api_url: str,
    vehicle_id: str,
    *,
    timeout: float,
) -> Optional[Dict[str, Any]]:
    query = urlencode({"veh_id": str(vehicle_id)})
    try:
        payload = _read_json_url(
            f"{automated_api_url.rstrip('/')}/state?{query}",
            timeout=timeout,
        )
    except HTTPError as exc:
        if exc.code == 404:
            return None
        raise
    if not payload or payload.get("error"):
        return None
    return payload


def _read_autoware_vehicle_from_api(
    automated_api_url: str,
    *,
    timeout: float,
) -> Optional[Dict[str, Any]]:
    payload = _read_json_url(
        f"{automated_api_url.rstrip('/')}/vehicles",
        timeout=timeout,
    )
    for vehicle in payload.get("vehicles", []):
        if _is_autoware_sumo_vehicle(vehicle):
            return vehicle
    return None


def _normalize_sumo_edge_id(edge_id: Any) -> str:
    value = str(edge_id or "").strip()
    if re.search(r"_\d+$", value):
        return re.sub(r"_\d+$", "", value)
    return value


def _same_sumo_edge(left: Any, right: Any) -> bool:
    return bool(left and right) and _normalize_sumo_edge_id(left) == _normalize_sumo_edge_id(right)


def _ego_has_arrived_and_stopped(
    state: Dict[str, Any],
    *,
    destination_edge_completed: bool,
    destination_edge: Optional[str],
) -> bool:
    speed = _finite_float(state.get("speed"))
    distance_remaining = _finite_float(state.get("distance_remaining_m"))
    if speed is None or speed > 0.15:
        return False
    if destination_edge_completed:
        return True
    if destination_edge:
        return False
    # Fallback for SUMO-spawned vehicles whose SUMO route is known.
    if distance_remaining is None:
        return False
    return distance_remaining <= 2.0


def _ego_stopped_on_destination_edge(
    state: Dict[str, Any],
    *,
    destination_edge: Optional[str],
) -> bool:
    if not destination_edge or not _same_sumo_edge(state.get("edge"), destination_edge):
        return False

    route_final_edge = state.get("route_final_edge")
    if route_final_edge and not _same_sumo_edge(route_final_edge, destination_edge):
        return False

    speed = _finite_float(state.get("speed"))
    return speed is not None and speed <= 0.15


def _destination_stall_signature(state: Dict[str, Any]) -> Tuple[Any, Any, Optional[float]]:
    lane_position = _finite_float(state.get("lane_position_m"))
    return (
        _normalize_sumo_edge_id(state.get("edge")),
        state.get("lane"),
        round(lane_position, 1) if lane_position is not None else None,
    )


def _ego_is_stopped_for_stall(state: Dict[str, Any], *, speed_threshold: float) -> bool:
    speed = _finite_float(state.get("speed"))
    return speed is not None and speed <= max(float(speed_threshold), 0.0)


def _ego_stall_progress(
    state: Dict[str, Any],
) -> Tuple[Any, Any, Optional[float], Optional[float], Optional[float]]:
    return (
        _normalize_sumo_edge_id(state.get("edge")),
        state.get("lane"),
        _finite_float(state.get("lane_position_m")),
        _finite_float(state.get("distance_remaining_m")),
        _finite_float(state.get("distance_travelled_m")),
    )


def _ego_stall_progress_changed(
    previous: Tuple[Any, Any, Optional[float], Optional[float], Optional[float]],
    current: Tuple[Any, Any, Optional[float], Optional[float], Optional[float]],
    *,
    movement_tolerance: float,
) -> bool:
    if previous[:2] != current[:2]:
        return True

    tolerance = max(float(movement_tolerance), 0.0)
    for previous_value, current_value in zip(previous[2:], current[2:]):
        if previous_value is None or current_value is None:
            if previous_value != current_value:
                return True
            continue
        if abs(float(current_value) - float(previous_value)) > tolerance:
            return True
    return False


def _ego_completed_destination_edge(
    state: Dict[str, Any],
    *,
    destination_edge: Optional[str],
    end_tolerance: float,
) -> bool:
    if not destination_edge or not _same_sumo_edge(state.get("edge"), destination_edge):
        return False

    lane_position = _finite_float(state.get("lane_position_m"))
    edge_length = _finite_float(state.get("edge_length_m"))
    lane_length = _finite_float(state.get("lane_length_m"))
    target_length = edge_length or lane_length
    if lane_position is None or target_length is None or target_length <= 0:
        return False

    return lane_position >= max(0.0, target_length - max(float(end_tolerance), 0.0))


def _ego_battery_below_threshold(
    state: Dict[str, Any],
    *,
    fallback_threshold: float,
) -> bool:
    if str(state.get("battery_stop_applied", "")).strip().lower() in {"1", "true", "yes"}:
        return True

    battery = _finite_float(state.get("battery"))
    threshold = _finite_float(state.get("battery_failure_threshold"))
    if threshold is None or threshold <= 0:
        threshold = float(fallback_threshold)
    if threshold <= 0 or battery is None:
        return False
    return battery <= threshold


def _wait_for_completion(
    process: subprocess.Popen,
    output_dir: Path,
    *,
    simulation_end: float,
    wall_timeout: Optional[float],
    stop_on_ego_arrival: bool,
    critical_battery_threshold: float,
    ego_vehicle_id: Optional[str],
    destination_edge: Optional[str],
    destination_edge_end_tolerance: float,
    destination_stall_timeout: Optional[float],
    ego_stall_timeout: Optional[float],
    ego_stall_speed_threshold: float,
    ego_stall_movement_tolerance: float,
    completion_grace_period: float,
    automated_api_url: str = DEFAULT_AUTOMATED_API_URL,
    state_sink: Optional[Dict[str, Any]] = None,
    progress_log: Optional[_ProgressLogger] = None,
) -> str:
    timeout = (
        float(wall_timeout)
        if wall_timeout is not None
        else max(float(simulation_end) * 2.0, float(simulation_end) + 180.0)
    )
    deadline = time.time() + timeout
    tripinfo_path = output_dir / OUTPUT_XML_FILES["tripinfo"]
    completion_detected_at = None
    completion_reason = None
    resolved_ego_vehicle_id = ego_vehicle_id
    last_api_error = None
    destination_edge_completed = False
    destination_stopped_since = None
    destination_stopped_signature = None
    ego_stopped_since = None
    ego_stall_baseline = None
    last_state_log_at = 0.0

    while time.time() < deadline:
        if process.poll() is not None:
            if progress_log is not None:
                progress_log.log(
                    "run",
                    f"Synchronization process exited with code {process.returncode}.",
                )
            return "sync_process_finished"

        state = None
        try:
            if resolved_ego_vehicle_id:
                state = _read_vehicle_state(
                    automated_api_url,
                    resolved_ego_vehicle_id,
                    timeout=2.0,
                )
            if state is None:
                mirror_vehicle = _read_autoware_vehicle_from_api(
                    automated_api_url,
                    timeout=2.0,
                )
                if mirror_vehicle is not None:
                    resolved_ego_vehicle_id = str(mirror_vehicle.get("id") or "")
                    if resolved_ego_vehicle_id:
                        state = _read_vehicle_state(
                            automated_api_url,
                            resolved_ego_vehicle_id,
                            timeout=2.0,
                        )
                    if state is None:
                        state = mirror_vehicle
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            last_api_error = exc

        if state is not None and state_sink is not None:
            state_sink.clear()
            state_sink.update(state)

        if state is not None and completion_detected_at is None:
            if _ego_completed_destination_edge(
                state,
                destination_edge=destination_edge,
                end_tolerance=float(destination_edge_end_tolerance),
            ):
                destination_edge_completed = True

            if progress_log is not None and time.time() - last_state_log_at >= 10.0:
                last_state_log_at = time.time()
                progress_log.log(
                    "run",
                    (
                        "Live ego state: "
                        f"id={resolved_ego_vehicle_id}, edge={state.get('edge')}, "
                        f"dest={destination_edge}, dest_completed={destination_edge_completed}, "
                        f"speed={state.get('speed')}, "
                        f"lane_position={state.get('lane_position_m')}, "
                        f"edge_length={state.get('edge_length_m')}, "
                        f"distance_remaining={state.get('distance_remaining_m')}, "
                        f"battery={state.get('battery')}, "
                        f"threshold={state.get('battery_failure_threshold')}."
                    ),
                )

            if stop_on_ego_arrival and _ego_has_arrived_and_stopped(
                state,
                destination_edge_completed=destination_edge_completed,
                destination_edge=destination_edge,
            ):
                completion_reason = "ego_arrived_and_stopped"
            elif (
                stop_on_ego_arrival
                and destination_stall_timeout is not None
                and float(destination_stall_timeout) >= 0
                and _ego_stopped_on_destination_edge(
                    state,
                    destination_edge=destination_edge,
                )
            ):
                now = time.time()
                signature = _destination_stall_signature(state)
                if signature != destination_stopped_signature:
                    destination_stopped_signature = signature
                    destination_stopped_since = now
                    if progress_log is not None:
                        progress_log.log(
                            "run",
                            (
                                "Ego is stopped on the destination edge; "
                                f"waiting up to {float(destination_stall_timeout):.1f}s "
                                "before treating it as terminal."
                            ),
                        )
                elif destination_stopped_since is not None and (
                    now - destination_stopped_since >= float(destination_stall_timeout)
                ):
                    completion_reason = "ego_stopped_on_destination_edge"
            else:
                destination_stopped_since = None
                destination_stopped_signature = None

            if completion_reason is None and _ego_battery_below_threshold(
                state,
                fallback_threshold=float(critical_battery_threshold),
            ):
                completion_reason = "critical_battery_threshold"

            if (
                completion_reason is None
                and ego_stall_timeout is not None
                and float(ego_stall_timeout) >= 0
                and _ego_is_stopped_for_stall(
                    state,
                    speed_threshold=float(ego_stall_speed_threshold),
                )
            ):
                now = time.time()
                current_progress = _ego_stall_progress(state)
                if (
                    ego_stall_baseline is None
                    or _ego_stall_progress_changed(
                        ego_stall_baseline,
                        current_progress,
                        movement_tolerance=float(ego_stall_movement_tolerance),
                    )
                ):
                    ego_stall_baseline = current_progress
                    ego_stopped_since = now
                    if progress_log is not None:
                        progress_log.log(
                            "run",
                            (
                                "Ego is stopped without route progress; "
                                f"waiting up to {float(ego_stall_timeout):.1f}s "
                                "before treating it as terminal."
                            ),
                        )
                elif ego_stopped_since is not None and (
                    now - ego_stopped_since >= float(ego_stall_timeout)
                ):
                    completion_reason = "ego_stalled"
            else:
                ego_stopped_since = None
                ego_stall_baseline = None

            if completion_reason is not None:
                completion_detected_at = time.time()
                if progress_log is not None:
                    progress_log.log(
                        "run",
                        (
                            f"Stop condition detected: {completion_reason}; "
                            f"state={state}. Waiting {float(completion_grace_period):.1f}s "
                            "before stopping synchronization."
                        ),
                    )

        if completion_detected_at is not None:
            if time.time() - completion_detected_at >= max(float(completion_grace_period), 0.0):
                _stop_process(process, interrupt=True)
                return str(completion_reason)
            time.sleep(0.2)
            continue

        if stop_on_ego_arrival:
            tripinfo = _select_tripinfo(
                _read_tripinfos(tripinfo_path),
                fallback_vtypes=(route_tools.AUTOWARE_EGO_VTYPE,),
            )
            if tripinfo is not None:
                if progress_log is not None:
                    progress_log.log("run", f"Ego tripinfo detected: {tripinfo}.")
                _stop_process(process, interrupt=True)
                return "ego_arrived_tripinfo"
        time.sleep(1.0)

    _stop_process(process, interrupt=True)
    detail = f" Last automated API error: {last_api_error}" if last_api_error else ""
    raise TimeoutError(
        f"Timed out waiting for the automated co-simulation after {timeout:.1f}s."
        f"{detail}"
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


def _stop_autoware_processes(container_name: str, timeout: float = 20.0) -> None:
    docker = shutil.which("docker")
    if docker is None:
        return

    patterns = [
        "roslaunch autoware_mini start_carla.launch",
        "roslaunch autoware_mini start_carla_headless.launch",
        "start_carla.launch",
        "start_carla_headless.launch",
        "roslaunch",
        "autoware_mini",
        "carla_ros_bridge",
        "carla_waypoints_publisher",
        "lanelet2_global_planner",
        "rosmaster",
        "roscore",
        "rosout",
        "rviz",
    ]
    pattern = "|".join(re.escape(item) for item in patterns)
    cleanup_script = r"""
import os
import re
import signal
import time


pattern = re.compile(os.environ["AUTOWARE_STOP_PATTERN"])
timeout = float(os.environ.get("AUTOWARE_STOP_TIMEOUT_SECONDS", "20"))
current_pid = os.getpid()


def process_text(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            cmdline = handle.read().replace(b"\0", b" ").decode("utf-8", "ignore")
        with open(f"/proc/{pid}/comm", "r", encoding="utf-8", errors="ignore") as handle:
            comm = handle.read().strip()
    except OSError:
        return ""
    return f"{comm} {cmdline}"


def matching_pids():
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == current_pid:
            continue
        if pattern.search(process_text(pid)):
            pids.append(pid)
    return pids


def signal_matches(sig):
    for pid in matching_pids():
        try:
            os.kill(pid, sig)
        except OSError:
            pass


def wait_for_exit(seconds):
    deadline = time.time() + max(float(seconds), 0.0)
    while time.time() < deadline:
        if not matching_pids():
            return True
        time.sleep(0.25)
    return not matching_pids()


signal_matches(signal.SIGTERM)
if not wait_for_exit(timeout / 2.0):
    signal_matches(signal.SIGKILL)
    wait_for_exit(timeout / 2.0)
""".strip()
    try:
        subprocess.run(
            [
                docker,
                "exec",
                "-e",
                f"AUTOWARE_STOP_PATTERN={pattern}",
                "-e",
                f"AUTOWARE_STOP_TIMEOUT_SECONDS={float(timeout)}",
                str(container_name),
                "python3",
                "-c",
                cleanup_script,
            ],
            env=os.environ.copy(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=max(float(timeout) + 5.0, 5.0),
            check=False,
        )
    except subprocess.TimeoutExpired:
        pass


def _stop_carla(process: Optional[subprocess.Popen], version: str) -> None:
    try:
        route_tools.stop_carla_server(version, timeout=15.0)
    except Exception:
        pass
    _stop_process(process, interrupt=False)
    _wait_for_carla_port_closed(timeout=15.0)


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


def _export_output_csvs(
    output_paths: Dict[str, str],
    *,
    energy_data: pd.DataFrame,
    tripinfos: Sequence[Dict[str, str]],
    summary_records: Sequence[Dict[str, str]],
) -> Dict[str, str]:
    csv_dir = Path(output_paths["battery"]).parent / "csv"
    csv_dir.mkdir(parents=True, exist_ok=True)

    csv_paths = {
        "energy": csv_dir / "energy.csv",
        "battery": csv_dir / "battery.csv",
        "emission": csv_dir / "emission.csv",
        "tripinfo": csv_dir / "tripinfos.csv",
        "summary": csv_dir / "summary.csv",
        "vehroute": csv_dir / "vehroute.csv",
        "edgedata": csv_dir / "edgedata.csv",
    }

    _write_dataframe_csv(energy_data, csv_paths["energy"])
    _write_records_csv(
        _read_timestep_vehicle_records(Path(output_paths["battery"])),
        csv_paths["battery"],
    )
    _write_records_csv(
        _read_timestep_vehicle_records(Path(output_paths["emission"])),
        csv_paths["emission"],
    )
    _write_records_csv(tripinfos, csv_paths["tripinfo"])
    _write_records_csv(summary_records, csv_paths["summary"])
    _write_records_csv(
        _read_vehroute_records(Path(output_paths["vehroute"])),
        csv_paths["vehroute"],
    )
    _write_records_csv(
        _read_edgedata_records(Path(output_paths["edgedata"])),
        csv_paths["edgedata"],
    )

    return {key: str(path) for key, path in csv_paths.items()}


def _write_dataframe_csv(data: pd.DataFrame, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data.to_csv(path, index=False)


def _write_records_csv(records: Sequence[Dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(list(records)).to_csv(path, index=False)


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


def _read_timestep_vehicle_records(path: Path) -> List[Dict[str, str]]:
    records = []
    for timestep in _iter_xml_fragments(path, "timestep"):
        timestep_attrs = _prefixed_attributes(timestep.attrib, "timestep")
        for vehicle in timestep.findall("vehicle"):
            data = dict(timestep_attrs)
            data.update(vehicle.attrib)
            records.append(data)
    return records


def _read_vehroute_records(path: Path) -> List[Dict[str, str]]:
    records = []
    for vehicle in _iter_xml_fragments(path, "vehicle"):
        data = dict(vehicle.attrib)
        routes = vehicle.findall(".//route")
        data["route_count"] = str(len(routes))
        for index, route in enumerate(routes):
            prefix = "route" if index == 0 else f"route_{index}"
            data.update(_prefixed_attributes(route.attrib, prefix))
        records.append(data)
    return records


def _read_edgedata_records(path: Path) -> List[Dict[str, str]]:
    records = []
    for interval in _iter_xml_fragments(path, "interval"):
        interval_attrs = _prefixed_attributes(interval.attrib, "interval")
        for edge in interval.findall("edge"):
            edge_attrs = dict(edge.attrib)
            lane_elements = edge.findall("lane")
            if not lane_elements:
                data = dict(interval_attrs)
                data.update(edge_attrs)
                records.append(data)
                continue

            data = dict(interval_attrs)
            data.update(edge_attrs)
            data["record_type"] = "edge"
            records.append(data)
            for lane in lane_elements:
                lane_data = dict(interval_attrs)
                lane_data.update(_prefixed_attributes(edge_attrs, "edge"))
                lane_data.update(lane.attrib)
                lane_data["record_type"] = "lane"
                records.append(lane_data)
    return records


def _prefixed_attributes(attributes: Dict[str, str], prefix: str) -> Dict[str, str]:
    return {f"{prefix}_{key}": value for key, value in attributes.items()}


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
