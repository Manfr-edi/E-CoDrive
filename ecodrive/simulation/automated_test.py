"""Manual runner for selected automated co-simulation scenarios.

This file intentionally stays outside the simulation core used by Simfusion.
Edit ``SCENARIOS`` or pass ``--set name=value`` overrides to replay specific
cases and print the main oracle values after each run.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
from pathlib import Path
from random import randrange
import sys
from typing import Any, Dict, Iterable, Optional, Tuple
import xml.etree.ElementTree as ET

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ecodrive.simulation.automated_simulation import simulate


DEFAULT_EGO_MODEL_PARAMETERS = {
    "maximumPower": 240000,
    "constantPowerIntake": 360,
    "airDragCoefficient": 0.25,
    "frontSurfaceArea": 2.82,
    "mass": 2440,
    "rotatingMass": 80,
    "propulsionEfficiency": 0.80,
    "radialDragCoefficient": 0.1,
    "recuperationEfficiency": 0.80,
    "rollDragCoefficient": 0.01,
    "stoppingThreshold": 0.1,
}

SIMFUSION_TRAFFIC_CONGESTION_EDGE_CANDIDATES = [
    "-41.0.00",
    "-46.0.00",
    "-47.0.00",
    "-48.0.00",
    "-7.0.00",
    "-8.0.00",
    "-9.0.00",
    "-10.0.00",
    "-11.0.00",
    "-12.0.00",
    "-13.0.00",
]

SIMFUSION_TRAFFIC_SOURCE_EDGE_CANDIDATES = [
    "-40.0.00",
    "31.0.00",
    "-23.0.00",
    "18.0.00",
    "-28.0.00",
    "4.0.00",
    "-51.0.00",
    "-52.0.00",
    "-17.0.00",
    "29.0.00",
    "-3.0.00",
]

SIMFUSION_TRAFFIC_DESTINATION_EDGE_CANDIDATES = [
    "-22.0.00",
    "-49.0.00",
    "34.0.00",
    "-18.0.00",
    "28.0.00",
    "-4.0.00",
    "24.0.00",
    "-14.0.00",
    "51.0.00",
    "52.0.00",
    "17.0.00",
    "-29.0.00",
    "3.0.00",
    "-25.0.00",
]


SCENARIOS: Dict[str, Dict[str, Any]] = {
    "town04_congestion": {
        "town": "Town04",
        "headless": True,
        "traffic_generation_mode": "congestion",
        "traffic_congestion_edge": "-41.0.00",
        "traffic_source_edge": "-40.0.00",
        "traffic_destination_edge": "-22.0.00",
        "traffic_vehicle_count": None,
        "traffic_spawn_time": 5,
        "traffic_stop_spawn_time": 20,
        "traffic_vehicle_type": "vehicle.tesla.model3",
        "ego_starting_delay": 15.0,
        "ego_source_edge": "-17.0.00",
        "ego_destination_edge": "-26.0.00",
        "ego_energy_model": "Energy",
        "ego_max_battery_capacity": 75000,
        "ego_current_battery_charge": 680,
        "ego_critical_battery_threshold": 500,
        "ego_model_parameters": DEFAULT_EGO_MODEL_PARAMETERS,
    },
    "simfusion_default": {
        "town": "Town04",
        "headless": True,
        "traffic_generation_mode": "random",
        "traffic_congestion_edge": "-41.0.00",
        "traffic_source_edge": None,
        "traffic_destination_edge": None,
        "traffic_vehicle_count": 5,
        "traffic_seed": 42,
        "traffic_spawn_time": 0.0,
        "traffic_stop_spawn_time": 20.0,
        "traffic_vehicle_type": "random",
        "ego_starting_delay": 0.0,
        "ego_source_edge": "-38.0.00",
        "ego_destination_edge": "-41.0.00",
        "ego_energy_model": "Energy",
        "ego_max_battery_capacity": 75000,
        "ego_current_battery_charge": 680,
        "ego_critical_battery_threshold": 500,
        "ego_stall_timeout": 300.0,
        "ego_stall_speed_threshold": 0.05,
        "ego_stall_movement_tolerance": 1.0,
        "runtime_retries": 2,
        "generate_plots": True,
        "ego_model_parameters": DEFAULT_EGO_MODEL_PARAMETERS,
    },
}

SUCCESS_REASONS = {
    "ego_arrived_and_stopped",
    "ego_arrived_tripinfo",
    "ego_stopped_on_destination_edge",
}


def _finite_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    return str(value)


def _parse_value(raw_value: str) -> Any:
    try:
        return json.loads(raw_value)
    except json.JSONDecodeError:
        return raw_value


def _apply_overrides(config: Dict[str, Any], overrides: Iterable[str]) -> Dict[str, Any]:
    updated = dict(config)
    for override in overrides:
        if "=" not in override:
            raise ValueError(f"Override must be name=value, got: {override}")
        name, raw_value = override.split("=", 1)
        name = name.strip()
        if not name:
            raise ValueError(f"Override has an empty name: {override}")

        target = updated
        parts = name.split(".")
        for part in parts[:-1]:
            target = target.setdefault(part, {})
            if not isinstance(target, dict):
                raise ValueError(f"Cannot apply nested override to non-dict key: {name}")
        target[parts[-1]] = _parse_value(raw_value)
    return updated


def _load_scenario_json(path: Optional[Path]) -> Dict[str, Any]:
    if path is None:
        return {}
    with Path(path).open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Scenario JSON must contain an object: {path}")
    return data


def _bool_from_csv(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def _int_from_csv(value: Any) -> int:
    number = _finite_float(value)
    if number is None:
        raise ValueError(f"Expected numeric CSV value, got: {value!r}")
    return int(round(number))


def _candidate_from_index(candidates: Iterable[str], index: Any) -> str:
    values = list(candidates)
    if not values:
        raise ValueError("No edge candidates configured.")
    selected = max(0, min(len(values) - 1, _int_from_csv(index)))
    return values[selected]


def _load_testcase_rows(path: Path) -> list[Dict[str, str]]:
    with Path(path).open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"No testcase rows found in {path}")
    required = {
        "traffic_vehicle_count",
        "traffic_congestion_edge_index",
        "traffic_source_edge_index",
        "traffic_destination_edge_index",
        "Critical",
    }
    missing = required.difference(rows[0])
    if missing:
        raise ValueError(f"Missing required testcase CSV columns: {sorted(missing)}")
    return rows


def _testcase_features(row: Dict[str, str]) -> Tuple[int, int, int, int]:
    return (
        _int_from_csv(row["traffic_vehicle_count"]),
        _int_from_csv(row["traffic_congestion_edge_index"]),
        _int_from_csv(row["traffic_source_edge_index"]),
        _int_from_csv(row["traffic_destination_edge_index"]),
    )


def _normalized_feature_distance(
    left: Tuple[int, int, int, int],
    right: Tuple[int, int, int, int],
    spans: Tuple[float, float, float, float],
) -> float:
    return sum(
        abs(float(left_value) - float(right_value)) / max(float(span), 1.0)
        for left_value, right_value, span in zip(left, right, spans)
    )


def _select_diverse_rows(rows: list[Dict[str, str]], count: int) -> list[Dict[str, str]]:
    if count <= 0:
        return []
    if len(rows) <= count:
        return list(rows)

    feature_pairs = [(row, _testcase_features(row)) for row in rows]
    spans = tuple(
        max(features[column] for _, features in feature_pairs)
        - min(features[column] for _, features in feature_pairs)
        for column in range(4)
    )

    # Start with the densest traffic case in the class, then greedily maximize
    # distance from already selected rows to vary vehicle count and edge indices.
    selected = [max(feature_pairs, key=lambda item: (item[1][0], item[1][1], item[1][2], item[1][3]))]
    remaining = [item for item in feature_pairs if item[0] is not selected[0][0]]

    while remaining and len(selected) < count:
        best = max(
            remaining,
            key=lambda item: (
                min(
                    _normalized_feature_distance(item[1], selected_item[1], spans)
                    for selected_item in selected
                ),
                item[1][0],
            ),
        )
        selected.append(best)
        remaining.remove(best)

    return [row for row, _ in selected]


def select_testcases(
    rows: list[Dict[str, str]],
    *,
    critical_count: int,
    non_critical_count: int,
) -> list[Dict[str, str]]:
    critical_rows = [row for row in rows if _bool_from_csv(row.get("Critical"))]
    non_critical_rows = [row for row in rows if not _bool_from_csv(row.get("Critical"))]
    selected = _select_diverse_rows(critical_rows, critical_count)
    selected.extend(_select_diverse_rows(non_critical_rows, non_critical_count))
    return selected


def testcase_config(base_config: Dict[str, Any], row: Dict[str, str]) -> Dict[str, Any]:
    config = copy.deepcopy(base_config)
    config["traffic_vehicle_count"] = _int_from_csv(row["traffic_vehicle_count"])
    config["traffic_congestion_edge"] = (
        row.get("traffic_congestion_edge")
        or _candidate_from_index(
            SIMFUSION_TRAFFIC_CONGESTION_EDGE_CANDIDATES,
            row["traffic_congestion_edge_index"],
        )
    )
    config["traffic_source_edge"] = (
        row.get("traffic_source_edge")
        or _candidate_from_index(
            SIMFUSION_TRAFFIC_SOURCE_EDGE_CANDIDATES,
            row["traffic_source_edge_index"],
        )
    )
    config["traffic_destination_edge"] = (
        row.get("traffic_destination_edge")
        or _candidate_from_index(
            SIMFUSION_TRAFFIC_DESTINATION_EDGE_CANDIDATES,
            row["traffic_destination_edge_index"],
        )
    )
    return config


def testcase_metadata(row: Dict[str, str]) -> Dict[str, Any]:
    vehicle_count, congestion_index, source_index, destination_index = _testcase_features(row)
    testcase_id = row.get("Evaluation_ID") or row.get("Index")
    return {
        "testcase_id": testcase_id,
        "testcase_index": row.get("Index"),
        "evaluation_id": row.get("Evaluation_ID"),
        "scenario_folder": row.get("Scenario_Folder"),
        "expected_critical": _bool_from_csv(row.get("Critical")),
        "expected_completion_reason": row.get("Completion_Reason"),
        "expected_distance_remaining_m": _finite_float(row.get("Ego_Distance_Remaining_M")),
        "expected_battery_remaining_wh": _finite_float(row.get("Fitness_Final battery capacity")),
        "expected_stop_and_go_count": _int_from_csv(row.get("Ego_Stop_And_Go_Count")),
        "traffic_vehicle_count": vehicle_count,
        "traffic_congestion_edge_index": congestion_index,
        "traffic_source_edge_index": source_index,
        "traffic_destination_edge_index": destination_index,
    }


def _last_battery_record(path: Optional[str], vehicle_id: Optional[str]) -> Dict[str, str]:
    if not path:
        return {}
    xml_path = Path(path)
    if not xml_path.exists():
        return {}

    last_record: Dict[str, str] = {}
    requested_id = str(vehicle_id) if vehicle_id else None
    current_time = None
    for event, elem in ET.iterparse(xml_path, events=("start", "end")):
        if event == "start" and elem.tag == "timestep":
            current_time = elem.attrib.get("time")
        elif event == "end" and elem.tag == "vehicle":
            if requested_id and elem.attrib.get("id") != requested_id:
                elem.clear()
                continue
            last_record = dict(elem.attrib)
            if current_time is not None:
                last_record["time"] = current_time
            elem.clear()
    return last_record


def _count_stop_and_go_from_battery(
    path: Optional[str],
    vehicle_id: Optional[str],
    *,
    stopped_threshold: float = 0.1,
    moving_threshold: float = 0.5,
) -> Optional[int]:
    if not path:
        return None
    xml_path = Path(path)
    if not xml_path.exists():
        return None

    requested_id = str(vehicle_id) if vehicle_id else None
    was_stopped = False
    moved_once = False
    count = 0
    seen = False

    for _, elem in ET.iterparse(xml_path, events=("end",)):
        if elem.tag != "vehicle":
            continue
        if requested_id and elem.attrib.get("id") != requested_id:
            elem.clear()
            continue

        speed = _finite_float(elem.attrib.get("speed"))
        if speed is None:
            elem.clear()
            continue

        seen = True
        if speed <= stopped_threshold:
            if moved_once:
                was_stopped = True
        elif speed >= moving_threshold:
            if was_stopped:
                count += 1
                was_stopped = False
            moved_once = True
        elem.clear()

    return count if seen else None


def _tripinfo_stop_and_go(ego_tripinfo: Optional[Dict[str, str]]) -> Tuple[Optional[int], str]:
    if not ego_tripinfo:
        return None, "not_available"
    for key in ("waitingCount", "stopCount", "stops"):
        value = _finite_float(ego_tripinfo.get(key))
        if value is not None:
            return int(value), f"tripinfo.{key}"
    return None, "not_available"


def _reached_destination(completion_reason: str, ego_tripinfo: Optional[Dict[str, str]]) -> bool:
    if completion_reason in SUCCESS_REASONS:
        return True
    if ego_tripinfo and _finite_float(ego_tripinfo.get("arrival")) is not None:
        return True
    return False


def build_summary(
    result: Any,
    scenario_name: str,
    iteration: int,
    metadata: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    state = dict(result.last_ego_state or {})
    ego_tripinfo = dict(result.ego_tripinfo or {})
    vehicle_id = state.get("vehicle_id") or ego_tripinfo.get("id")
    battery_record = _last_battery_record(result.output_paths.get("battery"), vehicle_id)

    distance_remaining_m = _finite_float(state.get("distance_remaining_m"))
    battery_remaining_wh = _finite_float(state.get("battery"))
    if battery_remaining_wh is None:
        battery_remaining_wh = _finite_float(battery_record.get("actualBatteryCapacity"))

    stop_and_go_count, stop_and_go_source = _tripinfo_stop_and_go(ego_tripinfo)
    if stop_and_go_count is None:
        stop_and_go_count = _count_stop_and_go_from_battery(
            result.output_paths.get("battery"),
            vehicle_id,
        )
        stop_and_go_source = "battery.speed_transitions" if stop_and_go_count is not None else "not_available"

    reached_destination = _reached_destination(result.completion_reason, ego_tripinfo)
    status = "reached_destination" if reached_destination else "failed"

    summary = {
        "scenario": scenario_name,
        "iteration": iteration,
        "status": status,
        "reached_destination": reached_destination,
        "completion_reason": result.completion_reason,
        "distance_remaining_m": distance_remaining_m,
        "battery_remaining_wh": battery_remaining_wh,
        "stop_and_go_count": stop_and_go_count,
        "stop_and_go_source": stop_and_go_source,
        "ego_vehicle_id": vehicle_id,
        "last_edge": state.get("edge"),
        "last_speed_m_s": _finite_float(state.get("speed")),
        "tripinfo": ego_tripinfo,
        "last_ego_state": state,
        "output_paths": dict(result.output_paths),
        "csv_paths": dict(result.csv_paths),
        "progress_log_path": result.progress_log_path,
    }
    if metadata:
        summary.update(metadata)
    return summary


def print_summary(summary: Dict[str, Any]) -> None:
    print("\n=== Automated test result ===")
    print(f"Scenario: {summary['scenario']}  iteration: {summary['iteration']}")
    if "testcase_index" in summary:
        print(
            "Testcase: "
            f"id={summary.get('testcase_id')} "
            f"index={summary.get('testcase_index')} "
            f"evaluation_id={summary.get('evaluation_id')} "
            f"expected_critical={summary.get('expected_critical')}"
        )
        print(
            "Traffic input: "
            f"vehicles={summary.get('traffic_vehicle_count')} "
            f"congestion_idx={summary.get('traffic_congestion_edge_index')} "
            f"source_idx={summary.get('traffic_source_edge_index')} "
            f"destination_idx={summary.get('traffic_destination_edge_index')}"
        )
    print(f"Status: {summary['status']}")
    print(f"Completion reason: {summary['completion_reason']}")
    print(f"Reached destination: {summary['reached_destination']}")
    print(f"Distance remaining [m]: {summary['distance_remaining_m']}")
    print(f"Battery remaining [Wh]: {summary['battery_remaining_wh']}")
    print(
        "Stop and go count: "
        f"{summary['stop_and_go_count']} ({summary['stop_and_go_source']})"
    )
    print(f"Ego vehicle id: {summary['ego_vehicle_id']}")
    print(f"Last edge: {summary['last_edge']}")
    print(f"Last speed [m/s]: {summary['last_speed_m_s']}")
    print(f"Progress log: {summary['progress_log_path']}")
    print("Output files:")
    for key, path in summary["output_paths"].items():
        print(f"  {key}: {path}")


def run_case(
    *,
    scenario_name: str,
    config: Dict[str, Any],
    iteration: int,
    metadata: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    run_config = copy.deepcopy(config)
    if run_config.get("traffic_vehicle_count") is None:
        run_config["traffic_vehicle_count"] = randrange(1, 10, 1)

    print(f"\nRunning scenario={scenario_name}, iteration={iteration}")
    print(f"Traffic vehicles: {run_config['traffic_vehicle_count']}")
    print(
        "Traffic edges: "
        f"congestion={run_config.get('traffic_congestion_edge')}, "
        f"source={run_config.get('traffic_source_edge')}, "
        f"destination={run_config.get('traffic_destination_edge')}"
    )

    result = simulate(**run_config)
    summary = build_summary(result, scenario_name, iteration, metadata=metadata)
    print_summary(summary)
    return summary


def print_selected_testcases(rows: list[Dict[str, str]], base_config: Dict[str, Any]) -> None:
    print("\n=== Selected CSV testcases ===")
    for index, row in enumerate(rows):
        metadata = testcase_metadata(row)
        config = testcase_config(base_config, row)
        print(
            f"{index:02d}: testcase_id={metadata['testcase_id']} "
            f"index={metadata['testcase_index']} "
            f"critical={metadata['expected_critical']} "
            f"vehicles={metadata['traffic_vehicle_count']} "
            f"edges=({config['traffic_congestion_edge']}, "
            f"{config['traffic_source_edge']}, {config['traffic_destination_edge']}) "
            f"expected_reason={metadata['expected_completion_reason']}"
        )


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        default=None,
        choices=sorted(SCENARIOS),
        help=(
            "Scenario profile to run. Defaults to town04_congestion. Use "
            "simfusion_default explicitly only for CSV files generated with "
            "that scenario."
        ),
    )
    parser.add_argument(
        "--scenario-json",
        type=Path,
        help="Optional JSON object merged into the selected scenario.",
    )
    parser.add_argument("--iterations", type=int, default=1)
    parser.add_argument("--vehicle-count", type=int, help="Override traffic_vehicle_count.")
    parser.add_argument("--traffic-seed", type=int, help="Override traffic_seed.")
    parser.add_argument(
        "--testcases-csv",
        nargs="?",
        const=Path(__file__).with_name("all_testcases.csv"),
        type=Path,
        help=(
            "Run selected cases from a Simfusion all_testcases CSV. If no path is "
            "provided, uses all_testcases.csv next to this script."
        ),
    )
    parser.add_argument(
        "--critical-count",
        type=int,
        default=10,
        help="Number of critical CSV testcases to run.",
    )
    parser.add_argument(
        "--non-critical-count",
        type=int,
        default=10,
        help="Number of non-critical CSV testcases to run.",
    )
    parser.add_argument(
        "--dry-run-selected",
        action="store_true",
        help="Only print the selected CSV testcases without running simulations.",
    )
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        help="Override any simulate() argument, e.g. --set ego_current_battery_charge=650.",
    )
    parser.add_argument("--json-out", type=Path, help="Write all summaries to this JSON file.")
    parser.add_argument(
        "--show-scenarios",
        action="store_true",
        help="Print the available scenario profiles and exit.",
    )
    return parser


def main() -> int:
    args = build_argparser().parse_args()
    if args.show_scenarios:
        print(json.dumps(SCENARIOS, indent=2, default=_json_default))
        return 0

    scenario_name = args.scenario or "town04_congestion"
    config = copy.deepcopy(SCENARIOS[scenario_name])
    config.update(_load_scenario_json(args.scenario_json))
    if args.vehicle_count is not None:
        config["traffic_vehicle_count"] = args.vehicle_count
    if args.traffic_seed is not None:
        config["traffic_seed"] = args.traffic_seed
    config = _apply_overrides(config, args.overrides)

    summaries = []
    if args.testcases_csv:
        rows = _load_testcase_rows(args.testcases_csv)
        selected_rows = select_testcases(
            rows,
            critical_count=max(0, int(args.critical_count)),
            non_critical_count=max(0, int(args.non_critical_count)),
        )
        print_selected_testcases(selected_rows, config)
        if args.dry_run_selected:
            return 0

        run_items = [
            (
                testcase_config(config, row),
                testcase_metadata(row),
            )
            for row in selected_rows
        ]
    else:
        run_items = [(config, None) for _ in range(max(1, int(args.iterations)))]

    for iteration, (run_config, metadata) in enumerate(run_items):
        try:
            summaries.append(
                run_case(
                    scenario_name=scenario_name,
                    config=run_config,
                    iteration=iteration,
                    metadata=metadata,
                )
            )
        except Exception as exc:  # pragma: no cover - depends on live CARLA/SUMO runtime.
            last_state = dict(getattr(exc, "last_ego_state", {}) or {})
            summary = {
                "scenario": scenario_name,
                "iteration": iteration,
                "status": "failed",
                "reached_destination": False,
                "completion_reason": f"{type(exc).__name__}: {exc}",
                "distance_remaining_m": _finite_float(last_state.get("distance_remaining_m")),
                "battery_remaining_wh": _finite_float(last_state.get("battery")),
                "stop_and_go_count": None,
                "stop_and_go_source": "not_available",
                "ego_vehicle_id": last_state.get("vehicle_id"),
                "last_edge": last_state.get("edge"),
                "last_speed_m_s": _finite_float(last_state.get("speed")),
                "last_ego_state": last_state,
            }
            if metadata:
                summary.update(metadata)
            print_summary({**summary, "output_paths": {}, "progress_log_path": None})
            summaries.append(summary)

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(summaries, indent=2, default=_json_default),
            encoding="utf-8",
        )
        print(f"\nSaved JSON summary: {args.json_out}")

    return 0 if all(item["status"] != "failed" for item in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
