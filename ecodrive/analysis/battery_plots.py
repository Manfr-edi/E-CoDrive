"""Plot battery metrics from SUMO battery and emission output XML files."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Optional, Set, Union
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns


MMPEVEM_ECLASS_PREFIX = "MMPEVEM"

BATTERY_PLOT_FILENAMES = {
    "energyConsumed": "energy_consumed",
    "totalEnergyConsumed": "total_energy_consumed",
    "actualBatteryCapacity": "actual_battery_capacity",
}


def _finite_float(value: Optional[str]) -> Optional[float]:
    """Return a finite float parsed from an XML attribute."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _read_initial_battery_state(
    xml_file: Optional[Path],
    vehicle_ids: Optional[Set[str]] = None,
) -> Dict[str, Optional[float]]:
    """Read the first battery state for the selected vehicles."""
    if xml_file is None or not xml_file.exists():
        return {"charge": None, "maximum": None}

    selected_ids = {str(vehicle_id) for vehicle_id in vehicle_ids or set()}
    states: Dict[str, Dict[str, Optional[float]]] = {}

    for event, elem in ET.iterparse(xml_file, events=("end",)):
        if elem.tag != "vehicle":
            continue

        vehicle_id = elem.attrib.get("id")
        if not vehicle_id or vehicle_id in states:
            elem.clear()
            continue
        if selected_ids and vehicle_id not in selected_ids:
            elem.clear()
            continue

        charge = _finite_float(
            elem.attrib.get("actualBatteryCapacity")
            or elem.attrib.get("chargeLevel")
            or elem.attrib.get("batteryChargeLevel")
        )
        maximum = _finite_float(
            elem.attrib.get("maximumBatteryCapacity")
            or elem.attrib.get("maxBatteryCapacity")
        )
        if charge is not None or maximum is not None:
            states[vehicle_id] = {"charge": charge, "maximum": maximum}
        elem.clear()

    if not states:
        return {"charge": None, "maximum": None}

    charge_values = [state["charge"] for state in states.values() if state["charge"] is not None]
    maximum_values = [state["maximum"] for state in states.values() if state["maximum"] is not None]
    return {
        "charge": sum(charge_values) if charge_values else None,
        "maximum": sum(maximum_values) if maximum_values else None,
    }


def load_battery_data(xml_file: Path, vehicle_id: Optional[str] = None) -> pd.DataFrame:
    """Read SUMO battery output as a dataframe."""
    rows: List[Dict[str, Union[float, str]]] = []
    current_time: Optional[float] = None

    for event, elem in ET.iterparse(xml_file, events=("start", "end")):
        if event == "start" and elem.tag == "timestep":
            current_time = float(elem.attrib["time"])
        elif event == "end" and elem.tag == "vehicle":
            if current_time is None:
                continue
            if vehicle_id and elem.attrib.get("id") != vehicle_id:
                elem.clear()
                continue

            rows.append(
                {
                    "time": current_time,
                    "vehicle_id": elem.attrib["id"],
                    "energyConsumed": float(elem.attrib["energyConsumed"]),
                    "totalEnergyConsumed": float(elem.attrib["totalEnergyConsumed"]),
                    "actualBatteryCapacity": float(elem.attrib["actualBatteryCapacity"]),
                }
            )
            elem.clear()

    if not rows:
        target = f" for vehicle '{vehicle_id}'" if vehicle_id else ""
        raise ValueError(f"No vehicle battery records found{target} in {xml_file}")

    data = pd.DataFrame(rows)
    if vehicle_id:
        return data.sort_values("time")

    return (
        data.groupby("time", as_index=False)
        .agg(
            energyConsumed=("energyConsumed", "sum"),
            totalEnergyConsumed=("totalEnergyConsumed", "sum"),
            actualBatteryCapacity=("actualBatteryCapacity", "sum"),
        )
        .sort_values("time")
    )


def load_mmpevem_emission_data(
    xml_file: Path,
    battery_xml_file: Optional[Path] = None,
    vehicle_id: Optional[str] = None,
) -> pd.DataFrame:
    """Read MMPEVEM net electricity from SUMO emission output as a dataframe."""
    rows: List[Dict[str, Union[float, str]]] = []
    vehicle_ids: Set[str] = set()
    current_time: Optional[float] = None
    requested_vehicle_id = str(vehicle_id) if vehicle_id else None

    for event, elem in ET.iterparse(xml_file, events=("start", "end")):
        if event == "start" and elem.tag == "timestep":
            current_time = _finite_float(elem.attrib.get("time"))
        elif event == "end" and elem.tag == "vehicle":
            if current_time is None:
                elem.clear()
                continue

            record_vehicle_id = elem.attrib.get("id")
            eclass = elem.attrib.get("eclass", "")
            if not eclass.startswith(MMPEVEM_ECLASS_PREFIX):
                elem.clear()
                continue
            if requested_vehicle_id and record_vehicle_id != requested_vehicle_id:
                elem.clear()
                continue

            electricity = _finite_float(elem.attrib.get("electricity"))
            if electricity is None or not record_vehicle_id:
                elem.clear()
                continue

            vehicle_ids.add(record_vehicle_id)
            rows.append(
                {
                    "time": current_time,
                    "vehicle_id": record_vehicle_id,
                    "energyConsumed": electricity,
                }
            )
            elem.clear()

    if not rows:
        target = f" for vehicle '{vehicle_id}'" if vehicle_id else ""
        raise ValueError(f"No MMPEVEM emission records found{target} in {xml_file}")

    data = pd.DataFrame(rows)
    if vehicle_id:
        data = data.sort_values("time")
    else:
        data = (
            data.groupby("time", as_index=False)
            .agg(energyConsumed=("energyConsumed", "sum"))
            .sort_values("time")
        )

    data["totalEnergyConsumed"] = data["energyConsumed"].clip(lower=0).cumsum()
    net_energy = data["energyConsumed"].cumsum()

    initial_state = _read_initial_battery_state(battery_xml_file, vehicle_ids)
    initial_charge = initial_state["charge"]
    maximum_charge = initial_state["maximum"]
    if initial_charge is None:
        data["actualBatteryCapacity"] = float("nan")
    else:
        battery = initial_charge - net_energy
        if maximum_charge is not None:
            battery = battery.clip(lower=0, upper=maximum_charge)
        data["actualBatteryCapacity"] = battery

    return data


def load_energy_data(
    battery_xml_file: Path,
    emission_xml_file: Optional[Path] = None,
    vehicle_id: Optional[str] = None,
    prefer_mmpevem: bool = False,
) -> pd.DataFrame:
    """Read the best available energy-output data for the selected model."""
    if prefer_mmpevem and emission_xml_file and emission_xml_file.exists():
        return load_mmpevem_emission_data(
            emission_xml_file,
            battery_xml_file=battery_xml_file,
            vehicle_id=vehicle_id,
        )
    return load_battery_data(battery_xml_file, vehicle_id)


def filter_time_range(data: pd.DataFrame, start: Optional[float], end: Optional[float]) -> pd.DataFrame:
    """Return records within an optional time range."""
    if start is not None and end is not None and start > end:
        raise ValueError(f"Invalid time range: start ({start}) is greater than end ({end})")

    filtered = data
    if start is not None:
        filtered = filtered[filtered["time"] >= start]
    if end is not None:
        filtered = filtered[filtered["time"] <= end]

    if filtered.empty:
        available_start = data["time"].min()
        available_end = data["time"].max()
        raise ValueError(
            "No data in the requested time range. "
            f"Available data is from {available_start:.2f}s to {available_end:.2f}s."
        )

    return filtered


def save_single_plot(
    data: pd.DataFrame,
    metric: str,
    output: Path,
    ylabel: str,
    color: str,
    fill_positive_negative: bool = False,
    close_after_save: bool = True,
) -> None:
    """Save a single battery metric line plot."""
    sns.set_theme(style="whitegrid", context="talk")
    fig, ax = plt.subplots(figsize=(18, 4))

    x = data["time"].to_numpy()
    y = data[metric].to_numpy()

    sns.lineplot(data=data, x="time", y=metric, ax=ax, color=color, linewidth=3)
    if fill_positive_negative:
        ax.fill_between(x, 0, y, where=y < 0, interpolate=True, color="#2ca25f", alpha=0.35)
        ax.fill_between(x, 0, y, where=y > 0, interpolate=True, color="#de2d26", alpha=0.35)
        ax.axhline(0, color="#555555", linewidth=1.2)

    ax.set_xlabel("Time[s]", fontsize=14)
    ax.set_ylabel(ylabel, fontsize=14)
    ax.margins(x=0)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output, dpi=180, bbox_inches="tight")
    if close_after_save:
        plt.close(fig)


def plot_battery_data(
    data: pd.DataFrame,
    output_dir: Path,
    prefix: str = "battery",
    keep_open: bool = False,
) -> List[Path]:
    """Save the three battery plots and return their paths."""
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = [
        output_dir / f"{prefix}_{BATTERY_PLOT_FILENAMES['energyConsumed']}.png",
        output_dir / f"{prefix}_{BATTERY_PLOT_FILENAMES['totalEnergyConsumed']}.png",
        output_dir / f"{prefix}_{BATTERY_PLOT_FILENAMES['actualBatteryCapacity']}.png",
    ]

    save_single_plot(
        data,
        metric="energyConsumed",
        output=outputs[0],
        ylabel="Instant Energy Consumption [Wh]",
        color="#303030",
        fill_positive_negative=True,
        close_after_save=not keep_open,
    )
    save_single_plot(
        data,
        metric="totalEnergyConsumed",
        output=outputs[1],
        ylabel="Energy Consumed [Wh]",
        color="#c73e1d",
        close_after_save=not keep_open,
    )
    save_single_plot(
        data,
        metric="actualBatteryCapacity",
        output=outputs[2],
        ylabel="Battery[Wh]",
        color="#1f77b4",
        close_after_save=not keep_open,
    )

    return outputs


def generate_battery_plots(
    xml_file: Path,
    output_dir: Path,
    prefix: str = "battery",
    start: Optional[float] = None,
    end: Optional[float] = None,
    vehicle_id: Optional[str] = None,
    emission_xml_file: Optional[Path] = None,
    prefer_mmpevem: bool = False,
) -> List[Path]:
    """Load battery output and save the standard plots."""
    data = load_energy_data(
        xml_file,
        emission_xml_file=emission_xml_file,
        vehicle_id=vehicle_id,
        prefer_mmpevem=prefer_mmpevem,
    )
    data = filter_time_range(data, start, end)
    return plot_battery_data(data, output_dir, prefix)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot energyConsumed, totalEnergyConsumed and actualBatteryCapacity from battery.out.xml."
    )
    parser.add_argument(
        "xml_file",
        nargs="?",
        default="scenario/battery.out.xml",
        help="Path to the SUMO battery output XML file.",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default=".",
        help="Directory where the three plot images will be saved.",
    )
    parser.add_argument(
        "--emission-file",
        help="Optional SUMO emission-output XML file, used when plotting MMPEVEM.",
    )
    parser.add_argument(
        "--prefer-mmpevem",
        action="store_true",
        help="Read MMPEVEM electricity from --emission-file instead of battery output.",
    )
    parser.add_argument("--prefix", default="battery", help="Prefix for the output image filenames.")
    parser.add_argument("--start", type=float, help="Start time in seconds to include in the plots.")
    parser.add_argument("--end", type=float, help="End time in seconds to include in the plots.")
    parser.add_argument("--vehicle-id", help="Optional vehicle id to plot.")
    parser.add_argument("--show", action="store_true", help="Show the figure interactively after saving it.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    xml_file = Path(args.xml_file)
    output_dir = Path(args.output_dir)
    emission_file = Path(args.emission_file) if args.emission_file else None
    data = load_energy_data(
        xml_file,
        emission_xml_file=emission_file,
        vehicle_id=args.vehicle_id,
        prefer_mmpevem=args.prefer_mmpevem,
    )
    data = filter_time_range(data, args.start, args.end)
    outputs = plot_battery_data(data, output_dir, args.prefix, keep_open=args.show)

    if args.show:
        plt.show()

    print("Saved plots:")
    for output in outputs:
        print(f"  {output}")


if __name__ == "__main__":
    main()
