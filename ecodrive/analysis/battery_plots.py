"""Plot battery metrics from a SUMO battery-output XML file."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Optional, Union
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns


BATTERY_PLOT_FILENAMES = {
    "energyConsumed": "energy_consumed",
    "totalEnergyConsumed": "total_energy_consumed",
    "actualBatteryCapacity": "actual_battery_capacity",
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
        ylabel="Istant Energy Consumption [Wh]",
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
) -> List[Path]:
    """Load battery output and save the standard plots."""
    data = load_battery_data(xml_file, vehicle_id)
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
    data = load_battery_data(xml_file, args.vehicle_id)
    data = filter_time_range(data, args.start, args.end)
    outputs = plot_battery_data(data, output_dir, args.prefix, keep_open=args.show)

    if args.show:
        plt.show()

    print("Saved plots:")
    for output in outputs:
        print(f"  {output}")


if __name__ == "__main__":
    main()
