import streamlit as st
import streamlit.components.v1 as components
import requests
import pandas as pd
import folium
import matplotlib.pyplot as plt
import seaborn as sns
import heapq
import math
import time
import json
import re
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from streamlit_folium import st_folium
from urllib.parse import quote

from ecodrive.analysis.battery_plots import generate_battery_plots
from ecodrive.scenario.sumo_route_tools import (
    AUTOWARE_EGO_VTYPE,
    DEFAULT_CARLA_HOST,
    DEFAULT_CARLA_PORT,
    DEFAULT_MAP,
    DEFAULT_EGO_BATTERY_CAPACITY,
    DEFAULT_EGO_BLUEPRINT,
    DEFAULT_VEHICLE_TYPE,
    EGO_SUMO_VTYPE,
    ENERGY_EMISSION_CLASS,
    MMPEVEM_EMISSION_CLASS,
    active_carla_version,
    available_carla_versions,
    available_maps,
    available_carla_vehicle_types,
    available_vehicle_types,
    build_run_command,
    current_sumo_dir,
    ego_emission_class_value,
    ego_model_defaults,
    edge_label,
    edge_direction_options,
    generate_congestion_scenario,
    generate_random_trips_scenario,
    carla_server_status,
    is_carla_server_ready,
    load_carla_map,
    nearest_edge,
    opposite_edge_id,
    publish_autoware_route_in_container,
    read_ego_vtype_config,
    read_autoware_ego_vtype_config,
    read_sumo_edges,
    set_active_carla_version,
    start_carla_server,
    start_synchronization,
    stop_carla_server,
    launch_autoware_carla_in_container,
    write_ego_vtype_config,
    write_autoware_ego_vtype_config,
)

API_URL = "http://localhost:5000"
AUTOWARE_ALLOWED_MAPS = ("Town01", "Town04", "Town05")
STEP_CARLA_LABEL = "1. Start CARLA"
STEP_ROUTES_LABEL = "2. Generate SUMO Routes"
STEP_PLOT_OUTPUT_LABEL = "4. Plot Output"
STEP_MONITORING_LABEL = "5. Monitoring"
EGO_ROUTE_VIA = "Pass through congested edge"
EGO_ROUTE_FROM_CONGESTION = "Use congested edge as start"
EGO_ROUTE_TO_CONGESTION = "Use congested edge as destination"
EDGE_SELECTION_MAP_HEIGHT = 680
BASE_EGO_COLORS = [
    "white",
    "black",
    "gray",
    "silver",
    "red",
    "green",
    "blue",
    "yellow",
    "orange",
    "cyan",
    "magenta",
]
SHOW_TRAFFIC_SIMULATION_END_INPUT = False
SHOW_TRAFFIC_SPAWN_DISTRIBUTION_INPUT = False
SHOW_SIMULATION_STEP_TAB = False
SHOW_AUTOWARE_VEHICLE_TYPE_INPUT = False
SHOW_AUTOWARE_SPEED_CAP_INPUT = False
SHOW_SUMO_GUI_INPUT = False
SHOW_AUTOWARE_STARTUP_WAIT_INPUT = False
SHOW_MONITORING_TAB = False
EGO_NUMERIC_LIMITS = {
    "minGap": (0.0, 20.0, 0.1),
    "maxSpeed": (0.1, 100.0, 0.1),
    "accel": (0.01, 20.0, 0.1),
    "decel": (0.01, 20.0, 0.1),
    "sigma": (0.0, 1.0, 0.01),
    "mass": (1.0, 100000.0, 10.0),
    "actionStepLength": (0.01, 10.0, 0.1),
    "airDragCoefficient": (0.0, 2.0, 0.01),
    "constantPowerIntake": (0.0, 100000.0, 10.0),
    "frontSurfaceArea": (0.1, 20.0, 0.01),
    "rotatingMass": (0.0, 10000.0, 1.0),
    "maximumPower": (0.0, 1000000.0, 1000.0),
    "propulsionEfficiency": (0.0, 1.0, 0.01),
    "radialDragCoefficient": (0.0, 2.0, 0.01),
    "recuperationEfficiency": (0.0, 1.0, 0.01),
    "rollDragCoefficient": (0.0, 1.0, 0.001),
    "stoppingThreshold": (0.0, 10.0, 0.1),
    "vehicleMass": (1.0, 100000.0, 10.0),
    "wheelRadius": (0.01, 5.0, 0.001),
    "internalMomentOfInertia": (0.0, 10000.0, 0.1),
    "gearRatio": (0.01, 100.0, 0.01),
    "gearEfficiency": (0.0, 1.0, 0.01),
    "maximumTorque": (0.0, 10000.0, 1.0),
    "maximumRecuperationTorque": (0.0, 10000.0, 1.0),
    "maximumRecuperationPower": (0.0, 1000000.0, 1000.0),
    "internalBatteryResistance": (0.0001, 10.0, 0.0001),
    "nominalBatteryVoltage": (1.0, 2000.0, 1.0),
}

st.set_page_config(layout="wide", initial_sidebar_state="collapsed")


def preserve_scroll_position(storage_key="dashboard_scroll_position"):
    """Keep the dashboard scroll position stable across Streamlit reruns."""
    components.html(
        f"""
        <script>
        (() => {{
            const storageKey = {storage_key!r};
            const parentWindow = window.parent;
            const parentDocument = parentWindow.document;
            const handlerKey = `__streamlit_scroll_keeper_${{storageKey}}`;

            function isScrollable(element) {{
                if (!element) {{
                    return false;
                }}

                const style = parentWindow.getComputedStyle(element);
                return /(auto|scroll)/.test(style.overflowY)
                    && element.scrollHeight > element.clientHeight;
            }}

            function findScroller() {{
                const selectors = [
                    '[data-testid="stAppViewContainer"]',
                    'section.main',
                    '.main',
                ];

                for (const selector of selectors) {{
                    for (const element of parentDocument.querySelectorAll(selector)) {{
                        if (isScrollable(element)) {{
                            return element;
                        }}
                    }}
                }}

                for (const element of parentDocument.querySelectorAll('body *')) {{
                    if (isScrollable(element)) {{
                        return element;
                    }}
                }}

                return parentDocument.scrollingElement || parentDocument.documentElement;
            }}

            function getScrollY(scroller) {{
                if (
                    scroller === parentDocument.scrollingElement
                    || scroller === parentDocument.documentElement
                    || scroller === parentDocument.body
                ) {{
                    return parentWindow.scrollY || scroller.scrollTop || 0;
                }}

                return scroller.scrollTop || 0;
            }}

            function setScrollY(scroller, y) {{
                if (
                    scroller === parentDocument.scrollingElement
                    || scroller === parentDocument.documentElement
                    || scroller === parentDocument.body
                ) {{
                    parentWindow.scrollTo(0, y);
                }}

                scroller.scrollTop = y;
            }}

            const previousHandler = parentWindow[handlerKey];
            if (previousHandler) {{
                previousHandler.scroller.removeEventListener(
                    'scroll',
                    previousHandler.onScroll
                );
                parentWindow.removeEventListener(
                    'beforeunload',
                    previousHandler.save
                );
                parentWindow.removeEventListener('pagehide', previousHandler.save);
            }}

            const scroller = findScroller();
            const savedY = Number(parentWindow.sessionStorage.getItem(storageKey) || 0);

            if (savedY > 0) {{
                let attempts = 0;
                const restore = () => {{
                    setScrollY(scroller, savedY);
                    attempts += 1;

                    if (attempts < 60 && Math.abs(getScrollY(scroller) - savedY) > 2) {{
                        parentWindow.requestAnimationFrame(restore);
                    }}
                }};

                parentWindow.requestAnimationFrame(restore);
            }}

            let ticking = false;
            const save = () => {{
                parentWindow.sessionStorage.setItem(
                    storageKey,
                    String(getScrollY(scroller))
                );
                ticking = false;
            }};
            const onScroll = () => {{
                if (!ticking) {{
                    parentWindow.requestAnimationFrame(save);
                    ticking = true;
                }}
            }};

            scroller.addEventListener('scroll', onScroll, {{ passive: true }});
            parentWindow.addEventListener('beforeunload', save);
            parentWindow.addEventListener('pagehide', save);
            parentWindow[handlerKey] = {{ scroller, onScroll, save }};
        }})();
        </script>
        """,
        height=0,
    )

st.title("🚗 E-CoDrive")

# =====================================================
# BACKEND CHECK
# =====================================================

def is_backend_alive():
    """Check whether the dashboard backend API is reachable."""
    try:
        r = requests.get(f"{API_URL}/state", timeout=0.5)
        return r.status_code == 200
    except:
        return False


backend_alive = is_backend_alive()

if not backend_alive:
    st.warning(
        # "Backend non attivo. Puoi comunque generare lo scenario traffico "
        # "e avviare la co-simulazione dal tab dedicato."
    "Backend is not active. You can still generate the traffic scenario and start the co-simulation from the dedicated tab. "
    )
else:
    st.success("🟢 Backend connected")

# =====================================================
# SESSION STATE
# =====================================================

for key, default in {
    "start_edge": None,
    "end_edge": None,
    "battery_data": [],
    "monitoring_samples": [],
    "monitoring_summary": None,
    "monitoring_export_paths": None,
    "plot_output_paths": None,
    "plot_output_source": None,
    "monitoring_session_token": None,
    "monitoring_saved_session_token": None,
    "monitoring": False,
    "map_ready": False,
    "refresh_rate": 1000,
    "last_monitoring_poll": 0.0,
    "latest_monitoring_state": None,
    "monitoring_events": [],
    "dashboard_events": [],
    "vehicle_terminal_event": None,
    "last_vehicle_state": None,
    "monitoring_started_at": None,
    "traffic_map_name": DEFAULT_MAP,
    "runtime_map_selection": DEFAULT_MAP,
    "traffic_target_edge": "",
    "traffic_destination_edge": "",
    "traffic_source_edge": "",
    "traffic_last_click": None,
    "traffic_clicked_direction": "",
    "traffic_generation_result": None,
    "traffic_process": None,
    "traffic_process_log": None,
    "carla_process": None,
    "carla_process_log": None,
    "carla_server_action_level": None,
    "carla_server_action_message": None,
    "traffic_carla_mode": None,
    "traffic_carla_mode_selection": None,
    "traffic_carla_timeout_seconds": 120,
    "traffic_carla_timeout_selection": None,
    "traffic_sumo_gui": True,
    "traffic_sumo_gui_selection": None,
    "traffic_waiting_for_autoware": False,
    "traffic_start_gate_file": None,
    "selected_carla_version": None,
    "applied_carla_version": None,
    "applied_runtime_map": None,
    "ego_clicked_direction": "",
    "ego_use_traffic_route": False,
    "ego_traffic_route_mode": EGO_ROUTE_VIA,
    "ego_active_start_edge": None,
    "ego_active_end_edge": None,
    "ego_active_via_edge": None,
    "ego_use_farthest_route": False,
    "ego_config_loaded": False,
    "ego_config_version": None,
    "ego_carla_blueprint": DEFAULT_EGO_BLUEPRINT,
    "ego_emission_model": ENERGY_EMISSION_CLASS,
    "ego_last_emission_model": ENERGY_EMISSION_CLASS,
    "ego_max_battery_capacity": DEFAULT_EGO_BATTERY_CAPACITY,
    "ego_initial_battery": min(5000.0, DEFAULT_EGO_BATTERY_CAPACITY),
    "ego_battery_failure_threshold": 0.0,
    "ego_active_battery_failure_threshold": 0.0,
    "autoware_ego_config_loaded": False,
    "autoware_ego_config_version": None,
    "autoware_ego_carla_blueprint": AUTOWARE_EGO_VTYPE,
    "autoware_ego_emission_model": ENERGY_EMISSION_CLASS,
    "autoware_ego_last_emission_model": ENERGY_EMISSION_CLASS,
    "autoware_ego_max_battery_capacity": DEFAULT_EGO_BATTERY_CAPACITY,
    "autoware_ego_initial_battery": min(5000.0, DEFAULT_EGO_BATTERY_CAPACITY),
    "autoware_battery_failure_threshold": 0.0,
    "autoware_active_battery_failure_threshold": 0.0,
    "autoware_start_edge": None,
    "autoware_goal_edge": None,
    "autoware_use_farthest_route": False,
    "autoware_start_edge_selection": "",
    "autoware_goal_edge_selection": "",
    "autoware_pending_start_edge_selection": None,
    "autoware_pending_goal_edge_selection": None,
    "autoware_last_click": None,
    "autoware_clicked_direction": "",
    "autoware_planner_speed_limit_kmh": 50.0,
    "autoware_sync_delay_seconds": 10,
    "autoware_sync_delay_selection": None,
    "autoware_last_launch": None,
    "traffic_simulation_end": 420.0,
    "monitor_vehicle_selected_id": None,
    "monitor_vehicle_loaded_for": None,
    "live_vehicle_selected_id": None,
    "live_vehicle_config_loaded_for": None,
    "live_vehicle_config_version": None,
    "live_vehicle_carla_blueprint": DEFAULT_EGO_BLUEPRINT,
    "live_vehicle_emission_model": ENERGY_EMISSION_CLASS,
    "live_vehicle_last_emission_model": ENERGY_EMISSION_CLASS,
    "live_vehicle_max_battery_capacity": DEFAULT_EGO_BATTERY_CAPACITY,
    "live_vehicle_has_battery_device": True,
}.items():
    if key not in st.session_state:
        st.session_state[key] = default

for emission_state_key in (
    "ego_emission_model",
    "ego_last_emission_model",
    "autoware_ego_emission_model",
    "autoware_ego_last_emission_model",
    "live_vehicle_emission_model",
    "live_vehicle_last_emission_model",
):
    if st.session_state[emission_state_key] not in {ENERGY_EMISSION_CLASS, MMPEVEM_EMISSION_CLASS}:
        st.session_state[emission_state_key] = ENERGY_EMISSION_CLASS


def apply_selected_carla_version():
    """Apply the CARLA version currently selected in the UI state."""
    versions = available_carla_versions()
    if not versions:
        st.error("No supported CARLA installation found in the project root.")
        st.stop()

    if st.session_state.selected_carla_version not in versions:
        if "0.9.13" in versions:
            default_version = "0.9.13"
        else:
            default_version = active_carla_version()
            if default_version not in versions:
                default_version = versions[-1]
        st.session_state.selected_carla_version = default_version

    control_cols = st.columns([1.2, 2.9, 1.3])
    with control_cols[0]:
        selected_version = st.selectbox(
            "CARLA version",
            options=versions,
            key="selected_carla_version",
        )

    previous_version = st.session_state.applied_carla_version
    set_active_carla_version(selected_version)
    if previous_version not in (None, selected_version):
        st.session_state.traffic_generation_result = None
        st.session_state.ego_config_loaded = False
        st.session_state.ego_config_version = None
        st.session_state.autoware_ego_config_loaded = False
        st.session_state.autoware_ego_config_version = None
        st.session_state.autoware_active_battery_failure_threshold = 0.0
        st.session_state.live_vehicle_config_loaded_for = None
        st.session_state.live_vehicle_config_version = None
        st.session_state.autoware_last_launch = None
        gate_file = st.session_state.get("traffic_start_gate_file")
        if gate_file:
            try:
                Path(gate_file).unlink(missing_ok=True)
            except OSError:
                pass
        st.session_state.traffic_waiting_for_autoware = False
        st.session_state.traffic_start_gate_file = None

    st.session_state.applied_carla_version = selected_version
    server_status = carla_server_status(selected_version)

    with control_cols[1]:
        st.caption(
            "Generated SUMO files, Town loading, and co-simulation startup use the selected "
            f"CARLA installation for version {selected_version}."
        )
        if server_status["running"]:
            detected_version = server_status.get("detected_version")
            if detected_version:
                st.caption(
                    f"CARLA server active on `{DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}` "
                    f"from version `{detected_version}`."
                )
            elif server_status.get("external"):
                st.caption(
                    f"CARLA server active on `{DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}`, "
                    "but not mapped to a local supported installation."
                )
            else:
                st.caption(
                    f"CARLA process detected. Server readiness on "
                    f"`{DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}`: "
                    f"`{'ready' if server_status['ready'] else 'starting'}`."
                )
        else:
            st.caption(
                f"CARLA server inactive on `{DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}`."
            )

    with control_cols[2]:
        toggle_label = (
            "Kill CARLA"
            if server_status["running"]
            else f"Run CARLA {selected_version}"
        )
        if st.button(toggle_label, key="toggle_carla_server", use_container_width=True):
            try:
                if server_status["running"]:
                    stop_version = server_status.get("detected_version")
                    stopped = stop_carla_server(stop_version)
                    st.session_state.carla_process = None
                    if stopped.get("port_closed"):
                        if stop_version:
                            message = (
                                f"CARLA {stop_version} stopped "
                                f"({len(stopped.get('stopped_pids', []))} process(es))."
                            )
                        else:
                            message = "CARLA stopped."
                        level = "warning"
                    else:
                        message = (
                            "Local CARLA processes were terminated, but port 2000 is still open."
                        )
                        level = "error"
                else:
                    selected_map = st.session_state.traffic_map_name
                    launch = start_carla_server(selected_version, map_name=selected_map)
                    st.session_state.traffic_map_name = selected_map
                    st.session_state.runtime_map_selection = selected_map
                    st.session_state.applied_runtime_map = selected_map
                    st.session_state.carla_process = launch["process"]
                    st.session_state.carla_process_log = launch["log_file"]
                    message = (
                        f"CARLA {launch['version']} started, PID {launch['process'].pid}, "
                        f"Town `{selected_map}` loaded."
                    )
                    level = "success"
                st.session_state.carla_server_action_message = message
                st.session_state.carla_server_action_level = level
                st.rerun()
            except Exception as exc:
                st.error(f"CARLA server action failed: {exc}")

    event_message = st.session_state.carla_server_action_message
    event_level = st.session_state.carla_server_action_level
    if event_message:
        if event_level == "success":
            st.success(event_message)
        elif event_level == "warning":
            st.warning(event_message)
        elif event_level == "error":
            st.error(event_message)
        else:
            st.info(event_message)
        st.session_state.carla_server_action_message = None
        st.session_state.carla_server_action_level = None

    return selected_version


apply_selected_carla_version()


def format_elapsed(seconds):
    """Format an elapsed duration for the monitoring UI."""
    if seconds is None:
        return "-"

    minutes, seconds = divmod(max(0, int(seconds)), 60)
    return f"{minutes:02d}:{seconds:02d}"


def extract_state_time(state):
    """Extract the most useful timestamp from a backend state payload."""
    if not state:
        return None

    for key in ("time", "sim_time", "simulation_time", "timestamp"):
        value = state.get(key)
        if value is not None:
            return value

    return None


def _coerce_float(value):
    """Convert a value to float when possible."""
    try:
        if value is None or value == "":
            return None
        number = float(value)
        if not math.isfinite(number):
            return None
        return number
    except (TypeError, ValueError):
        return None


def _safe_filename_fragment(value):
    """Sanitize free-form text so it is safe to use in filenames."""
    text = str(value or "monitoring").strip()
    normalized = [
        char if char.isalnum() or char in {"-", "_", "."} else "_"
        for char in text
    ]
    return "".join(normalized).strip("._") or "monitoring"


def monitoring_output_dir():
    """Return the output directory used for persisted monitoring sessions."""
    base_dir = current_sumo_dir() or Path(__file__).resolve().parent
    output_dir = Path(base_dir) / "output" / "monitoring"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def sumo_runtime_output_dir():
    """Return the SUMO runtime output directory for the selected CARLA installation."""
    base_dir = current_sumo_dir()
    if base_dir is None:
        return Path(__file__).resolve().parent / "output"
    return Path(base_dir) / "examples" / "output"


def battery_output_path():
    """Return the active SUMO battery output path."""
    return sumo_runtime_output_dir() / "battery.out.xml"


def is_sumo_simulation_running():
    """Return whether the dashboard-launched SUMO/CARLA process is still active."""
    sync_process = st.session_state.get("traffic_process")
    return bool(sync_process is not None and sync_process.poll() is None)


def plot_output_available():
    """Return whether battery plots can be generated without reading a live SUMO file."""
    path = battery_output_path()
    if not path.exists() or path.stat().st_size == 0 or is_sumo_simulation_running():
        return False
    return "<vehicle" in _read_text(path)


def render_plot_output():
    """Generate and render battery-output plots for the latest completed SUMO run."""
    st.subheader("📈 Plot Output")

    if is_sumo_simulation_running():
        st.info("Plot output will be available after the SUMO/CARLA simulation process stops.")
        return

    source_path = battery_output_path()
    if not source_path.exists() or source_path.stat().st_size == 0 or "<vehicle" not in _read_text(source_path):
        st.warning("No completed battery vehicle records are available yet.")
        st.caption(f"Expected source: `{source_path}`")
        return

    output_dir = sumo_runtime_output_dir() / "plots"
    try:
        plot_paths = generate_battery_plots(
            source_path,
            output_dir,
            prefix="battery",
        )
    except Exception as exc:
        st.error(f"Could not generate battery plots: {exc}")
        st.caption(f"Battery source: `{source_path}`")
        return

    st.session_state.plot_output_paths = [str(path) for path in plot_paths]
    st.session_state.plot_output_source = str(source_path)

    st.caption(f"Battery source: `{source_path}`")
    st.caption(f"Plots saved in `{output_dir}`.")

    captions = [
        "Energy consumed per timestep",
        "Total energy consumed",
        "Actual battery capacity",
    ]
    for plot_path, caption in zip(plot_paths, captions):
        st.image(str(plot_path), caption=caption, use_container_width=True)


def _format_number(value, unit="", precision=1):
    """Format numeric dashboard values consistently."""
    number = _coerce_float(value)
    if number is None:
        return "-"
    suffix = f" {unit}" if unit else ""
    return f"{number:.{precision}f}{suffix}"


def _plot_axis_dataframe(dataframe, y_column, preferred_x_columns=("sim_time", "elapsed_seconds")):
    """Build a clean dataframe for a line plot with a stable x axis."""
    if dataframe is None or dataframe.empty or y_column not in dataframe:
        return pd.DataFrame(), "Sample", "sample_index"

    plot_df = dataframe.copy()
    x_label = "Sample"
    x_column = "sample_index"

    for candidate in preferred_x_columns:
        if candidate in plot_df and not plot_df[candidate].dropna().empty:
            x_column = candidate
            if candidate in {"sim_time", "time_s"}:
                x_label = "Simulation time [s]"
            elif candidate == "elapsed_seconds":
                x_label = "Elapsed time [s]"
            else:
                x_label = candidate
            break
    else:
        plot_df[x_column] = range(len(plot_df))

    plot_df = plot_df[[x_column, y_column]].copy()
    plot_df[x_column] = pd.to_numeric(plot_df[x_column], errors="coerce")
    plot_df[y_column] = pd.to_numeric(plot_df[y_column], errors="coerce")
    plot_df = plot_df.dropna(subset=[x_column, y_column])
    return plot_df, x_label, x_column


def build_line_figure(
    dataframe,
    x_column,
    y_column,
    title,
    x_label,
    y_label,
    color="#2563eb",
):
    """Build a Matplotlib/Seaborn line figure for dashboard display or export."""
    sns.set_theme(style="whitegrid")
    fig, ax = plt.subplots(figsize=(9, 3.6), dpi=140)
    sns.lineplot(
        data=dataframe,
        x=x_column,
        y=y_column,
        ax=ax,
        color=color,
        linewidth=2.2,
        marker="o" if len(dataframe) <= 40 else None,
        markersize=4,
    )
    ax.set_title(title, fontsize=12, fontweight="bold", pad=10)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.grid(True, alpha=0.35)
    fig.tight_layout()
    return fig


def render_line_figure(
    container,
    dataframe,
    y_column,
    title,
    y_label,
    color="#2563eb",
    preferred_x_columns=("sim_time", "elapsed_seconds"),
):
    """Render a labelled monitoring line chart in a Streamlit placeholder."""
    plot_df, x_label, x_column = _plot_axis_dataframe(
        dataframe,
        y_column,
        preferred_x_columns=preferred_x_columns,
    )
    if plot_df.empty:
        return None

    fig = build_line_figure(plot_df, x_column, y_column, title, x_label, y_label, color)
    container.pyplot(fig, clear_figure=True)
    plt.close(fig)
    return plot_df


def save_line_figure(
    dataframe,
    path,
    y_column,
    title,
    y_label,
    color="#2563eb",
    preferred_x_columns=("sim_time", "elapsed_seconds"),
):
    """Save a labelled line chart to disk and return the path when data exists."""
    plot_df, x_label, x_column = _plot_axis_dataframe(
        dataframe,
        y_column,
        preferred_x_columns=preferred_x_columns,
    )
    if plot_df.empty:
        return None

    fig = build_line_figure(plot_df, x_column, y_column, title, x_label, y_label, color)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return path


def _read_text(path):
    """Read a text file while tolerating files that SUMO is still writing."""
    try:
        return Path(path).read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def _iter_xml_fragments(path, tag_name):
    """Yield XML fragments even when the surrounding output document is incomplete."""
    text = _read_text(path)
    if not text:
        return

    open_tag = rf"<{tag_name}(?=[\s>/])[^>]*"
    pattern = re.compile(
        rf"{open_tag}/>|{open_tag}>.*?</{tag_name}>",
        re.DOTALL,
    )
    for match in pattern.finditer(text):
        try:
            yield ET.fromstring(match.group(0))
        except ET.ParseError:
            continue


def _float_attribute(element, *attribute_names):
    """Return the first parseable float attribute from an XML element."""
    for attribute_name in attribute_names:
        value = element.get(attribute_name)
        number = _coerce_float(value)
        if number is not None:
            return number
    return None


def _pick_vehicle_id(records_by_vehicle, vehicle_id=None, fallback_vehicle_id=None):
    """Pick the best vehicle id from an output file grouped by vehicle id."""
    for candidate in (vehicle_id, fallback_vehicle_id):
        if candidate is None:
            continue
        candidate = str(candidate)
        if candidate in records_by_vehicle:
            return candidate

    if len(records_by_vehicle) == 1:
        return next(iter(records_by_vehicle))

    return None


def read_sumo_battery_output(vehicle_id=None, fallback_vehicle_id=None):
    """Read the battery output curve produced by SUMO for a completed vehicle trip."""
    path = sumo_runtime_output_dir() / "battery.out.xml"
    records_by_vehicle = {}

    for timestep in _iter_xml_fragments(path, "timestep") or []:
        time_s = _float_attribute(timestep, "time")
        for vehicle in timestep.findall("vehicle"):
            record_vehicle_id = vehicle.get("id")
            if not record_vehicle_id:
                continue

            record = {
                "time_s": time_s,
                "battery_charge_wh": _float_attribute(
                    vehicle,
                    "actualBatteryCapacity",
                    "chargeLevel",
                    "batteryChargeLevel",
                    "device.battery.chargeLevel",
                ),
                "step_consumption_wh": _float_attribute(
                    vehicle,
                    "energyConsumed",
                    "energyConsumedByVehicle",
                ),
                "total_energy_consumed_wh": _float_attribute(
                    vehicle,
                    "totalEnergyConsumed",
                ),
                "total_energy_regenerated_wh": _float_attribute(
                    vehicle,
                    "totalEnergyRegenerated",
                ),
                "step_charged_wh": _float_attribute(
                    vehicle,
                    "energyCharged",
                    "energyChargedIntoBattery",
                    "energyChargedInTransit",
                    "energyChargedStopped",
                ),
                "maximum_battery_capacity_wh": _float_attribute(
                    vehicle,
                    "maximumBatteryCapacity",
                    "maxBatteryCapacity",
                ),
            }
            records_by_vehicle.setdefault(str(record_vehicle_id), []).append(record)

    selected_vehicle_id = _pick_vehicle_id(
        records_by_vehicle,
        vehicle_id=vehicle_id,
        fallback_vehicle_id=fallback_vehicle_id,
    )
    if selected_vehicle_id is None:
        return {
            "path": str(path),
            "vehicle_id": None,
            "records": [],
            "vehicle_ids": sorted(records_by_vehicle),
            "total_energy_wh": None,
        }

    records = sorted(
        records_by_vehicle.get(selected_vehicle_id, []),
        key=lambda item: item["time_s"] if item["time_s"] is not None else -1.0,
    )
    initial_charge = next(
        (
            record["battery_charge_wh"]
            for record in records
            if record["battery_charge_wh"] is not None
        ),
        None,
    )
    cumulative_consumption = 0.0
    has_cumulative_consumption = False

    for record in records:
        if initial_charge is not None and record["battery_charge_wh"] is not None:
            record["cumulative_consumption_wh"] = max(
                0.0,
                initial_charge - record["battery_charge_wh"],
            )
            has_cumulative_consumption = True
        elif record["total_energy_consumed_wh"] is not None:
            regenerated_wh = record["total_energy_regenerated_wh"] or 0.0
            record["cumulative_consumption_wh"] = max(
                0.0,
                float(record["total_energy_consumed_wh"]) - float(regenerated_wh),
            )
            has_cumulative_consumption = True
        elif record["step_consumption_wh"] is not None:
            cumulative_consumption += float(record["step_consumption_wh"])
            if record["step_charged_wh"] is not None:
                cumulative_consumption -= float(record["step_charged_wh"])
            record["cumulative_consumption_wh"] = cumulative_consumption
            has_cumulative_consumption = True
        else:
            record["cumulative_consumption_wh"] = None

    total_energy_wh = next(
        (
            record["cumulative_consumption_wh"]
            for record in reversed(records)
            if record["cumulative_consumption_wh"] is not None
        ),
        None,
    )

    return {
        "path": str(path),
        "vehicle_id": selected_vehicle_id,
        "records": records if has_cumulative_consumption else [],
        "vehicle_ids": sorted(records_by_vehicle),
        "total_energy_wh": total_energy_wh,
    }


def read_sumo_tripinfo_output(vehicle_id=None, fallback_vtypes=None):
    """Read the completed tripinfo entry for the monitored vehicle."""
    path = sumo_runtime_output_dir() / "tripinfos.xml"
    if fallback_vtypes is None:
        fallback_vtypes = (AUTOWARE_EGO_VTYPE, EGO_SUMO_VTYPE)
    elif isinstance(fallback_vtypes, str):
        fallback_vtypes = (fallback_vtypes,)

    tripinfos = []

    for tripinfo in _iter_xml_fragments(path, "tripinfo") or []:
        data = dict(tripinfo.attrib)
        emissions = tripinfo.find("emissions")
        if emissions is not None:
            for key, value in emissions.attrib.items():
                data[f"emissions_{key}"] = value
        tripinfos.append(data)

    selected_tripinfo = None
    requested_vehicle_id = str(vehicle_id) if vehicle_id is not None else None
    if requested_vehicle_id:
        selected_tripinfo = next(
            (tripinfo for tripinfo in tripinfos if tripinfo.get("id") == requested_vehicle_id),
            None,
        )

    if selected_tripinfo is None and fallback_vtypes:
        matching_vtype = [
            tripinfo
            for tripinfo in tripinfos
            if tripinfo.get("vType") in fallback_vtypes
        ]
        if matching_vtype:
            selected_tripinfo = max(
                matching_vtype,
                key=lambda tripinfo: _coerce_float(tripinfo.get("arrival")) or -1.0,
            )

    if selected_tripinfo is None:
        carla_tripinfos = [
            tripinfo
            for tripinfo in tripinfos
            if str(tripinfo.get("id", "")).startswith("carla")
        ]
        if len(carla_tripinfos) == 1:
            selected_tripinfo = carla_tripinfos[0]

    return {
        "path": str(path),
        "tripinfo": selected_tripinfo,
        "vehicle_ids": [tripinfo.get("id") for tripinfo in tripinfos if tripinfo.get("id")],
    }


def completed_trip_output(selected_vehicle_id=None):
    """Find the most likely completed trip output for the current monitoring target."""
    summary = st.session_state.get("monitoring_summary") or {}
    last_state = st.session_state.get("last_vehicle_state") or {}
    candidates = [
        selected_vehicle_id,
        summary.get("vehicle_id"),
        last_state.get("vehicle_id"),
        st.session_state.get("monitor_vehicle_selected_id"),
    ]

    seen = set()
    for candidate in candidates:
        if candidate is None:
            continue
        candidate = str(candidate)
        if candidate in seen:
            continue
        seen.add(candidate)
        output = read_sumo_tripinfo_output(candidate)
        if output.get("tripinfo"):
            return output

    return read_sumo_tripinfo_output(None)


def mark_summary_arrived_from_tripinfo(tripinfo):
    """Update the in-memory monitoring summary when SUMO confirms arrival."""
    if not tripinfo:
        return

    summary = st.session_state.get("monitoring_summary")
    if not summary:
        return

    summary["arrived"] = True
    summary["reason"] = "destination_reached"
    summary["terminal_event"] = summary.get("terminal_event") or "destination_reached"
    summary["distance_remaining_m"] = 0.0

    route_length = _coerce_float(tripinfo.get("routeLength"))
    if route_length is not None:
        summary["distance_travelled_m"] = route_length


def infer_arrival_from_sumo_tripinfo(vehicle_id=None):
    """Infer destination arrival from SUMO tripinfo output after a vehicle disappears."""
    trip_output = completed_trip_output(vehicle_id)
    tripinfo = trip_output.get("tripinfo")
    if not tripinfo:
        return False

    mark_summary_arrived_from_tripinfo(tripinfo)
    state = dict(st.session_state.last_vehicle_state or {})
    state.update(
        {
            "vehicle_id": tripinfo.get("id") or vehicle_id or state.get("vehicle_id"),
            "edge": tripinfo.get("arrivalLane") or state.get("edge"),
            "distance_remaining_m": 0.0,
            "sim_time": tripinfo.get("arrival"),
        }
    )
    record_dashboard_event("destination_reached", state)
    return True


def append_monitoring_sample(state):
    """Append a live monitoring sample to the current session buffer."""
    if not state:
        return

    wall_timestamp = time.time()
    started_at = st.session_state.monitoring_started_at
    elapsed_seconds = (
        max(0.0, wall_timestamp - started_at)
        if started_at is not None
        else None
    )

    sample = {
        "vehicle_id": state.get("vehicle_id"),
        "edge": state.get("edge"),
        "wall_timestamp": wall_timestamp,
        "wall_time": datetime.fromtimestamp(wall_timestamp).isoformat(timespec="seconds"),
        "elapsed_seconds": elapsed_seconds,
        "sim_time": _coerce_float(extract_state_time(state)),
        "speed_mps": _coerce_float(state.get("speed")),
        "battery_wh": _coerce_float(state.get("battery")),
        "total_energy_consumed_wh": _coerce_float(state.get("total_energy_consumed_wh")),
        "distance_travelled_m": _coerce_float(state.get("distance_travelled_m")),
        "distance_remaining_m": _coerce_float(state.get("distance_remaining_m")),
        "route_final_edge": state.get("route_final_edge"),
    }

    samples = st.session_state.monitoring_samples
    if samples and sample["sim_time"] is not None:
        last_sim_time = samples[-1].get("sim_time")
        if last_sim_time is not None and abs(last_sim_time - sample["sim_time"]) < 1e-9:
            samples[-1] = sample
            return

    samples.append(sample)


def build_monitoring_summary(reason=None):
    """Build a summary from the current monitoring session."""
    samples = st.session_state.monitoring_samples
    if not samples:
        return None

    dataframe = pd.DataFrame(samples)
    vehicle_id = (
        dataframe["vehicle_id"].dropna().iloc[-1]
        if "vehicle_id" in dataframe and not dataframe["vehicle_id"].dropna().empty
        else st.session_state.monitor_vehicle_selected_id
    )
    terminal_event = st.session_state.vehicle_terminal_event or {}
    terminal_kind = terminal_event.get("kind")

    speed_series = dataframe["speed_mps"].dropna() if "speed_mps" in dataframe else pd.Series(dtype=float)
    battery_series = dataframe["battery_wh"].dropna() if "battery_wh" in dataframe else pd.Series(dtype=float)
    total_energy_series = (
        dataframe["total_energy_consumed_wh"].dropna()
        if "total_energy_consumed_wh" in dataframe
        else pd.Series(dtype=float)
    )
    travelled_series = (
        dataframe["distance_travelled_m"].dropna()
        if "distance_travelled_m" in dataframe
        else pd.Series(dtype=float)
    )
    remaining_series = (
        dataframe["distance_remaining_m"].dropna()
        if "distance_remaining_m" in dataframe
        else pd.Series(dtype=float)
    )

    distance_travelled_m = float(travelled_series.iloc[-1]) if not travelled_series.empty else None
    distance_remaining_m = float(remaining_series.iloc[-1]) if not remaining_series.empty else None
    battery_initial_wh = float(battery_series.iloc[0]) if not battery_series.empty else None
    battery_final_wh = float(battery_series.iloc[-1]) if not battery_series.empty else None
    battery_consumed_wh = (
        battery_initial_wh - battery_final_wh
        if battery_initial_wh is not None and battery_final_wh is not None
        else None
    )
    if battery_consumed_wh is None and not total_energy_series.empty:
        battery_consumed_wh = float(total_energy_series.iloc[-1] - total_energy_series.iloc[0])
    average_consumption_wh_per_km = (
        battery_consumed_wh / (distance_travelled_m / 1000.0)
        if battery_consumed_wh is not None
        and distance_travelled_m is not None
        and distance_travelled_m > 0.0
        else None
    )

    speed_min_kmh = float(speed_series.min() * 3.6) if not speed_series.empty else None
    speed_max_kmh = float(speed_series.max() * 3.6) if not speed_series.empty else None
    speed_avg_kmh = float(speed_series.mean() * 3.6) if not speed_series.empty else None

    arrived = (
        terminal_kind == "destination_reached"
        or (distance_remaining_m is not None and distance_remaining_m <= 1.0)
    )

    return {
        "vehicle_id": vehicle_id,
        "reason": reason or terminal_kind or "stopped",
        "sample_count": int(len(samples)),
        "wall_time_started": (
            datetime.fromtimestamp(st.session_state.monitoring_started_at).isoformat(timespec="seconds")
            if st.session_state.monitoring_started_at is not None
            else None
        ),
        "wall_time_finished": datetime.now().isoformat(timespec="seconds"),
        "elapsed_seconds": (
            float(dataframe["elapsed_seconds"].dropna().iloc[-1])
            if "elapsed_seconds" in dataframe and not dataframe["elapsed_seconds"].dropna().empty
            else None
        ),
        "sim_time_finished": (
            float(dataframe["sim_time"].dropna().iloc[-1])
            if "sim_time" in dataframe and not dataframe["sim_time"].dropna().empty
            else None
        ),
        "terminal_event": terminal_kind,
        "arrived": bool(arrived),
        "distance_travelled_m": distance_travelled_m,
        "distance_remaining_m": 0.0 if arrived else distance_remaining_m,
        "battery_initial_wh": battery_initial_wh,
        "battery_final_wh": battery_final_wh,
        "battery_consumed_wh": battery_consumed_wh,
        "average_consumption_wh_per_km": average_consumption_wh_per_km,
        "speed_min_kmh": speed_min_kmh,
        "speed_max_kmh": speed_max_kmh,
        "speed_avg_kmh": speed_avg_kmh,
    }


def persist_monitoring_session(reason=None):
    """Persist the current monitoring session to disk."""
    samples = st.session_state.monitoring_samples
    session_token = st.session_state.monitoring_session_token
    if not samples or session_token is None:
        return None
    if st.session_state.monitoring_saved_session_token == session_token:
        return st.session_state.monitoring_summary

    summary = build_monitoring_summary(reason=reason)
    if summary is None:
        return None

    dataframe = pd.DataFrame(samples)
    battery_initial_wh = summary.get("battery_initial_wh")
    if battery_initial_wh is not None and "battery_wh" in dataframe:
        dataframe["battery_consumed_wh"] = dataframe["battery_wh"].apply(
            lambda value: battery_initial_wh - value if pd.notna(value) else None
        )
    elif (
        "total_energy_consumed_wh" in dataframe
        and not dataframe["total_energy_consumed_wh"].dropna().empty
    ):
        energy_initial_wh = dataframe["total_energy_consumed_wh"].dropna().iloc[0]
        dataframe["battery_consumed_wh"] = dataframe["total_energy_consumed_wh"].apply(
            lambda value: value - energy_initial_wh if pd.notna(value) else None
        )
    if "speed_mps" in dataframe:
        dataframe["speed_kmh"] = dataframe["speed_mps"].apply(
            lambda value: value * 3.6 if pd.notna(value) else None
        )

    output_dir = monitoring_output_dir()
    vehicle_fragment = _safe_filename_fragment(summary.get("vehicle_id"))
    csv_path = output_dir / f"{session_token}_{vehicle_fragment}_monitoring.csv"
    json_path = output_dir / f"{session_token}_{vehicle_fragment}_summary.json"
    consumption_plot_path = output_dir / f"{session_token}_{vehicle_fragment}_consumption.png"

    saved_consumption_plot = None
    if "battery_consumed_wh" in dataframe:
        saved_consumption_plot = save_line_figure(
            dataframe,
            consumption_plot_path,
            "battery_consumed_wh",
            "Cumulative Energy Consumption",
            "Energy consumed [Wh]",
            color="#dc2626",
        )
        if saved_consumption_plot is not None:
            summary["consumption_plot"] = str(saved_consumption_plot)

    dataframe.to_csv(csv_path, index=False)
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    st.session_state.monitoring_summary = summary
    st.session_state.monitoring_export_paths = {
        "csv": str(csv_path),
        "json": str(json_path),
    }
    if saved_consumption_plot is not None:
        st.session_state.monitoring_export_paths["consumption_plot"] = str(saved_consumption_plot)
    st.session_state.monitoring_saved_session_token = session_token
    return summary


def render_saved_monitoring_summary():
    """Render the most recently saved monitoring summary in the UI."""
    summary = st.session_state.get("monitoring_summary")
    if not summary:
        return

    st.write("### Monitoring Summary")
    summary_cols = st.columns(4)
    summary_cols[0].metric(
        "Remaining distance",
        "0 m" if summary.get("arrived") else (
            "-"
            if summary.get("distance_remaining_m") is None
            else f"{summary['distance_remaining_m']:.1f} m"
        ),
    )
    summary_cols[1].metric(
        "Avg consumption",
        "-"
        if summary.get("average_consumption_wh_per_km") is None
        else f"{summary['average_consumption_wh_per_km']:.1f} Wh/km",
    )
    summary_cols[2].metric(
        "Battery used",
        "-"
        if summary.get("battery_consumed_wh") is None
        else f"{summary['battery_consumed_wh']:.1f} Wh",
    )
    summary_cols[3].metric(
        "Avg speed",
        "-"
        if summary.get("speed_avg_kmh") is None
        else f"{summary['speed_avg_kmh']:.1f} km/h",
    )

    speed_cols = st.columns(3)
    speed_cols[0].metric(
        "Min speed",
        "-"
        if summary.get("speed_min_kmh") is None
        else f"{summary['speed_min_kmh']:.1f} km/h",
    )
    speed_cols[1].metric(
        "Max speed",
        "-"
        if summary.get("speed_max_kmh") is None
        else f"{summary['speed_max_kmh']:.1f} km/h",
    )
    speed_cols[2].metric(
        "Distance travelled",
        "-"
        if summary.get("distance_travelled_m") is None
        else f"{summary['distance_travelled_m']:.1f} m",
    )

    export_paths = st.session_state.get("monitoring_export_paths") or {}
    if export_paths.get("csv"):
        st.caption(f"Saved CSV: `{export_paths['csv']}`")
    if export_paths.get("json"):
        st.caption(f"Saved JSON: `{export_paths['json']}`")
    if export_paths.get("consumption_plot"):
        st.caption(f"Saved consumption plot: `{export_paths['consumption_plot']}`")
        st.image(
            export_paths["consumption_plot"],
            caption="Cumulative energy consumption exported from monitoring samples.",
            use_container_width=True,
        )
    if export_paths.get("sumo_consumption_plot"):
        st.caption(f"Saved SUMO consumption plot: `{export_paths['sumo_consumption_plot']}`")
        st.image(
            export_paths["sumo_consumption_plot"],
            caption="SUMO cumulative energy consumption from battery.out.xml.",
            use_container_width=True,
        )


def render_sumo_completed_trip_dashboard(selected_vehicle_id=None):
    """Render SUMO output metrics and energy curve for a completed monitored trip."""
    summary = st.session_state.get("monitoring_summary") or {}
    vehicle_id = selected_vehicle_id or summary.get("vehicle_id")
    trip_output = completed_trip_output(vehicle_id)
    tripinfo = trip_output.get("tripinfo")

    if not tripinfo:
        st.info("SUMO trip output will be available after the monitored vehicle completes its route.")
        st.caption(f"Tripinfo source: `{trip_output['path']}`")
        return

    battery_output = read_sumo_battery_output(
        vehicle_id,
        fallback_vehicle_id=tripinfo.get("id"),
    )
    distance_m = _coerce_float(tripinfo.get("routeLength"))
    duration_s = _coerce_float(tripinfo.get("duration"))
    avg_speed_kmh = (
        (distance_m / duration_s) * 3.6
        if distance_m is not None and duration_s is not None and duration_s > 0
        else summary.get("speed_avg_kmh")
    )
    electricity_abs_wh = _coerce_float(tripinfo.get("emissions_electricity_abs"))
    total_energy_wh = (
        battery_output.get("total_energy_wh")
        if battery_output.get("total_energy_wh") is not None
        else summary.get("battery_consumed_wh")
    )
    if total_energy_wh is None:
        total_energy_wh = electricity_abs_wh

    average_consumption_wh_per_km = (
        total_energy_wh / (distance_m / 1000.0)
        if total_energy_wh is not None and distance_m is not None and distance_m > 0.0
        else summary.get("average_consumption_wh_per_km")
    )

    st.write("### SUMO Output Dashboard")
    metric_cols = st.columns(4)
    metric_cols[0].metric("Total energy", _format_number(total_energy_wh, "Wh"))
    metric_cols[1].metric("Distance", _format_number(distance_m, "m"))
    metric_cols[2].metric("Average speed", _format_number(avg_speed_kmh, "km/h"))
    metric_cols[3].metric(
        "Avg consumption",
        _format_number(average_consumption_wh_per_km, "Wh/km"),
    )

    speed_cols = st.columns(4)
    speed_cols[0].metric(
        "Min speed",
        _format_number(summary.get("speed_min_kmh"), "km/h"),
    )
    speed_cols[1].metric(
        "Max speed",
        _format_number(summary.get("speed_max_kmh"), "km/h"),
    )
    speed_cols[2].metric(
        "Arrival speed",
        _format_number(
            (_coerce_float(tripinfo.get("arrivalSpeed")) or 0.0) * 3.6
            if _coerce_float(tripinfo.get("arrivalSpeed")) is not None
            else None,
            "km/h",
        ),
    )
    speed_cols[3].metric("Duration", format_elapsed(duration_s))

    battery_records = battery_output.get("records") or []
    if battery_records:
        curve_df = pd.DataFrame(battery_records)
        curve_df = curve_df.dropna(subset=["time_s", "cumulative_consumption_wh"])
        if not curve_df.empty:
            plot_path = None
            session_token = st.session_state.get("monitoring_session_token") or "latest"
            plot_vehicle_id = tripinfo.get("id") or vehicle_id or "vehicle"
            plot_path = (
                monitoring_output_dir()
                / f"{_safe_filename_fragment(session_token)}_"
                f"{_safe_filename_fragment(plot_vehicle_id)}_sumo_consumption.png"
            )
            saved_plot = save_line_figure(
                curve_df,
                plot_path,
                "cumulative_consumption_wh",
                "SUMO Cumulative Energy Consumption",
                "Energy consumed [Wh]",
                color="#b91c1c",
                preferred_x_columns=("time_s",),
            )
            render_line_figure(
                st,
                curve_df,
                "cumulative_consumption_wh",
                "SUMO Cumulative Energy Consumption",
                "Energy consumed [Wh]",
                color="#b91c1c",
                preferred_x_columns=("time_s",),
            )
            if saved_plot is not None:
                export_paths = st.session_state.get("monitoring_export_paths") or {}
                export_paths["sumo_consumption_plot"] = str(saved_plot)
                st.session_state.monitoring_export_paths = export_paths
                st.caption(f"Saved SUMO consumption plot: `{saved_plot}`")
    else:
        available_ids = battery_output.get("vehicle_ids") or []
        if available_ids:
            st.warning(
                "No battery samples matched the completed vehicle. "
                f"Vehicles in battery output: `{', '.join(available_ids[:8])}`."
            )
        else:
            st.warning("No battery samples are available in `battery.out.xml` for this run.")

    source_cols = st.columns(2)
    source_cols[0].caption(f"Battery source: `{battery_output['path']}`")
    source_cols[1].caption(f"Tripinfo source: `{trip_output['path']}`")


def render_monitoring_result_tabs(selected_vehicle_id=None):
    """Render monitoring outputs, adding SUMO results only for completed trips."""
    summary = st.session_state.get("monitoring_summary")
    trip_output = completed_trip_output(selected_vehicle_id)
    tripinfo = trip_output.get("tripinfo")

    if tripinfo:
        mark_summary_arrived_from_tripinfo(tripinfo)
        summary = st.session_state.get("monitoring_summary")

    if summary and tripinfo:
        summary_tab, sumo_tab = st.tabs(["Monitoring Summary", "SUMO Output Dashboard"])
        with summary_tab:
            render_saved_monitoring_summary()
        with sumo_tab:
            render_sumo_completed_trip_dashboard(selected_vehicle_id)
    elif tripinfo:
        render_sumo_completed_trip_dashboard(selected_vehicle_id)
    elif summary:
        render_saved_monitoring_summary()
    else:
        return


def vehicle_setup_step_label():
    """Return the label for the vehicle-setup step based on the active workflow."""
    if active_carla_version() == "0.9.13":
        return "3. Launch Autoware AV"
    return "3. Spawn SUMO Ego Vehicle"


def simulation_step_label():
    """Return the label for the simulation step based on the active workflow."""
    if active_carla_version() == "0.9.13":
        return "3. Configure Simulation"
    return "4. Run Simulation"


def runtime_map_options():
    """Return the runtime map options available to the current session."""
    allowed_maps = [map_name for map_name in available_maps() if map_name in AUTOWARE_ALLOWED_MAPS]
    return allowed_maps or available_maps()


def apply_selected_runtime_map():
    """Apply the runtime map selected in the dashboard state."""
    maps = runtime_map_options()
    if not maps:
        st.error("No SUMO/CARLA Town is available in the selected installation.")
        st.stop()

    if st.session_state.traffic_map_name not in maps:
        st.session_state.traffic_map_name = DEFAULT_MAP if DEFAULT_MAP in maps else maps[0]

    selected_map = st.session_state.traffic_map_name
    previous_map = st.session_state.applied_runtime_map
    if previous_map not in (None, selected_map):
        st.session_state.traffic_generation_result = None
        st.session_state.traffic_target_edge = ""
        st.session_state.traffic_destination_edge = ""
        st.session_state.traffic_source_edge = ""
        st.session_state.traffic_last_click = None
        st.session_state.traffic_clicked_direction = ""
        st.session_state.start_edge = None
        st.session_state.end_edge = None
        st.session_state.last_click = None
        st.session_state.ego_clicked_direction = ""
        st.session_state.autoware_start_edge = None
        st.session_state.autoware_goal_edge = None
        st.session_state.autoware_start_edge_selection = ""
        st.session_state.autoware_goal_edge_selection = ""
        st.session_state.autoware_pending_start_edge_selection = None
        st.session_state.autoware_pending_goal_edge_selection = None
        st.session_state.autoware_last_click = None
        st.session_state.autoware_clicked_direction = ""
        st.session_state.autoware_last_launch = None
        st.session_state.autoware_active_battery_failure_threshold = 0.0
        clear_waiting_synchronization_state(remove_gate_file=True)

    st.session_state.applied_runtime_map = selected_map
    return selected_map, maps


def ensure_simulation_launch_defaults():
    """Populate default launch settings for the co-simulation controls."""
    carla_server_ready = is_carla_server_ready()
    if st.session_state.traffic_carla_mode is None:
        st.session_state.traffic_carla_mode = "reuse" if carla_server_ready else "prepare"
    return carla_server_ready


def sync_launch_command_for_ui(sumocfg_file):
    """Format the synchronization launch command for display in the UI."""
    return build_run_command(
        sumocfg_file,
        sumo_gui=bool(st.session_state.traffic_sumo_gui),
    )


def sync_launch_widget_defaults():
    """Initialize default widget values for the synchronization launcher."""
    if st.session_state.traffic_carla_mode_selection not in {"reuse", "prepare"}:
        st.session_state.traffic_carla_mode_selection = st.session_state.traffic_carla_mode
    if not SHOW_SUMO_GUI_INPUT:
        st.session_state.traffic_sumo_gui = True
        st.session_state.traffic_sumo_gui_selection = True
    if not isinstance(st.session_state.traffic_sumo_gui_selection, bool):
        st.session_state.traffic_sumo_gui_selection = bool(st.session_state.traffic_sumo_gui)
    if not isinstance(st.session_state.traffic_carla_timeout_selection, int):
        st.session_state.traffic_carla_timeout_selection = int(
            st.session_state.traffic_carla_timeout_seconds
        )
    if not isinstance(st.session_state.autoware_sync_delay_selection, int):
        st.session_state.autoware_sync_delay_selection = int(
            st.session_state.autoware_sync_delay_seconds
        )


def effective_autoware_startup_wait_seconds(sumo_gui=None):
    """Return the Autoware warm-up delay currently in effect."""
    configured_wait = int(st.session_state.autoware_sync_delay_seconds)
    if sumo_gui is None:
        sumo_gui = bool(st.session_state.traffic_sumo_gui)
    return 0 if sumo_gui else configured_wait


def autoware_launch_sync_state():
    """Return the UI state associated with Autoware launch synchronization."""
    launch = st.session_state.autoware_last_launch
    if not launch:
        return None

    started_at = launch.get("started_at")
    if started_at is None:
        return None

    wait_seconds = int(launch.get("startup_wait_seconds", 0))
    ready_at = float(started_at) + max(wait_seconds, 0)
    remaining_seconds = max(0.0, ready_at - time.time())
    return {
        "map_name": launch.get("map_name"),
        "container_name": launch.get("container_name"),
        "started_at": float(started_at),
        "startup_wait_seconds": wait_seconds,
        "ready_at": ready_at,
        "remaining_seconds": remaining_seconds,
        "is_ready": remaining_seconds <= 1e-6,
    }


def waiting_for_autoware_sync():
    """Report whether the co-simulation is waiting for Autoware to start."""
    sync_process = st.session_state.traffic_process
    sync_running = sync_process is not None and sync_process.poll() is None
    if st.session_state.traffic_waiting_for_autoware and not sync_running:
        clear_waiting_synchronization_state(remove_gate_file=True)
        return False
    return bool(st.session_state.traffic_waiting_for_autoware and sync_running)


def clear_waiting_synchronization_state(remove_gate_file=False):
    """Clear UI state related to a waiting synchronization launch."""
    gate_file = st.session_state.traffic_start_gate_file
    if remove_gate_file and gate_file:
        try:
            Path(gate_file).unlink(missing_ok=True)
        except OSError:
            pass
    st.session_state.traffic_waiting_for_autoware = False
    st.session_state.traffic_start_gate_file = None


def release_waiting_synchronization_gate():
    """Release the file gate that lets a waiting synchronization continue."""
    gate_file = st.session_state.traffic_start_gate_file
    if not gate_file:
        raise RuntimeError("No SUMO start gate is armed for the current session.")

    gate_path = Path(gate_file)
    gate_path.parent.mkdir(parents=True, exist_ok=True)
    gate_path.write_text("start\n", encoding="utf-8")
    clear_waiting_synchronization_state(remove_gate_file=False)


def start_dashboard_synchronization_launch(
    result,
    carla_mode,
    carla_timeout,
    sumo_gui,
    wait_for_autoware=False,
):
    """Start the dashboard synchronization process from the UI workflow."""
    carla_server_ready = is_carla_server_ready()
    if carla_mode == "reuse" and not carla_server_ready:
        raise RuntimeError(
            f"CARLA is not reachable on {DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}. "
            "Start CARLA first or switch to 'Start CARLA and load the Town'."
        )

    last_autoware_launch = st.session_state.get("autoware_last_launch") or {}
    autoware_already_launched = (
        active_carla_version() == "0.9.13"
        and last_autoware_launch.get("map_name") == result.map_name
    )
    launch_state = autoware_launch_sync_state() if autoware_already_launched else None
    effective_wait_for_autoware = bool(wait_for_autoware)
    if effective_wait_for_autoware and launch_state is not None:
        remaining_seconds = max(0.0, float(launch_state.get("remaining_seconds") or 0.0))
        if remaining_seconds > 0:
            time.sleep(remaining_seconds)
        effective_wait_for_autoware = False

    launch = start_synchronization(
        result.sumocfg_file,
        map_name=result.map_name,
        ensure_carla=(carla_mode == "prepare"),
        carla_process=st.session_state.carla_process,
        carla_timeout=int(carla_timeout),
        sumo_gui=bool(sumo_gui),
        wait_for_start=effective_wait_for_autoware,
        load_map=not autoware_already_launched,
    )

    st.session_state.traffic_process = launch.sync_process
    st.session_state.traffic_process_log = launch.sync_log_file
    st.session_state.carla_process = launch.carla_process
    st.session_state.carla_process_log = launch.carla_log_file
    st.session_state.traffic_waiting_for_autoware = launch.start_gate_file is not None
    st.session_state.traffic_start_gate_file = (
        str(launch.start_gate_file) if launch.start_gate_file is not None else None
    )
    return launch


def synchronization_success_message(launch, result, carla_mode, sumo_gui):
    """Build the success message shown after a synchronization launch."""
    if launch.start_gate_file is not None:
        message = (
            f"SUMO/CARLA bridge started, PID {launch.sync_process.pid}, "
            "and is waiting for Autoware before simulation time advances."
        )
    else:
        message = f"Co-simulation started, PID {launch.sync_process.pid}."
    if carla_mode == "reuse":
        message += (
            f" Using CARLA already running on "
            f"{DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}."
        )
    if launch.map_loaded:
        message += f" Town {result.map_name} loaded."
    if carla_mode != "reuse" and launch.carla_started:
        message += " CARLA started."
    message += " SUMO GUI enabled." if sumo_gui else " SUMO running headless."
    return message


def render_carla_step():
    """Render the dashboard step used to start CARLA and choose the map."""
    selected_version = active_carla_version()
    status = carla_server_status(selected_version)
    sync_process = st.session_state.traffic_process
    sync_running = sync_process is not None and sync_process.poll() is None
    selected_map, maps = apply_selected_runtime_map()
    if st.session_state.runtime_map_selection not in maps:
        st.session_state.runtime_map_selection = selected_map

    st.subheader("🚘 Start CARLA")
    st.caption(
        "Select the Town here. All following steps use this Town and no longer expose a separate map selector."
    )

    town_cols = st.columns([2, 3])
    with town_cols[0]:
        selected_map_choice = st.selectbox(
            "Selected Town",
            options=maps,
            index=edge_select_index(maps, st.session_state.runtime_map_selection),
            key="runtime_map_selection",
            disabled=sync_running,
        )
    with town_cols[1]:
        if sync_running:
            st.caption(
                "The Town selector is locked while the co-simulation is running."
            )
        else:
            st.caption(
                "This Town will be used by SUMO route generation, Autoware launch, and the co-simulation startup."
            )

    if selected_map_choice != st.session_state.traffic_map_name:
        st.session_state.traffic_map_name = selected_map_choice
    selected_map, _ = apply_selected_runtime_map()

    if status["running"]:
        detected_version = status.get("detected_version")
        if detected_version:
            st.success(
                f"CARLA server is active on `{DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}` "
                f"for version `{detected_version}`."
            )
        else:
            st.success(
                f"CARLA server is active on `{DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}`."
            )
    else:
        st.warning(
            f"CARLA is not running on `{DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}`."
        )

    if status["running"] and status.get("detected_version") not in (None, selected_version):
        st.warning(
            f"The selected version is `{selected_version}`, but the running CARLA server "
            f"was detected as `{status['detected_version']}`."
        )

    info_cols = st.columns(3)
    info_cols[0].metric("Selected CARLA", selected_version)
    info_cols[1].metric("Selected Town", selected_map)
    info_cols[2].metric("Status", "Running" if status["running"] else "Stopped")

    if status["running"]:
        load_cols = st.columns([1, 3])
        with load_cols[0]:
            if st.button("Load selected Town", use_container_width=True):
                try:
                    with st.spinner(f"Loading `{selected_map}` in CARLA..."):
                        load_carla_map(selected_map)
                    st.success(f"Town `{selected_map}` loaded in CARLA.")
                except Exception as exc:
                    st.error(f"Town load failed: {exc}")
        with load_cols[1]:
            st.caption(
                "Use this if CARLA is already running and you want to force the selected Town before the next steps."
            )

    if st.session_state.carla_process_log:
        st.caption(f"CARLA Log: {st.session_state.carla_process_log}")


def reset_monitoring_trip_state():
    """Reset the dashboard state used to track a monitored trip."""
    st.session_state.battery_data = []
    st.session_state.monitoring_samples = []
    st.session_state.monitoring_summary = None
    st.session_state.monitoring_export_paths = None
    st.session_state.monitoring_session_token = None
    st.session_state.monitoring_saved_session_token = None
    st.session_state.dashboard_events = []
    st.session_state.vehicle_terminal_event = None
    st.session_state.latest_monitoring_state = None
    st.session_state.last_vehicle_state = None
    st.session_state.monitoring_events = []
    st.session_state.last_monitoring_poll = 0.0
    st.session_state.monitoring_started_at = None


def record_dashboard_event(kind, state):
    """Append a normalized event to the monitoring event log."""
    if st.session_state.vehicle_terminal_event is not None:
        return

    vehicle_id = state.get("vehicle_id", "-") if state else "-"
    edge = state.get("edge", "-") if state else "-"
    battery = state.get("battery") if state else None
    wall_time = datetime.now().strftime("%H:%M:%S")
    elapsed = None

    if st.session_state.monitoring_started_at is not None:
        elapsed = time.time() - st.session_state.monitoring_started_at

    event = {
        "kind": kind,
        "vehicle_id": vehicle_id,
        "edge": edge,
        "battery": battery,
        "wall_time": wall_time,
        "elapsed": format_elapsed(elapsed),
        "sim_time": extract_state_time(state),
    }

    if kind == "battery_depleted":
        default_threshold = (
            st.session_state.ego_active_battery_failure_threshold
            if vehicle_id == "ego_vehicle"
            else st.session_state.autoware_active_battery_failure_threshold
        )
        threshold = state.get(
            "battery_failure_threshold",
            default_threshold,
        ) if state else default_threshold
        event["message"] = (
            f"[{vehicle_id}] Battery below critical threshold "
            f"({threshold} Wh): failure at {wall_time}, "
            f"t={event['elapsed']}, edge={edge}"
        )
    elif kind == "destination_reached":
        event["message"] = (
            f"[{vehicle_id}] Destination reached "
            f"at {wall_time}, t={event['elapsed']}, edge={edge}"
        )
    else:
        event["message"] = f"[{vehicle_id}] Vehicle event at {wall_time}, edge={edge}"

    if event["sim_time"] is not None:
        event["message"] += f", t_sim={event['sim_time']}"

    st.session_state.vehicle_terminal_event = event
    st.session_state.dashboard_events.append(event)


def update_vehicle_events(state):
    """Detect and record relevant monitoring events for the current vehicle state."""
    if not state:
        return

    st.session_state.last_vehicle_state = state

    vehicle_id = state.get("vehicle_id") or st.session_state.monitor_vehicle_selected_id
    edge = state.get("edge")
    raw_battery = state.get("battery")
    battery = _coerce_float(raw_battery)

    default_threshold = (
        st.session_state.ego_active_battery_failure_threshold
        if vehicle_id == "ego_vehicle"
        else st.session_state.autoware_active_battery_failure_threshold
    )
    failure_threshold = float(
        state.get(
            "battery_failure_threshold",
            default_threshold,
        )
        or 0
    )
    if battery is not None and battery <= failure_threshold:
        record_dashboard_event("battery_depleted", state)
        st.session_state.monitoring = False
        return

    remaining_distance_m = _coerce_float(state.get("distance_remaining_m"))
    if remaining_distance_m is not None and remaining_distance_m <= 1.0:
        record_dashboard_event("destination_reached", state)
        st.session_state.monitoring = False
        return

    destination_edge = (
        st.session_state.autoware_goal_edge
        if st.session_state.autoware_last_launch and st.session_state.autoware_goal_edge
        else (st.session_state.ego_active_end_edge or st.session_state.end_edge)
    )
    if destination_edge and edge == destination_edge:
        record_dashboard_event("destination_reached", state)
        st.session_state.monitoring = False


def render_event(event):
    """Render a single monitoring event in the dashboard."""
    if isinstance(event, dict):
        kind = event.get("kind")
        message = event.get("message", str(event))

        if kind == "battery_depleted":
            st.error(message)
        elif kind == "destination_reached":
            st.success(message)
        else:
            st.write(message)
    else:
        st.write(event)


def poll_monitoring_snapshot(selected_vehicle_id):
    """Fetch one monitoring sample on explicit user request."""
    if not selected_vehicle_id:
        return False

    try:
        response = requests.get(
            f"{API_URL}/state",
            params={"veh_id": selected_vehicle_id},
            timeout=1,
        )
        res = response.json() if response.status_code == 200 else None
    except Exception:
        res = None

    if res:
        st.session_state.latest_monitoring_state = res
        raw_battery = res.get("battery")
        battery_val = _coerce_float(raw_battery)

        if battery_val is not None:
            st.session_state.battery_data.append(battery_val)
        append_monitoring_sample(res)
        update_vehicle_events(res)
    elif infer_arrival_from_sumo_tripinfo(selected_vehicle_id):
        st.session_state.monitoring = False
        persist_monitoring_session("destination_reached")
    elif st.session_state.monitoring:
        st.session_state.latest_monitoring_state = None

    try:
        events_response = requests.get(f"{API_URL}/events", timeout=1)
        if events_response.status_code == 200:
            st.session_state.monitoring_events = events_response.json()
    except Exception:
        pass

    st.session_state.last_monitoring_poll = time.monotonic()
    return bool(res)


def render_monitoring():
    """Render the monitoring step and its live metrics."""
    st.subheader("📊 Monitoring")

    if not backend_alive:
        st.warning("Start the co-simulation from the simulation step before monitoring.")
        render_monitoring_result_tabs(st.session_state.monitor_vehicle_selected_id)
        return

    try:
        live_vehicles = get_live_sumo_vehicles()
    except Exception as exc:
        st.error(f"Could not load vehicles for monitoring: {exc}")
        render_monitoring_result_tabs(st.session_state.monitor_vehicle_selected_id)
        return

    monitoring_candidates = [
        vehicle
        for vehicle in live_vehicles
        if vehicle.get("id") == "ego_vehicle" or is_carla_spawned_vehicle(vehicle)
    ]
    if not monitoring_candidates:
        check_clicked = st.button("Check SUMO output", use_container_width=True)
        if st.session_state.monitoring or check_clicked:
            reason = "vehicle_unavailable"
            if infer_arrival_from_sumo_tripinfo(st.session_state.monitor_vehicle_selected_id):
                reason = "destination_reached"
            st.session_state.monitoring = False
            if st.session_state.monitoring_session_token:
                persist_monitoring_session(reason)
        st.info("No ego or CARLA-spawned SUMO vehicle is currently active.")
        render_monitoring_result_tabs(st.session_state.monitor_vehicle_selected_id)
        return

    candidate_ids = [vehicle["id"] for vehicle in monitoring_candidates]
    candidate_labels = {
        vehicle["id"]: vehicle_display_label(vehicle)
        for vehicle in monitoring_candidates
    }
    preferred_vehicle_id = preferred_monitoring_vehicle_id(monitoring_candidates)
    current_vehicle_id = st.session_state.monitor_vehicle_selected_id

    if current_vehicle_id not in candidate_ids:
        if (
            current_vehicle_id
            and st.session_state.monitoring
            and preferred_vehicle_id
            and is_autoware_monitoring_vehicle_id(current_vehicle_id)
            and is_autoware_monitoring_vehicle_id(preferred_vehicle_id)
        ):
            current_vehicle_id = preferred_vehicle_id
        elif current_vehicle_id and st.session_state.monitoring:
            reason = "vehicle_unavailable"
            if infer_arrival_from_sumo_tripinfo(current_vehicle_id):
                reason = "destination_reached"
            st.session_state.monitoring = False
            persist_monitoring_session(reason)
            st.info("The monitored vehicle is no longer active in SUMO.")
            render_monitoring_result_tabs(current_vehicle_id)
            return

        if current_vehicle_id and st.session_state.monitoring_summary:
            if st.button("Check SUMO output", use_container_width=True):
                infer_arrival_from_sumo_tripinfo(current_vehicle_id)
            st.info("The monitored vehicle is no longer active in SUMO.")
            render_monitoring_result_tabs(current_vehicle_id)
            return

        current_vehicle_id = preferred_vehicle_id
    elif (
        current_vehicle_id == "ego_vehicle"
        and preferred_vehicle_id
        and preferred_vehicle_id != current_vehicle_id
        and not st.session_state.monitoring
        and not st.session_state.monitoring_summary
    ):
        current_vehicle_id = preferred_vehicle_id

    st.session_state.monitor_vehicle_selected_id = current_vehicle_id
    selected_vehicle_id = current_vehicle_id
    st.caption(f"Monitoring target: {candidate_labels[selected_vehicle_id]}")

    if st.session_state.monitor_vehicle_loaded_for != selected_vehicle_id:
        previous_vehicle_id = st.session_state.monitor_vehicle_loaded_for
        keep_autoware_session = (
            st.session_state.monitoring
            and is_autoware_monitoring_vehicle_id(previous_vehicle_id)
            and is_autoware_monitoring_vehicle_id(selected_vehicle_id)
        )
        if not keep_autoware_session:
            persist_monitoring_session("target_changed")
            reset_monitoring_trip_state()
        st.session_state.monitor_vehicle_loaded_for = selected_vehicle_id

    control_cols = st.columns(3)

    with control_cols[0]:
        if st.button("Start Monitoring", disabled=st.session_state.monitoring):
            reset_monitoring_trip_state()
            st.session_state.monitoring = True
            st.session_state.monitoring_session_token = datetime.now().strftime("%Y%m%d_%H%M%S")
            st.session_state.last_monitoring_poll = 0.0
            st.session_state.monitoring_started_at = time.time()
            poll_monitoring_snapshot(selected_vehicle_id)

    with control_cols[1]:
        update_clicked = st.button(
            "Update Data & Charts",
            disabled=not st.session_state.monitoring and not st.session_state.monitoring_summary,
            use_container_width=True,
        )
        if update_clicked:
            if st.session_state.monitoring:
                poll_monitoring_snapshot(selected_vehicle_id)
            else:
                infer_arrival_from_sumo_tripinfo(selected_vehicle_id)

    with control_cols[2]:
        if st.button("Stop Monitoring", disabled=not st.session_state.monitoring):
            st.session_state.monitoring = False
            persist_monitoring_session("manual_stop")

    chart_placeholder = st.empty()
    consumption_chart_placeholder = st.empty()
    metrics_placeholder = st.empty()
    battery_event_placeholder = st.empty()
    event_placeholder = st.empty()

    if st.session_state.monitoring_session_token and not st.session_state.monitoring:
        terminal_event = st.session_state.vehicle_terminal_event or {}
        persist_monitoring_session(terminal_event.get("kind") or "stopped")

    if st.session_state.monitoring_samples:
        monitoring_df = pd.DataFrame(st.session_state.monitoring_samples)
        if "battery_wh" in monitoring_df and not monitoring_df["battery_wh"].dropna().empty:
            render_line_figure(
                chart_placeholder,
                monitoring_df,
                "battery_wh",
                "Battery Level During Trip",
                "Battery charge [Wh]",
                color="#2563eb",
            )
            battery_initial = monitoring_df["battery_wh"].dropna().iloc[0]
            monitoring_df["battery_consumed_wh"] = monitoring_df["battery_wh"].apply(
                lambda value: battery_initial - value if pd.notna(value) else None
            )
            render_line_figure(
                consumption_chart_placeholder,
                monitoring_df,
                "battery_consumed_wh",
                "Cumulative Energy Consumption",
                "Energy consumed [Wh]",
                color="#dc2626",
            )
        elif (
            "total_energy_consumed_wh" in monitoring_df
            and not monitoring_df["total_energy_consumed_wh"].dropna().empty
        ):
            energy_initial = monitoring_df["total_energy_consumed_wh"].dropna().iloc[0]
            monitoring_df["battery_consumed_wh"] = monitoring_df["total_energy_consumed_wh"].apply(
                lambda value: value - energy_initial if pd.notna(value) else None
            )
            render_line_figure(
                consumption_chart_placeholder,
                monitoring_df,
                "battery_consumed_wh",
                "Cumulative Energy Consumption",
                "Energy consumed [Wh]",
                color="#dc2626",
            )

    res = st.session_state.latest_monitoring_state
    if res:
        raw_battery = res.get("battery")
        battery_val = _coerce_float(raw_battery)
        speed = _coerce_float(res.get("speed"))
        remaining_distance_m = _coerce_float(res.get("distance_remaining_m"))
        edge = res.get("edge", "-")
        vehicle_id = res.get("vehicle_id", selected_vehicle_id)

        with metrics_placeholder.container():
            c1, c2, c3, c4, c5 = st.columns(5)
            c1.metric("🚘 Vehicle", vehicle_id)
            c2.metric("🔋 Battery", "-" if battery_val is None else battery_val)
            c3.metric(
                "🚗 Speed",
                "-" if speed is None else f"{speed * 3.6:.1f} km/h",
            )
            c4.metric("🛣️ Edge", edge)
            c5.metric(
                "🎯 Remaining",
                "-"
                if remaining_distance_m is None
                else f"{remaining_distance_m:.1f} m",
            )

        if battery_val is not None and battery_val <= 0:
            battery_event_placeholder.error("⚠️ Battery depleted!")
    elif st.session_state.monitoring:
        metrics_placeholder.info("Waiting for vehicle state...")

    with event_placeholder.container():
        st.write("### 🚨 Events")
        events = st.session_state.dashboard_events + st.session_state.monitoring_events

        if events:
            for e in events[-8:]:
                render_event(e)
        else:
            st.write("No events yet.")

    render_monitoring_result_tabs(selected_vehicle_id)


def get_network():
    """Load the active road network for map interactions."""
    if "network" not in st.session_state:
        res = requests.get(f"{API_URL}/network", timeout=2).json()
        st.session_state.network = res["edges"]

    return st.session_state.network


def to_map_coords(x, y):
    """Convert SUMO coordinates to the map coordinates used by the UI."""
    return [y, x]


def _edge_length(edge):
    """Return a positive length estimate for an edge."""
    try:
        length = float(edge.length)
    except (TypeError, ValueError):
        length = 0.0

    if length > 0:
        return length

    total = 0.0
    for start, end in zip(edge.shape, edge.shape[1:]):
        total += ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
    return total


def _farthest_geometric_edge_pair(edges):
    """Return the directed edge pair with the largest straight-line span."""
    usable_edges = [edge for edge in edges if len(edge.shape) >= 2]
    best_start = None
    best_end = None
    best_distance_sq = -1.0

    for start_edge in usable_edges:
        start_x, start_y = start_edge.shape[0]
        for end_edge in usable_edges:
            if end_edge.edge_id == start_edge.edge_id:
                continue

            end_x, end_y = end_edge.shape[-1]
            distance_sq = (start_x - end_x) ** 2 + (start_y - end_y) ** 2
            if distance_sq > best_distance_sq:
                best_start = start_edge
                best_end = end_edge
                best_distance_sq = distance_sq

    if best_start is None or best_end is None:
        return None, None, None

    return best_start.edge_id, best_end.edge_id, best_distance_sq ** 0.5


def farthest_directed_edge_pair(edges):
    """Return the reachable directed edge pair with the largest estimated route length."""
    usable_edges = [
        edge
        for edge in edges
        if len(edge.shape) >= 2 and edge.from_node and edge.to_node
    ]
    if len(usable_edges) < 2:
        return _farthest_geometric_edge_pair(edges)

    adjacency = {}
    for edge in usable_edges:
        adjacency.setdefault(edge.from_node, []).append((edge.to_node, _edge_length(edge)))

    distance_cache = {}

    def shortest_distances(source_node):
        """Run Dijkstra from a node over the directed edge graph."""
        if source_node in distance_cache:
            return distance_cache[source_node]

        distances = {source_node: 0.0}
        heap = [(0.0, source_node)]
        while heap:
            current_distance, node = heapq.heappop(heap)
            if current_distance > distances.get(node, float("inf")):
                continue

            for next_node, edge_length in adjacency.get(node, []):
                candidate_distance = current_distance + edge_length
                if candidate_distance < distances.get(next_node, float("inf")):
                    distances[next_node] = candidate_distance
                    heapq.heappush(heap, (candidate_distance, next_node))

        distance_cache[source_node] = distances
        return distances

    best_start = None
    best_end = None
    best_route_length = -1.0
    edge_lengths = {edge.edge_id: _edge_length(edge) for edge in usable_edges}

    for start_edge in usable_edges:
        distances = shortest_distances(start_edge.to_node)
        start_length = edge_lengths[start_edge.edge_id]

        for end_edge in usable_edges:
            if end_edge.edge_id == start_edge.edge_id:
                continue

            connector_length = distances.get(end_edge.from_node)
            if connector_length is None:
                continue

            route_length = start_length + connector_length + edge_lengths[end_edge.edge_id]
            if route_length > best_route_length:
                best_start = start_edge
                best_end = end_edge
                best_route_length = route_length

    if best_start is None or best_end is None:
        return _farthest_geometric_edge_pair(edges)

    return best_start.edge_id, best_end.edge_id, best_route_length


@st.cache_data(show_spinner=False)
def get_offline_edges(map_name):
    """Load cached edge data for the selected map."""
    return read_sumo_edges(map_name)


@st.cache_data(show_spinner=False)
def get_sumo_vtypes():
    """Return the available SUMO vehicle types for the active setup."""
    return available_vehicle_types()


def edge_select_index(options, current_value):
    """Return the select-box index matching the current edge selection."""
    try:
        return options.index(current_value)
    except ValueError:
        return 0


def pick_auto_edge(edges, excluded=None, preferred=()):
    """Pick a fallback edge from a list using exclusion and preference hints."""
    excluded = {edge_id for edge_id in (excluded or []) if edge_id}
    edge_by_id = {edge.edge_id: edge for edge in edges}

    for edge_id in preferred:
        if edge_id and edge_id in edge_by_id and edge_id not in excluded:
            return edge_id

    candidates = [edge for edge in edges if edge.edge_id not in excluded]
    if not candidates:
        return None

    return max(candidates, key=lambda edge: (edge.length, edge.lane_count, edge.edge_id)).edge_id


def generated_route_hints(route_file, target_edge):
    """Summarize the key properties of a generated route file."""
    if not route_file or not target_edge:
        return None, None

    try:
        root = ET.parse(route_file).getroot()
    except (ET.ParseError, OSError):
        return None, None

    fallback = (None, None)

    for route in root.findall(".//route"):
        route_edges = (route.get("edges") or "").split()
        if target_edge not in route_edges:
            continue

        candidate = (route_edges[0], route_edges[-1])
        if fallback == (None, None):
            fallback = candidate

        target_index = route_edges.index(target_edge)
        if 0 < target_index < len(route_edges) - 1:
            return candidate

    return fallback


def apply_vtype_config_to_state(prefix, config):
    """Copy a vehicle-type configuration into Streamlit session state."""
    attribute_defaults, parameter_defaults = ego_model_defaults(config["emission_model"])
    merged_attributes = dict(attribute_defaults)
    merged_attributes.update(config.get("attributes") or {})
    merged_parameters = dict(parameter_defaults)
    merged_parameters.update(config.get("parameters") or {})

    st.session_state[f"{prefix}_sumo_vtype"] = config.get("sumo_vtype", "")
    st.session_state[f"{prefix}_carla_blueprint"] = config["carla_blueprint"]
    st.session_state[f"{prefix}_emission_model"] = config["emission_model"]
    st.session_state[f"{prefix}_last_emission_model"] = config["emission_model"]
    st.session_state[f"{prefix}_max_battery_capacity"] = config["battery_capacity"]
    if "battery_charge_level" in config:
        st.session_state[f"{prefix}_initial_battery"] = config["battery_charge_level"]
    default_has_battery = prefix in {"ego", "autoware_ego"}
    st.session_state[f"{prefix}_has_battery_device"] = bool(
        config.get("has_battery_device", default_has_battery)
    )

    for key, value in merged_attributes.items():
        st.session_state[f"{prefix}_attr_{key}"] = value
    for key, value in merged_parameters.items():
        st.session_state[f"{prefix}_param_{key}"] = value


def initialize_ego_config_state():
    """Initialize dashboard state for the SUMO ego vehicle configuration."""
    current_version = active_carla_version()
    if (
        st.session_state.ego_config_loaded
        and st.session_state.ego_config_version == current_version
    ):
        return

    config = read_ego_vtype_config()
    apply_vtype_config_to_state("ego", config)
    st.session_state.ego_initial_battery = min(
        st.session_state.ego_initial_battery,
        config["battery_capacity"],
    )
    st.session_state.ego_config_loaded = True
    st.session_state.ego_config_version = current_version


def initialize_autoware_ego_config_state():
    """Initialize dashboard state for the Autoware ego vehicle configuration."""
    current_version = active_carla_version()
    if (
        st.session_state.autoware_ego_config_loaded
        and st.session_state.autoware_ego_config_version == current_version
    ):
        return

    config = read_autoware_ego_vtype_config()
    apply_vtype_config_to_state("autoware_ego", config)
    st.session_state.autoware_battery_failure_threshold = float(
        config.get("battery_failure_threshold", 0.0) or 0.0
    )
    st.session_state.autoware_ego_config_loaded = True
    st.session_state.autoware_ego_config_version = current_version


def reset_vtype_model_state(prefix, emission_model):
    """Reset emission-model-specific vehicle-type values in session state."""
    attributes, parameters = ego_model_defaults(emission_model)

    for key, value in attributes.items():
        st.session_state[f"{prefix}_attr_{key}"] = value
    for key, value in parameters.items():
        st.session_state[f"{prefix}_param_{key}"] = value


def bounded_float(value, minimum, maximum):
    """Clamp a numeric value to the requested range."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = minimum

    return min(max(number, minimum), maximum)


def parameter_input_grid(values, key_prefix, text_area_keys=()):
    """Render editable vehicle parameter fields in a compact grid."""
    text_area_keys = set(text_area_keys)
    result = {}
    normal_items = [(key, value) for key, value in values.items() if key not in text_area_keys]
    cols = st.columns(3)

    for index, (key, value) in enumerate(normal_items):
        widget_key = f"{key_prefix}_{key}"

        with cols[index % len(cols)]:
            if key == "color":
                current_color = st.session_state.get(widget_key, value)
                if current_color not in BASE_EGO_COLORS:
                    current_color = "white"
                st.session_state[widget_key] = current_color
                result[key] = st.selectbox(
                    key,
                    options=BASE_EGO_COLORS,
                    index=edge_select_index(BASE_EGO_COLORS, current_color),
                    key=widget_key,
                )
            elif key in EGO_NUMERIC_LIMITS:
                minimum, maximum, step = EGO_NUMERIC_LIMITS[key]
                st.session_state[widget_key] = bounded_float(
                    st.session_state.get(widget_key, value),
                    minimum,
                    maximum,
                )
                result[key] = st.number_input(
                    key,
                    min_value=minimum,
                    max_value=maximum,
                    step=step,
                    key=widget_key,
                )
            else:
                if widget_key not in st.session_state:
                    st.session_state[widget_key] = value
                result[key] = st.text_input(key, key=widget_key)

    for key in text_area_keys:
        if key not in values:
            continue

        widget_key = f"{key_prefix}_{key}"
        if widget_key not in st.session_state:
            st.session_state[widget_key] = values[key]

        result[key] = st.text_area(
            key,
            key=widget_key,
            help="Leave empty to use the SUMO default.",
        )

    return result


def render_ego_vehicle_config():
    """Render the SUMO ego-vehicle configuration editor."""
    initialize_ego_config_state()

    carla_vtypes = available_carla_vehicle_types()
    if not carla_vtypes:
        st.error("No CARLA vehicle blueprint found in vtypes.json.")
        return None

    if st.session_state.ego_carla_blueprint not in carla_vtypes:
        st.session_state.ego_carla_blueprint = DEFAULT_EGO_BLUEPRINT

    st.write("##### Ego Vehicle Type")
    top_cols = st.columns([2, 1, 1])
    with top_cols[0]:
        carla_blueprint = st.selectbox(
            "CARLA vType / blueprint",
            options=carla_vtypes,
            index=edge_select_index(carla_vtypes, st.session_state.ego_carla_blueprint),
            key="ego_carla_blueprint",
        )
    with top_cols[1]:
        emission_model = st.selectbox(
            "Energy model",
            options=[ENERGY_EMISSION_CLASS, MMPEVEM_EMISSION_CLASS],
            key="ego_emission_model",
        )
    with top_cols[2]:
        battery_capacity = st.number_input(
            "Max battery capacity [Wh]",
            min_value=1.0,
            max_value=500000.0,
            value=float(st.session_state.ego_max_battery_capacity),
            step=100.0,
            key="ego_max_battery_capacity",
        )

    if st.session_state.ego_last_emission_model != emission_model:
        reset_vtype_model_state("ego", emission_model)
        st.session_state.ego_last_emission_model = emission_model

    attribute_defaults, parameter_defaults = ego_model_defaults(emission_model)

    with st.expander("Ego vType parameters", expanded=False):
        st.caption(
            "Empty parameters are omitted from the XML file, so SUMO uses its own defaults."
        )
        st.write("Attributes")
        attributes = parameter_input_grid(attribute_defaults, "ego_attr")
        st.write("Parameters")
        parameters = parameter_input_grid(
            parameter_defaults,
            "ego_param",
            text_area_keys=("powerLossMap",),
        )

    if st.button("Save ego vType XML"):
        write_ego_vtype_config(
            carla_blueprint,
            emission_model,
            battery_capacity,
            attributes=attributes,
            parameters=parameters,
        )
        st.success("egovtype.xml updated.")

    return {
        "sumo_vtype": EGO_SUMO_VTYPE,
        "carla_blueprint": carla_blueprint,
        "emission_model": emission_model,
        "battery_capacity": battery_capacity,
        "attributes": attributes,
        "parameters": parameters,
        "emission_class": ego_emission_class_value(emission_model),
    }


def render_autoware_ego_vtype_editor():
    """Render the Autoware ego-vehicle configuration editor."""
    initialize_autoware_ego_config_state()

    st.subheader("🔧 Ego vType")
    st.caption(
        f"The Autoware Docker launch currently expects the CARLA blueprint `{AUTOWARE_EGO_VTYPE}`."
    )
    if st.session_state.autoware_last_launch:
        last_launch = st.session_state.autoware_last_launch
        st.caption(
            f"Last Autoware launch: map `{last_launch.get('map_name', '-')}`, "
            f"container `{last_launch.get('container_name', '-')}`."
        )
    selected_map_name, _ = apply_selected_runtime_map()
    st.caption(
        f"Autoware launch map fixed to `{selected_map_name}` from step 1."
    )
    autoware_edges = get_offline_edges(selected_map_name)
    autoware_edge_ids = [edge.edge_id for edge in autoware_edges]
    autoware_labels = {
        edge.edge_id: edge_label(edge)
        for edge in autoware_edges
    }
    autoware_edge_options = [""] + autoware_edge_ids
    if st.session_state.autoware_start_edge not in autoware_edge_ids:
        st.session_state.autoware_start_edge = (
            st.session_state.start_edge
            if st.session_state.start_edge in autoware_edge_ids
            else None
        )
    if st.session_state.autoware_goal_edge not in autoware_edge_ids:
        st.session_state.autoware_goal_edge = (
            st.session_state.end_edge
            if st.session_state.end_edge in autoware_edge_ids
            else None
        )
    scenario_result = st.session_state.traffic_generation_result
    traffic_target_edge = (
        getattr(scenario_result, "target_edge", "")
        if scenario_result is not None and getattr(scenario_result, "target_edge", "")
        else st.session_state.traffic_target_edge
    )
    if traffic_target_edge not in autoware_edge_ids:
        traffic_target_edge = ""
    sync_process = st.session_state.traffic_process
    sync_running = sync_process is not None and sync_process.poll() is None
    sync_waiting_for_autoware = waiting_for_autoware_sync()
    sumo_gui_enabled = bool(st.session_state.traffic_sumo_gui)
    autoware_delay = effective_autoware_startup_wait_seconds(sumo_gui_enabled)

    top_cols = st.columns([1, 1])
    if SHOW_AUTOWARE_VEHICLE_TYPE_INPUT:
        top_cols = st.columns([2, 1, 1])
        with top_cols[0]:
            st.text_input(
                "SUMO/CARLA vehicle type",
                value=AUTOWARE_EGO_VTYPE,
                key="autoware_ego_carla_blueprint",
                disabled=True,
            )
    with top_cols[-2]:
        emission_model = st.selectbox(
            "Emission model",
            options=[ENERGY_EMISSION_CLASS, MMPEVEM_EMISSION_CLASS],
            key="autoware_ego_emission_model",
        )
    with top_cols[-1]:
        battery_capacity = st.number_input(
            "Max battery capacity [Wh]",
            min_value=1.0,
            max_value=500000.0,
            value=float(st.session_state.autoware_ego_max_battery_capacity),
            step=100.0,
            key="autoware_ego_max_battery_capacity",
        )

    if st.session_state.autoware_ego_initial_battery > battery_capacity:
        st.session_state.autoware_ego_initial_battery = battery_capacity

    if st.session_state.autoware_battery_failure_threshold > battery_capacity:
        st.session_state.autoware_battery_failure_threshold = battery_capacity

    battery_cols = st.columns(2)
    with battery_cols[0]:
        battery_charge_level = st.slider(
            "Current battery charge [Wh]",
            min_value=0.0,
            max_value=float(battery_capacity),
            step=max(1.0, float(battery_capacity) / 100.0),
            key="autoware_ego_initial_battery",
            help="Saved as `device.battery.chargeLevel` in the active vtypes.json.",
        )
    with battery_cols[1]:
        battery_failure_threshold = st.number_input(
            "Critical battery threshold [Wh]",
            min_value=0.0,
            max_value=float(battery_capacity),
            value=float(st.session_state.autoware_battery_failure_threshold),
            step=max(1.0, float(battery_capacity) / 100.0),
            key="autoware_battery_failure_threshold",
            help=(
                "If the Autoware ego battery drops below this threshold, the dashboard "
                "requests a stop and cancels the active route."
            ),
        )

    battery_threshold_valid = battery_charge_level > battery_failure_threshold
    if not battery_threshold_valid:
        st.warning(
            "The current battery must be greater than the critical threshold "
            "to avoid an immediate stop."
        )

    if st.session_state.autoware_ego_last_emission_model != emission_model:
        reset_vtype_model_state("autoware_ego", emission_model)
        st.session_state.autoware_ego_last_emission_model = emission_model

    attribute_defaults, parameter_defaults = ego_model_defaults(emission_model)

    with st.expander("Ego vType parameters", expanded=False):
        st.caption(
            "This updates the SUMO/vtypes metadata associated with the Autoware ego vehicle."
        )
        st.write("Attributes")
        attributes = parameter_input_grid(attribute_defaults, "autoware_ego_attr")
        st.write("Parameters")
        parameters = parameter_input_grid(
            parameter_defaults,
            "autoware_ego_param",
            text_area_keys=("powerLossMap",),
        )

    # Autoware Mini is a ROS 1 stack. Start and goal edges are converted into a
    # ROS initial pose and a ROS navigation goal after the docker launch starts.
    st.write("### Autoware Route")
    st.caption(
        "Select the route exactly like in the scenario step: click the map, choose the edge direction, "
        "or reuse the current congestion/source/destination edges with the quick actions below."
    )
    (
        farthest_autoware_start,
        farthest_autoware_goal,
        farthest_autoware_distance,
    ) = farthest_directed_edge_pair(autoware_edges)
    if st.checkbox(
        "Auto-select farthest Autoware route",
        key="autoware_use_farthest_route",
        disabled=not (farthest_autoware_start and farthest_autoware_goal),
        help=(
            "Selects the two directed edges with the largest reachable path length "
            "on the current map. Disable it to edit the route manually."
        ),
    ):
        st.session_state.autoware_start_edge = farthest_autoware_start
        st.session_state.autoware_goal_edge = farthest_autoware_goal
        st.caption(
            "Auto route: "
            f"`{farthest_autoware_start}` -> `{farthest_autoware_goal}` "
            f"({farthest_autoware_distance:.1f} m estimated path length)."
        )

    all_x = [point[0] for edge in autoware_edges for point in edge.shape]
    all_y = [point[1] for edge in autoware_edges for point in edge.shape]
    center_x = (min(all_x) + max(all_x)) / 2
    center_y = (min(all_y) + max(all_y)) / 2

    route_map = folium.Map(
        location=[center_y, center_x],
        zoom_start=1,
        crs="Simple",
        tiles=None,
    )
    for edge in autoware_edges:
        color = "#2563eb"
        weight = 4
        if edge.edge_id == traffic_target_edge:
            color = "#f97316"
            weight = 7
        if edge.edge_id == st.session_state.autoware_start_edge:
            color = "#16a34a"
            weight = 8
        elif edge.edge_id == st.session_state.autoware_goal_edge:
            color = "#dc2626"
            weight = 8

        coords = [to_map_coords(point[0], point[1]) for point in edge.shape]
        folium.PolyLine(
            coords,
            color=color,
            weight=weight,
            tooltip=edge.edge_id,
        ).add_to(route_map)

    if st.session_state.autoware_last_click:
        click_x, click_y = st.session_state.autoware_last_click
        folium.CircleMarker([click_y, click_x], radius=5, color="black", fill=True).add_to(route_map)

    route_map_data = st_folium(
        route_map,
        use_container_width=True,
        height=EDGE_SELECTION_MAP_HEIGHT,
        key=f"autoware_route_map_{selected_map_name}",
    )

    if route_map_data and route_map_data.get("last_clicked"):
        click_y = route_map_data["last_clicked"]["lat"]
        click_x = route_map_data["last_clicked"]["lng"]
        st.session_state.autoware_last_click = (click_x, click_y)

    if st.session_state.autoware_last_click:
        click_x, click_y = st.session_state.autoware_last_click
        candidate, distance = nearest_edge(autoware_edges, click_x, click_y)
        direction_options = edge_direction_options(autoware_edges, candidate.edge_id)
        if st.session_state.autoware_clicked_direction not in direction_options:
            st.session_state.autoware_clicked_direction = candidate.edge_id

        st.write(
            f"Nearest edge to click: `{candidate.edge_id}` "
            f"({distance:.1f} m)"
        )
        clicked_edge = st.selectbox(
            "Clicked edge direction",
            options=direction_options,
            format_func=lambda value: autoware_labels[value],
            key="autoware_clicked_direction",
        )

        click_cols = st.columns(4)
        with click_cols[0]:
            if st.button("Use as START", key="autoware_set_start_edge"):
                st.session_state.autoware_start_edge = clicked_edge
                st.rerun()
        with click_cols[1]:
            if st.button("Use as END", key="autoware_set_goal_edge"):
                st.session_state.autoware_goal_edge = clicked_edge
                st.rerun()
        with click_cols[2]:
            st.write("")
        with click_cols[3]:
            st.write("")
    else:
        st.info("Click on the map to select an Autoware edge.")

    edge_cols = st.columns(2)
    with edge_cols[0]:
        selected_start_edge = st.selectbox(
            "Autoware start edge",
            options=autoware_edge_options,
            index=edge_select_index(
                autoware_edge_options,
                st.session_state.autoware_start_edge or "",
            ),
            format_func=lambda value: "Select a start edge" if not value else autoware_labels[value],
        )
    with edge_cols[1]:
        selected_goal_edge = st.selectbox(
            "Autoware goal edge",
            options=autoware_edge_options,
            index=edge_select_index(
                autoware_edge_options,
                st.session_state.autoware_goal_edge or "",
            ),
            format_func=lambda value: "Select a goal edge" if not value else autoware_labels[value],
        )

    st.session_state.autoware_start_edge = selected_start_edge or None
    st.session_state.autoware_goal_edge = selected_goal_edge or None

    if st.session_state.autoware_start_edge and st.session_state.autoware_goal_edge:
        st.caption(
            "Autoware edge route: "
            f"`{st.session_state.autoware_start_edge}` -> `{st.session_state.autoware_goal_edge}`"
        )
    else:
        st.caption(
            "If start and goal edges are left empty, Autoware still launches but the route must be set manually in RViz."
        )

    if traffic_target_edge:
        st.caption(f"Current congestion edge: `{traffic_target_edge}`")

    route_action_cols = st.columns(6)
    start_opposite = (
        opposite_edge_id(st.session_state.autoware_start_edge, set(autoware_edge_ids))
        if st.session_state.autoware_start_edge else None
    )
    goal_opposite = (
        opposite_edge_id(st.session_state.autoware_goal_edge, set(autoware_edge_ids))
        if st.session_state.autoware_goal_edge else None
    )
    with route_action_cols[0]:
        if st.button(
            "Use congestion START",
            key="autoware_quick_congestion_start",
            disabled=not traffic_target_edge,
        ):
            st.session_state.autoware_start_edge = traffic_target_edge
            st.rerun()
    with route_action_cols[1]:
        if st.button(
            "Use congestion END",
            key="autoware_quick_congestion_end",
            disabled=not traffic_target_edge,
        ):
            st.session_state.autoware_goal_edge = traffic_target_edge
            st.rerun()
    with route_action_cols[2]:
        if st.button(
            "Use scenario START",
            key="autoware_use_scenario_start",
            disabled=st.session_state.start_edge not in autoware_edge_ids,
        ):
            st.session_state.autoware_start_edge = st.session_state.start_edge
            st.rerun()
    with route_action_cols[3]:
        if st.button(
            "Use scenario END",
            key="autoware_use_scenario_end",
            disabled=st.session_state.end_edge not in autoware_edge_ids,
        ):
            st.session_state.autoware_goal_edge = st.session_state.end_edge
            st.rerun()
    with route_action_cols[4]:
        if st.button("Invert START", key="autoware_invert_start", disabled=not start_opposite):
            st.session_state.autoware_start_edge = start_opposite
            st.rerun()
    with route_action_cols[5]:
        if st.button("Invert END", key="autoware_invert_end", disabled=not goal_opposite):
            st.session_state.autoware_goal_edge = goal_opposite
            st.rerun()

    if SHOW_AUTOWARE_SPEED_CAP_INPUT:
        st.number_input(
            "Autoware planner speed cap [km/h]",
            min_value=5.0,
            max_value=130.0,
            value=50.0,
            step=1.0,
            key="autoware_planner_speed_limit_kmh",
            help=(
                "Autoware Mini uses map speed limits only if the Lanelet2 map contains them. "
                "These Town01/Town04/Town05 maps currently do not, so this value acts as the effective route speed cap."
            ),
        )
        st.caption(
            "The current Lanelet2 Town maps do not expose per-lane `speed_limit` or `speed_ref` tags, "
            "so the planner falls back to the configured global cap loaded at Autoware startup."
        )

    st.info(
        "If CARLA does not have the UT Lexus asset imported, the Docker launch will fail before spawning the ego vehicle."
    )
    if scenario_result is None:
        st.caption("Generate a SUMO scenario in step 2 before starting the synchronized launch.")
    else:
        if SHOW_SUMO_GUI_INPUT:
            gui_label = "enabled" if st.session_state.traffic_sumo_gui else "disabled"
            st.caption(
                f"Step 2 will reuse the running CARLA instance on map `{scenario_result.map_name}` "
                f"with SUMO GUI {gui_label}."
            )
        else:
            st.caption(
                f"Step 2 will reuse the running CARLA instance on map `{scenario_result.map_name}`."
            )
    if sync_waiting_for_autoware:
        st.info(
            "SUMO/CARLA is already armed from step 3 and waiting for Autoware. "
            "Click `Spawn Autoware` to start the warm-up and then release the simulation."
        )
    elif sync_running:
        st.info(
            "SUMO/CARLA is already running. Autoware will attach to the existing CARLA ticks "
            "instead of driving the simulator clock itself."
        )
    if SHOW_SUMO_GUI_INPUT or SHOW_AUTOWARE_STARTUP_WAIT_INPUT:
        if sumo_gui_enabled:
            st.caption(
                "SUMO GUI is enabled, so the Autoware startup wait is disabled and the simulation "
                "is released immediately when you click `Spawn Autoware`."
            )
        else:
            st.caption(f"Autoware startup wait is `{autoware_delay}s`.")

    route_start_edge = st.session_state.autoware_start_edge
    route_goal_edge = st.session_state.autoware_goal_edge
    action_cols = st.columns(3)
    with action_cols[0]:
        save_clicked = st.button("Save ego vType") # in vtypes.json
    with action_cols[1]:
        spawn_clicked = st.button(
            "Spawn Ego Vehicle",
            disabled=not battery_threshold_valid,
        )
    with action_cols[2]:
        route_clicked = st.button(
            "Start Ego Vehicle route",
            disabled=not (route_start_edge and route_goal_edge),
        )

    if save_clicked or spawn_clicked:
        autoware_vtype_params = dict(parameters)
        autoware_vtype_params["dashboard.battery.failureThreshold"] = str(
            float(battery_failure_threshold)
        )
        saved_vtypes_file = write_autoware_ego_vtype_config(
            emission_model,
            battery_capacity,
            battery_charge_level,
            attributes=attributes,
            parameters=autoware_vtype_params,
        )
        if save_clicked:
            st.success(f"`{AUTOWARE_EGO_VTYPE}` updated in vtypes.json.")
            st.caption(
                f"Saved `{saved_vtypes_file}` with color `{attributes.get('color', '-')}`."
            )

    if spawn_clicked:
        selected_start_edge = st.session_state.autoware_start_edge
        selected_goal_edge = st.session_state.autoware_goal_edge
        if bool(selected_start_edge) != bool(selected_goal_edge):
            st.error("Select both the Autoware start edge and the Autoware goal edge.")
            return {
                "sumo_vtype": AUTOWARE_EGO_VTYPE,
                "carla_blueprint": AUTOWARE_EGO_VTYPE,
                "emission_model": emission_model,
                "battery_capacity": battery_capacity,
                "battery_charge_level": battery_charge_level,
                "attributes": attributes,
                "parameters": parameters,
                "emission_class": ego_emission_class_value(emission_model),
            }
        try:
            started_at = time.time()
            carla_bridge_passive = bool(sync_running and not sync_waiting_for_autoware)
            launch = launch_autoware_carla_in_container(
                selected_map_name,
                spawn_edge=selected_start_edge or st.session_state.start_edge,
                start_edge=selected_start_edge,
                goal_edge=selected_goal_edge,
                speed_limit_kmh=float(st.session_state.autoware_planner_speed_limit_kmh),
                carla_bridge_passive=carla_bridge_passive,
                publish_route=False,
            )
            st.session_state.autoware_last_launch = {
                "map_name": selected_map_name,
                "container_name": launch.get("container_name"),
                "command": launch.get("command"),
                "carla_bridge_passive": launch.get("carla_bridge_passive"),
                "spawn_edge": launch.get("spawn_edge"),
                "spawn_point": launch.get("spawn_point"),
                "spawn_point_passthrough": launch.get("spawn_point_passthrough"),
                "started_at": started_at,
                "startup_wait_seconds": autoware_delay,
                "start_edge": selected_start_edge,
                "goal_edge": selected_goal_edge,
                "speed_limit_kmh": float(st.session_state.autoware_planner_speed_limit_kmh),
                "battery_failure_threshold": float(battery_failure_threshold),
                "planner_speed_limit_setup": launch.get("planner_speed_limit_setup"),
                "dynamic_speed_limit_setup": launch.get("dynamic_speed_limit_setup"),
                "route_deferred": launch.get("route_deferred"),
                "route_publication": None,
                "route_publication_error": None,
                "route_started_at": None,
            }
            st.session_state.autoware_active_battery_failure_threshold = float(
                battery_failure_threshold
            )
        except Exception as exc:
            st.error(f"Autoware start failed: {exc}")
            return {
                "sumo_vtype": AUTOWARE_EGO_VTYPE,
                "carla_blueprint": AUTOWARE_EGO_VTYPE,
                "emission_model": emission_model,
                "battery_capacity": battery_capacity,
                "battery_charge_level": battery_charge_level,
                "attributes": attributes,
                "parameters": parameters,
                "emission_class": ego_emission_class_value(emission_model),
            }

        st.success(
            f"Autoware spawn launch started in container `{launch['container_name']}` "
            f"for map `{selected_map_name}`."
        )
        st.caption(f"Host prep: `{launch['host_command']}` with `DISPLAY={launch['display']}`")
        st.caption(f"Executed: `{launch['command']}`")
        if launch.get("carla_bridge_passive"):
            st.caption("CARLA ROS bridge mode: passive, using the already running SUMO/CARLA tick loop.")
        if launch.get("spawn_point"):
            st.caption(
                f"Autoware spawn point from `{launch.get('spawn_edge')}`: `{launch['spawn_point']}`"
            )
        spawn_point_passthrough = launch.get("spawn_point_passthrough") or {}
        if spawn_point_passthrough.get("status"):
            st.caption(
                f"Autoware spawn-point passthrough: `{spawn_point_passthrough['status']}`."
            )
        if selected_start_edge and selected_goal_edge:
            st.caption(
                f"Pending ROS route: `{selected_start_edge}` -> `{selected_goal_edge}`. "
                "Use `Start Autoware route` after the vehicle is spawned."
            )
        initial_pose_data = launch.get("initial_pose") or {}
        goal_pose_data = launch.get("goal_pose") or {}
        if initial_pose_data and goal_pose_data:
            st.caption(
                "Resolved ROS poses with "
                f"`{initial_pose_data.get('projection_mode')}` projection."
            )
        st.caption(
            f"Planner speed cap: `{float(st.session_state.autoware_planner_speed_limit_kmh):.0f} km/h`."
        )
        st.caption(
            f"Critical battery threshold: `{float(battery_failure_threshold):.1f} Wh`."
        )
        planner_speed_limit_setup = launch.get("planner_speed_limit_setup") or {}
        if planner_speed_limit_setup.get("planning_yaml"):
            st.caption(
                "Planner config written to "
                f"`{planner_speed_limit_setup['planning_yaml']}` before `roslaunch`."
            )
        dynamic_speed_limit_setup = launch.get("dynamic_speed_limit_setup") or {}
        if dynamic_speed_limit_setup:
            dynamic_status = (
                "updated"
                if dynamic_speed_limit_setup.get("updated")
                else "already present"
            )
            st.caption(f"Dynamic route speed-cap handling: `{dynamic_status}`.")
        if sync_waiting_for_autoware:
            try:
                spinner_message = (
                    "Releasing the SUMO/CARLA simulation immediately after the Autoware spawn..."
                    if autoware_delay == 0
                    else (
                        f"Waiting {autoware_delay}s for Autoware startup, then releasing the "
                        "SUMO/CARLA simulation..."
                    )
                )
                with st.spinner(spinner_message):
                    time.sleep(max(0, autoware_delay))
                    if not waiting_for_autoware_sync():
                        raise RuntimeError(
                            "SUMO/CARLA is no longer waiting for Autoware. "
                            "The synchronization process may have stopped."
                        )
                    release_waiting_synchronization_gate()
                st.success("Autoware warm-up completed. SUMO/CARLA simulation released.")
            except Exception as exc:
                st.error(f"Autoware started, but the SUMO release failed: {exc}")
        else:
            if sync_running:
                st.caption("SUMO/CARLA was already running; Autoware attached without releasing a start gate.")
            elif autoware_delay == 0:
                st.caption(
                    "Warm-up wait is disabled. Step 3 can start the simulation immediately."
                )
            else:
                st.caption(
                    f"Warm-up timer started now. Step 3 can start the simulation after `{autoware_delay}s`."
                )

    if route_clicked:
        selected_start_edge = st.session_state.autoware_start_edge
        selected_goal_edge = st.session_state.autoware_goal_edge
        if not selected_start_edge or not selected_goal_edge:
            st.error("Select both the Autoware start edge and the Autoware goal edge.")
            return {
                "sumo_vtype": AUTOWARE_EGO_VTYPE,
                "carla_blueprint": AUTOWARE_EGO_VTYPE,
                "emission_model": emission_model,
                "battery_capacity": battery_capacity,
                "battery_charge_level": battery_charge_level,
                "attributes": attributes,
                "parameters": parameters,
                "emission_class": ego_emission_class_value(emission_model),
            }

        last_launch = st.session_state.get("autoware_last_launch") or {}
        spawned_on_selected_start = (
            last_launch.get("container_name")
            and last_launch.get("map_name") == selected_map_name
            and last_launch.get("spawn_edge") == selected_start_edge
        )
        publish_initial_pose = False
        try:
            route = publish_autoware_route_in_container(
                selected_map_name,
                container_name=last_launch.get("container_name"),
                start_edge=selected_start_edge,
                goal_edge=selected_goal_edge,
                speed_limit_kmh=float(st.session_state.autoware_planner_speed_limit_kmh),
                publish_initial_pose=publish_initial_pose,
            )
            updated_launch = dict(last_launch)
            updated_launch.update(
                {
                    "map_name": selected_map_name,
                    "container_name": route.get("container_name"),
                    "start_edge": selected_start_edge,
                    "goal_edge": selected_goal_edge,
                    "speed_limit_kmh": float(st.session_state.autoware_planner_speed_limit_kmh),
                    "route_deferred": False,
                    "route_publication": route.get("route_publication"),
                    "route_publication_error": None,
                    "route_started_at": time.time(),
                    "initial_pose": route.get("initial_pose"),
                    "goal_pose": route.get("goal_pose"),
                    "initial_pose_published": route.get("initial_pose_published"),
                    "dynamic_speed_limit_setup": route.get("dynamic_speed_limit_setup"),
                }
            )
            st.session_state.autoware_last_launch = updated_launch
            st.success(
                f"Autoware route started: `{selected_start_edge}` -> `{selected_goal_edge}`."
            )
            st.caption(
                f"Published only `/move_base_simple/goal` in container "
                f"`{route['container_name']}`; the already spawned ego pose and SUMO battery state were preserved."
            )
            if not spawned_on_selected_start:
                st.caption(
                    "The selected start edge differs from the last recorded Autoware spawn. "
                    "Use `Spawn Autoware` again if you need to reposition the ego vehicle."
                )
        except Exception as exc:
            last_launch = dict(last_launch)
            last_launch["route_publication_error"] = str(exc)
            st.session_state.autoware_last_launch = last_launch
            st.error(f"Autoware route start failed: {exc}")

    return {
        "sumo_vtype": AUTOWARE_EGO_VTYPE,
        "carla_blueprint": AUTOWARE_EGO_VTYPE,
        "emission_model": emission_model,
        "battery_capacity": battery_capacity,
        "battery_charge_level": battery_charge_level,
        "attributes": attributes,
        "parameters": parameters,
        "emission_class": ego_emission_class_value(emission_model),
    }


def get_live_sumo_vehicles():
    """Fetch the live SUMO vehicles exposed by the dashboard backend."""
    response = requests.get(f"{API_URL}/vehicles", timeout=2)
    if response.status_code != 200:
        raise RuntimeError(response.text)
    return response.json().get("vehicles", [])


def is_carla_spawned_vehicle(vehicle):
    """Return whether a live vehicle was spawned from the CARLA side."""
    vehicle_id = str(vehicle.get("id", ""))
    type_id = str(vehicle.get("type_id", ""))
    return vehicle_id.startswith("carla") or type_id == "vehicle.lexus.utlexus"


def is_autoware_monitoring_vehicle_id(vehicle_id):
    """Return whether a SUMO vehicle id is an Autoware CARLA mirror."""
    return str(vehicle_id or "").startswith("carla")


def vehicle_display_label(vehicle):
    """Build a human-readable label for a live vehicle."""
    battery_state = "battery" if vehicle.get("has_battery_device") else "no battery"
    return (
        f"{vehicle.get('id', '-')} | type={vehicle.get('type_id', '-')} | "
        f"edge={vehicle.get('edge', '-')} | {battery_state}"
    )


def preferred_monitoring_vehicle_id(vehicles):
    """Pick the default vehicle to monitor from the available live vehicles."""
    for vehicle in vehicles:
        if is_carla_spawned_vehicle(vehicle):
            return vehicle["id"]

    for vehicle in vehicles:
        if vehicle.get("id") == "ego_vehicle":
            return vehicle["id"]

    return vehicles[0]["id"] if vehicles else None


def fetch_live_vehicle_vtype_config(veh_id):
    """Fetch the current live vType configuration for a vehicle."""
    encoded_vehicle_id = quote(str(veh_id), safe="")
    response = requests.get(f"{API_URL}/vehicle/{encoded_vehicle_id}", timeout=2)
    if response.status_code != 200:
        raise RuntimeError(response.text)
    return response.json()


def initialize_live_vehicle_config_state(veh_id):
    """Initialize session state for editing a live vehicle type."""
    current_version = active_carla_version()
    if (
        st.session_state.live_vehicle_config_loaded_for == veh_id
        and st.session_state.live_vehicle_config_version == current_version
    ):
        return

    config = fetch_live_vehicle_vtype_config(veh_id)
    apply_vtype_config_to_state("live_vehicle", config)
    st.session_state.live_vehicle_config_loaded_for = veh_id
    st.session_state.live_vehicle_config_version = current_version


def render_live_vehicle_vtype_config(veh_id):
    """Render the editor for a live vehicle vType."""
    initialize_live_vehicle_config_state(veh_id)

    current_type_id = st.session_state.get("live_vehicle_sumo_vtype", "")
    has_battery_device = bool(st.session_state.get("live_vehicle_has_battery_device"))
    if current_type_id:
        st.caption(f"Current SUMO vType: `{current_type_id}`")
    st.caption(
        "The update changes only SUMO vType data for this vehicle. "
        "Route and CARLA blueprint stay unchanged."
    )

    top_cols = st.columns([2, 1, 1])
    with top_cols[0]:
        st.text_input(
            "CARLA vType / blueprint",
            value=st.session_state.live_vehicle_carla_blueprint,
            key="live_vehicle_carla_blueprint",
            disabled=True,
        )
    with top_cols[1]:
        emission_model = st.selectbox(
            "Energy model",
            options=[ENERGY_EMISSION_CLASS, MMPEVEM_EMISSION_CLASS],
            key="live_vehicle_emission_model",
        )
    with top_cols[2]:
        battery_capacity = st.number_input(
            "Max battery capacity [Wh]",
            min_value=1.0,
            max_value=500000.0,
            value=float(st.session_state.live_vehicle_max_battery_capacity),
            step=100.0,
            key="live_vehicle_max_battery_capacity",
            disabled=not has_battery_device,
        )

    if st.session_state.live_vehicle_last_emission_model != emission_model:
        reset_vtype_model_state("live_vehicle", emission_model)
        st.session_state.live_vehicle_last_emission_model = emission_model

    attribute_defaults, parameter_defaults = ego_model_defaults(emission_model)

    with st.expander("Live vType parameters", expanded=False):
        st.caption(
            "Only vType data is edited here. Path, route, and departure remain unchanged."
        )
        st.write("Attributes")
        attributes = parameter_input_grid(attribute_defaults, "live_vehicle_attr")
        st.write("Parameters")
        parameters = parameter_input_grid(
            parameter_defaults,
            "live_vehicle_param",
            text_area_keys=("powerLossMap",),
        )

    return {
        "vehicle_id": veh_id,
        "sumo_vtype": current_type_id,
        "carla_blueprint": st.session_state.live_vehicle_carla_blueprint,
        "emission_model": emission_model,
        "battery_capacity": battery_capacity,
        "attributes": attributes,
        "parameters": parameters,
        "emission_class": ego_emission_class_value(emission_model),
        "has_battery_device": has_battery_device,
    }


def resolve_ego_route_from_congestion(
    edges,
    selected_start,
    selected_end,
    target_edge,
    route_mode,
    traffic_source_edge=None,
    traffic_destination_edge=None,
):
    """Build ego routing choices starting from the selected congestion edge."""
    start_edge = selected_start
    end_edge = selected_end
    via_edge = None
    auto_fields = []

    if route_mode == EGO_ROUTE_TO_CONGESTION:
        end_edge = target_edge
        if not start_edge or start_edge == end_edge:
            start_edge = pick_auto_edge(
                edges,
                excluded={target_edge},
                preferred=(traffic_source_edge,),
            )
            if start_edge:
                auto_fields.append("start")

    elif route_mode == EGO_ROUTE_FROM_CONGESTION:
        start_edge = target_edge
        if not end_edge or end_edge == start_edge:
            end_edge = pick_auto_edge(
                edges,
                excluded={target_edge},
                preferred=(traffic_destination_edge,),
            )
            if end_edge:
                auto_fields.append("destination")

    else:
        via_edge = target_edge
        if not start_edge:
            start_edge = pick_auto_edge(
                edges,
                excluded={target_edge, end_edge},
                preferred=(traffic_source_edge,),
            )
            if start_edge:
                auto_fields.append("start")

        if not end_edge:
            end_edge = pick_auto_edge(
                edges,
                excluded={target_edge, start_edge},
                preferred=(traffic_destination_edge,),
            )
            if end_edge:
                auto_fields.append("destination")

    return start_edge, end_edge, via_edge, auto_fields


def render_traffic_scenario(show_runner=True):
    """Render the scenario-generation step and its routing tools."""
    st.subheader("🚦 Traffic Scenario")

    maps = available_maps()
    if not maps:
        st.error("No SUMO map found in examples/net.")
        return

    map_name, _ = apply_selected_runtime_map()
    st.caption(
        f"Scenario Town fixed to `{map_name}` from step 1."
    )

    edges = get_offline_edges(map_name)
    edge_ids = [edge.edge_id for edge in edges]
    labels = {edge.edge_id: edge_label(edge) for edge in edges}

    for state_key in (
        "traffic_target_edge",
        "traffic_source_edge",
        "traffic_destination_edge",

    ):
        if st.session_state[state_key] not in edge_ids:
            st.session_state[state_key] = ""

    mode = st.radio(
        "Scenario type",
        ["Congestion Edge", "Random Traffic"],
        horizontal=True,
    )

    st.write("### Map")

    all_x = [point[0] for edge in edges for point in edge.shape]
    all_y = [point[1] for edge in edges for point in edge.shape]
    center_x = (min(all_x) + max(all_x)) / 2
    center_y = (min(all_y) + max(all_y)) / 2

    target_edge = st.session_state.traffic_target_edge
    source_edge = st.session_state.traffic_source_edge
    destination_edge = st.session_state.traffic_destination_edge


    m = folium.Map(
        location=[center_y, center_x],
        zoom_start=1,
        crs="Simple",
        tiles=None,
    )

    for edge in edges:
        color = "#2563eb"
        weight = 4

        if edge.edge_id == target_edge:
            color = "#dc2626"
            weight = 8
        elif edge.edge_id == source_edge:
            color = "#9333ea"
            weight = 7
        elif edge.edge_id == destination_edge:
            color = "#16a34a"
            weight = 8


        coords = [to_map_coords(point[0], point[1]) for point in edge.shape]
        folium.PolyLine(
            coords,
            color=color,
            weight=weight,
            tooltip=edge.edge_id,
        ).add_to(m)

    if st.session_state.traffic_last_click:
        x, y = st.session_state.traffic_last_click
        folium.CircleMarker([y, x], radius=5, color="black", fill=True).add_to(m)

    map_data = st_folium(
        m,
        use_container_width=True,
        height=EDGE_SELECTION_MAP_HEIGHT,
        key=f"traffic_map_{map_name}",
    )

    if map_data and map_data.get("last_clicked"):
        y = map_data["last_clicked"]["lat"]
        x = map_data["last_clicked"]["lng"]
        st.session_state.traffic_last_click = (x, y)

    if st.session_state.traffic_last_click:
        x, y = st.session_state.traffic_last_click
        candidate, distance = nearest_edge(edges, x, y)
        direction_options = edge_direction_options(edges, candidate.edge_id)
        if st.session_state.traffic_clicked_direction not in direction_options:
            st.session_state.traffic_clicked_direction = candidate.edge_id

        st.write(
            f"Nearest edge to click: `{candidate.edge_id}` "
            f"({distance:.1f} m)"
        )

        clicked_edge = st.selectbox(
            "Clicked edge direction",
            options=direction_options,
            format_func=lambda value: labels[value],
            key="traffic_clicked_direction",
        )

        click_cols = st.columns(3)
        with click_cols[0]:
            if st.button("Use as congestion"):
                st.session_state.traffic_target_edge = clicked_edge
                st.rerun()
        with click_cols[1]:
            if st.button("Use as source"):
                st.session_state.traffic_source_edge = clicked_edge
                st.rerun()
        with click_cols[2]:
            if st.button("Use as destination"):
                st.session_state.traffic_destination_edge = clicked_edge
                st.rerun()

    else:
        st.info("Click on the map to select an edge.")

    st.write("### Parameters")

    empty_label = "Select by clicking on the map or on the list"
    target_options = [""] + edge_ids
    target_selection = st.selectbox(
        "Edge to congestion",
        options=target_options,
        index=edge_select_index(target_options, st.session_state.traffic_target_edge),
        format_func=lambda value: empty_label if not value else labels[value],
        disabled=mode == "Random Traffic",
    )

    source_options = [""] + edge_ids
    source_selection = st.selectbox(
        "Source Edge (optional)",
        options=source_options,
        index=edge_select_index(source_options, st.session_state.traffic_source_edge),
        format_func=lambda value: "Random over the whole map" if not value else labels[value],
        disabled=mode == "Random Traffic",
    )

    destination_selection = st.selectbox(
        "Destination Edge (optional)",
        options=target_options,
        index=edge_select_index(target_options, st.session_state.traffic_destination_edge),
        format_func=lambda value: "Random over the whole map" if not value else labels[value],
        disabled=mode == "Random Traffic",
    )



    st.session_state.traffic_target_edge = target_selection
    st.session_state.traffic_source_edge = source_selection
    st.session_state.traffic_destination_edge = destination_selection


    if mode != "Random Traffic":
        edge_id_set = set(edge_ids)
        direction_cols = st.columns(3)
        target_opposite = (
            opposite_edge_id(target_selection, edge_id_set)
            if target_selection else None
        )
        source_opposite = (
            opposite_edge_id(source_selection, edge_id_set)
            if source_selection else None
        )
        destination_opposite = (
            opposite_edge_id(destination_selection, edge_id_set)
            if destination_selection else None
        )


        with direction_cols[0]:
            if st.button(
                "Invert congestion",
                disabled=not target_opposite,
            ):
                st.session_state.traffic_target_edge = target_opposite
                st.rerun()
        with direction_cols[1]:
            if st.button(
                "Invert source",
                disabled=not source_opposite,
            ):
                st.session_state.traffic_source_edge = source_opposite
                st.rerun()
        with direction_cols[2]:
            if st.button(
                "Invert destination",
                disabled=not destination_opposite,
            ):
                st.session_state.traffic_destination_edge = destination_opposite
                st.rerun()



    param_cols = st.columns(4)
    with param_cols[0]:
        vehicle_count = st.number_input(
            "Vehicles Number",
            min_value=0,
            max_value=5000,
            value=50,
            step=10,
            help="Set 0 to generate an empty SUMO traffic scenario and run only the ego vehicle.",
        )
    with param_cols[1]:
        begin = st.number_input("Start spawn at t[s]", min_value=0.0, value=0.0, step=1.0)
    with param_cols[2]:
        end = st.number_input("Stop spawn at t[s]", min_value=0.0, value=120.0, step=1.0)
    with param_cols[3]:
        seed = st.number_input("Seed", min_value=0, max_value=999999, value=42, step=1)

    if float(st.session_state.traffic_simulation_end) < float(end):
        st.session_state.traffic_simulation_end = float(end)
    simulation_end = float(st.session_state.traffic_simulation_end)
    if SHOW_TRAFFIC_SIMULATION_END_INPUT:
        simulation_end = st.number_input(
            "Simulation end t[s]",
            min_value=float(end),
            step=10.0,
            key="traffic_simulation_end",
            help="Written as `<time><end>` in the generated custom SUMO configuration.",
        )

    spawn_pattern = "Equidistant"
    if SHOW_TRAFFIC_SPAWN_DISTRIBUTION_INPUT:
        spawn_pattern = st.selectbox(
            "Spawn distribution",
            ["Equidistant", "Randomly", "All together"],
            disabled=mode == "Random Traffic",
        )

    vtype_options = get_sumo_vtypes()
    if not vtype_options:
        st.error("No SUMO vType found in carlavtypes/egovtype files.")
        return

    st.write("##### Vehicle Type")
    vtype_cols = st.columns([1, 3])
    with vtype_cols[0]:
        random_vehicle_type = st.checkbox("Random vType", value=True)
    with vtype_cols[1]:
        vehicle_type = st.selectbox(
            "SUMO vType",
            options=vtype_options,
            index=edge_select_index(vtype_options, DEFAULT_VEHICLE_TYPE),
            disabled=random_vehicle_type,
        )

    if random_vehicle_type:
        st.caption(
            f"Each vehicle will use a random vType among {len(vtype_options)} available types."
        )

    if mode == "Congestion Edge" and target_selection:
        st.info(
            "Source and destination are optional. If left blank, "
            "generate random traffic but only keep the routes that pass through the congested edge."
        )

    can_generate = int(vehicle_count) == 0 or mode == "Random Traffic" or bool(target_selection)
    if not can_generate:
        st.warning("Select the edge to congestion.")

    if st.button("Generate route and SUMO Configuration", disabled=not can_generate):
        try:
            if mode == "Congestion Edge":
                result = generate_congestion_scenario(
                    map_name=map_name,
                    target_edge=target_selection,
                    destination_edge=destination_selection or None,
                    vehicle_count=int(vehicle_count),
                    begin=float(begin),
                    end=float(end),
                    simulation_end=float(simulation_end),
                    spawn_pattern=spawn_pattern,
                    source_edge=source_selection or None,
                    seed=int(seed),
                    vehicle_type=vehicle_type,
                    random_vehicle_type=random_vehicle_type,
                    vehicle_types=vtype_options,
                )
            else:
                result = generate_random_trips_scenario(
                    map_name=map_name,
                    vehicle_count=int(vehicle_count),
                    begin=float(begin),
                    end=float(end),
                    simulation_end=float(simulation_end),
                    seed=int(seed),
                    vehicle_type=vehicle_type,
                    random_vehicle_type=random_vehicle_type,
                    vehicle_types=vtype_options,
                )

            st.session_state.traffic_generation_result = result
            st.success(
                f"Generated {result.generated_count}/{result.requested_count} vehicles."
            )
        except Exception as exc:
            st.error(f"Generation failed: {exc}")

    result = st.session_state.traffic_generation_result
    if result:
        st.write("### Output")
        metric_cols = st.columns(4)
        metric_cols[0].metric("Route Vehicles", result.generated_count)
        metric_cols[1].metric("crossing edge", result.target_count)
        metric_cols[2].metric("Tool", result.mode)
        metric_cols[3].metric(
            "Simulation end",
            f"{getattr(result, 'simulation_end', 0.0):.0f} s",
        )

        st.write("Route file:", str(result.route_file))
        st.write("Trips file:", str(result.trip_file))
        st.write("SUMO cfg:", str(result.sumocfg_file))
        st.code(
            "cd " + str(current_sumo_dir()) + "\n" + " ".join(sync_launch_command_for_ui(result.sumocfg_file)),
            language="bash",
        )

        if show_runner:
            render_simulation_runner(result)


def render_simulation_runner(result=None):
    """Render the co-simulation launch controls and runtime status."""
    if result is None:
        result = st.session_state.traffic_generation_result

    autoware_workflow = active_carla_version() == "0.9.13"
    st.subheader("▶️ Configure Simulation" if autoware_workflow else "▶️ Run Simulation")
    if result is None:
        st.warning(
            "Generate a SUMO route scenario first in step 2 before starting the co-simulation."
        )
        return

    sync_process = st.session_state.traffic_process
    sync_running = sync_process is not None and sync_process.poll() is None
    carla_server_ready = is_carla_server_ready()
    sync_launch_widget_defaults()

    if autoware_workflow:
        st.caption(
            f"Scenario ready for map `{result.map_name}`. "
            "Start SUMO/CARLA here, then launch Autoware from step 4 if needed."
        )
    else:
        st.caption(
            f"Scenario ready for map `{result.map_name}`. "
            "This step loads the correct Town in CARLA and then starts the SUMO/CARLA synchronization."
        )
    if autoware_workflow:
        st.info(
            "For the Autoware workflow, this step starts SUMO/CARLA immediately. "
            "Autoware can be launched later from step 4 and will attach to the running simulation."
        )
        if waiting_for_autoware_sync():
            st.success(
                "SUMO/CARLA bridge is running and waiting for Autoware before advancing simulation time."
            )
            launch_state = autoware_launch_sync_state()
            if launch_state is None:
                st.info("Autoware has not been launched from step 4 yet.")
            elif launch_state["is_ready"]:
                st.success(
                    f"Autoware launched in container `{launch_state['container_name']}`. "
                    "Warm-up completed."
                )
            else:
                st.info(
                    f"Autoware launched in container `{launch_state['container_name']}`. "
                    f"{launch_state['remaining_seconds']:.1f}s of startup wait remain."
                )
        else:
            st.warning("Start simulation here first to arm SUMO/CARLA, then launch Autoware from step 3.")

    if SHOW_SUMO_GUI_INPUT:
        sumo_gui = st.checkbox(
            "SUMO GUI",
            key="traffic_sumo_gui_selection",
            help="Disable this to run SUMO headless.",
        )
    else:
        st.session_state.traffic_sumo_gui = True
        st.session_state.traffic_sumo_gui_selection = True
        sumo_gui = True

    if autoware_workflow and SHOW_AUTOWARE_STARTUP_WAIT_INPUT:
        st.number_input(
            "Autoware startup wait [s]",
            min_value=0,
            max_value=120,
            step=1,
            key="autoware_sync_delay_selection",
            disabled=bool(sumo_gui),
            help=(
                "Warm-up delay used only when Autoware is launched before releasing SUMO. "
                "Disabled when SUMO GUI is enabled."
            ),
        )
    st.session_state.traffic_carla_mode = "reuse"
    st.session_state.traffic_sumo_gui = bool(sumo_gui)
    if autoware_workflow and SHOW_AUTOWARE_STARTUP_WAIT_INPUT:
        st.session_state.autoware_sync_delay_seconds = int(
            st.session_state.autoware_sync_delay_selection
        )
        if sumo_gui:
            st.caption("Autoware startup wait is disabled while SUMO GUI is enabled.")

    if carla_server_ready:
        st.caption(
            f"CARLA server detected on {DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}. "
            "The dashboard will still load the scenario Town before starting the co-simulation."
        )
    else:
        st.caption(
            f"No CARLA server detected on {DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}. "
            "Start CARLA from step 1 before running the co-simulation."
        )

    run_cols = st.columns(2)
    with run_cols[0]:
        if st.button(
            "Start simulation" if autoware_workflow else "Run co-simulation",
            disabled=sync_running,
        ):
            try:
                spinner_message = (
                    "Loading the selected Town and starting SUMO/CARLA..."
                    if autoware_workflow
                    else "Loading the selected Town and starting SUMO/co-simulation..."
                )

                with st.spinner(spinner_message):
                    launch = start_dashboard_synchronization_launch(
                        result,
                        carla_mode="reuse",
                        carla_timeout=int(st.session_state.traffic_carla_timeout_seconds),
                        sumo_gui=sumo_gui,
                        wait_for_autoware=False,
                    )
                st.success(
                    synchronization_success_message(
                        launch,
                        result,
                        carla_mode="reuse",
                        sumo_gui=sumo_gui,
                    )
                )
            except Exception as exc:
                st.error(f"Start failed: {exc}")
    with run_cols[1]:
        if st.button("Stop co-simulation", disabled=not sync_running):
            sync_process.terminate()
            st.session_state.traffic_process = None
            clear_waiting_synchronization_state(remove_gate_file=True)
            st.warning("Process stopped.")

    sync_process = st.session_state.traffic_process
    if sync_process is not None and sync_process.poll() is None:
        st.info(f"Co-simulation started, PID {sync_process.pid}.")
    elif sync_process is not None:
        st.info(f"Last process ended with code {sync_process.returncode}.")

    carla_process = st.session_state.carla_process
    if carla_process is not None and carla_process.poll() is None:
        st.info(f"CARLA running, PID {carla_process.pid}.")
    elif carla_server_ready:
        st.info(
            f"CARLA server detected on {DEFAULT_CARLA_HOST}:{DEFAULT_CARLA_PORT}."
        )
    elif carla_process is not None:
        st.info(f"Last CARLA process ended with code {carla_process.returncode}.")

    if st.session_state.traffic_process_log:
        st.caption(f"Co-simulation Log: {st.session_state.traffic_process_log}")
    if st.session_state.carla_process_log:
        st.caption(f"CARLA Log: {st.session_state.carla_process_log}")


def render_setup():
    """Render the full dashboard workflow and shared setup state."""
    if not backend_alive:
        st.warning(
            "Backend is not active: you can configure the ego route and vType, "
            "but spawning will only be available after the co-simulation starts."
        )

    traffic_result = st.session_state.traffic_generation_result
    map_name = (
        traffic_result.map_name
        if traffic_result is not None
        else st.session_state.traffic_map_name
    )
    edges = get_offline_edges(map_name)
    edge_ids = [edge.edge_id for edge in edges]
    edge_id_set = set(edge_ids)
    labels = {edge.edge_id: edge_label(edge) for edge in edges}

    if st.session_state.start_edge not in edge_id_set:
        st.session_state.start_edge = None
    if st.session_state.end_edge not in edge_id_set:
        st.session_state.end_edge = None

    # =====================================================
    # MAP (STATIC - NOT REFRESHED LOGICALLY)
    # =====================================================

    st.subheader("🗺️ Network Map (click to select)")
    st.caption(f"Ego map: {map_name}")
    farthest_start_edge, farthest_end_edge, farthest_distance = farthest_directed_edge_pair(edges)
    if st.checkbox(
        "Auto-select farthest ego route",
        key="ego_use_farthest_route",
        disabled=not (farthest_start_edge and farthest_end_edge),
        help=(
            "Selects the two directed edges with the largest reachable path length "
            "on the current map. Disable it to edit start/end manually."
        ),
    ):
        st.session_state.start_edge = farthest_start_edge
        st.session_state.end_edge = farthest_end_edge
        st.caption(
            "Auto route: "
            f"`{farthest_start_edge}` -> `{farthest_end_edge}` "
            f"({farthest_distance:.1f} m estimated path length)."
        )

    all_x = [point[0] for edge in edges for point in edge.shape]
    all_y = [point[1] for edge in edges for point in edge.shape]

    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    m = folium.Map(
        location=[center_y, center_x],
        zoom_start=1,
        crs="Simple",
        tiles=None,
    )

    traffic_target_edge = (
        getattr(traffic_result, "target_edge", "")
        if traffic_result is not None and getattr(traffic_result, "target_edge", "")
        else st.session_state.traffic_target_edge
    )

    for edge in edges:
        color = "#2563eb"
        weight = 4

        if edge.edge_id == st.session_state.start_edge:
            color = "#16a34a"
            weight = 8
        elif edge.edge_id == st.session_state.end_edge:
            color = "#dc2626"
            weight = 8
        elif edge.edge_id == traffic_target_edge:
            color = "#f97316"
            weight = 7

        coords = [to_map_coords(point[0], point[1]) for point in edge.shape]
        folium.PolyLine(
            coords,
            color=color,
            weight=weight,
            tooltip=edge.edge_id,
        ).add_to(m)

    # marker persistente
    if st.session_state.get("last_click"):
        x, y = st.session_state.last_click
        folium.CircleMarker([y, x], radius=5, color="red", fill=True).add_to(m)

    map_data = st_folium(
        m,
        use_container_width=True,
        height=EDGE_SELECTION_MAP_HEIGHT,
        key="network_map",
    )

    # click handling
    if map_data and map_data.get("last_clicked"):
        y = map_data["last_clicked"]["lat"]
        x = map_data["last_clicked"]["lng"]

        st.session_state.last_click = (x, y)

    if st.session_state.get("last_click"):
        x, y = st.session_state.last_click
        candidate, distance = nearest_edge(edges, x, y)
        direction_options = edge_direction_options(edges, candidate.edge_id)
        if st.session_state.ego_clicked_direction not in direction_options:
            st.session_state.ego_clicked_direction = candidate.edge_id

        st.write(f"Clicked SUMO coords: {x:.2f}, {y:.2f}")
        st.write(
            f"Edge closest to point clicked: `{candidate.edge_id}` "
            f"({distance:.1f} m)"
        )

        clicked_edge = st.selectbox(
            "Edge Direction",
            options=direction_options,
            format_func=lambda value: labels[value],
            key="ego_clicked_direction",
        )

        col1, col2 = st.columns(2)

        with col1:
            if st.button("Set START", key="set_start_edge"):
                st.session_state.start_edge = clicked_edge
                st.rerun()

        with col2:
            if st.button("Set END", key="set_end_edge"):
                st.session_state.end_edge = clicked_edge
                st.rerun()
    else:
        st.info("Click on the map to select a SUMO coordinate.")

    st.write("Start:", labels.get(st.session_state.start_edge, st.session_state.start_edge))
    st.write("End:", labels.get(st.session_state.end_edge, st.session_state.end_edge))

    direction_cols = st.columns(2)
    start_opposite = (
        opposite_edge_id(st.session_state.start_edge, edge_id_set)
        if st.session_state.start_edge else None
    )
    end_opposite = (
        opposite_edge_id(st.session_state.end_edge, edge_id_set)
        if st.session_state.end_edge else None
    )

    with direction_cols[0]:
        if st.button("Invert START", disabled=not start_opposite):
            st.session_state.start_edge = start_opposite
            st.rerun()
    with direction_cols[1]:
        if st.button("Invert END", disabled=not end_opposite):
            st.session_state.end_edge = end_opposite
            st.rerun()

    # =====================================================
    # SPAWN VEHICLE
    # =====================================================

    st.subheader("🚗 Spawn Ego Vehicle")

    ego_config = render_ego_vehicle_config()
    if ego_config is None:
        return

    max_battery_capacity = float(ego_config["battery_capacity"])
    if st.session_state.ego_initial_battery > max_battery_capacity:
        st.session_state.ego_initial_battery = max_battery_capacity
    if st.session_state.ego_battery_failure_threshold > max_battery_capacity:
        st.session_state.ego_battery_failure_threshold = max_battery_capacity

    battery_cols = st.columns(2)
    with battery_cols[0]:
        battery_init = st.slider(
            "Initial battery [Wh]",
            min_value=0.0,
            max_value=max_battery_capacity,
            step=max(1.0, max_battery_capacity / 100.0),
            key="ego_initial_battery",
        )
    with battery_cols[1]:
        battery_failure_threshold = st.number_input(
            "Critical battery threshold [Wh]",
            min_value=0.0,
            max_value=max_battery_capacity,
            value=float(st.session_state.ego_battery_failure_threshold),
            step=max(1.0, max_battery_capacity / 100.0),
            key="ego_battery_failure_threshold",
            help=(
                "The vehicle is stopped when the remaining battery is "
                "less than or equal to this threshold."
            ),
        )

    battery_threshold_valid = battery_init > battery_failure_threshold
    if not battery_threshold_valid:
        st.warning(
            "The initial battery must be greater than the critical threshold "
            "to avoid an immediate failure."
        )

    congestion_available = bool(
        traffic_result is not None
        and traffic_target_edge
        and traffic_target_edge in edge_id_set
    )
    effective_start_edge = st.session_state.start_edge
    effective_end_edge = st.session_state.end_edge
    via_edge = None
    auto_route_fields = []
    route_hint_start_edge = None
    route_hint_end_edge = None

    if traffic_result is not None:
        route_hint_start_edge, route_hint_end_edge = generated_route_hints(
            traffic_result.route_file,
            traffic_target_edge,
        )

    if congestion_available:
        use_traffic_route = st.checkbox(
            "Use the generated congestion for the ego route",
            key="ego_use_traffic_route",
        )
        if use_traffic_route:
            route_modes = [
                EGO_ROUTE_VIA,
                EGO_ROUTE_FROM_CONGESTION,
                EGO_ROUTE_TO_CONGESTION,
            ]
            if st.session_state.ego_traffic_route_mode not in route_modes:
                st.session_state.ego_traffic_route_mode = route_modes[0]

            route_mode = st.radio(
                "Ego route preference",
                route_modes,
                horizontal=True,
                key="ego_traffic_route_mode",
            )

            (
                effective_start_edge,
                effective_end_edge,
                via_edge,
                auto_route_fields,
            ) = resolve_ego_route_from_congestion(
                edges,
                st.session_state.start_edge,
                st.session_state.end_edge,
                traffic_target_edge,
                route_mode,
                traffic_source_edge=(
                    st.session_state.traffic_source_edge or route_hint_start_edge
                ),
                traffic_destination_edge=(
                    st.session_state.traffic_destination_edge or route_hint_end_edge
                ),
            )

            if route_mode == EGO_ROUTE_VIA:
                st.info(f"The ego route will pass through: {labels[traffic_target_edge]}")
            elif route_mode == EGO_ROUTE_FROM_CONGESTION:
                st.info(f"The ego start edge will be: {labels[traffic_target_edge]}")
            else:
                st.info(f"The ego destination edge will be: {labels[traffic_target_edge]}")

            route_parts = [
                f"START `{effective_start_edge or '-'}`",
                f"END `{effective_end_edge or '-'}`",
            ]
            if via_edge:
                route_parts.insert(1, f"VIA `{via_edge}`")
            st.caption("Effective ego route: " + " -> ".join(route_parts))
            if auto_route_fields:
                st.caption(
                    "Auto-selected edges for: "
                    + ", ".join(auto_route_fields)
                    + "."
                )
    elif traffic_result is not None:
        st.info("No congested edge is available in the generated traffic scenario.")

    if st.button(
        "Spawn Ego Vehicle",
        disabled=not backend_alive or not battery_threshold_valid,
    ):
        if effective_start_edge and effective_end_edge:
            write_ego_vtype_config(
                ego_config["carla_blueprint"],
                ego_config["emission_model"],
                ego_config["battery_capacity"],
                attributes=ego_config["attributes"],
                parameters=ego_config["parameters"],
            )
            vtype_params = {
                "has.battery.device": "true",
                "carla.blueprint": ego_config["carla_blueprint"],
                "device.battery.capacity": str(ego_config["battery_capacity"]),
                "device.battery.maximumBatteryCapacity": str(ego_config["battery_capacity"]),
                "dashboard.battery.failureThreshold": str(battery_failure_threshold),
            }
            vtype_params.update(
                {
                    key: value
                    for key, value in ego_config["parameters"].items()
                    if value is not None and str(value).strip() != ""
                }
            )

            payload = {
                "start": effective_start_edge,
                "end": effective_end_edge,
                "battery": battery_init,
                "battery_failure_threshold": battery_failure_threshold,
                "vtype": ego_config["sumo_vtype"],
                "carla_blueprint": ego_config["carla_blueprint"],
                "vtype_attrs": {
                    **ego_config["attributes"],
                    "emissionClass": ego_config["emission_class"],
                },
                "vtype_params": vtype_params,
            }
            if via_edge and via_edge not in {effective_start_edge, effective_end_edge}:
                payload["via"] = via_edge

            response = requests.post(
                f"{API_URL}/spawn",
                json=payload,
                timeout=2,
            )
            if response.status_code == 200:
                reset_monitoring_trip_state()
                st.session_state.monitoring = False
                st.session_state.ego_active_start_edge = effective_start_edge
                st.session_state.ego_active_end_edge = effective_end_edge
                st.session_state.ego_active_via_edge = via_edge
                st.session_state.ego_active_battery_failure_threshold = (
                    battery_failure_threshold
                )
                st.success("Ego vehicle spawned")
            else:
                st.error(f"Spawn failed: {response.text}")
        else:
            if st.session_state.ego_use_traffic_route and congestion_available:
                st.warning(
                    "I could not resolve an ego route automatically: "
                    "select at least a start edge or an end edge."
                )
            else:
                st.warning("Select start and end edges first")


def render_live_vehicle_editor():
    """Render the UI used to inspect and edit live SUMO vehicles."""
    st.subheader("🔧 Live Vehicle vType")

    if not backend_alive:
        st.warning(
            "Backend not active: start the co-simulation first to inspect and edit live SUMO vehicles."
        )
        return

    try:
        vehicles = get_live_sumo_vehicles()
    except Exception as exc:
        st.error(f"Could not load SUMO vehicles: {exc}")
        return

    vehicles = [vehicle for vehicle in vehicles if is_carla_spawned_vehicle(vehicle)]
    if not vehicles:
        st.info("No CARLA-spawned SUMO vehicle is currently active in the simulation.")
        return

    vehicle_ids = [vehicle["id"] for vehicle in vehicles]
    vehicle_labels = {vehicle["id"]: vehicle_display_label(vehicle) for vehicle in vehicles}

    if st.session_state.live_vehicle_selected_id not in vehicle_ids:
        st.session_state.live_vehicle_selected_id = vehicle_ids[0]
        st.session_state.live_vehicle_config_loaded_for = None

    selector_cols = st.columns([3, 1])
    with selector_cols[0]:
        selected_vehicle_id = st.selectbox(
            "SUMO vehicle",
            options=vehicle_ids,
            format_func=lambda veh_id: vehicle_labels[veh_id],
            key="live_vehicle_selected_id",
        )
    with selector_cols[1]:
        st.write("")
        if st.button("Refresh vehicle list"):
            st.rerun()

    st.caption(
        "Only CARLA-spawned SUMO vehicles are listed here. "
        "The current route is left untouched."
    )

    try:
        live_vehicle_config = render_live_vehicle_vtype_config(selected_vehicle_id)
    except Exception as exc:
        st.error(f"Could not load vehicle vType data: {exc}")
        st.session_state.live_vehicle_config_loaded_for = None
        return

    if live_vehicle_config is None:
        return

    if not live_vehicle_config.get("has_battery_device", False):
        st.info(
            "This vehicle has no SUMO battery device. Live update will still change the "
            "vehicle type data, but battery-specific fields stay inactive."
        )

    if st.button("Apply live vType update"):
        vtype_params = {
            "carla.blueprint": live_vehicle_config["carla_blueprint"],
        }
        if live_vehicle_config.get("has_battery_device"):
            vtype_params.update(
                {
                    "has.battery.device": "true",
                    "device.battery.capacity": str(live_vehicle_config["battery_capacity"]),
                    "device.battery.maximumBatteryCapacity": str(
                        live_vehicle_config["battery_capacity"]
                    ),
                }
            )
        battery_param_keys = {
            "has.battery.device",
            "device.battery.capacity",
            "device.battery.maximumBatteryCapacity",
            "device.battery.chargeLevel",
            "device.battery.actualBatteryCapacity",
            "dashboard.battery.failureThreshold",
        }
        vtype_params.update(
            {
                key: value
                for key, value in live_vehicle_config["parameters"].items()
                if value is not None and str(value).strip() != ""
                and (
                    live_vehicle_config.get("has_battery_device")
                    or key not in battery_param_keys
                )
            }
        )

        payload = {
            "emission_model": live_vehicle_config["emission_model"],
            "battery_capacity": live_vehicle_config["battery_capacity"],
            "vtype_attrs": {
                **live_vehicle_config["attributes"],
                "emissionClass": live_vehicle_config["emission_class"],
            },
            "vtype_params": vtype_params,
        }
        encoded_vehicle_id = quote(str(selected_vehicle_id), safe="")
        try:
            response = requests.post(
                f"{API_URL}/vehicle/{encoded_vehicle_id}/vtype",
                json=payload,
                timeout=3,
            )
        except Exception as exc:
            st.error(f"Live update failed: {exc}")
            return

        if response.status_code == 200:
            updated_vehicle = response.json().get("vehicle", {})
            if updated_vehicle:
                apply_vtype_config_to_state("live_vehicle", updated_vehicle)
                st.session_state.live_vehicle_config_loaded_for = selected_vehicle_id
                st.session_state.live_vehicle_config_version = active_carla_version()
            st.success(f"Live vType updated for `{selected_vehicle_id}`.")
        else:
            st.error(f"Live update failed: {response.text}")


simulation_step = simulation_step_label()
vehicle_step = vehicle_setup_step_label()
autoware_workflow = active_carla_version() == "0.9.13"
if autoware_workflow:
    section_options = [
        STEP_CARLA_LABEL,
        STEP_ROUTES_LABEL,
        vehicle_step,
    ]
    if SHOW_SIMULATION_STEP_TAB:
        section_options.insert(2, simulation_step)
    section_options.append(STEP_PLOT_OUTPUT_LABEL)
    if SHOW_MONITORING_TAB:
        section_options.append(STEP_MONITORING_LABEL)
    workflow_caption = (
        "Follow the workflow from left to right: start CARLA, generate SUMO routes, "
        "configure the run, then launch Autoware and start the co-simulation."
    )
else:
    section_options = [
        STEP_CARLA_LABEL,
        STEP_ROUTES_LABEL,
        vehicle_step,
    ]
    if SHOW_SIMULATION_STEP_TAB:
        section_options.insert(3, simulation_step)
    section_options.append(STEP_PLOT_OUTPUT_LABEL)
    if SHOW_MONITORING_TAB:
        section_options.append(STEP_MONITORING_LABEL)
    workflow_caption = (
        "Follow the workflow from left to right: start CARLA, generate SUMO routes, "
        "prepare the vehicle, then run the co-simulation."
    )
current_section = st.session_state.get("dashboard_section")
if current_section not in section_options:
    mapped_section = None
    if current_section:
        current_title = current_section.split(". ", 1)[-1]
        if "Simulation" in current_title and SHOW_SIMULATION_STEP_TAB:
            mapped_section = simulation_step
        elif "Simulation" in current_title:
            mapped_section = STEP_ROUTES_LABEL
        elif "Plot Output" in current_title:
            mapped_section = STEP_PLOT_OUTPUT_LABEL
        elif "Autoware" in current_title or "SUMO Ego Vehicle" in current_title:
            mapped_section = vehicle_step
    st.session_state.dashboard_section = mapped_section or STEP_CARLA_LABEL
elif current_section is None:
    st.session_state.dashboard_section = STEP_CARLA_LABEL

section = st.segmented_control(
    "Execution steps",
    section_options,
    default=STEP_CARLA_LABEL,
    key="dashboard_section",
    label_visibility="collapsed",
)

st.caption(
    workflow_caption
)

if section == STEP_CARLA_LABEL:
        preserve_scroll_position("dashboard_carla_step_scroll_position")
        render_carla_step()
elif section == STEP_ROUTES_LABEL:
        preserve_scroll_position("dashboard_routes_step_scroll_position")
        render_traffic_scenario(show_runner=True)
elif section == simulation_step:
        preserve_scroll_position("dashboard_simulation_step_scroll_position")
        render_simulation_runner()
elif section == vehicle_step:
        preserve_scroll_position("dashboard_vehicle_step_scroll_position")
        if active_carla_version() == "0.9.13":
            render_autoware_ego_vtype_editor()
        else:
            render_setup()
elif section == STEP_PLOT_OUTPUT_LABEL:
        preserve_scroll_position("dashboard_plot_output_step_scroll_position")
        render_plot_output()
else:
        render_monitoring()
