#!/usr/bin/env python
"""SUMO-CARLA runner used by the Streamlit dashboard.

This keeps CARLA's default ``run_synchronization.py`` free from the Flask API
and ego-vehicle helpers required by the dashboard.
"""

import argparse
import logging
from pathlib import Path
import os
import sys
import threading
import time

PROJECT_ROOT = Path(__file__).resolve().parents[2]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from aev_drivelab.scenario.sumo_route_tools import (  # pylint: disable=wrong-import-position
    DEFAULT_CARLA_VERSION,
    available_carla_versions,
    carla_paths,
    ensure_carla_python_api_ready,
    resolve_carla_python_executable,
    selected_carla_python_api_archive,
    selected_carla_runtime_library_dirs,
    set_active_carla_version,
)


def configure_carla_version(version):
    """Configure Python paths and runtime libraries for the selected CARLA version."""
    selected_version = set_active_carla_version(version)
    paths = carla_paths(selected_version)
    sumo_dir = paths["sumo_dir"]
    carla_dist_dir = paths["carla_dist_dir"]

    for path in (PROJECT_ROOT, sumo_dir):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))

    api_archive = selected_carla_python_api_archive()
    if api_archive is not None:
        if str(api_archive) in sys.path:
            sys.path.remove(str(api_archive))
        sys.path.insert(0, str(api_archive))

    # Keep subprocesses aligned with the selected CARLA Python API instead of the
    # globally installed package in the active environment.
    existing_pythonpath = [
        item for item in os.environ.get("PYTHONPATH", "").split(os.pathsep) if item
    ]
    filtered_pythonpath = [
        item
        for item in existing_pythonpath
        if "/PythonAPI/carla/dist/" not in item and "site-packages/carla" not in item
    ]
    if api_archive is not None:
        filtered_pythonpath.insert(0, str(api_archive))
    if filtered_pythonpath:
        os.environ["PYTHONPATH"] = os.pathsep.join(filtered_pythonpath)

    library_dirs = (
        selected_carla_runtime_library_dirs(resolve_carla_python_executable(selected_version))
        if api_archive is not None
        else []
    )
    if library_dirs:
        existing_ld_library_path = [
            item for item in os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep) if item
        ]
        for library_dir in reversed(library_dirs):
            if str(library_dir) in existing_ld_library_path:
                existing_ld_library_path.remove(str(library_dir))
            existing_ld_library_path.insert(0, str(library_dir))
        os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(existing_ld_library_path)

    return selected_version


def synchronization_loop(args):
    """Run the SUMO-CARLA synchronization loop and optional dashboard API."""
    from run_synchronization import (  # pylint: disable=import-outside-toplevel
        BridgeHelper,
        CarlaSimulation,
        SimulationSynchronization,
    )
    from aev_drivelab.cosimulation.dashboard_backend import run_api  # pylint: disable=import-outside-toplevel
    from aev_drivelab.cosimulation.dashboard_sumo import (  # pylint: disable=import-outside-toplevel
        DashboardSumoSimulation,
        patch_bridge_helper,
    )

    patch_bridge_helper(BridgeHelper)

    sumo_simulation = DashboardSumoSimulation(
        args.sumo_cfg_file,
        args.step_length,
        args.sumo_host,
        args.sumo_port,
        args.sumo_gui,
        args.client_order,
    )
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

    synchronization = SimulationSynchronization(
        sumo_simulation,
        carla_simulation,
        args.tls_manager,
        args.sync_vehicle_color,
        args.sync_vehicle_lights,
    )

    if not args.no_dashboard_api:
        threading.Thread(
            target=run_api,
            args=(synchronization, args.dashboard_api_host, args.dashboard_api_port),
            daemon=True,
        ).start()

    if args.wait_start_file:
        wait_path = Path(args.wait_start_file).expanduser().resolve()
        logging.info("Waiting for dashboard start signal: %s", wait_path)
        while not wait_path.exists():
            time.sleep(0.2)
        logging.info("Dashboard start signal received.")
        try:
            wait_path.unlink()
        except OSError:
            pass

    try:
        while True:
            start = time.time()
            synchronization.tick()

            elapsed = time.time() - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info("Cancelled by user.")

    finally:
        logging.info("Cleaning synchronization")
        synchronization.close()


def build_argparser():
    """Build the command-line parser for the dashboard synchronization runner."""
    version_choices = available_carla_versions() or [DEFAULT_CARLA_VERSION]
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--carla-version",
        default=DEFAULT_CARLA_VERSION if DEFAULT_CARLA_VERSION in version_choices else version_choices[0],
        choices=version_choices,
        help="CARLA installation to use (default: %(default)s)",
    )
    argparser.add_argument("sumo_cfg_file", type=str, help="sumo configuration file")
    argparser.add_argument(
        "--carla-host",
        metavar="H",
        default="127.0.0.1",
        help="IP of the carla host server (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "--carla-port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to (default: 2000)",
    )
    argparser.add_argument(
        "--sumo-host",
        metavar="H",
        default=None,
        help="IP of the sumo host server (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "--sumo-port",
        metavar="P",
        default=None,
        type=int,
        help="TCP port to listen to (default: 8813)",
    )
    argparser.add_argument("--sumo-gui", action="store_true", help="run the gui version of sumo")
    argparser.add_argument(
        "--step-length",
        default=0.05,
        type=float,
        help="set fixed delta seconds (default: 0.05s)",
    )
    argparser.add_argument(
        "--client-order",
        metavar="TRACI_CLIENT_ORDER",
        default=1,
        type=int,
        help="client order number for the co-simulation TraCI connection (default: 1)",
    )
    argparser.add_argument(
        "--sync-vehicle-lights",
        action="store_true",
        help="synchronize vehicle lights state (default: False)",
    )
    argparser.add_argument(
        "--sync-vehicle-color",
        action="store_true",
        help="synchronize vehicle color (default: False)",
    )
    argparser.add_argument(
        "--sync-vehicle-all",
        action="store_true",
        help="synchronize all vehicle properties (default: False)",
    )
    argparser.add_argument(
        "--tls-manager",
        type=str,
        choices=["none", "sumo", "carla"],
        help="select traffic light manager (default: none)",
        default="none",
    )
    argparser.add_argument(
        "--dashboard-api-host",
        default="127.0.0.1",
        help="dashboard API host (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "--dashboard-api-port",
        default=5000,
        type=int,
        help="dashboard API port (default: 5000)",
    )
    argparser.add_argument(
        "--no-dashboard-api",
        action="store_true",
        help="disable the dashboard Flask API",
    )
    argparser.add_argument(
        "--wait-start-file",
        type=str,
        default=None,
        help="wait for this file to appear before advancing the simulation loop",
    )
    argparser.add_argument("--debug", action="store_true", help="enable debug messages")
    return argparser


if __name__ == "__main__":
    arguments = build_argparser().parse_args()
    arguments.carla_version = configure_carla_version(arguments.carla_version)
    ensure_carla_python_api_ready()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.DEBUG)
    else:
        logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    synchronization_loop(arguments)
