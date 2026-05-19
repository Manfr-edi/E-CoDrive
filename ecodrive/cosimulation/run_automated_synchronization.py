#!/usr/bin/env python
"""Gated SUMO-CARLA runner used only by the programmatic simulate() API."""

import argparse
import logging
from pathlib import Path
import sys
import threading
import time

PROJECT_ROOT = Path(__file__).resolve().parents[2]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ecodrive.cosimulation.run_dashboard_synchronization import (  # pylint: disable=wrong-import-position
    configure_carla_version,
    release_carla_synchronous_mode,
)
from ecodrive.scenario.sumo_route_tools import (  # pylint: disable=wrong-import-position
    DEFAULT_CARLA_VERSION,
    available_carla_versions,
    ensure_carla_python_api_ready,
)


def make_long_wait_carla_simulation(base_class, carla_module, client_timeout):
    """Return a CarlaSimulation variant with a longer CARLA RPC timeout."""

    class LongWaitCarlaSimulation(base_class):
        """CARLA simulation client with configurable client timeout."""

        def __init__(self, host, port, step_length):
            self.client = carla_module.Client(host, port)
            self.client.set_timeout(float(client_timeout))

            self.world = self.client.get_world()
            self.blueprint_library = self.world.get_blueprint_library()
            self.step_length = step_length

            self._active_actors = set()
            self.spawned_actors = set()
            self.destroyed_actors = set()

            self._tls = {}
            tmp_map = self.world.get_map()
            for landmark in tmp_map.get_all_landmarks_of_type("1000001"):
                if landmark.id == "":
                    continue
                traffic_light = self.world.get_traffic_light(landmark)
                if traffic_light is not None:
                    self._tls[landmark.id] = traffic_light
                else:
                    logging.warning("Landmark %s is not linked to any traffic light", landmark.id)

    return LongWaitCarlaSimulation


def synchronization_loop(args):
    """Initialize SUMO/CARLA, wait on a start gate, then run the bridge."""
    import carla  # pylint: disable=import-error,import-outside-toplevel
    from run_synchronization import (  # pylint: disable=import-outside-toplevel
        BridgeHelper,
        CarlaSimulation,
        SimulationSynchronization,
    )
    from ecodrive.cosimulation.dashboard_backend import run_api  # pylint: disable=import-outside-toplevel
    from ecodrive.cosimulation.dashboard_sumo import (  # pylint: disable=import-outside-toplevel
        DashboardSumoSimulation,
        patch_bridge_helper,
    )

    patch_bridge_helper(BridgeHelper)
    automated_carla_simulation = make_long_wait_carla_simulation(
        CarlaSimulation,
        carla,
        args.carla_client_timeout,
    )

    sumo_simulation = None
    carla_simulation = None
    synchronization = None

    try:
        logging.info("Initializing SUMO/TraCI from cfg: %s", args.sumo_cfg_file)
        sumo_simulation = DashboardSumoSimulation(
            args.sumo_cfg_file,
            args.step_length,
            args.sumo_host,
            args.sumo_port,
            args.sumo_gui,
            args.client_order,
        )
        logging.info(
            "SUMO/TraCI initialized and waiting for simulation ticks "
            "(sumo_gui=%s, step_length=%s).",
            args.sumo_gui,
            args.step_length,
        )

        logging.info(
            "Connecting CARLA client at %s:%s with %.1fs timeout.",
            args.carla_host,
            args.carla_port,
            args.carla_client_timeout,
        )
        carla_simulation = automated_carla_simulation(
            args.carla_host,
            args.carla_port,
            args.step_length,
        )
        logging.info("CARLA client connected.")

        if args.wait_start_file:
            release_carla_synchronous_mode(carla_simulation)
            wait_path = Path(args.wait_start_file).expanduser().resolve()
            if args.wait_ready_file:
                ready_path = Path(args.wait_ready_file).expanduser().resolve()
                ready_path.parent.mkdir(parents=True, exist_ok=True)
                ready_path.write_text("ready\n", encoding="utf-8")
                logging.info(
                    "Automated ready signal written before SUMO/CARLA time advances: %s",
                    ready_path,
                )

            logging.info(
                "Waiting for automated start signal before enabling CARLA synchronous mode: %s",
                wait_path,
            )
            while not wait_path.exists():
                time.sleep(0.2)
            logging.info("Automated start signal received.")
            try:
                wait_path.unlink()
            except OSError:
                pass

        logging.info("Creating SUMO/CARLA SimulationSynchronization.")
        synchronization = SimulationSynchronization(
            sumo_simulation,
            carla_simulation,
            args.tls_manager,
            args.sync_vehicle_color,
            args.sync_vehicle_lights,
        )
        logging.info("SUMO/CARLA synchronization ready; entering tick loop.")

        if not args.no_dashboard_api:
            threading.Thread(
                target=run_api,
                args=(synchronization, args.dashboard_api_host, args.dashboard_api_port),
                daemon=True,
            ).start()

        traci_lock = getattr(sumo_simulation, "traci_lock", None)
        while True:
            start = time.time()
            if traci_lock is None:
                synchronization.tick()
            else:
                with traci_lock:
                    synchronization.tick()

            elapsed = time.time() - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info("Cancelled by user.")

    finally:
        logging.info("Cleaning synchronization")
        traci_lock = getattr(sumo_simulation, "traci_lock", None)
        try:
            if synchronization is not None:
                if traci_lock is None:
                    synchronization.close()
                else:
                    with traci_lock:
                        synchronization.close()
            else:
                if carla_simulation is not None:
                    carla_simulation.close()
                if sumo_simulation is not None:
                    sumo_simulation.close()
        except Exception as error:  # pragma: no cover - depends on live CARLA/SUMO runtime
            logging.warning("Synchronization cleanup failed: %s", error)


def build_argparser():
    """Build command-line parser for the automated synchronization runner."""
    version_choices = available_carla_versions() or [DEFAULT_CARLA_VERSION]
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--carla-version",
        default=DEFAULT_CARLA_VERSION if DEFAULT_CARLA_VERSION in version_choices else version_choices[0],
        choices=version_choices,
        help="CARLA installation to use (default: %(default)s)",
    )
    argparser.add_argument("sumo_cfg_file", type=str, help="sumo configuration file")
    argparser.add_argument("--carla-host", metavar="H", default="127.0.0.1")
    argparser.add_argument("--carla-port", metavar="P", default=2000, type=int)
    argparser.add_argument(
        "--carla-client-timeout",
        default=60.0,
        type=float,
        help="CARLA client RPC timeout in seconds (default: 60)",
    )
    argparser.add_argument("--sumo-host", metavar="H", default=None)
    argparser.add_argument("--sumo-port", metavar="P", default=None, type=int)
    argparser.add_argument("--sumo-gui", action="store_true", help="run the gui version of sumo")
    argparser.add_argument("--step-length", default=0.05, type=float)
    argparser.add_argument("--client-order", metavar="TRACI_CLIENT_ORDER", default=1, type=int)
    argparser.add_argument("--sync-vehicle-lights", action="store_true")
    argparser.add_argument("--sync-vehicle-color", action="store_true")
    argparser.add_argument("--sync-vehicle-all", action="store_true")
    argparser.add_argument(
        "--tls-manager",
        type=str,
        choices=["none", "sumo", "carla"],
        default="none",
    )
    argparser.add_argument("--dashboard-api-host", default="127.0.0.1")
    argparser.add_argument("--dashboard-api-port", default=5000, type=int)
    argparser.add_argument("--no-dashboard-api", action="store_true")
    argparser.add_argument("--wait-start-file", type=str, default=None)
    argparser.add_argument("--wait-ready-file", type=str, default=None)
    argparser.add_argument("--debug", action="store_true")
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
