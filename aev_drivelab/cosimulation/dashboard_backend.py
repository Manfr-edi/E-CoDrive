#!/usr/bin/env python
"""Flask API used by the Streamlit dashboard during SUMO-CARLA co-simulation."""

import traceback
from flask import Flask, jsonify, request


def create_app(sync):
    """Create the Flask API used by the dashboard during co-simulation."""
    app = Flask(__name__)

    def require_sync():
        """Ensure the synchronization object is available before serving a request."""
        if sync is None:
            return jsonify({"error": "Synchronization is not ready"}), 503
        return None

    @app.route("/spawn", methods=["POST"])
    def spawn():
        """Spawn the dashboard ego vehicle with the requested route and battery settings."""
        error = require_sync()
        if error:
            return error

        data = request.json or {}
        start = data["start"]
        end = data["end"]
        via = data.get("via") or data.get("via_edge")
        vtype = data.get("vtype") or "vehicle.tesla.model3"
        carla_blueprint = data.get("carla_blueprint")
        vtype_attrs = data.get("vtype_attrs") or {}
        vtype_params = dict(data.get("vtype_params") or {})
        battery = data.get("battery", None)
        battery_failure_threshold = data.get("battery_failure_threshold", 0)

        if battery is not None:
            vtype_params["has.battery.device"] = "true"
            vtype_params["device.battery.chargeLevel"] = str(battery)
            vtype_params["device.battery.actualBatteryCapacity"] = str(battery)
        vtype_params["dashboard.battery.failureThreshold"] = str(battery_failure_threshold)

        try:
            spawned = sync.sumo.spawn_ego_vehicle(
                start,
                end,
                via_edge=via,
                vtype=vtype,
                carla_blueprint=carla_blueprint,
                vtype_attrs=vtype_attrs,
                vtype_params=vtype_params,
                battery_charge_level=battery,
                battery_failure_threshold=battery_failure_threshold,
            )
        except RuntimeError as exc:
            return {"status": "error", "message": str(exc)}, 500
        if not spawned:
            return {"status": "error", "message": "Route non valida"}, 400

        state_data = sync.sumo.get_vehicle_state("ego_vehicle")
        if battery is not None and not (state_data or {}).get("has_battery_device"):
            return {
                "status": "error",
                "message": "Ego vehicle spawned without a SUMO battery device.",
            }, 500

        return {"status": "spawned", "vehicle": state_data or {}}

    @app.route("/state", methods=["GET"])
    def state():
        """Return the live state of the requested vehicle."""
        error = require_sync()
        if error:
            return error

        requested_vehicle_id = request.args.get("veh_id")
        vehicle_id = requested_vehicle_id or "ego_vehicle"
        state_data = sync.sumo.get_vehicle_state(vehicle_id)

        if state_data is None and requested_vehicle_id:
            return jsonify({"error": f"Vehicle {vehicle_id} not found"}), 404

        return jsonify(state_data)

    @app.route("/vehicles", methods=["GET"])
    def vehicles():
        """Return the live vehicle list exposed by the synchronization backend."""
        error = require_sync()
        if error:
            return error

        return jsonify({"vehicles": sync.sumo.list_vehicles()})

    @app.route("/vehicle/<path:veh_id>", methods=["GET"])
    def vehicle(veh_id):
        """Return the current vType configuration for a vehicle."""
        error = require_sync()
        if error:
            return error

        config = sync.sumo.get_vehicle_vtype_config(veh_id)
        if config is None:
            return {"error": f"Vehicle {veh_id} not found"}, 404

        return jsonify(config)

    @app.route("/vehicle/<path:veh_id>/vtype", methods=["POST"])
    def update_vehicle_vtype(veh_id):
        """Apply a live vType update to the selected vehicle."""
        error = require_sync()
        if error:
            return error

        data = request.json or {}
        try:
            config = sync.sumo.update_vehicle_vtype(
                veh_id,
                emission_model=data.get("emission_model"),
                battery_capacity=data.get("battery_capacity"),
                attributes=data.get("vtype_attrs") or {},
                parameters=data.get("vtype_params") or {},
            )
        except RuntimeError as exc:
            return {"status": "error", "message": str(exc)}, 400
        except Exception as exc:  # pragma: no cover - depends on live TraCI/runtime.
            app.logger.error("Unexpected live vType update error for %s\n%s", veh_id, traceback.format_exc())
            return {
                "status": "error",
                "message": f"Unexpected live vType update error for `{veh_id}`: {exc}",
            }, 500

        return jsonify({"status": "updated", "vehicle": config})

    @app.route("/nearest_edge", methods=["POST"])
    def nearest_edge():
        """Resolve the nearest SUMO edge to a clicked map position."""
        error = require_sync()
        if error:
            return error

        data = request.json or {}
        x = data["x"]
        y = data["y"]

        radius = 20
        edges = []
        while not edges and radius < 200:
            edges = sync.sumo.net.getNeighboringEdges(x, y, radius)
            radius *= 2

        if not edges:
            return {"error": "No edge found"}, 404

        closest_edge = min(edges, key=lambda item: item[1])[0]
        return {"edge": closest_edge.getID()}

    @app.route("/edges")
    def edges():
        """Return a lightweight list of edge identifiers for the active network."""
        error = require_sync()
        if error:
            return error

        edge_ids = [edge.getID() for edge in sync.sumo.net.getEdges()]
        return {"edges": edge_ids[:1000]}

    @app.route("/network")
    def network():
        """Return the active SUMO network geometry for map rendering."""
        error = require_sync()
        if error:
            return error

        edges_data = []
        for edge in sync.sumo.net.getEdges():
            edges_data.append(
                {
                    "id": edge.getID(),
                    "shape": edge.getShape(),
                }
            )

        return {"edges": edges_data}

    return app


def run_api(sync, host="127.0.0.1", port=5000):
    """Run the dashboard Flask API server."""
    create_app(sync).run(host=host, port=port)
