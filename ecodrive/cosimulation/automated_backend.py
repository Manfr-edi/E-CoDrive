#!/usr/bin/env python
"""Minimal Flask API used by the automated SUMO-CARLA runner."""

from flask import Flask, jsonify, request


def create_app(sync):
    """Create the local API used by the automated orchestration code."""
    app = Flask(__name__)

    def require_sync():
        if sync is None:
            return jsonify({"error": "Synchronization is not ready"}), 503
        return None

    @app.route("/state", methods=["GET"])
    def state():
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
        error = require_sync()
        if error:
            return error

        return jsonify({"vehicles": sync.sumo.list_vehicles()})

    return app


def run_api(sync, host="127.0.0.1", port=5000):
    """Run the automated local API server."""
    create_app(sync).run(host=host, port=port)
