# backend_bridge.py

from sumo_integration.sumo_simulation import SumoSimulation

class CoSimBridge:
    """Small wrapper around the SUMO simulation used by legacy code paths."""
    def __init__(self, sumo_simulation: SumoSimulation):
        """Initialize the CoSimBridge instance."""
        self.sumo = sumo_simulation

    # -------------------------
    # EGO VEHICLE
    # -------------------------
    def spawn_ego(self, start_edge, end_edge, veh_id="ego_vehicle", vtype="test_ev"):
        """Spawn a legacy ego vehicle through the wrapped SUMO simulation."""
        import traci

        route = traci.simulation.findRoute(start_edge, end_edge).edges

        traci.route.add("ego_route", route)

        traci.vehicle.add(
            vehID=veh_id,
            routeID="ego_route",
            typeID=vtype
        )

    # -------------------------
    # STATE
    # -------------------------
    def get_ego_state(self, veh_id="ego_vehicle"):
        """Return the live state of the legacy ego vehicle."""
        import traci

        if veh_id not in traci.vehicle.getIDList():
            return None

        return {
            "speed": traci.vehicle.getSpeed(veh_id),
            "edge": traci.vehicle.getRoadID(veh_id),
            "battery": traci.vehicle.getParameter(
                veh_id,
                "device.battery.chargeLevel"
            )
        }
