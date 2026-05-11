import traci
from ecodrive.simulation.config import EGO_ID, EGO_TYPE


def spawn_ego(start_edge, end_edge):
    """Spawn the legacy SUMO ego vehicle on a route between two edges."""
    route_id = f"{EGO_ID}_route"

    # Validazione route
    route = traci.simulation.findRoute(start_edge, end_edge)

    if not route.edges:
        raise ValueError("Route non valida")

    traci.route.add(route_id, route.edges)

    traci.vehicle.add(
        vehID=EGO_ID,
        routeID=route_id,
        typeID=EGO_TYPE
    )


def get_battery():
    """Return the current battery charge level of the legacy ego vehicle."""
    for key in ("device.battery.actualBatteryCapacity", "device.battery.chargeLevel"):
        try:
            val = traci.vehicle.getParameter(EGO_ID, key)
            return float(val)
        except Exception:
            continue
    return None


def stop_if_needed():
    """Stop if needed."""
    battery = get_battery()

    if battery is None:
        return

    if battery <= 0:
        traci.vehicle.setSpeed(EGO_ID, 0)
        print("⚠️ Batteria esaurita")


def get_vehicle_state():
    """Return the current legacy ego vehicle state from TraCI."""
    if EGO_ID not in traci.vehicle.getIDList():
        return None

    return {
        "speed": traci.vehicle.getSpeed(EGO_ID),
        "edge": traci.vehicle.getRoadID(EGO_ID),
        "battery": get_battery()
    }
