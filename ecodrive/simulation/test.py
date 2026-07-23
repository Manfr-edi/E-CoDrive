from pathlib import Path

from ecodrive.simulation.automated_simulation import simulate


PROJECT_ROOT = Path(__file__).resolve().parents[2]
BOLOGNA_ROUTES = (
    PROJECT_ROOT
    / "carla"
    / "CARLA_0.9.13"
    / "Co-Simulation"
    / "Sumo"
    / "examples"
    / "rou"
    / "bologna.rou.xml"
)


if __name__ == "__main__":
    result = simulate(
        town="bologna",
        headless=False,
        # Gli NPC sono letti dal file, senza generarne di nuovi.
        traffic_generation_mode="route_file",
        traffic_route_file=BOLOGNA_ROUTES,
        simulation_end=3600,
        ego_starting_delay=5.0,
        ego_source_edge="-1204509157#2",
        ego_destination_edge="150018085#0",
        ego_energy_model="Energy",
        ego_max_battery_capacity=75000,
        ego_current_battery_charge=650,
        ego_critical_battery_threshold=500,
        ego_model_parameters={
            "maximumPower": 350000,
            "constantPowerIntake": 360,
            "airDragCoefficient": 0.23,
            "frontSurfaceArea": 2.2,
            "mass": 1919,
            "rotatingMass": 80,
            "propulsionEfficiency": 0.80,
            "radialDragCoefficient": 0.1,
            "recuperationEfficiency": 0.80,
            "rollDragCoefficient": 0.01,
            "stoppingThreshold": 0.1,
        },
    )
    print(f"Co-simulazione terminata: {result.completion_reason}")
