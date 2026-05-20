from ecodrive.simulation.automated_simulation import simulate


# Example of run with spatial random spawning traffic that congestion a specific road
result = simulate(
    town="Town04",
    traffic_generation_mode="random" ,
    traffic_congestion_edge="-41.0.00",
    traffic_source_edge="-40.0.00",
    traffic_destination_edge="-22.0.00",
    traffic_vehicle_count=20,
    traffic_spawn_time=0,
    traffic_stop_spawn_time=20,
    traffic_vehicle_type="random",
    ego_starting_delay= 10.0,
    ego_source_edge="-38.0.00",
    ego_destination_edge="-41.0.00",
    ego_energy_model="Energy",
    ego_max_battery_capacity=75000,
    ego_current_battery_charge=600,
    ego_critical_battery_threshold=500,
    ego_model_parameters={
        "maximumPower": 350000,
        "constantPowerIntake": 360,
        "airDragCoefficient": 0.23,
        "frontSurfaceArea": 2.2,
        "mass": 1919,
        "rotatingMass": 80,
        "propulsionEfficiency": .98,
        "radialDragCoefficient": 0.1,
        "recuperationEfficiency": .96,
        "rollDragCoefficient": 0.01,
        "stoppingThreshold": 0.1,
    },
)

print(result)

# traffic_generation_mode="congestion"          # default, source+destination+congestion
# traffic_generation_mode="random"              # only use congestion edge and vehicle count
# traffic_generation_mode="random_traffic"