from ecodrive.simulation.automated_simulation import simulate

result = simulate(
    town="Town04",
    traffic_congestion_edge="-41.0.00",
    traffic_source_edge="-40.0.00",
    traffic_destination_edge="-22.0.00",
    traffic_vehicle_count=50,
    traffic_spawn_time=0,
    traffic_stop_spawn_time=120,
    traffic_vehicle_type="random",
    ego_source_edge="-38.0.00",
    ego_destination_edge="-41.0.00",
    ego_energy_model="Energy",
    ego_max_battery_capacity=75000,
    ego_current_battery_charge=1000,
    ego_critical_battery_threshold=500,
    ego_model_parameters={
        "maximumPower": 350000,
        "constantPowerIntake": 360,
    },
)

print(result)