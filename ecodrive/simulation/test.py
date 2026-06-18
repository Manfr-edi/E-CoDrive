from random import randrange

from ecodrive.simulation.automated_simulation import simulate


# Example of run with spatial random spawning traffic that congestion a specific road
for iteration in range(20):
    print("Iteration no:" + str(iteration))
    vehicle_number = randrange(1, 10, 1)
    print("Generated :" + str(vehicle_number))
    result = simulate(
        town="Town04", #FIXED (one per case-study)
        headless=False,
        traffic_generation_mode="random" ,
        traffic_congestion_edge="-41.0.00", # Select on the prebuilt list of roads
        traffic_source_edge="-40.0.00",
        traffic_destination_edge="-22.0.00",
        traffic_vehicle_count=vehicle_number, # Set thresholds
        traffic_spawn_time=0,
        traffic_stop_spawn_time=20,
        traffic_vehicle_type="vehicle.tesla.model3", # Fix one vehicle type
        ego_starting_delay= 5.0,
        ego_source_edge="-17.0.00",
        ego_destination_edge="-26.0.00",
        ego_energy_model="Energy", # Fixing for testing different SUT
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
            "propulsionEfficiency": .80,
            "radialDragCoefficient": 0.1,
            "recuperationEfficiency": .80,
            "rollDragCoefficient": 0.01,
            "stoppingThreshold": 0.1,
        },
    )
    print("Ended CoSim")
    # print(result)

# traffic_generation_mode="congestion"          # default, source+destination+congestion
# traffic_generation_mode="random"              # only use congestion edge and vehicle count
# traffic_generation_mode="random_traffic"
