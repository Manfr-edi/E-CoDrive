import traci
import subprocess
import time

from ecodrive.simulation.config import SUMO_BINARY, SUMO_CONFIG
from ecodrive.simulation.ego_controller import stop_if_needed

class SimulationManager:
    """Manage the legacy direct SUMO simulation lifecycle."""
    def __init__(self):
        """Initialize the SimulationManager instance."""
        self.running = False

    def start(self):
        """Start start."""
        if self.running:
            return

        traci.start([
            SUMO_BINARY,
            "-c", SUMO_CONFIG,
            "--start"
        ])

        self.running = True
        print("✅ Simulation started")

    def step(self):
        """Advance the legacy SUMO simulation by one TraCI step."""
        if not self.running:
            raise RuntimeError("Simulation not started")

        traci.simulationStep()
        stop_if_needed()

    def close(self):
        """Close the legacy SUMO TraCI session if it is running."""
        if self.running:
            traci.close()
            self.running = False
