# AEV-DriveLab

AEV-DriveLab is a Streamlit-based tool for generating and running traffic scenarios that stress the energy consumption of autonomous electric vehicles. It combines SUMO traffic generation with CARLA co-simulation to study how route choice, congestion, traffic density, vehicle type, and battery configuration affect electric-vehicle consumption.

The tool supports both a SUMO-managed ego vehicle workflow and an Autoware-based autonomous vehicle workflow, while keeping scenario generation, co-simulation launch, vehicle configuration, and monitoring in a single dashboard.

## 🧪 Tested Environment

This repository is currently tested on:

- Ubuntu `24.04`
- Python `3.8`
- Docker `29.4.1`
- Docker Compose `5.1.1`
- SUMO `1.26`

## ⚙️ Requirements and Setup

### System Requirements

The minimum setup used to run the project is:

- Ubuntu/Linux (tested on Ubuntu `24.04`)
- Python `3.8`
- Docker and Docker Compose. Follow the installation guide [here](https://docs.docker.com/engine/install/ubuntu/)
- SUMO `1.26`, with the SUMO Python tools available. Follow the installation guide [here](https://sumo.dlr.de/docs/Downloads.php#linux)
- Python packages from `requirements.txt`
- At least one CARLA installation under `carla/`

Install the Python dependencies from the repository root:

```bash
pip install -r requirements.txt
```

If SUMO is installed in the default Ubuntu path, the project should pick it up automatically through `/usr/share/sumo`. Set `SUMO_HOME` manually only when SUMO is installed elsewhere:

```bash
export SUMO_HOME=/path/to/sumo
```

If the CARLA Python API bundled with your CARLA install is not compatible with the interpreter used to start the dashboard, set a dedicated interpreter before launching the project:

```bash
export CARLA_PYTHON=/path/to/python
```

or per CARLA version:

```bash
export CARLA_PYTHON_0_9_13=/path/to/python
export CARLA_PYTHON_0_9_15=/path/to/python
```

That interpreter must be able to import at least `carla`, `flask`, `lxml`, `traci`, `sumolib`, and `setuptools`.

These environment variables are runtime conditions, not persistent setup steps: `./scripts/setup_carla.sh` does not export them into your shell.

### CARLA Setup

At least one CARLA version is required:

- `CARLA 0.9.13`: use this for the Autoware workflow. Download: [CARLA 0.9.13 Linux](https://tiny.carla.org/carla-0-9-13-linux)
- `CARLA 0.9.15`: use this for the SUMO-managed ego vehicle workflow. Download: [CARLA 0.9.15 Linux](https://tiny.carla.org/carla-0-9-15-linux)

You can install one version only, or both.

After downloading, extract the archives inside the repository `carla/` directory, keeping the original folder names intact:

```text
carla/CARLA_0.9.13
carla/CARLA_0.9.15
```

Do not flatten the extracted directories.

Once at least one CARLA installation is in place, run:

```bash
./scripts/setup_carla.sh
```

The script bootstraps the selected CARLA installation(s), installs the project-specific SUMO/CARLA files, writes the vehicle type files, patches the runtime-critical SUMO integration files, and imports the UT Lexus asset required by the Autoware workflow.

If `CARLA 0.9.13` is part of the setup and Docker Compose is available, the script also builds and starts the `autoware_mini` container automatically.

To bootstrap a specific installation:

```bash
./scripts/setup_carla.sh carla/CARLA_0.9.15 0.9.15
```

### Autoware Docker Setup

The Autoware workflow requires `CARLA 0.9.13` and an `autoware_mini` Docker container.

When you run `./scripts/setup_carla.sh` with a local `CARLA 0.9.13` installation present, the script also prepares the Docker side by running the equivalent of:

```bash
cd autoware_mini_docker_compose
docker compose up -d --build autoware_mini
```

Keep the manual commands below as a fallback if the container was deleted, if you want to rebuild it explicitly, or if you prefer not to use the setup script.

Build and start the container manually:

```bash
cd autoware_mini_docker_compose
docker compose up -d --build autoware_mini
```

If the image/container is already built and only needs to be started again:

```bash
docker start autoware_mini
```


## 🚀 Framework Usage

### Typical Local Startup Sequence

1. Install system requirements: Python `3.8`, Docker, Docker Compose, SUMO `1.26`.
2. Install Python packages with `pip install -r requirements.txt`.
3. Download and extract `CARLA 0.9.13` and/or `CARLA 0.9.15` into `carla/`.
4. Run `./scripts/setup_carla.sh`.
5. If using Autoware, make sure Docker Compose is available so the setup script can also build/start `autoware_mini`; otherwise run the manual Docker commands from `autoware_mini_docker_compose/`.
6. Start the dashboard with `streamlit run app.py`.


### Startup

After the requirements and CARLA setup are complete, start the dashboard from the repository root:

```bash
streamlit run app.py
```

If the default Streamlit port is already in use:

```bash
streamlit run app.py --server.port 8502
```

Then open the dashboard in the browser, for example:

```text
http://127.0.0.1:8501
```



The dashboard is organized as an execution stepper. Each step corresponds to one phase of the experimental workflow.

The available steps depend on the selected CARLA version:

- `CARLA 0.9.15`: `Start CARLA` -> `Generate SUMO Routes` -> `SUMO Ego Vehicle` -> `Run Simulation` -> `Monitoring`
- `CARLA 0.9.13`: `Start CARLA` -> `Generate SUMO Routes` -> `Configure Simulation` -> `Ego vType / Autoware` -> `Monitoring`

The workflow below describes the standard autonomous-vehicle experiment: generate traffic, create or select congestion, configure the ego vehicle, run SUMO/CARLA, and monitor energy consumption.

### 1. Start CARLA

Use this step to select the active CARLA version and the Town used by the rest of the workflow.

Main options:

- `CARLA version`: selects the local CARLA installation (`0.9.13` or `0.9.15`).
- `Selected Town`: fixes the map used for route generation, Autoware launch, and co-simulation.
- `Run CARLA` / `Kill CARLA`: starts or stops the selected CARLA server.
- `Load selected Town`: forces the selected Town in a CARLA server that is already running.

### 2. Generate SUMO Routes

Use this step to create the traffic scenario. The scenario is generated on the Town selected in step 1.

Scenario modes:

- `Congestion Edge`: generates traffic that crosses a selected SUMO edge. Source and destination edges are optional constraints.
- `Random Traffic`: generates random routes over the whole network.

Map interaction:

- Click the map to select the nearest SUMO edge.
- Choose the correct edge direction when both directions are available.
- Use the selected edge as congestion, source, or destination.
- Use the invert buttons to switch to the opposite lane direction.

Generation parameters:

- `Vehicles Number`: requested number of traffic vehicles.
- `Start spawn at t[s]` and `Stop spawn at t[s]`: departure time interval.
- `Seed`: deterministic random seed.
- `Spawn distribution`: equidistant, random, or all together for congestion scenarios.
- `Random vType`: assigns a random SUMO vehicle type to each generated vehicle.
- `SUMO vType`: uses a fixed vehicle type when random assignment is disabled.

The step writes the route file and the custom SUMO configuration used by the co-simulation.

### 3A. SUMO Ego Vehicle (`CARLA 0.9.15`)

Use this step when the ego vehicle is managed directly through SUMO and the dashboard backend.

Route options:

- Click the network map and assign `START` and `END` edges.
- Select the explicit edge direction before setting start or end.
- Invert start/end direction when needed.
- Reuse the generated congestion scenario for the ego route.

Congestion-aware ego routing:

- `Pass through congested edge`: builds the route as `start -> congestion -> end`.
- `Use congested edge as start`: uses the congested edge as the ego start.
- `Use congested edge as destination`: uses the congested edge as the ego destination.

Vehicle options:

- CARLA blueprint associated with the ego vehicle.
- SUMO emission model: `Energy` or `MMPEVEM`.
- Maximum battery capacity.
- Initial battery charge.
- Critical battery threshold.
- Detailed SUMO vType attributes and parameters.

Click `Spawn Ego Vehicle` after the co-simulation backend is active.

### 3B. Configure Simulation (`CARLA 0.9.13` / Autoware)

Use this step to arm the SUMO/CARLA bridge before launching Autoware.

Main options:

- `SUMO GUI`: toggles SUMO graphical mode.
- `Autoware startup wait [s]`: warm-up delay used when SUMO runs headless.
- `Start simulation`: starts the bridge and waits for Autoware before simulation time advances.
- `Stop co-simulation`: terminates the bridge process.

In the Autoware workflow, start this step before running Autoware from the ego configuration step.

### 4. Ego vType / Autoware (`CARLA 0.9.13`)

Use this step to configure the Autoware ego vehicle and launch the Autoware stack.

Vehicle options:

- Fixed Autoware CARLA blueprint: `vehicle.lexus.utlexus`.
- SUMO emission model: `Energy` or `MMPEVEM`.
- Maximum battery capacity.
- Current battery charge.
- Critical battery threshold.
- Detailed SUMO vType attributes and parameters.

Route options:

- Click the map and choose explicit start/goal edge directions.
- Reuse the current congestion edge as start or goal.
- Reuse the SUMO ego start/end selections if available.
- Invert start/goal direction.
- Set an Autoware planner speed cap.

Actions:

- `Save ego vType in vtypes.json`: persists the Autoware vehicle type metadata.
- `Run Autoware`: launches Autoware in the Docker container, publishes the initial pose and goal when both edges are selected, and releases the waiting SUMO/CARLA simulation after the configured warm-up.

### 5. Monitoring

Use this step to monitor the ego vehicle or a CARLA-spawned SUMO vehicle during the experiment.

Available outputs:

- Battery level over time.
- Energy consumed over time.
- Vehicle ID, current speed, current SUMO edge, and remaining distance.
- Events such as battery depletion or destination reached.
- Persisted monitoring summary after stop, target change, or terminal event.

Main options:

- `Refresh rate (ms)`: polling interval.
- `Start Monitoring`: clears previous samples and starts a new session.
- `Stop Monitoring`: stops polling and persists the session summary.

## 📁 Generated Files

The dashboard writes generated scenario files to the selected CARLA installation:

```text
<carla_install_dir>/Co-Simulation/Sumo/examples/rou/custom_<Town>_traffic.trips.xml
<carla_install_dir>/Co-Simulation/Sumo/examples/rou/custom_<Town>_traffic.rou.xml
<carla_install_dir>/Co-Simulation/Sumo/examples/custom_<Town>.sumocfg
```

Main log files:

```text
<carla_install_dir>/Co-Simulation/Sumo/examples/output/carla_server.log
<carla_install_dir>/Co-Simulation/Sumo/examples/output/carla_map.log
<carla_install_dir>/Co-Simulation/Sumo/examples/output/run_dashboard_synchronization.log
```

Monitoring exports are written under:

```text
<carla_install_dir>/Co-Simulation/Sumo/examples/output/monitoring/
```

## 🧩 Repository Structure

```text
AEV-DriveLab/
├── app.py                                  # Streamlit dashboard entry point and workflow stepper.
├── requirements.txt                        # Python dependencies for the dashboard and helpers.
├── aev_drivelab/                           # Main Python package.
│   ├── scenario/
│   │   └── sumo_route_tools.py             # SUMO route generation, map helpers, CARLA discovery, and Autoware launch helpers.
│   ├── cosimulation/
│   │   ├── backend_bridge.py               # Starts and manages the dashboard co-simulation backend process.
│   │   ├── dashboard_backend.py            # Flask API exposed to the Streamlit dashboard during co-simulation.
│   │   ├── dashboard_sumo.py               # Ego vehicle management, monitoring data extraction, and battery handling.
│   │   └── run_dashboard_synchronization.py # Dashboard-specific SUMO/CARLA synchronization runner.
│   └── simulation/
│       ├── config.py                       # Shared simulation configuration values.
│       ├── ego_controller.py               # Legacy helpers for ego-vehicle control.
│       └── simulation_backend.py           # Legacy direct-SUMO backend kept separate from the dashboard path.
├── scripts/
│   └── setup_carla.sh                      # Bootstraps CARLA, installs project patches/templates, and starts Autoware when needed.
├── bootstrap_templates/                    # Version-specific files copied into vanilla CARLA installs.
│   ├── 0.9.13/                             # Templates used for the Autoware workflow.
│   ├── 0.9.15/                             # Templates used for the SUMO ego-vehicle workflow.
│   └── common/                             # Shared patched SUMO/CARLA integration files.
├── carla/                                  # Fill this folder with the extracted CARLA distributions used by the project.
│   ├── (TO ADD) CARLA_0.9.13/              # Required for the Autoware-based workflow.
│   ├── (TO ADD) CARLA_0.9.15/              # Required for the SUMO-managed ego vehicle workflow.
│   └── README.txt                          # Short reminder about which CARLA versions are expected here.
└── autoware_mini_docker_compose/           # Docker setup used to build and run the `autoware_mini` container.
    ├── docker-compose.yml                  # Compose definition for the Autoware container.
    ├── README.md                           # Manual container management instructions and fallback commands.
    ├── missing_commands.txt                # Notes about commands expected inside the Autoware environment.
    └── autoware_mini/
        └── Dockerfile                      # Image definition for the Autoware container.
```

Notes:

- `carla/` is intentionally a host folder for local simulator installations: extract the official `CARLA 0.9.13` and/or `CARLA 0.9.15` archives there without renaming or flattening the directories.
- `autoware_mini_docker_compose/` contains the material needed to build and start the Autoware container; `./scripts/setup_carla.sh` will also invoke it automatically when `CARLA 0.9.13` is available.

The synchronization command is launched from:

```text
<carla_install_dir>/Co-Simulation/Sumo
```

and uses the dashboard runner in:

```text
<repo_root>/aev_drivelab/cosimulation/run_dashboard_synchronization.py
```

Main dashboard API endpoints exposed on `localhost:5000` during co-simulation:

```text
GET  /state
GET  /network
GET  /edges
GET  /events
POST /nearest_edge
POST /spawn
POST /vehicle/<vehicle_id>/vtype
```

## 📝 Operational Notes

- Do not run multiple instances of the dashboard synchronization runner on the same Flask port `5000`.
- CARLA must be ready on port `2000` before the selected Town is loaded. The dashboard handles this when CARLA is started from the UI.
- Congestion generation relies on SUMO routing. If the network does not allow a valid route through the selected edge, the final number of generated vehicles may be lower than requested.
- The Autoware workflow requires the UT Lexus CARLA asset imported by `scripts/setup_carla.sh`.
- The dashboard uses the selected CARLA installation to locate maps, route files, SUMO configs, logs, vehicle types, and generated outputs.
