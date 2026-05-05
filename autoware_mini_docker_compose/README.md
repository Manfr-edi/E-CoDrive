## Prerequisite

- `Docker` with `docker compose`
- A local `CARLA 0.9.13` installation for the Autoware workflow in the main repository setup
- X11 access on the host if you want to launch the GUI stack from the container

## Setup script integration

From the repository root, `./scripts/setup_carla.sh` already runs the equivalent of:

```bash
cd autoware_mini_docker_compose
docker compose up -d --build autoware_mini
```

The manual commands below are still useful if you deleted the container or want to manage it without the setup script.

## To build/start autoware manually

```bash
docker compose up -d --build autoware_mini
```

## To start autoware container

```bash
docker start autoware_mini
```

## Get into autoware_mini container 

```bash
docker exec -it autoware_mini bash
catkin build
. devel/setup.bash
roslaunch autoware_mini start_carla.launch
```
