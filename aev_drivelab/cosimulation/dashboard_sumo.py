#!/usr/bin/env python
"""Dashboard-specific SUMO extensions kept outside CARLA's default runner."""

import collections
import logging
import random
import re

import carla  # pylint: disable=import-error
import traci  # pylint: disable=import-error

from sumo_integration.sumo_simulation import SumoActorClass, SumoSimulation
from aev_drivelab.scenario.sumo_route_tools import (
    AUTOWARE_EGO_VTYPE,
    DEFAULT_EGO_BLUEPRINT,
    DEFAULT_EGO_BATTERY_CAPACITY,
    ENERGY_ATTRIBUTE_DEFAULTS,
    ENERGY_EMISSION_CLASS,
    ENERGY_PARAM_DEFAULTS,
    EGO_SUMO_VTYPE,
    MMPEVEM_ATTRIBUTE_DEFAULTS,
    MMPEVEM_EMISSION_CLASS,
    MMPEVEM_PARAM_DEFAULTS,
    ego_emission_class_value,
    request_autoware_battery_stop,
)


DashboardSumoActor = collections.namedtuple(
    "DashboardSumoActor",
    "type_id vclass transform signals extent color carla_blueprint",
)


COLOR_VALUES = {
    "white": (255, 255, 255, 255),
    "black": (0, 0, 0, 255),
    "gray": (128, 128, 128, 255),
    "silver": (192, 192, 192, 255),
    "red": (255, 0, 0, 255),
    "green": (0, 128, 0, 255),
    "blue": (0, 0, 255, 255),
    "yellow": (255, 255, 0, 255),
    "orange": (255, 165, 0, 255),
    "cyan": (0, 255, 255, 255),
    "magenta": (255, 0, 255, 255),
}
COLOR_NAMES = {value: name for name, value in COLOR_VALUES.items()}


class DashboardSumoSimulation(SumoSimulation):
    """SUMO simulation extension used by the dashboard backend."""

    @staticmethod
    def _is_dashboard_battery_vehicle(veh_id, type_id=None):
        """Return whether the vehicle should be tracked by dashboard battery logic."""
        vehicle_id = str(veh_id)
        resolved_type_id = "" if type_id is None else str(type_id)
        return vehicle_id == "ego_vehicle" or vehicle_id.startswith("carla") or resolved_type_id in {
            EGO_SUMO_VTYPE,
            DEFAULT_EGO_BLUEPRINT,
            AUTOWARE_EGO_VTYPE,
        }

    @staticmethod
    def _resolve_vehicle_id(veh_id):
        """Resolve a user-provided vehicle id against the live TraCI vehicle list."""
        target_id = str(veh_id)
        for candidate_id in traci.vehicle.getIDList():
            if candidate_id == veh_id or str(candidate_id) == target_id:
                return candidate_id
        return None

    @staticmethod
    def _has_battery_device(veh_id):
        """Return whether a vehicle exposes battery-related TraCI parameters."""
        resolved_vehicle_id = DashboardSumoSimulation._resolve_vehicle_id(veh_id)
        if resolved_vehicle_id is None:
            return False

        def _is_enabled(value):
            """Return whether a TraCI parameter value should be treated as enabled."""
            if value in (None, ""):
                return False
            return str(value).strip().lower() not in {"0", "false", "none"}

        for key in ("device.battery.capacity", "has.battery.device"):
            try:
                if _is_enabled(traci.vehicle.getParameter(resolved_vehicle_id, key)):
                    return True
            except traci.exceptions.TraCIException:
                continue

        try:
            type_id = traci.vehicle.getTypeID(resolved_vehicle_id)
        except traci.exceptions.TraCIException:
            return False

        getter = getattr(traci.vehicletype, "getParameter", None)
        if getter is None:
            return False

        for key in ("device.battery.capacity", "has.battery.device"):
            try:
                if _is_enabled(getter(type_id, key)):
                    return True
            except traci.exceptions.TraCIException:
                continue

        return False

    @staticmethod
    def get_actor(actor_id):
        """Build the CARLA-facing actor representation for a subscribed SUMO vehicle."""
        results = traci.vehicle.getSubscriptionResults(actor_id)

        type_id = results[traci.constants.VAR_TYPE]
        vclass = SumoActorClass(results[traci.constants.VAR_VEHICLECLASS])
        color = results[traci.constants.VAR_COLOR]

        length = results[traci.constants.VAR_LENGTH]
        width = results[traci.constants.VAR_WIDTH]
        height = results[traci.constants.VAR_HEIGHT]

        location = list(results[traci.constants.VAR_POSITION3D])
        rotation = [results[traci.constants.VAR_SLOPE], results[traci.constants.VAR_ANGLE], 0.0]
        transform = carla.Transform(
            carla.Location(location[0], location[1], location[2]),
            carla.Rotation(rotation[0], rotation[1], rotation[2]),
        )

        signals = results[traci.constants.VAR_SIGNALS]
        extent = carla.Vector3D(length / 2.0, width / 2.0, height / 2.0)

        try:
            carla_blueprint = traci.vehicle.getParameter(actor_id, "carla.blueprint")
        except traci.exceptions.TraCIException:
            carla_blueprint = ""

        return DashboardSumoActor(
            type_id,
            vclass,
            transform,
            signals,
            extent,
            color,
            carla_blueprint,
        )

    @staticmethod
    def _set_vehicletype_attribute(type_id, setter_name, value, cast=str):
        """Set vehicletype attribute."""
        if value is None or value == "":
            return

        setter = getattr(traci.vehicletype, setter_name, None)
        if setter is None:
            return

        try:
            setter(type_id, cast(value))
        except (TypeError, ValueError, traci.exceptions.TraCIException) as error:
            logging.warning("Could not set vType attribute %s=%s: %s", setter_name, value, error)

    @staticmethod
    def _set_vehicletype_color(type_id, color):
        """Set vehicletype color."""
        if not color:
            return

        try:
            traci.vehicletype.setColor(type_id, COLOR_VALUES.get(color, color))
        except (TypeError, traci.exceptions.TraCIException) as error:
            logging.warning("Could not set vType color %s=%s: %s", type_id, color, error)

    @staticmethod
    def _ensure_vehicle_type(
        type_id,
        carla_blueprint=None,
        vtype_attrs=None,
        vtype_params=None,
        source_type_id=None,
    ):
        """Ensure vehicle type."""
        if not type_id:
            return False

        vtype_attrs = vtype_attrs or {}
        vtype_params = vtype_params or {}
        type_ids = traci.vehicletype.getIDList()

        if type_id not in type_ids:
            if source_type_id in type_ids:
                source_type = source_type_id
            elif carla_blueprint in type_ids:
                source_type = carla_blueprint
            else:
                source_type = "DEFAULT_VEHTYPE"
            try:
                traci.vehicletype.copy(source_type, type_id)
            except traci.exceptions.TraCIException as error:
                logging.error("Could not create ego vType %s from %s: %s", type_id, source_type, error)
                return False

        attr_setters = {
            "emissionClass": ("setEmissionClass", str),
            "vClass": ("setVehicleClass", str),
            "length": ("setLength", float),
            "width": ("setWidth", float),
            "height": ("setHeight", float),
            "maxSpeed": ("setMaxSpeed", float),
            "accel": ("setAccel", float),
            "decel": ("setDecel", float),
            "mass": ("setMass", float),
            "minGap": ("setMinGap", float),
            "sigma": ("setImperfection", float),
            "actionStepLength": ("setActionStepLength", float),
        }
        for key, value in vtype_attrs.items():
            if key == "color":
                DashboardSumoSimulation._set_vehicletype_color(type_id, value)
            elif key in attr_setters:
                setter_name, cast = attr_setters[key]
                DashboardSumoSimulation._set_vehicletype_attribute(
                    type_id,
                    setter_name,
                    value,
                    cast,
                )

        for key, value in vtype_params.items():
            if value is None or value == "":
                continue
            try:
                traci.vehicletype.setParameter(type_id, key, str(value))
            except (AttributeError, traci.exceptions.TraCIException) as error:
                logging.warning("Could not set vType parameter %s=%s: %s", key, value, error)

        return True

    @staticmethod
    def _detect_emission_model(emission_class):
        """Map a SUMO emission class to the dashboard emission-model selector."""
        if emission_class == MMPEVEM_EMISSION_CLASS:
            return MMPEVEM_EMISSION_CLASS
        return ENERGY_EMISSION_CLASS

    @staticmethod
    def _get_vtype_default_config(emission_model):
        """Return default attribute and parameter sets for an emission model."""
        if emission_model == MMPEVEM_EMISSION_CLASS:
            return dict(MMPEVEM_ATTRIBUTE_DEFAULTS), dict(MMPEVEM_PARAM_DEFAULTS)
        return dict(ENERGY_ATTRIBUTE_DEFAULTS), dict(ENERGY_PARAM_DEFAULTS)

    @staticmethod
    def _get_vehicletype_value(type_id, getter_name, default=None):
        """Read a TraCI vehicle-type attribute while tolerating unsupported getters."""
        getter = getattr(traci.vehicletype, getter_name, None)
        if getter is None:
            return default

        try:
            return getter(type_id)
        except traci.exceptions.TraCIException:
            return default

    @staticmethod
    def _get_vehicletype_color_name(type_id, default="white"):
        """Return the normalized color name for a SUMO vehicle type."""
        color = DashboardSumoSimulation._get_vehicletype_value(type_id, "getColor")
        if color is None:
            return default

        if len(color) == 3:
            color = tuple(color) + (255,)
        else:
            color = tuple(color)

        return COLOR_NAMES.get(color, default)

    @staticmethod
    def _vehicle_type_config(type_id, carla_blueprint=None):
        """Build the editable dashboard configuration for a SUMO vehicle type."""
        emission_class = (
            DashboardSumoSimulation._get_vehicletype_value(type_id, "getEmissionClass", "")
            or ""
        )
        emission_model = DashboardSumoSimulation._detect_emission_model(emission_class)
        attributes, parameters = DashboardSumoSimulation._get_vtype_default_config(
            emission_model
        )

        attribute_getters = {
            "minGap": "getMinGap",
            "maxSpeed": "getMaxSpeed",
            "accel": "getAccel",
            "decel": "getDecel",
            "sigma": "getImperfection",
            "mass": "getMass",
            "actionStepLength": "getActionStepLength",
        }

        for key in list(attributes):
            if key == "color":
                attributes[key] = DashboardSumoSimulation._get_vehicletype_color_name(
                    type_id,
                    default=attributes[key],
                )
                continue

            getter_name = attribute_getters.get(key)
            if getter_name is None:
                continue

            value = DashboardSumoSimulation._get_vehicletype_value(type_id, getter_name)
            if value is not None:
                attributes[key] = value

        for key in list(parameters):
            try:
                value = traci.vehicletype.getParameter(type_id, key)
            except traci.exceptions.TraCIException:
                value = ""
            if value not in (None, ""):
                parameters[key] = value

        try:
            battery_capacity = float(
                traci.vehicletype.getParameter(type_id, "device.battery.capacity")
            )
        except (TypeError, ValueError, traci.exceptions.TraCIException):
            battery_capacity = DEFAULT_EGO_BATTERY_CAPACITY

        return {
            "sumo_vtype": type_id,
            "carla_blueprint": carla_blueprint or DEFAULT_EGO_BLUEPRINT,
            "emission_model": emission_model,
            "emission_class": ego_emission_class_value(emission_model),
            "battery_capacity": battery_capacity,
            "attributes": attributes,
            "parameters": parameters,
        }

    @staticmethod
    def _vehicle_charge_level(veh_id, battery_capacity):
        """Return the current battery charge level capped to the configured capacity."""
        try:
            current_charge = float(
                traci.vehicle.getParameter(veh_id, "device.battery.chargeLevel")
            )
            return min(current_charge, float(battery_capacity))
        except (TypeError, ValueError, traci.exceptions.TraCIException):
            return float(battery_capacity)

    @staticmethod
    def _vehicle_failure_threshold(veh_id):
        """Return the configured battery-stop threshold for a vehicle."""
        try:
            value = traci.vehicle.getParameter(veh_id, "dashboard.battery.failureThreshold")
            if value not in (None, ""):
                return value
        except traci.exceptions.TraCIException:
            pass

        try:
            type_id = traci.vehicle.getTypeID(veh_id)
            return traci.vehicletype.getParameter(type_id, "dashboard.battery.failureThreshold")
        except traci.exceptions.TraCIException:
            return ""

    @staticmethod
    def _battery_stop_applied(veh_id):
        """Return whether a battery-stop action has already been applied."""
        try:
            value = traci.vehicle.getParameter(veh_id, "dashboard.battery.stopApplied")
            return str(value).strip().lower() in {"1", "true", "yes"}
        except traci.exceptions.TraCIException:
            return False

    @staticmethod
    def _mark_battery_stop_applied(veh_id):
        """Mark a vehicle as already processed by the battery-stop logic."""
        try:
            traci.vehicle.setParameter(veh_id, "dashboard.battery.stopApplied", "true")
        except traci.exceptions.TraCIException:
            pass

    @staticmethod
    def _request_battery_stop(veh_id, type_id=None):
        """Request battery stop."""
        resolved_type_id = str(type_id or "")
        try:
            traci.vehicle.setSpeed(veh_id, 0)
        except traci.exceptions.TraCIException as error:
            logging.warning("Could not stop vehicle %s after battery depletion: %s", veh_id, error)

        if resolved_type_id == AUTOWARE_EGO_VTYPE:
            try:
                request_autoware_battery_stop()
            except Exception as error:  # pragma: no cover - depends on live ROS runtime
                logging.warning("Could not request Autoware emergency stop for %s: %s", veh_id, error)

        DashboardSumoSimulation._mark_battery_stop_applied(veh_id)

    @staticmethod
    def _live_vehicle_type_id(veh_id):
        """Build a unique temporary vType id for a live vehicle edit."""
        sanitized = re.sub(r"[^A-Za-z0-9_]+", "_", str(veh_id))
        return f"dashboard_live_{sanitized}_{int(traci.simulation.getTime() * 1000)}"

    def list_vehicles(self):
        """Return the live SUMO vehicles visible to the dashboard."""
        vehicles = []
        for veh_id in sorted(
            traci.vehicle.getIDList(),
            key=lambda item: (str(item) != "ego_vehicle", str(item)),
        ):
            type_id = traci.vehicle.getTypeID(veh_id)
            vehicles.append(
                {
                    "id": str(veh_id),
                    "type_id": type_id,
                    "edge": traci.vehicle.getRoadID(veh_id),
                    "speed": traci.vehicle.getSpeed(veh_id),
                    "has_battery_device": (
                        self._has_battery_device(veh_id)
                        if self._is_dashboard_battery_vehicle(veh_id, type_id=type_id)
                        else False
                    ),
                }
            )
        return vehicles

    def get_vehicle_vtype_config(self, veh_id):
        """Return the editable vType configuration for a live vehicle."""
        resolved_vehicle_id = self._resolve_vehicle_id(veh_id)
        if resolved_vehicle_id is None:
            return None

        type_id = traci.vehicle.getTypeID(resolved_vehicle_id)
        try:
            carla_blueprint = traci.vehicle.getParameter(
                resolved_vehicle_id,
                "carla.blueprint",
            )
        except traci.exceptions.TraCIException:
            carla_blueprint = ""
        if not carla_blueprint and type_id.startswith("vehicle."):
            carla_blueprint = type_id

        config = self._vehicle_type_config(type_id, carla_blueprint=carla_blueprint)
        config.update(
            {
                "vehicle_id": str(resolved_vehicle_id),
                "edge": traci.vehicle.getRoadID(resolved_vehicle_id),
                "speed": traci.vehicle.getSpeed(resolved_vehicle_id),
                "has_battery_device": self._has_battery_device(resolved_vehicle_id),
            }
        )
        return config

    def update_vehicle_vtype(
        self,
        veh_id,
        emission_model=ENERGY_EMISSION_CLASS,
        battery_capacity=DEFAULT_EGO_BATTERY_CAPACITY,
        attributes=None,
        parameters=None,
    ):
        """Update vehicle vtype."""
        resolved_vehicle_id = self._resolve_vehicle_id(veh_id)
        if resolved_vehicle_id is None:
            return None
        has_battery_device = self._has_battery_device(resolved_vehicle_id)

        current_type_id = traci.vehicle.getTypeID(resolved_vehicle_id)
        live_type_id = self._live_vehicle_type_id(resolved_vehicle_id)
        try:
            applied_blueprint = traci.vehicle.getParameter(
                resolved_vehicle_id,
                "carla.blueprint",
            )
        except traci.exceptions.TraCIException:
            applied_blueprint = ""
        if not applied_blueprint and current_type_id.startswith("vehicle."):
            applied_blueprint = current_type_id

        vtype_attrs = dict(attributes or {})
        vtype_attrs["emissionClass"] = ego_emission_class_value(emission_model)

        vtype_params = {
        }
        if has_battery_device:
            vtype_params["has.battery.device"] = "true"
            vtype_params["device.battery.capacity"] = str(float(battery_capacity))
        vtype_params.update(
            {
                key: value
                for key, value in (parameters or {}).items()
                if value is not None and str(value).strip() != ""
            }
        )
        if applied_blueprint:
            vtype_params["carla.blueprint"] = applied_blueprint

        if not self._ensure_vehicle_type(
            live_type_id,
            applied_blueprint,
            vtype_attrs,
            vtype_params,
            source_type_id=current_type_id,
        ):
            raise RuntimeError(
                f"Could not create live vType derived from `{current_type_id}` for vehicle `{resolved_vehicle_id}`."
            )

        failure_threshold = self._vehicle_failure_threshold(resolved_vehicle_id)
        charge_level = self._vehicle_charge_level(resolved_vehicle_id, battery_capacity)

        try:
            traci.vehicle.setType(resolved_vehicle_id, live_type_id)
            if applied_blueprint:
                traci.vehicle.setParameter(
                    resolved_vehicle_id,
                    "carla.blueprint",
                    str(applied_blueprint),
                )
            if has_battery_device:
                traci.vehicle.setParameter(resolved_vehicle_id, "has.battery.device", "true")
                traci.vehicle.setParameter(
                    resolved_vehicle_id,
                    "device.battery.capacity",
                    str(float(battery_capacity)),
                )
                traci.vehicle.setParameter(
                    resolved_vehicle_id,
                    "device.battery.chargeLevel",
                    str(charge_level),
                )
                if failure_threshold not in (None, ""):
                    traci.vehicle.setParameter(
                        resolved_vehicle_id,
                        "dashboard.battery.failureThreshold",
                        str(failure_threshold),
                    )
        except traci.exceptions.TraCIException as error:
            raise RuntimeError(
                f"SUMO rejected live vType update for vehicle `{resolved_vehicle_id}`: {error}"
            ) from error

        return self.get_vehicle_vtype_config(resolved_vehicle_id)

    def spawn_ego_vehicle(
        self,
        start_edge,
        end_edge,
        via_edge=None,
        veh_id="ego_vehicle",
        vtype="vehicle.tesla.model3",
        carla_blueprint=None,
        vtype_attrs=None,
        vtype_params=None,
    ):
        """Spawn the dashboard ego vehicle, optionally routing it through a via edge."""
        if not self._ensure_vehicle_type(vtype, carla_blueprint, vtype_attrs, vtype_params):
            return False

        if via_edge and via_edge not in (start_edge, end_edge):
            first_leg = traci.simulation.findRoute(start_edge, via_edge).edges
            second_leg = traci.simulation.findRoute(via_edge, end_edge).edges

            if not first_leg or not second_leg:
                print("Route con via non valida")
                return False

            route = list(first_leg)
            if route[-1] == second_leg[0]:
                route.extend(second_leg[1:])
            else:
                route.extend(second_leg)
        else:
            route = traci.simulation.findRoute(start_edge, end_edge).edges

        if not route:
            print("Route non valida")
            return False

        route_id = f"{veh_id}_route_{int(traci.simulation.getTime() * 1000)}"
        if route_id in traci.route.getIDList():
            route_id = f"{route_id}_{len(traci.route.getIDList())}"

        if veh_id in traci.vehicle.getIDList():
            traci.vehicle.remove(veh_id)

        traci.route.add(route_id, route)
        traci.vehicle.add(vehID=veh_id, routeID=route_id, typeID=vtype)

        vehicle_params = dict(vtype_params or {})
        if carla_blueprint:
            vehicle_params["carla.blueprint"] = carla_blueprint

        for key, value in vehicle_params.items():
            if value is None or value == "":
                continue
            try:
                traci.vehicle.setParameter(veh_id, key, str(value))
            except traci.exceptions.TraCIException as error:
                logging.warning("Could not set ego vehicle parameter %s=%s: %s", key, value, error)

        return True

    def get_vehicle_state(self, veh_id):
        """Return the live dashboard state for a SUMO vehicle."""
        resolved_vehicle_id = self._resolve_vehicle_id(veh_id)
        if resolved_vehicle_id is None:
            return None

        type_id = traci.vehicle.getTypeID(resolved_vehicle_id)
        route_final_edge = None
        distance_travelled_m = None
        distance_remaining_m = None
        battery = None
        battery_failure_threshold = 0.0

        try:
            distance_travelled_m = float(traci.vehicle.getDistance(resolved_vehicle_id))
        except (TypeError, ValueError, traci.exceptions.TraCIException):
            pass

        try:
            route = traci.vehicle.getRoute(resolved_vehicle_id)
            route_final_edge = route[-1] if route else None
        except traci.exceptions.TraCIException:
            route_final_edge = None

        if route_final_edge:
            try:
                target_position = max(0.0, float(self.net.getEdge(route_final_edge).getLength()) - 0.1)
                distance_remaining_m = float(
                    traci.vehicle.getDrivingDistance(
                        resolved_vehicle_id,
                        route_final_edge,
                        target_position,
                    )
                )
                if distance_remaining_m < -1e8:
                    distance_remaining_m = None
                elif distance_remaining_m is not None:
                    distance_remaining_m = max(0.0, distance_remaining_m)
            except (AttributeError, TypeError, ValueError, traci.exceptions.TraCIException):
                distance_remaining_m = None

        if self._is_dashboard_battery_vehicle(resolved_vehicle_id, type_id=type_id):
            try:
                battery = float(
                    traci.vehicle.getParameter(
                        resolved_vehicle_id,
                        "device.battery.chargeLevel",
                    )
                )
            except (TypeError, ValueError, traci.exceptions.TraCIException):
                pass
            try:
                battery_failure_threshold = float(
                    self._vehicle_failure_threshold(resolved_vehicle_id)
                )
            except (TypeError, ValueError, traci.exceptions.TraCIException):
                pass

        return {
            "vehicle_id": str(resolved_vehicle_id),
            "type_id": type_id,
            "speed": traci.vehicle.getSpeed(resolved_vehicle_id),
            "edge": traci.vehicle.getRoadID(resolved_vehicle_id),
            "sim_time": traci.simulation.getTime(),
            "route_final_edge": route_final_edge,
            "distance_travelled_m": distance_travelled_m,
            "distance_remaining_m": distance_remaining_m,
            "battery": battery,
            "battery_failure_threshold": battery_failure_threshold,
        }

    def tick(self):
        """Advance the bridge and enforce dashboard battery-stop rules."""
        super().tick()

        for veh_id in traci.vehicle.getIDList():
            type_id = traci.vehicle.getTypeID(veh_id)
            if not self._is_dashboard_battery_vehicle(veh_id, type_id=type_id):
                continue
            if self._battery_stop_applied(veh_id):
                continue

            state = self.get_vehicle_state(veh_id)
            if not state:
                continue

            battery = state.get("battery")
            threshold = float(state.get("battery_failure_threshold") or 0)
            if threshold <= 0 or battery is None or float(battery) > threshold:
                continue

            self._request_battery_stop(veh_id, type_id=type_id)


def patch_bridge_helper(bridge_helper):
    """Map dashboard vTypes to the selected CARLA blueprint."""

    if getattr(bridge_helper, "_dashboard_blueprint_patch", False):
        return

    original_get_carla_blueprint = bridge_helper.get_carla_blueprint

    def get_carla_blueprint(sumo_actor, sync_color=False):
        """Resolve the CARLA blueprint to use for a dashboard-managed SUMO actor."""
        carla_blueprint = getattr(sumo_actor, "carla_blueprint", "")
        blueprint_library = bridge_helper.blueprint_library
        blueprint_ids = [blueprint.id for blueprint in blueprint_library]

        if carla_blueprint and carla_blueprint in blueprint_ids:
            blueprint = blueprint_library.filter(carla_blueprint)[0]
            logging.debug(
                "[BridgeHelper] sumo vtype %s mapped to carla blueprint %s",
                sumo_actor.type_id,
                carla_blueprint,
            )
        else:
            return original_get_carla_blueprint(sumo_actor, sync_color)

        if blueprint.has_attribute("color"):
            if sync_color:
                color = "{},{},{}".format(
                    sumo_actor.color[0],
                    sumo_actor.color[1],
                    sumo_actor.color[2],
                )
            else:
                color = random.choice(blueprint.get_attribute("color").recommended_values)
            blueprint.set_attribute("color", color)

        if blueprint.has_attribute("driver_id"):
            driver_id = random.choice(blueprint.get_attribute("driver_id").recommended_values)
            blueprint.set_attribute("driver_id", driver_id)

        blueprint.set_attribute("role_name", "sumo_driver")
        return blueprint

    bridge_helper.get_carla_blueprint = staticmethod(get_carla_blueprint)
    bridge_helper._dashboard_blueprint_patch = True
