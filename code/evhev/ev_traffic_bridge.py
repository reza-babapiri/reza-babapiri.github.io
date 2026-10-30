import os
import sys
import csv
import time

# ============================================================
# Project settings
# ============================================================

SUMO_HOME = r"<local path redacted>"
PROJECT_DIR = r"<local path redacted>"

os.environ["SUMO_HOME"] = SUMO_HOME
sys.path.append(os.path.join(SUMO_HOME, "tools"))

import traci
import paho.mqtt.client as mqtt


# ============================================================
# SUMO / scenario settings
# ============================================================

USE_SUMO_GUI = True

if USE_SUMO_GUI:
    sumo_binary = os.path.join(SUMO_HOME, "bin", "sumo-gui.exe")
else:
    sumo_binary = os.path.join(SUMO_HOME, "bin", "sumo.exe")


# Choose: "light", "medium", or "heavy"
SCENARIO = "heavy"

if SCENARIO == "light":
    sumo_config = "scenario_ev_light.sumocfg"
    scenario_name = "EV_Light"

elif SCENARIO == "medium":
    sumo_config = "scenario_ev_medium.sumocfg"
    scenario_name = "EV_Medium"

elif SCENARIO == "heavy":
    sumo_config = "scenario_ev_heavy.sumocfg"
    scenario_name = "EV_Heavy"

else:
    raise ValueError("SCENARIO must be 'light', 'medium', or 'heavy'.")


# Must match the EV vehicle ID in ev_vehicle.rou.xml
vehicle_id = "ev"


# ============================================================
# Timing
# ============================================================

dt = 0.5

# Total SUMO steps for 3600 seconds
# 3600 / 0.5 = 7200
max_sumo_steps = 7200

# EV appears at depart = 120 s
# Co-simulation duration after EV appears:
# (3600 - 120) / 0.5 = 6960
target_cosim_steps = 6960

mqtt_timeout = 250

# ============================================================
# MQTT settings
# ============================================================

mqtt_broker = "broker.hivemq.com"
mqtt_port = 1883

topic_v_ref = "cosim/ev/v_ref"
topic_v_actual = "cosim/ev/v_actual"
topic_control = "cosim/ev/control"


# ============================================================
# Output file
# ============================================================

python_result_file = f"{scenario_name}_python_live_result.csv"


# ============================================================
# MQTT storage
# ============================================================

latest_v_actual = {}
received_steps = set()


def on_message(client, userdata, msg):
    payload = msg.payload.decode()

    try:
        parts = payload.split(":")
        sumo_step = int(float(parts[0]))
        v_actual = float(parts[1])

        latest_v_actual[sumo_step] = v_actual
        received_steps.add(sumo_step)

    except Exception:
        print("BAD MQTT PAYLOAD:", payload)


def setup_mqtt():
    client = mqtt.Client()
    client.on_message = on_message

    client.connect(mqtt_broker, mqtt_port, 60)
    client.subscribe(topic_v_actual, qos=1)
    client.loop_start()

    client.publish(topic_control, "INIT", qos=1)

    return client


# ============================================================
# Safe helper functions
# ============================================================

def safe_call(func, default_value):
    try:
        return func()
    except Exception:
        return default_value


def safe_get_decel(veh_id, default_value=4.5):
    try:
        return traci.vehicle.getDecel(veh_id)
    except Exception:
        return default_value


def angle_difference_deg(a, b):
    if a < 0 or b < 0:
        return -1.0

    diff = abs(a - b)

    if diff > 180:
        diff = 360 - diff

    return diff


def safe_next_edge_info(veh_id):
    try:
        route = traci.vehicle.getRoute(veh_id)
        route_index = traci.vehicle.getRouteIndex(veh_id)

        if route_index < 0 or route_index >= len(route):
            return "", "", -1.0, -1.0, -1.0

        current_edge = route[route_index]

        if route_index < len(route) - 1:
            next_edge = route[route_index + 1]
        else:
            next_edge = ""

        current_angle = safe_call(lambda: traci.edge.getAngle(current_edge), -1.0)

        if next_edge != "":
            next_angle = safe_call(lambda: traci.edge.getAngle(next_edge), -1.0)
        else:
            next_angle = -1.0

        edge_angle_diff = angle_difference_deg(current_angle, next_angle)

        return current_edge, next_edge, current_angle, next_angle, edge_angle_diff

    except Exception:
        return "", "", -1.0, -1.0, -1.0


def safe_emissions(veh_id):
    fuel_mg_s = 0.0
    electricity_Wh_s = safe_call(lambda: traci.vehicle.getElectricityConsumption(veh_id), -1.0)
    co2_mg_s = safe_call(lambda: traci.vehicle.getCO2Emission(veh_id), -1.0)
    co_mg_s = safe_call(lambda: traci.vehicle.getCOEmission(veh_id), -1.0)
    nox_mg_s = safe_call(lambda: traci.vehicle.getNOxEmission(veh_id), -1.0)
    pmx_mg_s = safe_call(lambda: traci.vehicle.getPMxEmission(veh_id), -1.0)
    hc_mg_s = safe_call(lambda: traci.vehicle.getHCEmission(veh_id), -1.0)
    noise_db = safe_call(lambda: traci.vehicle.getNoiseEmission(veh_id), -1.0)

    return (
        fuel_mg_s,
        electricity_Wh_s,
        co2_mg_s,
        co_mg_s,
        nox_mg_s,
        pmx_mg_s,
        hc_mg_s,
        noise_db
    )


# ============================================================
# Reference speed calculation
# ============================================================

def compute_traffic_aware_v_ref(veh_id):
    """
    Python is not the controller.
    It computes the traffic-aware speed demand from SUMO
    and sends it to Simulink as Target_Speed.

    No steering correction is added here.
    Turning/steering remains an observation/limitation.
    """

    current_speed = traci.vehicle.getSpeed(veh_id)
    allowed_speed = traci.vehicle.getAllowedSpeed(veh_id)

    v_ref = allowed_speed

    leader_exists = 0
    leader_id = ""
    leader_gap = -1.0
    leader_speed = -1.0
    follow_speed = allowed_speed

    tls_id = ""
    tls_index = -1
    tls_state = ""
    tls_distance = -1.0
    stop_speed = allowed_speed

    # Leader / car-following effect
    leader_info = traci.vehicle.getLeader(veh_id, 150)

    if leader_info is not None:
        leader_exists = 1
        leader_id, leader_gap = leader_info
        leader_speed = traci.vehicle.getSpeed(leader_id)
        leader_decel = safe_get_decel(leader_id)

        try:
            follow_speed = traci.vehicle.getFollowSpeed(
                veh_id,
                current_speed,
                leader_gap,
                leader_speed,
                leader_decel,
                leader_id
            )

        except TypeError:
            try:
                follow_speed = traci.vehicle.getFollowSpeed(
                    veh_id,
                    current_speed,
                    leader_gap,
                    leader_speed,
                    leader_decel
                )
            except Exception:
                follow_speed = allowed_speed

        except Exception:
            follow_speed = allowed_speed

        if follow_speed < 0 or follow_speed != follow_speed:
            if leader_gap < 8:
                follow_speed = 0.0
            elif leader_gap < 20:
                follow_speed = min(allowed_speed, leader_speed)
            else:
                follow_speed = allowed_speed

        v_ref = min(v_ref, follow_speed)

    # Traffic-light effect
    tls_info = traci.vehicle.getNextTLS(veh_id)

    if len(tls_info) > 0:
        tls_id, tls_index, tls_distance, tls_state = tls_info[0]

        if tls_state.lower() in ["r", "y"]:

            try:
                stop_speed = traci.vehicle.getStopSpeed(
                    veh_id,
                    current_speed,
                    max(tls_distance, 0.0)
                )

            except Exception:
                if tls_distance < 5:
                    stop_speed = 0.0
                elif tls_distance < 40:
                    stop_speed = max(0.0, allowed_speed * tls_distance / 40.0)
                else:
                    stop_speed = allowed_speed

            v_ref = min(v_ref, stop_speed)

            # Force full stop very close to a red/yellow light
            if tls_distance <= 1.5:
                v_ref = 0.0

    # Allow full stop near red lights or blocked leaders
    v_ref = max(v_ref, 0.0)

    diagnostics = {
        "current_speed": current_speed,
        "allowed_speed": allowed_speed,

        "leader_exists": leader_exists,
        "leader_id": leader_id,
        "leader_gap": leader_gap,
        "leader_speed": leader_speed,
        "follow_speed": follow_speed,

        "tls_id": tls_id,
        "tls_index": tls_index,
        "tls_state": tls_state,
        "tls_distance": tls_distance,
        "stop_speed": stop_speed,
    }

    return v_ref, diagnostics


def wait_for_v_actual(sumo_step):
    wait_start = time.time()

    while sumo_step not in received_steps:
        if time.time() - wait_start > mqtt_timeout:
            return None

        time.sleep(0.005)

    return latest_v_actual[sumo_step]


# ============================================================
# Main
# ============================================================

def main():
    os.chdir(PROJECT_DIR)

    print("Starting Python EV live co-simulation")
    print("Scenario:", scenario_name)
    print("SUMO config:", sumo_config)
    print("Vehicle ID:", vehicle_id)
    print("dt:", dt)
    print("max_sumo_steps:", max_sumo_steps)
    print("target_cosim_steps:", target_cosim_steps)
    print("Output:", python_result_file)

    mqtt_client = setup_mqtt()

    if USE_SUMO_GUI:
        sumo_cmd = [
            sumo_binary,
            "-c", sumo_config,
            "--step-length", str(dt),
            "--start",
            "--delay", "50",
            "--quit-on-end", "false"
        ]
    else:
        sumo_cmd = [
            sumo_binary,
            "-c", sumo_config,
            "--step-length", str(dt),
            "--no-step-log", "true",
            "--no-warnings", "true"
        ]

    traci.start(sumo_cmd, numRetries=20)

    log = []
    previous_v_actual = None
    previous_angle = None

    cosim_count = 0
    vehicle_seen_once = False

    cumulative_sumo_fuel_mg = 0.0
    cumulative_sumo_electricity_Wh = 0.0

    try:
        for sumo_step in range(max_sumo_steps):

            traci.simulationStep()

            vehicle_ids = traci.vehicle.getIDList()

            if sumo_step % 20 == 0 and cosim_count == 0:
                print(
                    "diagnostic:",
                    "SUMO step =", sumo_step,
                    "SUMO time =", round(traci.simulation.getTime(), 2),
                    "vehicle count =", len(vehicle_ids),
                    "EV present =", vehicle_id in vehicle_ids
                )

            # Warm-up until EV appears
            if vehicle_id not in vehicle_ids:
                if vehicle_seen_once:
                    print(f"{vehicle_id} finished route or disappeared. Stopping co-simulation.")
                    break
                continue

            if not vehicle_seen_once:
                vehicle_seen_once = True
                print("EV appeared at SUMO step:", sumo_step)
                print("EV appeared at SUMO time:", traci.simulation.getTime())

                if USE_SUMO_GUI:
                    try:
                        traci.gui.trackVehicle("View #0", vehicle_id)
                        traci.gui.setZoom("View #0", 1200)
                    except Exception:
                        pass

            # ------------------------------------------------
            # SUMO outputs
            # ------------------------------------------------

            v_ref, diag = compute_traffic_aware_v_ref(vehicle_id)

            sumo_speed = safe_call(lambda: traci.vehicle.getSpeed(vehicle_id), -1.0)
            allowed_speed = safe_call(lambda: traci.vehicle.getAllowedSpeed(vehicle_id), -1.0)
            acceleration_sumo = safe_call(lambda: traci.vehicle.getAcceleration(vehicle_id), 0.0)

            position = safe_call(lambda: traci.vehicle.getPosition(vehicle_id), (-1.0, -1.0))
            x_pos, y_pos = position

            vehicle_angle = safe_call(lambda: traci.vehicle.getAngle(vehicle_id), -1.0)

            if previous_angle is None:
                angle_rate = 0.0
            else:
                angle_rate = angle_difference_deg(vehicle_angle, previous_angle) / dt

            previous_angle = vehicle_angle

            road_id = safe_call(lambda: traci.vehicle.getRoadID(vehicle_id), "")
            lane_id = safe_call(lambda: traci.vehicle.getLaneID(vehicle_id), "")
            lane_index = safe_call(lambda: traci.vehicle.getLaneIndex(vehicle_id), -1)
            lane_position = safe_call(lambda: traci.vehicle.getLanePosition(vehicle_id), -1.0)
            route_index = safe_call(lambda: traci.vehicle.getRouteIndex(vehicle_id), -1)
            distance_travelled = safe_call(lambda: traci.vehicle.getDistance(vehicle_id), -1.0)

            current_edge, next_edge, current_edge_angle, next_edge_angle, edge_angle_diff = safe_next_edge_info(vehicle_id)

            waiting_time = safe_call(lambda: traci.vehicle.getWaitingTime(vehicle_id), -1.0)
            accumulated_waiting_time = safe_call(lambda: traci.vehicle.getAccumulatedWaitingTime(vehicle_id), -1.0)
            time_loss = safe_call(lambda: traci.vehicle.getTimeLoss(vehicle_id), -1.0)

            (
                fuel_mg_s,
                electricity_Wh_s,
                co2_mg_s,
                co_mg_s,
                nox_mg_s,
                pmx_mg_s,
                hc_mg_s,
                noise_db
            ) = safe_emissions(vehicle_id)

            fuel_mg_this_step = fuel_mg_s * dt if fuel_mg_s >= 0 else -1.0
            electricity_Wh_this_step = electricity_Wh_s * dt if electricity_Wh_s >= 0 else -1.0

            if fuel_mg_this_step >= 0:
                cumulative_sumo_fuel_mg += fuel_mg_this_step

            if electricity_Wh_this_step >= 0:
                cumulative_sumo_electricity_Wh += electricity_Wh_this_step

            # ------------------------------------------------
            # Send Target_Speed to Simulink
            # ------------------------------------------------

            payload = f"{sumo_step}:{v_ref}"
            mqtt_client.publish(topic_v_ref, payload, qos=1)

            v_actual = wait_for_v_actual(sumo_step)

            if v_actual is None:
                print(f"TIMEOUT at SUMO step {sumo_step}. Using v_ref as fallback.")
                v_actual = v_ref

            # Apply Simulink output velocity back to SUMO
            traci.vehicle.setSpeed(vehicle_id, v_actual)

            if previous_v_actual is None:
                acceleration_from_simulink = 0.0
            else:
                acceleration_from_simulink = (v_actual - previous_v_actual) / dt

            previous_v_actual = v_actual

            # ------------------------------------------------
            # Log
            # ------------------------------------------------

            log.append([
                sumo_step,
                cosim_count,
                sumo_step * dt,
                cosim_count * dt,

                v_ref,
                v_actual,
                acceleration_from_simulink,

                sumo_speed,
                allowed_speed,
                acceleration_sumo,

                x_pos,
                y_pos,
                vehicle_angle,
                angle_rate,

                road_id,
                lane_id,
                lane_index,
                lane_position,
                route_index,
                distance_travelled,

                current_edge,
                next_edge,
                current_edge_angle,
                next_edge_angle,
                edge_angle_diff,

                diag["leader_exists"],
                diag["leader_id"],
                diag["leader_gap"],
                diag["leader_speed"],
                diag["follow_speed"],

                diag["tls_id"],
                diag["tls_index"],
                diag["tls_state"],
                diag["tls_distance"],
                diag["stop_speed"],

                waiting_time,
                accumulated_waiting_time,
                time_loss,

                fuel_mg_s,
                fuel_mg_this_step,
                cumulative_sumo_fuel_mg,

                electricity_Wh_s,
                electricity_Wh_this_step,
                cumulative_sumo_electricity_Wh,

                co2_mg_s,
                co_mg_s,
                nox_mg_s,
                pmx_mg_s,
                hc_mg_s,
                noise_db
            ])

            print(
                f"SUMO step={sumo_step:03d} "
                f"local={cosim_count:03d} "
                f"v_ref={v_ref:.3f} "
                f"v_actual={v_actual:.3f} "
                f"leader={diag['leader_exists']} "
                f"gap={diag['leader_gap']:.1f} "
                f"tls={diag['tls_state']} "
                f"tls_dist={diag['tls_distance']:.1f} "
                f"angle={vehicle_angle:.1f} "
                f"dAngle={angle_rate:.1f} "
                f"fuel={fuel_mg_s:.2f} "
                f"elec={electricity_Wh_s:.4f}"
            )

            cosim_count += 1

            if cosim_count >= target_cosim_steps:
                print("Target co-simulation steps reached.")
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user. Saving available data.")

    finally:
        try:
            traci.close()
        except Exception:
            pass

        try:
            mqtt_client.publish(topic_control, "STOP", qos=1)
            time.sleep(0.5)
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        except Exception:
            pass

        with open(python_result_file, "w", newline="") as file:
            writer = csv.writer(file)

            writer.writerow([
                "SUMO_step",
                "LocalStep",
                "SUMO_time_s",
                "Local_time_s",

                "v_ref_mps",
                "v_actual_mps",
                "accel_from_simulink_mps2",

                "sumo_speed_mps",
                "allowed_speed_mps",
                "sumo_acceleration_mps2",

                "x_position_m",
                "y_position_m",
                "vehicle_angle_deg",
                "angle_rate_deg_per_s",

                "road_id",
                "lane_id",
                "lane_index",
                "lane_position_m",
                "route_index",
                "distance_travelled_m",

                "current_edge",
                "next_edge",
                "current_edge_angle_deg",
                "next_edge_angle_deg",
                "edge_angle_difference_deg",

                "leader_exists",
                "leader_id",
                "leader_gap_m",
                "leader_speed_mps",
                "follow_speed_mps",

                "tls_id",
                "tls_index",
                "tls_state",
                "tls_distance_m",
                "stop_speed_mps",

                "waiting_time_s",
                "accumulated_waiting_time_s",
                "time_loss_s",

                "sumo_fuel_mg_per_s",
                "sumo_fuel_mg_this_step",
                "sumo_cumulative_fuel_mg",

                "sumo_electricity_Wh_per_s",
                "sumo_electricity_Wh_this_step",
                "sumo_cumulative_electricity_Wh",

                "CO2_mg_per_s",
                "CO_mg_per_s",
                "NOx_mg_per_s",
                "PMx_mg_per_s",
                "HC_mg_per_s",
                "noise_dB"
            ])

            writer.writerows(log)

        print("Finished EV live co-simulation.")
        print(f"Saved file: {python_result_file}")


if __name__ == "__main__":
    main()