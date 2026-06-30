import time
import math
import threading
import numpy as np
import cv2
import ml.NN as nn_module
from pymavlink import mavutil
from enum import Enum, auto

from pymavlink import mavutil
from arm_pipeline import (
    pipeline_quadplane_vtol_takeoff_to_auto,
    transition_fw_to_vtol,
    set_mode_and_confirm,
    check_arspd_use,
)
from goto import upload_loiter_unlim, navigate_to_target_vtol, release_rc_override
from connection import send_gcs_heartbeat, connect_serial, check_airspeed_sensor
from ml.conversion import GPS_target
from ml.NN import get_normalized_coordinates, latest_detection

METERS_PER_DEG_LAT = 111320.0

TRANSITION_TRIGGER_M = 200.0   
APPROACH_RADIUS_M    = 15.0    
APPROACH_TIMEOUT_S   = 120.0  



def gcs_keepalive_tick(master, last_hb, period_s=1.0):
    now = time.time()
    if now - last_hb >= period_s:
        send_gcs_heartbeat(master)
        return now
    return last_hb

def meters_per_deg_lon(lat_deg: float) -> float:
    return 111320.0 * math.cos(math.radians(lat_deg))

def dist_to_wp_m(ref_lat, cur_lat, cur_lon, wp_lat, wp_lon):
    m_per_lon = meters_per_deg_lon(ref_lat)
    dN = (wp_lat - cur_lat) * METERS_PER_DEG_LAT
    dE = (wp_lon - cur_lon) * m_per_lon
    return math.sqrt(dN*dN + dE*dE)

def ned_to_latlon(center_lat, center_lon, dN_m, dE_m):
    lat = center_lat + dN_m / METERS_PER_DEG_LAT
    lon = center_lon + dE_m / meters_per_deg_lon(center_lat)
    return lat, lon

def generate_hypodrome_wps(center_lat, center_lon, L=98.0, W=48.0,
                           step_straight_m=10.0, step_arc_deg=15.0):
    R = W / 2.0
    straight_len = max(0.0, L - 2.0 * R)
    half_straight = straight_len / 2.0
    cN_north = +half_straight
    cN_south = -half_straight
    wps = []

    E = +R
    n = -half_straight
    while n <= +half_straight + 1e-6:
        wps.append(ned_to_latlon(center_lat, center_lon, n, E))
        n += step_straight_m

    ang = 0.0
    while ang <= 180.0 + 1e-6:
        phi = math.radians(ang)
        N = cN_north + R * math.cos(phi)
        E = R * math.sin(phi)
        wps.append(ned_to_latlon(center_lat, center_lon, N, E))
        ang += step_arc_deg

    E = -R
    n = +half_straight
    while n >= -half_straight - 1e-6:
        wps.append(ned_to_latlon(center_lat, center_lon, n, E))
        n -= step_straight_m

    ang = 180.0
    while ang <= 360.0 + 1e-6:
        phi = math.radians(ang)
        N = cN_south + R * math.cos(phi)
        E = R * math.sin(phi)
        wps.append(ned_to_latlon(center_lat, center_lon, N, E))
        ang += step_arc_deg

    return wps


def wait_disarmed(master, timeout=120):
    t0 = time.time()
    last_hb = 0.0
    while time.time() - t0 < timeout:
        last_hb = gcs_keepalive_tick(master, last_hb, period_s=1.0)
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if not armed:
                print("UAV disarmed")
                return True
    raise RuntimeError("Disarming timeout")


def _kill_switch_listener(master):
    """
    Thread de fond qui écoute la touche K.
    Appuyer sur K + Entrée déclenche le kill switch.
    """
    print("Kill switch actif – appuie sur K + Entrée pour désarmer en vol")
    while True:
        try:
            key = input()
            if key.strip().upper() == "K":
                from arm_pipeline import emergency_kill
                emergency_kill(master)
                break
        except Exception:
            break

def wait_landed(master, timeout=240):
    """
    EXTENDED_SYS_STATE.landed_state:
      1 = ON_GROUND
    """
    t0 = time.time()
    last_print = 0.0
    last_hb = 0.0
    while time.time() - t0 < timeout:
        last_hb = gcs_keepalive_tick(master, last_hb, period_s=1.0)
        msg = master.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=1)
        if msg and int(msg.landed_state) == 1:
            print("UAV landed")
            return True
        if time.time() - last_print > 2.0:
            print("Waiting for landing...")
            last_print = time.time()
    raise RuntimeError("Timeout waiting for landing")


class State(Enum):
    SEARCH_FW             = auto()
    TRACK_DETECTED        = auto()
    ANTICIPATE_TRANSITION = auto()   # attend d'être à TRANSITION_TRIGGER_M
    TRANSITION_TO_VTOL    = auto()
    VTOL_HOLD_OVER_TARGET = auto()   # 100 % AUTO via LOITER_UNLIM
    RETURN_HOME           = auto()
    FAILSAFE              = auto()


def main():
    ALT_TARGET_M = 30.0
    FW_SEARCH_AIRSPD_MPS = 10.0
    DT           = 0.5
    HOLD_TIME_S  = 30.0

    master = connect_serial(port="COM5", baud=57600)
    kill_thread = threading.Thread(target=_kill_switch_listener, args=(master,), daemon=True)
    kill_thread.start()

    get_normalized_coordinates_thread = threading.Thread(target=get_normalized_coordinates, daemon=True)
    get_normalized_coordinates_thread.start()

    print("\n=== PRÉ-VOL: vérification capteur pitot ===")
    airspeed_ok = check_airspeed_sensor(master, timeout=5.0)
    if airspeed_ok:
        check_arspd_use(master)
    else:
        print("Capteur pitot non détecté – DO_CHANGE_SPEED sera rejeté")
        print(" Continuer ? (Ctrl+C pour annuler, Enter pour continuer sans pitot)")
        try:
            input()
        except KeyboardInterrupt:
            print("Annulé.")
            return

    start_time = time.time()

    pipeline_quadplane_vtol_takeoff_to_auto(master, target_alt=ALT_TARGET_M, airspeed_mps=FW_SEARCH_AIRSPD_MPS)

    state = State.SEARCH_FW
    target_latlon = None
    last_lat = None
    last_lon = None
    last_gcs_hb = time.time()

    while True:

        last_gcs_hb = gcs_keepalive_tick(master, last_gcs_hb, period_s=1.0)

        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg is not None:
            last_lat = msg.lat / 1e7
            last_lon = msg.lon / 1e7

        if last_lat is None or last_lon is None:
            time.sleep(DT)
            continue

        if state == State.SEARCH_FW:

            if latest_detection is not None:
                x, y, w, h, track_id = latest_detection
                print(f"Track ID: {track_id}, Normalized Box: (x={x}, y={y}, w={w}, h={h})")
                target_latlon = GPS_target(master, x, y, w, h, track_id)
                print(f"Target GPS: lat={target_latlon[0]:.7f}, lon={target_latlon[1]:.7f}")
                state = State.TRACK_DETECTED
                nn_module.latest_detection = None  

            time.sleep(DT)
            if state != State.SEARCH_FW:
                continue
        elif state == State.TRACK_DETECTED:
            if target_latlon is None:
                state = State.SEARCH_FW
                continue
            tgt_lat, tgt_lon = target_latlon
            d = dist_to_wp_m(tgt_lat, last_lat, last_lon, tgt_lat, tgt_lon)
            print(f"Target locked at {tgt_lat:.7f}, {tgt_lon:.7f} (d={d:.1f}m)")
            print(f"Transition will trigger at d <= {TRANSITION_TRIGGER_M}m")
            state = State.TRANSITION_TO_VTOL

        elif state == State.TRANSITION_TO_VTOL:
            transition_fw_to_vtol(master)
            state = State.VTOL_HOLD_OVER_TARGET

        elif state == State.VTOL_HOLD_OVER_TARGET:
            tgt_lat, tgt_lon = target_latlon
            last_gcs_hb = time.time()

            print(f"Navigating to target in AUTO: lat={tgt_lat:.7f}, lon={tgt_lon:.7f}")

            def _hb():
                nonlocal last_gcs_hb
                last_gcs_hb = gcs_keepalive_tick(master, last_gcs_hb, period_s=1.0)

            arrived = navigate_to_target_vtol(
                master,
                tgt_lat=tgt_lat,
                tgt_lon=tgt_lon,
                alt_rel_m=ALT_TARGET_M,
                arrival_radius_m=APPROACH_RADIUS_M,
                timeout_s=APPROACH_TIMEOUT_S,
                gcs_keepalive_fn=_hb,
            )

            if arrived:
                print("On target — holding in AUTO/LOITER_UNLIM")
            else:
                print("Navigation timeout — uploading LOITER_UNLIM at current position")
                upload_loiter_unlim(
                    master,
                    lat_deg=last_lat,
                    lon_deg=last_lon,
                    alt_rel_m=ALT_TARGET_M,
                    gcs_keepalive_fn=_hb,
                )

            print(f"HOLD AUTO/LOITER_UNLIM pendant {HOLD_TIME_S:.0f}s...")
            t0 = time.time()
            while True:
                last_gcs_hb = gcs_keepalive_tick(master, last_gcs_hb, period_s=1.0)

                msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
                if msg is not None:
                    last_lat = msg.lat / 1e7
                    last_lon = msg.lon / 1e7

                d = dist_to_wp_m(tgt_lat, last_lat, last_lon, tgt_lat, tgt_lon)
                elapsed = time.time() - t0

                if elapsed >= HOLD_TIME_S:
                    print(f"HOLD done: t={elapsed:.1f}s (d={d:.1f}m) -> RETURN_HOME")
                    break

                time.sleep(DT)

            state = State.RETURN_HOME

        elif state == State.RETURN_HOME:
            # Seul endroit où l'on quitte AUTO : QRTL pour le retour/atterrissage.
            print("RETURN HOME: switching to QRTL (safe VTOL return mode)")
            set_mode_and_confirm(master, "QRTL", timeout=15)
            print("Waiting for landing + disarm...")
            wait_landed(master, timeout=240)
            wait_disarmed(master, timeout=120)
            print("RETURN HOME complete")
            break
        elif state == State.FAILSAFE:
            print("FAILSAFE: switching to QRTL")
            try:
                set_mode_and_confirm(master, "QRTL", timeout=10)
            except Exception:
                pass
            break


if __name__ == "__main__":
    main()
