import time
from pymavlink import mavutil


def _set_msg_interval(master, msg_id, hz):
    interval_us = int(1e6 / hz) if hz > 0 else 0
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0, 0, 0, 0, 0
    )


def connect_serial(port="COM5", baud=57600, heartbeat_timeout=10):
   
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat(timeout=heartbeat_timeout)
    print(f"Heartbeat ok (system={master.target_system}, component={master.target_component})")

    _set_msg_interval(master, 33,  10)   # GLOBAL_POSITION_INT  @ 10 Hz
    _set_msg_interval(master, 245,  5)   # EXTENDED_SYS_STATE   @  5 Hz
    _set_msg_interval(master, 74,   5)   # VFR_HUD              @  5 Hz
    _set_msg_interval(master, 1,    4)   # SYS_STATUS           @  4 Hz
    _set_msg_interval(master, 30,   2)   # ATTITUDE             @  2 Hz

    time.sleep(0.3)
    return master


def check_airspeed_sensor(master, timeout=5.0):
    """
    Vérifie que le capteur pitot est présent, activé et sain via SYS_STATUS.
    """
    AIRSPEED_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE

    msg = master.recv_match(type="SYS_STATUS", blocking=True, timeout=timeout)
    if msg is None:
        print("SYS_STATUS non reçu – vérification capteur vitesse impossible")
        return False

    present = bool(msg.onboard_control_sensors_present & AIRSPEED_BIT)
    enabled = bool(msg.onboard_control_sensors_enabled & AIRSPEED_BIT)
    healthy = bool(msg.onboard_control_sensors_health  & AIRSPEED_BIT)

    if present and enabled and healthy:
        print("Capteur pitot : présent, activé, sain")
        return True
    else:
        print(f"Problème capteur pitot : present={present} enabled={enabled} healthy={healthy}")
        print("Vérifier ARSPD_TYPE != 0, ARSPD_USE = 1, câblage pitot")
        return False


def send_gcs_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )
