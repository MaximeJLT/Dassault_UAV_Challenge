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
    """
    Connexion MAVLink au drone via liaison série (télémétrie radio).

    Paramètres :
      port : port série du module telemetrie
             Linux   → "/dev/ttyUSB0"  ou  "/dev/ttyACM0"
             Windows → "COM3", "COM5", etc.
      baud : débit en bauds — 57600 est le standard SiK/RFD900

    Détection du bon port Linux :
      ls /dev/ttyUSB* /dev/ttyACM*
    """
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

# def connect_udp(host="127.0.0.1", port=14550, heartbeat_timeout=10):
#     """
#     Connexion MAVLink au drone via UDP.
#     Paramètres :
#       host : adresse IP source des paquets UDP
#              GCS locale      → "127.0.0.1"
#              GCS réseau      → "0.0.0.0"  (écoute sur toutes les interfaces)
#              SITL ArduPilot  → "127.0.0.1:14550"
#       port : port UDP — 14550 est le standard MAVLink/GCS
#     Exemples de chaînes de connexion :
#       udp:127.0.0.1:14550   ← reçoit les paquets envoyés à ce port
#       udpout:192.168.1.1:14550  ← se connecte à une adresse distante
#     """
#     connection_string = f"udp:{host}:{port}"
#     master = mavutil.mavlink_connection(connection_string)
#     master.wait_heartbeat(timeout=heartbeat_timeout)
#     print(f"Heartbeat ok (system={master.target_system}, component={master.target_component})")
#     _set_msg_interval(master, 33,  10)   # GLOBAL_POSITION_INT                 @ 10 Hz
#     _set_msg_interval(master, 245,  5)   # EXTENDED_SYS_STATE                  @  5 Hz
#     _set_msg_interval(master, 74,   5)   # VFR_HUD                             @  5 Hz
#     _set_msg_interval(master, 1,    4)   # SYS_STATUS                          @  4 Hz
#     _set_msg_interval(master, 30,   2)   # ATTITUDE                            @  2 Hz
#     _set_msg_interval(master, 132, 10)   # distance LiDAR (DISTANCE_SENSOR)    @ 10 Hz
#     _set_msg_interval(master, 154, 10)   # GIMBAL_DEVICE_ATTITUDE_STATUS       @ 10 Hz
#     time.sleep(0.3)
#     return master


# def connect_udp(ip="127.0.0.1", port=14551, heartbeat_timeout=10):
#     """
#     Connexion MAVLink au drone via UDP (SITL / simulation uniquement).

#     Ports :
#       14551 = script Python (dédié, sans MAVProxy entre les deux)
#       14550 = MAVProxy / Mission Planner

#     Pour un vol réel via radio télémétrie série, utiliser connect_serial().
#     """
#     master = mavutil.mavlink_connection(f"udp:{ip}:{port}")
#     master.wait_heartbeat(timeout=heartbeat_timeout)
#     print(f"Heartbeat ok (system={master.target_system}, component={master.target_component})")

#     _set_msg_interval(master, 33,  10)   # GLOBAL_POSITION_INT  @ 10 Hz
#     _set_msg_interval(master, 245,  5)   # EXTENDED_SYS_STATE   @  5 Hz
#     _set_msg_interval(master, 74,   5)   # VFR_HUD              @  5 Hz (airspeed live)
#     _set_msg_interval(master, 1,    4)   # SYS_STATUS           @  4 Hz (santé capteurs)
#     _set_msg_interval(master, 30,   2)   # ATTITUDE             @  2 Hz

#     time.sleep(0.3)
#     return master


def check_airspeed_sensor(master, timeout=5.0):
    """
    Vérifie que le capteur pitot est présent, activé et sain via SYS_STATUS.
    À appeler en pré-vol avant le décollage.
    Retourne True si OK, False sinon.
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
        print("   → Vérifier ARSPD_TYPE != 0, ARSPD_USE = 1, câblage pitot")
        return False


def send_gcs_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )
