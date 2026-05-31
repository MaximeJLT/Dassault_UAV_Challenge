import numpy as np
from NN import normalised_coordinates
#from connection import connect_udp
from read_gps import get_lat_lon_relalt
import math
from ..read_gps import get_lat_lon_relalt

def GPS_target(master, x, y, w, h, track_id):

    #----------------------------Conversion des coordonnées normalisées en pixels en angle dans le repère caméra------------------------------

    x_centré = x - 0.5
    y_centré = y - 0.5

    # fov a mesurer sur notre camera

    fov_horizontal = 90  # champ de vision horizontal de la caméra en degrés
    fov_vertical = 60    # champ de vision vertical de la caméra en degrés

    angle_horizontal = x_centré * fov_horizontal
    angle_vertical = y_centré * fov_vertical

    #----------------------------Conversion des angles caméra en angle monde-------------------------------------------------------------------

    # Altitude AGL (car pas de LiDAR)
    lat, lon, altitude = get_lat_lon_relalt(master)

    #offset au sol dans le repere drone (trigo)
    offset_right_drone = altitude * math.tan(math.radians(angle_horizontal))
    offset_forward_drone = -altitude * math.tan(math.radian(angle_vertical))

    # Rotation par le yaw UAV pour passer en repère NORD/EST monde
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=5)
    uav_yaw = msg.yaw if msg is not None else 0.0   # en radians, message MAVLink lu donnant roll, pitch et yaw  

    cos_y = math.cos(uav_yaw)
    sin_y = math.sin(uav_yaw)
    delta_nord = offset_forward_drone * cos_y - offset_right_drone * sin_y
    delta_est  = offset_forward_drone * sin_y + offset_right_drone * cos_y

    target_lat = lat + delta_nord / 111320 # conversion approximative de mètres en degrés de latitude
    target_lon = lon + delta_est / (111320 * np.cos(np.radians(lat))) # conversion approximative de mètres en degrés de longitude

    return target_lat, target_lon
