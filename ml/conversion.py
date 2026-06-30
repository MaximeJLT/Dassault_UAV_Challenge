import numpy as np
from NN import normalised_coordinates
from read_gps import get_lat_lon_relalt
import math
from ..read_gps import get_lat_lon_relalt

def GPS_target(master, x, y, w, h, track_id):


    x_centré = x - 0.5
    y_centré = y - 0.5


    fov_horizontal = 90  
    fov_vertical = 60    

    angle_horizontal = x_centré * fov_horizontal
    angle_vertical = y_centré * fov_vertical

    lat, lon, altitude = get_lat_lon_relalt(master)

    offset_right_drone = altitude * math.tan(math.radians(angle_horizontal))
    offset_forward_drone = -altitude * math.tan(math.radian(angle_vertical))

    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=5)
    uav_yaw = msg.yaw if msg is not None else 0.0  

    cos_y = math.cos(uav_yaw)
    sin_y = math.sin(uav_yaw)
    delta_nord = offset_forward_drone * cos_y - offset_right_drone * sin_y
    delta_est  = offset_forward_drone * sin_y + offset_right_drone * cos_y

    target_lat = lat + delta_nord / 111320 
    target_lon = lon + delta_est / (111320 * np.cos(np.radians(lat))) 

    return target_lat, target_lon
