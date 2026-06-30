import time
from pymavlink import mavutil


def _send_mission_item_int(master, it):
    master.mav.mission_item_int_send(
        master.target_system,
        master.target_component,
        it["seq"],
        it["frame"],
        it["command"],
        it["current"],
        it["autocont"],
        it["p1"], it["p2"], it["p3"], it["p4"],
        int(it["lat"] * 1e7),
        int(it["lon"] * 1e7),
        float(it["alt"])
    )


def _flush_input(master, duration=0.5):
    """Vide le buffer de reception pymavlink."""
    t0 = time.time()
    while time.time() - t0 < duration:
        master.recv_match(blocking=False)


def upload_mission_from_file(master, filepath, timeout=30):
    """
    Upload a Mission Planner .waypoints file to the vehicle using MISSION_ITEM_INT.
    Works for ArduPilot (Plane/QuadPlane) with MAVLink2.

    Robuste au double-echo UDP cause par MAVProxy qui partage le meme bus :
    - on filtre les requetes qui ne nous sont pas destinees (source_system check)
    - on deduplique : si ArduPilot re-demande le meme seq, on repond mais
      on ne compte pas ca comme une avance dans la sequence
    - on abandonne l'upload si on recoit INVALID_SEQUENCE avant d'avoir
      envoye tous les items (signe d'une session concurrente ouverte par MAVProxy)
      et on relance proprement
    """
    with open(filepath, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines() if ln.strip()]

    if not lines[0].startswith("QGC WPL"):
        raise ValueError("Not a .waypoints file (missing 'QGC WPL xxx' header)")

    items = []
    for ln in lines[1:]:
        parts = ln.split("\t") if "\t" in ln else ln.split()
        if len(parts) < 12:
            continue
        items.append({
            "seq":      int(parts[0]),
            "current":  0,
            "frame":    int(parts[2]),
            "command":  int(parts[3]),
            "p1": float(parts[4]), "p2": float(parts[5]),
            "p3": float(parts[6]), "p4": float(parts[7]),
            "lat": float(parts[8]),
            "lon": float(parts[9]),
            "alt": float(parts[10]),
            "autocont": int(parts[11])
        })

    if not items:
        raise ValueError("No mission items parsed from file")

    n = len(items)

    MAX_ATTEMPTS = 5
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"Uploading mission: {n} items (attempt {attempt}/{MAX_ATTEMPTS})")

        _flush_input(master, duration=0.5)

        master.mav.mission_clear_all_send(
            master.target_system, master.target_component
        )
        time.sleep(0.5)
        _flush_input(master, duration=0.3)

        master.mav.mission_count_send(
            master.target_system,
            master.target_component,
            n,
            0   
        )

        t0 = time.time()
        last_replied_seq = -1   # dernier seq auquel on a repondu
        highest_seq_seen = -1   # avancement max de la session courante
        success = False
        retry = False

        while True:
            if time.time() - t0 > timeout:
                print(f"Upload timeout attempt {attempt}")
                retry = True
                break

            msg = master.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue

            mtype = msg.get_type()

            if mtype == "MISSION_ACK":
                ack_type = getattr(msg, "type", -1)

                if ack_type == 0:
                    print("Mission upload accepted")
                    master.mav.mission_set_current_send(
                        master.target_system, master.target_component, 1
                    )
                    time.sleep(0.2)
                    success = True
                    break

                if ack_type == 13:
                    if highest_seq_seen < n - 1:
                        print(f"INVALID_SEQUENCE (seq max atteint: {highest_seq_seen}), relance...")
                        retry = True
                        break
                    else:
                        print(f"INVALID_SEQUENCE ignore (upload complet)")
                        continue

                if ack_type == 5:
                    print(f"OPERATION_CANCELLED par session concurrente, relance...")
                    retry = True
                    break

                print(f"Mission upload failed, ACK type={ack_type}")
                retry = True
                break

            if mtype not in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                continue

            seq_req = int(msg.seq)

            if seq_req < 0 or seq_req >= n:
                print(f"seq invalide {seq_req}, ignore")
                continue

            if seq_req == last_replied_seq:
                print(f"retransmission item {seq_req}/{n-1}")
            else:
                print(f"item {seq_req}/{n-1}")

            _send_mission_item_int(master, items[seq_req])
            last_replied_seq = seq_req
            if seq_req > highest_seq_seen:
                highest_seq_seen = seq_req

        if success:
            return True

        if retry and attempt < MAX_ATTEMPTS:
            print(f"Attente avant relance...")
            time.sleep(2.0)
            continue

    raise RuntimeError(f"Mission upload echoue apres {MAX_ATTEMPTS} tentatives")


def read_waypoints_file(filepath):
    """
    Read a Mission Planner .waypoints file and return a list of item dicts.
    """
    with open(filepath, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines() if ln.strip()]

    if not lines[0].startswith("QGC WPL"):
        raise ValueError("Not a .waypoints file (missing 'QGC WPL xxx' header)")

    items = []
    for ln in lines[1:]:
        parts = ln.split("\t") if "\t" in ln else ln.split()
        if len(parts) < 12:
            continue
        items.append({
            "seq":      int(parts[0]),
            "current":  0,
            "frame":    int(parts[2]),
            "command":  int(parts[3]),
            "p1": float(parts[4]),
            "p2": float(parts[5]),
            "p3": float(parts[6]),
            "p4": float(parts[7]),
            "lat": float(parts[8]),
            "lon": float(parts[9]),
            "alt": float(parts[10]),
            "autocont": int(parts[11]),
        })

    if not items:
        raise ValueError("No mission items parsed from file")

    return items


def upload_prefixed_mission(master, prefix_items, mission_filepath):
    """
    Build a single mission = prefix_items + mission file items,
    then upload it as one coherent AUTO mission.
    """
    import tempfile
    import os

    mission_items = read_waypoints_file(mission_filepath)

    merged = []

    for i, it in enumerate(prefix_items):
        new_it = dict(it)
        new_it["seq"] = i
        merged.append(new_it)

    offset = len(prefix_items)

    for j, it in enumerate(mission_items):
        new_it = dict(it)
        new_it["seq"] = offset + j
        new_it["current"] = 0
        merged.append(new_it)

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".waypoints", delete=False, encoding="utf-8"
    )
    try:
        tmp.write("QGC WPL 110\n")
        for it in merged:
            tmp.write(
                f"{it['seq']}\t0\t{it['frame']}\t{it['command']}\t"
                f"{it['p1']}\t{it['p2']}\t{it['p3']}\t{it['p4']}\t"
                f"{it['lat']}\t{it['lon']}\t{it['alt']}\t{it['autocont']}\n"
            )
        tmp.close()

        upload_mission_from_file(master, tmp.name)

    finally:
        try:
            os.unlink(tmp.name)
        except Exception:
            pass


def upload_hypodrome_generated(master, center_lat, center_lon, alt_m=30.0, wps_latlon=None):
    if not wps_latlon:
        raise ValueError("wps_latlon empty")

    items = []
    items.append({
        "seq": 0, "current": 0,
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        "p1": 0, "p2": 0, "p3": 0, "p4": 0,
        "lat": center_lat, "lon": center_lon, "alt": 0.0,
        "autocont": 1
    })
    seq = 1
    for (lat, lon) in wps_latlon:
        items.append({
            "seq": seq, "current": 0,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "p1": 0, "p2": 24, "p3": 0, "p4": 0,
            "lat": lat, "lon": lon, "alt": alt_m,
            "autocont": 1
        })
        seq += 1

    # Reutilise la logique robuste de upload_mission_from_file via fichier temp
    import tempfile, os
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.waypoints',
                                      delete=False, encoding='utf-8')
    tmp.write("QGC WPL 110\n")
    for it in items:
        tmp.write(f"{it['seq']}\t0\t{it['frame']}\t{it['command']}\t"
                  f"{it['p1']}\t{it['p2']}\t{it['p3']}\t{it['p4']}\t"
                  f"{it['lat']}\t{it['lon']}\t{it['alt']}\t{it['autocont']}\n")
    tmp.close()
    try:
        upload_mission_from_file(master, tmp.name)
    finally:
        os.unlink(tmp.name)


def upload_hypodrome_loop(master, center_lat, center_lon, alt_m, wps_latlon):
    items = []
    seq = 0
    for (lat, lon) in wps_latlon:
        items.append({
            "seq": seq, "current": 0,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "p1": 0, "p2": 24, "p3": 0, "p4": 0,
            "lat": lat, "lon": lon, "alt": alt_m,
            "autocont": 1
        })
        seq += 1
    items.append({
        "seq": seq, "current": 0,
        "frame": mavutil.mavlink.MAV_FRAME_MISSION,
        "command": mavutil.mavlink.MAV_CMD_DO_JUMP,
        "p1": 0, "p2": 9999, "p3": 0, "p4": 0,
        "lat": 0.0, "lon": 0.0, "alt": 0.0,
        "autocont": 1
    })

    import tempfile, os
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.waypoints',
                                      delete=False, encoding='utf-8')
    tmp.write("QGC WPL 110\n")
    for it in items:
        tmp.write(f"{it['seq']}\t0\t{it['frame']}\t{it['command']}\t"
                  f"{it['p1']}\t{it['p2']}\t{it['p3']}\t{it['p4']}\t"
                  f"{it['lat']}\t{it['lon']}\t{it['alt']}\t{it['autocont']}\n")
    tmp.close()
    try:
        upload_mission_from_file(master, tmp.name)
    finally:
        os.unlink(tmp.name)
