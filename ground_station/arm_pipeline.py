from pymavlink import mavutil
import time

from read_gps import get_lat_lon_relalt
from connection import send_gcs_heartbeat


MAV_CMD_DO_VTOL_TRANSITION = 3000
MAV_VTOL_STATE_MC = 3
MAV_VTOL_STATE_FW = 4

VTOL_STATE_NAME = {
    0: "UNDEFINED",
    1: "TRANSITION_TO_FW",
    2: "TRANSITION_TO_MC",
    3: "MC",
    4: "FW",
}

def _gcs_keepalive_tick(master, last_hb, period_s=1.0):
    """Send GCS heartbeat every period_s seconds. Returns updated last_hb."""
    now = time.time()
    if now - last_hb >= period_s:
        send_gcs_heartbeat(master)
        return now
    return last_hb


class _RCThrottleKeepAlive:
    """
    Envoie rc_channels_override (throttle 1500) toutes les 100 ms dans un
    thread de fond, pour maintenir l'altitude en QLOITER pendant les op�rations
    bloquantes (configure_failsafes, mission_upload\u2026).

    Usage :
        with _RCThrottleKeepAlive(master):
            configure_failsafes_for_flight(master)
            upload_mission_from_file(master, "...")
        # � la sortie du with, l'override est rel�ch� proprement
    """
    def __init__(self, master, interval_s=0.1):
        self._master   = master
        self._interval = interval_s
        self._stop     = False
        self._thread   = None

    def _run(self):
        IGN = 65535
        while not self._stop:
            try:
                self._master.mav.rc_channels_override_send(
                    self._master.target_system,
                    self._master.target_component,
                    1500, 1500, 1500, 1500,
                    IGN, IGN, IGN, IGN
                )
            except Exception:
                pass
            time.sleep(self._interval)

    def __enter__(self):
        self._stop   = False
        self._thread = __import__("threading").Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *_):
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        # Rel�cher proprement tous les overrides
        try:
            self._master.mav.rc_channels_override_send(
                self._master.target_system,
                self._master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
        except Exception:
            pass


def _drain_statustext(master, n=10):
    """Print a few queued STATUSTEXT messages (non-blocking)."""
    for _ in range(n):
        st = master.recv_match(type="STATUSTEXT", blocking=False)
        if not st:
            break
        txt = st.text
        if isinstance(txt, (bytes, bytearray)):
            txt = txt.decode(errors="ignore")
        print("STATUSTEXT:", txt)


def _set_param(master, name: str, value: float,
               ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32, timeout=3.0):
    """
    Envoie un PARAM_SET et attend l'\ufffdcho PARAM_VALUE.
    Le type MAVLink doit correspondre au type r\ufffdel du param\ufffdtre ArduPlane :
      - REAL32  pour les floats  (FS_EKF_THRESH, etc.)
      - INT8    pour les entiers courts (FS_EKF_ACTION, FS_GCS_ENABL, etc.)
    Un type incorrect cause un NO_ECHO silencieux c\ufffdt\ufffd ArduPilot.
    """
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode("ascii"), float(value), ptype
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg:
            pid = msg.param_id
            if isinstance(pid, (bytes, bytearray)):
                pid = pid.decode(errors="ignore")
            pid = str(pid).strip("\x00")
            if pid == name:
                return True
    return False


def _read_param_float(master, name: str, timeout=3.0):
    """
    Lit la valeur courante d'un param\ufffdtre via PARAM_REQUEST_READ.
    Retourne la valeur float, ou None si timeout.
    """
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode("ascii"), -1
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg:
            pid = msg.param_id
            if isinstance(pid, (bytes, bytearray)):
                pid = pid.decode(errors="ignore")
            pid = str(pid).strip("\x00")
            if pid == name:
                return float(msg.param_value)
    return None


def configure_failsafes_for_flight(master):
    """
    Configure les failsafes ArduPlane pour un vol r\ufffdel QuadPlane.
    Noms et types de param\ufffdtres corrects pour ArduPlane 4.x.

    Types MAVLink importants :
      INT8  \u2192 param\ufffdtres entiers courts (actions, flags)
      REAL32 \u2192 param\ufffdtres flottants (seuils)

    Un type incorrect \u2192 ArduPilot ignore le PARAM_SET sans r\ufffdpondre (NO_ECHO).
    """
    print("Configuring failsafes for real flight (ArduPlane QuadPlane)")

    INT8   = mavutil.mavlink.MAV_PARAM_TYPE_INT8
    REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    candidates = [
        # (nom, valeur, type_mavlink)
        ("FS_GCS_ENABL",  1,   INT8),    # 1 = RTL si GCS perdu
        ("THR_FAILSAFE",  1,   INT8),    # 1 = enabled RC throttle failsafe
        ("FS_EKF_ACTION", 2,   INT8),    # 2 = RTL si EKF d\ufffdrive  \u2190 INT8, pas REAL32
        ("FS_EKF_THRESH", 0.8, REAL32),  # seuil variance EKF
        ("FS_LONG_ACTN",  1,   INT8),    # 1 = RTL sur perte RC longue
        ("FS_SHORT_ACTN", 0,   INT8),    # 0 = FBWA sur perte RC courte
        ("RTL_AUTOLAND",  1,   INT8),    # 1 = auto-land apr\ufffds RTL
    ]

    # --- VTOL speed limits (tracking / loiter) ---
    print("Setting VTOL navigation speeds...")

    _set_param(master, "Q_WP_SPEED",    350, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    _set_param(master, "Q_LOIT_SPEED",  350, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    # --- Vitesse de croisi�re FW : limiter � 13 m/s max pour prot�ger la structure ---
    # Q_TRANSITION_MS : vitesse � laquelle ArduPlane consid�re la transition FW termin�e.
    #   D�faut ArduPlane = 0 (utilise AIRSPEED_MIN ~10 m/s) mais souvent ~18 m/s en pratique.
    #   On le force � 12 m/s \u2192 la transition FW est d�clar�e compl�te � 12 m/s.
    # AIRSPEED_CRUISE : vitesse de croisi�re cible en AUTO (cm/s).
    #   1300 cm/s = 13 m/s
    # AIRSPEED_MAX : plafond absolu de la r�gulation vitesse air.
    #   15 m/s \u2192 ArduPlane ne d�passera pas 15 m/s m�me avec du vent.
    print("Setting FW airspeed limits (max 15 m/s for structural safety)...")
    _set_param(master, "AIRSPEED_CRUISE", 13.0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    _set_param(master, "AIRSPEED_MAX",    15.0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    _set_param(master, "AIRSPEED_MIN",    10.0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    _set_param(master, "Q_TRANSITION_MS", 12.0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    _set_param(master, "Q_ASSIST_SPEED", 12.0, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    for name, value, ptype in candidates:
        ok = _set_param(master, name, float(value), ptype=ptype, timeout=3.0)
        print(f"   - {name} = {value}  ({'OK' if ok else 'NO_ECHO \u2013 verify manually'})")


def check_arspd_use(master):
    """
    V\ufffdrifie que ARSPD_USE=1 (pitot activ\ufffd pour le contr\ufffdle).
    Si =0, DO_CHANGE_SPEED sera rejet\ufffd m\ufffdme avec un pitot sain.
    Retourne True si OK, False sinon.
    """
    val = _read_param_float(master, "ARSPD_USE", timeout=3.0)
    if val is None:
        print("\u26a0\ufe0f  ARSPD_USE : impossible \ufffd lire (timeout)")
        return False
    if int(val) == 1:
        print(f"\u2705 ARSPD_USE = {int(val)} (pitot utilis\ufffd pour le contr\ufffdle)")
        return True
    else:
        print(f"\u26a0\ufe0f  ARSPD_USE = {int(val)} \u2013 DO_CHANGE_SPEED sera rejet\ufffd !")
        print("   \u2192 Mettre ARSPD_USE=1 dans Mission Planner ou via MAVProxy")
        return False


def send_rc_neutral(master):
    IGN = 65535
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, IGN, 1500,
        IGN, IGN, IGN, IGN
    )

def send_rc_qloiter_hold(master):
    IGN = 65535
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, 1500, 1500,
        IGN, IGN, IGN, IGN
    )


def release_rc_override(master):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0,0,0,0,0,0,0,0
    )


def set_mode_and_confirm(master, mode_name: str, timeout=15):
    modes = master.mode_mapping()
    if mode_name not in modes:
        raise RuntimeError(f"Mode {mode_name} not available. Available: {list(modes.keys())}")

    master.set_mode(modes[mode_name])
    print(f"\U0001f7e1 {mode_name} requested")

    t0 = time.time()
    last_hb = 0.0
    while time.time() - t0 < timeout:
        if time.time() - last_hb > 1.0:
            send_gcs_heartbeat(master)
            last_hb = time.time()

        if mode_name == "AUTO":
            send_rc_neutral(master)

        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        _drain_statustext(master, n=10)

        if hb:
            try:
                print(f"DEBUG flightmode={master.flightmode} custom_mode={hb.custom_mode}")
            except Exception:
                pass
            if hb.custom_mode == modes[mode_name]:
                print(f"\u2705 Mode {mode_name} confirmed")
                return True

    raise RuntimeError(f"{mode_name} timeout (mode not confirmed)")


def request_airspeed(master, airspeed_mps: float):
    """
    Demande une vitesse air cible en AUTO.
    N\ufffdcessite ARSPD_USE=1 et un capteur pitot fonctionnel.
    result=4 (UNSUPPORTED) \u2192 ARSPD_USE=0, corriger le param\ufffdtre.
    """
    print(f"\U0001f7e1 Airspeed request: {airspeed_mps:.1f} m/s")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        0, float(airspeed_mps), -1, 0, 0, 0, 0
    )
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=2.0)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
        if ack.result == 0:
            print("\u2705 Airspeed command accepted")
        else:
            print(f"\u26a0\ufe0f  Airspeed command result={ack.result} \u2013 check ARSPD_USE=1 and pitot sensor")


def transition_vtol_to_fw(master, timeout=60):
    """
    Request VTOL (MC) -> FW and wait until vtol_state == FW.
    Le v\ufffdhicule doit avoir un mode de vol avant actif (AUTO/CRUISE) pour acc\ufffdl\ufffdrer.
    """
    print("\U0001f7e1 VTOL->FW transition requested")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        MAV_CMD_DO_VTOL_TRANSITION, 0,
        float(MAV_VTOL_STATE_FW),
        0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=1)
        if msg:
            last = int(msg.vtol_state)
            print(f"DEBUG vtol_state = {last} ({VTOL_STATE_NAME.get(last, '???')})")
            if last == MAV_VTOL_STATE_FW:
                print("\u2705 VTOL_STATE_FW confirmed")
                return True
        _drain_statustext(master, n=5)

    print(f"DEBUG last vtol_state = {last}")
    raise RuntimeError("Transition timeout (still no VTOL_STATE_FW)")


def pipeline_quadplane_vtol_takeoff_to_auto(master, target_alt=30.0, airspeed_mps=14.0):
    """
    Décollage VTOL + transition FW + lancement AUTO sur hypodrome.
    100 % AUTO du décollage à la fin — aucun passage en QLOITER, aucun RC override.

    Stratégie :
      1. Attendre position GPS valide
      2. Configurer failsafes + vitesses AVANT armement (drone au sol, aucun risque)
      3. Uploader mission complète = [ HOME + NAV_VTOL_TAKEOFF + hypodrome.waypoints ]
      4. Passer en AUTO → armer → le drone décolle et enchaîne directement la mission
      5. Attendre altitude cible, puis transition VTOL→FW en AUTO
    """
    lat, lon, alt = get_lat_lon_relalt(master, timeout=10)
    print(f"\U0001f4e1 Current position: lat={lat}, lon={lon}, rel_alt={alt} m")

    modes = master.mode_mapping()
    if "AUTO" not in modes:
        raise RuntimeError("AUTO not available")

    # --- Attente position GPS ---
    print("\U0001f7e1 Waiting for position estimate...")
    good = 0
    t0 = time.time()
    last_hb = 0.0
    while time.time() - t0 < 15:
        last_hb = _gcs_keepalive_tick(master, last_hb, period_s=1.0)
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg:
            good += 1
        if good >= 5:
            print("\u2705 Position estimate OK")
            break
    if good < 5:
        raise RuntimeError("Position estimate timeout")

    # --- Failsafes + vitesses AU SOL (avant armement = pas de risque de chute) ---
    print("\U0001f7e1 Configuring failsafes and speeds (on ground, before arming)...")
    configure_failsafes_for_flight(master)
    time.sleep(0.5)

    # --- Upload mission complète : décollage VTOL + hypodrome ---
    # On uploade TOUT avant d'armer : pas besoin de changer de mode
    # ou d'envoyer du RC override pendant le vol.
    cur_lat, cur_lon, _ = get_lat_lon_relalt(master, timeout=5)

    takeoff_prefix = [
        # seq=0 : HOME (obligatoire, alt=0 absolu)
        dict(seq=0, frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
             command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
             current=0, autocont=1,
             p1=0, p2=0, p3=0, p4=0,
             lat=cur_lat, lon=cur_lon, alt=0.0),
        # seq=1 : décollage VTOL vertical jusqu'à target_alt
        dict(seq=1, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             command=mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
             current=1, autocont=1,
             p1=0, p2=0, p3=0, p4=float('nan'),
             lat=cur_lat, lon=cur_lon, alt=float(target_alt)),
    ]

    print("\U0001f7e1 Uploading full AUTO mission (takeoff + hypodrome)...")
    from mission_upload import upload_prefixed_mission
    upload_prefixed_mission(master, takeoff_prefix, "hypodrome.waypoints")
    print("\u2705 Mission uploaded")

    print("Dumping STATUSTEXT...")
    _drain_statustext(master, n=50)

    # --- Passer en AUTO AVANT d'armer ---
    set_mode_and_confirm(master, "AUTO", timeout=15)

    # --- Armement ---
    master.arducopter_arm()
    print("\U0001f7e1 Arming requested")
    t0 = time.time()
    while time.time() - t0 < 15:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("\u2705 UAV armed")
            break
    else:
        raise RuntimeError("Arming timeout")

    # --- Attente altitude cible (le drone décolle seul en AUTO) ---
    print("\U0001f7e1 Waiting VTOL altitude...")
    t0 = time.time()
    last_hb = 0.0
    last_log = 0.0
    while time.time() - t0 < 120:
        last_hb = _gcs_keepalive_tick(master, last_hb, period_s=1.0)
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if not msg:
            continue
        rel_alt_m = msg.relative_alt / 1000.0
        if time.time() - last_log > 3.0:
            print(f"   alt={rel_alt_m:.1f}m / {target_alt:.1f}m")
            last_log = time.time()
        if rel_alt_m >= target_alt - 0.5:
            print(f"\u2705 VTOL altitude reached (~{rel_alt_m:.2f} m)")
            break
    else:
        raise RuntimeError("VTOL altitude timeout")

    # --- Transition VTOL→FW en AUTO ---
    request_airspeed(master, min(airspeed_mps, 13.0))
    time.sleep(1.0)
    transition_vtol_to_fw(master, timeout=90)

    print("\u2705 Pipeline OK: AUTO running + FW state confirmed")
    return True


def transition_fw_to_vtol(master, timeout=60):
    print("\U0001f7e1 FW->VTOL transition requested")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        MAV_CMD_DO_VTOL_TRANSITION, 0,
        float(MAV_VTOL_STATE_MC), 0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=1)
        if msg and int(msg.vtol_state) == MAV_VTOL_STATE_MC:
            print("\u2705 VTOL_STATE_MC confirmed")
            return True
    raise RuntimeError("FW->VTOL transition timeout")

#Kill switch
def emergency_kill(master):
    """
    Kill switch r�glementaire : d�sarmement forc� en vol.
    Envoie MAV_CMD_COMPONENT_ARM_DISARM avec force flag = 21196.
    Le magic number 21196 est le code ArduPilot pour forcer le d�sarmement en vol.
    """
    print("\U0001f6a8 KILL SWITCH ACTIVATED \u2013 FORCE DISARM")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,        # 0 = disarm
        21196.0,  # magic number ArduPilot = force disarm in flight
        0, 0, 0, 0, 0
    )
