import ultralytics
from ultralytics import YOLO
import cv2
import numpy as np

# ============================================================================
# === Classe COCO à détecter ===
# ============================================================================
# 0  = person       ← cible humaine (mission réelle)
# 67 = cell phone   ← cible de test indoor
# 32 = sports ball  ← cible de test outdoor
TARGET_CLASS = [0]
CONF_THRESHOLD = 0.4

model = YOLO("yolov8m.pt")

# ============================================================================
# === Source vidéo — Flux WiFi du CosmoStreamer ===
# ============================================================================
# URL RTSP du CosmoStreamer (à valider d'abord avec VLC).
# Format typique (à adapter selon l'IP/port/chemin réels du boîtier) :
#   rtsp://192.168.4.1:8554/live
#   rtsp://192.168.1.1:554/stream
#
# Procédure :
#   1) Connecter le PC au WiFi du CosmoStreamer
#   2) Récupérer l'URL exacte via l'app/page admin du boîtier
#   3) Tester l'URL dans VLC (Média → Ouvrir un flux réseau)
#   4) Coller l'URL validée ci-dessous
SOURCE = "rtsp://192.168.4.1:8554/live"   # ← REMPLACER par l'URL réelle

CAP_WIDTH = 1280
CAP_HEIGHT = 720

latest_detection = None

# Backend FFMPEG forcé : plus fiable que le backend par défaut pour le RTSP
cap = cv2.VideoCapture(SOURCE, cv2.CAP_FFMPEG)

# Buffer réduit à 1 frame : on veut la frame la plus récente, pas une frame en retard
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print(f"  Impossible d'ouvrir le flux : {SOURCE}")
    print("   Vérifier :")
    print("   - PC connecté au WiFi du CosmoStreamer")
    print("   - URL RTSP correcte (tester d'abord avec VLC)")
    print("   - CosmoStreamer alimenté et casque FPV actif")


def normalised_coordinates(frame):
    """Traite une frame et retourne (x, y, w, h, track_id) normalisés.
    Retourne None tant qu'aucune cible n'est verrouillée par ByteTrack."""
    results = model.track(
        frame,
        tracker="bytetrack.yaml",
        persist=True,
        classes=TARGET_CLASS,
        conf=CONF_THRESHOLD,
        verbose=False,
    )
    boxes = results[0].boxes.xywhn
    ids = results[0].boxes.id

    # ByteTrack n'a pas encore verrouillé → on retourne None
    if ids is None or len(boxes) == 0:
        return None

    # Première détection (la plus confiante, YOLO les trie)
    x, y, w, h = boxes[0].tolist()
    track_id = int(ids[0])
    return x, y, w, h, track_id


def get_normalized_coordinates():
    global latest_detection

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erreur de capture vidéo")
            break

        result = normalised_coordinates(frame)
        if result is not None:
            x, y, w, h, track_id = result
            latest_detection = (x, y, w, h, track_id)
            print(f"Track ID: {track_id}, Box: (x={x:.3f}, y={y:.3f}, w={w:.3f}, h={h:.3f})")

        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    get_normalized_coordinates()
