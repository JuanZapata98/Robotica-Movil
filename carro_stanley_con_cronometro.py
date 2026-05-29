import argparse
import csv
import logging
import math
import os
import subprocess
import time
import sys

from carla.client import make_carla_client
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.client import VehicleControl

HOST = "localhost"
PORT = 2000

WHEELBASE = 2.7

STANLEY_GAIN = 1.5
TARGET_SPEED = 70.0         # km/h
SPEED_KP    = 0.8
SPEED_KI    = 0.05
MAX_STEER   = 0.6           # rad, ángulo máximo de dirección del vehículo
MIN_SPEED_FOR_STANLEY = 0.1


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# =============================================================================
# CRONÓMETRO DE VUELTA
# =============================================================================

# Coordenadas de la línea de meta en RaceTrack
META_Y = -9.0
META_Y_TOL = 2.0
META_X_MIN = -200.0
META_X_MAX = -160.0
LAP_MIN_DIST_M = 200.0


class LapTimer:
    def __init__(self):
        self._lap_start = None
        self._lap_count = 0
        self._dist_acum = 0.0
        self._last_x = None
        self._last_y = None
        self._in_trigger = False
        self._started = False
        self._elapsed = 0.0
        self.lap_times = []

    def _on_meta_cross(self):
        now = time.perf_counter()
        if not self._started:
            self._lap_start = now
            self._dist_acum = 0.0
            self._started = True
            print('\n[LAP] Cronómetro iniciado.')
            return

        elapsed = now - self._lap_start
        self._lap_count += 1
        self.lap_times.append(elapsed)
        print('\n[LAP {}] {:.3f} s  ({}:{:04.1f})'.format(
            self._lap_count, elapsed,
            int(elapsed // 60), elapsed % 60))
        self._lap_start = now
        self._dist_acum = 0.0

    def update(self, x, y):
        if self._last_x is not None:
            dx = x - self._last_x
            dy = y - self._last_y
            self._dist_acum += math.sqrt(dx * dx + dy * dy)
        self._last_x = x
        self._last_y = y

        if self._lap_start is not None:
            self._elapsed = time.perf_counter() - self._lap_start

        in_meta = (abs(y - META_Y) <= META_Y_TOL) and (META_X_MIN <= x <= META_X_MAX)

        if in_meta and not self._in_trigger:
            if not self._started or self._dist_acum >= LAP_MIN_DIST_M:
                self._on_meta_cross()
            self._in_trigger = True
        elif not in_meta:
            self._in_trigger = False

    def get_lap_count(self):
        return self._lap_count

    def get_elapsed(self):
        return self._elapsed

    def get_last_times(self, n=3):
        return self.lap_times[-n:]

    def is_started(self):
        return self._started


def render_hud(lap_timer, speed_kmh, closest_idx, x, y):
    if not lap_timer.is_started():
        status = 'Esperando meta  Y={:.1f} (actual:{:.1f})'.format(META_Y, y)
    else:
        status = 'V:{:<2}  Parcial:{:7.2f}s'.format(
            lap_timer.get_lap_count() + 1, lap_timer.get_elapsed())

    lasts = lap_timer.get_last_times(3)
    times_str = '  '.join('L{}:{:.2f}s'.format(i + 1, t)
                          for i, t in enumerate(lasts)) if lasts else '--'

    line = '\r[HUD] {}  {:6.1f}km/h  X:{:8.1f} Y:{:7.1f}  WP:{:04d}  | {}'.format(
        status, speed_kmh, x, y, closest_idx, times_str)
    sys.stdout.write('{:<140}'.format(line))
    sys.stdout.flush()



def launch_carla():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    carla_path = os.path.abspath(os.path.join(current_dir, "..", "CarlaUE4.exe"))
    print("Ruta CARLA:", carla_path)
    subprocess.Popen([
        carla_path,
        "/Game/Maps/RaceTrack",
        "-windowed", "-carla-server", "-benchmark",
        "-fps=15", "-ResX=800", "-ResY=450", "-quality-level=Low",
    ])
    print("Abriendo CARLA...")
    time.sleep(20)


def load_waypoints(csv_path):
    waypoints = []
    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or "x" not in reader.fieldnames or "y" not in reader.fieldnames:
            raise ValueError("El CSV de waypoints debe tener columnas x,y")
        for row in reader:
            try:
                waypoints.append((float(row["x"]), float(row["y"])))
            except (ValueError, KeyError):
                continue
    if len(waypoints) < 2:
        raise ValueError("Se necesitan al menos 2 waypoints validos")
    return waypoints


def closest_waypoint_index(x, y, waypoints):
    best_i, best_d = 0, float("inf")
    for i, (wx, wy) in enumerate(waypoints):
        d = (wx - x) ** 2 + (wy - y) ** 2
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def path_heading(waypoints, i):
    if i >= len(waypoints) - 1:
        i = len(waypoints) - 2
    x1, y1 = waypoints[i]
    x2, y2 = waypoints[i + 1]
    return math.atan2(y2 - y1, x2 - x1)


def stanley_control(x, y, yaw, speed, waypoints):
    front_x = x + (WHEELBASE / 2.0) * math.cos(yaw)
    front_y = y + (WHEELBASE / 2.0) * math.sin(yaw)

    closest_idx = closest_waypoint_index(front_x, front_y, waypoints)

    # Stanley usa el mismo punto de referencia para heading y cte.
    yaw_ref = path_heading(waypoints, closest_idx)
    wx, wy = waypoints[closest_idx]

    heading_error = wrap_to_pi(yaw_ref - yaw)

    dx = front_x - wx
    dy = front_y - wy
    cross_track_error = dx * math.sin(yaw_ref) - dy * math.cos(yaw_ref)

    v = max(speed, MIN_SPEED_FOR_STANLEY)
    steer_rad = heading_error + math.atan2(STANLEY_GAIN * cross_track_error, v)
    steer_rad = max(-MAX_STEER, min(MAX_STEER, steer_rad))

    steer_normalized = steer_rad / MAX_STEER
    return steer_normalized, cross_track_error, heading_error, closest_idx


def run_carla_client(waypoints_file):
    waypoints = load_waypoints(waypoints_file)
    print(f"Waypoints cargados: {len(waypoints)}")
    print("Primeros puntos:", waypoints[:3])

    target_speed_ms = TARGET_SPEED / 3.6   # m/s
    lap_timer = LapTimer()

    with make_carla_client(HOST, PORT) as client:
        settings = CarlaSettings()
        settings.set(
            SynchronousMode=True,
            SendNonPlayerAgentsInfo=False,
            NumberOfVehicles=0,
            NumberOfPedestrians=0,
        )
        client.load_settings(settings)
        client.start_episode(0)
        print("Stanley activo. CTRL+C para detener.")

        speed_integral = 0.0   # acumulador del termino integral de velocidad

        while True:
            try:
                measurements, _ = client.read_data()
            except Exception as e:
                logging.warning("Stream TCP corrupto, reiniciando episodio: %s", e)
                client.start_episode(0)
                speed_integral = 0.0
                continue

            pm = measurements.player_measurements
            transform = pm.transform

            x     = transform.location.x
            y     = transform.location.y
            yaw   = math.radians(transform.rotation.yaw)
            speed = pm.forward_speed   # m/s

            steer, cte, he, closest_idx = stanley_control(x, y, yaw, speed, waypoints)

            # Controlador PI de velocidad con anti-windup
            speed_error    = target_speed_ms - speed
            speed_integral = max(-20.0, min(20.0, speed_integral + speed_error))
            throttle_raw   = SPEED_KP * speed_error + SPEED_KI * speed_integral
            throttle       = max(0.0, min(1.0, throttle_raw))
            throttle      *= max(0.25, 1.0 - abs(steer))

            control            = VehicleControl()
            control.throttle   = throttle
            control.steer      = steer
            control.brake      = 0.0
            control.hand_brake = False
            control.reverse    = False
            client.send_control(control)

            lap_timer.update(x, y)
            render_hud(lap_timer, speed * 3.6, closest_idx, x, y)


def main():
    parser = argparse.ArgumentParser(description="CARLA 0.8 Stanley controller")
    parser.add_argument("--waypoints-file", default="waypoints.csv",
                        help="CSV con columnas x,y (por defecto: waypoints.csv)")
    parser.add_argument("--no-launch", action="store_true",
                        help="No abrir CARLA automaticamente")
    args = parser.parse_args()

    current_dir    = os.path.dirname(os.path.abspath(__file__))
    waypoints_file = args.waypoints_file
    if not os.path.isabs(waypoints_file):
        waypoints_file = os.path.join(current_dir, waypoints_file)

    if not args.no_launch:
        launch_carla()
    else:
        print("Se omitio el lanzamiento automatico de CARLA.")

    try:
        run_carla_client(waypoints_file)
    except KeyboardInterrupt:
        print("\nPrograma detenido")
    except TCPConnectionError as error:
        print(error)


if __name__ == "__main__":
    main()
