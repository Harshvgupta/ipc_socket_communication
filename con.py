# #!/usr/bin/env python3
# """
# consumer.py

# A Unix-domain IPC consumer that connects to the publisher’s socket, receives IMU payloads,
# parses each CSV line of 12 values, computes orientation (roll, pitch, yaw, quaternion), and optionally
# writes a rolling CSV of orientations to disk. This version adds:

#   1. No forced clamping of pitch. Instead, if pitch → ±90°, we still compute but issue a warning.
#   2. Detection of HEARTBEAT packets and logging them.
#   3. Detection of malformed lines (including “MALFORMED_DATA”) without crashing.
#   4. CLI‐option: --output-csv <path> to write "timestamp_ms,roll_deg,pitch_deg,yaw_deg" in append mode.
#   5. RotatingFileHandler + console logging, similar to publisher.
#   6. Reconnect logic, timeout handling, and unit tests that simulate random/malformed data via a mock publisher socket.

# Usage:
#     python3 consumer.py \
#       --socket-path /tmp/imu_socket \
#       --timeout-ms 100 \
#       --log-dir /var/log/imu_consumer \
#       --log-level INFO \
#       --output-csv /tmp/imu_orientations.csv

# CLI Options:
#   --socket-path PATH
#       Path to the Unix-domain socket (e.g. /tmp/imu_socket).
#   --timeout-ms M
#       Milliseconds to wait for data before timing out (default=100).
#   --log-dir DIR
#       Directory for file logs; if omitted, logs only to console.
#   --log-level {DEBUG,INFO,WARNING,ERROR,CRITICAL}
#       Logging verbosity (default=INFO).
#   --output-csv PATH
#       If provided, append "timestamp_ms,roll_deg,pitch_deg,yaw_deg" to this CSV after each valid IMU parse.

# Requirements:
#   - Debian/Ubuntu or any Linux.
#   - Python 3.6+
#   - Standard library only.
# """

# import argparse
# import logging
# import logging.handlers
# import os
# import socket
# import sys
# import math
# import time
# from typing import Optional, Tuple

# # =============================================================================
# # Helper: Orientation computation
# # =============================================================================
# def compute_roll_pitch_from_accel(x_acc: float, y_acc: float, z_acc: float) -> Tuple[float, float]:
#     """
#     Compute roll & pitch (radians) from accelerometer:
#       roll  = atan2(y_acc, z_acc)
#       pitch = atan2(-x_acc, sqrt(y_acc^2 + z_acc^2))
#     We do NOT clamp pitch here; if denominator = 0, we return pitch=±pi/2.

#     Args:
#         x_acc (float): Accelerometer X in mg.
#         y_acc (float): Accelerometer Y in mg.
#         z_acc (float): Accelerometer Z in mg.

#     Returns:
#         (roll, pitch) in radians.
#     """
#     # roll = atan2(Y, Z). If Z≈0 & Y≈0, roll≈0
#     if y_acc == 0.0 and z_acc == 0.0:
#         roll = 0.0
#     else:
#         roll = math.atan2(y_acc, z_acc)

#     denom = math.sqrt(y_acc * y_acc + z_acc * z_acc)
#     if denom == 0.0:
#         # If denom=0, x_acc big → pitch=±pi/2 depending on x_acc sign
#         pitch = math.copysign(math.pi / 2.0, -x_acc)
#     else:
#         pitch = math.atan2(-x_acc, denom)

#     return roll, pitch


# def compute_yaw_from_mag(x_mag: float, y_mag: float, z_mag: float,
#                          roll: float, pitch: float) -> float:
#     """
#     Compute yaw (radians) from magnetometer + current roll/pitch.
#     Tilt compensation formula:
#         mag_xc = x_mag * cos(pitch) + z_mag * sin(pitch)
#         mag_yc = x_mag * sin(roll)*sin(pitch) + y_mag * cos(roll) - z_mag * sin(roll)*cos(pitch)
#         yaw = atan2(-mag_yc, mag_xc)

#     Args:
#         x_mag, y_mag, z_mag (float): Magnetometer readings in mGauss.
#         roll (float): Roll in radians.
#         pitch (float): Pitch in radians.

#     Returns:
#         yaw (float) in radians.
#     """
#     cos_r = math.cos(roll)
#     sin_r = math.sin(roll)
#     cos_p = math.cos(pitch)
#     sin_p = math.sin(pitch)

#     mag_xc = x_mag * cos_p + z_mag * sin_p
#     mag_yc = x_mag * sin_r * sin_p + y_mag * cos_r - z_mag * sin_r * cos_p
#     yaw = math.atan2(-mag_yc, mag_xc)
#     return yaw


# def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
#     """
#     Convert Euler angles (roll, pitch, yaw) → quaternion (qw, qx, qy, qz).
#     Reference: Wikipedia “Conversion between quaternions and Euler angles”.

#     Args:
#         roll, pitch, yaw (radians)

#     Returns:
#         (qw, qx, qy, qz)
#     """
#     cy = math.cos(yaw * 0.5)
#     sy = math.sin(yaw * 0.5)
#     cp = math.cos(pitch * 0.5)
#     sp = math.sin(pitch * 0.5)
#     cr = math.cos(roll * 0.5)
#     sr = math.sin(roll * 0.5)

#     qw = cr * cp * cy + sr * sp * sy
#     qx = sr * cp * cy - cr * sp * sy
#     qy = cr * sp * cy + sr * cp * sy
#     qz = cr * cp * sy - sr * sp * cy

#     return qw, qx, qy, qz


# # =============================================================================
# # Main Consumer Class
# # =============================================================================
# class IMUConsumer:
#     """
#     IMUConsumer connects to a Unix-domain socket, receives lines of CSV (or special tags),
#     parses valid 12-value IMU payloads, computes orientation, optionally writes CSV of
#     (timestamp, roll_deg, pitch_deg, yaw_deg), and logs everything. It:
#       1. Handles HEARTBEAT packets (logs them at DEBUG).
#       2. Skips malformed or non‐CSV lines.
#       3. Does NOT forcibly clamp pitch; instead warns if |pitch|→90° (gimbal‐lock danger).
#       4. Retries connection if socket disappears or times out.
#       5. Logs both to console and to a rotating file if --log-dir is specified.
#     """

#     RECONNECT_DELAY_SEC = 1.0  # seconds before retrying to connect

#     def __init__(self,
#                  socket_path: str,
#                  timeout_ms: int,
#                  output_csv: Optional[str],
#                  log_dir: str,
#                  log_level: str):
#         """
#         Args:
#             socket_path (str): Path to Unix-domain socket (publisher’s).
#             timeout_ms (int): Milliseconds to wait for data before timeout.
#             output_csv (str|None): If set, path to append "timestamp_ms,roll_deg,pitch_deg,yaw_deg".
#             log_dir (str): Directory for rotated logs. If empty, console only.
#             log_level (str): DEBUG/INFO/WARNING/ERROR/CRITICAL.
#         """
#         self.socket_path = socket_path
#         self.timeout_ms = timeout_ms
#         self.output_csv = output_csv or ""
#         self.log_dir = log_dir.rstrip("/") if log_dir else ""
#         self.log_level = log_level.upper()
#         self.logger = self._setup_logging()
#         self.sock: Optional[socket.socket] = None
#         self.fileobj = None  # file‐like wrapper around sock
#         self._first_csv_write = True  # to write header once

#         # Pre‐create output CSV if needed
#         if self.output_csv:
#             try:
#                 # If file does not exist, create and write header
#                 if not os.path.exists(self.output_csv):
#                     with open(self.output_csv, "w") as f:
#                         f.write("timestamp_ms,roll_deg,pitch_deg,yaw_deg\n")
#                 else:
#                     # If the file already exists, assume it has a header
#                     pass
#             except Exception as e:
#                 raise IOError(f"Cannot create/write to output CSV {self.output_csv}: {e}")

#     def _setup_logging(self) -> logging.Logger:
#         """
#         Configure a logger with console + optional rotating file handler.
#         """
#         logger = logging.getLogger("IMUConsumer")
#         level = getattr(logging, self.log_level, logging.INFO)
#         logger.setLevel(level)

#         # Console handler
#         console_h = logging.StreamHandler(sys.stdout)
#         console_h.setLevel(level)
#         fmt = "[%(asctime)s] [%(levelname)s] %(name)s: %(message)s"
#         datefmt = "%Y-%m-%d %H:%M:%S"
#         formatter = logging.Formatter(fmt, datefmt=datefmt)
#         console_h.setFormatter(formatter)
#         logger.addHandler(console_h)

#         # File handler if log_dir provided
#         if self.log_dir:
#             os.makedirs(self.log_dir, exist_ok=True)
#             log_path = os.path.join(self.log_dir, "imu_consumer.log")
#             file_h = logging.handlers.RotatingFileHandler(
#                 log_path, maxBytes=10 * 1024 * 1024, backupCount=5
#             )
#             file_h.setLevel(level)
#             file_h.setFormatter(formatter)
#             logger.addHandler(file_h)

#         return logger

#     def _connect(self):
#         """
#         Attempt to connect to the Unix-domain socket. On success, wrap sock in fileobj.
#         Retries indefinitely (every RECONNECT_DELAY_SEC) if either the socket file doesn't exist
#         or connection is refused.
#         """
#         while True:
#             if not os.path.exists(self.socket_path):
#                 self.logger.warning(f"Socket '{self.socket_path}' not found. Retrying in {self.RECONNECT_DELAY_SEC}s...")
#                 time.sleep(self.RECONNECT_DELAY_SEC)
#                 continue

#             try:
#                 self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
#                 self.sock.settimeout(self.timeout_ms / 1000.0)
#                 self.sock.connect(self.socket_path)
#                 # Turn it into a file‐like object for readline()
#                 self.fileobj = self.sock.makefile("r")
#                 self.logger.info(f"Connected to publisher socket '{self.socket_path}' (timeout={self.timeout_ms}ms).")
#                 break
#             except (ConnectionRefusedError, FileNotFoundError) as e:
#                 self.logger.warning(f"Connection issue: {e}. Retrying in {self.RECONNECT_DELAY_SEC}s...")
#                 time.sleep(self.RECONNECT_DELAY_SEC)
#             except Exception as e:
#                 self.logger.error(f"Unexpected connect error: {e}")
#                 time.sleep(self.RECONNECT_DELAY_SEC)

#     def run(self):
#         """
#         Main loop:
#           1. If not connected, call _connect()
#           2. Attempt to read one line via self.fileobj.readline()
#              - If empty string: publisher closed connection → cleanup & reconnect
#              - If timeout: log warning, continue (so we can still catch future payloads)
#              - Otherwise: call self._process_line()
#         """
#         try:
#             while True:
#                 if self.sock is None or self.fileobj is None:
#                     self._connect()

#                 try:
#                     line = self.fileobj.readline()
#                     if not line:
#                         # Publisher did a clean shutdown or closed socket
#                         self.logger.warning("Publisher has closed the connection (EOF). Will reconnect.")
#                         self._cleanup_connection()
#                         continue

#                     self._process_line(line.strip())
#                 except socket.timeout:
#                     self.logger.debug(f"No data in {self.timeout_ms}ms → continuing to wait.")
#                     continue
#                 except Exception as e:
#                     self.logger.error(f"Error reading from socket: {e}. Will reconnect.")
#                     self._cleanup_connection()
#                     continue

#         except KeyboardInterrupt:
#             self.logger.info("Interrupted by user. Exiting.")
#         finally:
#             self._cleanup_connection()

#     def _cleanup_connection(self):
#         """
#         Close current socket + fileobj. Reset them to None to force a reconnect on next loop.
#         """
#         if self.fileobj:
#             try:
#                 self.fileobj.close()
#             except Exception:
#                 pass
#             self.fileobj = None

#         if self.sock:
#             try:
#                 self.sock.close()
#             except Exception:
#                 pass
#             self.sock = None

#     def _process_line(self, line: str):
#         """
#         Parse an incoming line:
#           - If line starts with "HEARTBEAT,": log at DEBUG and return.
#           - If “MALFORMED_DATA” or len(split) != 12: log a warning or error, return.
#           - Else: parse 12 CSV fields → float/int → compute roll, pitch, yaw & quaternion → log.
#           - If --output-csv given: append “timestampAcc,roll_deg,pitch_deg,yaw_deg\n”.

#         Args:
#             line (str): A newline‐stripped string from publisher.
#         """
#         if not line:
#             return

#         # 1) HEARTBEAT detection
#         if line.startswith("HEARTBEAT,"):
#             try:
#                 _, ts_str = line.split(",", 1)
#                 ts = int(ts_str)
#                 self.logger.debug(f"Received HEARTBEAT at publisher‐timestamp={ts}")
#             except Exception:
#                 self.logger.warning(f"Malformed HEARTBEAT line: '{line}'")
#             return

#         # 2) MALFORMED_DATA or anything that doesn’t parse to 12 CSV fields
#         parts = [p.strip() for p in line.split(",")]
#         if len(parts) != 12:
#             self.logger.warning(f"Skipping malformed IMU line (expected 12 fields): '{line}'")
#             return

#         # 3) Parse numeric fields
#         try:
#             xAcc = float(parts[0])
#             yAcc = float(parts[1])
#             zAcc = float(parts[2])
#             timestampAcc = int(parts[3])

#             xGyro = int(parts[4])
#             yGyro = int(parts[5])
#             zGyro = int(parts[6])
#             timestampGyro = int(parts[7])

#             xMag = float(parts[8])
#             yMag = float(parts[9])
#             zMag = float(parts[10])
#             timestampMag = int(parts[11])
#         except ValueError as e:
#             self.logger.error(f"ValueError parsing IMU line '{line}': {e}")
#             return

#         # 4) Compute roll & pitch. We DO NOT clamp pitch; if denom=0, pitch → ±pi/2.
#         roll, pitch = compute_roll_pitch_from_accel(xAcc, yAcc, zAcc)

#         # 4a) If |pitch| > ~89°, issue a gimbal-lock warning
#         if abs(abs(pitch) - (math.pi / 2)) < math.radians(1.0):
#             self.logger.warning(
#                 "Pitch is within 1° of ±90° (gimbal lock risk): "
#                 f"pitch={math.degrees(pitch):.2f}°"
#             )

#         # 5) Compute yaw
#         yaw = compute_yaw_from_mag(xMag, yMag, zMag, roll, pitch)

#         # 6) Convert to degrees for logging
#         roll_deg = math.degrees(roll)
#         pitch_deg = math.degrees(pitch)
#         yaw_deg = math.degrees(yaw)

#         # 7) Convert to quaternion
#         qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)

#         # 8) Log the orientation
#         self.logger.info(
#             f"Orientation | AccTs={timestampAcc} | "
#             f"Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°, Yaw={yaw_deg:.2f}° | "
#             f"Quat=(w={qw:.4f}, x={qx:.4f}, y={qy:.4f}, z={qz:.4f})"
#         )

#         # 9) Append to CSV if requested
#         if self.output_csv:
#             try:
#                 with open(self.output_csv, "a") as f:
#                     f.write(f"{timestampAcc},{roll_deg:.4f},{pitch_deg:.4f},{yaw_deg:.4f}\n")
#             except Exception as e:
#                 self.logger.error(f"Failed to write to output CSV '{self.output_csv}': {e}")


# # =============================================================================
# # CLI argument parsing
# # =============================================================================
# def parse_args():
#     parser = argparse.ArgumentParser(description="IMU Consumer (Unix-domain IPC)")

#     parser.add_argument(
#         "--socket-path", type=str, required=True,
#         help="Path to the Unix-domain socket (e.g. /tmp/imu_socket)."
#     )
#     parser.add_argument(
#         "--timeout-ms", type=int, default=100,
#         help="Milliseconds to wait for data before timeout (default=100)."
#     )
#     parser.add_argument(
#         "--output-csv", type=str, default="",
#         help="If provided, append 'timestamp_ms,roll,pitch,yaw' rows to this CSV path."
#     )
#     parser.add_argument(
#         "--log-dir", type=str, default="",
#         help="Directory for rotating log files; if omitted, logs only to console."
#     )
#     parser.add_argument(
#         "--log-level", type=str, default="INFO",
#         choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
#         help="Logging verbosity (default=INFO)."
#     )

#     return parser.parse_args()


# # =============================================================================
# # Entrypoint
# # =============================================================================
# if __name__ == "__main__":
#     args = parse_args()
#     try:
#         consumer = IMUConsumer(
#             socket_path=args.socket_path,
#             timeout_ms=args.timeout_ms,
#             output_csv=args.output_csv,
#             log_dir=args.log_dir,
#             log_level=args.log_level
#         )
#         consumer.run()
#     except Exception as e:
#         print(f"[FATAL] {e}", file=sys.stderr)
#         sys.exit(1)
#!/usr/bin/env python3
"""
consumer.py

A Unix-domain IPC consumer that connects to a publisher’s socket, receives IMU payloads,
parses each CSV line of 12 values, computes orientation (roll, pitch, yaw, quaternion), and optionally
writes a rolling CSV of orientations to disk. Now refactored to load the dashboard logic from dashboard.py.

Usage:
    python3 consumer.py \
      --socket-path /tmp/imu_socket \
      --timeout-ms 100 \
      --log-dir /var/log/imu_consumer \
      --log-level INFO \
      --output-csv /tmp/imu_orientations.csv \
      [--dashboard]

CLI Options:
  --socket-path PATH
      Path to the Unix-domain socket (e.g. /tmp/imu_socket).
  --timeout-ms M
      Milliseconds to wait for data before timeout (default=100).
  --output-csv PATH
      If provided, append 'timestamp_ms,roll_deg,pitch_deg,yaw_deg' rows to this CSV.
  --log-dir DIR
      Directory for rotating log files; if omitted, logs only to console.
  --log-level {DEBUG,INFO,WARNING,ERROR,CRITICAL}
      Logging verbosity (default=INFO).
  --dashboard
      If present, spins up a live dashboard thread (see dashboard.py) that prints recent orientations
      every second.

Requirements:
  - Python 3.6+
  - Standard library only.
"""

import argparse
import logging
import logging.handlers
import os
import socket
import sys
import math
import time
from typing import Optional, Tuple

# Import the Dashboard class from our new module
from dashboard import Dashboard

# =============================================================================
# Helper: Orientation computation
# (Same as before)
# =============================================================================
def compute_roll_pitch_from_accel(x_acc: float, y_acc: float, z_acc: float) -> Tuple[float, float]:
    # ... (unchanged) ...
    if y_acc == 0.0 and z_acc == 0.0:
        roll = 0.0
    else:
        roll = math.atan2(y_acc, z_acc)

    denom = math.sqrt(y_acc * y_acc + z_acc * z_acc)
    if denom == 0.0:
        pitch = math.copysign(math.pi / 2.0, -x_acc)
    else:
        pitch = math.atan2(-x_acc, denom)

    return roll, pitch

def compute_yaw_from_mag(x_mag: float, y_mag: float, z_mag: float,
                         roll: float, pitch: float) -> float:
    # ... (unchanged) ...
    cos_r = math.cos(roll)
    sin_r = math.sin(roll)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)

    mag_xc = x_mag * cos_p + z_mag * sin_p
    mag_yc = x_mag * sin_r * sin_p + y_mag * cos_r - z_mag * sin_r * cos_p
    yaw = math.atan2(-mag_yc, mag_xc)
    return yaw

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    # ... (unchanged) ...
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz

# =============================================================================
# Main Consumer Class
# =============================================================================
class IMUConsumer:
    """
    IMUConsumer connects to a Unix-domain socket, receives lines of CSV (or special tags),
    parses valid 12-value IMU payloads, computes orientation, optionally writes CSV of
    (timestamp, roll_deg, pitch_deg, yaw_deg), and logs everything. Optionally starts
    a Dashboard (from dashboard.py) to show recent orientations in the terminal.
    """

    RECONNECT_DELAY_SEC = 1.0  # seconds before retrying to connect

    def __init__(self,
                 socket_path: str,
                 timeout_ms: int,
                 output_csv: Optional[str],
                 log_dir: str,
                 log_level: str,
                 use_dashboard: bool = False):
        """
        Args:
            socket_path (str): Path to Unix-domain socket (publisher’s).
            timeout_ms (int): Milliseconds to wait for data before timeout.
            output_csv (str|None): If set, path to append "timestamp_ms,roll_deg,pitch_deg,yaw_deg".
            log_dir (str): Directory for rotated logs. If empty, console only.
            log_level (str): DEBUG/INFO/WARNING/ERROR/CRITICAL.
            use_dashboard (bool): If True, start Dashboard thread (defined in dashboard.py).
        """
        self.socket_path = socket_path
        self.timeout_ms = timeout_ms
        self.output_csv = output_csv or ""
        self.log_dir = log_dir.rstrip("/") if log_dir else ""
        self.log_level = log_level.upper()
        self.logger = self._setup_logging()
        self.sock: Optional[socket.socket] = None
        self.fileobj = None  # file‐like wrapper around sock

        # A shared list that holds (timestamp_ms, roll_deg, pitch_deg, yaw_deg)
        self._recent_orientations = []

        # Dashboard thread
        self.dashboard: Optional[Dashboard] = None
        if use_dashboard:
            self.dashboard = Dashboard(self._recent_orientations, refresh_interval=1.0, max_entries=5)
            self.dashboard.start()
            self.logger.info("Dashboard thread started.")

        # Pre‐create output CSV if needed
        if self.output_csv:
            try:
                if not os.path.exists(self.output_csv):
                    with open(self.output_csv, "w") as f:
                        f.write("timestamp_ms,roll_deg,pitch_deg,yaw_deg\n")
            except Exception as e:
                raise IOError(f"Cannot create/write to output CSV {self.output_csv}: {e}")

    def _setup_logging(self) -> logging.Logger:
        """
        Configure a logger with console + optional rotating file handler.
        """
        logger = logging.getLogger("IMUConsumer")
        level = getattr(logging, self.log_level, logging.INFO)
        logger.setLevel(level)

        # Console handler
        console_h = logging.StreamHandler(sys.stdout)
        console_h.setLevel(level)
        fmt = "[%(asctime)s] [%(levelname)s] %(name)s: %(message)s"
        datefmt = "%Y-%m-%d %H:%M:%S"
        formatter = logging.Formatter(fmt, datefmt=datefmt)
        console_h.setFormatter(formatter)
        logger.addHandler(console_h)

        # File handler if log_dir provided
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
            log_path = os.path.join(self.log_dir, "imu_consumer.log")
            file_h = logging.handlers.RotatingFileHandler(
                log_path, maxBytes=10 * 1024 * 1024, backupCount=5
            )
            file_h.setLevel(level)
            file_h.setFormatter(formatter)
            logger.addHandler(file_h)

        return logger

    def _connect(self):
        """
        Attempt to connect to the Unix-domain socket. On success, wrap sock in fileobj.
        Retries indefinitely if socket file doesn't exist or connection is refused.
        """
        while True:
            if not os.path.exists(self.socket_path):
                self.logger.warning(f"Socket '{self.socket_path}' not found. Retrying in {self.RECONNECT_DELAY_SEC}s...")
                time.sleep(self.RECONNECT_DELAY_SEC)
                continue

            try:
                self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.sock.settimeout(self.timeout_ms / 1000.0)
                self.sock.connect(self.socket_path)
                self.fileobj = self.sock.makefile("r")
                self.logger.info(f"Connected to publisher socket '{self.socket_path}' (timeout={self.timeout_ms}ms).")
                break
            except (ConnectionRefusedError, FileNotFoundError) as e:
                self.logger.warning(f"Connection issue: {e}. Retrying in {self.RECONNECT_DELAY_SEC}s...")
                time.sleep(self.RECONNECT_DELAY_SEC)
            except Exception as e:
                self.logger.error(f"Unexpected connect error: {e}")
                time.sleep(self.RECONNECT_DELAY_SEC)

    def run(self):
        """
        Main loop:
          1. If not connected, call _connect()
          2. Attempt to read one line via self.fileobj.readline()
             - If empty string: publisher closed connection → cleanup & reconnect
             - If timeout: log warning, continue
             - Otherwise: call self._process_line()
        """
        try:
            while True:
                if self.sock is None or self.fileobj is None:
                    self._connect()

                try:
                    line = self.fileobj.readline()
                    if not line:
                        # Publisher closed socket
                        self.logger.warning("Publisher closed the connection (EOF). Reconnecting.")
                        self._cleanup_connection()
                        continue

                    self._process_line(line.strip())
                except socket.timeout:
                    self.logger.debug(f"No data in {self.timeout_ms}ms → waiting again.")
                    continue
                except Exception as e:
                    self.logger.error(f"Error reading from socket: {e}. Reconnecting.")
                    self._cleanup_connection()
                    continue

        except KeyboardInterrupt:
            self.logger.info("Interrupted by user. Exiting.")
        finally:
            # Clean up socket
            self._cleanup_connection()
            # Stop dashboard if running
            if self.dashboard:
                self.dashboard.stop()
                self.logger.info("Dashboard thread stopped.")

    def _cleanup_connection(self):
        """
        Close current socket & fileobj. Reset them so next loop reconnects.
        """
        if self.fileobj:
            try:
                self.fileobj.close()
            except Exception:
                pass
            self.fileobj = None

        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _process_line(self, line: str):
        """
        Parse an incoming line:
          - "HEARTBEAT,<timestamp>": log at DEBUG, return.
          - Malformed or !=12 fields: log a warning, return.
          - Else: parse floats/ints, compute roll/pitch/yaw + quaternion, log INFO.
            If output_csv was given, append “timestampAcc,roll,pitch,yaw” to it.
            If dashboard is running, append (timestampAcc, roll, pitch, yaw) to shared list.
        """
        if not line:
            return

        # HEARTBEAT detection
        if line.startswith("HEARTBEAT,"):
            try:
                _, ts_str = line.split(",", 1)
                ts = int(ts_str)
                self.logger.debug(f"Received HEARTBEAT at {ts} ms")
            except Exception:
                self.logger.warning(f"Malformed HEARTBEAT line: '{line}'")
            return

        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 12:
            self.logger.warning(f"Skipping malformed IMU line (expected 12 fields): '{line}'")
            return

        # Parse numeric fields
        try:
            xAcc = float(parts[0])
            yAcc = float(parts[1])
            zAcc = float(parts[2])
            timestampAcc = int(parts[3])

            xGyro = int(parts[4])
            yGyro = int(parts[5])
            zGyro = int(parts[6])
            timestampGyro = int(parts[7])

            xMag = float(parts[8])
            yMag = float(parts[9])
            zMag = float(parts[10])
            timestampMag = int(parts[11])
        except ValueError as e:
            self.logger.error(f"ValueError parsing IMU line '{line}': {e}")
            return

        # Compute roll & pitch (no clamping)
        roll, pitch = compute_roll_pitch_from_accel(xAcc, yAcc, zAcc)

        # Warn if near gimbal lock (|pitch| ≈ 90°)
        if abs(abs(pitch) - (math.pi / 2)) < math.radians(1.0):
            self.logger.warning(
                f"Pitch is within 1° of ±90° (gimbal lock risk): pitch={math.degrees(pitch):.2f}°"
            )

        # Compute yaw
        yaw = compute_yaw_from_mag(xMag, yMag, zMag, roll, pitch)

        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Convert to quaternion
        qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)

        # Log orientation
        self.logger.info(
            f"Orientation | AccTs={timestampAcc} | "
            f"Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°, Yaw={yaw_deg:.2f}° | "
            f"Quat=(w={qw:.4f}, x={qx:.4f}, y={qy:.4f}, z={qz:.4f})"
        )

        # Append to rolling list for dashboard
        if self.dashboard is not None:
            self._recent_orientations.append((timestampAcc, roll_deg, pitch_deg, yaw_deg))
            # Trim list to, say, 50 entries max to avoid unbounded growth
            if len(self._recent_orientations) > 50:
                self._recent_orientations.pop(0)

        # Write to output CSV if requested
        if self.output_csv:
            try:
                with open(self.output_csv, "a") as f:
                    f.write(f"{timestampAcc},{roll_deg:.4f},{pitch_deg:.4f},{yaw_deg:.4f}\n")
            except Exception as e:
                self.logger.error(f"Failed to write to output CSV '{self.output_csv}': {e}")


# =============================================================================
# CLI argument parsing
# =============================================================================
def parse_args():
    parser = argparse.ArgumentParser(description="IMU Consumer (Unix-domain IPC)")

    parser.add_argument(
        "--socket-path", type=str, required=True,
        help="Path to the Unix-domain socket (e.g. /tmp/imu_socket)."
    )
    parser.add_argument(
        "--timeout-ms", type=int, default=100,
        help="Milliseconds to wait for data before timeout (default=100)."
    )
    parser.add_argument(
        "--output-csv", type=str, default="",
        help="If provided, append 'timestamp_ms,roll_deg,pitch_deg,yaw_deg' to this CSV."
    )
    parser.add_argument(
        "--log-dir", type=str, default="",
        help="Directory for rotating log files; if omitted, logs only to console."
    )
    parser.add_argument(
        "--log-level", type=str, default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Logging verbosity (default=INFO)."
    )
    parser.add_argument(
        "--dashboard", action="store_true",
        help="If present, start a live dashboard (from dashboard.py) to show recent orientations."
    )

    return parser.parse_args()


# =============================================================================
# Entrypoint
# =============================================================================
if __name__ == "__main__":
    args = parse_args()
    try:
        consumer = IMUConsumer(
            socket_path=args.socket_path,
            timeout_ms=args.timeout_ms,
            output_csv=args.output_csv,
            log_dir=args.log_dir,
            log_level=args.log_level,
            use_dashboard=args.dashboard
        )
        consumer.run()
    except Exception as e:
        print(f"[FATAL] {e}", file=sys.stderr)
        sys.exit(1)
