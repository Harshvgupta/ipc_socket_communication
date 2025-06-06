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


def compute_roll_pitch_from_accel(x_acc: float, y_acc: float, z_acc: float) -> Tuple[float, float]:
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
    cos_r = math.cos(roll)
    sin_r = math.sin(roll)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)

    mag_xc = x_mag * cos_p + z_mag * sin_p
    mag_yc = x_mag * sin_r * sin_p + y_mag * cos_r - z_mag * sin_r * cos_p
    yaw = math.atan2(-mag_yc, mag_xc)
    return yaw

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
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
                ):
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

        # Write to output CSV if requested
        if self.output_csv:
            try:
                with open(self.output_csv, "a") as f:
                    f.write(f"{timestampAcc},{roll_deg:.4f},{pitch_deg:.4f},{yaw_deg:.4f}\n")
            except Exception as e:
                self.logger.error(f"Failed to write to output CSV '{self.output_csv}': {e}")


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
    

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        consumer = IMUConsumer(
            socket_path=args.socket_path,
            timeout_ms=args.timeout_ms,
            output_csv=args.output_csv,
            log_dir=args.log_dir,
            log_level=args.log_level,
        )
        consumer.run()
    except Exception as e:
        print(f"[FATAL] {e}", file=sys.stderr)
        sys.exit(1)
