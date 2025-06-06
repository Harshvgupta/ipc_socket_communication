#!/usr/bin/env python3

import argparse
import logging
import logging.handlers
import os
import socket
import sys
import time
import random
from typing import Tuple


def generate_imu_payload() -> Tuple[float, float, float, int,
                                    int, int, int, int,
                                    float, float, float, int]:
    """
    Generate a random IMU payload with 12 values:
      1. xAcc (float mg)       : ±2000 mg
      2. yAcc (float mg)
      3. zAcc (float mg)
      4. timestampAcc (int ms) : ms since epoch

      5. xGyro (int mDeg/s)    : ±250000 mDeg/s (±250°/s)
      6. yGyro (int mDeg/s)
      7. zGyro (int mDeg/s)
      8. timestampGyro (int ms)

      9. xMag (float mGauss)   : ±4800 mGauss (±4.8 Gauss)
     10. yMag (float mGauss)
     11. zMag (float mGauss)
     12. timestampMag (int ms)

    Returns:
        Tuple of 12 randomly generated values.
    """
    now_ms = int(time.time() * 1000)

    # Accelerometer (±2000 mg)
    xAcc = random.uniform(-2000.0, 2000.0)
    yAcc = random.uniform(-2000.0, 2000.0)
    zAcc = random.uniform(-2000.0, 2000.0)
    timestampAcc = now_ms

    # Gyroscope (±250000 mDeg/s)
    xGyro = random.randint(-250000, 250000)
    yGyro = random.randint(-250000, 250000)
    zGyro = random.randint(-250000, 250000)
    timestampGyro = now_ms

    # Magnetometer (±4800 mGauss)
    xMag = random.uniform(-4800.0, 4800.0)
    yMag = random.uniform(-4800.0, 4800.0)
    zMag = random.uniform(-4800.0, 4800.0)
    timestampMag = now_ms

    return (xAcc, yAcc, zAcc, timestampAcc,
            xGyro, yGyro, zGyro, timestampGyro,
            xMag, yMag, zMag, timestampMag)



class IMUPublisher:
    """
    IMUPublisher binds to a Unix-domain socket and waits for a single consumer to connect.
    Tracks how many payloads have been sent, and uses a rotating file handler for logs.
    """

    def __init__(self,
                 socket_path: str,
                 frequency_hz: int,
                 heartbeat_every: int,
                 test_mode: bool,
                 log_dir: str,
                 log_level: str):
        """
        Args:
            socket_path (str): Filesystem path for Unix-domain socket.
            frequency_hz (int): Payloads per second.
            heartbeat_every (int): Send a heartbeat every N payloads. 0 = disabled.
            test_mode (bool): If True, inject a malformed line every ~500 payloads.
            log_dir (str): Directory for file‐based logs; if empty, logs only to console.
            log_level (str): One of DEBUG, INFO, WARNING, ERROR, CRITICAL.
        """
        self.socket_path = socket_path
        self.frequency_hz = frequency_hz
        self.heartbeat_every = heartbeat_every
        self.test_mode = test_mode
        self.log_dir = log_dir.rstrip("/") if log_dir else ""
        self.log_level = log_level.upper()
        self.logger = self._setup_logging()
        self.server_socket = None
        self._payload_count = 0

    def _setup_logging(self) -> logging.Logger:
        """
        Configure logging:
        """
        logger = logging.getLogger("IMUPublisher")
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

        # File handler, if requested
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
            log_path = os.path.join(self.log_dir, "imu_publisher.log")
            file_h = logging.handlers.RotatingFileHandler(
                log_path, maxBytes=10 * 1024 * 1024, backupCount=5
            )
            file_h.setLevel(level)
            file_h.setFormatter(formatter)
            logger.addHandler(file_h)

        return logger

    def _cleanup_socket(self):
        """
        If the Unix-domain socket file already exists, remove it before binding.
        """
        if os.path.exists(self.socket_path):
            os.remove(self.socket_path)

    def run(self):
        """
        Bind to the Unix socket and wait for a consumer to connect.
        Then call _send_loop() to push data at the configured rate.
        """
        self._cleanup_socket()

        # Create and bind socket
        self.server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self.server_socket.bind(self.socket_path)
            self.server_socket.listen(1)
            self.logger.info(f"Bound to Unix socket: {self.socket_path}. Listening for consumer...")
        except Exception as e:
            self.logger.critical(f"Cannot bind/listen on socket '{self.socket_path}': {e}")
            self.server_socket.close()
            sys.exit(1)

        try:
            while True:
                self.logger.info("Waiting for a consumer to connect...")
                try:
                    conn, _ = self.server_socket.accept()
                except KeyboardInterrupt:
                    self.logger.info("Interrupted by user. Shutting down.")
                    break
                except Exception as e:
                    self.logger.error(f"Accept failed: {e}")
                    continue

                self.logger.info("Consumer connected. Entering send loop.")
                try:
                    self._send_loop(conn)
                except Exception as e:
                    self.logger.warning(f"Send loop error: {e}")
                finally:
                    conn.close()
                    self.logger.info("Connection closed. Returning to listening state.")

        finally:
            if self.server_socket:
                self.server_socket.close()
            if os.path.exists(self.socket_path):
                os.remove(self.socket_path)
            self.logger.info("Publisher cleanly shut down.")

    def _send_loop(self, conn: socket.socket):
        """
        Send IMU or heartbeat or malformed lines in a tight loop at frequency_hz.

        Raises:
            Exception: for any send failure.
        """
        interval = 1.0 / self.frequency_hz
        self.logger.info(f"Sending at {self.frequency_hz} Hz (interval={interval:.4f}s), heartbeat_every={self.heartbeat_every}, test_mode={self.test_mode}")

        while True:
            now_ms = int(time.time() * 1000)
            line_to_send: str

            # 1) Heartbeat check
            if self.heartbeat_every > 0 and (self._payload_count > 0) and (self._payload_count % self.heartbeat_every == 0):
                line_to_send = f"HEARTBEAT,{now_ms}\n"
                self.logger.debug(f"Sending HEARTBEAT at count={self._payload_count}")

            # 2) Test‐mode malformed line
            elif self.test_mode and (self._payload_count > 0) and (self._payload_count % 500 == 0):
                line_to_send = "MALFORMED_DATA\n"
                self.logger.warning("Injecting MALFORMED_DATA (test mode).")

            # 3) Normal IMU payload
            else:
                payload = generate_imu_payload()
                # Format as CSV with 4 decimal places for floats
                parts = []
                for v in payload:
                    if isinstance(v, float):
                        parts.append(f"{v:.4f}")
                    else:
                        parts.append(str(v))
                line_to_send = ",".join(parts) + "\n"

            # Attempt to send
            data_bytes = line_to_send.encode("utf-8")
            try:
                conn.sendall(data_bytes)
                self.logger.debug(f"Sent: {line_to_send.strip()}")
            except Exception as e:
                self.logger.error(f"Send failed: {e}")
                raise

            self._payload_count += 1
            time.sleep(interval)


def parse_args():
    parser = argparse.ArgumentParser(description="IMU Publisher (Unix-domain IPC)")

    parser.add_argument(
        "--socket-path", type=str, required=True,
        help="Path to the Unix-domain socket (e.g. /tmp/imu_socket)."
    )
    parser.add_argument(
        "--frequency-hz", type=int, default=100,
        help="IMU payloads per second (default: 100)."
    )
    parser.add_argument(
        "--heartbeat-every", type=int, default=0,
        help="Send a heartbeat every N payloads (0 = disabled)."
    )
    parser.add_argument(
        "--test-mode", type=lambda s: s.lower() in ("true", "1", "t"), default=False,
        help="If true, inject a malformed line every 500 payloads to test consumer robustness."
    )
    parser.add_argument(
        "--log-dir", type=str, default="",
        help="Directory for log files (rotating). If omitted, logs only to stdout."
    )
    parser.add_argument(
        "--log-level", type=str, default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Logging verbosity level (default: INFO)."
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        publisher = IMUPublisher(
            socket_path=args.socket_path,
            frequency_hz=args.frequency_hz,
            heartbeat_every=args.heartbeat_every,
            test_mode=args.test_mode,
            log_dir=args.log_dir,
            log_level=args.log_level
        )
        publisher.run()
    except Exception as e:
        print(f"[FATAL] {e}", file=sys.stderr)
        sys.exit(1)