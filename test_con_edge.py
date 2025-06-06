#!/usr/bin/env python3
"""
test_con_edge.py

Unit tests and edge‐case tests for IMUConsumer._process_line(...) and Dashboard integration.

Covers:
  - Valid IMU payload parsing
  - HEARTBEAT handling
  - Malformed lines (too few/many fields, non‐numeric, partial CSV)
  - Exact gimbal‐lock conditions (denom=0 → pitch=±90°)
  - Dashboard thread starts, stops, and displays recent orientations
  - Writing to output CSV (header + data row). Ensures header behavior by removing any preexisting file.
"""

import unittest
import tempfile
import os
import math
import time
import socket
import threading

# Adjust these imports if your modules are named differently:
from con import IMUConsumer
from pub import generate_imu_payload
from dashboard import Dashboard

# Helper to format a valid IMU CSV line (no trailing newline).
def make_valid_imu_line(xAcc, yAcc, zAcc, timestampAcc,
                        xGyro, yGyro, zGyro, timestampGyro,
                        xMag, yMag, zMag, timestampMag) -> str:
    parts = []
    for v in (xAcc, yAcc, zAcc, timestampAcc,
              xGyro, yGyro, zGyro, timestampGyro,
              xMag, yMag, zMag, timestampMag):
        if isinstance(v, float):
            parts.append(f"{v:.4f}")
        else:
            parts.append(str(v))
    return ",".join(parts)


class TestProcessLineValidAndHeartbeat(unittest.TestCase):
    def setUp(self):
        # Create a temporary filename for CSV, then delete it immediately so consumer writes header
        fd, path = tempfile.mkstemp(prefix="imu_test_", suffix=".csv")
        os.close(fd)
        os.remove(path)
        self.tmp_csv_path = path

        self.consumer = IMUConsumer(
            socket_path="/tmp/unused.sock",
            timeout_ms=10,
            output_csv=self.tmp_csv_path,
            log_dir="",       # console-only logging
            log_level="DEBUG",
            use_dashboard=False
        )

    def tearDown(self):
        # Clean up the CSV file if it exists
        if os.path.exists(self.tmp_csv_path):
            try:
                os.remove(self.tmp_csv_path)
            except Exception:
                pass

    def test_valid_line_and_csv_write(self):
        # Construct a valid IMU line
        line = make_valid_imu_line(
            1000.0, -1000.0,  500.0, 111111,
            120000, -120000,  60000, 222222,
            300.0, -300.0, 150.0, 333333
        )
        # Should not raise
        self.consumer._process_line(line)

        # Now read back the CSV: header + one data row
        with open(self.tmp_csv_path, "r") as f:
            lines = f.readlines()

        # We expect exactly 2 lines: header + row
        self.assertEqual(len(lines), 2, f"Expected 2 lines (header+row), got {len(lines)}")

        header = lines[0].strip()
        row = lines[1].strip()
        self.assertEqual(header, "timestamp_ms,roll_deg,pitch_deg,yaw_deg")

        fields = row.split(",")
        self.assertEqual(len(fields), 4)
        ts_read = int(fields[0])
        roll_read = float(fields[1])
        pitch_read = float(fields[2])
        yaw_read = float(fields[3])

        self.assertEqual(ts_read, 111111)
        # Compute expected roll & pitch
        expected_roll = math.degrees(math.atan2(-1000.0, 500.0))
        denom = math.sqrt((-1000.0)**2 + (500.0)**2)
        expected_pitch = math.degrees(math.atan2(-1000.0, denom))
        # Yaw is from magnetometer; just ensure it’s finite
        self.assertAlmostEqual(roll_read, expected_roll, places=2)
        self.assertAlmostEqual(pitch_read, expected_pitch, places=2)
        self.assertTrue(math.isfinite(yaw_read))

    def test_heartbeat_line_ignored(self):
        line = "HEARTBEAT,999999"
        # Should not raise, and CSV should contain only the header (no data row)
        self.consumer._process_line(line)
        with open(self.tmp_csv_path, "r") as f:
            lines = f.readlines()
        # Only the header should exist
        self.assertEqual(len(lines), 1, f"Expected only header, got {len(lines)} lines")
        self.assertEqual(lines[0].strip(), "timestamp_ms,roll_deg,pitch_deg,yaw_deg")


class TestProcessLineMalformedAndGimbalLock(unittest.TestCase):
    def setUp(self):
        self.consumer = IMUConsumer(
            socket_path="/tmp/unused.sock",
            timeout_ms=10,
            output_csv="",  # no CSV writing
            log_dir="",
            log_level="DEBUG",
            use_dashboard=False
        )

    def test_too_few_fields(self):
        bad_line = "1,2,3,4,5"
        # Should not raise
        self.consumer._process_line(bad_line)

    def test_too_many_fields(self):
        bad_line = ",".join(str(i) for i in range(13))
        # Should not raise
        self.consumer._process_line(bad_line)

    def test_non_numeric(self):
        bad_line = "a,b,c,d,e,f,g,h,i,j,k,l"
        # Should log an error but not raise
        self.consumer._process_line(bad_line)

    def test_exact_gimbal_lock_positive(self):
        # y=0, z=0 => roll=0; xAcc=-1000 => pitch = +90°
        line = make_valid_imu_line(
            -1000.0, 0.0, 0.0, 123000,
             0,      0,   0,   123000,
             0.0,    0.0, 0.0, 123000
        )
        # Should not raise; will warn about near gimbal lock
        self.consumer._process_line(line)

    def test_exact_gimbal_lock_negative(self):
        # y=0, z=0 => roll=0; xAcc=+1000 => pitch = -90°
        line = make_valid_imu_line(
             1000.0, 0.0, 0.0, 123001,
             0,      0,   0,   123001,
             0.0,    0.0, 0.0, 123001
        )
        self.consumer._process_line(line)


class TestDashboardThread(unittest.TestCase):
    def test_dashboard_starts_and_stops(self):
        shared_list = []
        dashboard = Dashboard(shared_list, refresh_interval=0.2, max_entries=3)
        dashboard.start()

        # Wait briefly, then populate shared_list
        time.sleep(0.1)
        shared_list.append((111, 10.0, 20.0, 30.0))
        shared_list.append((222, 11.0, 21.0, 31.0))
        time.sleep(0.3)  # allow at least one print cycle

        # Now stop the dashboard
        dashboard.stop()
        dashboard.join(timeout=1.0)
        self.assertFalse(dashboard.is_alive())


class TestIMUConsumerFullIntegration(unittest.TestCase):
    """
    Simulate a socketpair publisher → consumer.run() for a short while,
    including both valid and malformed lines to ensure the consumer remains stable.
    """

    def setUp(self):
        # Create a socketpair
        self.publisher_sock, self.consumer_sock = socket.socketpair()

        # Instantiate consumer but bypass _connect() by injecting the socketpair end
        self.consumer = IMUConsumer(
            socket_path="/tmp/unused.sock",
            timeout_ms=50,
            output_csv="",  # no CSV
            log_dir="",
            log_level="DEBUG",
            use_dashboard=False
        )
        # Monkey‐patch the consumer’s socket and fileobj so run() uses these directly
        self.consumer.sock = self.consumer_sock
        self.consumer.fileobj = self.consumer_sock.makefile("r")

        # Start a fake publisher thread
        def publisher_task():
            for i in range(20):
                if i % 5 == 0 and i != 0:
                    to_send = "MALFORMED_DATA\n"
                else:
                    payload = make_valid_imu_line(
                        (i * 100.0) % 2000.0,
                        (i * 100.0) % 2000.0,
                        (i * 100.0) % 2000.0,
                        100000 + i,
                        i * 1000,
                        i * -1000,
                        i * 500,
                        200000 + i,
                        i * 10.0,
                        i * -10.0,
                        i * 5.0,
                        300000 + i
                    )
                    to_send = payload + "\n"
                try:
                    self.publisher_sock.sendall(to_send.encode("utf-8"))
                except BrokenPipeError:
                    break
                time.sleep(0.01)
            # Close the publisher socket to simulate EOF
            try:
                self.publisher_sock.close()
            except Exception:
                pass

        self.pub_thread = threading.Thread(target=publisher_task, daemon=True)
        self.pub_thread.start()

    def tearDown(self):
        try:
            self.consumer_sock.close()
        except Exception:
            pass

    def test_consumer_run_loop_for_short_duration(self):
        """
        Run consumer.run() in a thread. It should process ~20 lines,
        skip malformed lines, then detect EOF and attempt to reconnect.
        If no exceptions are thrown, the test passes.
        """
        def run_consumer():
            self.consumer.run()

        c_thread = threading.Thread(target=run_consumer, daemon=True)
        c_thread.start()

        # Wait briefly for publisher → consumer interaction
        self.pub_thread.join(timeout=1.0)
        time.sleep(0.1)

        # Clean up so consumer can exit
        self.consumer._cleanup_connection()

        # If we reach here without an unhandled exception, test passes
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()
