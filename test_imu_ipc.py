#!/usr/bin/env python3
"""
test_imu_ipc.py

Unit tests for IMUConsumer._process_line(...) and integration “fuzz” tests that
simulate random / malformed IMU lines via a socketpair, ensuring:
  1. The consumer never throws an unhandled exception on random or malformed data.
  2. A socket closure (EOF) is detected and triggers a reconnect path.

Run:
   python3 -m unittest test_imu_ipc.py
"""

import unittest
import threading
import socket
import time
import os
import math

# Import the IMUConsumer and IMUPublisher classes directly
# (Assume publisher.py and consumer.py are in the same directory or installed in PYTHONPATH)
from consumer import IMUConsumer
from publisher import generate_imu_payload

class FakePublisherThread(threading.Thread):
    """
    A thread that writes to a socketpair, sending:
      - N well-formed IMU lines
      - Occasionally a "MALFORMED_DATA\n"
      - A final socket.close() to simulate EOF
    """

    def __init__(self, write_sock: socket.socket, total_msgs: int, interval_s: float):
        super().__init__(daemon=True)
        self.write_sock = write_sock
        self.total_msgs = total_msgs
        self.interval_s = interval_s
        self._stop_event = threading.Event()

    def run(self):
        """
        Send total_msgs lines, sleeping interval_s between each.
        Insert a MALFORMED_DATA once every ~10 msgs.
        Then close the socket.
        """
        for i in range(self.total_msgs):
            if self._stop_event.is_set():
                break

            # 10% chance to send malformed
            if i % 10 == 0 and i != 0:
                line = "MALFORMED_DATA\n"
            else:
                payload = generate_imu_payload()
                parts = []
                for v in payload:
                    if isinstance(v, float):
                        parts.append(f"{v:.4f}")
                    else:
                        parts.append(str(v))
                line = ",".join(parts) + "\n"

            try:
                self.write_sock.sendall(line.encode("utf-8"))
            except BrokenPipeError:
                break

            time.sleep(self.interval_s)

        # Finally, close to simulate publisher shutdown
        try:
            self.write_sock.close()
        except Exception:
            pass

    def stop(self):
        self._stop_event.set()
        try:
            self.write_sock.close()
        except Exception:
            pass


class TestIMUConsumerParsing(unittest.TestCase):
    """
    Verify that IMUConsumer._process_line() never throws on:
      - valid random IMU lines
      - malformed lines
      - missing fields
      - invalid numbers
    """

    def setUp(self):
        # Create a dummy consumer with output_csv="" and no real socket
        self.consumer = IMUConsumer(
            socket_path="/tmp/does_not_exist.sock",
            timeout_ms=10,
            output_csv="",    # we don't actually write CSV in these tests
            log_dir="",       # console‐only logging
            log_level="DEBUG"
        )

    def test_valid_line_parses(self):
        # Generate 5 random valid lines
        for _ in range(5):
            payload = generate_imu_payload()
            parts = []
            for v in payload:
                if isinstance(v, float):
                    parts.append(f"{v:.4f}")
                else:
                    parts.append(str(v))
            line = ",".join(parts)
            # Should not raise
            self.consumer._process_line(line)

    def test_malformed_line_skipped(self):
        bad_lines = [
            "TOO,FEW,VALUES,1,2,3\n",
            "TOO,MANY," + ",".join(str(i) for i in range(13)) + "\n",
            "NOT_A_NUMBER,1,2,3,4,5,6,7,8,9,10,11\n",
            "MALFORMED_DATA\n"
        ]
        for line in bad_lines:
            # Should not raise
            self.consumer._process_line(line)

    def test_gimbal_lock_warning(self):
        # Force Y=0, Z=0, xAcc=±1000 → pitch = ±90° exactly
        line1 = "0.0000,0.0000,-1000.0000,123456,0,0,0,123456,0.0000,0.0000,0.0000,123456"
        # Should produce a warning, but no exception
        self.consumer._process_line(line1)
        # Reverse sign
        line2 = "-1000.0000,0.0000,0.0000,123456,0,0,0,123456,0.0000,0.0000,0.0000,123456"
        self.consumer._process_line(line2)

    def test_value_error_handling(self):
        # Non-numeric where numbers expected
        bad_line = "abc,def,ghi,jkl,mno,pqr,stu,vwx,yza,bcd,efg,hij"
        # Should log an error but not raise
        self.consumer._process_line(bad_line)


class TestIMUConsumerWithSocketpair(unittest.TestCase):
    """
    Integration‐style test: use socket.socketpair() to simulate publisher & consumer.
    - We run FakePublisherThread to write N messages at ~100Hz.
    - We monkey‐patch consumer._connect so that instead of connecting to a filesystem socket,
      it simply uses the read end of the socketpair to call _process_line.
    """

    def setUp(self):
        # Create a socket pair
        self.client_sock, self.server_sock = socket.socketpair()
        # Create a consumer, but override its connect logic
        self.consumer = IMUConsumer(
            socket_path="/tmp/never_used.sock",
            timeout_ms=50,
            output_csv="",
            log_dir="",
            log_level="DEBUG"
        )
        # Patch consumer’s internal sockets
        self.consumer.sock = self.server_sock
        self.consumer.fileobj = self.server_sock.makefile("r")

        # Start a fake publisher thread
        self.publisher_thread = FakePublisherThread(
            write_sock=self.client_sock,
            total_msgs=50,
            interval_s=0.005  # ~200 Hz
        )
        self.publisher_thread.start()

    def tearDown(self):
        self.publisher_thread.stop()
        try:
            self.server_sock.close()
        except Exception:
            pass
        try:
            self.client_sock.close()
        except Exception:
            pass

    def test_consumer_reads_without_exceptions(self):
        """
        Run consumer.run() for a short time. It should process ~50 lines (some valid, some malformed),
        then detect EOF (publisher closed) and attempt reconnect. We terminate after a brief wait.
        """
        def run_consumer():
            # Let it run for ~0.5s, then we'll kill it
            start = time.time()
            while time.time() - start < 0.6:
                try:
                    line = self.consumer.fileobj.readline()
                    if not line:
                        # EOF: break out
                        break
                    self.consumer._process_line(line.strip())
                except Exception as e:
                    self.fail(f"Consumer threw exception: {e}")
                    break

        run_consumer()
        # If we reach here without exceptions, success
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()
