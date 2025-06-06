#!/usr/bin/env python3
"""
dashboard.py

A small “live dashboard” thread that periodically (once per second) prints
the most recent orientations (roll, pitch, yaw) stored in a shared list.

Usage:
    - Instantiate Dashboard(shared_list, refresh_interval=1.0)
    - Call dashboard.start() to launch its background thread.
    - The shared_list is expected to contain tuples of (timestamp_ms, roll_deg, pitch_deg, yaw_deg).
    - Every refresh_interval seconds, the dashboard prints the last up to N entries
      (configurable) in a compact table format.

This module is designed to be imported by consumer.py, which will append to shared_list
whenever a new valid IMU orientation is computed.
"""

import threading
import time
import sys

class Dashboard(threading.Thread):
    """
    A background thread that prints out recent orientations at a fixed interval.

    Attributes:
        shared_orientations (list): A thread‐safe list of (timestamp_ms, roll, pitch, yaw) tuples.
        refresh_interval (float): How many seconds to wait between screen updates.
        max_entries (int): How many recent entries to show at each refresh.
        stop_event (threading.Event): Signal to request thread shutdown.
    """

    def __init__(self,
                 shared_orientations: list,
                 refresh_interval: float = 1.0,
                 max_entries: int = 5):
        """
        Args:
            shared_orientations (list): A shared list (protected by consumer) of orientation tuples.
            refresh_interval (float): Seconds between prints (default 1.0).
            max_entries (int): How many of the most recent entries to display (default 5).
        """
        super().__init__(daemon=True)
        self.shared_orientations = shared_orientations
        self.refresh_interval = refresh_interval
        self.max_entries = max_entries
        self.stop_event = threading.Event()

    def run(self):
        """
        Loop until stop_event is set. Every refresh_interval seconds, print
        the last up to max_entries entries from shared_orientations.
        """
        while not self.stop_event.is_set():
            time.sleep(self.refresh_interval)
            if not self.shared_orientations:
                continue

            # Prepare a slice of the last entries
            recent = self.shared_orientations[-self.max_entries:]
            print("\n==== Last {} Orientations ====".format(len(recent)), file=sys.stdout)
            print(f"{'Timestamp (ms)':>13} | {'Roll (°)':>8} | {'Pitch (°)':>9} | {'Yaw (°)':>7}", file=sys.stdout)
            print("-" * 50, file=sys.stdout)
            for ts, roll, pitch, yaw in recent:
                print(f"{ts:13} | {roll:8.2f} | {pitch:9.2f} | {yaw:7.2f}", file=sys.stdout)
            print("=" * 50 + "\n", file=sys.stdout)

    def stop(self):
        """
        Signal the dashboard thread to exit on its next wake‐up.
        """
        self.stop_event.set()
