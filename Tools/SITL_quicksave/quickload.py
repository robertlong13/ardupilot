#!/usr/bin/env python3
"""Trigger a SITL quickload via MAVLink over UDP 14553.

Uses the PREFLIGHT_REBOOT_SHUTDOWN developer magic sequence (42,24,71)
with param4=111, which the SITL build maps to "quickload". After this the
original (parent) process exits and the saved child takes over, so expect
to reconnect your GCS.
"""
import sys
from pymavlink import mavutil

CONN = 'udpin:0.0.0.0:14553'
PARAM4 = 111.0  # 110 = quicksave, 111 = quickload


def main():
    m = mavutil.mavlink_connection(CONN)
    print(f"waiting for heartbeat on {CONN} ...")
    if not m.wait_heartbeat(timeout=10):
        print("no heartbeat - is SITL streaming to UDP 14553?")
        return 1
    print(f"heartbeat from system {m.target_system} component {m.target_component}")

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        42, 24, 71, PARAM4, 0, 0, 0)
    print("quickload command sent")

    ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack is not None:
        print(f"ACK: result={ack.result}")
    else:
        print("no ACK seen (parent likely exited before ACK - expected on quickload)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
