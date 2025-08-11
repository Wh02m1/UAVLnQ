#!/usr/bin/env python3
"""
mavlink_zmq_raw.py

Listens for MAVLink packets on SITL UDP ports 14551, 14561, 14571,
and republishes the *raw* MAVLink frames (header+payload+CRC) unchanged
over separate ZMQ PUB sockets:
  14551 → tcp://*:5550
  14561 → tcp://*:5551
  14571 → tcp://*:5552
"""

import threading
import time
import zmq
from pymavlink import mavutil

# mapping SITL UDP port → ZMQ PUB port
PORT_MAP = {
    14551: 5550,  # Drone 1
    14561: 5551,  # Drone 2
    14571: 5552,  # Drone 3
}


def forward_raw(udp_port: int, zmq_port: int):
    """
    Thread function: connect via pymavlink to udp_port, receive
    MAVLink messages, and forward raw bytes (header+payload+CRC)
    unchanged to zmq_port.
    """
    # 1) open a MAVLink connection (this gives us parsing + framing)
    conn_str = f"udp:127.0.0.1:{udp_port}"
    master = mavutil.mavlink_connection(conn_str)
    print(f"[MAVLINK] Listening on {conn_str}")

    # wait for initial heartbeat
    master.wait_heartbeat()
    print(f"[MAVLINK] Heartbeat received on {conn_str}")

    # 2) set up ZMQ PUB socket
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    bind_addr = f"tcp://*:{zmq_port}"
    pub.bind(bind_addr)
    print(f"[ZMQ] Publishing raw MAVLink → {bind_addr}")

    # 3) main loop: recv_msg gives you a MAVLinkMessage object,
    #    but we want its raw wire bytes.  We can use get_msgbuf().
    while True:
        msg = master.recv_msg()
        if msg is None:
            # no message, just loop
            time.sleep(0.001)
            continue

        try:
            # get raw bytes (including STX, header, payload, CRC)
            raw_bytes = msg.get_msgbuf()
            # send as a single binary ZMQ frame
            pub.send(raw_bytes)
        except Exception as e:
            print(f"[ERROR] UDP:{udp_port} → ZMQ:{zmq_port}: {e}")


if __name__ == "__main__":
    threads = []
    for udp_port, zmq_port in PORT_MAP.items():
        t = threading.Thread(
            target=forward_raw,
            args=(udp_port, zmq_port),
            daemon=True
        )
        t.start()
        threads.append(t)

    print("Raw‐packet MAVLink→ZMQ bridge running.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down bridge.")
