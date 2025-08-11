import sys
from scapy.all import rdpcap, UDP, IP
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import io
from collections import defaultdict
from tabulate import tabulate

def decode_with_stats(pcap_file):
    packets = rdpcap(pcap_file)
    decoder = mavlink.MAVLink(None)

    ip_stats = defaultdict(lambda: {
        'total': 0,
        'gps_raw_int': 0,
        'mission_item': 0,
        'destinations': defaultdict(int)
    })

    for pkt in packets:
        if pkt.haslayer(UDP) and pkt.haslayer(IP):
            src = f"{pkt[IP].src}:{pkt[UDP].sport}"
            dst = f"{pkt[IP].dst}:{pkt[UDP].dport}"
            payload = bytes(pkt[UDP].payload)
            buf = io.BytesIO(payload)

            while True:
                byte = buf.read(1)
                if not byte:
                    break
                try:
                    msg = decoder.parse_char(byte)
                    if msg:
                        ip_stats[src]['total'] += 1
                        ip_stats[src]['destinations'][dst] += 1

                        if msg.get_msgId() == mavlink.MAVLINK_MSG_ID_GPS_RAW_INT:
                            ip_stats[src]['gps_raw_int'] += 1
                        elif msg.get_msgId() == mavlink.MAVLINK_MSG_ID_MISSION_ITEM:
                            ip_stats[src]['mission_item'] += 1
                except Exception:
                    continue

    # Summary Table
    summary_table = []
    for src, stats in ip_stats.items():
        summary_table.append([
            src,
            stats['total'],
            stats['gps_raw_int'],
            stats['mission_item']
        ])
    
    print("\n=== MAVLink Message Summary by Source ===")
    print(tabulate(summary_table, headers=["Source", "Total", "GPS_RAW_INT", "MISSION_ITEM"], tablefmt="pretty"))

    # Per-source destination breakdown
    print("\n=== Message Destinations by Source ===")
    for src, stats in ip_stats.items():
        print(f"\n {src}")
        dest_table = [[dst, count] for dst, count in stats['destinations'].items()]
        print(tabulate(dest_table, headers=["Destination", "Messages"], tablefmt="github"))

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python mavlink_pcap_summary.py <file.pcap>")
        sys.exit(1)

    decode_with_stats(sys.argv[1])
