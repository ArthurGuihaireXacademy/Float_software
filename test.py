import socket_client as client
import time
from datetime import datetime

client.add_packet(f"[{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}] Logged Depth: {1.000:.3f} m")
time.sleep(1)
client.add_packet(f"[{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}] Logged Depth: {1.125:.3f} m")
time.sleep(1)
client.connect("127.0.0.1")
client.send_packets()
client.disconnect()
client.add_packet(f"[{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}] Logged Depth: {1.500:.3f} m")
time.sleep(1)
client.add_packet(f"[{datetime.now().strftime("%Y-%m-%d %H:%M:%S")}] Logged Depth: {1.625:.3f} m")
client.add_packet("done")
client.connect("127.0.0.1")
client.send_packets()
client.disconnect()