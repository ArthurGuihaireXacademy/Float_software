import socket_client as client
import time

time_start = time.time()

client.add_packet(f"Time: {time.time()-time_start}, Depth: 1.5 m")
time.sleep(1)
client.add_packet(f"Time: {time.time()-time_start}, Depth: 1.1 m")
time.sleep(1)
client.connect()
client.send_packets()
client.disconnect()
client.add_packet(f"Time: {time.time()-time_start}, Depth: 0.4 m")
time.sleep(1)
client.add_packet(f"Time: {time.time()-time_start}, Depth: 0.0 m")
client.add_packet("done")
client.connect()
client.send_packets()
client.disconnect()