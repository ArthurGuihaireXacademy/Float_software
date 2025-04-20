import socket_client as client

ip = "192.168.1.160"

client.connect(ip)
client.add_packet("1")
client.add_packet("2")
client.add_packet("3")
client.send_packets()
client.disconnect()