import socket
import pickle
from graph_data import graph_data
import time
host, port = '0.0.0.0', 9876
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', port))
server.listen()

data_array = []

ping_packet = pickle.dumps(0)

def handle_connection():
    conn, addr = server.accept()
    print("Connected")
    with conn:
        while True:
            try:
                data_packet = conn.recv(1024)
                if not data_packet: return False
                data_packet = pickle.loads(data_packet)
                if data_packet == "done":
                    return True
                print(data_packet)
                data_array.append(data_packet)
                conn.send(ping_packet)
            except Exception as e:
                print(e)
                time.sleep(0.5)

while True:
    if handle_connection():
        break

time_array = []
depth_array = []
print("Deconstructed values:")
for packet in data_array:
    time_str, depth_str = packet.split(", ")
    time_value = float(time_str.split(": ")[1])
    time_array.append(time_value)
    depth_value = float(depth_str.split(": ")[1].replace(" m", ""))
    depth_array.append(depth_value)

graph_data(time_array, depth_array)