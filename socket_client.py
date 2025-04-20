import socket
import queue
import pickle
packet_queue = queue.Queue()
host, port = '127.0.0.1', 9876
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((host, port))

def add_packet(data_packet):
    packet_queue.put(data_packet)

def send_packets():
    try:
        while not packet_queue.empty():
            data_packet = pickle.dumps(packet_queue.get())
            client.sendall(data_packet)
            client.recv(32)

    except Exception as e:
        print(e)
        send_packets()

add_packet("Depth: 0.5 m")
add_packet("Depth: 1.0 m")
add_packet("Depth: 1.5 m")
send_packets()
client.close()