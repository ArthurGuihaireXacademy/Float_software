import socket
import queue
import pickle
import time
packet_queue = queue.Queue()
port = 9876

def connect(ip):
    global client
    while True:
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((ip, port))
        except Exception as e:
            print(e)
            print("Connection failed, trying again in 1 second...")
            time.sleep(1)
        else:
            break

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
        disconnect()
        connect()
        send_packets()

def disconnect():
    client.close()