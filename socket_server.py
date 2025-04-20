import socket
import pickle
import graph_data
host, port = '0.0.0.0', 9876
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', port))
server.listen()

data_array = []

ping_packet = pickle.dumps(0)

def handle_connection():
    conn, addr = server.accept()
    print("Connected")
    while True:
        try:
            data_packet = pickle.loads(conn.recv(1024))
            if data_packet == "done":
                break
            print(data_packet)
            data_array.append(data_packet)
            conn.send(ping_packet)
        except Exception as e:
            print(e)
            handle_connection()

handle_connection()

graph_data