import socket

HOST = '0.0.0.0'
PORT = 8000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
while True:
    conn, addr = s.accept()
    print('Connected by', addr)
    while True:
        data = input()
        if len(data) == 0:
            break
        conn.sendall(data.encode())

# conn.close()
