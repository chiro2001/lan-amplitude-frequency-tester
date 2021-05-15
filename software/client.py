import socket


def start_tcp_client(ip, port, handler=None):
    # server port and ip
    server_ip = ip
    server_port = port
    tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        tcp_client.connect((server_ip, server_port))
    except socket.error:
        print('fail to setup socket connection')
    if handler is not None:
        while True:
            data = tcp_client.recv(4096)
            handler(data)
    tcp_client.close()


def u16_to_v(val):
    return float(val) * 3.3 / 65535


def branch(data: int):
    print(u16_to_v(data))


def handle_data(data):
    s = data.decode()
    if len(s) == 0:
        return
    sp = s.split('\r\n')
    for p in sp:
        if len(p) == 0:
            continue
        branch(int(p))


def main():
    start_tcp_client('192.168.137.240', 80, handler=handle_data)
    # start_tcp_client('127.0.0.1', 8000, handler=handle_data)


if __name__ == '__main__':
    main()
