import socket


class Com:
    def __init__(self, host, port):
        self.host = host
        self.port = port

    def receive(self, buff_size):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            while True:
                conn, addr = s.accept()
                with conn:
                    while True:
                        data = conn.recv(buff_size)
                        if not data:
                            break
                        yield data


def main() -> None:
    HOST = "Localhost"
    PORT = 12345
    com = Com(HOST, PORT)
    rec = com.receive(1024)
    while True:
        print(next(rec))
        print("recebido")


if __name__ == "__main__":
    main()
