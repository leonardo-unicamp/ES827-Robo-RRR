import socket

class Ev3Client:

    def __init__(self, host: str = "169.254.21.9", port: int = 12345):
        self.host = host
        self.port = port
        self.client = self.connect()

    def connect(self):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            client.connect((self.host, self.port))
            print(f"Connected to the server {self.host}:{self.port}")
            return client
        except ConnectionRefusedError:
            print(f"Error: was not possible connect to {self.host}:{self.port}")
            return None

    def set_position(self, j1, j2, j3, j4):

        try:
            msg = f"#{j1};{j2};{j3};{j4}".encode("ASCII")
            self.client.sendall(msg)
        except:
            print(f"Error: was not possible send the point to Ev3")

    def close(self):
        self.client.close()

if __name__ == "__main__":
    ev3 = Ev3Client()
    ev3.set_position(0,-10,20,-150)