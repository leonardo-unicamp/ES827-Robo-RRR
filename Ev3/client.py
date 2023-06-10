import socket
from time import sleep, time


def echo_client(host, port, msg):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        client_socket.connect((host, port))
        print("Connected to the server {}:{}".format(host, port))

        
        while True:
            value = input()
            msg = f"#{value};0;0;0".encode("ASCII")
            # sleep(0.06)
            client_socket.sendall(msg)


    except ConnectionRefusedError:
        print("Error: Connection refused.")

    finally:
        # Close the client socket
        client_socket.close()


if __name__ == "__main__":
    host = "localhost"
    host = "10.42.0.3"
    port = 12345
    msg = b"#1;2;3;4"
    echo_client(host, port, msg)
