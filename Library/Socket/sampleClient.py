import time

from SocketCommon import EmbdTcpClient

HOST = "192.168.0.184"  # The server's hostname or IP address
PORT = 7506  # The port used by the server

if __name__ == '__main__':
    sampleClient = EmbdTcpClient()
    sampleClient.connect_to_server(HOST, PORT)
    while True:
        sampleClient.sendData("Hello, world!")
        time.sleep(1)