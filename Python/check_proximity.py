import socket

sock = socket.create_connection(("127.0.0.1", 5556))
sock.sendall(b'{"command":"get_proximity"}\n')
raw = sock.recv(4096)
print(repr(raw))
sock.close()