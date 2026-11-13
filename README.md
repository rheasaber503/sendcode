# sendcode


python3 - <<'PY'
import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("192.168.50.10", 0))

for i in range(200):
    s.sendto(b"TEST1234", ("192.168.50.2", 5001))
    time.sleep(0.05)

s.close()
print("200 paquets envoyés")
PY
