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



UDP_Receive (FB)
FUNCTION_BLOCK PUBLIC FINAL UDP_Receive EXTENDS LConC

This function block serves to receive data.
The pointer pData specifies the memory area to write the read data to. udiSize defines the max number of bytes to read. When data was read successfully, xReady is set to TRUE and udiCount indicates the actually read number of bytes. itfIPAddressFrom and uiPortFrom indicate the ip and port the data was received from.
InOut:
Scope	Name	Type	Comment	Inherited from
Input	xEnable	BOOL	TRUE: Activates the defined operation FALSE: Aborts/resets the defined operation	LConC
Output	xBusy	BOOL	TRUE: Operation is running	LConC
xError	BOOL	TRUE: Error condition reached	LConC
eErrorID	ERROR	 	LConC
Input	itfPeer	IPeer	 	 
pData	__XWORD	 	 
udiSize	UDINT	 	 
Output	xReady	BOOL	 	 
itfIPAddressFrom	IIPAddress	 	 
uiPortFrom	UINT	 	 
udiCount
