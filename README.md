

CycleCount := CycleCount + 1;


// 1. Activer le port UDP 5000
UDP_Peer(
    xEnable := EnableUDP, //TRUE,
    itfIPAddress := myIPAddress,
    uiPort := 5001
);

// 2. Recevoir les donnees
UDP_Receive(
    xEnable := UDP_Peer.xActive,
    itfPeer := UDP_Peer.itfPeer,
    pData := ADR(RxBuffer),
    udiSize := SIZEOF(RxBuffer)
);

IF UDP_Receive.xReady THEN
    ReceptionCount := ReceptionCount + 1;
END_IF;
// 3. Verifier la reception
DataReceived := UDP_Receive.xReady;
BytesReceived := UDP_Receive.udiCount;
