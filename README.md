VAR
    PeerActive : BOOL;
    PeerError : BOOL;
    ReceiveBusy : BOOL;
    ReceiveError : BOOL;
    ReceiveErrorID : ERROR;
END_VAR








CycleCount := CycleCount + 1;

// 1. Ouvrir le port UDP
UDP_Peer(
    xEnable := EnableUDP,
    itfIPAddress := myIPAddress,
    uiPort := 5001
);

// Diagnostic du Peer
PeerActive := UDP_Peer.xActive;
PeerError := UDP_Peer.xError;

// 2. Recevoir les données
UDP_Receive(
    xEnable := UDP_Peer.xActive,
    itfPeer := UDP_Peer.itfPeer,
    pData := ADR(RxBuffer),
    udiSize := SIZEOF(RxBuffer)
);

// Diagnostic de la réception
ReceiveBusy := UDP_Receive.xBusy;
ReceiveError := UDP_Receive.xError;
ReceiveErrorID := UDP_Receive.eErrorID;

// 3. Compteur de réception
IF UDP_Receive.xReady THEN
    ReceptionCount := ReceptionCount + 1;
END_IF;

DataReceived := UDP_Receive.xReady;
BytesReceived := UDP_Receive.udiCount;
