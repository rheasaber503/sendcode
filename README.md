PROGRAM PLC_PRG
VAR
    // 1. Network configuration (leave IP as '0.0.0.0' to listen on all local interfaces)
    ipLocalAddr   : NBS.IPv4Address := (ipAddress := '0.0.0.0');
    uiListenPort  : UINT := 5001;

    // 2. Net Base Services Function Blocks
    fbUdpPeer     : NBS.UDP_Peer;
    fbUdpReceive  : NBS.UDP_Receive;

    // 3. Receive buffer & decoded values
    aRxBuffer     : ARRAY[0..7] OF BYTE;
    uConvVx       : U_BytesToReal;
    uConvWz       : U_BytesToReal;

    rVx           : REAL; // Linear velocity [m/s]
    rWz           : REAL; // Angular velocity [rad/s]
    udiRxCount    : UDINT;
END_VAR



// Step 1: Open and bind the local UDP socket on port 5001
fbUdpPeer(
    xEnable      := TRUE,
    itfIPAddress := ipLocalAddr,
    uiPort       := uiListenPort
);

// Step 2: Continuously listen once the peer is active
fbUdpReceive(
    xEnable := fbUdpPeer.xActive,
    itfPeer := fbUdpPeer.itfPeer,
    pData   := ADR(aRxBuffer),
    udiSize := SIZEOF(aRxBuffer)
);

// Step 3: When a full 8-byte packet arrives, swap Big-Endian bytes to Little-Endian REAL
IF fbUdpReceive.xReady AND (fbUdpReceive.udiCount = 8) THEN
    // Swap bytes 0..3 for vx (Big-Endian -> Little-Endian)
    uConvVx.aBytes[0] := aRxBuffer[3];
    uConvVx.aBytes[1] := aRxBuffer[2];
    uConvVx.aBytes[2] := aRxBuffer[1];
    uConvVx.aBytes[3] := aRxBuffer[0];
    rVx := uConvVx.rValue;

    // Swap bytes 4..7 for wz (Big-Endian -> Little-Endian)
    uConvWz.aBytes[0] := aRxBuffer[7];
    uConvWz.aBytes[1] := aRxBuffer[6];
    uConvWz.aBytes[2] := aRxBuffer[5];
    uConvWz.aBytes[3] := aRxBuffer[4];
    rWz := uConvWz.rValue;

    udiRxCount := udiRxCount + 1;
END_IF
