# naza2frsky_S.Port

This project was initially created by alezz in this Thread:
https://www.rcgroups.com/forums/showthread.php?t=2197099

The goal is to get Information of the DJI Phantom's Naza Flight controller, or to be more specific from the GPS module.
The Information is then provided for a FrSky S.Port for sending telemetry back to the transmitter.

Just connect the Tx line of the GPS Module to the Rx Module of the arduino and the S.Port to pin 9.

Transmitted are:
- Altitude
- Speed
- Variometry
- Number of Satelites (as T2)
- Battery Voltage
- Heading
