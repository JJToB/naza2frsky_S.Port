//#include "mSoftwareSerial.h"

#define GPS_PROTOCOL_DJI
//#define GPS_PROTOCOL_UBX
#define GPS_UBX_BAUD         38400

//Serial Interfaces
#if defined(GPS_PROTOCOL_DJI) || defined(GPS_PROTOCOL_UBX)
  #define GPS_Serial              Serial // serial port: 0 (RX), 1 (TX)
#endif

//SoftwareSerial _FrSkySPort_Serial(9, 9,true); // RX, TX

#define time_zone  2

//#define  DEBUG_CONSOLE
//DEBUG
#ifdef DEBUG_CONSOLE
  SoftwareSerial ConsoleSerial(6, 6,true); // RX, TX
  #define DEBUG_Timer 1000 //"Delay" timer to reduce output....
#endif
