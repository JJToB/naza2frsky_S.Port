/*
Version 0.3
Naza2Frsky
based on: 
NazaToFrSky is a simple Naza to FrSky Telemetry Adapter based on Teensy 3.1 https://code.google.com/p/nazatofrsky/
and
NAZADECODER from http://www.rcgroups.com/forums/showthread.php?t=1995704 Added some snipets from Airmamaf (Bagaosd https://code.google.com/p/bagaosd/wiki/NazaConnector)
and
APM MavLink to FrSky X8R S.Port converter from http://diydrones.ning.com/forum/topics/amp-to-frsky-x8r-sport-converter?commentId=705844%3AComment%3A1539864&xg_source=activity

NOTE: Some Parts of the code are NOT for commercial use (NAZADECODER)!!

*/
#include "config.h"
#include "mPwmInt.h"
#include "FrSkySPort.h"

#ifdef GPS_PROTOCOL_DJI
// see http://www.rcgroups.com/forums/showthread.php?t=1995704
#include "NazaDecoderLib.h"
#endif

#ifdef GPS_PROTOCOL_UBX
// see http://openpilot.org
#include "GPS_UBX.h"
#endif

int timer = 0;
int ledpin = 13;

//----------------------FrSkySPort------------------------------
// Message #24  GPS_RAW_INT 
uint8_t    gps_fixtype = 0;                  //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    sat_visible = 0;           // numbers of visible satelites
uint16_t   ap_gps_hdop = 255;             // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
// FrSky Taranis uses the first recieved lat/long as homeposition. 
static int32_t    latitude = 0;              // 585522540;
static int32_t    longitude = 0;            // 162344467;
double    gps_altitude = 0;        // 100 = 1m
double    home_altitude = 0;    // 100 = 1m
int32_t longitude2 =0;
int32_t latitude2 =0;

int32_t gps_hour = 0;
int32_t gps_min = 0;
int32_t gps_sec = 0;
int32_t gps_year = 0;
int32_t gps_month = 0;
int32_t gps_day = 0;

//int32_t gps_altitude2 = 0;
// Message #74 VFR_HUD 
uint32_t  groundspeed = 0;
uint32_t  heading = 0;
// FrSky Taranis uses the first recieved value after 'PowerOn' or  'Telemetry Reset'  as zero altitude
int32_t    climb_rate=0;        // 100= 1m/s
// These are special for FrSky
int32_t     gps_status = 0;     // (ap_sat_visible * 10) + ap_fixtype                                             // ex. 83 = 8 sattelites visible, 3D lock 

// Messages needed to use current Angles and axis speeds
// Message #0  HEARTHBEAT 
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
int32_t    ap_custom_mode = -1;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

// Message #30 ATTITUDE           //MAVLINK_MSG_ID_ATTITUDE
uint32_t ap_roll_angle = 0;    //Roll angle (rad)
uint32_t ap_pitch_angle = 0;   //Pitch angle (rad)
uint32_t ap_yaw_angle = 0;     //Yaw angle (rad)
uint32_t ap_roll_speed = 0;    //Roll angular speed (rad/s)
uint32_t ap_pitch_speed = 0;   //Pitch angular speed (rad/s)
uint32_t ap_yaw_speed = 0;     //Yaw angular speed (rad/s)


// Message #253 MAVLINK_MSG_ID_STATUSTEXT
uint16_t   ap_status_severity = 255;
uint16_t   ap_status_send_count = 0;
uint16_t   ap_status_text_id = 0;


void setup()
{
#ifdef DEBUG_CONSOLE
  ConsoleSerial.begin(9600);
#endif  
  
  FrSkySPort_Init();

#ifdef GPS_PROTOCOL_DJI
  GPS_Serial.begin(115200);
#endif  
#ifdef GPS_PROTOCOL_UBX
  GPS_Serial.begin(GPS_UBX_BAUD);
#endif
  
  pwm_int_init();

  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin,HIGH);
}

void x_setup(){
  FrSkySPort_Init();
  Serial.begin(9600);
}
void x_loop(){ 
  FrSkySPort_Process();
}

void loop()
{                                                  
#ifdef GPS_PROTOCOL_DJI
  // Process Naza Informations
  if(GPS_Serial.available())
  {
    uint8_t decodedMessage = NazaDecoder.decode(GPS_Serial.read());
    switch (decodedMessage)
    {
      case NAZA_MESSAGE_GPS:
        gps_fixtype = NazaDecoder.getFixType();          // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix  
        sat_visible =  NazaDecoder.getNumSat();          // numbers of visible satelites
        gps_status = (sat_visible*10) + gps_fixtype; 
        if(gps_fixtype == 3)  {
          latitude = NazaDecoder.getLat();
          longitude = NazaDecoder.getLon();

          gps_hour = NazaDecoder.getHour();
	  if (gps_hour < 5) gps_hour += 16; // NOTE 2: for the time between 16:00 and 23:59 the hour will be returned as 0-7 (could not find the way to differentiate here).
          gps_hour += time_zone;
          gps_hour = gps_hour%24;

          gps_min = NazaDecoder.getMinute();
          gps_sec = NazaDecoder.getSecond();
          gps_year = NazaDecoder.getYear();
          gps_month = NazaDecoder.getMonth();
          gps_day = NazaDecoder.getDay();

          gps_altitude = NazaDecoder.getAlt();    // 1m =1000
          if ((home_altitude == 0) && (sat_visible > 5)) home_altitude = gps_altitude;
          groundspeed = NazaDecoder.getSpeed();
          climb_rate = NazaDecoder.getClimbSpeed();
        }       
        break;
      case NAZA_MESSAGE_COMPASS:
        heading=NazaDecoder.getHeading();
        break;
      case NAZA_MESSAGE_VER:  //ID 30 which contains GPS module version numbers, sent every 2s  
         break;
    }
  }
#endif

#ifdef GPS_PROTOCOL_UBX
  // Process GPS Informations
  if(GPS_Serial.available() > 0)
  {
    uint8_t c = GPS_Serial.read();
    if (parse_ubx(c) == PARSER_COMPLETE_SET) {
      gps_fixtype	= get_ubx_status();
      sat_visible      	= get_ubx_satellites();
      gps_status        = (sat_visible*10) + gps_fixtype;
      latitude		= (int32_t)(get_ubx_latitude()*10000000);
      longitude		= (int32_t)(get_ubx_longitude()*10000000);
      gps_altitude	= get_ubx_altitude();
      if ((home_altitude == 0) && (sat_visible > 5)) home_altitude = gps_altitude;
      heading   	= get_ubx_heading();
      groundspeed 	= get_ubx_groundspeed();
      climb_rate	= -get_ubx_down();
      
      gps_sec = (int32_t)(get_ubx_itow()/1000)%60;
      gps_min = (int32_t)(get_ubx_itow()/1000/60)%60;
      gps_hour = (int32_t)(get_ubx_itow()/1000/60/60)%24;          
    }
  }
#endif

  FrSkySPort_Process();               // Process Informations and send it to FrSky
 
  // DBEUG
#ifdef DEBUG_CONSOLE  
if(timer == DEBUG_Timer)
{
 if(gps_fixtype == 3 )
 {
  ConsoleSerial.print("Lat: "); ConsoleSerial.print(latitude, 7);
  ConsoleSerial.print(", Lon: "); ConsoleSerial.print(longitude, 7);
  ConsoleSerial.print(", Alt: "); ConsoleSerial.print(gps_altitude, 7);
  ConsoleSerial.print(", Fix: "); ConsoleSerial.print(gps_fixtype);
  ConsoleSerial.print(", Sat: "); ConsoleSerial.println(sat_visible); 
  ConsoleSerial.print("Speed: "); ConsoleSerial.println(groundspeed);
  ConsoleSerial.print("climb_rate: "); ConsoleSerial.println(climb_rate);
 }
 ConsoleSerial.print("Heading: "); ConsoleSerial.println(NazaDecoder.getHeading(), 2);  
 timer=0;
}
else
{
  timer++;
}
#endif
} 

