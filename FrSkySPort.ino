#include "FrSkySPort.h"
#include <SoftwareSerial.h>

SoftwareSerial _FrSkySPort_Serial(9, 9,true); // RX, TX
//#define _FrSkySPort_C1                UART0_C1
//#define _FrSkySPort_C3                UART0_C3
//#define _FrSkySPort_S2                UART0_S2
#define _FrSkySPort_BAUD              57600

short crc;                         // used for crc calc of frsky-packet
boolean waitingForSensorId = false;
uint8_t cell_count = 0;
uint8_t latlong_flag = 0;
uint32_t latlong = 0;

uint8_t nextFLVSS = 0;
uint8_t nextFAS = 0;
uint8_t nextVARIO = 0;
uint8_t nextGPS = 0;
uint8_t nextDefault = 0;
uint8_t nextSP2UR = 0;
uint8_t nextRPM = 0;
// ***********************************************************************
void FrSkySPort_Init(void)  {
  _FrSkySPort_Serial.begin(_FrSkySPort_BAUD);
 // _FrSkySPort_C3 = 0x10;            // Tx invert
  //_FrSkySPort_C1= 0xA0;            // Single wire mode
 // _FrSkySPort_S2 = 0x10;           // Rx Invert

}

// ***********************************************************************
void FrSkySPort_Process(void) {
  uint8_t data = 0;
  uint32_t temp=0;
  uint8_t offset;
  while ( _FrSkySPort_Serial.available()) 
  {
    data =  _FrSkySPort_Serial.read();

    if(data == START_STOP)
    {
      waitingForSensorId = true; 
      continue; 
    }
    if(!waitingForSensorId)
      continue;

    FrSkySPort_ProcessSensorRequest(data);

    waitingForSensorId = false;
  }
}

// ***********************************************************************
uint16_t sendValueFlvssVoltage = 0;
uint16_t sendValueFASCurrent = 0;
uint16_t sendValueFASVoltage = 0;
void FrSkySPort_ProcessSensorRequest(uint8_t sensorId) 
{
  uint32_t temp=0;
  uint8_t offset;
    //Serial.write(sensorId);
      uint32_t gps_time_date;

  switch(sensorId)
        {
            // ***********SENSORS***********    
#if defined(GPS_PROTOCOL_DJI) || defined(GPS_PROTOCOL_UBX)
            case SENSOR_ID_GPS:
          { 
                 switch(nextGPS)
              {
                 case 0:        // Sends the latitude value, setting bit 31 low
                       if(gps_fixtype > 2) {
                         latitude2 = (int32_t)(latitude / 100 * 6);
                         if(latitude2 < 0)
                             latlong = (-latitude2) | 0x40000000;
                             else
                             latlong = latitude2;
                         FrSkySPort_SendPackage(FR_ID_LATLONG,latlong);
                         }
                 break;
                 case 1:        // Sends the longitude value, setting bit 31 high  
                     if(gps_fixtype > 2) {
                       longitude2 =(int32_t)(longitude / 100 * 6);
                       if(longitude2 < 0)
                           latlong=(-longitude2) | 0xC0000000;
                           else
                           latlong=(longitude2) | 0x80000000;
                       FrSkySPort_SendPackage(FR_ID_LATLONG,latlong);
                       }
                 break;  
                 case 2:
                     if(gps_fixtype==3) { 
                       FrSkySPort_SendPackage(FR_ID_GPS_ALT,gps_altitude * 100);   // from GPS,  100=1m
                    }
                    
                 break;
                 case 3:
                   // Note: This is sending GPS Speed now
                   if(gps_fixtype==3) {
                           FrSkySPort_SendPackage(FR_ID_SPEED, groundspeed * 36); 
                     //FrSkySPort_SendPackage(FR_ID_AIR_SPEED_FIRST,groundspeed *20 );  // from GPS converted to km/h
                   }
                 break;
                 case 4:
                   // before we were sending this: FrSkySPort_SendPackage(FR_ID_HEADING,ap_cog * 100); 
                         FrSkySPort_SendPackage(FR_ID_HEADING,heading * 100);   // 10000 = 100 deg MODIFIED
                 break;
                 case 5: 
                         //GPS Satellites
                         FrSkySPort_SendPackage(FR_ID_T2, sat_visible); 
                 break;
            
              }
                    if(++nextGPS > 6)
                nextGPS = 0;
            }
            break;  //case SENSOR_ID_GPS    
  
            case SENSOR_ID_VARIO:
      {
            printDebugPackageSend("VARIO", nextVARIO+1, 2);
            switch(nextVARIO)
            {
              case 0:
                      FrSkySPort_SendPackage(FR_ID_VARIO, climb_rate);        
              break;
              case 1: 
                      FrSkySPort_SendPackage(FR_ID_ALTITUDE, (gps_altitude - home_altitude) * 100);   // from barometer, 100 = 1m
              break;
            }
            if(++nextVARIO > 1)
              nextVARIO = 0;
      }
            break;  //SENSOR_ID_VARIO:
#endif        

            case SENSOR_ID_SP2UR:
            {
              switch(nextSP2UR)
              {
                case 0:  // Note: gps_status = (sat_visible*10) + gps_fixtype
                  FrSkySPort_SendPackage(FR_ID_ADC2, gps_status);                  
                break;       
                case 1:
                  //FrSkySPort_SendPackage(FR_ID_ACCX, analogRead(A1));    
                break;
                case 2:
                 //FrSkySPort_SendPackage(FR_ID_ACCY, analogRead(A2)); 
                break; 
                case 3:
                    //FrSkySPort_SendPackage(FR_ID_ACCZ, analogRead(A3)); 
                break; 
                case 4:
                  //FrSkySPort_SendPackage(FR_ID_T1, motor1_percent_get());  //Motor1  
                break; 
                case 5:
                  //FrSkySPort_SendPackage(FR_ID_T2, motor2_percent_get());  //Motor2
                break;
                case 6:
                  //FrSkySPort_SendPackage(FR_ID_A3_FIRST,analogRead(A3));
                break;
                case 7:
                        //FrSkySPort_SendPackage(FR_ID_A4_FIRST, analogRead(A4)); 
                break;
                case 8:
                        //FrSkySPort_SendPackage(FR_ID_FUEL, motor4_percent_get());  //Motor4 
                      break;
              }
              if(++nextSP2UR > 8)
                nextSP2UR = 0;
       } 
             break; //case SENSOR_ID_SP2UR        
  
         }
}

// ***********************************************************************
void printDebugPackageSend(char* pkg_name, uint8_t pkg_nr, uint8_t pkg_max)
{
#ifdef DEBUG_FRSKY_SENSOR_REQUEST

#endif
}


// ***********************************************************************
void FrSkySPort_SendByte(uint8_t byte) {

  _FrSkySPort_Serial.write(byte);

  // CRC update
  crc += byte;         //0-1FF
  crc += crc >> 8;   //0-100
  crc &= 0x00ff;
  crc += crc >> 8;   //0-0FF
  crc &= 0x00ff;
}


// ***********************************************************************
void FrSkySPort_SendCrc() {
  _FrSkySPort_Serial.write(0xFF-crc);
  crc = 0;          // CRC reset
}


// ***********************************************************************
void FrSkySPort_SendPackage(uint16_t id, uint32_t value) {

 // _FrSkySPort_C3 |= 32;      //  Transmit direction, to S.Port
  FrSkySPort_SendByte(DATA_FRAME);
  uint8_t *bytes = (uint8_t*)&id;
  FrSkySPort_SendByte(bytes[0]);
  FrSkySPort_SendByte(bytes[1]);
  bytes = (uint8_t*)&value;
  FrSkySPort_SendByte(bytes[0]);
  FrSkySPort_SendByte(bytes[1]);
  FrSkySPort_SendByte(bytes[2]);
  FrSkySPort_SendByte(bytes[3]);
  FrSkySPort_SendCrc();
  _FrSkySPort_Serial.flush();
 // _FrSkySPort_C3 ^= 32;      // Transmit direction, from S.Port

}
