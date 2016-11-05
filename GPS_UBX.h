/**
 ******************************************************************************
 *
 * @file       GPS_UBX.h
 * @author     Joerg-D. Rothfuchs
 * @brief      GPS module, handles GPS UBX stream
 *             Taken from OpenPilot Modules: UBX.h
 *             Code refactored because of Atmel 328p memory restrictions
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
 

#ifdef GPS_PROTOCOL_UBX

#ifndef GPS_UBX_H
#define GPS_UBX_H


#define GPSPOSITIONSENSOR_STATUS_NOGPS	0
#define GPSPOSITIONSENSOR_STATUS_NOFIX	1
#define GPSPOSITIONSENSOR_STATUS_FIX2D	2
#define GPSPOSITIONSENSOR_STATUS_FIX3D	3

struct GPS_VALUES {
    uint32_t	iTow;
    uint8_t	Status;
    uint8_t	Satellites;
    float	Latitude;
    float	Longitude;
    float	Altitude;
    float	Heading;
    float	Groundspeed;
    float	Down;
};


#define NO_PARSER          -3 // no parser available
#define PARSER_OVERRUN     -2 // message buffer overrun before completing the message
#define PARSER_ERROR       -1 // message unparsable by this parser
#define PARSER_INCOMPLETE   0 // parser needs more data to complete the message
#define PARSER_COMPLETE     1 // parser has received a complete message and finished processing
#define PARSER_COMPLETE_SET 2 // parser has received a complete message set and finished processing

struct GPS_RX_STATS {
    uint16_t gpsRxReceived;
    uint16_t gpsRxChkSumError;
    uint16_t gpsRxOverflow;
    uint16_t gpsRxParserError;
};


#define UBX_SYNC1      0xb5  // UBX protocol synchronization characters
#define UBX_SYNC2      0x62

// From u-blox6 receiver protocol specification

// Messages classes
#define UBX_CLASS_NAV  0x01

// Message IDs
#define UBX_ID_POSLLH  0x02
#define UBX_ID_STATUS  0x03
#define UBX_ID_DOP     0x04
#define UBX_ID_SOL     0x06
#define UBX_ID_VELNED  0x12
#define UBX_ID_TIMEUTC 0x21
#define UBX_ID_SVINFO  0x30

// private structures

// Geodetic Position Solution
struct UBX_NAV_POSLLH {
    uint32_t iTOW;   // GPS Millisecond Time of Week (ms)
    int32_t  lon;    // Longitude (deg*1e-7)
    int32_t  lat;    // Latitude (deg*1e-7)
    int32_t  height; // Height above Ellipsoid (mm)
    int32_t  hMSL;   // Height above mean sea level (mm)
    uint32_t hAcc;   // Horizontal Accuracy Estimate (mm)
    uint32_t vAcc;   // Vertical Accuracy Estimate (mm)
};

// Receiver Navigation Status

#define STATUS_GPSFIX_NOFIX    0x00
#define STATUS_GPSFIX_DRONLY   0x01
#define STATUS_GPSFIX_2DFIX    0x02
#define STATUS_GPSFIX_3DFIX    0x03
#define STATUS_GPSFIX_GPSDR    0x04
#define STATUS_GPSFIX_TIMEONLY 0x05

#define STATUS_FLAGS_GPSFIX_OK (1 << 0)
#define STATUS_FLAGS_DIFFSOLN  (1 << 1)
#define STATUS_FLAGS_WKNSET    (1 << 2)
#define STATUS_FLAGS_TOWSET    (1 << 3)

struct UBX_NAV_STATUS {
    uint32_t iTOW;    // GPS Millisecond Time of Week (ms)
    uint8_t  gpsFix;  // GPS fix type
    uint8_t  flags;   // Navigation Status Flags
    uint8_t  fixStat; // Fix Status Information
    uint8_t  flags2;  // Additional navigation output information
    uint32_t ttff;    // Time to first fix (ms)
    uint32_t msss;    // Milliseconds since startup/reset (ms)
};

// Dilution of precision
struct UBX_NAV_DOP {
    uint32_t iTOW; // GPS Millisecond Time of Week (ms)
    uint16_t gDOP; // Geometric DOP
    uint16_t pDOP; // Position DOP
    uint16_t tDOP; // Time DOP
    uint16_t vDOP; // Vertical DOP
    uint16_t hDOP; // Horizontal DOP
    uint16_t nDOP; // Northing DOP
    uint16_t eDOP; // Easting DOP
};

// Navigation solution
struct UBX_NAV_SOL {
    uint32_t iTOW;      // GPS Millisecond Time of Week (ms)
    int32_t  fTOW;      // fractional nanoseconds (ns)
    int16_t  week;      // GPS week
    uint8_t  gpsFix;    // GPS fix type
    uint8_t  flags;     // Fix status flags
    int32_t  ecefX;     // ECEF X coordinate (cm)
    int32_t  ecefY;     // ECEF Y coordinate (cm)
    int32_t  ecefZ;     // ECEF Z coordinate (cm)
    uint32_t pAcc;      // 3D Position Accuracy Estimate (cm)
    int32_t  ecefVX;    // ECEF X coordinate (cm/s)
    int32_t  ecefVY;    // ECEF Y coordinate (cm/s)
    int32_t  ecefVZ;    // ECEF Z coordinate (cm/s)
    uint32_t sAcc;      // Speed Accuracy Estimate
    uint16_t pDOP;      // Position DOP
    uint8_t  reserved1; // Reserved
    uint8_t  numSV;     // Number of SVs used in Nav Solution
    uint32_t reserved2; // Reserved
};

// North/East/Down velocity
struct UBX_NAV_VELNED {
    uint32_t iTOW;    // ms GPS Millisecond Time of Week
    int32_t  velN;    // cm/s NED north velocity
    int32_t  velE;    // cm/s NED east velocity
    int32_t  velD;    // cm/s NED down velocity
    uint32_t speed;   // cm/s Speed (3-D)
    uint32_t gSpeed;  // cm/s Ground Speed (2-D)
    int32_t  heading; // 1e-5 *deg Heading of motion 2-D
    uint32_t sAcc;    // cm/s Speed Accuracy Estimate
    uint32_t cAcc;    // 1e-5 *deg Course / Heading Accuracy Estimate
};

// UTC Time Solution
#define TIMEUTC_VALIDTOW (1 << 0)
#define TIMEUTC_VALIDWKN (1 << 1)
#define TIMEUTC_VALIDUTC (1 << 2)
struct UBX_NAV_TIMEUTC {
    uint32_t iTOW;  // GPS Millisecond Time of Week (ms)
    uint32_t tAcc;  // Time Accuracy Estimate (ns)
    int32_t  nano;  // Nanoseconds of second
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid; // Validity Flags
};

// Space Vehicle (SV) Information
// Single SV information block
#define SVUSED     (1 << 0) // This SV is used for navigation
#define DIFFCORR   (1 << 1) // Differential correction available
#define ORBITAVAIL (1 << 2) // Orbit information available
#define ORBITEPH   (1 << 3) // Orbit information is Ephemeris
#define UNHEALTHY  (1 << 4) // SV is unhealthy
#define ORBITALM   (1 << 5) // Orbit information is Almanac Plus
#define ORBITAOP   (1 << 6) // Orbit information is AssistNow Autonomous
#define SMOOTHED   (1 << 7) // Carrier smoothed pseudoranges used
struct UBX_NAV_SVINFO_SV {
    uint8_t chn;     // Channel number
    uint8_t svid;    // Satellite ID
    uint8_t flags;   // Misc SV information
    uint8_t quality; // Misc quality indicators
    uint8_t cno;     // Carrier to Noise Ratio (dbHz)
    int8_t  elev;    // Elevation (integer degrees)
    int16_t azim;    // Azimuth	(integer degrees)
    int32_t prRes;   // Pseudo range residual (cm)
};

// SV information message
#define MAX_SVS 16
struct UBX_NAV_SVINFO {
    uint32_t iTOW;        // GPS Millisecond Time of Week (ms)
    uint8_t  numCh;       // Number of channels
    uint8_t  globalFlags; //
    uint16_t reserved2;   // Reserved
    struct UBX_NAV_SVINFO_SV sv[MAX_SVS]; // Repeated 'numCh' times
};

typedef union {
    uint8_t payload[0];
    struct UBX_NAV_POSLLH  nav_posllh;
    struct UBX_NAV_STATUS  nav_status;
    struct UBX_NAV_DOP     nav_dop;
    struct UBX_NAV_SOL     nav_sol;
    struct UBX_NAV_VELNED  nav_velned;
#if !defined(PIOS_GPS_MINIMAL)
    struct UBX_NAV_TIMEUTC nav_timeutc;
    struct UBX_NAV_SVINFO  nav_svinfo;
#endif
} UBXPayload;

struct UBXHeader {
    uint8_t  class_nav;
    uint8_t  id;
    uint16_t len;
    uint8_t  ck_a;
    uint8_t  ck_b;
};

struct UBXPacket {
    struct UBXHeader header;
    UBXPayload payload;
};


int parse_ubx(uint8_t);
uint8_t get_ubx_status(void);
uint8_t get_ubx_satellites(void);
float get_ubx_latitude(void);
float get_ubx_longitude(void);
float get_ubx_altitude(void);
float get_ubx_heading(void);
float get_ubx_groundspeed(void);
float get_ubx_down(void);

#endif /* GPS_UBX_H */

#endif // GPS_PROTOCOL_UBX
