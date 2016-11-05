/**
 ******************************************************************************
 *
 * @file       GPS_UBX.ino
 * @author     Joerg-D. Rothfuchs
 * @brief      GPS module, handles GPS UBX stream
 *             Taken from OpenPilot Modules: UBX.c
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

#include "GPS_UBX.h"


static char gps_rx_buffer[sizeof(struct UBXPacket)];
static struct GPS_VALUES GPSValues;
static struct GPS_RX_STATS gpsRxStats;


// Keep track of various GPS messages needed to make up a single UAVO update
// time-of-week timestamp is used to correlate matching messages

#define POSLLH_RECEIVED (1 << 0)
#define STATUS_RECEIVED (1 << 1)
#define DOP_RECEIVED    (1 << 2)
#define VELNED_RECEIVED (1 << 3)
#define SOL_RECEIVED    (1 << 4)
#define ALL_RECEIVED    (SOL_RECEIVED | POSLLH_RECEIVED | VELNED_RECEIVED)
#define NONE_RECEIVED   0

static struct msgtracker {
    uint32_t currentTOW;   // TOW of the message set currently in progress
    uint8_t  msg_received; // keep track of received message types
} msgtracker;

// Check if a message belongs to the current data set and register it as 'received'
bool check_msgtracker(uint32_t tow, uint8_t msg_flag)
{
    if (tow > msgtracker.currentTOW ? true 				// start of a new message set
        : (msgtracker.currentTOW - tow > 6 * 24 * 3600 * 1000)) {	// 6 days, TOW wrap around occured
        msgtracker.currentTOW   = tow;
        msgtracker.msg_received = NONE_RECEIVED;
    } else if (tow < msgtracker.currentTOW) {				// message outdated (don't process)
        return false;
    }

    msgtracker.msg_received |= msg_flag;				// register reception of this msg type
    return true;
}


void parse_ubx_nav_sol(struct UBX_NAV_SOL *sol)
{
    if (check_msgtracker(sol->iTOW, SOL_RECEIVED)) {
        GPSValues.Satellites = sol->numSV;
        if (sol->flags & STATUS_FLAGS_GPSFIX_OK) {
            switch (sol->gpsFix) {
            case STATUS_GPSFIX_2DFIX:
                GPSValues.Status = GPSPOSITIONSENSOR_STATUS_FIX2D;
                break;
            case STATUS_GPSFIX_3DFIX:
                GPSValues.Status = GPSPOSITIONSENSOR_STATUS_FIX3D;
                break;
            default: GPSValues.Status = GPSPOSITIONSENSOR_STATUS_NOFIX;
            }
        } else { // fix is not valid so we make sure to treat it as NOFIX
            GPSValues.Status = GPSPOSITIONSENSOR_STATUS_NOFIX;
        }
    }
}


void parse_ubx_nav_posllh(struct UBX_NAV_POSLLH *posllh)
{
    if (check_msgtracker(posllh->iTOW, POSLLH_RECEIVED)) {
        if (GPSValues.Status != GPSPOSITIONSENSOR_STATUS_NOFIX) {
            GPSValues.Altitude  = (float)posllh->hMSL * 0.001f;
            GPSValues.Latitude  = (float)posllh->lat / 10000000.0;
            GPSValues.Longitude = (float)posllh->lon / 10000000.0;
            GPSValues.iTow = (float)posllh->iTOW;
        }
    }
}


void parse_ubx_nav_velned(struct UBX_NAV_VELNED *velned)
{
    if (check_msgtracker(velned->iTOW, VELNED_RECEIVED)) {
        if (GPSValues.Status != GPSPOSITIONSENSOR_STATUS_NOFIX) {
            GPSValues.Down        = (float)velned->velD / 100.0f;
            GPSValues.Groundspeed = (float)velned->gSpeed * 0.01f;
            GPSValues.Heading     = (float)velned->heading * 1.0e-5f;
        }
    }
}


bool checksum_ubx_message(struct UBXPacket *ubx)
{
    int i;
    uint8_t ck_a, ck_b;

    ck_a  = ubx->header.class_nav;
    ck_b  = ck_a;

    ck_a += ubx->header.id;
    ck_b += ck_a;

    ck_a += ubx->header.len & 0xff;
    ck_b += ck_a;

    ck_a += ubx->header.len >> 8;
    ck_b += ck_a;

    for (i = 0; i < ubx->header.len; i++) {
        ck_a += ubx->payload.payload[i];
        ck_b += ck_a;
    }

    if (ubx->header.ck_a == ck_a &&
        ubx->header.ck_b == ck_b) {
        return true;
    } else {
        return false;
    }
}


// UBX message parser
void parse_ubx_message(struct UBXPacket *ubx)
{
    switch (ubx->header.class_nav) {
    case UBX_CLASS_NAV:
        switch (ubx->header.id) {
        case UBX_ID_SOL:
            parse_ubx_nav_sol(&ubx->payload.nav_sol);
            break;
        case UBX_ID_POSLLH:
            parse_ubx_nav_posllh(&ubx->payload.nav_posllh);
            break;
        case UBX_ID_VELNED:
            parse_ubx_nav_velned(&ubx->payload.nav_velned);
            break;
        }
        break;
    }
}


// parse incoming character stream for messages in UBX binary format
int parse_ubx_stream(uint8_t c, char *gps_rx_buffer, struct GPS_RX_STATS *gpsRxStats)
{
    enum proto_states {
        START,
        UBX_SY2,
        UBX_CLASS,
        UBX_ID,
        UBX_LEN1,
        UBX_LEN2,
        UBX_PAYLOAD,
        UBX_CHK1,
        UBX_CHK2,
        FINISHED
    };

    static enum proto_states proto_state = START;
    static uint8_t rx_count = 0;
    struct UBXPacket *ubx   = (struct UBXPacket *)gps_rx_buffer;

    switch (proto_state) {
    case START:							// detect protocol
        if (c == UBX_SYNC1) {					// first UBX sync char found
            proto_state = UBX_SY2;
        }
        break;
    case UBX_SY2:
        if (c == UBX_SYNC2) {					// second UBX sync char found
            proto_state = UBX_CLASS;
        } else {
            proto_state = START;				// reset state
        }
        break;
    case UBX_CLASS:
        ubx->header.class_nav = c;
        proto_state      = UBX_ID;
        break;
    case UBX_ID:
        ubx->header.id   = c;
        proto_state      = UBX_LEN1;
        break;
    case UBX_LEN1:
        ubx->header.len  = c;
        proto_state      = UBX_LEN2;
        break;
    case UBX_LEN2:
        ubx->header.len += (c << 8);
        if (ubx->header.len > sizeof(UBXPayload)) {
            gpsRxStats->gpsRxOverflow++;
            proto_state = START;
        } else {
            rx_count    = 0;
            proto_state = UBX_PAYLOAD;
        }
        break;
    case UBX_PAYLOAD:
        if (rx_count < ubx->header.len) {
            ubx->payload.payload[rx_count] = c;
            if (++rx_count == ubx->header.len) {
                proto_state = UBX_CHK1;
            }
        } else {
            gpsRxStats->gpsRxOverflow++;
            proto_state = START;
        }
        break;
    case UBX_CHK1:
        ubx->header.ck_a = c;
        proto_state = UBX_CHK2;
        break;
    case UBX_CHK2:
        ubx->header.ck_b = c;
        if (checksum_ubx_message(ubx)) {			// message complete and valid
            parse_ubx_message(ubx);
            proto_state = FINISHED;
        } else {
            gpsRxStats->gpsRxChkSumError++;
            proto_state = START;
        }
        break;
    default: break;
    }

    if (proto_state == START) {
        return PARSER_ERROR;					// parser couldn't use this byte
    } else if (proto_state == FINISHED) {
        gpsRxStats->gpsRxReceived++;
        proto_state = START;
	if (msgtracker.msg_received == ALL_RECEIVED) {
		msgtracker.msg_received = NONE_RECEIVED;
		return PARSER_COMPLETE_SET;			// message set complete & processed
	} else {
		return PARSER_COMPLETE;				// message complete & processed
	}
    }

    return PARSER_INCOMPLETE;					// message not (yet) complete
}


// wrapper und getter

int parse_ubx(uint8_t c)
{
	return parse_ubx_stream(c, gps_rx_buffer, &gpsRxStats);
}

uint8_t get_ubx_status(void)
{
	return GPSValues.Status;
}

uint8_t get_ubx_satellites(void)
{
	return GPSValues.Satellites;
}

float get_ubx_latitude(void)
{
	return GPSValues.Latitude;
}

float get_ubx_longitude(void)
{
	return GPSValues.Longitude;
}

float get_ubx_itow(void)
{
	return GPSValues.iTow;
}

float get_ubx_altitude(void)
{
	return GPSValues.Altitude;
}

float get_ubx_heading(void)
{
	return GPSValues.Heading;
}

float get_ubx_groundspeed(void)
{
	return GPSValues.Groundspeed;
}

float get_ubx_down(void)
{
	return GPSValues.Down;
}


#endif // GPS_PROTOCOL_UBX
