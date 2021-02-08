// dump1090, a Mode S messages decoder for RTLSDR devices.
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  *  Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//  *  Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "ppup1090.h"
//
// ===================== Mode S detection and decoding  ===================
//
// Parity table for MODE S Messages.
// The table contains 112 elements, every element corresponds to a bit set
// in the message, starting from the first bit of actual data after the
// preamble.
//
// For messages of 112 bit, the whole table is used.
// For messages of 56 bits only the last 56 elements are used.
//
// The algorithm is as simple as xoring all the elements in this table
// for which the corresponding bit on the message is set to 1.
//
// The latest 24 elements in this table are set to 0 as the checksum at the
// end of the message should not affect the computation.
//
// Note: this function can be used with DF11 and DF17, other modes have
// the CRC xored with the sender address as they are reply to interrogations,
// but a casual listener can't split the address from the checksum.
//
uint32_t modes_checksum_table[112] = {
0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};

uint32_t modesChecksum(unsigned char *msg, int bits) {
    uint32_t   crc = 0;
    uint32_t   rem = 0;
    int        offset = (bits == 112) ? 0 : (112-56);
    uint8_t    theByte = *msg;
    uint32_t * pCRCTable = &modes_checksum_table[offset];
    int j;

    // We don't really need to include the checksum itself
    bits -= 24;
    for(j = 0; j < bits; j++) {
        if ((j & 7) == 0)
            theByte = *msg++;

        // If bit is set, xor with corresponding table entry.
        if (theByte & 0x80) {crc ^= *pCRCTable;} 
        pCRCTable++;
        theByte = theByte << 1; 
    }

    rem = (msg[0] << 16) | (msg[1] << 8) | msg[2]; // message checksum
    return ((crc ^ rem) & 0x00FFFFFF); // 24 bit checksum syndrome.
}
//
//=========================================================================
//
// Given the Downlink Format (DF) of the message, return the message length in bits.
//
// All known DF's 16 or greater are long. All known DF's 15 or less are short. 
// There are lots of unused codes in both category, so we can assume ICAO will stick to 
// these rules, meaning that the most significant bit of the DF indicates the length.
//
int modesMessageLenByType(int type) {
    return (type & 0x10) ? MODES_LONG_MSG_BITS : MODES_SHORT_MSG_BITS ;
}
//
//=========================================================================
//
// Hash the ICAO address to index our cache of MODES_ICAO_CACHE_LEN
// elements, that is assumed to be a power of two
//
uint32_t ICAOCacheHashAddress(uint32_t a) {
    // The following three rounds wil make sure that every bit affects
    // every output bit with ~ 50% of probability.
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a);
    return a & (MODES_ICAO_CACHE_LEN-1);
}
//
//=========================================================================
//
// Add the specified entry to the cache of recently seen ICAO addresses.
// Note that we also add a timestamp so that we can make sure that the
// entry is only valid for MODES_ICAO_CACHE_TTL seconds.
//
void addRecentlySeenICAOAddr(uint32_t addr) {
    uint32_t h = ICAOCacheHashAddress(addr);
    Modes.icao_cache[h*2] = addr;
    Modes.icao_cache[h*2+1] = (uint32_t) time(NULL);
}
//
//=========================================================================
//
// Returns 1 if the specified ICAO address was seen in a DF format with
// proper checksum (not xored with address) no more than * MODES_ICAO_CACHE_TTL
// seconds ago. Otherwise returns 0.
//
int ICAOAddressWasRecentlySeen(uint32_t addr) {
    uint32_t h = ICAOCacheHashAddress(addr);
    uint32_t a = Modes.icao_cache[h*2];
    uint32_t t = Modes.icao_cache[h*2+1];
    uint64_t tn = time(NULL);

    return ( (a) && (a == addr) && ( (tn - t) <= MODES_ICAO_CACHE_TTL) );
}
//
//=========================================================================
//
// In the squawk (identity) field bits are interleaved as follows in
// (message bit 20 to bit 32):
//
// C1-A1-C2-A2-C4-A4-ZERO-B1-D1-B2-D2-B4-D4
//
// So every group of three bits A, B, C, D represent an integer from 0 to 7.
//
// The actual meaning is just 4 octal numbers, but we convert it into a hex 
// number tha happens to represent the four octal numbers.
//
// For more info: http://en.wikipedia.org/wiki/Gillham_code
//
int decodeID13Field(int ID13Field) {
    int hexGillham = 0;

    if (ID13Field & 0x1000) {hexGillham |= 0x0010;} // Bit 12 = C1
    if (ID13Field & 0x0800) {hexGillham |= 0x1000;} // Bit 11 = A1
    if (ID13Field & 0x0400) {hexGillham |= 0x0020;} // Bit 10 = C2
    if (ID13Field & 0x0200) {hexGillham |= 0x2000;} // Bit  9 = A2
    if (ID13Field & 0x0100) {hexGillham |= 0x0040;} // Bit  8 = C4
    if (ID13Field & 0x0080) {hexGillham |= 0x4000;} // Bit  7 = A4
  //if (ID13Field & 0x0040) {hexGillham |= 0x0800;} // Bit  6 = X  or M 
    if (ID13Field & 0x0020) {hexGillham |= 0x0100;} // Bit  5 = B1 
    if (ID13Field & 0x0010) {hexGillham |= 0x0001;} // Bit  4 = D1 or Q
    if (ID13Field & 0x0008) {hexGillham |= 0x0200;} // Bit  3 = B2
    if (ID13Field & 0x0004) {hexGillham |= 0x0002;} // Bit  2 = D2
    if (ID13Field & 0x0002) {hexGillham |= 0x0400;} // Bit  1 = B4
    if (ID13Field & 0x0001) {hexGillham |= 0x0004;} // Bit  0 = D4

    return (hexGillham);
    }
//
//=========================================================================
//
// Decode the 13 bit AC altitude field (in DF 20 and others).
// Returns the altitude, and set 'unit' to either MODES_UNIT_METERS or MDOES_UNIT_FEETS.
//
int decodeAC13Field(int AC13Field, int *unit) {
    int m_bit  = AC13Field & 0x0040; // set = meters, clear = feet
    int q_bit  = AC13Field & 0x0010; // set = 25 ft encoding, clear = Gillham Mode C encoding

    if (!m_bit) {
        *unit = MODES_UNIT_FEET;
        if (q_bit) {
            // N is the 11 bit integer resulting from the removal of bit Q and M
            int n = ((AC13Field & 0x1F80) >> 2) |
                    ((AC13Field & 0x0020) >> 1) |
                     (AC13Field & 0x000F);
            // The final altitude is resulting number multiplied by 25, minus 1000.
            return ((n * 25) - 1000);
        } else {
            // N is an 11 bit Gillham coded altitude
            int n = ModeAToModeC(decodeID13Field(AC13Field));
            if (n < -12) {n = 0;}

            return (100 * n);
        }
    } else {
        *unit = MODES_UNIT_METERS;
        // TODO: Implement altitude when meter unit is selected
    }
    return 0;
}
//
//=========================================================================
//
// Decode the 12 bit AC altitude field (in DF 17 and others).
//
int decodeAC12Field(int AC12Field, int *unit) {
    int q_bit  = AC12Field & 0x10; // Bit 48 = Q

    *unit = MODES_UNIT_FEET;
    if (q_bit) {
        /// N is the 11 bit integer resulting from the removal of bit Q at bit 4
        int n = ((AC12Field & 0x0FE0) >> 1) | 
                 (AC12Field & 0x000F);
        // The final altitude is the resulting number multiplied by 25, minus 1000.
        return ((n * 25) - 1000);
    } else {
        // Make N a 13 bit Gillham coded altitude by inserting M=0 at bit 6
        int n = ((AC12Field & 0x0FC0) << 1) | 
                 (AC12Field & 0x003F);
        n = ModeAToModeC(decodeID13Field(n));
        if (n < -12) {n = 0;}

        return (100 * n);
    }
}
//
//=========================================================================
//
// Decode the 7 bit ground movement field PWL exponential style scale
//
int decodeMovementField(int movement) {
    int gspeed;

    // Note : movement codes 0,125,126,127 are all invalid, but they are 
    //        trapped for before this function is called.

    if      (movement  > 123) gspeed = 199; // > 175kt
    else if (movement  > 108) gspeed = ((movement - 108)  * 5) + 100;
    else if (movement  >  93) gspeed = ((movement -  93)  * 2) +  70;
    else if (movement  >  38) gspeed = ((movement -  38)     ) +  15;
    else if (movement  >  12) gspeed = ((movement -  11) >> 1) +   2;
    else if (movement  >   8) gspeed = ((movement -   6) >> 2) +   1;
    else                      gspeed = 0;

    return (gspeed);
}
//
//=========================================================================
//
// Decode a raw Mode S message demodulated as a stream of bytes by detectModeS(), 
// and split it into fields populating a modesMessage structure.
//
void decodeModesMessage(struct modesMessage *mm, unsigned char *msg) {
    char *ais_charset = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

    // Work on our local copy
    memcpy(mm->msg, msg, MODES_LONG_MSG_BYTES);
    msg = mm->msg;

    // Get the message type ASAP as other operations depend on this
    mm->msgtype         = msg[0] >> 3; // Downlink Format
    mm->msgbits         = modesMessageLenByType(mm->msgtype);
    mm->crc             = modesChecksum(msg, mm->msgbits);

    //
    // Note that most of the other computation happens *after* we fix the 
    // single/two bit errors, otherwise we would need to recompute the fields again.
    //
    if (mm->msgtype == 11) { // DF 11
        mm->iid   =  mm->crc;
        mm->addr  = (msg[1] << 16) | (msg[2] << 8) | (msg[3]); 
        mm->ca    = (msg[0] & 0x07); // Responder capabilities

        if ((mm->crcok = (0 == mm->crc))) {
            // DF 11 : if crc == 0 try to populate our ICAO addresses whitelist.
            addRecentlySeenICAOAddr(mm->addr);
        } else if (mm->crc < 80) {
            mm->crcok = ICAOAddressWasRecentlySeen(mm->addr);
            if (mm->crcok) {
                addRecentlySeenICAOAddr(mm->addr);
            }
        }

    } else if (mm->msgtype == 17) { // DF 17
        mm->addr  = (msg[1] << 16) | (msg[2] << 8) | (msg[3]); 
        mm->ca    = (msg[0] & 0x07); // Responder capabilities

        if ((mm->crcok = (0 == mm->crc))) {
            // DF 17 : if crc == 0 try to populate our ICAO addresses whitelist.
            addRecentlySeenICAOAddr(mm->addr);
        }

    } else if (mm->msgtype == 18) { // DF 18
        mm->addr  = (msg[1] << 16) | (msg[2] << 8) | (msg[3]); 
        mm->ca    = (msg[0] & 0x07); // Control Field

        if ((mm->crcok = (0 == mm->crc))) {
            // DF 18 : if crc == 0 try to populate our ICAO addresses whitelist.
            addRecentlySeenICAOAddr(mm->addr);
        }

    } else { // All other DF's
        // Compare the checksum with the whitelist of recently seen ICAO 
        // addresses. If it matches one, then declare the message as valid
        mm->crcok = ICAOAddressWasRecentlySeen(mm->addr = mm->crc);
    }

    // If we're checking CRC and the CRC is invalid, then we can't trust any 
    // of the data contents, so save time and give up now.
    if (!mm->crcok) { return;}

    // Fields for DF0, DF16
    if (mm->msgtype == 0  || mm->msgtype == 16) {
        if (msg[0] & 0x04) {                       // VS Bit
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
        } else {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
        }
    }

    // Fields for DF11, DF17
    if (mm->msgtype == 11 || mm->msgtype == 17) {
        if (mm->ca == 4) {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
        } else if (mm->ca == 5) {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
        }
    }
          
    // Fields for DF5, DF21 = Gillham encoded Squawk
    if (mm->msgtype == 5  || mm->msgtype == 21) {
        int ID13Field = ((msg[2] << 8) | msg[3]) & 0x1FFF; 
        if (ID13Field) {
            mm->bFlags |= MODES_ACFLAGS_SQUAWK_VALID;
            mm->modeA   = decodeID13Field(ID13Field);
        }
    }

    // Fields for DF0, DF4, DF16, DF20 13 bit altitude
    if (mm->msgtype == 0  || mm->msgtype == 4 ||
        mm->msgtype == 16 || mm->msgtype == 20) {
        int AC13Field = ((msg[2] << 8) | msg[3]) & 0x1FFF; 
        if (AC13Field) { // Only attempt to decode if a valid (non zero) altitude is present
            mm->bFlags  |= MODES_ACFLAGS_ALTITUDE_VALID;
            mm->altitude = decodeAC13Field(AC13Field, &mm->unit);
        }
    }

    // Fields for DF4, DF5, DF20, DF21
    if ((mm->msgtype == 4) || (mm->msgtype == 20) ||
        (mm->msgtype == 5) || (mm->msgtype == 21)) {
        mm->bFlags  |= MODES_ACFLAGS_FS_VALID;
        mm->fs       = msg[0]  & 7;               // Flight status for DF4,5,20,21
        if (mm->fs <= 3) {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
            if (mm->fs & 1)
                {mm->bFlags |= MODES_ACFLAGS_AOG;}
        }
    }

    // Fields for DF17, DF18_CF0, DF18_CF1, DF18_CF6 squitters
    if (  (mm->msgtype == 17) 
      || ((mm->msgtype == 18) && ((mm->ca == 0) || (mm->ca == 1) || (mm->ca == 6)) )) {
         int metype = mm->metype = msg[4] >> 3;   // Extended squitter message type
         int mesub  = mm->mesub  = (metype == 29 ? ((msg[4]&6)>>1) : (msg[4]  & 7));   // Extended squitter message subtype

        // Decode the extended squitter message

        if (metype >= 1 && metype <= 4) { // Aircraft Identification and Category
            uint32_t chars;
            mm->bFlags |= MODES_ACFLAGS_CALLSIGN_VALID;

            chars = (msg[5] << 16) | (msg[6] << 8) | (msg[7]);
            mm->flight[3] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[2] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[1] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[0] = ais_charset[chars & 0x3F];

            chars = (msg[8] << 16) | (msg[9] << 8) | (msg[10]);
            mm->flight[7] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[6] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[5] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[4] = ais_charset[chars & 0x3F];

            mm->flight[8] = '\0';

        } else if (metype == 19) { // Airborne Velocity Message

           // Presumably airborne if we get an Airborne Velocity Message
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID; 

            if ( (mesub >= 1) && (mesub <= 4) ) {
                int vert_rate = ((msg[8] & 0x07) << 6) | (msg[9] >> 2);
                if (vert_rate) {
                    --vert_rate;
                    if (msg[8] & 0x08) 
                      {vert_rate = 0 - vert_rate;}
                    mm->vert_rate =  vert_rate * 64;
                    mm->bFlags   |= MODES_ACFLAGS_VERTRATE_VALID;
                }
            }

            if ((mesub == 1) || (mesub == 2)) {
                int ew_raw = ((msg[5] & 0x03) << 8) |  msg[6];
                int ew_vel = ew_raw - 1;
                int ns_raw = ((msg[7] & 0x7F) << 3) | (msg[8] >> 5);
                int ns_vel = ns_raw - 1;

                if (mesub == 2) { // If (supersonic) unit is 4 kts
                   ns_vel = ns_vel << 2;
                   ew_vel = ew_vel << 2;
                }

                if (ew_raw) { // Do East/West  
                    mm->bFlags |= MODES_ACFLAGS_EWSPEED_VALID;
                    if (msg[5] & 0x04)
                        {ew_vel = 0 - ew_vel;}                   
                    mm->ew_velocity = ew_vel;
                }

                if (ns_raw) { // Do North/South
                    mm->bFlags |= MODES_ACFLAGS_NSSPEED_VALID;
                    if (msg[7] & 0x80)
                        {ns_vel = 0 - ns_vel;}                   
                    mm->ns_velocity = ns_vel;
                }

                if (ew_raw && ns_raw) {
                    // Compute velocity and angle from the two speed components
                    mm->bFlags |= (MODES_ACFLAGS_SPEED_VALID | MODES_ACFLAGS_HEADING_VALID | MODES_ACFLAGS_NSEWSPD_VALID);
                    mm->velocity = (int) sqrt((ns_vel * ns_vel) + (ew_vel * ew_vel));

                    if (mm->velocity) {
                        mm->heading = (int) (atan2(ew_vel, ns_vel) * 180.0 / M_PI);
                        // We don't want negative values but a 0-360 scale
                        if (mm->heading < 0) mm->heading += 360;
                    }
                }

            } else if (mesub == 3 || mesub == 4) {
                int airspeed = ((msg[7] & 0x7f) << 3) | (msg[8] >> 5);
                if (airspeed) {
                    mm->bFlags |= MODES_ACFLAGS_SPEED_VALID;
                    --airspeed;
                    if (mesub == 4)  // If (supersonic) unit is 4 kts
                        {airspeed = airspeed << 2;}
                    mm->velocity =  airspeed;
                }

                if (msg[5] & 0x04) {
                    mm->bFlags |= MODES_ACFLAGS_HEADING_VALID;
                    mm->heading = ((((msg[5] & 0x03) << 8) | msg[6]) * 45) >> 7;
                }
            }

        } else if (metype >= 5 && metype <= 22) { // Position Message
            mm->raw_latitude  = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1);
            mm->raw_longitude = ((msg[8] & 1) << 16) | (msg[9] << 8) | (msg[10]);
            mm->bFlags       |= (mm->msg[6] & 0x04) ? MODES_ACFLAGS_LLODD_VALID 
                                                    : MODES_ACFLAGS_LLEVEN_VALID;
            if (metype >= 9) {        // Airborne
                int AC12Field = ((msg[5] << 4) | (msg[6] >> 4)) & 0x0FFF;
                mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
                if (AC12Field) {// Only attempt to decode if a valid (non zero) altitude is present
                    mm->bFlags |= MODES_ACFLAGS_ALTITUDE_VALID;
                    mm->altitude = decodeAC12Field(AC12Field, &mm->unit);
                }
            } else {                      // Ground
                int movement = ((msg[4] << 4) | (msg[5] >> 4)) & 0x007F;
                mm->bFlags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
                if ((movement) && (movement < 125)) {
                    mm->bFlags |= MODES_ACFLAGS_SPEED_VALID;
                    mm->velocity = decodeMovementField(movement);
                }

                if (msg[5] & 0x08) {
                    mm->bFlags |= MODES_ACFLAGS_HEADING_VALID;
                    mm->heading = ((((msg[5] << 4) | (msg[6] >> 4)) & 0x007F) * 45) >> 4;
                }
            }

        } else if (metype == 23) {	// Test metype squawk field
			if (mesub == 7) {		// (see 1090-WP-15-20)
				int ID13Field = (((msg[5] << 8) | msg[6]) & 0xFFF1)>>3;
				if (ID13Field) {
					mm->bFlags |= MODES_ACFLAGS_SQUAWK_VALID;
					mm->modeA   = decodeID13Field(ID13Field);
				}
            }

        } else if (metype == 24) { // Reserved for Surface System Status

        } else if (metype == 28) { // Extended Squitter Aircraft Status
			if (mesub == 1) {      // Emergency status squawk field
				int ID13Field = (((msg[5] << 8) | msg[6]) & 0x1FFF);
				if (ID13Field) {
					mm->bFlags |= MODES_ACFLAGS_SQUAWK_VALID;
					mm->modeA   = decodeID13Field(ID13Field);
				}
            }

        } else if (metype == 29) { // Aircraft Trajectory Intent

        } else if (metype == 30) { // Aircraft Operational Coordination

        } else if (metype == 31) { // Aircraft Operational Status

        } else { // Other metypes

        }
    }

    // Fields for DF20, DF21 Comm-B
    if ((mm->msgtype == 20) || (mm->msgtype == 21)){

        if (msg[4] == 0x20) { // Aircraft Identification
            uint32_t chars;
            mm->bFlags |= MODES_ACFLAGS_CALLSIGN_VALID;

            chars = (msg[5] << 16) | (msg[6] << 8) | (msg[7]);
            mm->flight[3] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[2] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[1] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[0] = ais_charset[chars & 0x3F];

            chars = (msg[8] << 16) | (msg[9] << 8) | (msg[10]);
            mm->flight[7] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[6] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[5] = ais_charset[chars & 0x3F]; chars = chars >> 6;
            mm->flight[4] = ais_charset[chars & 0x3F];

            mm->flight[8] = '\0';
        } else {
        }
    }
}
//
//=========================================================================
//
// When a new message is available, because it was decoded from the RTL device, 
// file, or received in the TCP input port, or any other way we can receive a 
// decoded message, we call this function in order to use the message.
//
// Basically this function passes a raw message to the upper layers for further
// processing and visualization
//
void useModesMessage(struct modesMessage *mm) {
    if (mm->crcok) { // not checking, ok or fixed

        if (mm->msgtype < 33) {
          Modes.nDF[mm->msgtype]++;
        }

        // Always track aircraft
        interactiveReceiveData(mm);
    }
}
//
//=========================================================================
//
// Always positive MOD operation, used for CPR decoding.
//
int cprModFunction(int a, int b) {
    int res = a % b;
    if (res < 0) res += b;
    return res;
}
//
//=========================================================================
//
// The NL function uses the precomputed table from 1090-WP-9-14
//
int cprNLFunction(double lat) {
    if (lat < 0) lat = -lat; // Table is simmetric about the equator
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    else return 1;
}
//
//=========================================================================
//
int cprNFunction(double lat, int fflag) {
    int nl = cprNLFunction(lat) - (fflag ? 1 : 0);
    if (nl < 1) nl = 1;
    return nl;
}
//
//=========================================================================
//
double cprDlonFunction(double lat, int fflag, int surface) {
    return (surface ? 90.0 : 360.0) / cprNFunction(lat, fflag);
}
//
//=========================================================================
//
// This algorithm comes from:
// http://www.lll.lu/~edward/edward/adsb/DecodingADSBposition.html.
//
// A few remarks:
// 1) 131072 is 2^17 since CPR latitude and longitude are encoded in 17 bits.
//
int decodeCPR(struct aircraft *a, int fflag, int surface) {
    double AirDlat0 = (surface ? 90.0 : 360.0) / 60.0;
    double AirDlat1 = (surface ? 90.0 : 360.0) / 59.0;
    double lat0 = a->even_cprlat;
    double lat1 = a->odd_cprlat;
    double lon0 = a->even_cprlon;
    double lon1 = a->odd_cprlon;

    // Compute the Latitude Index "j"
    int    j     = (int) floor(((59*lat0 - 60*lat1) / 131072) + 0.5);
    double rlat0 = AirDlat0 * (cprModFunction(j,60) + lat0 / 131072);
    double rlat1 = AirDlat1 * (cprModFunction(j,59) + lat1 / 131072);

    time_t now = time(NULL);
    double surface_rlat = MODES_USER_LATITUDE_DFLT;
    double surface_rlon = MODES_USER_LONGITUDE_DFLT;

    if (surface) {
        // If we're on the ground, make sure we have a (likely) valid Lat/Lon
        if ((a->bFlags & MODES_ACFLAGS_LATLON_VALID) && (((int)(now - a->seenLatLon)) < Modes.interactive_display_ttl)) {
            surface_rlat = a->lat;
            surface_rlon = a->lon;
        } else if (Modes.bUserFlags & MODES_USER_LATLON_VALID) {
            surface_rlat = Modes.fUserLat;
            surface_rlon = Modes.fUserLon;
        } else {
            // No local reference, give up
            return (-1);
        }
        rlat0 += floor(surface_rlat / 90.0) * 90.0;  // Move from 1st quadrant to our quadrant
        rlat1 += floor(surface_rlat / 90.0) * 90.0;
    } else {
        if (rlat0 >= 270) rlat0 -= 360;
        if (rlat1 >= 270) rlat1 -= 360;
    }

    // Check to see that the latitude is in range: -90 .. +90
    if (rlat0 < -90 || rlat0 > 90 || rlat1 < -90 || rlat1 > 90)
        return (-1);

    // Check that both are in the same latitude zone, or abort.
    if (cprNLFunction(rlat0) != cprNLFunction(rlat1))
        return (-1);

    // Compute ni and the Longitude Index "m"
    if (fflag) { // Use odd packet.
        int ni = cprNFunction(rlat1,1);
        int m = (int) floor((((lon0 * (cprNLFunction(rlat1)-1)) -
                              (lon1 * cprNLFunction(rlat1))) / 131072.0) + 0.5);
        a->lon = cprDlonFunction(rlat1, 1, surface) * (cprModFunction(m, ni)+lon1/131072);
        a->lat = rlat1;
    } else {     // Use even packet.
        int ni = cprNFunction(rlat0,0);
        int m = (int) floor((((lon0 * (cprNLFunction(rlat0)-1)) -
                              (lon1 * cprNLFunction(rlat0))) / 131072) + 0.5);
        a->lon = cprDlonFunction(rlat0, 0, surface) * (cprModFunction(m, ni)+lon0/131072);
        a->lat = rlat0;
    }

    if (surface) {
        a->lon += floor(surface_rlon / 90.0) * 90.0;  // Move from 1st quadrant to our quadrant
    } else if (a->lon > 180) {
        a->lon -= 360;
    }

    a->seenLatLon      = a->seen;
    a->timestampLatLon = a->timestamp;
    a->bFlags         |= (MODES_ACFLAGS_LATLON_VALID | MODES_ACFLAGS_LATLON_REL_OK);

    return 0;
}
//
//=========================================================================
//
// This algorithm comes from:
// 1090-WP29-07-Draft_CPR101 (which also defines decodeCPR() )
//
// There is an error in this document related to CPR relative decode.
// Should use trunc() rather than the floor() function in Eq 38 and related for deltaZI.
// floor() returns integer less than argument
// trunc() returns integer closer to zero than argument.
// Note:   text of document describes trunc() functionality for deltaZI calculation
//         but the formulae use floor().
//
int decodeCPRrelative(struct aircraft *a, int fflag, int surface) {
    double AirDlat;
    double AirDlon;
    double lat;
    double lon;
    double lonr, latr;
    double rlon, rlat;
    int j,m;

    if (a->bFlags & MODES_ACFLAGS_LATLON_REL_OK) { // Ok to try aircraft relative first
        latr = a->lat;
        lonr = a->lon;
    } else if (Modes.bUserFlags & MODES_USER_LATLON_VALID) { // Try ground station relative next
        latr = Modes.fUserLat;
        lonr = Modes.fUserLon;
    } else {
        return (-1); // Exit with error - can't do relative if we don't have ref.
    }

    if (fflag) { // odd
        AirDlat = (surface ? 90.0 : 360.0) / 59.0;
        lat = a->odd_cprlat;
        lon = a->odd_cprlon;
    } else {    // even
        AirDlat = (surface ? 90.0 : 360.0) / 60.0;
        lat = a->even_cprlat;
        lon = a->even_cprlon;
    }

    // Compute the Latitude Index "j"
    j = (int) (floor(latr/AirDlat) +
               trunc(0.5 + cprModFunction((int)latr, (int)AirDlat)/AirDlat - lat/131072));
    rlat = AirDlat * (j + lat/131072);
    if (rlat >= 270) rlat -= 360;

    // Check to see that the latitude is in range: -90 .. +90
    if (rlat < -90 || rlat > 90) {
        a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
        return (-1);                               // Time to give up - Latitude error
    }

    // Check to see that answer is reasonable - ie no more than 1/2 cell away 
    if (fabs(rlat - a->lat) > (AirDlat/2)) {
        a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
        return (-1);                               // Time to give up - Latitude error 
    }

    // Compute the Longitude Index "m"
    AirDlon = cprDlonFunction(rlat, fflag, surface);
    m = (int) (floor(lonr/AirDlon) +
               trunc(0.5 + cprModFunction((int)lonr, (int)AirDlon)/AirDlon - lon/131072));
    rlon = AirDlon * (m + lon/131072);
    if (rlon > 180) rlon -= 360;

    // Check to see that answer is reasonable - ie no more than 1/2 cell away
    if (fabs(rlon - a->lon) > (AirDlon/2)) {
        a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
        return (-1);                               // Time to give up - Longitude error
    }

    a->lat = rlat;
    a->lon = rlon;

    a->seenLatLon      = a->seen;
    a->timestampLatLon = a->timestamp;
    a->bFlags         |= (MODES_ACFLAGS_LATLON_VALID | MODES_ACFLAGS_LATLON_REL_OK);
    return (0);
}
//
// ===================== Mode S detection and decoding  ===================
//
