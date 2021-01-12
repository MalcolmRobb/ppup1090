// ppup1090, a Mode S PlanePlotter Uploader for dump1090 devices.
//
// Copyright (C) 2013 by Malcolm Robb <Support@ATTAvionics.com>
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
#ifndef __PPUP1090_H
#define __PPUP1090_H

// File Version number
// ====================
// Format is : MajorVer.MinorVer.DayMonth.Year"
// MajorVer changes only with significant changes
// MinorVer changes when additional features are added, but not for bug fixes (range 00-99)
// DayDate & Year changes for all changes, including for bug fixes. It represent the release date of the update
//
#define MODES_PPUP1090_VERSION     "1.11.0601.21"

// ============================= Include files ==========================

#ifndef _WIN32
    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include <pthread.h>
    #include <stdint.h>
    #include <errno.h>
    #include <unistd.h>
    #include <math.h>
    #include <sys/time.h>
    #include <sys/timeb.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <arpa/inet.h>
    #include <signal.h>
    #include <fcntl.h>
    #include <ctype.h>
    #include <sys/stat.h>
    #include <sys/ioctl.h>
    #include "rtl-sdr.h"
    #include "anet.h"
    #include <netdb.h>
#else
    #include "winstubs.h" //Put everything Windows specific in here
    #include "anet.h"
#endif

// ============================= #defines ===============================
//
// If you have a valid coaa.h, these values will come from it. If not,
// then you can enter your own values in the #else section here
//
#ifdef USER_LATITUDE
    #define MODES_USER_LATITUDE_DFLT   (USER_LATITUDE)
    #define MODES_USER_LONGITUDE_DFLT  (USER_LONGITUDE)
#else
    #define MODES_USER_LATITUDE_DFLT   (0.0)
    #define MODES_USER_LONGITUDE_DFLT  (0.0)
#endif

#define MODEAC_MSG_BYTES          2
#define MODEAC_MSG_SQUELCH_LEVEL  0x07FF                      // Average signal strength limit
#define MODEAC_MSG_FLAG          (1<<0)
#define MODEAC_MSG_MODES_HIT     (1<<1)
#define MODEAC_MSG_MODEA_HIT     (1<<2)
#define MODEAC_MSG_MODEC_HIT     (1<<3)
#define MODEAC_MSG_MODEA_ONLY    (1<<4)
#define MODEAC_MSG_MODEC_OLD     (1<<5)

#define MODES_LONG_MSG_BYTES     14
#define MODES_SHORT_MSG_BYTES    7
#define MODES_LONG_MSG_BITS     (MODES_LONG_MSG_BYTES    * 8)
#define MODES_SHORT_MSG_BITS    (MODES_SHORT_MSG_BYTES   * 8)

#define MODES_ICAO_CACHE_LEN 1024 // Power of two required
#define MODES_ICAO_CACHE_TTL 60   // Time to live of cached addresses
#define MODES_UNIT_FEET 0
#define MODES_UNIT_METERS 1

#define MODES_USER_LATLON_VALID (1<<0)

#define MODES_ACFLAGS_LATLON_VALID   (1<<0)  // Aircraft Lat/Lon is decoded
#define MODES_ACFLAGS_ALTITUDE_VALID (1<<1)  // Aircraft altitude is known
#define MODES_ACFLAGS_HEADING_VALID  (1<<2)  // Aircraft heading is known
#define MODES_ACFLAGS_SPEED_VALID    (1<<3)  // Aircraft speed is known
#define MODES_ACFLAGS_VERTRATE_VALID (1<<4)  // Aircraft vertical rate is known
#define MODES_ACFLAGS_SQUAWK_VALID   (1<<5)  // Aircraft Mode A Squawk is known
#define MODES_ACFLAGS_CALLSIGN_VALID (1<<6)  // Aircraft Callsign Identity
#define MODES_ACFLAGS_EWSPEED_VALID  (1<<7)  // Aircraft East West Speed is known
#define MODES_ACFLAGS_NSSPEED_VALID  (1<<8)  // Aircraft North South Speed is known
#define MODES_ACFLAGS_AOG            (1<<9)  // Aircraft is On the Ground
#define MODES_ACFLAGS_LLEVEN_VALID   (1<<10) // Aircraft Even Lot/Lon is known
#define MODES_ACFLAGS_LLODD_VALID    (1<<11) // Aircraft Odd Lot/Lon is known
#define MODES_ACFLAGS_AOG_VALID      (1<<12) // MODES_ACFLAGS_AOG is valid
#define MODES_ACFLAGS_FS_VALID       (1<<13) // Aircraft Flight Status is known
#define MODES_ACFLAGS_NSEWSPD_VALID  (1<<14) // Aircraft EW and NS Speed is known
#define MODES_ACFLAGS_LATLON_REL_OK  (1<<15) // Indicates it's OK to do a relative CPR

#define MODES_ACFLAGS_LLEITHER_VALID (MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID)
#define MODES_ACFLAGS_LLBOTH_VALID   (MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID)
#define MODES_ACFLAGS_AOG_GROUND     (MODES_ACFLAGS_AOG_VALID    | MODES_ACFLAGS_AOG)

#define MODES_INTERACTIVE_DELETE_TTL   300      // Delete from the list after 300 seconds
#define MODES_INTERACTIVE_DISPLAY_TTL   60      // Delete from display after 60 seconds

#define MODES_NET_OUTPUT_BEAST_PORT 30005
#define MODES_CLIENT_BUF_SIZE  1024

#define PPUP1090_NET_OUTPUT_IP_ADDRESS "127.0.0.1"

#define NOTUSED(V) ((void) V)

#define STR_HELPER(x)         #x
#define STR(x)                STR_HELPER(x)

// ======================== structure declarations ========================

// Structure used to describe a networking client
struct client {
    int    fd;                           // File descriptor
    int    buflen;                       // Amount of data in buffer
    char   buf[MODES_CLIENT_BUF_SIZE+1]; // Read buffer
};

// Structure used to describe an aircraft in iteractive mode
struct aircraft {
    uint32_t      addr;           // ICAO address
    char          flight[16];     // Flight number
    unsigned char signalLevel[8]; // Last 8 Signal Amplitudes
    int           altitude;       // Altitude
    int           speed;          // Velocity
    int           track;          // Angle of flight
    int           vert_rate;      // Vertical rate.
    time_t        seen;           // Time at which the last packet was received
    time_t        seenLatLon;     // Time at which the last lat long was calculated
    uint64_t      timestamp;      // Timestamp at which the last packet was received
    uint64_t      timestampLatLon;// Timestamp at which the last lat long was calculated
    long          messages;       // Number of Mode S messages received
    int           modeA;          // Squawk
    int           modeC;          // Altitude
    long          modeAcount;     // Mode A Squawk hit Count
    long          modeCcount;     // Mode C Altitude hit Count
    int           modeACflags;    // Flags for mode A/C recognition

    // Encoded latitude and longitude as extracted by odd and even CPR encoded messages
    int           odd_cprlat;
    int           odd_cprlon;
    int           even_cprlat;
    int           even_cprlon;
    uint64_t      odd_cprtime;
    uint64_t      even_cprtime;
    double        lat, lon;       // Coordinated obtained from CPR encoded data
    int           bFlags;         // Flags related to valid fields in this structure
    struct aircraft *next;        // Next aircraft in our linked list
};

struct stDF {
    struct stDF     *pNext;                      // Pointer to next item in the linked list
    struct stDF     *pPrev;                      // Pointer to previous item in the linked list
    struct aircraft *pAircraft;                  // Pointer to the Aircraft structure for this DF
    time_t           seen;                       // Dos/UNIX Time at which the this packet was received
    uint64_t         llTimestamp;                // Timestamp at which the this packet was received
    uint32_t         addr;                       // Timestamp at which the this packet was received
    unsigned char    msg[MODES_LONG_MSG_BYTES];  // the binary
} tDF;

// Program global state
struct {                             // Internal state
    pthread_t       reader_thread;

    pthread_mutex_t data_mutex;      // Mutex to synchronize buffer access
    pthread_cond_t  data_cond;       // Conditional variable associated
    uint32_t       *icao_cache;      // Recently seen ICAO addresses cache
    int             exit;            // Exit from the main loop when true

    // Networking
    char           aneterr[ANET_ERR_LEN];
#ifdef _WIN32
    WSADATA        wsaData;          // Windows socket initialisation
#endif

    // Configuration
    int   mode_ac;                   // Enable decoding of SSR Modes A & C
    int   net_input_beast_port;      // Beast input TCP port
    int   interactive_display_ttl;   // Interactive mode: TTL display
    int   interactive_delete_ttl;    // Interactive mode: TTL before deletion

    // User details
    double fUserLat;                // Users receiver/antenna lat/lon needed for initial surface location
    double fUserLon;                // Users receiver/antenna lat/lon needed for initial surface location
    int    bUserFlags;              // Flags relating to the user details

    // Interactive mode
    struct aircraft *aircrafts;
    uint64_t         interactive_last_update; // Last screen update in milliseconds
    time_t           last_cleanup_time;       // Last cleanup time in seconds

    // DF List mode
    pthread_mutex_t pDF_mutex;        // Mutex to synchronize pDF access
    struct stDF    *pDF;              // Pointer to DF list
} Modes;

// The struct we use to store information about a decoded message.
struct modesMessage {
    // Generic fields
    unsigned char msg[MODES_LONG_MSG_BYTES];      // Binary message.
    int           msgbits;                        // Number of bits in message 
    int           msgtype;                        // Downlink format #
    int           crcok;                          // True if CRC was valid
    uint32_t      crc;                            // Message CRC
    uint32_t      addr;                           // ICAO Address from bytes 1 2 and 3
    uint64_t      timestampMsg;                   // Timestamp of the message
    unsigned char signalLevel;                    // Signal Amplitude

    // DF 11
    int  ca;                    // Responder capabilities
    int  iid;

    // DF 17, DF 18
    int    metype;              // Extended squitter message type.
    int    mesub;               // Extended squitter message subtype.
    int    heading;             // Reported by aircraft, or computed from from EW and NS velocity
    int    raw_latitude;        // Non decoded latitude.
    int    raw_longitude;       // Non decoded longitude.
    double fLat;                // Coordinates obtained from CPR encoded data if/when decoded
    double fLon;                // Coordinates obtained from CPR encoded data if/when decoded
    char   flight[16];          // 8 chars flight number.
    int    ew_velocity;         // E/W velocity.
    int    ns_velocity;         // N/S velocity.
    int    vert_rate;           // Vertical rate.
    int    velocity;            // Reported by aircraft, or computed from from EW and NS velocity

    // DF4, DF5, DF20, DF21
    int  fs;                    // Flight status for DF4,5,20,21
    int  modeA;                 // 13 bits identity (Squawk).

    // Fields used by multiple message types.
    int  altitude;
    int  unit; 
    int  bFlags;                // Flags related to fields in this structure
};

struct {                           // Internal state
    int      quiet;
    // Networking
    uint32_t net_pp_ipaddr;              // IPv4 address of PP instance
    char     net_input_beast_ipaddr[32]; // IPv4 address or network name of server/RPi
}  ppup1090;

// COAA Initialisation structure
struct _coaa1090 {
    uint32_t ppIPAddr;
    double   fUserLat;
    double   fUserLon;
    char     strAuthCode[16];
    char     strRegNo[16];
    char     strVersion[16];
}  coaa1090;

// ======================== function declarations =========================

#ifdef __cplusplus
extern "C" {
#endif

//
// Functions exported from mode_ac.c
//
int  detectModeA       (uint16_t *m, struct modesMessage *mm);
void decodeModeAMessage(struct modesMessage *mm, int ModeA);
int  ModeAToModeC      (unsigned int ModeA);

//
// Functions exported from mode_s.c
//
void detectModeS        (uint16_t *m, uint32_t mlen);
void decodeModesMessage (struct modesMessage *mm, unsigned char *msg);
void useModesMessage    (struct modesMessage *mm);
int  decodeCPR          (struct aircraft *a, int fflag, int surface);
int  decodeCPRrelative  (struct aircraft *a, int fflag, int surface);
//
// Functions exported from interactive.c
//
struct aircraft* interactiveReceiveData(struct modesMessage *mm);
void  interactiveRemoveStaleAircrafts(void);
int   decodeBinMessage   (char *p);
struct aircraft *interactiveFindAircraft(uint32_t addr);
struct stDF     *interactiveFindDF      (uint32_t addr);

//
// Functions exported from coaa1090.c
//
int  openCOAA  (void);
int  closeCOAA (void);
int  initCOAA  (struct _coaa1090 coaa1090);
void postCOAA  (void);

#ifdef __cplusplus
}
#endif

#endif // __PPUP1090_H
