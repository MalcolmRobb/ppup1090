// ppup1090, a Mode S PlanePlotter Uploader for dump1090 devices.
//
// Copyright (C) 2013-2021 by Malcolm Robb <Support@ATTAvionics.com>
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
#include "coaa.h"
#include "ppup1090.h"
//
// ============================= Utility functions ==========================
//
void sigintHandler(int dummy) {
    NOTUSED(dummy);
    signal(SIGINT, SIG_DFL);  // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
}
//
// =============================== Initialization ===========================
//
void ppup1090InitConfig(void) {

    int iErr;

    // Default everything to zero/NULL
    memset(&Modes,    0, sizeof(Modes));
    memset(&ppup1090, 0, sizeof(ppup1090));

    // Now initialise things that should not be 0/NULL to their defaults
    strcpy(ppup1090.net_input_beast_ipaddr,PPUP1090_NET_OUTPUT_IP_ADDRESS);
    Modes.net_input_beast_port    = MODES_NET_OUTPUT_BEAST_PORT;
    Modes.interactive_delete_ttl  = MODES_INTERACTIVE_DELETE_TTL;
    Modes.interactive_display_ttl = MODES_INTERACTIVE_DISPLAY_TTL;
    Modes.fUserLat                = MODES_USER_LATITUDE_DFLT;
    Modes.fUserLon                = MODES_USER_LONGITUDE_DFLT;

    // Default Mode A/C handling to on
    Modes.mode_ac                 = 1;

    if ((iErr = openCOAA()))
    {
        fprintf(stderr, "Error 0x%X initialising uploader\n", iErr);
        exit(1);
    }
}
//
//=========================================================================
//
void ppup1090Init(void) {

    int iErr;

    pthread_mutex_init(&Modes.pDF_mutex,NULL);
    pthread_mutex_init(&Modes.data_mutex,NULL);
    pthread_cond_init(&Modes.data_cond,NULL);

    // Allocate the various buffers used by Modes
    if ( NULL == (Modes.icao_cache = (uint32_t *) malloc(sizeof(uint32_t) * MODES_ICAO_CACHE_LEN * 2)))
    {
        fprintf(stderr, "Out of memory allocating data buffer.\n");
        exit(1);
    }

    // Clear the buffers that have just been allocated, just in-case
    memset(Modes.icao_cache, 0,   sizeof(uint32_t) * MODES_ICAO_CACHE_LEN * 2);

    // Validate the users Lat/Lon home location inputs
    if ( (Modes.fUserLat >   90.0)  // Latitude must be -90 to +90
      || (Modes.fUserLat <  -90.0)  // and 
      || (Modes.fUserLon >  360.0)  // Longitude must be -180 to +360
      || (Modes.fUserLon < -180.0) ) {
        Modes.fUserLat = Modes.fUserLon = 0.0;
    } else if (Modes.fUserLon > 180.0) { // If Longitude is +180 to +360, make it -180 to 0
        Modes.fUserLon -= 360.0;
    }
    // If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the 
    // Atlantic ocean off the west coast of Africa. This is unlikely to be correct. 
    // Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian 
    // is at 0.0 Lon,so we must check for either fLat or fLon being non zero not both. 
    // Testing the flag at runtime will be much quicker than ((fLon != 0.0) || (fLat != 0.0))
    Modes.bUserFlags &= ~MODES_USER_LATLON_VALID;
    if ((Modes.fUserLat != 0.0) || (Modes.fUserLon != 0.0)) {
        Modes.bUserFlags |= MODES_USER_LATLON_VALID;
    }

    // Setup the uploader - read the user paramaters from the coaa.h header file
    coaa1090.ppIPAddr = ppup1090.net_pp_ipaddr;
    coaa1090.fUserLat = MODES_USER_LATITUDE_DFLT;
    coaa1090.fUserLon = MODES_USER_LONGITUDE_DFLT;
    strcpy(coaa1090.strAuthCode,STR(USER_AUTHCODE));
    strcpy(coaa1090.strRegNo,   STR(USER_REGNO));
    strcpy(coaa1090.strVersion, MODES_PPUP1090_VERSION);

    if ((iErr = initCOAA (coaa1090)))
    {
        fprintf(stderr, "Error 0x%X initialising uploader\n", iErr);
        exit(1);
    }
}
//
//=========================================================================
//
void modesInitNet(void) {

#ifdef _WIN32
    if ( (!Modes.wsaData.wVersion) 
      && (!Modes.wsaData.wHighVersion) ) {
      // Try to start the windows socket support
      if (WSAStartup(MAKEWORD(2,1),&Modes.wsaData) != 0) 
        {
        fprintf(stderr, "WSAStartup returned Error\n");
        }
      }
#else
    signal(SIGPIPE, SIG_IGN);
#endif
}
//
//=========================================================================
//
// This function decodes a Beast binary format message
//
// The message is passed to the higher level layers, so it feeds
// the selected screen output, the network output and so forth.
//
// If the message looks invalid it is silently discarded.
//
// The function always returns 0 (success) to the caller as there is no
// case where we want broken messages here to close the client connection.
//
int decodeBinMessage(char *p) {
    int msgLen = 0;
    int  j;
    char ch;
    char * ptr;
    unsigned char msg[MODES_LONG_MSG_BYTES];
    struct modesMessage mm;
    memset(&mm, 0, sizeof(mm));

    ch = *p++; /// Get the message type
    if (0x1A == ch) {p++;} 

    if       ((ch == '1') && (Modes.mode_ac)) { // skip ModeA/C unless user enables --modes-ac
        msgLen = MODEAC_MSG_BYTES;
    } else if (ch == '2') {
        msgLen = MODES_SHORT_MSG_BYTES;
    } else if (ch == '3') {
        msgLen = MODES_LONG_MSG_BYTES;
    }

    if (msgLen) {
        // Mark messages received over the internet as remote so that we don't try to
        // pass them off as being received by this instance when forwarding them

        ptr = (char*) &mm.timestampMsg;
        for (j = 0; j < 6; j++) { // Grab the timestamp (big endian format)
            ptr[5-j] = ch = *p++; 
            if (0x1A == ch) {p++;}
        }

        mm.signalLevel = ch = *p++;  // Grab the signal level
        if (0x1A == ch) {p++;}

        for (j = 0; j < msgLen; j++) { // and the data
            msg[j] = ch = *p++;
            if (0x1A == ch) {p++;}
        }

        if (msgLen == MODEAC_MSG_BYTES) { // ModeA or ModeC
            decodeModeAMessage(&mm, ((msg[0] << 8) | msg[1]));
        } else {
            decodeModesMessage(&mm, msg);
        }

        useModesMessage(&mm);
    }
    return (0);
}
//
//=========================================================================
//
// This function polls the clients using read() in order to receive new
// messages from dump1090.
//
// Every full message received is decoded and passed to the higher layers
// calling the function's 'handler'.
//
// The handler returns 0 on success, or 1 to signal this function we should
// close the connection with the client in case of non-recoverable errors.
//
void modesReadFromClient(struct client *c) {
    int left;
    int nread;
    int fullmsg;
    int bContinue = 1;
    char *s, *e, *p;

    while(bContinue) {

        fullmsg = 0;
        left = MODES_CLIENT_BUF_SIZE - c->buflen;
        // If our buffer is full discard it, this is some badly formatted shit
        if (left <= 0) {
            c->buflen = 0;
            left = MODES_CLIENT_BUF_SIZE;
            // If there is garbage, read more to discard it ASAP
        }
#ifndef _WIN32
        nread = read(c->fd, c->buf+c->buflen, left);
#else
        nread = recv(c->fd, c->buf+c->buflen, left, 0);
        if (nread < 0) {errno = WSAGetLastError();}
#endif
        if (nread == 0) {
            close(c->fd); 
            c->fd = ANET_ERR;
			return;
		}

        // If we didn't get all the data we asked for, then return once we've processed what we did get.
        if (nread != left) {
            bContinue = 0;
        }
#ifndef _WIN32
        if ( (nread < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK)) { // Error, or end of file
#else
        if ( (nread < 0)                      && (errno != EWOULDBLOCK)) { // Error, or end of file
#endif
            close(c->fd); 
            c->fd = ANET_ERR;
            return;
        }
        if (nread <= 0) {
            return;
        }
        c->buflen += nread;

        // Always null-term so we are free to use strstr() (it won't affect binary case)
        c->buf[c->buflen] = '\0';

        e = s = c->buf;                                // Start with the start of buffer, first message

        // This is the Beast Binary scanning case.
        // If there is a complete message still in the buffer, there must be the separator 'sep'
        // in the buffer, note that we full-scan the buffer at every read for simplicity.

        left = c->buflen;                                  // Length of valid search for memchr()
        while (left && ((s = memchr(e, (char) 0x1a, left)) != NULL)) { // The first byte of buffer 'should' be 0x1a
            s++;                                           // skip the 0x1a
            if        (*s == '1') {
                e = s + MODEAC_MSG_BYTES      + 8;         // point past remainder of message
            } else if (*s == '2') {
                e = s + MODES_SHORT_MSG_BYTES + 8;
            } else if (*s == '3') {
                e = s + MODES_LONG_MSG_BYTES  + 8;
            } else {
                e = s;                                     // Not a valid beast message, skip
                left = &(c->buf[c->buflen]) - e;
                continue;
            }
            // we need to be careful of double escape characters in the message body
            for (p = s; p < e; p++) {
                if (0x1A == *p) {
                    p++; e++;
                    if (e > &(c->buf[c->buflen])) {
                        break;
                    }
                }
            }
            left = &(c->buf[c->buflen]) - e;
            if (left < 0) {                                // Incomplete message in buffer
                e = s - 1;                                 // point back at last found 0x1a.
                break;
            }
            // Have a 0x1a followed by 1, 2 or 3 - pass message less 0x1a to handler.
            if (decodeBinMessage(s)) {
                close(c->fd); 
                c->fd = ANET_ERR;
                return;
            }
            fullmsg = 1;
        }
        s = e;     // For the buffer remainder below

        if (fullmsg) {                             // We processed something - so
            c->buflen = &(c->buf[c->buflen]) - s;  //     Update the unprocessed buffer length
            memmove(c->buf, s, c->buflen);         //     Move what's remaining to the start of the buffer
        } else {                                   // If no message was decoded process the next client
            break;
        }
    }
}
//
//=========================================================================
//
// Set up data connection
//
int setupConnection(void) {
    int fd;

    // Try to connect to the selected ip address and port. We only support *ONE* input connection which we initiate.here.
    fd = anetTcpConnect(Modes.aneterr, ppup1090.net_input_beast_ipaddr, Modes.net_input_beast_port);
    if (fd != ANET_ERR) {
        if (Modes.mode_ac) {
            send(fd, "\0321J", 3, 0);
        } else {
            send(fd, "\0321j", 3, 0);
        }
		//anetNonBlock(Modes.aneterr, fd);
    }
    return (fd);
}
//
// ================================ Main ====================================
//
void showHelp(void) {
    printf(
"-----------------------------------------------------------------------------\n"
"|    ppup1090 RPi Uploader for COAA Planeplotter         Ver : "MODES_PPUP1090_VERSION " |\n"
"-----------------------------------------------------------------------------\n"
  "--modeac                 Enable decoding of SSR Modes 3/A & 3/C\n"
  "--nomodeac               Disable decoding of SSR Modes 3/A & 3/C\n"
  "--net-bo-ipaddr <IPv4>   TCP Beast output listen IPv4 (default: 127.0.0.1)\n"
  "--net-bo-port <port>     TCP Beast output listen port (default: 30005)\n"
  "--net-pp-ipaddr <IPv4>   Plane Plotter LAN IPv4 Address (default: 0.0.0.0)\n"
  "--quiet                  Disable output to stdout. Use for daemon applications\n"
  "--help                   Show this help\n"
    );
}

#ifdef _WIN32
void showCopyright(void) {
    uint64_t llTime = time(NULL) + 1;

    printf(
"-----------------------------------------------------------------------------\n"
"|    ppup1090 RPi Uploader for COAA Planeplotter         Ver : "MODES_PPUP1090_VERSION " |\n"
"-----------------------------------------------------------------------------\n"
"\n"
" Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>\n"
" Copyright (C) 2014 by Malcolm Robb <support@attavionics.com>\n"
" Copyright (C) 2021 by Malcolm Robb <support@attavionics.com>\n"
"\n"
" All rights reserved.\n"
"\n"
" THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n"
" ""AS IS"" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n"
" LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR\n"
" A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT\n"
" HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,\n"
" SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT\n"
" LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,\n"
" DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY\n"
" THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT\n"
" (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE\n"
" OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n"
"\n"
" For further details refer to <https://github.com/MalcolmRobb/dump1090>\n" 
"\n"
    );

  // delay for 1 second to give the user a chance to read the copyright
  while (llTime >= time(NULL)) {}
}
#endif
//
//=========================================================================
//
int main(int argc, char **argv) {
    int j;
    struct client *c;

    // Set sane defaults

    ppup1090InitConfig();
    signal(SIGINT, sigintHandler); // Define Ctrl/C handler (exit program)

    // Parse the command line options
    for (j = 1; j < argc; j++) {
        int more = ((j + 1) < argc); // There are more arguments

        if        (!strcmp(argv[j],"--modeac")) {
            Modes.mode_ac = 1;
        } else if (!strcmp(argv[j],"--nomodeac")) {
            Modes.mode_ac = 0;
        } else if (!strcmp(argv[j],"--net-bo-port") && more) {
            Modes.net_input_beast_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bo-ipaddr") && more) {
            strcpy(ppup1090.net_input_beast_ipaddr, argv[++j]);
        } else if (!strcmp(argv[j],"--net-pp-ipaddr") && more) {
            inet_aton(argv[++j], (void *)&ppup1090.net_pp_ipaddr);
        } else if (!strcmp(argv[j],"--quiet")) {
            ppup1090.quiet = 1;
        } else if (!strcmp(argv[j],"--help")) {
            showHelp();
            exit(0);
        } else {
            fprintf(stderr, "Unknown or not enough arguments for option '%s'.\n\n", argv[j]);
            showHelp();
            exit(1);
        }
    }

#ifdef _WIN32
    // Try to comply with the Copyright license conditions for binary distribution
    if (!ppup1090.quiet) {showCopyright();}
#endif

    // Initialization
    ppup1090Init();

    c = (struct client *) malloc(sizeof(*c));
    c->buflen = 0;
    c->fd     = setupConnection();

    // Keep going till the user does something that stops us
    while (!Modes.exit) {
        interactiveRemoveStaleAircrafts();
        postCOAA ();

        if (c->fd == ANET_ERR) {
            // If the connection to dump1090 has failed, wait 1 second before trying to re-connect 
			usleep(1000000);

            // Try to connect to the selected ip address and port. We only support *ONE* input connection 
            // which we try to initiate here.
            c->fd     = setupConnection();
            c->buflen = 0;

       } else {
            // If the connecton to dupp1090 is up and running, try to read some data.
            modesReadFromClient(c);
       }
    }

    // The user has stopped us, so close any socket we opened
    if (c->fd != ANET_ERR)
      {close(c->fd);}
    free(c);

    closeCOAA ();
#ifndef _WIN32
    pthread_exit(0);
#else
    return (0);
#endif
}
//
//=========================================================================
//
