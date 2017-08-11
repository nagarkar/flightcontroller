/// @file
/// @brief QS/C++ QUTEST port for Win32
/// @ingroup qf
/// @cond
///***************************************************************************
/// Last updated for version 5.9.0
/// Last updated on  2017-05-17
///
///                    Q u a n t u m     L e a P s
///                    ---------------------------
///                    innovating embedded systems
///
/// Copyright (C) 2005-2017 Quantum Leaps, LLC. All rights reserved.
///
/// This program is open source software: you can redistribute it and/or
/// modify it under the terms of the GNU General Public License as published
/// by the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// Alternatively, this program may be distributed and modified under the
/// terms of Quantum Leaps commercial licenses, which expressly supersede
/// the GNU General Public License and are specifically designed for
/// licensees interested in retaining the proprietary status of their code.
///
/// This program is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with this program. If not, see <http://www.gnu.org/licenses/>.
///
/// Contact information:
/// https://state-machine.com
/// mailto:info@state-machine.com
///***************************************************************************
/// @endcond
///

#ifndef Q_SPY
    #error "Q_SPY must be defined for QTEST application"
#endif // Q_SPY

#define QP_IMPL       // this is QP implementation
#include "qf_port.h"  // QF port
#include "qassert.h"  // QP embedded systems-friendly assertions
#include "qs_port.h"  // include QS port

#include <stdio.h>    // for printf() and _snprintf_s()
#include <stdlib.h>
#include <time.h>
#include <conio.h>
#include <winsock2.h> // for Windows network facilities

#define QS_TX_SIZE    (4*1024)
#define QS_RX_SIZE    1024
#define QS_IMEOUT_MS  100

namespace QP {

Q_DEFINE_THIS_MODULE("qutest_port")

// local variables ...........................................................
static SOCKET l_sock = INVALID_SOCKET;

//............................................................................
bool QS::onStartup(void const *arg) {
    static uint8_t qsBuf[QS_TX_SIZE];   // buffer for QS-TX channel
    static uint8_t qsRxBuf[QS_RX_SIZE]; // buffer for QS-RX channel
    static WSADATA wsaData;
    char hostName[64];
    char const *src;
    char *dst;

    USHORT port_local  = 51234; // default local port
    USHORT port_remote = 6601;  // default QSPY server port
    BOOL   sockopt_bool;
    ULONG  ioctl_opt = 1;
    struct sockaddr_in sa_local;
    struct sockaddr_in sa_remote;
    struct hostent *host;

    initBuf(qsBuf, sizeof(qsBuf));
    rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    // initialize Windows sockets
    if (WSAStartup(MAKEWORD(2,0), &wsaData) == SOCKET_ERROR) {
        printf("<TARGET> Windows Sockets cannot be initialized\n");
        goto error;
    }

    src = (arg != (void const *)0)
          ? (char const *)arg
          : "localhost";
    dst = hostName;
    while ((*src != '\0')
           && (*src != ':')
           && (dst < &hostName[sizeof(hostName)]))
    {
        *dst++ = *src++;
    }
    *dst = '\0';
    if (*src == ':') {
        port_remote = (USHORT)strtoul(src + 1, NULL, 10);
    }

    l_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // TCP socket
    if (l_sock == INVALID_SOCKET){
        printf("<TARGET> Socket cannot be created Err=0x%08X\n",
               WSAGetLastError());
        goto error;
    }

    // configure the socket
    sockopt_bool = TRUE;
    setsockopt(l_sock, SOL_SOCKET, SO_REUSEADDR,
               (const char *)&sockopt_bool, sizeof(sockopt_bool));

    sockopt_bool = TRUE;
    setsockopt(l_sock, SOL_SOCKET, SO_DONTLINGER,
               (const char *)&sockopt_bool, sizeof(sockopt_bool));

    // disable the Nagle algorithm (good for small messages)
    //sockopt_bool = TRUE;
    //setsockopt(l_sock, IPPROTO_TCP, TCP_NODELAY,
    //           (const char *)&sockopt_bool, sizeof(sockopt_bool));

    // local address:port
    memset(&sa_local, 0, sizeof(sa_local));
    sa_local.sin_family = AF_INET;
    sa_local.sin_port = htons(port_local);
    host = gethostbyname(""); // local host
    sa_local.sin_addr.s_addr = inet_addr(
        inet_ntoa(*(struct in_addr *)*host->h_addr_list));
    //if (bind(l_sock, &sa_local, sizeof(sa_local)) == -1) {
    //    printf("<TARGET> Cannot bind to the local port Err=0x%08X\n",
    //           WSAGetLastError());
    //    // no error
    //}

    // remote hostName:port (QSPY server socket)
    host = gethostbyname(hostName);
    if (host == NULL) {
        printf("<TARGET> QSpy host name %s cannot be resolved Err=0x%08X\n",
               hostName, WSAGetLastError());
        goto error;
    }

    memset(&sa_remote, 0, sizeof(sa_remote));
    sa_remote.sin_family = AF_INET;
    memcpy(&sa_remote.sin_addr, host->h_addr, host->h_length);
    sa_remote.sin_port = htons(port_remote);

    // try to connect to the QSPY server
    if (connect(l_sock, (struct sockaddr *)&sa_remote, sizeof(sa_remote))
        == SOCKET_ERROR)
    {
        printf("<TARGET> Cannot connect to the QSPY server Err=0x%08X\n",
               WSAGetLastError());
        QS_EXIT();
        goto error;
    }

    // Set the socket to non-blocking mode.
    if (ioctlsocket(l_sock, FIONBIO, &ioctl_opt) == SOCKET_ERROR) {
        printf("<TARGET> Socket configuration failed Err=0x%08X\n",
               WSAGetLastError());
        QS_EXIT();
        goto error;
    }

    onFlush();

    //printf("<TARGET> Connected to QSPY via TCP/IP\n");
    return true;  // success

error:
    return false; // failure
}
//............................................................................
void QS::onCleanup(void) {
    if (l_sock != INVALID_SOCKET) {
        closesocket(l_sock);
        l_sock = INVALID_SOCKET;
    }
    WSACleanup();
    //printf("<TARGET> Disconnected from QSPY via TCP/IP\n");
}
//............................................................................
void QS::onReset(void) {
    onCleanup();
    exit(0);
}

//............................................................................
void QS::onTestLoop() {
    fd_set readSet;
    FD_ZERO(&readSet);

    rxPriv_.inTestLoop = true;
    while (rxPriv_.inTestLoop) {
        static struct timeval const timeout = {
            (long)0, (long)(QS_IMEOUT_MS * 1000)
        };
        int status;
        uint16_t nBytes;
        uint8_t const *block;
        int ch = 0;

        FD_SET(l_sock, &readSet);

        // selective, timed blocking on the TCP/IP socket...
        status = select(0, &readSet, (fd_set *)0, (fd_set *)0, &timeout);
        if (status == SOCKET_ERROR) {
            printf("<TARGET> ERROR socket select Err=%d", WSAGetLastError());
            onCleanup();
            exit(-2);
        }
        else if (FD_ISSET(l_sock, &readSet)) {
            uint8_t buf[QS_RX_SIZE];
            int status = recv(l_sock, (char *)buf, (int)sizeof(buf), 0);
            while (status > 0) { // any data received?
                uint8_t *pb;
                int i = (int)rxGetNfree();
                if (i > status) {
                    i = status;
                }
                status -= i;
                // reorder the received bytes into QS-RX buffer
                for (pb = &buf[0]; i > 0; --i, ++pb) {
                    rxPut(*pb);
                }
                rxParse(); // parse all n-bytes of data
            }
        }

        nBytes = QS_TX_SIZE;
        //QF_CRIT_ENTRY(dummy);
        block = getBlock(&nBytes);
        //QF_CRIT_EXIT(dummy);

        if (block != (uint8_t *)0) {
            send(l_sock, (char const *)block, nBytes, 0);
        }
        while (_kbhit()) { // any key pressed?
            ch = _getch();
        }
        switch (ch) {
            case 'x':      // 'x' pressed?
            case 'X':      // 'X' pressed?
            case '\033': { // ESC pressed?
                onCleanup();
                exit(0);
                break;
            }
        }
    }
    // set inTestLoop to true in case calls to QS_onTestLoop() nest,
    // which can happen through the calls to QS_TEST_PAUSE().
    rxPriv_.inTestLoop = true;
}
//............................................................................
void QS::onFlush(void) {
    if (l_sock != INVALID_SOCKET) {  // socket initialized?
        uint16_t nBytes = QS_TX_SIZE;
        uint8_t const *data;
        while ((data = getBlock(&nBytes)) != (uint8_t *)0) {
            send(l_sock, (char const *)data, nBytes, 0);
            nBytes = QS_TX_SIZE;
        }
    }
}

} // namespace QP
