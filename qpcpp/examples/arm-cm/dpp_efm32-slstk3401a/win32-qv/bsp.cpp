//****************************************************************************
// Product: DPP example, Win32-GUI
// Last updated for version 5.9.5
// Last updated on  2017-07-20
//
//                    Q u a n t u m     L e a P s
//                    ---------------------------
//                    innovating embedded systems
//
// Copyright (C) Quantum Leaps, LLC. All rights reserved.
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Alternatively, this program may be distributed and modified under the
// terms of Quantum Leaps commercial licenses, which expressly supersede
// the GNU General Public License and are specifically designed for
// licensees interested in retaining the proprietary status of their code.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.
//
// Contact information:
// https://state-machine.com
// mailto:info@state-machine.com
//****************************************************************************
#include "qpcpp.h"
#include "dpp.h"
#include "bsp.h"

#include "qwin_gui.h"  // QWIN GUI
#include "resource.h"  // GUI resource IDs generated by the resource editior

#include <stdio.h>     // for snprintf()
#include <stdlib.h>

#ifdef Q_SPY
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>  // Win32 API for multithreading
    #include <winsock2.h> // for Windows network facilities
#endif

Q_DEFINE_THIS_FILE

//****************************************************************************
// thread function for running the application main()
static DWORD WINAPI appThread(LPVOID par) {
    (void)par;         // unused parameter
    return main_gui(); // run the QF application
}

//****************************************************************************
namespace DPP {

// local variables -----------------------------------------------------------
static HINSTANCE l_hInst;   // this application instance
static HWND      l_hWnd;    // main window handle
static LPSTR     l_cmdLine; // the command line string

static SegmentDisplay   l_philos;   // SegmentDisplay to show Philo status
static OwnerDrawnButton l_pauseBtn; // owner-drawn button

static unsigned  l_rnd; // random seed

#ifdef Q_SPY
    enum {
        PHILO_STAT = QP::QS_USER,
        COMMAND_STAT
    };
    static SOCKET l_sock = INVALID_SOCKET;
    static uint8_t const l_clock_tick = 0U;
#endif

// Local functions -----------------------------------------------------------
static LRESULT CALLBACK WndProc(HWND hWnd, UINT iMsg,
                                WPARAM wParam, LPARAM lParam);

//............................................................................
extern "C" int WINAPI WinMain(HINSTANCE hInst, HINSTANCE /*hPrevInst*/,
                              LPSTR cmdLine, int iCmdShow)
{
    l_hInst   = hInst;   // save the application instance
    l_cmdLine = cmdLine; // save the command line string

    //AllocConsole();

    // create the main custom dialog window
    HWND hWnd = CreateCustDialog(hInst, IDD_APPLICATION, NULL,
                                 &WndProc, "QP_APP");
    ShowWindow(hWnd, iCmdShow); // show the main window

    // enter the message loop...
    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    //FreeConsole();
    BSP::terminate(0);

    return msg.wParam;
}
//............................................................................
static LRESULT CALLBACK WndProc(HWND hWnd, UINT iMsg,
                                WPARAM wParam, LPARAM lParam)
{
    switch (iMsg) {

        // Perform initialization upon cration of the main dialog window
        // NOTE: Any child-windows are NOT created yet at this time, so
        // the GetDlgItem() function can't be used (it will return NULL).
        //
        case WM_CREATE: {
            l_hWnd = hWnd; // save the window handle

            // initialize the owner-drawn buttons...
            // NOTE: must be done *before* the first drawing of the buttons,
            // so WM_INITDIALOG is too late.
            //
            OwnerDrawnButton_init(&l_pauseBtn, IDC_PAUSE,
                LoadBitmap(l_hInst, MAKEINTRESOURCE(IDB_BTN_UP)),
                LoadBitmap(l_hInst, MAKEINTRESOURCE(IDB_BTN_DWN)),
                LoadCursor(NULL, IDC_HAND));
            return 0;
        }

        // Perform initialization after all child windows have been created
        case WM_INITDIALOG: {

            SegmentDisplay_init(&l_philos,
                     N_PHILO,          /* N_PHILO "segments" for the Philos */
                     3U);         /* 3 bitmaps (for thinking/hungry/eating) */
            SegmentDisplay_initSegment(&l_philos, 0U, IDC_PHILO_0);
            SegmentDisplay_initSegment(&l_philos, 1U, IDC_PHILO_1);
            SegmentDisplay_initSegment(&l_philos, 2U, IDC_PHILO_2);
            SegmentDisplay_initSegment(&l_philos, 3U, IDC_PHILO_3);
            SegmentDisplay_initSegment(&l_philos, 4U, IDC_PHILO_4);
            SegmentDisplay_initBitmap(&l_philos,
                 0U, LoadBitmap(l_hInst, MAKEINTRESOURCE(IDB_THINKING)));
            SegmentDisplay_initBitmap(&l_philos,
                 1U, LoadBitmap(l_hInst, MAKEINTRESOURCE(IDB_HUNGRY)));
            SegmentDisplay_initBitmap(&l_philos,
                 2U, LoadBitmap(l_hInst, MAKEINTRESOURCE(IDB_EATING)));


            // --> QP: spawn the application thread to run main()
            Q_ALLEGE(CreateThread(NULL, 0, &appThread, NULL, 0, NULL)
                     != (HANDLE)0);
            return 0;
        }

        case WM_DESTROY: {
            PostQuitMessage(0);
            return 0;
        }

        // commands from regular buttons and menus...
        case WM_COMMAND: {
            SetFocus(hWnd);
            switch (wParam) {
                case IDOK:
                case IDCANCEL: {
                    PostQuitMessage(0);
                    break;
                }
            }
            return 0;
        }

        // owner-drawn buttons...
        case WM_DRAWITEM: {
            static QP::QEvt const pe = QEVT_INITIALIZER(PAUSE_SIG);
            LPDRAWITEMSTRUCT pdis = (LPDRAWITEMSTRUCT)lParam;
            switch (pdis->CtlID) {
                case IDC_PAUSE: {  // PAUSE owner-drawn button
                    switch (OwnerDrawnButton_draw(&l_pauseBtn,pdis)) {
                        case BTN_DEPRESSED: {
                            AO_Table->POST(&pe, static_cast<void *>(0));
                            break;
                        }
                        case BTN_RELEASED: {
                            static QP::QEvt const se =
                                QEVT_INITIALIZER(SERVE_SIG);
                            AO_Table->POST(&se, static_cast<void *>(0));
                            break;
                        }
                        default: {
                            break;
                        }
                    }
                    break;
                }
            }
            return 0;
        }

        // mouse input...
        case WM_MOUSEWHEEL: {
            return 0;
        }

        // keyboard input...
        case WM_KEYDOWN: {
            return 0;
        }
    }
    return DefWindowProc(hWnd, iMsg, wParam, lParam) ;
}
//............................................................................
void BSP::init(void) {
    if (!QS_INIT(l_cmdLine)) { // QS initialization failed?
        MessageBox(l_hWnd,
                   "Cannot connect to QSPY via TCP/IP\n"
                   "Please make sure that 'qspy -t' is running",
                   "QS_INIT() Error",
                   MB_OK | MB_ICONEXCLAMATION | MB_APPLMODAL);
    }
    QS_OBJ_DICTIONARY(&l_clock_tick); // must be called *after* QF::init()
    QS_USR_DICTIONARY(PHILO_STAT);
    QS_USR_DICTIONARY(COMMAND_STAT);
}
//............................................................................
void BSP::terminate(int16_t result) {
#ifdef Q_SPY
    if (l_sock != INVALID_SOCKET) {
        closesocket(l_sock);
        l_sock = INVALID_SOCKET;
    }
#endif
    QP::QF::stop();

    // cleanup all QWIN resources...
    OwnerDrawnButton_xtor(&l_pauseBtn); // cleanup the l_pauseBtn resources
    SegmentDisplay_xtor(&l_philos);     // cleanup the l_philos resources

    // end the main dialog
    EndDialog(l_hWnd, result);
}
//............................................................................
void BSP::displayPhilStat(uint8_t n, char const *stat) {
    UINT bitmapNum = 0;

    Q_REQUIRE(n < N_PHILO);

    switch (stat[0]) {
        case 't': bitmapNum = 0U; break;
        case 'h': bitmapNum = 1U; break;
        case 'e': bitmapNum = 2U; break;
        default: Q_ERROR();  break;
    }
    // set the "segment" # n to the bitmap # 'bitmapNum'
    SegmentDisplay_setSegment(&l_philos, (UINT)n, bitmapNum);

    QS_BEGIN(PHILO_STAT, AO_Philo[n]) // application-specific record begin
        QS_U8(1, n);   // Philosopher number
        QS_STR(stat);  // Philosopher status
    QS_END()
}
//............................................................................
void BSP::displayPaused(uint8_t paused) {
    char buf[16];
    LoadString(l_hInst,
        (paused != 0U) ? IDS_PAUSED : IDS_RUNNING, buf, Q_DIM(buf));
    SetDlgItemText(l_hWnd, IDC_PAUSED, buf);
}
//............................................................................
uint32_t BSP::random(void) {  // a very cheap pseudo-random-number generator
    // "Super-Duper" Linear Congruential Generator (LCG)
    // LCG(2^32, 3*7*11*13*23, 0, seed)
    //
    l_rnd = l_rnd * (3U*7U*11U*13U*23U);
    return l_rnd >> 8;
}
//............................................................................
void BSP::randomSeed(uint32_t seed) {
    l_rnd = seed;
}

} // namespace DPP

//****************************************************************************

namespace QP {

//............................................................................
void QF::onStartup(void) {
    QF_setTickRate(DPP::BSP::TICKS_PER_SEC); // set the desired tick rate
}
//............................................................................
void QF::onCleanup(void) {
}
//............................................................................
void QF_onClockTick(void) {
    QF::TICK(&DPP::l_clock_tick); // perform the QF clock tick processing
}
//............................................................................
extern "C" void Q_onAssert(char const * const module, int loc) {
    QF::stop();  // stop ticking
    QS_ASSERTION(module, loc, 10000U); // report assertion to QS

    char message[80];
    SNPRINTF_S(message, Q_DIM(message) - 1,
               "Assertion failed in module %s location %d", module, loc);
    MessageBox(DPP::l_hWnd, message, "!!! ASSERTION !!!",
               MB_OK | MB_ICONEXCLAMATION | MB_APPLMODAL);
    PostQuitMessage(-1);
}

//----------------------------------------------------------------------------
#ifdef Q_SPY // define QS callbacks

#include <time.h>

// In this demo, the QS software tracing output is sent out of the application
// through a TCP/IP socket. This requires the QSPY host application to
// be started first to open a server socket (qspy -t ...) to wait for the
// incoming TCP/IP connection from the DPP demo.
//
// In an embedded target, the QS software tracing output can be sent out
// using any method available, such as a UART. This would require changing
// the implementation of the functions in this section, but the rest of the
// application code does not "know" (and should not care) how the QS ouptut
// is actually performed. In other words, the rest of the application does NOT
// need to change in any way to produce QS output.

//............................................................................
extern "C" DWORD WINAPI idleThread(LPVOID par) { // signature for CreateThread()
    (void)par;
    while (DPP::l_sock != INVALID_SOCKET) {
        uint8_t const *block;

        // try to receive bytes from the QS socket...
        uint16_t nBytes = QS::rxGetNfree();
        if (nBytes > 0U) {
            uint8_t buf[64];
            int status;

            if (nBytes > sizeof(buf)) {
                nBytes = sizeof(buf);
            }
            status = recv(DPP::l_sock, reinterpret_cast<char *>(&buf[0]),
                          static_cast<int>(nBytes), 0);
            if (status != SOCKET_ERROR) {
                uint16_t i;
                nBytes = static_cast<uint16_t>(status);
                for (i = 0U; i < nBytes; ++i) {
                    QS::rxPut(buf[i]);
                }
            }
        }
        QS::rxParse();  // parse all the received bytes

        nBytes = 1024U;
        QF_CRIT_ENTRY(dummy);
        block = QS::getBlock(&nBytes);
        QF_CRIT_EXIT(dummy);

        if (block != static_cast<uint8_t *>(0)) {
            send(DPP::l_sock, reinterpret_cast<char const *>(block),
                 static_cast<int_t>(nBytes), 0);
        }
        Sleep(20); // sleep for xx milliseconds
    }
    return (DWORD)0; // return success
}
//............................................................................
bool QS::onStartup(void const *arg) {
    static uint8_t qsBuf[1024];  // buffer for QS output
    static uint8_t qsRxBuf[100]; // buffer for QS receive channel
    static WSADATA wsaData;
    char hostName[64];
    char const *src;
    char *dst;
    USHORT port = 6601; // default QSPY server port
    ULONG ioctl_opt = 1;
    struct sockaddr_in sockAddr;
    struct hostent *server;

    initBuf(qsBuf, sizeof(qsBuf));
    rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    // initialize Windows sockets
    if (WSAStartup(MAKEWORD(2,0), &wsaData) == SOCKET_ERROR) {
        printf("Windows Sockets cannot be initialized.");
        return (uint8_t)0;
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
        port = (USHORT)strtoul(src + 1, NULL, 10);
    }

    DPP::l_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // TCP socket
    if (DPP::l_sock == INVALID_SOCKET){
        printf("Socket cannot be created; error 0x%08X\n",
               WSAGetLastError());
        return false; // failure
    }

    server = gethostbyname(hostName);
    if (server == NULL) {
        printf("QSpy host name %s cannot be resolved; error 0x%08X\n",
               hostName, WSAGetLastError());
        return false;
    }

    memset(&sockAddr, 0, sizeof(sockAddr));
    sockAddr.sin_family = AF_INET;
    memcpy(&sockAddr.sin_addr, server->h_addr, server->h_length);
    sockAddr.sin_port = htons(port);
    if (connect(DPP::l_sock, reinterpret_cast<struct sockaddr *>(&sockAddr),
                sizeof(sockAddr)) == SOCKET_ERROR)
    {
        printf("Cannot connect to the QSPY server; error 0x%08X\n",
               WSAGetLastError());
        QS_EXIT();
        return false; // failure
    }

    // Set the socket to non-blocking mode.
    if (ioctlsocket(DPP::l_sock, FIONBIO, &ioctl_opt) == SOCKET_ERROR) {
        printf("Socket configuration failed.\n"
               "Windows socket error 0x%08X.",
               WSAGetLastError());
        QS_EXIT();
        return false; // failure
    }

    // set up the QS filters...
    QS_FILTER_ON(QS_QEP_STATE_ENTRY);
    QS_FILTER_ON(QS_QEP_STATE_EXIT);
    QS_FILTER_ON(QS_QEP_STATE_INIT);
    QS_FILTER_ON(QS_QEP_INIT_TRAN);
    QS_FILTER_ON(QS_QEP_INTERN_TRAN);
    QS_FILTER_ON(QS_QEP_TRAN);
    QS_FILTER_ON(QS_QEP_IGNORED);
    QS_FILTER_ON(QS_QEP_DISPATCH);
    QS_FILTER_ON(QS_QEP_UNHANDLED);

    QS_FILTER_ON(QS_QF_ACTIVE_POST_FIFO);
    QS_FILTER_ON(QS_QF_ACTIVE_POST_LIFO);
    QS_FILTER_ON(QS_QF_PUBLISH);

    QS_FILTER_ON(DPP::PHILO_STAT);
    QS_FILTER_ON(DPP::COMMAND_STAT);

    // return the status of creating the idle thread
    return (CreateThread(NULL, 1024, &idleThread, NULL, 0, NULL) != NULL)
           ? true : false;
}
//............................................................................
void QS::onCleanup(void) {
    if (DPP::l_sock != INVALID_SOCKET) {
        closesocket(DPP::l_sock);
        DPP::l_sock = INVALID_SOCKET;
    }
    WSACleanup();
}
//............................................................................
void QS::onFlush(void) {
    uint16_t nBytes = 1000U;
    uint8_t const *block;
    while ((block = getBlock(&nBytes)) != static_cast<uint8_t *>(0)) {
        send(DPP::l_sock, reinterpret_cast<char const *>(block), nBytes, 0);
        nBytes = 1000U;
    }
}
//............................................................................
QSTimeCtr QS::onGetTime(void) {
    return static_cast<QSTimeCtr>(clock());
}
//............................................................................
//! callback function to reset the target (to be implemented in the BSP)
void QS::onReset(void) {
    //TBD
}
//............................................................................
//! callback function to execute a uesr command (to be implemented in BSP)
void QS::onCommand(uint8_t cmdId, uint32_t param1,
                   uint32_t param2, uint32_t param3)
{
    (void)cmdId;
    (void)param1;
    (void)param2;
    (void)param3;

    // application-specific record
    QS_BEGIN(DPP::COMMAND_STAT, static_cast<void *>(0))
        QS_U8(2, cmdId);
        QS_U32(8, param1);
    QS_END()

    if (cmdId == 10U) {
        Q_onAssert("command", 10);
    }
}

#endif // Q_SPY
//----------------------------------------------------------------------------

} // namespace QP
