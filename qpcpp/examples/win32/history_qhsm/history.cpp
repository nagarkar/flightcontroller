//****************************************************************************
// Model: history.qm
// File:  ./history.cpp
//
// This code has been generated by QM tool (see state-machine.com/qm).
// DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//****************************************************************************
//${.::history.cpp} ..........................................................
#include "qep_port.h"
#include "qassert.h"
#include "history.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Q_DEFINE_THIS_FILE


#if ((QP_VERSION < 580) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8)))
#error qpcpp version 5.8.0 or higher required
#endif

//${SMs::ToastOven} ..........................................................
class ToastOven : public QP::QHsm {
public:
    ToastOven();

protected:
    static QP::QState initial(ToastOven * const me, QP::QEvt const * const e);
    static QP::QState doorClosed(ToastOven * const me, QP::QEvt const * const e);
    static QP::QState heating(ToastOven * const me, QP::QEvt const * const e);
    static QP::QState toasting(ToastOven * const me, QP::QEvt const * const e);
    static QP::QState baking(ToastOven * const me, QP::QEvt const * const e);
    static QP::QState off(ToastOven * const me, QP::QEvt const * const e);
    static QP::QState doorOpen(ToastOven * const me, QP::QEvt const * const e);
    static QP::QState final(ToastOven * const me, QP::QEvt const * const e);

protected:
    QP::QStateHandler his_doorClosed;
};


static ToastOven l_oven; // the only instance of the ToastOven class

// global-scope definitions ------------------------------------
QP::QHsm * const the_oven = &l_oven;       // the opaque pointer

//${SMs::ToastOven} ..........................................................
//${SMs::ToastOven::ToastOven} ...............................................
ToastOven::ToastOven()
 : QHsm(Q_STATE_CAST(&ToastOven::initial))
{}

//${SMs::ToastOven::SM} ......................................................
QP::QState ToastOven::initial(ToastOven * const me, QP::QEvt const * const e) {
    // ${SMs::ToastOven::SM::initial}
    (void)e; /* avoid compiler warning */
    // state history attributes
    me->his_doorClosed = Q_STATE_CAST(&off);
    return Q_TRAN(&doorClosed);
}
//${SMs::ToastOven::SM::doorClosed} ..........................................
QP::QState ToastOven::doorClosed(ToastOven * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${SMs::ToastOven::SM::doorClosed}
        case Q_ENTRY_SIG: {
            printf("door-Closed;");
            status_ = Q_HANDLED();
            break;
        }
        // ${SMs::ToastOven::SM::doorClosed}
        case Q_EXIT_SIG: {
            // save deep history
            me->his_doorClosed = me->state();
            status_ = Q_HANDLED();
            break;
        }
        // ${SMs::ToastOven::SM::doorClosed::initial}
        case Q_INIT_SIG: {
            status_ = Q_TRAN(&off);
            break;
        }
        // ${SMs::ToastOven::SM::doorClosed::TERMINATE}
        case TERMINATE_SIG: {
            status_ = Q_TRAN(&final);
            break;
        }
        // ${SMs::ToastOven::SM::doorClosed::OPEN}
        case OPEN_SIG: {
            status_ = Q_TRAN(&doorOpen);
            break;
        }
        // ${SMs::ToastOven::SM::doorClosed::TOAST}
        case TOAST_SIG: {
            status_ = Q_TRAN(&toasting);
            break;
        }
        // ${SMs::ToastOven::SM::doorClosed::BAKE}
        case BAKE_SIG: {
            status_ = Q_TRAN(&baking);
            break;
        }
        // ${SMs::ToastOven::SM::doorClosed::OFF}
        case OFF_SIG: {
            status_ = Q_TRAN(&off);
            break;
        }
        default: {
            status_ = Q_SUPER(&top);
            break;
        }
    }
    return status_;
}
//${SMs::ToastOven::SM::doorClosed::heating} .................................
QP::QState ToastOven::heating(ToastOven * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${SMs::ToastOven::SM::doorClosed::heating}
        case Q_ENTRY_SIG: {
            printf("heater-On;");
            status_ = Q_HANDLED();
            break;
        }
        // ${SMs::ToastOven::SM::doorClosed::heating}
        case Q_EXIT_SIG: {
            printf("heater-Off;");
            status_ = Q_HANDLED();
            break;
        }
        // ${SMs::ToastOven::SM::doorClosed::heating::initial}
        case Q_INIT_SIG: {
            status_ = Q_TRAN(&toasting);
            break;
        }
        default: {
            status_ = Q_SUPER(&doorClosed);
            break;
        }
    }
    return status_;
}
//${SMs::ToastOven::SM::doorClosed::heating::toasting} .......................
QP::QState ToastOven::toasting(ToastOven * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${SMs::ToastOven::SM::doorClosed::heating::toasting}
        case Q_ENTRY_SIG: {
            printf("toasting;");
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&heating);
            break;
        }
    }
    return status_;
}
//${SMs::ToastOven::SM::doorClosed::heating::baking} .........................
QP::QState ToastOven::baking(ToastOven * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${SMs::ToastOven::SM::doorClosed::heating::baking}
        case Q_ENTRY_SIG: {
            printf("baking;");
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&heating);
            break;
        }
    }
    return status_;
}
//${SMs::ToastOven::SM::doorClosed::off} .....................................
QP::QState ToastOven::off(ToastOven * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${SMs::ToastOven::SM::doorClosed::off}
        case Q_ENTRY_SIG: {
            printf("toaster-Off;");
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&doorClosed);
            break;
        }
    }
    return status_;
}
//${SMs::ToastOven::SM::doorOpen} ............................................
QP::QState ToastOven::doorOpen(ToastOven * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${SMs::ToastOven::SM::doorOpen}
        case Q_ENTRY_SIG: {
            printf("door-Open,lamp-On;");
            status_ = Q_HANDLED();
            break;
        }
        // ${SMs::ToastOven::SM::doorOpen}
        case Q_EXIT_SIG: {
            printf("lamp-Off;");
            status_ = Q_HANDLED();
            break;
        }
        // ${SMs::ToastOven::SM::doorOpen::CLOSE}
        case CLOSE_SIG: {
            status_ = Q_TRAN_HIST(me->his_doorClosed);
            break;
        }
        // ${SMs::ToastOven::SM::doorOpen::TERMINATE}
        case TERMINATE_SIG: {
            status_ = Q_TRAN(&final);
            break;
        }
        default: {
            status_ = Q_SUPER(&top);
            break;
        }
    }
    return status_;
}
//${SMs::ToastOven::SM::final} ...............................................
QP::QState ToastOven::final(ToastOven * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${SMs::ToastOven::SM::final}
        case Q_ENTRY_SIG: {
            printf("-> final\nBye!Bye!\n");
            _exit(0);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&top);
            break;
        }
    }
    return status_;
}

