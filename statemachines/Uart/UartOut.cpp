//****************************************************************************
// Model: Uart.qm
// File:  Uart/UartOut.cpp
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
//${Uart::Uart::UartOut.cpp} .................................................
#include "UartOut.h"
#include "qpcpp.h"
#include "active_events.h"
#include "active_pipe.h"
#include "active_log.h"
#include "bsp.h"

Q_DEFINE_THIS_MODULE("Uart Out")

using namespace QP;
using namespace StdEvents;

namespace AOs {


#if ((QP_VERSION < 580) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8)))
#error qpcpp version 5.8.0 or higher required
#endif

//${Uart::UartOut} ...........................................................
//${Uart::UartOut::UartOut} ..................................................
UartOut::UartOut(
    QP::QActive * owner,
    Fifo * fifo,
    USART_HANDLE_TYPE_DEF * devUart)
  : ASM(UART_OUT_ACTIVE_TIMER_SIG, (QStateHandler)&UartOut::initial, owner, "UartOut")
    , m_fifo(fifo)
    , m_devUart(devUart)
{}

//${Uart::UartOut::DmaDoneCallback} ..........................................
void UartOut::DmaDoneCallback() {
    static uint16_t counter = 0;
    Evt *evt = new Evt(UART_OUT_DMA_DONE_SIG, counter++);
    QF::PUBLISH(evt, 0);
}
//${Uart::UartOut::SM} .......................................................
QP::QState UartOut::initial(UartOut * const me, QP::QEvt const * const e) {
    // ${Uart::UartOut::SM::initial}
    return Q_TRAN(&Root);
}
//${Uart::UartOut::SM::Root} .................................................
QP::QState UartOut::Root(UartOut * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${Uart::UartOut::SM::Root}
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root}
        case Q_EXIT_SIG: {
            LOG_EVENT(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::initial}
        case Q_INIT_SIG: {
            LOG_EVENT(e);
            status_ = Q_TRAN(&Stopped);
            break;
        }
        // ${Uart::UartOut::SM::Root::UART_OUT_START_REQ}
        case UART_OUT_START_REQ_SIG: {
            LOG_EVENT(e);
            me->PublishConfirmationWithInvalidState(
                EVT_CAST(*e),
                UART_OUT_START_CFM_SIG);
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
//${Uart::UartOut::SM::Root::Stopped} ........................................
QP::QState UartOut::Stopped(UartOut * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${Uart::UartOut::SM::Root::Stopped}
        case Q_ENTRY_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Stopped}
        case Q_EXIT_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Stopped::UART_OUT_START_REQ}
        case UART_OUT_START_REQ_SIG: {
            LOG_EVENT(e);
            me->m_fifo->Reset();
            me->PublishConfirmation(EVT_CAST(*e), UART_OUT_START_CFM_SIG);
            status_ = Q_TRAN(&Started);
            break;
        }
        // ${Uart::UartOut::SM::Root::Stopped::UART_OUT_STOP_REQ}
        case UART_OUT_STOP_REQ_SIG: {
            LOG_EVENT(e);
            me->PublishConfirmation(
                EVT_CAST(*e),
                UART_OUT_STOP_CFM_SIG);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Root);
            break;
        }
    }
    return status_;
}
//${Uart::UartOut::SM::Root::Started} ........................................
QP::QState UartOut::Started(UartOut * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${Uart::UartOut::SM::Root::Started}
        case Q_ENTRY_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started}
        case Q_EXIT_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::initial}
        case Q_INIT_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_TRAN(&Inactive);
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::UART_OUT_STOP_REQ}
        case UART_OUT_STOP_REQ_SIG: {
            LOG_EVENT_NOQP(e);
            me->PublishConfirmation(
                EVT_CAST(*e),
                UART_OUT_STOP_CFM_SIG);
            status_ = Q_TRAN(&Stopped);
            break;
        }
        default: {
            status_ = Q_SUPER(&Root);
            break;
        }
    }
    return status_;
}
//${Uart::UartOut::SM::Root::Started::Inactive} ..............................
QP::QState UartOut::Inactive(UartOut * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${Uart::UartOut::SM::Root::Started::Inactive}
        case Q_ENTRY_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Inactive}
        case Q_EXIT_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Inactive::UART_OUT_WRITE_REQ}
        case UART_OUT_WRITE_REQ_SIG: {
            LOG_EVENT_NOQP(e);
            me->PublishConfirmation(
                EVT_CAST(*e),
                UART_OUT_WRITE_CFM_SIG);

            if (me->m_fifo->GetUsedCount() == 0) {
                status_ = Q_HANDLED();
                break;
            } else {
                status_ = Q_TRAN(&UartOut::Active);
                break;
            }
            status_ = Q_TRAN(&Active);
            break;
        }
        default: {
            status_ = Q_SUPER(&Started);
            break;
        }
    }
    return status_;
}
//${Uart::UartOut::SM::Root::Started::Active} ................................
QP::QState UartOut::Active(UartOut * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${Uart::UartOut::SM::Root::Started::Active}
        case Q_ENTRY_SIG: {
            LOG_EVENT_NOQP(e);
            me->m_timer.armX(ACTIVE_TIMEOUT);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active}
        case Q_EXIT_SIG: {
            LOG_EVENT_NOQP(e);
            me->m_timer.disarm();
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::initial}
        case Q_INIT_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_TRAN(&Normal);
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::UART_OUT_CONTINUE}
        case UART_OUT_CONTINUE_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_TRAN(&Active);
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::UART_OUT_DONE}
        case UART_OUT_DONE_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_TRAN(&Inactive);
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::UART_OUT_ACTIVE_TIMER}
        case UART_OUT_ACTIVE_TIMER_SIG: {
            LOG_EVENT_NOQP(e);
            me->PublishTimeout(UART_OUT_FAIL_IND_SIG);
            status_ = Q_TRAN(&Failed);
            break;
        }
        default: {
            status_ = Q_SUPER(&Started);
            break;
        }
    }
    return status_;
}
//${Uart::UartOut::SM::Root::Started::Active::Normal} ........................
QP::QState UartOut::Normal(UartOut * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${Uart::UartOut::SM::Root::Started::Active::Normal}
        case Q_ENTRY_SIG: {
            LOG_EVENT_NOQP(e);
            Fifo &fifo = *(me->m_fifo);
            uint32_t addr = fifo.GetReadAddr();
            uint32_t len = fifo.GetUsedCount();
            if ((addr + len - 1) > fifo.GetMaxAddr()) {
                len = fifo.GetMaxAddr() - addr + 1;
            }
            Q_ASSERT((len > 0) && (len <= fifo.GetUsedCount()));
            // Only applicable to STM32F7.
            //SCB_CleanDCache_by_Addr((uint32_t *)(ROUND_DOWN_32(addr)), ROUND_UP_32(addr + len - ROUND_DOWN_32(addr)));
            BSP_UART_TRANSMIT_DMA(me->m_devUart, (uint8_t*)addr, len);
            me->m_writeCount = len;
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::Normal::UART_OUT_DMA_DONE}
        case UART_OUT_DMA_DONE_SIG: {
            LOG_EVENT_NOQP(e);
            me->m_fifo->IncReadIndex(me->m_writeCount);
            Evt *evt;
            if (me->m_fifo->GetUsedCount()) {
                me->PostToOwnerLifo(UART_OUT_CONTINUE_SIG);
            } else {
                me->Publish(UART_OUT_EMPTY_IND_SIG);
                me->PostToOwnerLifo(UART_OUT_DONE_SIG);
            }
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::Normal::UART_OUT_STOP_REQ}
        case UART_OUT_STOP_REQ_SIG: {
            LOG_EVENT_NOQP(e);
            me->Defer(e);
            status_ = Q_TRAN(&StopWait);
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::Normal::UART_OUT_WRITE_REQ}
        case UART_OUT_WRITE_REQ_SIG: {
            LOG_EVENT_NOQP(e);
            me->PublishConfirmation(
                EVT_CAST(*e),
                UART_OUT_WRITE_CFM_SIG);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Active);
            break;
        }
    }
    return status_;
}
//${Uart::UartOut::SM::Root::Started::Active::StopWait} ......................
QP::QState UartOut::StopWait(UartOut * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${Uart::UartOut::SM::Root::Started::Active::StopWait}
        case Q_ENTRY_SIG: {
            //LOG_EVENT(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::StopWait}
        case Q_EXIT_SIG: {
            //LOG_EVENT(e);
            me->RecallDeferred();
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::StopWait::UART_OUT_STOP_REQ}
        case UART_OUT_STOP_REQ_SIG: {
            LOG_EVENT_NOQP(e);
            me->Defer(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Active::StopWait::UART_OUT_DMA_DONE}
        case UART_OUT_DMA_DONE_SIG: {
            LOG_EVENT_NOQP(e);
            me->m_fifo->IncReadIndex(me->m_writeCount);
            me->PostToOwnerLifo(UART_OUT_DONE_SIG);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Active);
            break;
        }
    }
    return status_;
}
//${Uart::UartOut::SM::Root::Started::Failed} ................................
QP::QState UartOut::Failed(UartOut * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${Uart::UartOut::SM::Root::Started::Failed}
        case Q_ENTRY_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${Uart::UartOut::SM::Root::Started::Failed}
        case Q_EXIT_SIG: {
            LOG_EVENT_NOQP(e);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Started);
            break;
        }
    }
    return status_;
}

} // namespace AOs
