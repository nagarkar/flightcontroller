//****************************************************************************
// Model: StdEvents.qm
// File:  stdevents/active_config.cpp
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
//${stdevents::active_config.cpp} ............................................
#include "active_config.h"
#include "qpcpp.h"
#include "macros.h"
#include "stm32f4xx_hal.h"

using namespace QP;

Q_DEFINE_THIS_MODULE("Active Config")

void QP_PreInitialize(void) {
    QF::init();
    QP_AllocateMemoryPools();
    QP_AllocateSubscriberLists();
}

int QP_InitializeRun(void) {
    QP_StartActiveObjectsAndPublishBootTimeEvents();
    return QF::run();
}

void QP_Systick_Handler(void) {
    QXK_ISR_ENTRY();
    QP::QF::TICK_X(0, NULL);
    QXK_ISR_EXIT();
}

void QP_QXK_ISR_ENTRY(void) {
	QXK_ISR_ENTRY();
}
void QP_QXK_ISR_EXIT(void) {
	QXK_ISR_EXIT();
}

// Must be overridden
__weak void QP_StartActiveObjectsAndPublishBootTimeEvents(void) {
   (void)0;
}

// Must be overridden
__weak void QP_AllocateSubscriberLists(void) {
   (void)0;
}

// MEMORY POOL ALLOCATION CODE START
enum {
    EVT_SIZE_SMALL = 32,
    EVT_SIZE_MEDIUM = 64,
    EVT_SIZE_LARGE = 256,
};

enum {
    EVT_COUNT_SMALL = 128,
    EVT_COUNT_MEDIUM = 16,
    EVT_COUNT_LARGE = 4
};

uint32_t evtPoolSmall[ROUND_UP_DIV_4(EVT_SIZE_SMALL * EVT_COUNT_SMALL)];
uint32_t evtPoolMedium[ROUND_UP_DIV_4(EVT_SIZE_MEDIUM * EVT_COUNT_MEDIUM)];
uint32_t evtPoolLarge[ROUND_UP_DIV_4(EVT_SIZE_LARGE * EVT_COUNT_LARGE)];

__weak void QP_AllocateMemoryPools(void) {
    QF::poolInit(evtPoolSmall, sizeof(evtPoolSmall), EVT_SIZE_SMALL);
    QF::poolInit(evtPoolMedium, sizeof(evtPoolMedium), EVT_SIZE_MEDIUM);
    QF::poolInit(evtPoolLarge, sizeof(evtPoolLarge), EVT_SIZE_LARGE);
}
// MEMORY POOL ALLOCATION CODE END

namespace QP {

__weak void QF::onCleanup(void) {
    (void)0;
}

__weak void QXK::onIdle(void) {
  QF_INT_DISABLE();
  QF_INT_ENABLE();
}

__weak void QF::onStartup(void) {
    (void)0;
}
extern "C" __weak void Q_onAssert(char const *module, int loc) {
    //
    // NOTE: add here your application-specific error handling
    //
    (void)module;
    (void)loc;
    QS_ASSERTION(module, loc, static_cast<uint32_t>(10000U));
    // for debugging, hang on in an endless loop...
    for (;;) {
    }
    //NVIC_SystemReset();
}

#ifdef Q_SPY
__weak bool QS::onStartup(void const *arg) {
  return true;
}
__weak void QS::onCleanup(void) {
  return;
}
__weak QSTimeCtr QS::onGetTime(void) {
  return 0;
}
__weak void QS::onFlush(void) {
  return;
}
__weak void QS::onReset(void) {
  return;
  //NVIC_SystemReset();
}
__weak void QS::onCommand(uint8_t cmdId, uint32_t param) {
  return;
}
#endif // Q_SPY
}


