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
#include "active_log.h"
#include "app_ao_config.h"
#include "qpcpp.h"
#include "macros.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "bsp.h"

Q_DEFINE_THIS_MODULE("active_config")

using namespace QP;


QP::QSubscrList subscrSto[MAX_PUB_SIG];

#ifdef Q_SPY

    QP::QSTimeCtr QS_tickTime_;
    QP::QSTimeCtr QS_tickPeriod_;
    static uint8_t l_SysTick_Handler;

    #define UART_BAUD_RATE      115200U
    #define UART_FR_TXFE        (1U << 7)
    #define UART_FR_RXFE        (1U << 4)
    #define UART_TXFIFO_DEPTH   16U

#endif


void QP_PreInitialize(void) {
    QF::init();
    QP_AllocateMemoryPools();
    QP_AllocateSubscriberLists();
}

int QP_InitializeRun(void) {

	if (QS_INIT((void *)0) == 0U) { // initialize the QS software tracing
        //Q_ERROR();
    }
    QS_OBJ_DICTIONARY(&l_SysTick_Handler);

    QP_StartActiveObjectsAndPublishBootTimeEvents();
    return QF::run();
}

extern "C" void HAL_SYSTICK_Callback(void) {
	QP_QXK_ISR_ENTRY();
	QP_Systick_Handler();
	QP_QXK_ISR_EXIT();
}

void QP_Systick_Handler(void) {
	uint32_t tmp;
	#ifdef Q_SPY
		{
			tmp = SysTick->CTRL; // clear SysTick_CTRL_COUNTFLAG
			QS_tickTime_ += QS_tickPeriod_; // account for the clock rollover
		}
		uint_fast8_t const tickRate = 0U;
		void const * const sender = &l_SysTick_Handler;
		QF::TICK_X(tickRate,  sender); // process time events for rate 0
	#else
		QP::QF::TICK_X(0U); // process time events for rate 0
	#endif
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

__weak void QP_AllocateSubscriberLists(void) {
	QF::psInit(subscrSto, Q_DIM(subscrSto)); // init publish-subscribe
}

// MEMORY POOL ALLOCATION CODE START
enum {
    EVT_SIZE_SMALL = 16,
    EVT_SIZE_MEDIUM = 36,
    EVT_SIZE_LARGE = 64,
};

enum {
    EVT_COUNT_SMALL = 128,
    EVT_COUNT_MEDIUM = 16,
    EVT_COUNT_LARGE = 100
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

static void flushToUart(bool useQp) {
	uint16_t nBytes = 500;
    QF_INT_DISABLE();
    uint8_t const * buffer = QS::getBlock(&nBytes);
    QF_INT_ENABLE();
    if (useQp) {
        bool status = false;
    	if (buffer != NULL) {
        	status = StdEvents::Log::Write((const char *)buffer, nBytes, useQp);
        }
        if (status && StdEvents::Log::m_writeSuccessSig != 0) {
        	QF::PUBLISH(new QEvt(StdEvents::Log::m_writeSuccessSig), NULL);
        }
    } else {
    	HAL_StatusTypeDef hal_status = HAL_TIMEOUT;
    	while (hal_status  != HAL_OK) {
    		hal_status = BSP_XMIT_ON_DEFAULT_UART(buffer, nBytes);
    	}
    }
}

__weak void QXK::onIdle(void) {
  #ifdef Q_SPY
	flushToUart(true);
    /*
    if ((huart2.Instance->SR & 0x0080U) != 0U) {  // TX done?
        QF_INT_DISABLE();
        uint16_t b = QS::getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {  // not End-Of-Data?
            huart2.Instance->DR  = (b & 0xFFU);  // put into the DR register
        }
    }
    */
#elif defined NDEBUG
    // Put the CPU and peripherals to the low-power mode.
    // you might need to customize the clock management for your application,
    // see the datasheet for your particular Cortex-M3 MCU.
    //
    __WFI(); // Wait-For-Interrupt
#endif
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

#define TICKS_PER_SEC 1000

#ifdef Q_SPY
__weak bool QS::onStartup(void const *arg) {
	static uint8_t qsBuf[2*1024]; // buffer for Quantum Spy
    initBuf(qsBuf, sizeof(qsBuf));

    QS_tickPeriod_ = SystemCoreClock / TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; // to start the timestamp at zero

    QS_FILTER_ON(ACTIVE_LOGGER);
    QS_FILTER_ON(QS_QEP_STATE_ENTRY);
    QS_FILTER_ON(QS_QEP_STATE_EXIT);
    QS_FILTER_ON(QS_QEP_STATE_INIT);
   QS_FILTER_ON(QS_QEP_INIT_TRAN);
    QS_FILTER_ON(QS_QEP_INTERN_TRAN);
    QS_FILTER_ON(QS_QEP_TRAN);
    QS_FILTER_ON(QS_QEP_IGNORED);
    QS_FILTER_ON(QS_QEP_DISPATCH);
    QS_FILTER_ON(QS_QEP_UNHANDLED);
    QS_FILTER_ON(QS_QF_PUBLISH);

  return true;
}
__weak void QS::onCleanup(void) {
  return;
}
__weak QSTimeCtr QS::onGetTime(void) {
	if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) { // not set?
	        return QS_tickTime_ - static_cast<QSTimeCtr>(SysTick->VAL);
	}
	else { // the rollover occured, but the SysTick_ISR did not run yet
		return QS_tickTime_ + QS_tickPeriod_
			   - static_cast<QSTimeCtr>(SysTick->VAL);
	}
}
__weak void QS::onFlush(void) {
	flushToUart(false);
	/*
    uint16_t b;
    QF_INT_DISABLE();
    while ((b = getByte()) != QS_EOD) { // while not End-Of-Data...
        QF_INT_ENABLE();
        while ((huart2.Instance->SR & 0x0080U) == 0U) { // while TXE not empty
        }
        huart2.Instance->DR  = (b & 0xFFU);  // put into the DR register
        QF_INT_DISABLE();
    }
    QF_INT_ENABLE();
    */
}

__weak void QS::onReset(void) {
  NVIC_SystemReset();
}
__weak void QS::onCommand(uint8_t cmdId, uint32_t param1, uint32_t param2, uint32_t param3) {
   (void)0;
}
#endif // Q_SPY
}
