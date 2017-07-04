/*
 * app_ao_config.h
 *
 *  Created on: May 19, 2017
 *      Author: chinm_000
 */
#ifndef APP_AO_CONFIG_H_
#define APP_AO_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "qpcpp.h"
#include "active_enum.h"

#define _ACTIVE_LOG_ENABLED_
using namespace QP;

#define SIG_LIST(m)  \
	m(Signal, DUMMY_NULL)    \
	m(Signal, ENTRY_SIG)    \
	m(Signal, EXIT_SIG)		\
	m(Signal, INIT_SIG)		\
	m(Signal, DONT_CARE_SIG = QP::Q_USER_SIG)	/* 4 */	\
	\
	m(Signal, SYSTEM_START_REQ_SIG)    \
	m(Signal, USER_LED_START_REQ_SIG)    \
	m(Signal, USER_LED_START_CFM_SIG)    \
	m(Signal, USER_LED_STOP_REQ_SIG)    \
	m(Signal, USER_LED_STOP_CFM_SIG)    \
	m(Signal, USER_LED_ON_REQ_SIG)    \
	m(Signal, USER_LED_ON_CFM_SIG)    \
	m(Signal, USER_LED_OFF_REQ_SIG)    \
	m(Signal, USER_LED_OFF_CFM_SIG)    \
	m(Signal, USER_LED_STATE_TIMER_SIG)    \
	m(Signal, USER_LED_DONE_SIG)    			/* 11, 15 */ \
	\
	m(Signal, UART_ACT_START_REQ_SIG)    \
	m(Signal, UART_ACT_START_CFM_SIG)    \
	m(Signal, UART_ACT_STOP_REQ_SIG)    \
	m(Signal, UART_ACT_STOP_CFM_SIG)    \
	m(Signal, UART_ACT_FAIL_IND_SIG)    \
	m(Signal, UART_ACT_STATE_TIMER_SIG)    \
	m(Signal, UART_ACT_START_SIG)    \
	m(Signal, UART_ACT_DONE_SIG)    \
	m(Signal, UART_ACT_FAIL_SIG)    			/* 9, 24 */\
	\
	m(Signal, UART_OUT_START_REQ_SIG)    \
	m(Signal, UART_OUT_START_CFM_SIG)    \
	m(Signal, UART_OUT_STOP_REQ_SIG)    \
	m(Signal, UART_OUT_STOP_CFM_SIG)    \
	m(Signal, UART_OUT_FAIL_IND_SIG)    \
	m(Signal, UART_OUT_WRITE_REQ_SIG)    \
	m(Signal, UART_OUT_WRITE_CFM_SIG)    \
	m(Signal, UART_OUT_EMPTY_IND_SIG)    \
	m(Signal, UART_OUT_ACTIVE_TIMER_SIG)    \
	m(Signal, UART_OUT_DONE_SIG)    \
	m(Signal, UART_OUT_DMA_DONE_SIG)    \
	m(Signal, UART_OUT_CONTINUE_SIG)    \
	m(Signal, UART_OUT_HW_FAIL_SIG)    			/* 13, 37 */\
	\
	m(Signal, UART_IN_START_REQ_SIG)    \
	m(Signal, UART_IN_START_CFM_SIG)    \
	m(Signal, UART_IN_STOP_REQ_SIG)    \
	m(Signal, UART_IN_STOP_CFM_SIG)    \
	m(Signal, UART_IN_CHAR_IND_SIG)    \
	m(Signal, UART_IN_DATA_IND_SIG)    \
	m(Signal, UART_IN_FAIL_IND_SIG)    \
	m(Signal, UART_IN_ACTIVE_TIMER_SIG)    \
	m(Signal, UART_IN_DONE_SIG)    \
	m(Signal, UART_IN_DATA_RDY_SIG)    \
	m(Signal, UART_IN_DMA_RECV_SIG)    \
	m(Signal, UART_IN_OVERFLOW_SIG)    \
	m(Signal, UART_IN_HW_FAIL_SIG)    			/* 13 , 50*/\
	\
	m(Signal, UART_COMMANDER_STOP_REQ_SIG)    \
	m(Signal, UART_COMMANDER_START_REQ_SIG)    \
	m(Signal, UART_COMMANDER_STOP_CFM_SIG)    \
	m(Signal, UART_COMMANDER_START_CFM_SIG)    \
	m(Signal, UART_COMMANDER_SHOW_USAGE_SIG)  	/* 6, 56 */  \
	\
	m(Signal, ATTITUDE_GUAGE_STOP_REQ_SIG)    \
	m(Signal, ATTITUDE_GUAGE_STOP_CFM_SIG)    \
	m(Signal, ATTITUDE_GUAGE_START_REQ_SIG)    \
	m(Signal, ATTITUDE_GUAGE_START_CFM_SIG)    \
	m(Signal, ATTITUDE_GUAGE_INTERVAL_TIMER_SIG)    \
	m(Signal, ATTITUDE_DATA_AVAILABLE_SIG)	\
	m(Signal, ATTITUDE_CHANGED_SIG)	\
	m(Signal, ATTITUDE_GUAGE_FAILED_SIG)	\
	m(Signal, ATTITUDE_GUAGE_REINIT_SIG)			/* 8, 64 */\
	\
	m(Signal, SYSTEM_PIN_SET_INTERRUPT_SIG)	\
	m(Signal, SYSTEM_PIN_RESET_INTERRUPT_SIG)	/* 2, 67 */\
	\
	m(Signal, MAX_PUB_SIG)

SMARTENUM_DEFINE_ENUM(Signal, SIG_LIST)
SMARTENUM_DECLARE_NAMES(Signal, SIG_LIST)
SMARTENUM_DECLARE_GET_VALUE_FROM_STRING(Signal, SIG_LIST)

enum SignalAliases {
	UART_COMMANDER_CMD_IND_SIG = UART_IN_DATA_IND_SIG,
};

// Higher value corresponds to higher priority, so later enum values are higher
// priority. The maximum priority is defined in qf_port.h as QF_MAX_ACTIVE (32)
enum
{
	PRIO_UART2_ACT = 1 /* Can't start with zero; throws assertion*/,
	PRIO_UART2_COMMANDER,
	PRIO_BUMPER,
	PRIO_USER_LED,
	PRIO_ATTITUDE_GUAGE_PRIO,
	PRIO_DRIVE,
	PRIO_MAX = QF_MAX_ACTIVE
};

void AO_InfiniteLoop( void );

void hard_fault_handler_c(unsigned long *hardfault_args);

#ifdef __cplusplus
}
#endif

#endif /* AO_CONFIG_H_ */