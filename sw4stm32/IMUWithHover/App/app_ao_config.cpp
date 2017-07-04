#include "app_ao_config.h"
#include "active_config.h"
#include "active_events.h"
#include "active_log.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "macros.h"
#include "bsp.h"
#include "AttitudeGuage.h"

#define MY_GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */

using namespace QP;
using namespace Attitude;
using namespace std;
using namespace StdEvents;

/*** DEFINE SIGNALS **/
SMARTENUM_DEFINE_NAMES(Signal, SIG_LIST)
SMARTENUM_DEFINE_GET_VALUE_FROM_STRING(Signal, SIG_LIST)


/** QP CONFIGURATION **/
static AttitudeGuage guage;

QP::QSubscrList subscrSto[MAX_PUB_SIG];

/** PRIVATE FUNCTION PROTOTYPES **/
/** End **/

void QP_StartActiveObjectsAndPublishBootTimeEvents(void) {

	//TOGGLE_EVENT_LOGGING();

	guage.Start(PRIO_ATTITUDE_GUAGE_PRIO);
	Evt * e = new Evt(ATTITUDE_GUAGE_START_REQ_SIG);
	QF::PUBLISH(e, NULL);
}

void QP_AllocateSubscriberLists(void) {
	QF::psInit(subscrSto, Q_DIM(subscrSto)); // init publish-subscribe
}

/** END QP CONFIGURATION **/

/** BSP OVERRIDES **/

// End BSP OVERRIDES

extern "C" void WWDG_IRQHandler( void ) {
	AO_InfiniteLoop();
}

//extern "C" void HardFault_Handler( void ) {
//	AO_InfiniteLoop();
//}

extern "C" void MemManage_Handler( void ) {
	AO_InfiniteLoop();
}

extern "C" void BusFault_Handler( void ) {
	AO_InfiniteLoop();
}

extern "C" void UsageFault_Handler( void ) {
	AO_InfiniteLoop();
}

extern "C" void DebugMon_Handler( void ) {
	AO_InfiniteLoop();
}

void AO_InfiniteLoop( void ) {
	static int tick = 0;
	for (;;) {
		tick++;
	}
}

void hard_fault_handler_c(uint32_t *hardfault_args){
  volatile uint32_t stacked_r0 = 	((uint32_t)hardfault_args[0]) ;
  volatile uint32_t stacked_r1 = 	((uint32_t)hardfault_args[1]) ;
  volatile uint32_t stacked_r2 = 	((uint32_t)hardfault_args[2]) ;
  volatile uint32_t stacked_r3 = 	((uint32_t)hardfault_args[3]) ;
  volatile uint32_t stacked_r12 = 	((uint32_t)hardfault_args[4]) ;
  volatile uint32_t stacked_lr = 	((uint32_t)hardfault_args[5]) ;
  volatile uint32_t stacked_pc = 	((uint32_t)hardfault_args[6]) ;
  volatile uint32_t stacked_psr = 	((uint32_t)hardfault_args[7]) ;

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  volatile uint32_t _CFSR = (*((volatile uint32_t *)(0xE000ED28))) ;

  // Hard Fault Status Register
  volatile uint32_t _HFSR = (*((volatile uint32_t *)(0xE000ED2C))) ;

  // Debug Fault Status Register
  volatile uint32_t _DFSR = (*((volatile uint32_t *)(0xE000ED30))) ;

  // Auxiliary Fault Status Register
  volatile uint32_t _AFSR = (*((volatile uint32_t *)(0xE000ED3C))) ;

  // Bus Fault Address Register
  volatile uint32_t _BFAR = (*((volatile uint32_t *)(0xE000ED38))) ;

  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  volatile uint32_t _MMAR = (*((volatile uint32_t *)(0xE000ED34))) ;

  __asm("BKPT #0\n") ; // Break into the debugger
}
