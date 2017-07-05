#include "app_ao_config.h"
#include "active_config.h"
#include "active_events.h"
#include "active_log.h"
#include "macros.h"
#include "bsp.h"
#include "AttitudeGuage.h"
#include "UartAct.h"
#include "UartCommander.h"

#define MY_GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */

using namespace QP;
using namespace Attitude;
using namespace std;
using namespace StdEvents;
using namespace AOs;

/*** DEFINE SIGNALS **/
SMARTENUM_DEFINE_NAMES(Signal, SIG_LIST)
SMARTENUM_DEFINE_GET_VALUE_FROM_STRING(Signal, SIG_LIST)

/** QP CONFIGURATION **/
static AttitudeGuage guage;
static UartAct uart(&huart2);
static UartCommander uartCmd(&uart.GetInFifo());

QP::QSubscrList subscrSto[MAX_PUB_SIG];

/** PRIVATE FUNCTION PROTOTYPES **/
/** End **/

void QP_StartActiveObjectsAndPublishBootTimeEvents(void) {

	Evt * e;
	//TOGGLE_EVENT_LOGGING();

	uart.Start(PRIO_UART2_ACT);
	e = new Evt(UART_ACT_START_REQ_SIG);
	QF::PUBLISH(e, NULL);

	uartCmd.Start(PRIO_UART2_COMMANDER);
	e = new Evt(UART_COMMANDER_START_REQ_SIG);
	QF::PUBLISH(e, NULL);

	guage.Start(PRIO_ATTITUDE_GUAGE_PRIO);
	e = new Evt(ATTITUDE_GUAGE_START_REQ_SIG);
	QF::PUBLISH(e, NULL);

	QF::PUBLISH(new Evt(UART_COMMANDER_SHOW_USAGE_SIG), NULL);

}

void QP_AllocateSubscriberLists(void) {
	QF::psInit(subscrSto, Q_DIM(subscrSto)); // init publish-subscribe
}
