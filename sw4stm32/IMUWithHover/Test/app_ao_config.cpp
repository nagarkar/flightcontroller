#include "app_ao_config.h"
#include "active_config.h"
#include "active_events.h"
#include "active_log.h"
#include "macros.h"
#include "bsp.h"
#include "AttitudeGuage.h"
#include "UartAct.h"
#include "UartCommander.h"
#include "I2CDriver.h"
#include "DeviceAO.h"

using namespace QP;
using namespace Attitude;
using namespace std;
using namespace StdEvents;
using namespace AOs;
using namespace RAD;

/** QP CONFIGURATION **/
static AttitudeGuage guage;
static UartAct uart(&huart2);
static UartCommander uartCmd(&uart.GetInFifo());
static DeviceAO device("DEVICE");
static I2CDriver i2cDriver(&device);

SMARTENUM_DEFINE_NAMES(Signal, SIG_LIST)
SMARTENUM_DEFINE_GET_VALUE_FROM_STRING(Signal, SIG_LIST)

/** PRIVATE FUNCTION PROTOTYPES **/
/** End **/

void QP_StartActiveObjectsAndPublishBootTimeEvents(void) {

	Evt * e;
	uart.Start(PRIO_UART2_ACT);
	QS_SIG_DICTIONARY(UART_ACT_START_REQ_SIG, &uart);
	e = new Evt(UART_ACT_START_REQ_SIG);
	QF::PUBLISH(e, NULL);

	device.Start(PRIO_DEVICE);
	i2cDriver.Start(PRIO_I2C_DRIVER);

	/*
	uartCmd.Start(PRIO_UART2_COMMANDER);
	e = new Evt(UART_COMMANDER_START_REQ_SIG);
	QF::PUBLISH(e, NULL);

	assert_param(guage.Start(PRIO_ATTITUDE_GUAGE_PRIO) >= 0);
	e = new Evt(ATTITUDE_GUAGE_START_REQ_SIG);
	QF::PUBLISH(e, NULL);

	QF::PUBLISH(new Evt(UART_COMMANDER_SHOW_USAGE_SIG), NULL);
	*/
}

void QS::onTestSetup(void) {
}
//............................................................................
void QS::onTestTeardown(void) {
}

void QS::onCommand(uint8_t cmdId, uint32_t param1, uint32_t param2, uint32_t param3) {
   (void)param1;
   (void)param2;
   (void)param3;

   //printf("<TARGET> Command id=%d param=%d\n", (int)cmdId, (int)param);
   switch (cmdId) {
      case 0U: {
          break;
      }
      default:
          break;
   }
}
