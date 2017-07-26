/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdio.h>

#include "controller.h"
			
GPIO_InitTypeDef GPIO_InitStructure;

//#define DEBUG_USART

extern void TimingDelay_Decrement(void);
static __IO uint32_t TimingDelay;
void Delay(__IO uint32_t nTime);


uint8_t buffr[300]; 	// Buffer for sprintf function
uint8_t rx_buffr[300]; 	// Buffer for serial receive
volatile uint16_t rx_buffr_ptr = 0;
volatile uint8_t rx_buffr_done = 0;

static void usart2_init(void) {
	// USART peripheral initialization settings
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	//Configure USART2 pins: Tx (PA2) and Rx (PA3)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Configure USART2 setting:
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2,ENABLE);

	// USART2 receive Interrupt Enable
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART2_IRQn);
}

static void usart2_putc(uint8_t c) {
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
}

static void usart2_puts(uint8_t *c) {
	while(*c) {
		usart2_putc(*c);
		c++;
	}
}

void USART2_IRQHandler(void) {
	uint8_t rcvd;
    /* RXNE handler */
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {

    	rcvd = (uint8_t)USART_ReceiveData(USART2);
//    	usart2_putc(rcvd);

    	if(rcvd == '\n' || rcvd == '\r') {
    		rx_buffr[rx_buffr_ptr++] = '\n';
    		rx_buffr_ptr = 0;
    		rx_buffr_done = 1;
    	} else {
    		rx_buffr[rx_buffr_ptr++] = rcvd;
    	}
    }
}

void Delay(__IO uint32_t nTime) {
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

void TimingDelay_Decrement(void) {
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

int main(void) {
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);

	SysTick_Config(SystemCoreClock / 1000);

//	usart1_init();
	usart2_init();

	sprintf(buffr, "USART Initialized !\r\n");
	usart2_puts(buffr);

	float32_t tmpVal;
	uint8_t floatBuffr[4];

	float32_t t = 0.0;
	stateStruct mystateStruct;
	destStateStruct mydestStateStruct;
	robotParamsStruct myrobotParamsStruct;
	resultFM myTestResult;

	for(;;) {

		float32_t numVals[22];

		if(rx_buffr_done) { // If all the reception complete

			uint8_t j = 0;

			for (uint8_t i = 0; i < 22; i++) {
				memcpy(&tmpVal, &rx_buffr[j], 4);
				numVals[i] = tmpVal;
				j += 4;
			}

			// ---------- Observe the input data -------------
			#if 1
				sprintf(buffr, "--------------------------------- Inputs ---------------------------------------------- \r\n");
				usart2_puts(buffr);
				for(uint8_t i = 0; i < 22; i++) {
					sprintf(buffr, "%d: %f\t",  i+1, numVals[i]);
					usart2_puts(buffr);
				}

				sprintf(buffr, "\r\n");
				usart2_puts(buffr);
				sprintf(buffr, "--------------------------------------------------------------------------------------- \r\n");
				usart2_puts(buffr);
			#endif

			// ----------------- Copy data to pass into the controller -------------------

			uint8_t arryIndex = 0;

			// Copy Time value
			t = numVals[arryIndex++];

			// Copy state position information
			mystateStruct.pos[I] = numVals[arryIndex++]; mystateStruct.pos[J] = numVals[arryIndex++]; mystateStruct.pos[K] = numVals[arryIndex++];

			// Copy state velocity information and so on...
			mystateStruct.vel[I] = numVals[arryIndex++]; mystateStruct.vel[J] = numVals[arryIndex++]; mystateStruct.vel[K] = numVals[arryIndex++];

			mystateStruct.rot[I] = numVals[arryIndex++]; mystateStruct.rot[J] = numVals[arryIndex++]; mystateStruct.rot[K] = numVals[arryIndex++];

			mystateStruct.omega[I] = numVals[arryIndex++]; mystateStruct.omega[J] = numVals[arryIndex++]; mystateStruct.omega[K] = numVals[arryIndex++];

			mydestStateStruct.pos[I] = numVals[arryIndex++]; mydestStateStruct.pos[J] = numVals[arryIndex++]; mydestStateStruct.pos[K] = numVals[arryIndex++];

			mydestStateStruct.vel[I] = numVals[arryIndex++]; mydestStateStruct.vel[J] = numVals[arryIndex++]; mydestStateStruct.vel[K] = numVals[arryIndex++];

			mydestStateStruct.acc[I] = numVals[arryIndex++]; mydestStateStruct.acc[J] = numVals[arryIndex++]; mydestStateStruct.acc[K] = numVals[arryIndex++];

			mydestStateStruct.yaw = 0;
			mydestStateStruct.yawdot = 0;

			myrobotParamsStruct.mass = 0.180;
			myrobotParamsStruct.gravity = 9.8100;

			myTestResult = controller(t, &mystateStruct, &mydestStateStruct, &myrobotParamsStruct);

			#if 1
				// ------------ See the result ------------------
				sprintf(buffr, "---------------------------------- Results ------------------------------------------- \r\n");
				usart2_puts(buffr);
				sprintf(buffr, "F: %f\tM[0]: %f\tM[1]: %f\tM[2]: %f\r\n", myTestResult.F, myTestResult.M[0], myTestResult.M[1], myTestResult.M[2]);
				usart2_puts(buffr);
				sprintf(buffr, "---------------------------------------------------------------------------------------- \r\n");
				usart2_puts(buffr);
			#endif


			rx_buffr_done = 0;

		} else {
//			sprintf(buffr, "\r\nThis is a floating point number: %f\r\n", 0.000134);
//			usart2_puts(buffr);
//			Delay(500);
		}
	}
}
