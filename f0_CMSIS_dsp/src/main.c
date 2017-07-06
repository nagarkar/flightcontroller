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
#include "stm32f0xx_usart.h"

#include "controller.h"

#include "unity.h"
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>

static __IO uint32_t TimingDelay;
GPIO_InitTypeDef GPIO_InitStructure;

void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

extern void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

void init_usart(void) {
//	/ USART periferial initialization settings
	  USART_InitTypeDef USART_InitStructure;
	  GPIO_InitTypeDef	GPIO_InitStructure;

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	  //Configure USART2 pins: Rx (PA2) and Tx (PA3)
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  //Configure USART2 setting: ----------------------------
	  USART_InitStructure.USART_BaudRate = 9600;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART2, &USART_InitStructure);
	  USART_Cmd(USART2,ENABLE);
}

extern void usart_putchar(uint8_t c)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
}

extern void testControllerF(float32_t);
extern void testControllerM0(void);
extern void testControllerM1(void);
extern void testControllerM2(void);

void runTest(UnityTestFunction test) {
	if(TEST_PROTECT()) {
		test();
	}
}

//float32_t waypoints = {
//		0, 0, 0,
//	    1, 1, 1,
//	    2, 0, 2,
//	    3, -1, 1,
//	    4, 0, 0
//};

extern volatile resultFM testResult;

int main () {

	SysTick_Config(SystemCoreClock / 1000);

	init_usart();

	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);

	float32_t t = 0;
	stateStruct mystateStruct;
	destStateStruct mydestStateStruct;
	robotParamsStruct myrobotParamsStruct;
	resultFM myTestResult;

	mystateStruct.pos[I] = 9.009674869927633;
	mystateStruct.pos[J] = 0.008753151434503;
	mystateStruct.pos[K] = 0.022256301430048;

	mystateStruct.vel[I] = -0.043010356651461;
	mystateStruct.vel[J] = -0.033837508067482;
	mystateStruct.vel[K] = -0.112262557664142;

	mystateStruct.rot[I] = -0.009919528320879;
	mystateStruct.rot[J] = 0.013551423817576;
	mystateStruct.rot[K] = 0.000004043095471483318;

	mystateStruct.omega[I] = 0.010744247098932;
	mystateStruct.omega[J] = -0.023245143405660;
	mystateStruct.omega[K] = -0.00001786094352695585;


	mydestStateStruct.pos[0] = 9.009370914048350;
	mydestStateStruct.pos[1] = 0.007616540305385;
	mydestStateStruct.pos[2] = 0.023886843191290;

	mydestStateStruct.vel[0] = -0.043293866417708;
	mydestStateStruct.vel[1] = -0.034029130063384;
	mydestStateStruct.vel[2] = -0.112866216586475;

	mydestStateStruct.acc[I] = 0.137697142228255;
	mydestStateStruct.acc[J] = 0.100522384647306;
	mydestStateStruct.acc[K] = 0.375678418010267;

	mydestStateStruct.yaw = 0;
	mydestStateStruct.yawdot = 0;

	t = 14.000000000000064;

	myrobotParamsStruct.mass = 0.180;
	myrobotParamsStruct.gravity = 9.8100;


	myTestResult = controller(t, &mystateStruct, &mydestStateStruct, &myrobotParamsStruct);

	testResult = myTestResult;

//	float32_t resultF = testResult.F;
//	float32_t resultM0 = testResult.M[0];
//	float32_t resultM1 = testResult.M[1];
//	float32_t resultM2 = testResult.M[2];

	UnityBegin("..\\unit_test\\test_module.c");

	RUN_TEST(testControllerF, __LINE__);
	RUN_TEST(testControllerM0, __LINE__);
	RUN_TEST(testControllerM1, __LINE__);
	RUN_TEST(testControllerM2, __LINE__);

	return (UnityEnd());
}
