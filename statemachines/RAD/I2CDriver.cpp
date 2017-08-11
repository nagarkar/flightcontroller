/*
 * I2CDriver.cpp
 *
 *  Created on: Aug 6, 2017
 *      Author: chinm_000
 */

#include <I2CDriver.h>

namespace RAD {

I2CDriver::I2CDriver(ComponentAO * parent)
	: ComponentImpl(parent, "I2CDriver") {
}

I2CDriver::~I2CDriver() {
	// TODO Auto-generated destructor stub
}

ResourceRequest I2CDriver::GetRequiredResources() {
	ResourceRequest req;
	req.sequenceNumber = m_sequenceNumber++;
	req.resources = &resources[0];
	req.nResources = ARRAY_COUNT(resources);
	return req;
}

static void I2C_Initialize(RAD_I2C_Config &i2cConfig);
static void I2C_Reset( I2C_HandleTypeDef &i2cHandle);
static uint8_t I2C_EXPBD_Init( RAD_I2C_Config &i2cConfig );
static void I2C_EXPBD_MspInit( RAD_I2C_Config &i2cConfig );

static void CreateConfiguration(RAD_I2C_Config &i2cConfig) {

	/* I2C_EXPBD peripheral configuration */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
    i2cConfig.i2cHandle.Init.ClockSpeed = 400000;
    i2cConfig.i2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
#endif

#if (defined (USE_STM32L0XX_NUCLEO))
    i2cConfig.i2cHandle.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_400KHZ;    /* 400KHz */
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
    i2cConfig.i2cHandle.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_400KHZ;    /* 400KHz */
#endif

    i2cConfig.i2cHandle.Init.OwnAddress1    = 0x33;
    i2cConfig.i2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2cConfig.i2cHandle.Instance            = i2cConfig.instance;

	GPIO_InitTypeDef GPIO_InitStruct = i2cConfig.GPIO_InitStruct;
  GPIO_InitStruct.Pin        = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
#endif

#if (defined (USE_STM32L1XX_NUCLEO))
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
#endif
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = GPIO_AF4_I2C1;

	I2C_Initialize(i2cConfig);
}

static void I2C_Initialize(RAD_I2C_Config &i2cConfig)
{

  /* De-initialize the I2C comunication bus */
  HAL_I2C_DeInit( &i2cConfig.i2cHandle);

  /* Reset I2C */
  I2C_Reset(i2cConfig.i2cHandle);

  /* Re-Initiaize the I2C comunication bus */
  I2C_EXPBD_Init(i2cConfig);
}

static uint8_t I2C_EXPBD_Init( RAD_I2C_Config &i2cConfig )
{

  if(HAL_I2C_GetState( &i2cConfig.i2cHandle) == HAL_I2C_STATE_RESET )
  {
    /* Init the I2C */
    I2C_EXPBD_MspInit(i2cConfig);
    HAL_I2C_Init( &i2cConfig.i2cHandle);
  }

  if( HAL_I2C_GetState( &i2cConfig.i2cHandle) == HAL_I2C_STATE_READY )
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

/**
 * From the Stm32 Reference Manual: "This (SWRST) bit can be used to reinitialize the peripheral after an error or a locked state. As an
 * example, if the BUSY bit is set and remains locked due to a glitch on the bus, the SWRST bit can be used to exit from this state
 */
static void I2C_Reset( I2C_HandleTypeDef &i2cHandle  ) {
  i2cHandle.Instance->CR1 |= I2C_CR1_SWRST;
  i2cHandle.Instance->CR1 ^= I2C_CR1_SWRST;
}


static void I2C_EXPBD_MspInit( RAD_I2C_Config &i2cConfig ) {

  /* Enable the I2C_EXPBD peripheral clock */
  __I2C1_CLK_ENABLE();

  /* Enable I2C GPIO clocks */
  if (i2cConfig.gpioInstance == GPIOA) 			{ __GPIOA_CLK_ENABLE(); }
  else if (i2cConfig.gpioInstance == GPIOB) { __GPIOB_CLK_ENABLE(); }
  else if (i2cConfig.gpioInstance == GPIOC) { __GPIOC_CLK_ENABLE(); }
  else if (i2cConfig.gpioInstance == GPIOD) { __GPIOD_CLK_ENABLE(); }
  else { while(1);}


  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/

  HAL_GPIO_Init( i2cConfig.gpioInstance, &i2cConfig.GPIO_InitStruct);

  if (i2cConfig.instance == I2C1) {
		  /* Enable the I2C_EXPBD peripheral clock */
		  __I2C1_CLK_ENABLE();
		  /* Force the I2C peripheral clock reset */
		  __I2C1_FORCE_RESET();
		  /* Release the I2C peripheral clock reset */
		  __I2C1_RELEASE_RESET();
		  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
		  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0x00);
		  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
		#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
		  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
		  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0x00);
		  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
		#endif
  } else {
  		  while(1);
  }
}
} // namespace
