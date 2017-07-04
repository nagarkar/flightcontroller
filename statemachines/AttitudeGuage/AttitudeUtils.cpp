#include "AttitudeUtils.h"

// QP/Framework
#include "bsp.h"
#include "active_events.h"
#include "active_log.h"

// STM32 Supporting Libs
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include <stdbool.h>

#include "LSM6DS0_ACC_GYRO_driver.h"
#include "LSM6DS0_ACC_GYRO_driver_HL.h"

#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"


using namespace StdEvents;

namespace Attitude {

float AttitudeUtils::cycleCounterAvg = 0;
int AttitudeUtils::cycleCounterCount = 0;
int AttitudeUtils::counter = 0;

status_t AttitudeUtils::Initialize(DrvStatusTypeDef &result, void **hhandle) {

	cycleCounterAvg = 0;
	cycleCounterCount = 0;
	static uint8_t isInitialized = 0;

	volatile status_t status;

	if (isInitialized == 1) {
		result = BSP_ACCELERO_DeInit(hhandle);
		isInitialized = 0;
	}
	result = BSP_ACCELERO_Init(LSM6DS0_X_0, hhandle);
	isInitialized = 1;
	void *handle = *hhandle;
	if (result == COMPONENT_OK) {
		result = BSP_ACCELERO_Sensor_Enable(handle);
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) != GPIO_PIN_RESET) {
			LSM6DS0_ACC_GYRO_W_ResetSW(handle, LSM6DS0_ACC_GYRO_SW_RESET_YES);
		}
	}
	if (result != COMPONENT_OK) {
		return Initialize(result, hhandle);
	}


	// CTRL_REG1_G Register
	//status = LSM6DS0_ACC_GYRO_W_GyroDataRate(handle, LSM6DS0_ACC_GYRO_ODR_G_952Hz);
	status = LSM6DS0_ACC_GYRO_W_GyroDataRate(handle, LSM6DS0_ACC_GYRO_ODR_G_15Hz);
	//status = LSM6DS0_ACC_GYRO_W_GyroBandwidthSelection(handle, LSM6DS0_ACC_GYRO_BW_G_ULTRA_HIGH);
	//status = LSM6DS0_ACC_GYRO_W_GyroFullScale(handle, LSM6DS0_ACC_GYRO_FS_G_2000dps);
	status = LSM6DS0_ACC_GYRO_W_AutoIndexOnMultiAccess(handle, LSM6DS0_ACC_GYRO_IF_ADD_INC_ENABLE);


	// CTRL_REG4 Register
	status = LSM6DS0_ACC_GYRO_W_AccelerometerDataRate(handle, LSM6DS0_ACC_GYRO_ODR_XL_50Hz);
	//status = LSM6DS0_ACC_GYRO_W_AccelerometerDataRate(handle, LSM6DS0_ACC_GYRO_ODR_XL_952Hz);
	status = LSM6DS0_ACC_GYRO_W_AccelerometerFullScale(handle, LSM6DS0_ACC_GYRO_FS_XL_2g);
	//status = LSM6DS0_ACC_GYRO_W_AccelerometerBandWitdthSelection(handle, LSM6DS0_ACC_GYRO_BW_SCAL_ODR_WITH_BW_XL);
	//status = LSM6DS0_ACC_GYRO_W_AccelerometerFilterBandwidth(handle, LSM6DS0_ACC_GYRO_BW_XL_105Hz);

	// CTRL_REG7_XL Register
	status = LSM6DS0_ACC_GYRO_W_AccelerometerAxisX(handle, LSM6DS0_ACC_GYRO_XEN_XL_ENABLE);
	status = LSM6DS0_ACC_GYRO_W_AccelerometerAxisY(handle, LSM6DS0_ACC_GYRO_YEN_XL_ENABLE);
	status = LSM6DS0_ACC_GYRO_W_AccelerometerAxisZ(handle, LSM6DS0_ACC_GYRO_ZEN_XL_ENABLE);

	status = LSM6DS0_ACC_GYRO_W_GyroAxisX(handle, LSM6DS0_ACC_GYRO_XEN_G_ENABLE);
	status = LSM6DS0_ACC_GYRO_W_GyroAxisY(handle, LSM6DS0_ACC_GYRO_YEN_G_ENABLE);
	status = LSM6DS0_ACC_GYRO_W_GyroAxisZ(handle, LSM6DS0_ACC_GYRO_ZEN_G_ENABLE);

	status = LSM6DS0_ACC_GYRO_W_AccelerometerHighResolutionMode(handle, LSM6DS0_ACC_GYRO_HR_ENABLE);
	status = LSM6DS0_ACC_GYRO_W_AccelerometerCutOff_filter(handle, LSM6DS0_ACC_GYRO_DCF_ODR_DIV_9);
	status = LSM6DS0_ACC_GYRO_W_AccelerometerFilteredDataSelection(handle, LSM6DS0_ACC_GYRO_FDS_FILTER_ENABLE);

	/////// 	INT_CTRL Register 	//////////
	status = LSM6DS0_ACC_GYRO_W_XL_DataReadyOnINT(handle, LSM6DS0_ACC_GYRO_INT_DRDY_XL_ENABLE);
	status = LSM6DS0_ACC_GYRO_W_GYRO_DataReadyOnINT(handle, LSM6DS0_ACC_GYRO_INT_DRDY_G_ENABLE);

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	HAL_StatusTypeDef halStatus;
	u8_t acc_8_t[6] = {0, 0, 0, 0, 0, 0};
	halStatus = HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_XL, I2C_MEMADD_SIZE_8BIT, acc_8_t, 6, NUCLEO_I2C_EXPBD_TIMEOUT_MAX);
	//status = HAL_I2C_Mem_Read_DMA(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_XL, I2C_MEMADD_SIZE_8BIT, acc_8_t, 6);
	//status = HAL_I2C_Mem_Read_IT(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_XL, I2C_MEMADD_SIZE_8BIT, acc_8_t, 6);
	//Log::Print("Finished Reading first acc data");
	return MEMS_SUCCESS;
}
status_t  AttitudeUtils::GetAttitude(Acceleration &acc, AngularRate &angRate, void *handle) {
	uint32_t marker = GetPerfCycle();
	BSP_LED_Off(LED2); // If the LED2 is off, it means the program hanged somewhere in this fn.

	u8_t value;
	LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, &value, 1);

	if ((value & LSM6DS0_ACC_GYRO_GDA_MASK) != LSM6DS0_ACC_GYRO_GDA_UP) {
		return MEMS_SUCCESS;
	}

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	HAL_StatusTypeDef status;

	u8_t acc_8_t[6] = {0, 0, 0, 0, 0, 0};
	status = HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_XL, I2C_MEMADD_SIZE_8BIT, acc_8_t, 6, NUCLEO_I2C_EXPBD_TIMEOUT_MAX);


	status = HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_G, I2C_MEMADD_SIZE_8BIT, acc_8_t, 6, NUCLEO_I2C_EXPBD_TIMEOUT_MAX);

	//status = HAL_I2C_Mem_Read_IT(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_G, I2C_MEMADD_SIZE_8BIT, acc_8_t, 6);
	counter++;
	uint32_t cycleCounter = GetElapsedCycles(marker);
	cycleCounterAvg = (cycleCounterAvg * ((float)cycleCounterCount)/(cycleCounterCount + 1));
	cycleCounterAvg += ((float)cycleCounter)/(cycleCounterCount + 1);
	cycleCounterCount++;
	BSP_LED_On(LED2);
	return MEMS_SUCCESS;
}
}// namespace