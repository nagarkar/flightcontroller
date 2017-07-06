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
float AttitudeUtils::accSensitivity = -1.0f;
float AttitudeUtils::gyroSensitivity = -1.0f;

#define STATUS_T_SET(status, fn)  status =  ((fn) == MEMS_ERROR) ? MEMS_ERROR: status
#define HAL_STATUS_SET(status, fn)  status = (status == HAL_OK) ? (fn) : status;

static void extractXYZ(u8_t *regValue, float &x, float &y, float &z, float sensitivity);

static void extractXYZ(u8_t *regValue, float &x, float &y, float &z, float sensitivity) {
	SensorAxes_t acceleration;
	int16_t dataRaw[3];
	dataRaw[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
	dataRaw[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
	dataRaw[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

	acceleration.AXIS_X = ( int32_t )( dataRaw[0] * sensitivity );
	acceleration.AXIS_Y = ( int32_t )( dataRaw[1] * sensitivity );
	acceleration.AXIS_Z = ( int32_t )( dataRaw[2] * sensitivity );

	x = dataRaw[0] * sensitivity;
	y = dataRaw[1] * sensitivity;
	z = dataRaw[2] * sensitivity;
}

/**
 * Copied from LSM6DS0_ACC_GYRO_driver_HL.c (method: LSM6DS0_X_Get_Sensitivity). Make sure this is in sync with method.
 */
status_t AttitudeUtils::ResetAccSensitivity(void *handle) {
	assert_param(handle != NULL);
	LSM6DS0_ACC_GYRO_FS_XL_t fullScale;

	/* Read actual full scale selection from sensor. */
	if ( LSM6DS0_ACC_GYRO_R_AccelerometerFullScale(handle, &fullScale ) == MEMS_ERROR )	{
		return MEMS_ERROR;
	}

	/* Store the sensitivity based on actual full scale. */
	switch( fullScale )	{
	case LSM6DS0_ACC_GYRO_FS_XL_2g:
		AttitudeUtils::accSensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_2G;
		break;
	case LSM6DS0_ACC_GYRO_FS_XL_4g:
		AttitudeUtils::accSensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_4G;
		break;
	case LSM6DS0_ACC_GYRO_FS_XL_8g:
		AttitudeUtils::accSensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_8G;
		break;
	case LSM6DS0_ACC_GYRO_FS_XL_16g:
		AttitudeUtils::accSensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_16G;
		break;
	default:
		AttitudeUtils::accSensitivity = -1.0f;
		return MEMS_ERROR;
	}

	return MEMS_SUCCESS;
}

/**
 * Copied from LSM6DS0_ACC_GYRO_driver_HL.c (method: LSM6DS0_G_Get_Sensitivity). Make sure this is in sync with method.
 */
status_t AttitudeUtils::ResetGyroSensitivity( void *handle ) {
	assert_param(handle != NULL);
	LSM6DS0_ACC_GYRO_FS_G_t fullScale;

	/* Read actual full scale selection from sensor. */
	if ( LSM6DS0_ACC_GYRO_R_GyroFullScale(handle, &fullScale ) == MEMS_ERROR ){
		return MEMS_ERROR;
	}

	/* Store the sensitivity based on actual full scale. */
	switch( fullScale )	{
	case LSM6DS0_ACC_GYRO_FS_G_245dps:
		AttitudeUtils::gyroSensitivity = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_245DPS;
		break;
	case LSM6DS0_ACC_GYRO_FS_G_500dps:
		AttitudeUtils::gyroSensitivity = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_500DPS;
		break;
	case LSM6DS0_ACC_GYRO_FS_G_2000dps:
		AttitudeUtils::gyroSensitivity = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_2000DPS;
		break;
	default:
		AttitudeUtils::gyroSensitivity = -1.0f;
		return MEMS_ERROR;
	}

	return MEMS_SUCCESS;
}

status_t AttitudeUtils::Initialize(DrvStatusTypeDef &result, void **hhandle) {

	/*Configure GPIO pin : PB5 for interrupts from LSM6DS0 */
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	cycleCounterAvg = 0;
	cycleCounterCount = 0;
	static uint8_t isInitialized = 0;

	QF_CRIT_ENTRY(0);

	volatile status_t status = MEMS_SUCCESS;

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
		QF_CRIT_EXIT(0);
		return Initialize(result, hhandle);
	}

	// CTRL_REG1_G Register
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroDataRate(handle, LSM6DS0_ACC_GYRO_ODR_G_952Hz));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroBandwidthSelection(handle, LSM6DS0_ACC_GYRO_BW_G_ULTRA_HIGH));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroFullScale(handle, LSM6DS0_ACC_GYRO_FS_G_2000dps));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AutoIndexOnMultiAccess(handle, LSM6DS0_ACC_GYRO_IF_ADD_INC_ENABLE));

	// CTRL_REG4 Register
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerDataRate(handle, LSM6DS0_ACC_GYRO_ODR_XL_952Hz));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerFullScale(handle, LSM6DS0_ACC_GYRO_FS_XL_2g));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerBandWitdthSelection(handle, LSM6DS0_ACC_GYRO_BW_SCAL_ODR_WITH_BW_XL));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerFilterBandwidth(handle, LSM6DS0_ACC_GYRO_BW_XL_105Hz));

	// CTRL_REG7_XL Register
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerAxisX(handle, LSM6DS0_ACC_GYRO_XEN_XL_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerAxisY(handle, LSM6DS0_ACC_GYRO_YEN_XL_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerAxisZ(handle, LSM6DS0_ACC_GYRO_ZEN_XL_ENABLE));

	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroAxisX(handle, LSM6DS0_ACC_GYRO_XEN_G_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroAxisY(handle, LSM6DS0_ACC_GYRO_YEN_G_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroAxisZ(handle, LSM6DS0_ACC_GYRO_ZEN_G_ENABLE));

	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerHighResolutionMode(handle, LSM6DS0_ACC_GYRO_HR_ENABLE));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerCutOff_filter(handle, LSM6DS0_ACC_GYRO_DCF_ODR_DIV_9));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerFilteredDataSelection(handle, LSM6DS0_ACC_GYRO_FDS_FILTER_ENABLE));

	STATUS_T_SET(status, ResetAccSensitivity(handle));
	STATUS_T_SET(status, ResetGyroSensitivity(handle));

	/////// 	INT_CTRL Register 	//////////
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_XL_DataReadyOnINT(handle, LSM6DS0_ACC_GYRO_INT_DRDY_XL_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GYRO_DataReadyOnINT(handle, LSM6DS0_ACC_GYRO_INT_DRDY_G_ENABLE));

	QF_CRIT_EXIT(0);

	return status;
}

status_t  AttitudeUtils::GetAttitude(Acceleration &acc, AngularRate &angRate, void *handle) {

	uint32_t marker = GetPerfCycle();

	u8_t value;

	LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, &value, 1);

	if ((value & LSM6DS0_ACC_GYRO_GDA_MASK) != LSM6DS0_ACC_GYRO_GDA_UP) {
		return MEMS_SUCCESS;
	}

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	HAL_StatusTypeDef status = HAL_OK;

	u8_t acc_8_t[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	// Read 12 registers at a time via I2C containing gyro and acc data.
	HAL_STATUS_SET(status, HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_G /* X Axis, low bit, gyro register */,
		I2C_MEMADD_SIZE_8BIT /* MemAddress Size */, acc_8_t /* Data */, 12 /* Data Size */, NUCLEO_I2C_EXPBD_TIMEOUT_MAX));
	extractXYZ(acc_8_t, angRate.x, angRate.y, angRate.z, gyroSensitivity/* mg/LSb */ * (1.0f/1000));
	extractXYZ(acc_8_t + 6, acc.x, acc.y, acc.z, accSensitivity/* mdps/LSb */ * (1.0f/1000) * ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SEC_SQ);

/*
	u8_t acc_8_t[6] = {0, 0, 0, 0, 0, 0};

	HAL_STATUS_SET(status, HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_XL, I2C_MEMADD_SIZE_8BIT, acc_8_t, 6, NUCLEO_I2C_EXPBD_TIMEOUT_MAX));
	extractXYZ(acc_8_t, acc.x, acc.y, acc.z, accSensitivity);

	HAL_STATUS_SET(status, HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_G, I2C_MEMADD_SIZE_8BIT, acc_8_t, 6, NUCLEO_I2C_EXPBD_TIMEOUT_MAX));
	extractXYZ(acc_8_t, angRate.x, angRate.y, angRate.z, gyroSensitivity);
*/

	counter++;
	uint32_t cycleCounter = GetElapsedCycles(marker);
	cycleCounterAvg = (cycleCounterAvg * ((float)cycleCounterCount)/(cycleCounterCount + 1));
	cycleCounterAvg += ((float)cycleCounter)/(cycleCounterCount + 1);
	cycleCounterCount++;

	return (status == HAL_OK) ? MEMS_SUCCESS : MEMS_ERROR;
}
}// namespace
