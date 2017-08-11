//****************************************************************************
// Model: NineAxisMeasurement.qm
// File:  AttitudeGuage/AttitudeUtils.h
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
//${AttitudeGuage::AttitudeGuage::AttitudeUtils.h} ...........................
#ifndef _AO_ATTITUDE_UTILS_H
#define _AO_ATTITUDE_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "Attitude.h"
#include "component.h"
#include "LSM6DS0_ACC_GYRO_driver.h"

#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"

#include "AltitudeEstimator.h"

namespace Attitude {
class AttitudeUtils {
private:
    static int cycleCounterCount;
    static float cycleCounterAvg;
    static int counter;
    static float accSensitivity;
    static float gyroSensitivity;
    static float magSensitivity;
    static AltitudeEstimator altitudeEstimator;
    static Q_cxyz q;
    static AngularPos orientation;
    static constexpr float ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SEC_SQ = 9.8f;
    static HAL_StatusTypeDef ReadMagneticField(void *magHandle, MagneticField &magField);
    static HAL_StatusTypeDef ReadAltitude(void *barHandle, Altitude &altitude, Temperature &temp);
public:
    static status_t Initialize(DrvStatusTypeDef & result, void **hhandle, void **magHandle, void **barHandle);
    static status_t isGyroDataReady(void *handle);
    static status_t GetAttitude2(u8_t* gyro_acc_data, Acceleration &acc, AngularRate &angRate, MagneticField magField, Altitude altitude, Altitude &faltitude, Temperature temp, Q_cxyz *attitude);
    static Altitude & KalmanAltitudeEstimate(Acceleration &acc, Q_cxyz &attitude, Altitude &altitude);
    static status_t StartDMATransfer(MagneticField &magField, Altitude &altitude, Temperature &temperature, void *handle,  void *magHandle, void *barHandle, u8_t *buff, uint32_t length);
    static status_t ResetAccSensitivity( void *handle );
    static status_t ResetGyroSensitivity( void *handle );
    static status_t ResetMagSensitivity( void *handle );
};

}// namespace

#ifdef __cplusplus
}
#endif

#endif // _AO_ATTITUDE_UTILS_H
