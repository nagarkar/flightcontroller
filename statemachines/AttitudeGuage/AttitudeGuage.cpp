//****************************************************************************
// Model: NineAxisMeasurement.qm
// File:  AttitudeGuage/AttitudeGuage.cpp
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
//${AttitudeGuage::AttitudeGuage::AttitudeGuage.cpp} .........................
#include "AttitudeGuage.h"

#include "x_nucleo_iks01a1_accelero.h"
#include "active_config.h"
#include "app_ao_config.h"
#include "active_log.h"

using namespace StdEvents;
using namespace QP;

extern "C" {

DMA_HandleTypeDef hdma_rx;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin != GPIO_PIN_5) {
        return;
    }
    QF::PUBLISH(new Evt(ATTITUDE_DATA_AVAILABLE_SIG), NULL);
}

void DMA1_Stream0_IRQHandler(void) {
    QP_QXK_ISR_ENTRY();
    HAL_DMA_IRQHandler(&hdma_rx);
    QP_QXK_ISR_EXIT();
}

void I2C1_ER_IRQHandler(void) {
    HAL_I2C_ER_IRQHandler(GetI2CHandle());
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    QF::PUBLISH(new Evt(ATTITUDE_GYRO_DMA_COMPLETE_SIG), NULL);
}

void BSP_I2C1_MspInit(I2C_HANDLE_TYPE_DEF* i2cHandle) {

    /// I2C Busy Flag Stuck Problem (I2C keeps timing out because the busy flag is always set)
    //  Mentioned here: https://goo.gl/mQdMUu
    //  Once this code runs, the busy flag will be reset.
    //  Generally speaking, you can leave this in after the issue is fixed.
    BSP_I2C_ClearBusyFlagErrata_2_14_7(i2cHandle, NUCLEO_I2C_EXPBD_SDA_PIN, NUCLEO_I2C_EXPBD_SCL_PIN);

    //1- Enable peripherals and GPIO Clocks #################################
    __HAL_RCC_DMA1_CLK_ENABLE();

    //3-DMA configuration
    hdma_rx.Instance                 = DMA1_Stream0;
    hdma_rx.Init.Channel             = DMA_CHANNEL_1;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_NORMAL;
    hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_rx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_rx);

    //4-link DMA with I2C
    __HAL_LINKDMA(GetI2CHandle(), hdmarx, hdma_rx);

    //5-configure the DMA interrupt
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

}// Extern C

namespace Attitude {


#if ((QP_VERSION < 580) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8)))
#error qpcpp version 5.8.0 or higher required
#endif

//${AttitudeGuage::AttitudeGuage} ............................................
//${AttitudeGuage::AttitudeGuage::AttitudeGuage} .............................
AttitudeGuage::AttitudeGuage()
  : AO(ATTITUDE_GUAGE_INTERVAL_TIMER_SIG,
        (QStateHandler)&initial,
        "ATTITUDE_GUAGE")
    , m_gyroRate(0)
    , m_measurements(0)
    , m_previousMeasurementCount(0)
    , m_acc_handle(NULL)
    , m_mag_handle(NULL)
	, m_bar_handle(NULL)
    , m_acc_gyro_data({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})
	, m_attitude({1.0f, 0.0f, 0.0f, 0.0f})
	, m_altitude(0.0f)
	, m_faltitude(0.0f)
	, m_temp(23)
{}

//${AttitudeGuage::AttitudeGuage::Init} ......................................
status_t AttitudeGuage::Init() {
    static int initializationAttempts = 0;
    volatile status_t status;
    DrvStatusTypeDef result = COMPONENT_OK;
    status = AttitudeUtils::Initialize(result, &m_acc_handle, &m_mag_handle, &m_bar_handle);
    initializationAttempts++;
    if (initializationAttempts > MAX_INIT_ATTEMPTS_BEFORE_RESET) {
        BSP_SystemResetOrLoop();
    }
    if (status == MEMS_ERROR) {
        PRINT("ERROR in AttitudeGuage.Init()\r\n");
        ErrorEvt *evt = new ErrorEvt(ATTITUDE_GUAGE_FAILED_SIG, 0, ERROR_HARDWARE);
        postLIFO(evt);
    }
    return status;
}
//${AttitudeGuage::AttitudeGuage::UpdateGyroRate} ............................
void AttitudeGuage::UpdateGyroRate() {
    static uint32_t numDelays = 0;
    numDelays++;
    int totalMillis = numDelays * CHECK_UP_INTERVAL;
    // TODO: Deal with overflow
    m_gyroRate = (1000.0 * m_measurements)/totalMillis; // Hz
}
//${AttitudeGuage::AttitudeGuage::GotNewMeasurements} ........................
bool AttitudeGuage::GotNewMeasurements() {
    if (m_previousMeasurementCount == m_measurements) {
        return false;
    } else {
        m_previousMeasurementCount = m_measurements;
        return true;
    }
}
//${AttitudeGuage::AttitudeGuage::ProcessAttitude} ...........................
status_t AttitudeGuage::ProcessAttitude() {
    //status_t status = AttitudeUtils::GetAttitude(
    //    linearAcc, angularRate, field, m_acc_handle, m_mag_handle);
    status_t status = AttitudeUtils::GetAttitude2(m_acc_gyro_data, m_acc, m_angularRate, m_field, m_altitude, m_faltitude, m_temp, &m_attitude);

    if (status == MEMS_ERROR) {
        postLIFO(new Evt(ATTITUDE_GUAGE_FAILED_SIG));
    } else {
        m_measurements++;
        Evt *evt = new AttitudeDataEvt(m_acc, m_angularRate, m_field);
        QF::PUBLISH(evt, this);
    }
    return status;
}
//${AttitudeGuage::AttitudeGuage::Start} .....................................
uint8_t AttitudeGuage::Start(uint8_t prio) {
    return AO::Start(prio);
}
//${AttitudeGuage::AttitudeGuage::StartDMATransfer} ..........................
status_t AttitudeGuage::StartDMATransfer() {
    return AttitudeUtils::StartDMATransfer(m_field, m_altitude, m_temp, m_acc_handle, m_mag_handle, m_bar_handle, m_acc_gyro_data, 12);
}
//${AttitudeGuage::AttitudeGuage::SM} ........................................
QP::QState AttitudeGuage::initial(AttitudeGuage * const me, QP::QEvt const * const e) {
    // ${AttitudeGuage::AttitudeGuage::SM::initial}
    me->subscribe(ATTITUDE_GUAGE_STOP_REQ_SIG);
    me->subscribe(ATTITUDE_GUAGE_START_REQ_SIG);
    me->subscribe(ATTITUDE_DATA_AVAILABLE_SIG);
    me->subscribe(ATTITUDE_GUAGE_FAILED_SIG);
    me->subscribe(ATTITUDE_GYRO_DMA_COMPLETE_SIG);
    return Q_TRAN(&Root);
}
//${AttitudeGuage::AttitudeGuage::SM::Root} ..................................
QP::QState AttitudeGuage::Root(AttitudeGuage * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${AttitudeGuage::AttitudeGuage::SM::Root::initial}
        case Q_INIT_SIG: {
            status_ = Q_TRAN(&Stopped);
            break;
        }
        default: {
            status_ = Q_SUPER(&top);
            break;
        }
    }
    return status_;
}
//${AttitudeGuage::AttitudeGuage::SM::Root::Stopped} .........................
QP::QState AttitudeGuage::Stopped(AttitudeGuage * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Stopped}
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            status_ = Q_HANDLED();
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Stopped::ATTITUDE_GUAGE_START_REQ}
        case ATTITUDE_GUAGE_START_REQ_SIG: {
            LOG_EVENT(e);
            me->PublishConfirmation(
                EVT_CAST(*e),
                ATTITUDE_GUAGE_START_CFM_SIG);
            status_ = Q_TRAN(&Started);
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Stopped::ATTITUDE_GUAGE_STOP_REQ}
        case ATTITUDE_GUAGE_STOP_REQ_SIG: {
            LOG_EVENT(e);
            me->PublishConfirmation(
                EVT_CAST(*e),
                ATTITUDE_GUAGE_STOP_CFM_SIG);
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Root);
            break;
        }
    }
    return status_;
}
//${AttitudeGuage::AttitudeGuage::SM::Root::Started} .........................
QP::QState AttitudeGuage::Started(AttitudeGuage * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Started}
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            me->m_timer.armX(CHECK_UP_INTERVAL, CHECK_UP_INTERVAL);
            status_ = Q_HANDLED();
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Started}
        case Q_EXIT_SIG: {
            me->m_timer.disarm();
            status_ = Q_HANDLED();
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Started::ATTITUDE_GUAGE_STOP_REQ}
        case ATTITUDE_GUAGE_STOP_REQ_SIG: {
            LOG_EVENT(e);
            me->PublishConfirmation(
                EVT_CAST(*e),
                ATTITUDE_GUAGE_STOP_CFM_SIG);
            status_ = Q_TRAN(&Stopped);
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Started::ATTITUDE_GUAGE_START_REQ}
        case ATTITUDE_GUAGE_START_REQ_SIG: {
            LOG_EVENT(e);
            me->PublishConfirmation(
                EVT_CAST(*e),
                ATTITUDE_GUAGE_START_CFM_SIG);
            status_ = Q_HANDLED();
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Started::ATTITUDE_GUAGE_INTERVAL_TIMER}
        case ATTITUDE_GUAGE_INTERVAL_TIMER_SIG: {
            //LOG_EVENT(e);
            if(!me->GotNewMeasurements()) {
                me->postLIFO(new Evt(ATTITUDE_GUAGE_FAILED_SIG));
            } else {
                me->UpdateGyroRate();
                PRINT("GYRO Rate: %f\r\n", me->m_gyroRate);
            }
            status_ = Q_HANDLED();
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Started::ATTITUDE_GUAGE_FAILED}
        case ATTITUDE_GUAGE_FAILED_SIG: {
            status_ = Q_TRAN(&Failed);
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Started::ATTITUDE_DATA_AVAILABLE}
        case ATTITUDE_DATA_AVAILABLE_SIG: {
            me->StartDMATransfer();
            status_ = Q_HANDLED();
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Started::ATTITUDE_GYRO_DMA_COMPLETE}
        case ATTITUDE_GYRO_DMA_COMPLETE_SIG: {
            me->ProcessAttitude();
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Root);
            break;
        }
    }
    return status_;
}
//${AttitudeGuage::AttitudeGuage::SM::Root::Failed} ..........................
QP::QState AttitudeGuage::Failed(AttitudeGuage * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Failed}
        case Q_ENTRY_SIG: {
            LOG_EVENT(e);
            uint8_t retries = 0;
            status_t status = MEMS_ERROR;
            while(retries < MAX_RETRIES && status == MEMS_ERROR) {
                retries++;
                // QF_CRIT_ENTRY(0);
                status = me->Init();
                // QF_CRIT_EXIT(0);
                if (status == MEMS_SUCCESS) {
                    me->postLIFO(new Evt(ATTITUDE_DATA_AVAILABLE_SIG));
                    break;
                }
            }
            if (status == MEMS_ERROR && retries == MAX_RETRIES) {
                PRINT("Exhaused Retries\r\n");
            }
            status_ = Q_HANDLED();
            break;
        }
        // ${AttitudeGuage::AttitudeGuage::SM::Root::Failed::ATTITUDE_DATA_AVAILABLE}
        case ATTITUDE_DATA_AVAILABLE_SIG: {
            me->StartDMATransfer();
            status_ = Q_TRAN(&Started);
            break;
        }
        default: {
            status_ = Q_SUPER(&Root);
            break;
        }
    }
    return status_;
}
//${AttitudeGuage::AttitudeDataEvt} ..........................................
//${AttitudeGuage::AttitudeDataEvt::AttitudeDataEvt} .........................
AttitudeDataEvt::AttitudeDataEvt(
    Acceleration acc,
    AngularRate angularRate,
    MagneticField field)
 : Evt(ATTITUDE_CHANGED_SIG)
    , m_acc(acc)
    , m_angularRate(angularRate)
    , m_field(field)
{}

} // namespace Attitude
