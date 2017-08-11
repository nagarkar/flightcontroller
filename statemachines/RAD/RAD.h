/*
 * RAD.h
 *
 *  Created on: Aug 3, 2017
 *      Author: chinm_000
 */

#ifndef RAD_H_
#define RAD_H_

#include <RAD.h>
#include "qpcpp.h"
#include "app_ao_config.h"
#include "active_events.h"
#include "active_enum.h"
#include "RadUtils.h"
#include "bsp.h"

namespace RAD {

typedef uint8_t 	InterruptNumber;
typedef enum {
	TX_COMPLETE,
	RX_COMPLETE,
	PIN_TRANSITION,
} InterruptType;

typedef uint8_t 	ShortID;
typedef uint16_t 	ID;
typedef ID ResourceID;
typedef uint32_t 	LongID;

typedef enum {
	UNKNOWN_COMPONENT_TYPE,
	DEVICE,
	SERVICE
} ComponentType;

typedef enum {
	UNKNOWN_RESOURCE,
	RAD_I2C1,
	RAD_I2C2,
	RAD_I2C3,
	RAD_DMAStream0Channel1,
	RAD_DMAStream0Channel2,
	RAD_USART1,
	RAD_USART2,
	RAD_USART3,
	RAD_SPI1,
	RAD_GPIOA_PIN5,
	RAD_GPIOB_PIN5,
	RAD_GPIOB_PIN6,
	RAD_GPIOB_PIN7,
	RAD_GPIOB_PIN8,
	RAD_GPIOB_PIN9,
} ResourceAndChannel;

typedef enum {
	UNKNOWN_RESERVATION,
	DRIVER,
	EXCLUSIVE,
	NONE
} ReservationType;


struct CompositeID {
	ID unusedDeviceId;
	ID componentId;
};

enum ErrorCodes {
	RAD_UNKNOWN,
	RAD_SUCCESS,
	RAD_FAILURE
};

enum ErrorReasons {
	NO_REASON,
	COMPONENT_NOT_INITIALIZED,
	RESOURCE_ALREADY_RESERVED,
	NO_DRIVER_REGISTERED
};


struct MessageHeader {
	CompositeID 	sender;
};

struct MessageData {
	MessageHeader header;
};

struct ResponseData:MessageData {
	LongID 			sequenceNumber;
	ShortID 			errorCode;
	ShortID 			errorReason;
	bool isSuccess() {
		return errorCode == RAD_SUCCESS;
	}
};

struct RequestData:MessageData {
	LongID 			sequenceNumber;
};

typedef enum_t CommandID;

struct ServiceRequestData:RequestData {
	CommandID 	commandId;
	void * 			commandParameters;
};

struct ServiceResponseData: ResponseData { };

struct StreamData:MessageData {
	ShortID 			streamID;
	LongID 			elementCounter;
};

struct ResourceRequestDataEl {
	ResourceAndChannel  	resourceType;
	//ReservationType				unused;
};

struct ResourceRequest: RequestData {
	const ResourceRequestDataEl *			resources;
	uint8_t 														nResources;
};

struct ResourceResponse: ResponseData {};


struct ProvisionComponentRequest: RequestData {
	QHsm  * 			self;
};

struct ProvisionComponentResponse: ResponseData {
	ID 						id;
};

struct DriverQueryRequest: RequestData {
	ResourceRequestDataEl 			resource;
};

struct DriverQueryResponse: ResponseData {
	ID						driverComponentID;
};


struct ResourceData {
	ResourceID 					id;
	ResourceAndChannel 	resourceChannel;
};

struct InterruptData {
	ResourceID 			resourceID;
	InterruptNumber 	num;
	InterruptType		type;
};

//#define SUBSCRIBE_MSG(sig)										do { \
//																							subscribe(sig); \
//																							QS_SIG_DICTIONARY(sig, this); \
//																						} while (0)

#define RAD_OBJ_DICTIONARY(obj_, name) do { \
    if (QS_GLB_FILTER_(QP::QS_OBJ_DICT)) { \
        QP::QS::obj_dict((obj_), name); \
    } \
} while (false)

/****************************** I2C COnfiguration ******************************/
typedef struct {
	I2C_TypeDef * instance; // = I2C1, I2C2 etc. See stm32f401xe.h
	GPIO_TypeDef * gpioInstance;
	I2C_HandleTypeDef i2cHandle;
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t i2cTimeout = 0x1000;

} RAD_I2C_Config;

struct I2CReadCommandParameters {
	I2CReadCommandParameter 		*parameters;
	int32_t &										nParameters;
};
struct I2CReadCommandParameter {
	RAD_I2C_Config &		i2cConfig;
	uint8_t 							devAddress;
	uint8_t 							memAddress;
	uint16_t 							memAddressSize;
	uint8_t 							* buffer;
	uint16_t 							bufferSize;
	bool 								withDMA;
};


} // namespace

#endif /* RAD_H_ */
