/*
 * RadMessages.cpp
 *
 *  Created on: Aug 2, 2017
 *      Author: chinm_000
 */
/*
typedef uint8_t 	InterruptNumber;
typedef uint8_t 	ShortID;
typedef uint16_t 	ID;
typedef uint32_t 	LongID;

struct CompositeID {
	ID deviceID;
	ID componentId;
};

struct MessageData {}

struct BroadcastData:MessageData {}

struct RequestData:MessageData {
	LongID sequenceNumber;
}

struct StreamData:MessageData {
	ShortID streamID;
	LongID elementCounter;
};

struct ResponseData:RequestData {
	ShortID errorCode;
	ShortID errorReason;
}

struct MessageHeader {
	enum_t 			messageType;
	uint8_t  			messageSubType;
	CompositeID 	sender;
	CompositeID 	receiver;
	MessageData 	&data;
};

enum StandardSignals {
	UNKNOWN,
	COMPONENT_ID_REQ,
	COMPONENT_ID_RESP,
	COMPONENT_START_REQ,
	COMPONENT_START_CFM,
	RESERVE_RES_REQ,
	RESERVE_RES_RESP,
	I2C_CONFIG_REQ,
	UART_CONFIG_REQ,
	DMA_CONFIG_REQ,
	GPIO_CONFIG_REQ,
	SPI_CONFIG_REQ,
	TIMER_CONFIG_REQ,
	CONFIG_RESP,
	INTERRUPT_EVT,
	INFORM_DRIVER_READY,
	DRIVER_READY,
};

enum ResourceAndChannel {
	UNKNOWN,
	I2C1,
	I2C2,
	I2C3,
	DMAStream0Channel1,
	DMAStream0Channel2,
	USART1,
	USART2,
	USART3,
	SPI1,
	GPIOA_PIN5,
	GPIOB_PIN5,
	GPIOB_PIN6,
	GPIOB_PIN7,
	GPIOB_PIN8,
	GPIOB_PIN9,
};

enum ReservationType {
	UNKNOWN,
	DRIVER,
	EXCLUSIVE
};

#define MSG_SUBTYPE_POINT_TO_POINT 	0x01
#define MSG_SUBTYPE_BROADCAST			 	MSG_SUBTYPE_POINT_TO_POINT << 1
#define MSG_SUBTYPE_REQUEST				 	MSG_SUBTYPE_POINT_TO_POINT << 2
#define MSG_SUBTYPE_RESPONSE			 	MSG_SUBTYPE_POINT_TO_POINT << 3
#define MSG_SUBTYPE_STREAM				 	MSG_SUBTYPE_POINT_TO_POINT << 4


*/
