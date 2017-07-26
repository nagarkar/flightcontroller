/*
 * controller.h
 *
 *  Created on: Jun 26, 2017
 *      Author: LOMAS
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "arm_math.h"

#ifndef DIM
	#define DIM	3
#endif

#define 	I		0
#define 	J		1
#define 	K		2

//#define 	KD1		1000
//#define 	KD2		1000
//#define 	KD3		1000
//
//#define 	KP1		600
//#define 	KP2		600
//#define 	KP3		600

#define 	ZERO 	0.0F


typedef struct structureState {
	float32_t pos[DIM];
	float32_t vel[DIM];
	float32_t rot[DIM];
	float32_t omega[DIM];
} stateStruct;

typedef struct structureDestState {
	float32_t pos[DIM];
	float32_t vel[DIM];
	float32_t acc[DIM];
	float32_t yaw;
	float32_t yawdot;
} destStateStruct;


typedef struct structureParams{
	float32_t gravity;
	float32_t mass;
} robotParamsStruct;

typedef struct structResultFM{
	float32_t F;
	float32_t M[DIM];
}resultFM;

resultFM controller(float32_t, stateStruct *, destStateStruct *, robotParamsStruct *);

#endif /* CONTROLLER_H_ */
