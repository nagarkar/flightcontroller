/*
 * coefnts.h
 *
 *  Created on: Jun 26, 2017
 *      Author: LOMAS
 */

#ifndef COEFNTS_H_
#define COEFNTS_H_

#include "arm_math.h"


//struct coeffProperties {
//	float m_T;
//	float m_S;
//	float m_waypoints;
//	float m_ncoeff;
//	float m_subscripts;
//	float m_velocity;
//	float m_lhs_v;
//	float m_rhs_v;
//	int eqn_index;
//	float m_all_coeffs;
//	float m_all_paths;
//	float t;
//};

void initCoeff(float arrayWayPointRaw, uint8_t matWayPointRows, uint8_t matWayPointColumns);

#endif /* COEFNTS_H_ */
