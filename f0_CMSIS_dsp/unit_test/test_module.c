/*
 * test_module.c
 *
 *  Created on: Jul 3, 2017
 *      Author: LOMAS
 */

#include "unity.h"
#include "controller.h"

#define 	DELTA	 0.000005F

resultFM testResult;

void setup() {
	printf("Test Setup...!\r\n");
}

void tearDown() {
	printf("Test tearDown...!\r\n");
}

void testControllerF(float32_t val) {

	TEST_ASSERT_FLOAT_WITHIN(2, val, testResult.F);
}

void testControllerM0(void) {

	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.00005753727696195064, testResult.M[0]);
}

void testControllerM1(void) {

	TEST_ASSERT_FLOAT_WITHIN(DELTA, -0.00004572033501721272, testResult.M[1]);
}

void testControllerM2(void) {

	TEST_ASSERT_FLOAT_WITHIN(DELTA, 0.0000008442808205112886, testResult.M[2]);
}

