/*
 * I2CDriver.h
 *
 *  Created on: Aug 6, 2017
 *      Author: chinm_000
 */

#ifndef I2CDRIVER_H_
#define I2CDRIVER_H_

#include "ComponentImpl.h"

namespace RAD {
class I2CDriver: public ComponentImpl {
private:
	enum {N_RESOURCES = 3};
	ResourceRequestDataEl resources[N_RESOURCES] = {{RAD_I2C1}, {RAD_I2C2}, {RAD_I2C3}};
public:
	I2CDriver(ComponentAO * parent);
	virtual ~I2CDriver();
	virtual ResourceRequest GetRequiredResources();
};
} // namespace

#endif /* I2CDRIVER_H_ */
