/*
 * ResourceManager.h
 *
 *  Created on: Aug 6, 2017
 *      Author: chinm_000
 */

#ifndef RESOURCEMANAGER_H_
#define RESOURCEMANAGER_H_

#include <RAD.h>
#include "ComponentAO.h"

namespace RAD {

class ResourceManager {
	friend class DeviceAO;
private:
	enum {MAX_RESOURCES = 255, MAX_COMPONENTS=255, MIN_COMPONENT_SEQ_NUMBER=1};
	ComponentAO * provisionedComponents[MAX_COMPONENTS] = {NULL};
	ShortID componentSequenceNumber  = MIN_COMPONENT_SEQ_NUMBER;
	ComponentAO * resourceReservations[MAX_RESOURCES] = {NULL};
public:
	ResourceManager();
	virtual ~ResourceManager();
	ProvisionComponentResponse provisionComponent(ProvisionComponentRequest &request);
	ResourceResponse  provisionResource(ResourceRequest &req);
	DriverQueryResponse queryDriver(DriverQueryRequest req);
};

} /* namespace RAD */

#endif /* RESOURCEMANAGER_H_ */
