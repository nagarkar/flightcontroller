/*
 * ResourceManager.cpp
 *
 *  Created on: Aug 6, 2017
 *      Author: chinm_000
 */

#include <ResourceManager.h>

namespace RAD {

Q_DEFINE_THIS_MODULE("ResourceManager");

ResourceManager::ResourceManager() {
	// TODO Auto-generated constructor stub
}

ResourceManager::~ResourceManager() {
	// TODO Auto-generated destructor stub
}

ProvisionComponentResponse ResourceManager::provisionComponent(
		ProvisionComponentRequest& req) {

	ID senderID = req.header.sender.componentId;
	ProvisionComponentResponse resp;
	resp.errorCode = RAD_SUCCESS; resp.errorReason = NO_REASON; resp.sequenceNumber = req.sequenceNumber; resp.id = senderID;
	ComponentAO * sender = (ComponentAO *) req.self; // TODO: Potentially unsafe if req is not initialized properly.

	Q_ASSERT(sender != NULL);
	if (senderID > 0) {
		ComponentAO * prevSender = provisionedComponents[senderID];
		if (prevSender == sender) {
			return resp;
		}
	}
	resp.id = componentSequenceNumber++;
	provisionedComponents[resp.id] = sender;
	return resp;
}

ResourceResponse ResourceManager::provisionResource(ResourceRequest& req) {
	ResourceResponse resp;
	resp.sequenceNumber = req.sequenceNumber;
	for(int i = 0; i < req.nResources; i++) {
		ResourceRequestDataEl el = req.resources[i];
		ResourceAndChannel idx = el.resourceType;
		Q_ASSERT(idx <= MAX_RESOURCES);
		if (ResourceManager::resourceReservations[idx] != NULL) {
			resp.errorCode = RAD_FAILURE;
			resp.errorReason = RESOURCE_ALREADY_RESERVED;
			return resp;
		}
		Q_ASSERT(provisionedComponents[req.header.sender.componentId] != NULL);
		ResourceManager::resourceReservations[idx] = provisionedComponents[req.header.sender.componentId];
	}
	resp.errorCode = RAD_SUCCESS;
	return resp;
}

DriverQueryResponse ResourceManager::queryDriver(DriverQueryRequest req) {
	DriverQueryResponse resp;
	resp.driverComponentID = 0;
	resp.sequenceNumber =  req.sequenceNumber;
	ResourceAndChannel idx = req.resource.resourceType;
	if (resourceReservations[idx] == NULL) {
		resp.errorCode = RAD_FAILURE;
		resp.errorReason = NO_DRIVER_REGISTERED;
	} else {
		resp.driverComponentID = resourceReservations[idx]->getOfficialId();
	}
	return resp;
}

} /* namespace RAD */
