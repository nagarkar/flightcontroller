/*
 * DeviceAO.cpp
 *
 *  Created on: Aug 4, 2017
 *      Author: chinm_000
 */

#include <DeviceAO.h>

namespace RAD {

DeviceAO::DeviceAO(const char * name)
	: ComponentImpl(NULL, name) {

}

DeviceAO::~DeviceAO() {
	// TODO Auto-generated destructor stub
}

void DeviceAO::ProvisionComponent(ProvisionComponentRequest &req) {
	ComponentAO * sender = (ComponentAO  *) req.self;
	ProvisionComponentResponse resp = m_resManager.provisionComponent(req);
	sender->POST(new ProvisionComponentResponseEvt(resp), this);
}

void DeviceAO::ReserveResource(ResourceRequest &req) {
	ID senderID = req.header.sender.componentId;
	ComponentAO * sender = m_resManager.provisionedComponents[senderID];

	ResourceResponse resp;
	if (sender == NULL) {
		resp.errorCode = RAD_FAILURE;
		resp.errorReason = COMPONENT_NOT_INITIALIZED;
	} else {
		resp = m_resManager.provisionResource(req);
	}
	ResourceResponseEvt *evt = new ResourceResponseEvt(resp);
	sender->POST(evt, this);
}

DriverQueryResponse DeviceAO::GetDriverQueryResponse(DriverQueryRequest &req) {
	ID senderID = req.header.sender.componentId;
	ComponentAO * sender = m_resManager.provisionedComponents[senderID];
	Q_ASSERT(sender != NULL);
	DriverQueryResponse resp = m_resManager.queryDriver(req);
	sender->POST(new DriverQueryResponseEvt(resp), this);

}
} /* namespace RAD */
