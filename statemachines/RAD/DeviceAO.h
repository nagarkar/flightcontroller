/*
 * DeviceAO.h
 *
 *  Created on: Aug 4, 2017
 *      Author: chinm_000
 */

#ifndef DEVICEAO_H_
#define DEVICEAO_H_

#include <ComponentImpl.h>
#include <ResourceManager.h>

namespace RAD {

class DeviceAO: public ComponentImpl {
private:
	ResourceManager m_resManager;
public:
	DeviceAO(const char * name);
	virtual ~DeviceAO();
	virtual void ReserveResource(ResourceRequest &req);
	virtual void ProvisionComponent(ProvisionComponentRequest &req);
	virtual DriverQueryResponse GetDriverQueryResponse(DriverQueryRequest &req);
};

} /* namespace RAD */

#endif /* DEVICEAO_H_ */
