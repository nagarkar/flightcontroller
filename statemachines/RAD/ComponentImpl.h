/*
 * ComponentImpl.h
 *
 *  Created on: Aug 3, 2017
 *      Author: chinm_000
 */

#ifndef COMPONENTIMPL_H_
#define COMPONENTIMPL_H_

#include <ComponentAO.h>

Q_DEFINE_THIS_FILE

using namespace QP;
using namespace StdEvents;

namespace RAD {

class ComponentImpl: public ComponentAO {
private:
	enum {EVT_QUEUE_COUNT = 16, DEFER_QUEUE_COUNT = 4, INITIALIZATION_TIMEOUT = 100, ACTIVITY_TIMEOUT = 100};
	QEvt const * m_evtQueueStor[EVT_QUEUE_COUNT];
    QEvt const * m_deferQueueStor[DEFER_QUEUE_COUNT];
    QEQueue m_deferQueue;
protected:
    uint32_t m_sequenceNumber;
	ID 		m_officialID;
	QTimeEvt m_initializationTimer;
	QTimeEvt m_activityTimer;
	bool 	m_allResourcesGranted;
public:

	ComponentImpl(ComponentAO * parent, const char * name);
	virtual ~ComponentImpl();
	virtual void Start(uint8_t prio);
    virtual void ArmActivityTimeout();
    virtual void ArmInitializationTimeout();
    virtual void DisarmActivityTimeout();
    virtual void DisarmInitializationTimeout();
    virtual void OnInitializationTimeout();
    virtual void OnActivityTimeout();
    virtual void RequestResources();
    virtual bool AllResourcesGranted();
    virtual void OnServiceRequest(RequestData const *  reques);
    virtual void OnServiceResponse(ResponseData const *  response);
    virtual void OnInterrupt(InterruptData const *  interrupt);
    virtual void OnInitial();
    virtual void OnCreate();
    virtual void OnStart();
    virtual void OnRestart();
    virtual void OnPause();
    virtual void OnStopped();
    virtual void OnError();
    virtual bool ErrorOccurred();
    virtual bool HasSufficientActivityOccurred();
    virtual void ConfirmComponentStart();
    virtual void OnResourceGrant(ResourceResponse & resp);
    virtual ID getOfficialId();
    virtual QEvt const * * GetEventQueueStore(uint32_t &size);
    virtual QEvt const * * GetDeferredQueueStore(uint32_t &size);
    virtual void RequestComponentId();
    virtual void OnProvisionComponentResponse(ProvisionComponentResponse & resp) ;

    virtual ResourceRequest GetRequiredResources() {
    	Q_onAssert("Unsupported Method", 0);
    }
};

class RADEvt: public QEvt {
public:
	RADEvt(Signal sig)
		: QEvt(sig) { }

	~RADEvt() {  }
    static void * operator new(size_t evtSize);
    static void operator delete(void * evt);
};

class StartReqEvt: public RADEvt {
public:
	RequestData m_data;
	StartReqEvt(RequestData req)
		: RADEvt(START_SIG), m_data(req) { }
};

class StartCfmEvt: public RADEvt {
public:
	ResponseData m_data;
	StartCfmEvt(ResponseData req)
		: RADEvt(START_CFM_SIG), m_data(req) { }
};


class ProvisionComponentRequestEvt: public RADEvt {
public:
	ProvisionComponentRequest m_data;
	ProvisionComponentRequestEvt(ProvisionComponentRequest req)
		: RADEvt(PROVISION_COMPONENT_REQ_SIG), m_data(req) { }
};

class ProvisionComponentResponseEvt: public RADEvt {
public:
	ProvisionComponentResponse m_data;
	ProvisionComponentResponseEvt(ProvisionComponentResponse req)
		: RADEvt(PROVISION_COMPONENT_RESP_SIG), m_data(req) { }
};

class InterruptDataEvt: public RADEvt {
public:
	InterruptData m_data;
	InterruptDataEvt(InterruptData req)
		: RADEvt(INTERRUPT_SIG), m_data(req) { }
};

class DriverQueryRequestEvt: public RADEvt {
public:
	DriverQueryRequest m_data;
	DriverQueryRequestEvt(DriverQueryRequest req)
		: RADEvt(DRIVER_REQ_SIG), m_data(req) { }
};

class DriverQueryResponseEvt: public RADEvt {
public:
	DriverQueryResponse m_data;
	DriverQueryResponseEvt(DriverQueryResponse resp)
		: RADEvt(DRIVER_RESPONSE_SIG), m_data(resp) { }
};


class ResourceRequestEvt: public RADEvt {
public:
	ResourceRequest m_data;
	ResourceRequestEvt(ResourceRequest req)
		: RADEvt(RESOURCE_REQ_SIG), m_data(req) { }
};


class ResourceResponseEvt: public RADEvt {
public:
	ResourceResponse m_data;
	ResourceResponseEvt(ResourceResponse req)
		: RADEvt(RESOURCE_GRANT_SIG), m_data(req) { }
};


class ServiceRequestEvt: public RADEvt {
public:
	ServiceRequestData m_data;
	ServiceRequestEvt(ServiceRequestData req)
		: RADEvt(SERVICE_REQ_SIG), m_data(req) { }
};

class ServiceResponseEvt: public RADEvt {
public:
	ServiceResponseData m_data;
	ServiceResponseEvt(ServiceResponseData req)
		: RADEvt(SERVICE_RESP_SIG), m_data(req) { }
};

/*
class ResponseEvt: public RADEvt {
private:
	ResponseData m_data;
public:
	ResponseEvt(Signal sig, ResponseData data)
		: RADEvt(sig)
		, m_data(data) { }
};

class ResourceRequestEvt: public RADEvt {
private:
	ResourceRequest m_data;
public:
	ResourceRequestEvt(ResourceRequest data)
		: RADEvt(RESOURCE_REQ_SIG)
		, m_data(data) { }
};
*/
} /* namespace RAD */


#endif /* COMPONENTIMPL_H_ */
