/*
 * ComponentImpl.cpp
 *
 *  Created on: Aug 3, 2017
 *      Author: chinm_000
 */

#include <ComponentImpl.h>

using namespace QP;

namespace RAD {


ComponentImpl::ComponentImpl(ComponentAO * parent, const char * name)
	: ComponentAO(parent, name)
	, m_allResourcesGranted(false)
    , m_initializationTimer(this, INITIALIZATION_TIMEOUT_SIG)
	, m_activityTimer(this, INITIALIZATION_TIMEOUT_SIG)
	, m_officialID(0)
	, m_sequenceNumber(0)
{
	//QS::obj_dict(this, "ComponentImpl");
}

static inline void SUBSCRIBE_MSG(ComponentImpl *me, enum_t sig) {
	me->subscribe(sig);
	if (QS_GLB_FILTER_(QP::QS_SIG_DICT)) {
		QP::QS::sig_dict(sig, me, getStringFromSym(Signal, sig));
	}
	//QS_SIG_DICTIONARY(sig, me);
}

void ComponentImpl::OnInitial() {

	static bool ComponentImplStaticInitializationCompleteFlag = false;
	HAL_Delay(10); // Delay to make sure the writes to UART succeed. There are lot of these done initially during SM initialization.
	RAD_OBJ_DICTIONARY(this, m_name);
	if (!ComponentImplStaticInitializationCompleteFlag) {
		ComponentImplStaticInitializationCompleteFlag = true;
		QS_FUN_DICTIONARY(&top);
		QS_FUN_DICTIONARY(&Zombie);
		QS_FUN_DICTIONARY(&initial);
		QS_FUN_DICTIONARY(&WaitingForInitialization);
		QS_FUN_DICTIONARY(&ServiceAcceptingRequests);
		QS_FUN_DICTIONARY(&DeviceAcceptingRequests);
		QS_FUN_DICTIONARY(&Error);
		QS_FUN_DICTIONARY(&Stopped);
		QS_FUN_DICTIONARY(&Paused);
		QS_FUN_DICTIONARY(&RequestingCompId);
		QS_FUN_DICTIONARY(&RequestingResources);
	}
	SUBSCRIBE_MSG(this, START_SIG);
	SUBSCRIBE_MSG(this, RESTART_SIG);
	SUBSCRIBE_MSG(this, RESUME_SIG);
	SUBSCRIBE_MSG(this, INITIALIZATION_TIMEOUT_SIG);
	SUBSCRIBE_MSG(this, ACTIVITY_TIMEOUT_SIG);
	SUBSCRIBE_MSG(this, START_CFM_SIG);
	SUBSCRIBE_MSG(this, PROVISION_COMPONENT_REQ_SIG);
	SUBSCRIBE_MSG(this, PROVISION_COMPONENT_RESP_SIG);
	SUBSCRIBE_MSG(this, RESOURCE_REQ_SIG);
	SUBSCRIBE_MSG(this, RESOURCE_GRANT_SIG);
	SUBSCRIBE_MSG(this, DRIVER_REQ_SIG);
	SUBSCRIBE_MSG(this, DRIVER_RESPONSE_SIG);
	SUBSCRIBE_MSG(this, DEVICE_ERROR_SIG);
	SUBSCRIBE_MSG(this, INTERRUPT_SIG);
	SUBSCRIBE_MSG(this, SERVICE_REQ_SIG);
	SUBSCRIBE_MSG(this, SERVICE_RESP_SIG);
}
void ComponentImpl::Start(uint8_t prio) {
	uint32_t size;
	QEvt const ** evtQueueStor = GetEventQueueStore(size);
	Q_ASSERT(size > 0);
	QActive::start(prio, evtQueueStor, size, NULL, 0);
	QEvt const ** deferredQueueStor = GetDeferredQueueStore(size);
	m_deferQueue.init(deferredQueueStor, size);
	OnInitial();
}

QEvt const * * ComponentImpl::GetEventQueueStore(uint32_t &size) {
	size = ARRAY_COUNT(m_evtQueueStor);
	return m_evtQueueStor;
}

QEvt const * * ComponentImpl::GetDeferredQueueStore(uint32_t &size) {
	size = ARRAY_COUNT(m_deferQueueStor);
	return m_deferQueueStor;
}

bool ComponentImpl::AllResourcesGranted() {
	return m_allResourcesGranted;
}

ComponentImpl::~ComponentImpl() {
	// TODO Auto-generated destructor stub
}


/**
 * On startup, components request resources. The concrete implementation must implement:
 *     ResourceTypeArray GetRequiredResources()
 */
void ComponentImpl::RequestResources() {
	ResourceRequest req = GetRequiredResources();
	req.header.sender.componentId = this->getOfficialId();
	const ResourceRequestEvt * const evt = new ResourceRequestEvt(req);
	m_parent->POST(evt, this);
}

void ComponentImpl::OnResourceGrant(ResourceResponse & resp) {
	m_allResourcesGranted = true;
}

ID ComponentImpl::getOfficialId() {
	return m_officialID;
}

void ComponentImpl::ArmInitializationTimeout() {
	m_initializationTimer.armX(INITIALIZATION_TIMEOUT, 0 /* One shot */);
}

void ComponentImpl::DisarmInitializationTimeout() {
	m_initializationTimer.disarm();
}

void ComponentImpl::OnInitializationTimeout() {
}

void ComponentImpl::ArmActivityTimeout() {
	m_activityTimer.armX(ACTIVITY_TIMEOUT, 0 /* One shot */);
}

void ComponentImpl::DisarmActivityTimeout() {
	m_activityTimer.disarm();
}

void ComponentImpl::OnActivityTimeout() {
}

void ComponentImpl::OnServiceRequest(RequestData const * response) {
}

void ComponentImpl::OnServiceResponse(ResponseData const * response) {
}

void ComponentImpl::OnInterrupt(InterruptData const * interrupt) {
}

void ComponentImpl::OnCreate() {
}

/**
 * Called when the START signal is received by the component.
 */
void ComponentImpl::OnStart() {
	//QP::QS::fun_dict(&QHsm::top, &fun_name_[0])

}

void ComponentImpl::RequestComponentId(){
	ProvisionComponentRequest req;
	req.header.sender.componentId = getOfficialId();
	req.self = this;
	req.sequenceNumber = 0;
	const ProvisionComponentRequestEvt * const evt = new ProvisionComponentRequestEvt(req);
	m_parent->POST(evt, this);
}

void ComponentImpl::OnProvisionComponentResponse(ProvisionComponentResponse & resp) {
	m_officialID = resp.id;
}
/**
 * Send the START_CFM event
 */
void ComponentImpl::ConfirmComponentStart() {
	ResponseData response;
	response.errorCode = RAD_SUCCESS;
	response.errorReason = NO_REASON;
	response.sequenceNumber = 0;
	response.header.sender.componentId = getOfficialId();
	const QEvt * const evt = new StartCfmEvt(response);
	postLIFO(evt);
}

void ComponentImpl::OnRestart() {
}

void ComponentImpl::OnPause() {
}

void ComponentImpl::OnStopped() {
}

void ComponentImpl::OnError() {
	(void)0;
}

bool ComponentImpl::ErrorOccurred() {
	return false;
}

bool ComponentImpl::HasSufficientActivityOccurred() {
	return true;
}

void * RADEvt::operator new(size_t evtSize) {
	return QF::newX_(evtSize, 0, 0);
}
static void RADEvt::operator delete(void * evt) {
	Q_onAssert("RADEvt:Delete operator not expected to be called", 0);
}


#define ASSERT_SIG(e, s) do { \
														if (e->sig !=  s) {	\
															while(1==1);	\
														}	\
												}while(0)

RequestData toRequest(QP::QEvt const *e) {
	ASSERT_SIG(e, SERVICE_REQ_SIG);
	ServiceRequestEvt  *radEvt = (ServiceRequestEvt *)e;
	return radEvt->m_data;
}

ResponseData toResponse(QP::QEvt const *e) {
	ASSERT_SIG(e, SERVICE_RESP_SIG);
	ServiceResponseEvt  *radEvt = (ServiceResponseEvt  *)e;
	return radEvt->m_data;
}

ResourceRequest toResourceRequest(QP::QEvt const *e) {
	ASSERT_SIG(e, RESOURCE_REQ_SIG);
	ResourceRequestEvt  *radEvt = (ResourceRequestEvt *)e;
	return radEvt->m_data;
}

ResourceResponse toResourceResponse(QP::QEvt const *e) {
	ASSERT_SIG(e, RESOURCE_GRANT_SIG);
	ResourceResponseEvt  *radEvt = (ResourceResponseEvt *)e;
	return radEvt->m_data;
}

ProvisionComponentRequest toProvisionComponentRequest(QP::QEvt const *e) {
	ASSERT_SIG(e, PROVISION_COMPONENT_REQ_SIG);
	ProvisionComponentRequestEvt * evt =  (ProvisionComponentRequestEvt *) e;
	return evt->m_data;
}

ProvisionComponentResponse toProvisionComponentResponse(QP::QEvt const *e) {
	ASSERT_SIG(e, PROVISION_COMPONENT_RESP_SIG);
	ProvisionComponentResponseEvt * evt =  (ProvisionComponentResponseEvt *) e;
	return evt->m_data;
}

InterruptData toInterrupt(QP::QEvt const *e) {
	ASSERT_SIG(e, INTERRUPT_SIG);
	InterruptDataEvt * evt =  (InterruptDataEvt *) e;
	return evt->m_data;
}

DriverQueryRequest toDriverQueryRequest(QP::QEvt const *e) {
	ASSERT_SIG(e, DRIVER_REQ_SIG);
	DriverQueryRequestEvt * evt =  (DriverQueryRequestEvt *) e;
	return evt->m_data;
}

} /* namespace RAD */
