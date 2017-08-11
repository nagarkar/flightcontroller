//****************************************************************************
// Model: RAD.qm
// File:  RAD/ComponentAO.cpp
//
// This code has been generated by QM tool (see state-machine.com/qm).
// DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
//
// This code is covered by the following commercial QP license:
// License #   : QPC-EVAL-170804A
// Issued to   : Chinmay Nagarkar
// Framework(s): qpc qpcpp qpn
// Support ends: 2017-12-31
// Product(s)  :
// This license is available only for evaluation purposes and
// the generated code is still licensed under the terms of GPL.
// Please submit request for extension of the evaluaion period at:
// https://state-machine.com/licensing/#RequestForm
//****************************************************************************
//${RAD::RAD::ComponentAO.cpp} ...............................................
#include "./ComponentAO.h"


using namespace QP;

namespace RAD {


#if ((QP_VERSION < 580) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8)))
#error qpcpp version 5.8.0 or higher required
#endif

//${RAD::ComponentAO} ........................................................

//${RAD::ComponentAO::ArmActivityTimeout} ....................................
void ComponentAO::ArmActivityTimeout() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::ArmInitializationTimeout} ..............................
void ComponentAO::ArmInitializationTimeout() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::DisarmActivityTimeout} .................................
void ComponentAO::DisarmActivityTimeout() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::DisarmInitializationTimeout} ...........................
void ComponentAO::DisarmInitializationTimeout() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnInitializationTimeout} ...............................
void ComponentAO::OnInitializationTimeout() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnActivityTimeout} .....................................
void ComponentAO::OnActivityTimeout() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::RequestResources} ......................................
void ComponentAO::RequestResources() {
    Q_onAssert("Unsupported Method", 0);
}

//${RAD::ComponentAO::OnServiceRequest} ......................................
void ComponentAO::OnServiceRequest(RequestData  request) {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnServiceResponse} .....................................
void ComponentAO::OnServiceResponse(ResponseData  response) {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnInterrupt} ...........................................
void ComponentAO::OnInterrupt(InterruptData interrupt) {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnCreate} ..............................................
void ComponentAO::OnCreate() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnStart} ...............................................
void ComponentAO::OnStart() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnRestart} .............................................
void ComponentAO::OnRestart() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnPause} ...............................................
void ComponentAO::OnPause() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnStopped} .............................................
void ComponentAO::OnStopped() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnError} ...............................................
void ComponentAO::OnError() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::ErrorOccurred} .........................................
bool ComponentAO::ErrorOccurred() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::HasSufficientActivityOccurred} .........................
bool ComponentAO::HasSufficientActivityOccurred() {
    Q_onAssert("Unsupported Method", 0);
}

//${RAD::ComponentAO::OnResourceGrant} .......................................
void ComponentAO::OnResourceGrant(ResourceResponse & resp) {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnResume} ..............................................
void ComponentAO::OnResume() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::ClearError} ............................................
void ComponentAO::ClearError() {
    m_errorOccurred = false;
}
//${RAD::ComponentAO::SetErrorOccurred} ......................................
void ComponentAO::SetErrorOccurred() {
    m_errorOccurred = true;
}
//${RAD::ComponentAO::ProvisionComponent} ....................................
void ComponentAO::ProvisionComponent(ProvisionComponentRequest & request) {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::ReserveResource} .......................................
void ComponentAO::ReserveResource(ResourceRequest & request) {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::GetDriverQueryResponse} ................................
DriverQueryResponse ComponentAO::GetDriverQueryResponse(DriverQueryRequest & req) {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::getOfficialId} .........................................
ID ComponentAO::getOfficialId() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::Start} .................................................
void ComponentAO::Start(uint8_t prio) {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::RequestComponentId} ....................................
void ComponentAO::RequestComponentId() {
    Q_onAssert("Unsupported Method", 0);
}
//${RAD::ComponentAO::OnProvisionComponentResponse} ..........................
void ComponentAO::OnProvisionComponentResponse(ProvisionComponentResponse & resp) {
    Q_onAssert("Unsupported Method", 0);
}

//${RAD::ComponentAO::SM} ....................................................
QP::QState ComponentAO::initial(ComponentAO * const me, QP::QEvt const * const e) {
    // ${RAD::ComponentAO::SM::initial}
    //me->OnInitial();
    return Q_TRAN(&Zombie);
}
//${RAD::ComponentAO::SM::Zombie} ............................................
QP::QState ComponentAO::Zombie(ComponentAO * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${RAD::ComponentAO::SM::Zombie}
        case Q_ENTRY_SIG: {
            me->OnCreate();
            status_ = Q_HANDLED();
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::START}
        case START_SIG: {
            me->OnStart();
            // ${RAD::ComponentAO::SM::Zombie::START::[IsService]}
            if (me->m_type == SERVICE) {
                status_ = Q_TRAN(&WaitingForInitialization);
            }
            // ${RAD::ComponentAO::SM::Zombie::START::[IsDevice]}
            else if (me->m_type == DEVICE) {
                status_ = Q_TRAN(&DeviceAcceptingRequests);
            }
            else {
                status_ = Q_UNHANDLED();
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&top);
            break;
        }
    }
    return status_;
}
//${RAD::ComponentAO::SM::Zombie::Error} .....................................
QP::QState ComponentAO::Error(ComponentAO * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${RAD::ComponentAO::SM::Zombie::Error}
        case Q_ENTRY_SIG: {
            me->OnError();
            if(!MAX_RETRIES_REACHED()) {
               me->POST(new QEvt(RESTART_SIG), me);
            }
            status_ = Q_HANDLED();
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::Error::RESTART}
        case RESTART_SIG: {
            me->OnRestart();
            status_ = Q_TRAN(&WaitingForInitialization);
            break;
        }
        default: {
            status_ = Q_SUPER(&Zombie);
            break;
        }
    }
    return status_;
}
//${RAD::ComponentAO::SM::Zombie::Stopped} ...................................
QP::QState ComponentAO::Stopped(ComponentAO * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${RAD::ComponentAO::SM::Zombie::Stopped}
        case Q_ENTRY_SIG: {
            me->OnStopped();
            status_ = Q_HANDLED();
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::Stopped::RESTART}
        case RESTART_SIG: {
            me->OnRestart();
            status_ = Q_TRAN(&WaitingForInitialization);
            break;
        }
        default: {
            status_ = Q_SUPER(&Zombie);
            break;
        }
    }
    return status_;
}
//${RAD::ComponentAO::SM::Zombie::Paused} ....................................
QP::QState ComponentAO::Paused(ComponentAO * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${RAD::ComponentAO::SM::Zombie::Paused}
        case Q_ENTRY_SIG: {
            me->OnPause();
            status_ = Q_HANDLED();
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::Paused::RESUME}
        case RESUME_SIG: {
            me->OnResume();
            status_ = Q_TRAN(&ServiceAcceptingRequests);
            break;
        }
        default: {
            status_ = Q_SUPER(&Zombie);
            break;
        }
    }
    return status_;
}
//${RAD::ComponentAO::SM::Zombie::ServiceAcceptingRequests} ..................
QP::QState ComponentAO::ServiceAcceptingRequests(ComponentAO * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${RAD::ComponentAO::SM::Zombie::ServiceAcceptingRequests}
        case Q_ENTRY_SIG: {
            //me->ConfirmComponentStarted();
            status_ = Q_HANDLED();
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::ServiceAccepting~::SERVICE_REQ}
        case SERVICE_REQ_SIG: {
            me->ClearError();
            me->OnServiceRequest(toRequest(e));
            // ${RAD::ComponentAO::SM::Zombie::ServiceAccepting~::SERVICE_REQ::[ErrorOccurred]}
            if (me->ErrorOccurred()) {
                //PUBLISH_FAILURE_RESPONSE(e);
                status_ = Q_TRAN(&Error);
            }
            else {
                status_ = Q_UNHANDLED();
            }
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::ServiceAccepting~::INTERRUPT}
        case INTERRUPT_SIG: {
            me->ClearError();
            me->OnInterrupt(toInterrupt(e));
            // ${RAD::ComponentAO::SM::Zombie::ServiceAccepting~::INTERRUPT::[ErrorOcurred]}
            if (me->ErrorOccurred()) {
                status_ = Q_TRAN(&Error);
            }
            else {
                status_ = Q_UNHANDLED();
            }
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::ServiceAccepting~::ACTIVITY_TIMEOUT}
        case ACTIVITY_TIMEOUT_SIG: {
            me->ClearError();
            me->OnActivityTimeout();
            // ${RAD::ComponentAO::SM::Zombie::ServiceAccepting~::ACTIVITY_TIMEOUT::[InsufficientActivity]}
            if (!me->HasSufficientActivityOccurred()) {
                status_ = Q_TRAN(&Error);
            }
            else {
                status_ = Q_UNHANDLED();
            }
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::ServiceAccepting~::SERVICE_RESP}
        case SERVICE_RESP_SIG: {
            me->ClearError();
            me->OnServiceResponse(toResponse(e));
            status_ = Q_HANDLED();
            break;
        }
        default: {
            status_ = Q_SUPER(&Zombie);
            break;
        }
    }
    return status_;
}
//${RAD::ComponentAO::SM::Zombie::WaitingForInitialization} ..................
QP::QState ComponentAO::WaitingForInitialization(ComponentAO * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${RAD::ComponentAO::SM::Zombie::WaitingForInitialization}
        case Q_ENTRY_SIG: {
            me->ArmInitializationTimeout();
            status_ = Q_HANDLED();
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::WaitingForInitialization}
        case Q_EXIT_SIG: {
            me->DisarmInitializationTimeout();
            status_ = Q_HANDLED();
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::WaitingForInitia~::initial}
        case Q_INIT_SIG: {
            status_ = Q_TRAN(&RequestingCompId);
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::WaitingForInitia~::INITIALIZATION_TIMEOUT}
        case INITIALIZATION_TIMEOUT_SIG: {
            me->OnInitializationTimeout();
            status_ = Q_TRAN(&Error);
            break;
        }
        default: {
            status_ = Q_SUPER(&Zombie);
            break;
        }
    }
    return status_;
}
//${RAD::ComponentAO::SM::Zombie::WaitingForInitia~::RequestingCompId} .......
QP::QState ComponentAO::RequestingCompId(ComponentAO * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${RAD::ComponentAO::SM::Zombie::WaitingForInitia~::RequestingCompId}
        case Q_ENTRY_SIG: {
            me->RequestComponentId();
            status_ = Q_HANDLED();
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::WaitingForInitia~::RequestingCompId::PROVISION_COMPONENT_RESP}
        case PROVISION_COMPONENT_RESP_SIG: {
            ProvisionComponentResponse resp =
                toProvisionComponentResponse(e);
            me->OnProvisionComponentResponse(resp);
            status_ = Q_TRAN(&RequestingResources);
            break;
        }
        default: {
            status_ = Q_SUPER(&WaitingForInitialization);
            break;
        }
    }
    return status_;
}
//${RAD::ComponentAO::SM::Zombie::WaitingForInitia~::RequestingResources} ....
QP::QState ComponentAO::RequestingResources(ComponentAO * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${RAD::ComponentAO::SM::Zombie::WaitingForInitia~::RequestingResources}
        case Q_ENTRY_SIG: {
            me->RequestResources();
            status_ = Q_HANDLED();
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::WaitingForInitia~::RequestingResour~::RESOURCE_GRANT}
        case RESOURCE_GRANT_SIG: {
            ResourceResponse resp = toResourceResponse(e);
            if (resp.isSuccess()) {
                me->OnResourceGrant(resp);
            }
            // ${RAD::ComponentAO::SM::Zombie::WaitingForInitia~::RequestingResour~::RESOURCE_GRANT::[AllResourcesGranted]}
            if (me->AllResourcesGranted()) {
                status_ = Q_TRAN(&ServiceAcceptingRequests);
            }
            else {
                status_ = Q_UNHANDLED();
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&WaitingForInitialization);
            break;
        }
    }
    return status_;
}
//${RAD::ComponentAO::SM::Zombie::DeviceAcceptingRequests} ...................
QP::QState ComponentAO::DeviceAcceptingRequests(ComponentAO * const me, QP::QEvt const * const e) {
    QP::QState status_;
    switch (e->sig) {
        // ${RAD::ComponentAO::SM::Zombie::DeviceAcceptingR~::PROVISION_COMPONENT_REQ}
        case PROVISION_COMPONENT_REQ_SIG: {
            ProvisionComponentRequest req =
                toProvisionComponentRequest(e);
            me->ProvisionComponent(req);
            // ${RAD::ComponentAO::SM::Zombie::DeviceAcceptingR~::PROVISION_COMPON~::[ErrorOccured]}
            if (me->ErrorOccurred()) {
                status_ = Q_TRAN(&Stopped);
            }
            else {
                status_ = Q_UNHANDLED();
            }
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::DeviceAcceptingR~::RESOURCE_REQ}
        case RESOURCE_REQ_SIG: {

            ResourceRequest req = toResourceRequest(e);
            me->ReserveResource(req);

            // ${RAD::ComponentAO::SM::Zombie::DeviceAcceptingR~::RESOURCE_REQ::[ErrorOccured]}
            if (me->ErrorOccurred()) {
                status_ = Q_TRAN(&Stopped);
            }
            else {
                status_ = Q_UNHANDLED();
            }
            break;
        }
        // ${RAD::ComponentAO::SM::Zombie::DeviceAcceptingR~::DRIVER_REQ}
        case DRIVER_REQ_SIG: {
            DriverQueryRequest req = toDriverQueryRequest(e);
            me->GetDriverQueryResponse(req);
            // ${RAD::ComponentAO::SM::Zombie::DeviceAcceptingR~::DRIVER_REQ::[ErrorOccured]}
            if (me->ErrorOccurred()) {
                status_ = Q_HANDLED();
            }
            else {
                status_ = Q_UNHANDLED();
            }
            break;
        }
        default: {
            status_ = Q_SUPER(&Zombie);
            break;
        }
    }
    return status_;
}

} // namespace RAD

//$define(RAD::DeviceAO)
