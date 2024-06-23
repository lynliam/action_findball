#ifndef EVENT_DEFINE_H
#define EVENT_DEFINE_H
#include "HStateMachine.hpp"

namespace HStateMachine {

// HSM Event Define
#define HSM_IDLE        (HSM_START)
#define HSM_OVER         (HSM_START+1)
#define HSM_CHASSIS_CORRECTED    (HSM_START+2)   
#define HSM_CHASSIS_CORRECTING   (HSM_START+3)
#define HSM_CHASSIS_READY        (HSM_START+4)
#define HSM_CHASSIS_AMING        (HSM_START+5)
#define HSM_CHASSIS_RUNNING      (HSM_START+6)
#define HSM_CHASSIS_ERROR        (HSM_START+7)

HSM_EVENT IDLE_Handler(HSM_EVENT event, void *param);
HSM_EVENT LOOKING_Handler(HSM_EVENT event, void *param);
HSM_EVENT NO_BALL_Handler(HSM_EVENT event, void *param);
HSM_EVENT UP_FINDING_Handler(HSM_EVENT event, void *param);
HSM_EVENT APPROACHING_Handler(HSM_EVENT event, void *param);
HSM_EVENT CATCHING_Handler(HSM_EVENT event, void *param);
HSM_EVENT TORWORD_Handler(HSM_EVENT event, void *param);
HSM_EVENT BACKWARD_Handler(HSM_EVENT event, void *param);
HSM_EVENT BUMP_UP_Handler(HSM_EVENT event, void *param);
HSM_EVENT CATCHING_PURPLE_Handler(HSM_EVENT event, void *param);
HSM_EVENT FAIL_Handler(HSM_EVENT event, void *param);
HSM_EVENT SUCCESS_Handler(HSM_EVENT event, void *param);

extern HSM_EVENT Next_Event;

} // namespace HStateMachine

#endif