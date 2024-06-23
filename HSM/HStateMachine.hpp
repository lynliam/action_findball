//
// Created by Liam
// 2023/12/01

#ifndef HSTATEMACHINE_H
#define HSTATEMACHINE_H

#include <cstdint>
#include <vector>

namespace HStateMachine {
#define HSM_MAX_DEPTH  5

//----State definitions----
#define HSM_NULL   0
#define HSM_START  1
#define HSM_INIT   ((HSM_EVENT)(-3))
#define HSM_ENTRY  ((HSM_EVENT)(-2))
#define HSM_EXIT   ((HSM_EVENT)(-1))

//----State machine definitions----
typedef uint16_t HSM_EVENT ;
typedef struct HSM_STATE_T HSM_STATE;
typedef HSM_EVENT (* HSM_FN)(HSM_EVENT event, void *param);
typedef struct HSM_STATE_T
{
    struct HSM_STATE_T *parent;          // parent state
    HSM_FN handler;             // associated event handler for state
    const char *name;           // name of state
    uint8_t level;              // depth level of the state
};

class HSM
{
    public:
    HSM();
    void HSM_STATE_Create(HSM_STATE *This,const char *name, HSM_FN handler, HSM_STATE *parent);
    void HSM_Create(const char *name, HSM_STATE *initState);
    HSM_STATE *HSM_GetState();
    uint8_t HSM_IsInState(HSM_STATE *state);
    void HSM_Run(HSM_EVENT event, void *param);
    void HSM_Tran(HSM_STATE *nextState, void *param, void (*method)(void *param));

    private:
    HSM_STATE *curState;        // Current HSM State
};

}

#endif