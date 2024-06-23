//
// Created by Liam
// 2023/12/01
//
/**
* @author Liam
* @file HStateMachine.c
* @brief 事件驱动型层次式状态机的实现代码
* 使用说明：
* 该文件为事件驱动型层次式状态机的实现代码。
* 最大优点：
* 1. 如果当前状态无法处理该事件，事件将会抛给父状态处理
* 2. 每个状态拥有 状态 进入和退出函数
* 
* 具体实现例子参照：
* Event_Define.c
* 步骤：
* 1. 定义事件
* 2. 定义状态
* 3. 定义状态处理函数
* 4. 状态机创建/初始化 （重要）
*     * 注意 HSM_CHASSIS_Init(swChassis_t *This, char *name) 里面的 This 结构体成员的第一个必须包含 HSM_STATE
*     * 例如：
*     typedef struct{
        HSM parent;
        swChassis_velocity_t target_v;
        swChassis_velocity_t current_v;
        uint8_t swheel_num;
        swChassis_state_e state;
        #ifdef FOUR_WHEELS_CHASSIS
        steering_wheel_t wheels[4];
        #endif
    }swChassis_t;
* @version 1.0
*/

#include "HStateMachine.hpp"
#include <iostream>

namespace HStateMachine {

/**
 * @brief 根状态处理函数
 * 
 * @param This 
 * @param event 
 * @param param 
 * @return HSM_EVENT 
 */
HSM_EVENT HSM_RootHandler(HSM_EVENT event, void *param)
{
    // This is the root state handler, it should never be called
    (void)event;
    (void)param;
    return HSM_NULL;
}

/**
 * @brief 根状态，每一个状态机必须有一个根状态
 * 
 */
HSM_STATE const HSM_ROOT =
{
    .parent = (HSM_STATE_T *)((void *)0),
    .handler = HSM_RootHandler,
    .name = ":ROOT:",
    .level = 0
};

/**
 * @brief 状态创建函数
 * 
 * @param curState 
 * @param name    状态名字
 * @param handler 可以传入一个函数指针，该函数将在状态创建时候被调用
 * @param parent  父状态
 */
void HSM::HSM_STATE_Create(HSM_STATE *This, const char *name, HSM_FN handler, HSM_STATE *parent)
{
    if ((HSM_STATE_T *)((void *)0) == parent)
    {
        parent = (HSM_STATE *)&HSM_ROOT;
    }
    This->name = name;
    This->handler = handler;
    This->parent = parent;
    This->level = parent->level + 1;
    if (curState->level >= HSM_MAX_DEPTH)
    {
        std::cerr << "Too Deep!!" << std::endl;
        while(1);
    }
}

/**
 * @brief HSM状态机创建函数
 * 
 * @param curState 
 * @param name 
 * @param initState 
 */
void HSM::HSM_Create(const char *name, HSM_STATE *initState)
{
    // Supress warning for unused variable if HSM_FEATURE_DEBUG_ENABLE is not defined
    (void)name;

    // Initialize state
    curState = initState;
    // Invoke ENTRY and INIT event
    curState->handler(HSM_ENTRY, 0);
    curState->handler(HSM_INIT, 0);
}

/**
 * @brief 获取当前状态
 * 
 * @param curState 
 * @return HSM_STATE* 
 */
HSM_STATE* HSM::HSM_GetState()
{
    // This returns the current HSM state
    return curState;
}

/**
 * @brief 把当前状态转换成字符串传给 HSM_GetState 要自己实现喔
 * 
 * @param event 
 * @return __weak const* 
 */
// __weak const char *HSM_Evt2Str(uint32_t event)
// {
//     return "UNKNOWN";
// }

/**
 * @brief 是否处于xx状态
 * 
 * @param curState 
 * @param state 
 * @return uint8_t 
 */
uint8_t HSM::HSM_IsInState(HSM_STATE *state)
{
    HSM_STATE *curState;
    // Traverse the parents to find the matching state.
    for (curState = this->curState; curState; curState = curState->parent)
    {
        if (state == curState)
        {
            // Match found, HSM is in state or parent state
            return 1;
        }
    }
    // This HSM is not in state or parent state
    return 0;
}

/**
 * @brief 状态机运行函数
 * 
 * @param curState 
 * @param event 
 * @param param 
 */
void HSM::HSM_Run(HSM_EVENT event, void *param)
{
    // This runs the state's event handler and forwards unhandled events to
    // the parent state
    HSM_STATE *state = this->curState;
    while (event)
    {
        event = state->handler(event, param);
        state = state->parent;
        if (event)
        {
            
        }
    }
}

/**
 * @brief 状态转换函数
 * 
 * @param This 
 * @param nextState 
 * @param param 
 * @param method 
 */
void HSM::HSM_Tran(HSM_STATE *nextState, void *param, void (*method)(void *param))
{
    HSM_STATE *list_exit[HSM_MAX_DEPTH];
    HSM_STATE *list_entry[HSM_MAX_DEPTH];
    uint8_t cnt_exit = 0;
    uint8_t cnt_entry = 0;
    uint8_t idx;
    // This performs the state transition with calls of exit, entry and init
    // Bulk of the work handles the exit and entry event during transitions
    // 1) Find the lowest common parent state
    HSM_STATE *src = this->curState;
    HSM_STATE *dst = nextState;
    // 1a) Equalize the levels
    while (src->level != dst->level)
    {
        if (src->level > dst->level)
        {
            // source is deeper
            list_exit[cnt_exit++] = src;
            src = src->parent;
        }
        else
        {
            // destination is deeper
            list_entry[cnt_entry++] = dst;
            dst = dst->parent;
        }
    }
    // 1b) find the common parent
    while (src != dst)
    {
        list_exit[cnt_exit++] = src;
        src = src->parent;
        list_entry[cnt_entry++] = dst;
        dst = dst->parent;
    }
    // 2) Process all the exit events
    for (idx = 0; idx < cnt_exit; idx++)
    {
        src = list_exit[idx];
        src->handler(HSM_EXIT, param);
    }
    // 3) Call the transitional method hook
    if (method)
    {
        method(param);
    }
    // 4) Process all the entry events
    for (idx = 0; idx < cnt_entry; idx++)
    {
        dst = list_entry[cnt_entry - idx - 1];
        dst->handler(HSM_ENTRY, param);
    }
    // 5) Now we can set the destination state
    this->curState = nextState;
}

} // namespace HStateMachine