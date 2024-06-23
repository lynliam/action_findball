#include "Event_Define.hpp"
#include "HStateMachine.hpp"

namespace HStateMachine {
/**
 * @brief 状态创建
 */
HSM_STATE IDLE;
HSM_STATE LOOKING;
HSM_STATE NO_BALL;
HSM_STATE UP_FINDING;
HSM_STATE APPROACHING;
HSM_STATE CATCHING;
HSM_STATE TORWORD;
HSM_STATE BACKWARD;
HSM_STATE BUMP_UP;
HSM_STATE CATCHING_PURPLE;
HSM_STATE FAIL;
HSM_STATE SUCCESS;


HSM_EVENT Next_Event;

HSM CatchBall;

HSM_EVENT IDLE_Handler(HSM_EVENT event, void *param)
{
    if(event == HSM_ENTRY)
    {
    }
    else if(event == HSM_EXIT)
    {
    }
    else if(event ==HSM_CHASSIS_START)
    {
        This->HSM_Tran(&CHASSIS_ON, 0, nullptr);
        Next_Event = HSM_CHASSIS_INIT;
        return 0;
    }
    return event;
}

HSM_EVENT CHASSIS_ON_Handler(HSM *This, HSM_EVENT event, void *param)
{
    if(event == HSM_ENTRY)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    else if(event == HSM_EXIT)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
    }
    else if(event == HSM_CHASSIS_INIT)
    {
        HSM_Tran(This, &CHASSIS_ONCorrecting, 0, NULL); 
        return 0;
    }
    else if(event == HSM_CHASSIS_ERROR)
    {
        HSM_Tran(This, &CHASSIS_ONError, 0, NULL); 
        return 0;
    }
    return event;
}

HSM_EVENT CHASSIS_ONError_Handler(HSM *This, HSM_EVENT event, void *param)
{
    if(event == HSM_ENTRY)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
    }
    else if(event == HSM_EXIT)
    {
    }
    else if(event == HSM_CHASSIS_ERROR)
    {
        //HSM_Tran(This, &CHASSIS_OFF, 0, NULL);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
        vTaskDelay(500/portTICK_RATE_MS);
        return 0;
    }
    return event;
}


HSM_EVENT CHASSIS_ONCorrecting_Handler(HSM *This, HSM_EVENT event, void *param)
{
    if(event == HSM_ENTRY)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
        Next_Event = HSM_CHASSIS_CORRECTING;
        if(swChassis_startCorrect(&mychassis) != HAL_OK)
        {
            HSM_Tran(This, &CHASSIS_ONError, 0, NULL); 
            Next_Event = HSM_CHASSIS_ERROR;
        }
    }
    else if(event == HSM_EXIT)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
    }
    else if (event == HSM_CHASSIS_CORRECTING) 
    {
        swChassis_executor(&mychassis);
        if(swChassis_CheckCorrect(&mychassis) == HAL_OK)
        {
            Next_Event = HSM_CHASSIS_CORRECTED;
        }
        return 0;
    }
    else if(event == HSM_CHASSIS_CORRECTED)
    {
        HSM_Tran(This, &CHASSIS_ONReady, 0, NULL); 
        Next_Event = HSM_CHASSIS_READY;
        return 0;
    }
    return event;
}


HSM_EVENT CHASSIS_ONReady_Handler(HSM *This, HSM_EVENT event, void *param)
{
    if(event == HSM_ENTRY)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
        swChassis_set_still(&mychassis);
        swChassis_executor(&mychassis);
    }
    else if(event == HSM_EXIT)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
    }
    else if(event == HSM_CHASSIS_READY)
    {
        swChassis_set_still(&mychassis);
        swChassis_executor(&mychassis);
        if(swChassis_check_velocity(&mychassis) == HAL_BUSY)
        {
            HSM_Tran(This, &CHASSIS_ONReadyRunning, 0, NULL);
            Next_Event = HSM_CHASSIS_RUNNING;
        }
        return 0;
    }
    else if(event == HSM_CHASSIS_ERROR)
    {
        HSM_Tran(This, &CHASSIS_ONError, 0, NULL); 
        return 0;
    }
    return event;
}

HSM_EVENT CHASSIS_ONReadyRunning_Handler(HSM *This, HSM_EVENT event, void *param)
{
    if(event == HSM_ENTRY)
    {
    }
    else if(event == HSM_EXIT)
    {
    }
    else if(event == HSM_CHASSIS_RUNNING)
    {
        calculate_target_velocity(&mychassis);
        swChassis_set_Running(&mychassis);
        if(swChassis_check_AimorRun(&mychassis) == 0)
        {
            Next_Event = HSM_CHASSIS_AMING;
        }
        swChassis_executor(&mychassis);

        
        if(swChassis_check_velocity(&mychassis) == HAL_OK)
        {
            HSM_Tran(This, &CHASSIS_ONReady, 0, NULL);
            Next_Event = HSM_CHASSIS_READY;
        }
        return 0;
    }
    else if(event == HSM_CHASSIS_AMING)
    {
        calculate_target_velocity(&mychassis);
        if(swChassis_check_AimorRun(&mychassis) == 0)
        {
            Next_Event = HSM_CHASSIS_RUNNING;
            swChassis_set_Running(&mychassis);
        }
        swChassis_executor(&mychassis);
        return 0;
    }
    return event;
}

void HSM_CHASSIS_Init(swChassis_t *This, char *name)
{
    HSM_STATE_Create(&CHASSIS_ON, "CHASSIS_ON", CHASSIS_ON_Handler, NULL);
    HSM_STATE_Create(&CHASSIS_OFF, "CHASSIS_OFF", CHASSIS_OFF_Handler, NULL);
    HSM_STATE_Create(&CHASSIS_ONReady, "CHASSIS_ONReady", CHASSIS_ONReady_Handler, &CHASSIS_ON);
    HSM_STATE_Create(&CHASSIS_ONError, "CHASSIS_ONError", CHASSIS_ONError_Handler, &CHASSIS_ON);
    HSM_STATE_Create(&CHASSIS_ONCorrecting, "CHASSIS_ONCorrecting", CHASSIS_ONCorrecting_Handler, &CHASSIS_ON);
    HSM_STATE_Create(&CHASSIS_ONReadyRunning, "CHASSIS_ONReadyRunning", CHASSIS_ONReadyRunning_Handler, &CHASSIS_ONReady);

    HSM_Create((HSM *)This, "chassis_main", &CHASSIS_OFF);
}

void HSM_CHASSIS_Run(swChassis_t *This, HSM_EVENT event, void *param)
{
    HSM_Run((HSM *)This, event, param);
}


}
