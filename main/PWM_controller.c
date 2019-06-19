#include "PWM_controller.h"

/**************************************************************************
* 
*   Name: PWM__controller_init
*   Author: Giordano Cechet Moro
*   Date: 18/05/2019
*   Explanation: 
*   
**************************************************************************/
void PWM__Controller_Init()
{
    mcpwm_config_t config = {
        .frequency = FREQUENCIA_PWM,
        .cmpr_a = 0.0,                      // duty cycle em A
        .cmpr_b = 0.0,                      // duty cycle em B
        .duty_mode = MCPWM_DUTY_MODE_0,     // nível alto
        .counter_mode = MCPWM_UP_COUNTER
    };

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_16);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, (mcpwm_config_t *)&config);
}

/**************************************************************************
* 
*   Name: PWM__Init
*   Author: Giordano Cechet Moro
*   Date: 21/05/2019
*   Explanation: 
*   
**************************************************************************/
void PWM__Start()
{
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

/**************************************************************************
* 
*   Name: PWM__Set_Duty
*   Author: Giordano Cechet Moro
*   Date: 21/05/2019
*   Explanation: 
*   
**************************************************************************/
void PWM__Set_Duty( float duty_cycle_percent )
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle_percent);
}