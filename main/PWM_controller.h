#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm.h"

//#define FREQUENCIA_PWM  50
//#define INIT_SCALE_DUTY_CYCLE_PERCENT   ( ( 1000 * FREQUENCIA_PWM ) / 10000 )
//#define MIDDLE_SCALE_DUTY_CYCLE_PERCENT ( ( 1500 * FREQUENCIA_PWM ) / 10000 )
//#define FULL_SCALE_DUTY_CYCLE_PERCENT   ( ( 2000 * FREQUENCIA_PWM ) / 10000 )
//#define BLDC__TIME_BETWEEN_CONFIG_MS    ( 10000 / FREQUENCIA_PWM )

enum {
    FREQUENCIA_PWM = 50,
    INIT_SCALE_DUTY_CYCLE_PERCENT = ( ( 1000 * FREQUENCIA_PWM ) / 10000 ),
    MIDDLE_SCALE_DUTY_CYCLE_PERCENT = ( ( 1500 * FREQUENCIA_PWM ) / 10000 ),
    FULL_SCALE_DUTY_CYCLE_PERCENT = ( ( 2000 * FREQUENCIA_PWM ) / 10000 ),
    BLDC__TIME_BETWEEN_CONFIG_MS = ( 10000 / FREQUENCIA_PWM ),
};

//-------------------------------------------------------------------------
// Function Headers
//-------------------------------------------------------------------------
void PWM__Controller_Init();
void PWM__Start();
void PWM__Set_Duty( float duty_cycle_percent );