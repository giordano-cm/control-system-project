#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "freertos/semphr.h"
//#include "freertos/portmacro.h"
#include "driver/adc.h"
//#include "driver/gpio.h"

enum {
    P_ADC_CHANNEL = ADC1_CHANNEL_6,   // GPIO34
    I_ADC_CHANNEL = ADC1_CHANNEL_7,   // GPIO35
    D_ADC_CHANNEL = ADC1_CHANNEL_4,   // GPIO32
};

enum {
    P_ADC_INDEX = 0,
    I_ADC_INDEX,
    D_ADC_INDEX,
};

extern uint16_t ADC_Value[3];

void ADC_Config();
void Sensors_Reading_Function();