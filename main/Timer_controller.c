#include "Timer_controller.h"

enum{
    TIMER_DIVIDER = 80,
    TIMER_FREQUENCY_HZ = ( TIMER_BASE_CLK / TIMER_DIVIDER ),
    PERIODO_AMOSTRAGEM_US = 250,
};

void IRAM_ATTR Timer_Intr_Controller(void *para)
{
    //TIMERG0.hw_timer[TIMER_0].update = 1;

    //uint32_t timer_counter_value = TIMERG0.hw_timer[TIMER_0].cnt_low;
    //timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_counter_value);

    //tempo_flag = 1;

    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    //ESP_LOGI("TIMER","Entrou ;)");

}

void Timer_Config()
{
    timer_config_t timer_config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_DOWN,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 80       // clock do timer = 80 MHz / 80 = 1 MHz
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, ( PERIODO_AMOSTRAGEM_US * TIMER_FREQUENCY_HZ ) / 1000000 );
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 0x00ULL);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, Timer_Intr_Controller, NULL, ESP_INTR_FLAG_IRAM, NULL);
}

void Timer_Start()
{
    timer_start(TIMER_GROUP_0, TIMER_0);
}