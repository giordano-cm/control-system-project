#include "ADC_Controller.h"

uint16_t ADC_Value[3] = {0,0,0};

/*FUNCTION*--------------------------------------------------------------------
* Function Name  : ADC_Config
* Input Value    :
* Author         : Giordano Cechet Moro
* Returned Value :
* Date			 : 31/05/2019
* Comments       : 
*END*------------------------------------------------------------------------*/
void ADC_Config()
{
    adc1_config_width( ADC_WIDTH_BIT_10 );
    adc1_config_channel_atten(P_ADC_CHANNEL, ADC_ATTEN_DB_11);     // GPIO34
    adc1_config_channel_atten(I_ADC_CHANNEL, ADC_ATTEN_DB_11);     // GPIO35
    adc1_config_channel_atten(D_ADC_CHANNEL, ADC_ATTEN_DB_11);     // GPIO32
}

/*FUNCTION*--------------------------------------------------------------------
* Function Name  : Sensors_Reading_Function
* Input Value    :
* Author         : Giordano Cechet Moro
* Returned Value :
* Date			 : 26/06/2019
* Comments       : 
*END*------------------------------------------------------------------------*/
void Sensors_Reading_Function()
{
    ADC_Value[P_ADC_INDEX] = adc1_get_raw(P_ADC_CHANNEL);
    ADC_Value[I_ADC_INDEX] = adc1_get_raw(I_ADC_CHANNEL);
    ADC_Value[D_ADC_INDEX] = adc1_get_raw(D_ADC_CHANNEL);
}