#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "math.h"
#include "esp_timer.h"
//#include "driver/gpio.h"
#include "I2C_controller.h"
#include "PWM_controller.h"
//#include "Timer_controller.h"

#define SAMPLE_PERIOD_MS	100

//-------------------------------------------------------------------------
// Handles for RTOS Tasks
//-------------------------------------------------------------------------
TaskHandle_t Get_Data_From_MPU_Task_Handle = NULL;
TaskHandle_t Angle_Calculation_Task_Handle = NULL;

//-------------------------------------------------------------------------
// Semaphore Handles for RTOS Tasks
//-------------------------------------------------------------------------
SemaphoreHandle_t Get_Data_From_MPU_Task_Semaphore = NULL;
SemaphoreHandle_t Angle_Calculation_Task_Semaphore = NULL;

//-------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------
int16_t data_accel_x = 0;
int16_t data_accel_y = 0;
int16_t data_accel_z = 0;
int16_t data_gyros_x = 0;
int16_t data_gyros_y = 0;
int16_t data_gyros_z = 0;
int8_t angle_x_degree = 0;
//uint64_t t0_us = 0;
//uint64_t t1_us = 0;
//double_t angle_x_degree;
//double_t angle_y_degree;
//double_t angle_z_degree;
//double_t angle_x_rad;
//double_t angle_y_rad;
//double_t angle_z_rad;

//-------------------------------------------------------------------------
// Function Headers
//-------------------------------------------------------------------------
void BLDC__ESC_Init();
void Get_Data_From_MPU_Task( void *pvParameters );
void Angle_Calculation_Task( void *pvParameters );

/**************************************************************************
* 
*   Name: app_main
*   Author: Giordano Cechet Moro
*   Date: xx/xx/2019
*   Explanation: Função principal da aplicação
*   
**************************************************************************/
void app_main()
{
	
	uint8_t data_config;		// para configuração dos registradores e do MPU-6050

	//-------------------------------------------------------------------------
	// Inicialização I2C
	//-------------------------------------------------------------------------
	I2C__master_init();
	/* data_config = MPU6050__FIFO_EN_VALUE; */
	/* I2C__write_byte(MPU6050__FIFO_EN_ADDRESS, &data); */
	/* data_config = MPU6050__I2C_MST_CTRL_VALUE; */
	/* I2C__write_byte(MPU6050__I2C_MST_CTRL_ADDRESS, &data); */
	/* data_config = MPU6050__SIGNAL_PATH_RESET_VALUE; */
	/* I2C__write_byte(MPU6050__SIGNAL_PATH_RESET_ADDRESS, &data); */
	data_config = MPU6050__GYRO_CONFIG_VALUE;
	I2C__write_byte(MPU6050__GYRO_CONFIG_ADDRESS, &data_config);
	data_config = MPU6050__ACCEL_CONFIG_VALUE;
	I2C__write_byte(MPU6050__ACCEL_CONFIG_ADDRESS, &data_config);
	data_config = MPU6050__CONFIG_VALUE;
	I2C__write_byte(MPU6050__CONFIG_ADRESS, &data_config);
	data_config = MPU6050__PWR_MGMT_1_VALUE;
	I2C__write_byte(MPU6050__PWR_MGMT_1_ADDRESS, &data_config);
	/* data_config = MPU6050__PWR_MGMT_2_VALUE; */
	/* I2C__write_byte(MPU6050__PWR_MGMT_2_ADDRESS, &data); */

	//-------------------------------------------------------------------------
	// Inicialização PWM
	//-------------------------------------------------------------------------
	//PWM__Controller_Init();
	//PWM__Start();
	//BLDC__ESC_Init();

	//-------------------------------------------------------------------------
	// Inicialização de Task RTOS
	//-------------------------------------------------------------------------
	//Get_Data_From_MPU_Task_Semaphore = xSemaphoreCreateBinary();
	//xTaskCreate( Get_Data_From_MPU_Task, "Get_Data_From_MPU_Task", ( 1 * 1024 ), NULL, 5, &Get_Data_From_MPU_Task_Handle );
    //configASSERT( Get_Data_From_MPU_Task_Handle );

	//Angle_Calculation_Task_Semaphore = xSemaphoreCreateBinary();
	//xTaskCreate( Angle_Calculation_Task, "Angle_Calculation_Task", ( 1 * 1024 ), NULL, 5, &Angle_Calculation_Task_Handle );
    //configASSERT( Angle_Calculation_Task_Handle );

	for (;;) {
		
		/*
		PWM__Set_Duty( (0.1)*(FULL_SCALE_DUTY_CYCLE_PERCENT-INIT_SCALE_DUTY_CYCLE_PERCENT) + INIT_SCALE_DUTY_CYCLE_PERCENT );
		ESP_LOGI("MAIN","10");
		vTaskDelay( 3000 / portTICK_PERIOD_MS );
		PWM__Set_Duty( (0.2)*(FULL_SCALE_DUTY_CYCLE_PERCENT-INIT_SCALE_DUTY_CYCLE_PERCENT) + INIT_SCALE_DUTY_CYCLE_PERCENT );
		ESP_LOGI("MAIN","20");
		vTaskDelay( 3000 / portTICK_PERIOD_MS );
		PWM__Set_Duty( (0.3)*(FULL_SCALE_DUTY_CYCLE_PERCENT-INIT_SCALE_DUTY_CYCLE_PERCENT) + INIT_SCALE_DUTY_CYCLE_PERCENT );
		ESP_LOGI("MAIN","30");
		vTaskDelay( 3000 / portTICK_PERIOD_MS );
		PWM__Set_Duty( (0.4)*(FULL_SCALE_DUTY_CYCLE_PERCENT-INIT_SCALE_DUTY_CYCLE_PERCENT) + INIT_SCALE_DUTY_CYCLE_PERCENT );
		ESP_LOGI("MAIN","40");
		vTaskDelay( 3000 / portTICK_PERIOD_MS );
		*/

		vTaskDelay( SAMPLE_PERIOD_MS / portTICK_PERIOD_MS );
		//t0_us = t1_us;
		//t1_us = esp_timer_get_time();
		Get_Data_From_MPU_Task( NULL );
		Angle_Calculation_Task( NULL );
		//ESP_LOGI("TIMER", "t0: %llu  t1: %llu  delta_time: %llu", t0_us, t1_us, (t1_us-t0_us));
		ESP_LOGI("VALUE","x: %d\tdata_gyros_y: %d", angle_x_degree, data_gyros_y);
		/*
		ESP_LOGI("VALUE","x: %.1f\taccel_x: %d\taccel_z: %d\tx²: %f\tz²: %f\tsqrt: %f\tarc: %f\tacos(arc): %f",
		angle_x_degree, data_accel_x, data_accel_z, (double_t)data_accel_x*data_accel_x, (double_t)data_accel_z*data_accel_z,
		sqrt(((double_t)data_accel_z*data_accel_z) + ((double_t)data_accel_x*data_accel_x)),
		data_accel_z/sqrt(((double_t)data_accel_z*data_accel_z) + ((double_t)data_accel_x*data_accel_x)),
		(180.0/3.1415927)*acos(data_accel_z/sqrt(((double_t)data_accel_z*data_accel_z) + ((double_t)data_accel_x*data_accel_x))) );
		*/
	}
}

/**************************************************************************
* 
*   Name: BLDC__ESC_Init
*   Author: Giordano Cechet Moro
*   Date: 25/05/2019
*   Explanation: Inicializa o ESC, varrendo o valor mínimo, médio e máximo
*				 do duty cycle do pwm que vai na entrada do ESC
*   
**************************************************************************/
void BLDC__ESC_Init()
{
	PWM__Set_Duty(INIT_SCALE_DUTY_CYCLE_PERCENT);
	ESP_LOGI("BLDC","INIT_SCALE_DUTY_CYCLE_PERCENT");
	vTaskDelay( 2000 / portTICK_PERIOD_MS );
}

/**************************************************************************
* 
*   Name: Get_Data_From_MPU_Task
*   Author: Giordano Cechet Moro
*   Date: 25/05/2019
*   Explanation: Tarefa RTOS que alimenta as variáveis:
*				 data_accel_x, data_accel_y, data_accel_z
*				 data_gyros_x, data_gyros_y, data_gyros_z
*   
**************************************************************************/
void Get_Data_From_MPU_Task( void *pvParameters )
{

	uint8_t data_flag = 0x00;

		// Acelerômetro
		I2C__read_byte(MPU6050__ACCEL_XOUT_MSB_ADDRESS, &data_flag);
		data_accel_x = (data_flag << 8);
		I2C__read_byte(MPU6050__ACCEL_XOUT_LSB_ADDRESS, &data_flag);
		data_accel_x += (uint8_t)data_flag;

		I2C__read_byte(MPU6050__ACCEL_YOUT_MSB_ADDRESS, &data_flag);
		data_accel_y = (data_flag << 8);
		I2C__read_byte(MPU6050__ACCEL_YOUT_LSB_ADDRESS, &data_flag);
		data_accel_y += (uint8_t)data_flag;

		I2C__read_byte(MPU6050__ACCEL_ZOUT_MSB_ADDRESS, &data_flag);
		data_accel_z = (data_flag << 8);
		I2C__read_byte(MPU6050__ACCEL_ZOUT_LSB_ADDRESS, &data_flag);
		data_accel_z += (uint8_t)data_flag;

		// Giroscópio
		I2C__read_byte(MPU6050__GYRO_XOUT_MSB_ADDRESS, &data_flag);
		data_gyros_x = (data_flag << 8);
		I2C__read_byte(MPU6050__GYRO_XOUT_LSB_ADDRESS, &data_flag);
		data_gyros_x += (uint8_t)data_flag;

		I2C__read_byte(MPU6050__GYRO_YOUT_MSB_ADDRESS, &data_flag);
		data_gyros_y = (data_flag << 8);
		I2C__read_byte(MPU6050__GYRO_YOUT_LSB_ADDRESS, &data_flag);
		data_gyros_y += (uint8_t)data_flag;

		I2C__read_byte(MPU6050__GYRO_ZOUT_MSB_ADDRESS, &data_flag);
		data_gyros_z = (data_flag << 8);
		I2C__read_byte(MPU6050__GYRO_ZOUT_LSB_ADDRESS, &data_flag);
		data_gyros_z += (uint8_t)data_flag;

}


void Angle_Calculation_Task( void *pvParameters )
{
	double_t x_squared_flag = (data_accel_x*data_accel_x);
	double_t z_squared_flag = (data_accel_z*data_accel_z);
	//uint64_t delta_time = t1_us - t0_us;

	angle_x_degree = (180.0/3.1415927) * ( (0.5*((angle_x_degree*((3.1415927/180.0))) + (data_gyros_y * (SAMPLE_PERIOD_MS/1000)))) + (0.5*acos( data_accel_z/sqrt( (x_squared_flag) + (z_squared_flag)))) );
	//angle_x_degree = (180.0/3.1415927) * ( (acos( data_accel_z/sqrt( (x_squared_flag) + (z_squared_flag)))) );
	//if (data_accel_x < 0) angle_x_degree *= (-1);	// quando vai para negativo, data_accel_x fica negativo
}