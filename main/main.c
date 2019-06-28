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
#include "ADC_Controller.h"

#define SAMPLE_PERIOD_MS	20
#define GYROS_CONVERSION_VALUE_TO_DEGREES_PER_SEC ( 32768 / 250 )
#define RAD_TO_DEGREES_CONVERSION ( 180.0/3.141592654 )
#define ANGLE_SET_POINT_DEGREE 0

//-------------------------------------------------------------------------
// Handles for RTOS Tasks
//-------------------------------------------------------------------------
//TaskHandle_t Get_Data_From_MPU_Task_Handle = NULL;
//TaskHandle_t Angle_Calculation_Task_Handle = NULL;
//TaskHandle_t PID_Calculation_Task_Handle = NULL;

//-------------------------------------------------------------------------
// Semaphore Handles for RTOS Tasks
//-------------------------------------------------------------------------
//SemaphoreHandle_t Get_Data_From_MPU_Task_Semaphore = NULL;
//SemaphoreHandle_t Angle_Calculation_Task_Semaphore = NULL;
//SemaphoreHandle_t PID_Calculation_Task_Semaphore = NULL;

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
int16_t angular_velocity_x_dps = 0;	// dps = degrees per second
//uint64_t t0_us = 0;
//uint64_t t1_us = 0;
//double_t angle_x_degree;
//double_t angle_y_degree;
//double_t angle_z_degree;
//double_t angle_x_rad;
//double_t angle_y_rad;
//double_t angle_z_rad;

// PID Variables and defines
double_t pid_k = 0.0;
double_t pid_p_value = 0.0;
double_t pid_i_value = 0.0;
double_t pid_d_value = 0.0;
int8_t erro_k = 0;
float_t sample_rate_ms = (float_t)SAMPLE_PERIOD_MS;
float_t kp = 0.0;
float_t ki = 0.0;
float_t kd = 0.0;
//#define kp 4
//#define ki 0
//#define kd 0
//#define UPPER_LIMIT_PID	200
//#define LOWER_LIMIT_PID	180
const uint8_t UPPER_LIMIT_PID = 200;
const uint8_t LOWER_LIMIT_PID = 180;

//-------------------------------------------------------------------------
// Function Headers
//-------------------------------------------------------------------------
void BLDC__ESC_Init();
void Get_Data_From_MPU_Task( void *pvParameters );
void Angle_Calculation_Task( void *pvParameters );
void PID_Calculation_Task( void *pvParameters );
void BLDC__ESC_Actuation();

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
	
	data_config = MPU6050__GYRO_CONFIG_VALUE;
	I2C__write_byte(MPU6050__GYRO_CONFIG_ADDRESS, &data_config);
	data_config = MPU6050__ACCEL_CONFIG_VALUE;
	I2C__write_byte(MPU6050__ACCEL_CONFIG_ADDRESS, &data_config);
	data_config = MPU6050__CONFIG_VALUE;
	I2C__write_byte(MPU6050__CONFIG_ADRESS, &data_config);
	data_config = MPU6050__PWR_MGMT_1_VALUE;
	I2C__write_byte(MPU6050__PWR_MGMT_1_ADDRESS, &data_config);

	//-------------------------------------------------------------------------
	// ADC Config
	//-------------------------------------------------------------------------
	ADC_Config();

	//-------------------------------------------------------------------------
	// Inicialização PWM
	//-------------------------------------------------------------------------
	PWM__Controller_Init();
	PWM__Start();
	BLDC__ESC_Init();

	for (;;) {

		vTaskDelay( SAMPLE_PERIOD_MS / portTICK_PERIOD_MS );
		Get_Data_From_MPU_Task( NULL );
		Angle_Calculation_Task( NULL );
		Sensors_Reading_Function();
		//pid_k = 0.185;
		//pid_k += (double_t)ADC_Value[D_ADC_INDEX]/100000;
		PID_Calculation_Task( NULL );
		BLDC__ESC_Actuation();

		//angular_velocity_x_dps = (int16_t)( ( ( data_gyros_y / GYROS_CONVERSION_VALUE_TO_DEGREES_PER_SEC ) ) + 5 );
		//ESP_LOGI("VALUE","x: %d\tdata_gyros_y: %d", angle_x_degree, angular_velocity_x_dps);
		//ESP_LOGI("PID","p: %f\ti: %f\td: %f\tpid: %f", pid_p_value, pid_i_value, pid_d_value, pid_k);
		ESP_LOGI("PID","kp: %f\tki: %f\tkd: %f", kp, ki, kd);
		//ESP_LOGI("PID","pid_K: %f", pid_k);

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

	angle_x_degree = (RAD_TO_DEGREES_CONVERSION) * ( (0.0*((angle_x_degree/RAD_TO_DEGREES_CONVERSION) + (data_gyros_y * (SAMPLE_PERIOD_MS/1000)))) + (1*acos( data_accel_z/sqrt( (x_squared_flag) + (z_squared_flag)))) );
	if (data_accel_x < 0) angle_x_degree *= (-1);	// quando vai para negativo, data_accel_x fica negativo
}


/**************************************************************************
* 
*   Name: PID_Calculation_Task
*   Author: Giordano Cechet Moro
*   Date: 19/06/2019
*   Explanation: Calculo do valor do PID com aproximação de Tustin e filtro passa-baixas na parcela derivativa
*   
**************************************************************************/
void PID_Calculation_Task ( void *pvParameters )
{

	//static double_t pid_p_value = 0.0;
	//static double_t pid_i_value = 0.0;
	//static double_t pid_d_value = 0.0;
	static int8_t erro_k_m1 = 0;

	erro_k_m1 = erro_k;
	erro_k = angle_x_degree - ANGLE_SET_POINT_DEGREE;

	kp = (float_t)ADC_Value[P_ADC_INDEX]/100;
	ki = (float_t)ADC_Value[I_ADC_INDEX]/100;
	kd = (float_t)ADC_Value[D_ADC_INDEX]/100;

	//#if kp != 0
	// Controlador P
	pid_p_value = (double_t)( kp * erro_k );
	//#endif

	//#if ki != 0
	// Controlador I
	if ( ki > 0.01 ) {
		pid_i_value += (double_t)( ki * (((erro_k + erro_k_m1)/2) * (sample_rate_ms/1000)) );
		if ( pid_i_value > (10) ) pid_i_value = 10;
		else if ( pid_i_value < (-10) ) pid_i_value = (-10);
	} else pid_i_value = 0.0;
	//#endif

	//#if kd != 0
	// Controlador D
	pid_d_value = (double_t)( kd * ((erro_k - erro_k_m1)/(float_t)(sample_rate_ms/1000)) );
	//#endif

	//   Controlador PID
	pid_k = pid_p_value + pid_i_value + pid_d_value;

	pid_k = (-1)*pid_k + 190;	// 190 é o valor de "repouso" do motor, isto é, onde ele fica em 0° (+/-)
	
	if ( pid_k > UPPER_LIMIT_PID ) pid_k = UPPER_LIMIT_PID;
	else if ( pid_k < LOWER_LIMIT_PID ) pid_k = LOWER_LIMIT_PID;

	pid_k /= 1000;

}

/**************************************************************************
* 
*   Name: BLDC__ESC_Actuation
*   Author: Giordano Cechet Moro
*   Date: 25/05/2019
*   Explanation: 
*   
**************************************************************************/
void BLDC__ESC_Actuation()
{
	PWM__Set_Duty( (pid_k)*(FULL_SCALE_DUTY_CYCLE_PERCENT-INIT_SCALE_DUTY_CYCLE_PERCENT) + INIT_SCALE_DUTY_CYCLE_PERCENT );
}