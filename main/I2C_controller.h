#include "stdint.h"
#include "driver/i2c.h"

//------------------------------------------------------
// GPIO Configs
//------------------------------------------------------
#define SDA_GPIO	GPIO_NUM_21
#define SCLK_GPIO 	GPIO_NUM_22

//------------------------------------------------------
// MPU6050 Registers Adress and Value
//------------------------------------------------------
#define MPU6050__CONFIG_ADRESS				0x1A
#define MPU6050__CONFIG_VALUE				0b00000000//0b00000110		// digital low pass filter & external frame synchronization
#define MPU6050__GYRO_CONFIG_ADDRESS		0x1B
#define MPU6050__GYRO_CONFIG_VALUE			0b00000000		// fundo de escala de medição --> (250), 500, 1000, 2000 °/s . Quanto maior o valor, menor o "passo", maior a precisão.
#define MPU6050__ACCEL_CONFIG_ADDRESS		0x1C
#define MPU6050__ACCEL_CONFIG_VALUE			0b00000000		// fundo de escala de medição --> (+-2g), +-4g, +-8g, +-16g . Quanto maior o valor, menor o "passo", maior a precisão.
#define MPU6050__FIFO_EN_ADDRESS			0x23
#define MPU6050__FIFO_EN_VALUE				0b01111000		// larga no FIFO --> valores do acelerômetro e giroscópio vão para o FIFO
#define MPU6050__I2C_MST_CTRL_ADDRESS		0x24
#define MPU6050__I2C_MST_CTRL_VALUE			0b00001101
#define MPU6050__ACCEL_XOUT_MSB_ADDRESS		0x3B
#define MPU6050__ACCEL_XOUT_LSB_ADDRESS		0x3C
#define MPU6050__ACCEL_YOUT_MSB_ADDRESS		0x3D
#define MPU6050__ACCEL_YOUT_LSB_ADDRESS		0x3E
#define MPU6050__ACCEL_ZOUT_MSB_ADDRESS		0x3F
#define MPU6050__ACCEL_ZOUT_LSB_ADDRESS		0x40
#define MPU6050__TEMP_OUT_MSB_ADDRESS		0x41			// Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
#define MPU6050__TEMP_OUT_LSB_ADDRESS		0x42			// Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
#define MPU6050__GYRO_XOUT_MSB_ADDRESS		0x43
#define MPU6050__GYRO_XOUT_LSB_ADDRESS		0x44
#define MPU6050__GYRO_YOUT_MSB_ADDRESS		0x45
#define MPU6050__GYRO_YOUT_LSB_ADDRESS		0x46
#define MPU6050__GYRO_ZOUT_MSB_ADDRESS		0x47
#define MPU6050__GYRO_ZOUT_LSB_ADDRESS		0x48
#define MPU6050__SIGNAL_PATH_RESET_ADDRESS	0x68			// 
#define MPU6050__SIGNAL_PATH_RESET_VALUE	0b00000111		// Reseta os "digital signal path"(?) dos sensores internos (acel, giro, temp) --> parece um flush()
#define MPU6050__USER_CTRL_ADDRESS			0x6A
#define MPU6050__USER_CTRL_VALUE			0b00000000
#define MPU6050__PWR_MGMT_1_ADDRESS			0x6B
#define MPU6050__PWR_MGMT_1_VALUE			0b00001000		// REGISTRADOR INTERESSANTE! --> DEVICE_RESET, SLEEP, CYCLE, TEMP_DIS, CLK_SEL. Bit '1' está desabilitando sensor de temperatura
#define MPU6050__PWR_MGMT_2_ADDRESS			0x6C
#define MPU6050__PWR_MGMT_2_VALUE			0b11011011		// REGISTRADOR INTERESSANTE! --> LOW_POWER_WAKE_CONTROL, STANDBY_ACCELL, STANDBY_GYRO. Nesse caso, estou deixando habilitado apenas o eixo x do acel e do giro
#define MPU6050__AD0_PIN_VALUE				0
#define MPU6050__WHO_AM_I_ADDRESS			0x75			// endereço do registrador onde está contido o valor 0b0110100X = 0x68
#define MPU6050__WHO_AM_I_VALUE				( 0x68 | MPU6050__AD0_PIN_VALUE )

#define REGISTER_ONE_ADDRESS_TEST			0x6B	// registrador 107, contém valor 0x40
#define REGISTER_TWO_ADDRESS_TEST			0x75	// registrador 117, contém valor 0x68


//-------------------------------------------------------------------------
// Functions Headers
//-------------------------------------------------------------------------
void I2C__master_init();
void I2C__read_byte(uint8_t register_address_to_read, uint8_t *data);
void I2C__write_byte(uint8_t register_address_to_write, uint8_t *data);