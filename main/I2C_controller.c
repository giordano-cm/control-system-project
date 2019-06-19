#include "I2C_controller.h"

// Parte da API do I2C foi retirada do exemplo "esp-idf\examples\peripherals\i2c\i2c_self_test"

/**************************************************************************
* 
*   Name: I2C__master_init
*   Author: Giordano Cechet Moro
*   Date: 20/04/2019
*   Explanation: Inicialização e configs do ESP como Master
*   
**************************************************************************/
void I2C__master_init()
{

	i2c_config_t i2c_config = {
		.mode           	= I2C_MODE_MASTER,
		.sda_io_num     	= SDA_GPIO,
		.sda_pullup_en  	= GPIO_PULLUP_ENABLE,		// or GPIO_PULLUP_ENABLE
		.scl_io_num     	= SCLK_GPIO,
		.scl_pullup_en  	= GPIO_PULLUP_ENABLE,		// vou deixar DISABLE, porque o módulo acelerômetro já contém os resistores de pullup
		.master.clk_speed   = 100000,
		//.addr_10bit_en = ;							// só para quando o esp for slave
		//.slave_addr     = MPU6050__WHO_AM_I_VALUE;	// só para quando o esp for slave
	};

	i2c_param_config(I2C_NUM_0, &i2c_config);	// I2C_NUM_0 --> existem dois ports (2 drivers). Escolhi qualquer um.
	// a função acima seta as funções abaixo com parâmetros default
	//i2c_set_period();
	//i2c_set_start_timing();
	//i2c_set_stop_timing();
	//i2c_set_data_timing();
	//i2c_set_timeout();
	//i2c_set_data_mode();
	//i2c_set_pin();
	//I2C_SLAVE_SDA_SAMPLE_DEFAULT
	//I2C_SLAVE_SDA_HOLD_DEFAULT
	//I2C_MASTER_TOUT_CNUM_DEFAULT
	//I2C_SLAVE_TIMEOUT_DEFAULT

	i2c_driver_install(I2C_NUM_0, i2c_config.mode, 0, 0, 0);
    //i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);  // conforme exemplo
}

/**************************************************************************
* 
*   Name: I2C__read_byte
*   Author: Giordano Cechet Moro
*   Date: 20/04/2019
*   Explanation: Lê um único byte via protocolo I²C
*   
**************************************************************************/
void I2C__read_byte(uint8_t register_address_to_read, uint8_t *data)
{
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MPU6050__WHO_AM_I_VALUE << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);	// or ACK_CHECK_EN. Configura comando de escrita
	i2c_master_write_byte(cmd, register_address_to_read, I2C_MASTER_ACK);									// or ACK_CHECK_EN. Configura comando de escrita
	i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050__WHO_AM_I_VALUE << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, (uint8_t *)data, sizeof(*data), I2C_MASTER_NACK);  						// or ACK_CHECK_DIS. Configura comando de leitura
    i2c_master_stop(cmd);   																		// configura comando de stop

    i2c_master_cmd_begin(I2C_NUM_0, cmd, ( 1000 / portTICK_PERIOD_MS )); 							// envia todos os comandos acima
	i2c_cmd_link_delete(cmd);
}

/**************************************************************************
* 
*   Name: I2C__write_byte
*   Author: Giordano Cechet Moro
*   Date: 18/05/2019
*   Explanation: Escreve um único byte via protocolo I²C
*   
**************************************************************************/
void I2C__write_byte(uint8_t register_address_to_write, uint8_t *data)
{

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MPU6050__WHO_AM_I_VALUE << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, register_address_to_write, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, (*data), I2C_MASTER_ACK);
	i2c_master_stop(cmd);

	i2c_master_cmd_begin(I2C_NUM_0, cmd, ( 1000 / portTICK_PERIOD_MS ));
	i2c_cmd_link_delete(cmd);

}