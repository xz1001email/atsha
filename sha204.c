/*
 * 
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "sha204.h"
//#include <linux/mub.h>


//===============================================================================================================
//software_IIC.c

void SCL_out_low(void)
{
	//software_IIC_quarter_period();// period/4
	udelay(16);
	IIC_SCL(GPIO_OUT_LOW);
	udelay(16);
	//software_IIC_quarter_period();// period/4	
}

void SCL_out_high(void)
{
	udelay(16);
	//software_IIC_quarter_period();// period/4
	IIC_SCL(GPIO_OUT_HIGH);
	udelay(16);
	//software_IIC_quarter_period();// period/4	
}

//SDA  --------_______
//SCL  __---------_____
void software_IIC_start(void)
{
	SCL_out_low();//为了避免意外的启动或停止条件，一定要确保SCL时钟线为低。

	IIC_SDA(GPIO_OUT_HIGH);//先改变SDA 管脚的输出寄存器ODR，高电平
	SDA_OUT();//设置SDA 输出
	
	
	SCL_out_high();//设置SCL管脚为高电平
	
	IIC_SDA(GPIO_OUT_LOW);//SDA输出低电平
	SDA_OUT();//设置SDA 输出

	
	SCL_out_low(); //设置SCL管脚为低电平，SCL产生下降沿
}

//SDA_____------____
//SCL___------______
void software_IIC_stop(void)
{
	SCL_out_low();//为了避免意外的启动或停止条件，一定要确保SCL时钟线为低。
	
	IIC_SDA(GPIO_OUT_LOW);//先改变SDA 管脚的输出寄存器ODR，低电平
	SDA_OUT();//设置SDA 输出

	SCL_out_high();	//设置SCL管脚为高电平。SCL产生上降沿

	IIC_SDA(GPIO_OUT_HIGH);//SDA输出高电平	
	SDA_OUT();//设置SDA 输出	,SDA产生上降沿
	
}

bool software_IIC_read_ack()
{
	uint8_t error_times = 0;
	
	SCL_out_low();
	
	SDA_IN();
	
	SCL_out_high();	
	
	while(READ_SDA()==NACK)
	{
		if(error_times >= 4*20)
		{//max wait time = 16*period/4
			SCL_out_low();
			
			//IIC_SDA(GPIO_OUT_HIGH);//SDA恢复初始化状态输出高电平
			//SDA_OUT();//	
			return NACK;
		}
		error_times++;
		//udelay(16);
		//software_IIC_quarter_period();
	}
	
	SCL_out_low();
	
	//IIC_SDA(GPIO_OUT_HIGH);//SDA恢复初始化状态输出高电平
	//SDA_OUT();//	
	return ACK;
}

void software_IIC_send_ack_nack(bool ack_nack)
{
	SCL_out_low();
	
	if(NACK == ack_nack)
	{
		IIC_SDA(GPIO_OUT_HIGH);//SDA输出高电平
		SDA_OUT();//		
	}
	else
	{
		IIC_SDA(GPIO_OUT_LOW);//SDA输出低电平
		SDA_OUT();//
	}
	SCL_out_high();	
	SCL_out_low();
}

void software_IIC_send_byte(uint8_t data)
{
	uint8_t byte_bit_count = 0;
	
	for(byte_bit_count = 8; byte_bit_count > 0; byte_bit_count--)
	{
		SCL_out_low();
		
		if((data >> (byte_bit_count - 1)) & 0x01)
		{
			IIC_SDA(GPIO_OUT_HIGH);//SDA输出高电平
			SDA_OUT();//	
		}
		else
		{
			IIC_SDA(GPIO_OUT_LOW);//SDA输出低电平
			SDA_OUT();//
		}
		
		SCL_out_high();
	}
}

uint8_t software_IIC_read_byte(void)
{
	uint8_t byte_bit_count = 0;
	uint8_t data = 0;
	
	for(byte_bit_count = 0;byte_bit_count < 8;byte_bit_count++)
	{
		SCL_out_low();
		SDA_IN();
		SCL_out_high();
			
		data = data << 1;
		
		if(READ_SDA())
		{
			data++;
		}
	}
	
	SCL_out_low();	
	
	return data;
}


/** \brief This function sends bytes to an I2C device.
 * \param[in] count number of bytes to send
 * \param[in] data pointer to tx buffer
 * \return status of the operation
 */
uint8_t i2c_send_bytes(uint8_t count,uint8_t *data)
{
	uint8_t temp;
	uint8_t ack_nack;
	
	for(temp = 0;temp < count;temp++)
	{
		software_IIC_send_byte(*data++);
		ack_nack = software_IIC_read_ack();
		if(NACK ==ack_nack)
		{
			return I2C_FUNCTION_RETCODE_NACK;
		}
	}
	return I2C_FUNCTION_RETCODE_SUCCESS;
}

/** \brief This function receives one byte from an I2C device.
 *
 * \param[out] data pointer to received byte
 * \return status of the operation
 */
uint8_t i2c_receive_byte(uint8_t *data)
{
	*data = software_IIC_read_byte();
	software_IIC_send_ack_nack(ACK);
	
	return I2C_FUNCTION_RETCODE_SUCCESS;	
}

/** \brief This function receives bytes from an I2C device
 *         and sends a Stop.
 *
 * \param[in] count number of bytes to receive
 * \param[out] data pointer to rx buffer
 * \return status of the operation
 */
 uint8_t i2c_receive_bytes(uint8_t count,uint8_t *data)
{
	uint8_t temp = 0;

	// Acknowledge all bytes except the last one.
	for(temp = 0;temp < count -1;temp++)
	{
		*data++ = software_IIC_read_byte();
		software_IIC_send_ack_nack(ACK);
	}

	*data = software_IIC_read_byte();
	software_IIC_send_ack_nack(NACK);
	
	software_IIC_stop();
	
	return I2C_FUNCTION_RETCODE_SUCCESS;
}

//===============================================================================================================
//sha204_i2c.c
/** 
 * \brief This enumeration lists all packet types sent to a SHA204 device.
 *
 * The following byte stream is sent to a SHA204 TWI device:
 *    {I2C start} {I2C address} {word address} [{data}] {I2C stop}.
 * Data are only sent after a word address of value #SHA204_I2C_PACKET_FUNCTION_NORMAL.
 */
enum i2c_word_address {
	SHA204_I2C_PACKET_FUNCTION_RESET,  //!< Reset device.
	SHA204_I2C_PACKET_FUNCTION_SLEEP,  //!< Put device into Sleep mode.
	SHA204_I2C_PACKET_FUNCTION_IDLE,   //!< Put device into Idle mode.
	SHA204_I2C_PACKET_FUNCTION_NORMAL  //!< Write / evaluate data that follow this word address byte.
};


/** \brief This enumeration lists flags for I2C read or write addressing. */
enum i2c_read_write_flag {
	I2C_WRITE = (uint8_t) 0x00,  //!< write command flag
	I2C_READ  = (uint8_t) 0x01   //!< read command flag
};


//! I2C address can be changed by calling #sha204p_set_device_id.
//static uint8_t device_address = SHA204_I2C_DEFAULT_ADDRESS >> 1;
static uint8_t device_address = SHA204_I2C_DEFAULT_ADDRESS;

/** 
 * \brief This I2C function sets the I2C address.
 *         Communication functions will use this address.
 *
 *  \param[in] id I2C address
 */
void sha204p_set_device_id(uint8_t id)
{
	//device_address = id >> 1;
	device_address = id;	
}


/** 
 * \brief This I2C function generates a Wake-up pulse and delays.
 * \return status of the operation
 */
uint8_t sha204p_wakeup(void)
{
	uint8_t dummy_byte = 0;
	
	software_IIC_start();
	
	//udelay(10 * SHA204_WAKEUP_PULSE_WIDTH);
	//delay_10us(SHA204_WAKEUP_PULSE_WIDTH);//60us

	(void)i2c_send_bytes(1,&dummy_byte);

	software_IIC_stop();

	mdelay(SHA204_WAKEUP_DELAY);
	//delay_ms(SHA204_WAKEUP_DELAY);

	return SHA204_SUCCESS;
}

uint8_t sha204p_send_slave_address(uint8_t read_write)
{
	uint8_t slave = device_address | read_write;
	uint8_t ret_code = I2C_FUNCTION_RETCODE_SUCCESS;
	software_IIC_start();

	ret_code = i2c_send_bytes(1,&slave);
	if(ret_code != I2C_FUNCTION_RETCODE_SUCCESS)
	{
		software_IIC_stop();
	}

	return ret_code;	
}


/** 
 * \brief This function sends a I2C packet enclosed by a I2C start and stop to a SHA204 device.
 *
 *         This function combines a I2C packet send sequence that is common to all packet types.
 *         Only if word_address is \ref SHA204_I2C_PACKET_FUNCTION_NORMAL, count and buffer parameters are
 *         expected to be non-zero.
 * \param[in] word_address packet function code listed in #i2c_word_address
 * \param[in] count number of bytes in data buffer
 * \param[in] buffer pointer to data buffer
 * \return status of the operation
 */
static uint8_t sha204p_send(uint8_t word_address, uint8_t count, uint8_t *buffer)
{
	uint8_t i2c_status = sha204p_send_slave_address(I2C_WRITE);
	if(i2c_status != I2C_FUNCTION_RETCODE_SUCCESS)
	{
		software_IIC_stop();
		return SHA204_COMM_FAIL;
	}

	i2c_status = i2c_send_bytes(1,&word_address);
	if(i2c_status != I2C_FUNCTION_RETCODE_SUCCESS)
	{
		return SHA204_COMM_FAIL;
	}	

	if(0 == count)
	{	// We are done for packets that are not commands (Sleep, Idle, Reset).
		software_IIC_stop();
		return SHA204_SUCCESS;
	}

	i2c_status = i2c_send_bytes(count,buffer);
	
	software_IIC_stop();
	
	if(i2c_status != I2C_FUNCTION_RETCODE_SUCCESS)
	{
		return SHA204_COMM_FAIL;
	}	

	
	return SHA204_SUCCESS;
}

/** 
 * \brief This I2C function sends a command to the device.
 * \param[in] count number of bytes to send
 * \param[in] command pointer to command buffer
 * \return status of the operation
 */
uint8_t sha204p_send_command(uint8_t count, uint8_t *command)
{
	return sha204p_send(SHA204_I2C_PACKET_FUNCTION_NORMAL, count, command);
}


/** 
 * \brief This I2C function puts the SHA204 device into idle state.
 * \return status of the operation
 */
uint8_t sha204p_idle(void)
{
	return sha204p_send(SHA204_I2C_PACKET_FUNCTION_IDLE, 0, NULL);
}


/** 
 * \brief This I2C function puts the SHA204 device into low-power state.
 * \return status of the operation
 */
uint8_t sha204p_sleep(void)
{
	return sha204p_send(SHA204_I2C_PACKET_FUNCTION_SLEEP, 0, NULL);
}


/** 
 * \brief This I2C function resets the I/O buffer of the SHA204 device.
 * \return status of the operation
 */
uint8_t sha204p_reset_io(void)
{
	return sha204p_send(SHA204_I2C_PACKET_FUNCTION_RESET, 0, NULL);
}

/** 
 * \brief This I2C function receives a response from the SHA204 device.
 *
 * \param[in] size size of receive buffer
 * \param[out] response pointer to receive buffer
 * \return status of the operation
 */
uint8_t sha204p_receive_response(uint8_t size, uint8_t *response)
{
	uint8_t count = 0;
	// Address the device and indicate that bytes are to be read.
	uint8_t i2c_status = sha204p_send_slave_address(I2C_READ);
	if(i2c_status != I2C_FUNCTION_RETCODE_SUCCESS)
	{
		return SHA204_COMM_FAIL;
	}

	// Receive count byte.
	i2c_status = i2c_receive_byte(response);
	if(i2c_status != I2C_FUNCTION_RETCODE_SUCCESS)
	{
		return SHA204_COMM_FAIL;
	}

	count = response[SHA204_BUFFER_POS_COUNT];

	if ((count < SHA204_RSP_SIZE_MIN) || (count > size))
	{
		return SHA204_INVALID_SIZE;
	}

	i2c_status = i2c_receive_bytes(count - 1, &response[SHA204_BUFFER_POS_DATA]);
	if (i2c_status != I2C_FUNCTION_RETCODE_SUCCESS)
	{
		return SHA204_COMM_FAIL;
	}
	else
	{
		return SHA204_SUCCESS;	
	}
}

/** 
 * \brief This I2C function resynchronizes communication.
 *
 * Parameters are not used for I2C.\n
 * Re-synchronizing communication is done in a maximum of three steps
 * listed below. This function implements the first step. Since
 * steps 2 and 3 (sending a Wake-up token and reading the response)
 * are the same for I2C and SWI, they are
 * implemented in the communication layer (\ref sha204c_resync).
 * See the excerpt from the SHA204 data sheet below.
  <ol>
     <li>
       To ensure an IO channel reset, the system should send
       the standard I2C software reset sequence, as follows:
       <ul>
         <li>a Start condition</li>
         <li>nine cycles of SCL, with SDA held high</li>
         <li>another Start condition</li>
         <li>a Stop condition</li>
       </ul>
       It should then be possible to send a read sequence and
       if synchronization has completed properly the ATSHA204 will
       acknowledge the device address. The chip may return data or
       may leave the bus floating (which the system will interpret
       as a data value of 0xFF) during the data periods.\n
       If the chip does acknowledge the device address, the system
       should reset the internal address counter to force the
       ATSHA204 to ignore any partial input command that may have
       been sent. This can be accomplished by sending a write
       sequence to word address 0x00 (Reset), followed by a
       Stop condition.
     </li>
     <li>
       If the chip does NOT respond to the device address with an ACK,
       then it may be asleep. In this case, the system should send a
       complete Wake token and wait t_whi after the rising edge. The
       system may then send another read sequence and if synchronization
       has completed the chip will acknowledge the device address.
     </li>
     <li>
       If the chip still does not respond to the device address with
       an acknowledge, then it may be busy executing a command. The
       system should wait the longest TEXEC and then send the
       read sequence, which will be acknowledged by the chip.
     </li>
  </ol>
 * \param[in] size size of response buffer
 * \param[out] response pointer to response buffer
 * \return status of the operation
 */
uint8_t sha204p_resync(uint8_t size, uint8_t *response)
{
	// Generate Start, nine clocks, Stop.
	// (Adding a Repeat Start before the Stop would additionally
	// prevent erroneously writing a byte, but a Stop right after a
	// Start is not "legal" for I2C and the SHA204 will not write
	// anything without a successful CRC check.)
	
	uint8_t nine_clocks = 0xFF;
	uint8_t ret_code;

	software_IIC_start();

	// Do not evaluate the return code that most likely indicates error,
	// since nine_clocks is unlikely to be acknowledged.
	(void) i2c_send_bytes(1, &nine_clocks);

	// Send another Start. The function sends also one byte,
	// the TWI address of the device, because TWI specification
	// does not allow sending a Stop right after a Start condition.
	ret_code = sha204p_send_slave_address(I2C_READ);

	// Send only a Stop if the above call succeeded.
	// Otherwise the above function has sent it already.
	if (ret_code == I2C_FUNCTION_RETCODE_SUCCESS)
	{
		software_IIC_stop();
	}

	// Return error status if we failed to re-sync.
	if (ret_code != I2C_FUNCTION_RETCODE_SUCCESS)
		return SHA204_COMM_FAIL;

	// Try to send a Reset IO command if re-sync succeeded.
	return sha204p_reset_io();
}

//============================================================================================================
//sha204_command_marshaling.c
/** \brief This function checks the parameters for sha204m_execute().
 *
 * \param[in, out] args pointer to parameter structure
 * \return status of the operation
 */
static uint8_t sha204m_check_parameters(struct sha204_command_parameters *args)
{
#ifdef SHA204_CHECK_PARAMETERS

	uint8_t len = args->data_len_1 + args->data_len_2 + args->data_len_3 + SHA204_CMD_SIZE_MIN;
	if (!args->tx_buffer || args->tx_size < len || args->rx_size < SHA204_RSP_SIZE_MIN || !args->rx_buffer)
		return SHA204_BAD_PARAM;

	if ((args->data_len_1 > 0 && !args->data_1) || (args->data_len_2 > 0 && !args->data_2) || (args->data_len_3 > 0 && !args->data_3))
		return SHA204_BAD_PARAM;

	// Check parameters depending on op-code.
	switch (args->op_code) {
	case SHA204_CHECKMAC:
		if (
				// no null pointers allowed
				!args->data_1 || !args->data_2
				// No reserved bits should be set.
				|| (args->param_1 | CHECKMAC_MODE_MASK) != CHECKMAC_MODE_MASK
				// key_id > 15 not allowed
				|| args->param_2 > SHA204_KEY_ID_MAX
			)
			return SHA204_BAD_PARAM;
		break;

	case SHA204_DERIVE_KEY:
		if (((args->param_1 & ~DERIVE_KEY_RANDOM_FLAG) != 0)
					 || (args->param_2 > SHA204_KEY_ID_MAX))
			return SHA204_BAD_PARAM;
		break;

	case SHA204_DEVREV:
		break;

	case SHA204_GENDIG:
		if ((args->param_1 != GENDIG_ZONE_OTP) && (args->param_1 != GENDIG_ZONE_DATA))
			return SHA204_BAD_PARAM;
		break;

	case SHA204_HMAC:
		if ((args->param_1 & ~HMAC_MODE_MASK) != 0)
			return SHA204_BAD_PARAM;
		break;

	case SHA204_LOCK:
		if (((args->param_1 & ~LOCK_ZONE_MASK) != 0)
					|| ((args->param_1 & LOCK_ZONE_NO_CRC) && (args->param_2 != 0)))
			return SHA204_BAD_PARAM;
		break;

	case SHA204_MAC:
		if (((args->param_1 & ~MAC_MODE_MASK) != 0)
					|| (((args->param_1 & MAC_MODE_BLOCK2_TEMPKEY) == 0) && !args->data_1))
			return SHA204_BAD_PARAM;
		break;

	case SHA204_NONCE:
		if (!args->data_1
				|| (args->param_1 > NONCE_MODE_PASSTHROUGH)
				|| (args->param_1 == NONCE_MODE_INVALID)
			)
			return SHA204_BAD_PARAM;
		break;

	case SHA204_PAUSE:
		break;

	case SHA204_RANDOM:
		if (args->param_1 > RANDOM_NO_SEED_UPDATE)
			return SHA204_BAD_PARAM;
		break;

	case SHA204_READ:
		if (((args->param_1 & ~READ_ZONE_MASK) != 0)
					|| ((args->param_1 & READ_ZONE_MODE_32_BYTES) && (args->param_1 == SHA204_ZONE_OTP)))
			return SHA204_BAD_PARAM;
		break;

	case SHA204_UPDATE_EXTRA:
		if (args->param_1 > UPDATE_CONFIG_BYTE_86)
			return SHA204_BAD_PARAM;
		break;

	case SHA204_WRITE:
		if (!args->data_1 || ((args->param_1 & ~WRITE_ZONE_MASK) != 0))
			return SHA204_BAD_PARAM;
		break;

	default:
		// unknown op-code
		return SHA204_BAD_PARAM;
	}

	return SHA204_SUCCESS;

#else
	return SHA204_SUCCESS;
#endif
}


/** \brief This function creates a command packet, sends it, and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_execute(struct sha204_command_parameters *args)
{
	uint8_t *p_buffer;
	uint8_t len;
	struct sha204_send_and_receive_parameters comm_parameters;
	uint8_t ret_code = sha204m_check_parameters(args);
	
	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;	
	
	if (ret_code != SHA204_SUCCESS)
		return ret_code;

	// Supply delays and response size.
	switch (args->op_code) {
	case SHA204_CHECKMAC:
		comm_parameters.poll_delay = CHECKMAC_DELAY;
		comm_parameters.poll_timeout = CHECKMAC_EXEC_MAX - CHECKMAC_DELAY;
		comm_parameters.rx_size = CHECKMAC_RSP_SIZE;
		break;

	case SHA204_DERIVE_KEY:
		comm_parameters.poll_delay = DERIVE_KEY_DELAY;
		comm_parameters.poll_timeout = DERIVE_KEY_EXEC_MAX - DERIVE_KEY_DELAY;
		comm_parameters.rx_size = DERIVE_KEY_RSP_SIZE;
		break;

	case SHA204_DEVREV:
		comm_parameters.poll_delay = DEVREV_DELAY;
		comm_parameters.poll_timeout = DEVREV_EXEC_MAX - DEVREV_DELAY;
		comm_parameters.rx_size = DEVREV_RSP_SIZE;
		break;

	case SHA204_GENDIG:
		comm_parameters.poll_delay = GENDIG_DELAY;
		comm_parameters.poll_timeout = GENDIG_EXEC_MAX - GENDIG_DELAY;
		comm_parameters.rx_size = GENDIG_RSP_SIZE;
		break;

	case SHA204_HMAC:
		comm_parameters.poll_delay = HMAC_DELAY;
		comm_parameters.poll_timeout = HMAC_EXEC_MAX - HMAC_DELAY;
		comm_parameters.rx_size = HMAC_RSP_SIZE;
		break;

	case SHA204_LOCK:
		comm_parameters.poll_delay = LOCK_DELAY;
		comm_parameters.poll_timeout = LOCK_EXEC_MAX - LOCK_DELAY;
		comm_parameters.rx_size = LOCK_RSP_SIZE;
		break;

	case SHA204_MAC:
		comm_parameters.poll_delay = MAC_DELAY;
		comm_parameters.poll_timeout = MAC_EXEC_MAX - MAC_DELAY;
		comm_parameters.rx_size = MAC_RSP_SIZE;
		break;

	case SHA204_NONCE:
		comm_parameters.poll_delay = NONCE_DELAY;
		comm_parameters.poll_timeout = NONCE_EXEC_MAX - NONCE_DELAY;
		comm_parameters.rx_size = args->param_1 == NONCE_MODE_PASSTHROUGH
							? NONCE_RSP_SIZE_SHORT : NONCE_RSP_SIZE_LONG;
		break;

	case SHA204_PAUSE:
		comm_parameters.poll_delay = PAUSE_DELAY;
		comm_parameters.poll_timeout = PAUSE_EXEC_MAX - PAUSE_DELAY;
		comm_parameters.rx_size = PAUSE_RSP_SIZE;
		break;

	case SHA204_RANDOM:
		comm_parameters.poll_delay = RANDOM_DELAY;
		comm_parameters.poll_timeout = RANDOM_EXEC_MAX - RANDOM_DELAY;
		comm_parameters.rx_size = RANDOM_RSP_SIZE;
		break;

	case SHA204_READ:
		comm_parameters.poll_delay = READ_DELAY;
		comm_parameters.poll_timeout = READ_EXEC_MAX - READ_DELAY;
		comm_parameters.rx_size = (args->param_1 & SHA204_ZONE_COUNT_FLAG)
							? READ_32_RSP_SIZE : READ_4_RSP_SIZE;
		break;

	case SHA204_UPDATE_EXTRA:
		comm_parameters.poll_delay = UPDATE_DELAY;
		comm_parameters.poll_timeout = UPDATE_EXEC_MAX - UPDATE_DELAY;
		comm_parameters.rx_size = UPDATE_RSP_SIZE;
		break;

	case SHA204_WRITE:
		comm_parameters.poll_delay = WRITE_DELAY;
		comm_parameters.poll_timeout = WRITE_EXEC_MAX - WRITE_DELAY;
		comm_parameters.rx_size = WRITE_RSP_SIZE;
		break;

	default:
		comm_parameters.poll_delay = 0;
		comm_parameters.poll_timeout = SHA204_COMMAND_EXEC_MAX;
		comm_parameters.rx_size = args->rx_size;
	}

	// Assemble command.
	len = args->data_len_1 + args->data_len_2 + args->data_len_3 + SHA204_CMD_SIZE_MIN;
	p_buffer = args->tx_buffer;
	*p_buffer++ = len;
	*p_buffer++ = args->op_code;
	*p_buffer++ = args->param_1;
	*p_buffer++ = args->param_2 & 0xFF;
	*p_buffer++ = args->param_2 >> 8;

	if (args->data_len_1 > 0) {
		memcpy(p_buffer, args->data_1, args->data_len_1);
		p_buffer += args->data_len_1;
	}
	if (args->data_len_2 > 0) {
		memcpy(p_buffer, args->data_2, args->data_len_2);
		p_buffer += args->data_len_2;
	}
	if (args->data_len_3 > 0) {
		memcpy(p_buffer, args->data_3, args->data_len_3);
		p_buffer += args->data_len_3;
	}

	sha204c_calculate_crc(len - SHA204_CRC_SIZE, args->tx_buffer, p_buffer);

	// Send command and receive response.
	return sha204c_send_and_receive(&comm_parameters);
}


/** \brief This function sends a CheckMAC command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_check_mac(struct sha204_check_mac_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;
	if (		// no null pointers allowed
				!args->tx_buffer || !args->rx_buffer || !args->client_response || !args->other_data
				// No reserved bits should be set.
				|| (args->mode | CHECKMAC_MODE_MASK) != CHECKMAC_MODE_MASK
				// key_id > 15 not allowed
				|| args->key_id > SHA204_KEY_ID_MAX)
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_COUNT_IDX] = CHECKMAC_COUNT;
	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_CHECKMAC;
	args->tx_buffer[CHECKMAC_MODE_IDX] = args->mode & CHECKMAC_MODE_MASK;
	args->tx_buffer[CHECKMAC_KEYID_IDX]= args->key_id;
	args->tx_buffer[CHECKMAC_KEYID_IDX + 1] = 0;
	if (args->client_challenge == NULL)
		memset(&args->tx_buffer[CHECKMAC_CLIENT_CHALLENGE_IDX], 0, CHECKMAC_CLIENT_CHALLENGE_SIZE);
	else
		memcpy(&args->tx_buffer[CHECKMAC_CLIENT_CHALLENGE_IDX], args->client_challenge, CHECKMAC_CLIENT_CHALLENGE_SIZE);

	memcpy(&args->tx_buffer[CHECKMAC_CLIENT_RESPONSE_IDX], args->client_response, CHECKMAC_CLIENT_RESPONSE_SIZE);
	memcpy(&args->tx_buffer[CHECKMAC_DATA_IDX], args->other_data, CHECKMAC_OTHER_DATA_SIZE);

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = CHECKMAC_RSP_SIZE;
	comm_parameters.poll_delay = CHECKMAC_DELAY;
	comm_parameters.poll_timeout = CHECKMAC_EXEC_MAX - CHECKMAC_DELAY;
	
	return sha204c_send_and_receive(&comm_parameters);
}


/** \brief This function sends a DeriveKey command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_derive_key(struct sha204_derive_key_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;

	if (!args->tx_buffer || !args->rx_buffer || ((args->use_random & ~DERIVE_KEY_RANDOM_FLAG) != 0)
				 || (args->target_key > SHA204_KEY_ID_MAX))
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_DERIVE_KEY;
	args->tx_buffer[DERIVE_KEY_RANDOM_IDX] = args->use_random;
	args->tx_buffer[DERIVE_KEY_TARGETKEY_IDX] = args->target_key;
	args->tx_buffer[DERIVE_KEY_TARGETKEY_IDX + 1] = 0;
	if (args->mac != NULL)
	{
		memcpy(&args->tx_buffer[DERIVE_KEY_MAC_IDX], args->mac, DERIVE_KEY_MAC_SIZE);
		args->tx_buffer[SHA204_COUNT_IDX] = DERIVE_KEY_COUNT_LARGE;
	}
	else
		args->tx_buffer[SHA204_COUNT_IDX] = DERIVE_KEY_COUNT_SMALL;

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = DERIVE_KEY_RSP_SIZE;
	comm_parameters.poll_delay = DERIVE_KEY_DELAY;
	comm_parameters.poll_timeout = DERIVE_KEY_EXEC_MAX - DERIVE_KEY_DELAY;
	
	return sha204c_send_and_receive(&comm_parameters);
}

/** \brief This function sends a DevRev command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_dev_rev(struct sha204_dev_rev_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;

	if (!args->tx_buffer || !args->rx_buffer)
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_COUNT_IDX] = DEVREV_COUNT;
	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_DEVREV;

	// Parameters are 0.
	args->tx_buffer[DEVREV_PARAM1_IDX] =
	args->tx_buffer[DEVREV_PARAM2_IDX] =
	args->tx_buffer[DEVREV_PARAM2_IDX + 1] = 0;

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = DEVREV_RSP_SIZE;
	comm_parameters.poll_delay = DEVREV_DELAY;
	comm_parameters.poll_timeout = DEVREV_EXEC_MAX - DEVREV_DELAY;

	return sha204c_send_and_receive(&comm_parameters);
}

/** \brief This function sends a GenDig command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_gen_dig(struct sha204_gen_dig_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;

	if (!args->tx_buffer || !args->rx_buffer
				|| ((args->zone != GENDIG_ZONE_OTP) && (args->zone != GENDIG_ZONE_DATA)))
		return SHA204_BAD_PARAM;

	if (((args->zone == GENDIG_ZONE_OTP) && (args->key_id > SHA204_OTP_BLOCK_MAX))
				|| ((args->zone == GENDIG_ZONE_DATA) && (args->key_id > SHA204_KEY_ID_MAX)))
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_GENDIG;
	args->tx_buffer[GENDIG_ZONE_IDX] = args->zone;
	args->tx_buffer[GENDIG_KEYID_IDX] = args->key_id;
	args->tx_buffer[GENDIG_KEYID_IDX + 1] = 0;
	if (args->other_data != NULL)
	{
		memcpy(&args->tx_buffer[GENDIG_DATA_IDX], args->other_data, GENDIG_OTHER_DATA_SIZE);
		args->tx_buffer[SHA204_COUNT_IDX] = GENDIG_COUNT_DATA;
	}
	else
		args->tx_buffer[SHA204_COUNT_IDX] = GENDIG_COUNT;


	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = GENDIG_RSP_SIZE;
	comm_parameters.poll_delay = GENDIG_DELAY;
	comm_parameters.poll_timeout = GENDIG_EXEC_MAX - GENDIG_DELAY;

	return sha204c_send_and_receive(&comm_parameters);
}

/** \brief This function sends an HMAC command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_hmac(struct sha204_hmac_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;

	if (!args->tx_buffer || !args->rx_buffer || ((args->mode & ~HMAC_MODE_MASK) != 0))
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_COUNT_IDX] = HMAC_COUNT;
	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_HMAC;
	args->tx_buffer[HMAC_MODE_IDX] = args->mode;

	// Although valid key identifiers are only
	// from 0 to 15, all 16 bits are used in the HMAC message.
	args->tx_buffer[HMAC_KEYID_IDX] = args->key_id & 0xFF;
	args->tx_buffer[HMAC_KEYID_IDX + 1] = args->key_id >> 8;


	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = HMAC_RSP_SIZE;
	comm_parameters.poll_delay = HMAC_DELAY;
	comm_parameters.poll_timeout = HMAC_EXEC_MAX - HMAC_DELAY;
		
	return sha204c_send_and_receive(&comm_parameters);
}

/** \brief This function sends a Lock command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_lock(struct sha204_lock_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;

	if (!args->tx_buffer || !args->rx_buffer || ((args->zone & ~LOCK_ZONE_MASK) != 0)
				|| ((args->zone & LOCK_ZONE_NO_CRC) && (args->summary != 0)))
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_COUNT_IDX] = LOCK_COUNT;
	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_LOCK;
	args->tx_buffer[LOCK_ZONE_IDX] = args->zone & LOCK_ZONE_MASK;
	args->tx_buffer[LOCK_SUMMARY_IDX]= args->summary & 0xFF;
	args->tx_buffer[LOCK_SUMMARY_IDX + 1]= args->summary >> 8;

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = LOCK_RSP_SIZE;
	comm_parameters.poll_delay = LOCK_DELAY;
	comm_parameters.poll_timeout = LOCK_EXEC_MAX - LOCK_DELAY;
	
	return sha204c_send_and_receive(&comm_parameters);
}

/** \brief This function sends a MAC command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_mac(struct sha204_mac_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;

	if (!args->tx_buffer || !args->rx_buffer || ((args->mode & ~MAC_MODE_MASK) != 0)
				|| (((args->mode & MAC_MODE_BLOCK2_TEMPKEY) == 0) && !args->challenge))
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_COUNT_IDX] = MAC_COUNT_SHORT;
	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_MAC;
	args->tx_buffer[MAC_MODE_IDX] = args->mode;
	args->tx_buffer[MAC_KEYID_IDX] = args->key_id & 0xFF;
	args->tx_buffer[MAC_KEYID_IDX + 1] = args->key_id >> 8;
	if ((args->mode & MAC_MODE_BLOCK2_TEMPKEY) == 0)
	{
		memcpy(&args->tx_buffer[MAC_CHALLENGE_IDX], args->challenge, MAC_CHALLENGE_SIZE);
		args->tx_buffer[SHA204_COUNT_IDX] = MAC_COUNT_LONG;
	}

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = MAC_RSP_SIZE;
	comm_parameters.poll_delay = MAC_DELAY;
	comm_parameters.poll_timeout = MAC_EXEC_MAX - MAC_DELAY;
	
	return sha204c_send_and_receive(&comm_parameters);
}

/** \brief This function sends a Nonce command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_nonce(struct sha204_nonce_parameters *args)
{
	uint8_t rx_size;
	struct sha204_send_and_receive_parameters comm_parameters;	

	if (!args->tx_buffer || !args->rx_buffer || !args->num_in
				|| (args->mode > NONCE_MODE_PASSTHROUGH) || (args->mode == NONCE_MODE_INVALID))
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_NONCE;
	args->tx_buffer[NONCE_MODE_IDX] = args->mode;

	// 2. parameter is 0.
	args->tx_buffer[NONCE_PARAM2_IDX] =
	args->tx_buffer[NONCE_PARAM2_IDX + 1] = 0;

	if (args->mode != NONCE_MODE_PASSTHROUGH)
	{
		memcpy(&args->tx_buffer[NONCE_INPUT_IDX], args->num_in, NONCE_NUMIN_SIZE);
		args->tx_buffer[SHA204_COUNT_IDX] = NONCE_COUNT_SHORT;
		rx_size = NONCE_RSP_SIZE_LONG;
	}
	else
	{
		memcpy(&args->tx_buffer[NONCE_INPUT_IDX], args->num_in, NONCE_NUMIN_SIZE_PASSTHROUGH);
		args->tx_buffer[SHA204_COUNT_IDX] = NONCE_COUNT_LONG;
		rx_size = NONCE_RSP_SIZE_SHORT;
	}

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = rx_size;
	comm_parameters.poll_delay = NONCE_DELAY;
	comm_parameters.poll_timeout = NONCE_EXEC_MAX - NONCE_DELAY;

	return sha204c_send_and_receive(&comm_parameters);
}

/** \brief This function sends a Pause command to SWI devices and receives a response from the selected device.
 *         All others pause.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_pause(struct sha204_pause_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;

	if (!args->tx_buffer || !args->rx_buffer)
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_COUNT_IDX] = PAUSE_COUNT;
	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_PAUSE;
	args->tx_buffer[PAUSE_SELECT_IDX] = args->selector;

	// 2. parameter is 0.
	args->tx_buffer[PAUSE_PARAM2_IDX] =
	args->tx_buffer[PAUSE_PARAM2_IDX + 1] = 0;

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = PAUSE_RSP_SIZE;
	comm_parameters.poll_delay = PAUSE_DELAY;
	comm_parameters.poll_timeout = PAUSE_EXEC_MAX - PAUSE_DELAY;

	return sha204c_send_and_receive(&comm_parameters);
}

/** \brief This function sends a Random command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_random(struct sha204_random_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;

	if (!args->tx_buffer || !args->rx_buffer || (args->mode > RANDOM_NO_SEED_UPDATE))
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_COUNT_IDX] = RANDOM_COUNT;
	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_RANDOM;
	args->tx_buffer[RANDOM_MODE_IDX] = args->mode & RANDOM_SEED_UPDATE;

	// 2. parameter is 0.
	args->tx_buffer[RANDOM_PARAM2_IDX] =
	args->tx_buffer[RANDOM_PARAM2_IDX + 1] = 0;

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = RANDOM_RSP_SIZE;
	comm_parameters.poll_delay = RANDOM_DELAY;
	comm_parameters.poll_timeout = RANDOM_EXEC_MAX - RANDOM_DELAY;
	
	return sha204c_send_and_receive(&comm_parameters);
}

/** \brief This function sends a Read command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_read(struct sha204_read_parameters *args)
{
	uint8_t rx_size;
	uint16_t address;

	struct sha204_send_and_receive_parameters comm_parameters;


	if (!args->tx_buffer || !args->rx_buffer || ((args->zone & ~READ_ZONE_MASK) != 0)
				|| ((args->zone & READ_ZONE_MODE_32_BYTES) && (args->zone == SHA204_ZONE_OTP)))
		return SHA204_BAD_PARAM;

	// If we would just mask address bits, we would
	// read from an address that was not intended.
	address = args->address >> 2;
	if ((args->zone & SHA204_ZONE_MASK) == SHA204_ZONE_CONFIG) {
		if (address > SHA204_ADDRESS_MASK_CONFIG)
			return SHA204_BAD_PARAM;
	}		
	if ((args->zone & SHA204_ZONE_MASK) == SHA204_ZONE_OTP) {
		if (address > SHA204_ADDRESS_MASK_OTP)
			// If we would just mask this bit, we would
			// read from an address that was not intended.
			return SHA204_BAD_PARAM;
	}		
	if ((args->zone & SHA204_ZONE_MASK) == SHA204_ZONE_DATA) {
		if (address > SHA204_ADDRESS_MASK)
			// If we would just mask this bit, we would
			// read from an address that was not intended.
			return SHA204_BAD_PARAM;
	}

	args->tx_buffer[SHA204_COUNT_IDX] = READ_COUNT;
	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_READ;
	args->tx_buffer[READ_ZONE_IDX] = args->zone;
	args->tx_buffer[READ_ADDR_IDX] = (uint8_t) address;
	args->tx_buffer[READ_ADDR_IDX + 1] = 0;

	rx_size = (args->zone & SHA204_ZONE_COUNT_FLAG) ? READ_32_RSP_SIZE : READ_4_RSP_SIZE;

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = rx_size;
	comm_parameters.poll_delay = READ_DELAY;
	comm_parameters.poll_timeout = READ_EXEC_MAX - READ_DELAY;
	
	return sha204c_send_and_receive(&comm_parameters);
}


/** \brief This function sends an UpdateExtra command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_update_extra(struct sha204_update_extra_parameters *args)
{
	struct sha204_send_and_receive_parameters comm_parameters;

	if (!args->tx_buffer || !args->rx_buffer || (args->mode > UPDATE_CONFIG_BYTE_86))
		return SHA204_BAD_PARAM;

	args->tx_buffer[SHA204_COUNT_IDX] = UPDATE_COUNT;
	args->tx_buffer[SHA204_OPCODE_IDX] = SHA204_UPDATE_EXTRA;
	args->tx_buffer[UPDATE_MODE_IDX] = args->mode;
	args->tx_buffer[UPDATE_VALUE_IDX] = args->new_value;
	args->tx_buffer[UPDATE_VALUE_IDX + 1] = 0;

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = UPDATE_RSP_SIZE;
	comm_parameters.poll_delay = UPDATE_DELAY;
	comm_parameters.poll_timeout = UPDATE_EXEC_MAX - UPDATE_DELAY;
		
	return sha204c_send_and_receive(&comm_parameters);
}


/**\brief This function sends a Write command to the device and receives its response.
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204m_write(struct sha204_write_parameters *args)
{
	uint8_t *p_command;
	uint8_t count;
	uint16_t address;

	struct sha204_send_and_receive_parameters comm_parameters;
	

	if (!args->tx_buffer || !args->rx_buffer || !args->new_value || ((args->zone & ~WRITE_ZONE_MASK) != 0))
		return SHA204_BAD_PARAM;

	// If we would just mask address bits, we would
	// read from an address that was not intended.
	address = args->address >> 2;
	if ((args->zone & SHA204_ZONE_MASK) == SHA204_ZONE_CONFIG) {
		if (address > SHA204_ADDRESS_MASK_CONFIG)
			return SHA204_BAD_PARAM;
	}		
	if ((args->zone & SHA204_ZONE_MASK) == SHA204_ZONE_OTP) {
		if (address > SHA204_ADDRESS_MASK_OTP)
			// If we would just mask this bit, we would
			// read from an address that was not intended.
			return SHA204_BAD_PARAM;
	}		
	if ((args->zone & SHA204_ZONE_MASK) == SHA204_ZONE_DATA) {
		if (address > SHA204_ADDRESS_MASK)
			// If we would just mask this bit, we would
			// read from an address that was not intended.
			return SHA204_BAD_PARAM;
	}

	p_command = &args->tx_buffer[SHA204_OPCODE_IDX];
	*p_command++ = SHA204_WRITE;
	*p_command++ = args->zone;
	*p_command++ = (uint8_t) address;
	*p_command++ = 0;

	count = (args->zone & SHA204_ZONE_COUNT_FLAG) ? SHA204_ZONE_ACCESS_32 : SHA204_ZONE_ACCESS_4;
	memcpy(p_command, args->new_value, count);
	p_command += count;

	if (args->mac != NULL)
	{
		memcpy(p_command, args->mac, WRITE_MAC_SIZE);
		p_command += WRITE_MAC_SIZE;
	}

	// Supply count.
	args->tx_buffer[SHA204_COUNT_IDX] = (uint8_t) (p_command - &args->tx_buffer[0] + SHA204_CRC_SIZE);

	comm_parameters.tx_buffer = args->tx_buffer;
	comm_parameters.rx_buffer = args->rx_buffer;
	comm_parameters.rx_size = WRITE_RSP_SIZE;
	comm_parameters.poll_delay = WRITE_DELAY;
	comm_parameters.poll_timeout = WRITE_EXEC_MAX - WRITE_DELAY;
		
	return sha204c_send_and_receive(&comm_parameters);
}


//=============================================================================================================
//sha_comm.c
uint8_t sha204c_check_crc(uint8_t *response);
uint8_t sha204c_resync(uint8_t size, uint8_t *response);

/** \brief This function calculates CRC.
 *
 * \param[in] length number of bytes in buffer
 * \param[in] data pointer to data for which CRC should be calculated
 * \param[out] crc pointer to 16-bit CRC
 */
void sha204c_calculate_crc(uint8_t length, uint8_t *data, uint8_t *crc) {
	uint8_t counter;
	uint16_t crc_register = 0;
	uint16_t polynom = 0x8005;
	uint8_t shift_register;
	uint8_t data_bit, crc_bit;

	for (counter = 0; counter < length; counter++) {
	  for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1) {
		 data_bit = (data[counter] & shift_register) ? 1 : 0;
		 crc_bit = crc_register >> 15;

		 // Shift CRC to the left by 1.
		 crc_register <<= 1;

		 if ((data_bit ^ crc_bit) != 0)
			crc_register ^= polynom;
	  }
	}
	crc[0] = (uint8_t) (crc_register & 0x00FF);
	crc[1] = (uint8_t) (crc_register >> 8);
}


/** \brief This function checks the consistency of a response.
 * \param[in] response pointer to response
 * \return status of the consistency check
 */
uint8_t sha204c_check_crc(uint8_t *response)
{
	uint8_t crc[SHA204_CRC_SIZE];
	uint8_t count = response[SHA204_BUFFER_POS_COUNT];

	count -= SHA204_CRC_SIZE;
	sha204c_calculate_crc(count, response, crc);

	return (crc[0] == response[count] && crc[1] == response[count + 1])
		? SHA204_SUCCESS : SHA204_BAD_CRC;
}


/** \brief This function wakes up a SHA204 device
 *         and receives a response.
 *  \param[out] response pointer to four-byte response
 *  \return status of the operation
 */
uint8_t sha204c_wakeup(uint8_t *response)
{
	uint8_t ret_code = sha204p_wakeup();
	if (ret_code != SHA204_SUCCESS)
		return ret_code;

	ret_code = sha204p_receive_response(SHA204_RSP_SIZE_MIN, response);
	if (ret_code != SHA204_SUCCESS)
		return ret_code;

	// Verify status response.
	if (response[SHA204_BUFFER_POS_COUNT] != SHA204_RSP_SIZE_MIN)
		ret_code = SHA204_INVALID_SIZE;
	else if (response[SHA204_BUFFER_POS_STATUS] != SHA204_STATUS_BYTE_WAKEUP)
		ret_code = SHA204_COMM_FAIL;
	else {
		if ((response[SHA204_RSP_SIZE_MIN - SHA204_CRC_SIZE] != 0x33)
					|| (response[SHA204_RSP_SIZE_MIN + 1 - SHA204_CRC_SIZE] != 0x43))
			ret_code = SHA204_BAD_CRC;
	}
	if (ret_code != SHA204_SUCCESS)
		mdelay(SHA204_COMMAND_EXEC_MAX);
		//delay_ms(SHA204_COMMAND_EXEC_MAX);
		
	return ret_code;
}


/** \brief This function re-synchronizes communication.
 *
  Be aware that succeeding only after waking up the
  device could mean that it had gone to sleep and lost
  its TempKey in the process.\n
  Re-synchronizing communication is done in a maximum of
  three steps:
  <ol>
    <li>
      Try to re-synchronize without sending a Wake token.
      This step is implemented in the Physical layer.
    </li>
    <li>
      If the first step did not succeed send a Wake token.
    </li>
    <li>
      Try to read the Wake response.
    </li>
  </ol>
 *
 * \param[in] size size of response buffer
 * \param[out] response pointer to Wake-up response buffer
 * \return status of the operation
 */
uint8_t sha204c_resync(uint8_t size, uint8_t *response)
{
	// Try to re-synchronize without sending a Wake token
	// (step 1 of the re-synchronization process).
	uint8_t ret_code = sha204p_resync(size, response);
	if (ret_code == SHA204_SUCCESS)
		return ret_code;

	// We lost communication. Send a Wake pulse and try
	// to receive a response (steps 2 and 3 of the
	// re-synchronization process).
	(void) sha204p_sleep();
	ret_code = sha204c_wakeup(response);

	// Translate a return value of success into one
	// that indicates that the device had to be woken up
	// and might have lost its TempKey.
	return (ret_code == SHA204_SUCCESS ? SHA204_RESYNC_WITH_WAKEUP : ret_code);
}


/** \brief This function runs a communication sequence:
 * Append CRC to tx buffer, send command, delay, and verify response after receiving it.
 *
 * The first byte in tx buffer must be the byte count of the packet.
 * If CRC or count of the response is incorrect, or a command byte got "nacked" (TWI),
 * this function requests re-sending the response.
 * If the response contains an error status, this function resends the command.
 *
 * \param[in, out]  args pointer to parameter structure
 * \return status of the operation
 */
uint8_t sha204c_send_and_receive(struct sha204_send_and_receive_parameters *args)
{
	uint8_t ret_code = SHA204_FUNC_FAIL;
	uint8_t ret_code_resync;
	uint8_t n_retries_send;
	uint8_t n_retries_receive;
	uint8_t i;
	uint8_t status_byte;
	uint8_t count = args->tx_buffer[SHA204_BUFFER_POS_COUNT];
	uint8_t count_minus_crc = count - SHA204_CRC_SIZE;

	uint16_t execution_timeout_us = (uint16_t) (args->poll_timeout* 1000) + SHA204_RESPONSE_TIMEOUT;
	volatile uint16_t timeout_countdown;

	// Append CRC.
	sha204c_calculate_crc(count_minus_crc, args->tx_buffer, args->tx_buffer + count_minus_crc);

	// Retry loop for sending a command and receiving a response.
	n_retries_send = SHA204_RETRY_COUNT + 1;

	while ((n_retries_send-- > 0) && (ret_code != SHA204_SUCCESS)) 
	{
		// Send command.
		ret_code = sha204p_send_command(count, args->tx_buffer);
		if (ret_code != SHA204_SUCCESS) 
		{
			if (sha204c_resync(args->rx_size, args->rx_buffer) == SHA204_RX_NO_RESPONSE)
				// The device seems to be dead in the water.
				return ret_code;
			else
				continue;
		}

		// Wait typical command execution time and then start polling for a response.
		mdelay(args->poll_delay + 10);
		//delay_ms(args->poll_delay);

		// Retry loop for receiving a response.
		n_retries_receive = SHA204_RETRY_COUNT + 1;
		while (n_retries_receive-- > 0) 
		{
			// Reset response buffer.
			for (i = 0; i < args->rx_size; i++)
				args->rx_buffer[i] = 0;

			mdelay(args->poll_timeout);
			//delay_ms(args->poll_timeout);
			
			// Poll for response.
			timeout_countdown = execution_timeout_us;//????????????????????????????
			do 
			{
				ret_code = sha204p_receive_response(args->rx_size, args->rx_buffer);
				timeout_countdown -= SHA204_RESPONSE_TIMEOUT;//????????????????????
			} while ((timeout_countdown > SHA204_RESPONSE_TIMEOUT)&&(ret_code == SHA204_RX_NO_RESPONSE));

			if (ret_code == SHA204_RX_NO_RESPONSE) 
			{
				// We did not receive a response. Re-synchronize and send command again.
				if (sha204c_resync(args->rx_size, args->rx_buffer) == SHA204_RX_NO_RESPONSE)
					// The device seems to be dead in the water.
					return ret_code;
				else
					break;
			}

			// Check whether we received a valid response.
			if (ret_code == SHA204_INVALID_SIZE) 
			{
				// We see 0xFF for the count when communication got out of sync.
				ret_code_resync = sha204c_resync(args->rx_size, args->rx_buffer);
				if (ret_code_resync == SHA204_SUCCESS)
					// We did not have to wake up the device. Try receiving response again.
					continue;
				if (ret_code_resync == SHA204_RESYNC_WITH_WAKEUP)
					// We could re-synchronize, but only after waking up the device.
					// Re-send command.
					break;
				else
					// We failed to re-synchronize.
					return ret_code;
			}

			// We received a response of valid size.
			// Check the consistency of the response.
			ret_code = sha204c_check_crc(args->rx_buffer);
			if (ret_code == SHA204_SUCCESS) 
			{
				// Received valid response.
				if (args->rx_buffer[SHA204_BUFFER_POS_COUNT] > SHA204_RSP_SIZE_MIN)
					// Received non-status response. We are done.
					return ret_code;

				// Received status response.
				status_byte = args->rx_buffer[SHA204_BUFFER_POS_STATUS];

				// Translate the three possible device status error codes
				// into library return codes.
				if (status_byte == SHA204_STATUS_BYTE_PARSE)
					return SHA204_PARSE_ERROR;
				if (status_byte == SHA204_STATUS_BYTE_EXEC)
					return SHA204_CMD_FAIL;
				if (status_byte == SHA204_STATUS_BYTE_COMM) 
				{
					// In case of the device status byte indicating a communication
					// error this function exits the retry loop for receiving a response
					// and enters the overall retry loop
					// (send command / receive response).
					ret_code = SHA204_STATUS_CRC;
					break;
				}

				// Received status response from CheckMAC, DeriveKey, GenDig,
				// Lock, Nonce, Pause, UpdateExtra, or Write command.
				return ret_code;
			}
			else 
			{
				// Received response with incorrect CRC.
				ret_code_resync = sha204c_resync(args->rx_size, args->rx_buffer);
				if (ret_code_resync == SHA204_SUCCESS)
					// We did not have to wake up the device. Try receiving response again.
					continue;
				if (ret_code_resync == SHA204_RESYNC_WITH_WAKEUP)
					// We could re-synchronize, but only after waking up the device.
					// Re-send command.
					break;
				else
					// We failed to re-synchronize.
					return ret_code;
			} // block end of check response consistency

		} // block end of receive retry loop

	} // block end of send and receive retry loop

	return ret_code;
}

//=============================================================================================================
//atsha204_wakeup_and_validate_device.c
const uint8_t SHA204_DEVREV_VALUE[WRITE_BUFFER_SIZE_SHORT] = {0x00, 0x00, 0x00, 0x04}; /* The device revision - currently DevRev 4 */	
uint8_t wakeup_response_buffer[SHA204_RSP_SIZE_MIN] = {0}; 

uint8_t atsha204_wakeup_and_validate_device(void)
{
	static uint8_t sha204_lib_return = SHA204_SUCCESS; 
	static uint8_t tries = 10;
	uint8_t tmp = 0;

	// Use the DevRev command to check communication to chip by validating value received.
	// Note that DevRev value is not constant over future revisions of the chip so failure
	// of this function may not mean bad connection.	
	uint8_t transmit_buffer[SHA204_CMD_SIZE_MIN] = {0};
	uint8_t response_buffer[DEVREV_RSP_SIZE] = {0};
	struct sha204_dev_rev_parameters dev_rev;
	
	/* Identify the device on the communication bus and initialize the interface */
	sha204p_set_device_id(SHA204_I2C_DEFAULT_ADDRESS);
	
	mdelay(200);
	//delay_ms(200);
	
	/* Wake up the device or abort operation if unsuccessful */
	while(tries) 
	{
		sha204_lib_return |= sha204p_wakeup();
		if(sha204_lib_return == SHA204_SUCCESS) 
		{
			break;		
		}
		else
		{
			tries--;
			if(tries == 0) 
			{
				return sha204_lib_return;
			}		
		}
	}	

	dev_rev.rx_buffer = response_buffer;
	dev_rev.tx_buffer = transmit_buffer;
				
	memset(response_buffer, 0, sizeof(response_buffer));

	// Send DevRev command and receive its response.
	sha204_lib_return |= sha204m_dev_rev(&dev_rev);

	// Send Sleep command.
	sha204_lib_return |= sha204p_sleep();	
		
	// validate the received value for DevRev
	for(tmp = 0; tmp < 4; tmp++) 
	{
		if(response_buffer[tmp + 1] != SHA204_DEVREV_VALUE[tmp])
		{
			sha204_lib_return |= SHA204_FUNC_FAIL;
		}
	}
		
	return sha204_lib_return;	
}
//=======================================================================================================================
//sha256.c
#define SHFR(x, n)    (x >> n)
#define ROTR(x, n)   ((x >> n) | (x << ((sizeof(x) << 3) - n)))
#define ROTL(x, n)   ((x << n) | (x >> ((sizeof(x) << 3) - n)))
#define CH(x, y, z)  ((x & y) ^ (~x & z))
#define MAJ(x, y, z) ((x & y) ^ (x & z) ^ (y & z))

#define SHA256_F1(x) (ROTR(x,  2) ^ ROTR(x, 13) ^ ROTR(x, 22))
#define SHA256_F2(x) (ROTR(x,  6) ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SHA256_F3(x) (ROTR(x,  7) ^ ROTR(x, 18) ^ SHFR(x,  3))
#define SHA256_F4(x) (ROTR(x, 17) ^ ROTR(x, 19) ^ SHFR(x, 10))

#define UNPACK32(x, str)                      \
{                                             \
    *((str) + 3) = (uint8) ((x)      );       \
    *((str) + 2) = (uint8) ((x) >>  8);       \
    *((str) + 1) = (uint8) ((x) >> 16);       \
    *((str) + 0) = (uint8) ((x) >> 24);       \
}

#define PACK32(str, x)                        \
{                                             \
    *(x) =   ((uint32) *((str) + 3)      )    \
           | ((uint32) *((str) + 2) <<  8)    \
           | ((uint32) *((str) + 1) << 16)    \
           | ((uint32) *((str) + 0) << 24);   \
}


/* Macros used for loops unrolling */

#define SHA256_SCR(i)                         \
{                                             \
    w[i] =  SHA256_F4(w[i -  2]) + w[i -  7]  \
          + SHA256_F3(w[i - 15]) + w[i - 16]; \
}

#define SHA256_EXP(a, b, c, d, e, f, g, h, j)               \
{                                                           \
    t1 = wv[h] + SHA256_F2(wv[e]) + CH(wv[e], wv[f], wv[g]) \
         + sha256_k[j] + w[j];                              \
    t2 = SHA256_F1(wv[a]) + MAJ(wv[a], wv[b], wv[c]);       \
    wv[d] += t1;                                            \
    wv[h] = t1 + t2;                                        \
}

//flash uint32 sha256_h0[8] =
uint32 sha256_h0[8] =
            {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
             0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

//flash uint32 sha256_k[64] =
uint32 sha256_k[64] =
            {0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
             0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
             0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
             0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
             0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
             0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
             0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
             0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
             0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
             0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
             0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
             0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
             0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
             0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
             0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
             0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

/* SHA-256 functions */

// some helper macros for TPM implementation
//#define zram32  ((UINT32*) zram)
//#define eob32(buf) ((sizeof(buf)/sizeof(UINT32)))
void sha256_transf(sha256_ctx *ctx, const uint8 *message,
uint32 block_nb);
void sha256_transf(sha256_ctx *ctx, const uint8 *message,
                   uint32 block_nb)
{
    uint32 w[64];
    uint32 wv[8];
//	uint32 *wv = &zram32[eob32(zram)-8];
//	uint32 *w = &zram32[eob32(zram)-(8+64)];

    uint32 t1, t2;
    const uint8 *sub_block;
    int i, j;

    for (i = 0; i < (int) block_nb; i++) {
        sub_block = message + (i << 6);

        for (j = 0; j < 16; j++) {
            PACK32(&sub_block[j << 2], &w[j]);
        }

        for (j = 16; j < 64; j++) {
            SHA256_SCR(j);
        }

        for (j = 0; j < 8; j++) {
            wv[j] = ctx->h[j];
        }

        for (j = 0; j < 64; j++) {
            t1 = wv[7] + SHA256_F2(wv[4]) + CH(wv[4], wv[5], wv[6])
                + sha256_k[j] + w[j];
            t2 = SHA256_F1(wv[0]) + MAJ(wv[0], wv[1], wv[2]);
            wv[7] = wv[6];
            wv[6] = wv[5];
            wv[5] = wv[4];
            wv[4] = wv[3] + t1;
            wv[3] = wv[2];
            wv[2] = wv[1];
            wv[1] = wv[0];
            wv[0] = t1 + t2;
        }

        for (j = 0; j < 8; j++) {
            ctx->h[j] += wv[j];
        }

    }
}

void sha256(const uint8 *message, uint32 len, uint8 *digest)
{
    sha256_ctx ctx;

    sha256_init(&ctx);
    sha256_update(&ctx, message, len);
    sha256_final(&ctx, digest);
}

void sha256_init(sha256_ctx *ctx)
{
    int i;
    for (i = 0; i < 8; i++) {
        ctx->h[i] = sha256_h0[i];
    }
	
    ctx->len = 0;
    ctx->tot_len = 0;
}

void sha256_update(sha256_ctx *ctx, const uint8 *message,
                   uint32 len)
{
    uint32 block_nb;
    uint32 new_len, rem_len, tmp_len;
    const uint8 *shifted_message;

    tmp_len = SHA256_BLOCK_SIZE - ctx->len;
    rem_len = len < tmp_len ? len : tmp_len;

    memcpy(&ctx->block[ctx->len], message, rem_len);

    if (ctx->len + len < SHA256_BLOCK_SIZE) {
        ctx->len += len;
        return;
    }

    new_len = len - rem_len;
    block_nb = new_len / SHA256_BLOCK_SIZE;

    shifted_message = message + rem_len;

    sha256_transf(ctx, ctx->block, 1);
    sha256_transf(ctx, shifted_message, block_nb);

    rem_len = new_len % SHA256_BLOCK_SIZE;

    memcpy(ctx->block, &shifted_message[block_nb << 6],
           rem_len);

    ctx->len = rem_len;
    ctx->tot_len += (block_nb + 1) << 6;
}

void sha256_final(sha256_ctx *ctx, uint8 *digest)
{
    uint32 block_nb;
    uint32 pm_len;
    uint32 len_b;
    int i;


    block_nb = (1 + ((SHA256_BLOCK_SIZE - 9)
                     < (ctx->len % SHA256_BLOCK_SIZE)));

    len_b = (ctx->tot_len + ctx->len) << 3;
    pm_len = block_nb << 6;

    memset(ctx->block + ctx->len, 0, pm_len - ctx->len);
    ctx->block[ctx->len] = 0x80;
    UNPACK32(len_b, ctx->block + pm_len - 4);

    sha256_transf(ctx, ctx->block, block_nb);

    for (i = 0 ; i < 8; i++) {
        UNPACK32(ctx->h[i], &digest[i << 2]);
    }
}

void sha256_noPad(sha256_ctx *ctx, uint8 *digest)
{
   int i;

   for (i = 0 ; i < 8; i++) {
        UNPACK32(ctx->h[i], &digest[i << 2]);
    }
}

//=======================================================================================================================
//sha204_helper.c
/** \brief This function calculates a 32-byte nonce based on 20-byte input value (NumIn) and 32-byte random number (RandOut).
 *
 *         This nonce will match with the nonce generated in the Device by Nonce opcode.
 *         To use this function, Application first executes Nonce command in the Device, with a chosen NumIn.
 *         Nonce opcode Mode parameter must be set to use random nonce (mode 0 or 1).
 *         The Device generates a nonce, stores it in its TempKey, and outputs random number RandOut to host.
 *         This RandOut along with NumIn are passed to nonce calculation function. The function calculates the nonce, and returns it.
 *         This function can also be used to fill in the nonce directly to TempKey (pass-through mode). The flags will automatically set according to the mode used.
 *
 * \param [in,out] param Structure for input/output parameters. Refer to sha204h_nonce_in_out.
 * \return status of the operation.
 */ 
uint8_t sha204h_nonce(struct sha204h_nonce_in_out param)
{
	// Local Variables
	uint8_t temporary[SHA204_MSG_SIZE_NONCE];	
	uint8_t *p_temp;
	
	// Check parameters
	if (	!param.temp_key || !param.num_in
			|| (param.mode > NONCE_MODE_PASSTHROUGH)
			|| (param.mode == NONCE_MODE_INVALID)
			|| (param.mode == NONCE_MODE_SEED_UPDATE && !param.rand_out)
			|| (param.mode == NONCE_MODE_NO_SEED_UPDATE && !param.rand_out) )
		return SHA204_BAD_PARAM;

	// Calculate or pass-through the nonce to TempKey.Value
	if ((param.mode == NONCE_MODE_SEED_UPDATE) || (param.mode == NONCE_MODE_NO_SEED_UPDATE)) {
		// Calculate nonce using SHA-256 (refer to datasheet)
		p_temp = temporary;
		
		memcpy(p_temp, param.rand_out, 32);
		p_temp += 32;
		
		memcpy(p_temp, param.num_in, 20);
		p_temp += 20;
		
		*p_temp++ = SHA204_NONCE;
		*p_temp++ = param.mode;
		*p_temp++ = 0x00;
			
		sha256(temporary, SHA204_MSG_SIZE_NONCE, param.temp_key->value);
		
		// Update TempKey.SourceFlag to 0 (random)
		param.temp_key->source_flag = 0;
	} else if (param.mode == NONCE_MODE_PASSTHROUGH) {
		// Pass-through mode
		memcpy(param.temp_key->value, param.num_in, 32);
		
		// Update TempKey.SourceFlag to 1 (not random)
		param.temp_key->source_flag = 1;
	}
	
	// Update TempKey fields
	param.temp_key->key_id = 0;
	param.temp_key->gen_data = 0;
	param.temp_key->check_flag = 0;
	param.temp_key->valid = 1;
	
	return SHA204_SUCCESS;
}


/** \brief This function generates an SHA-256 digest (MAC) of a key, challenge, and other informations.
 *
 *         The resulting digest will match with those generated in the Device by MAC opcode.
 *         The TempKey (if used) should be valid (temp_key.valid = 1) before executing this function.
 *
 * \param [in,out] param Structure for input/output parameters. Refer to sha204h_mac_in_out.
 * \return status of the operation.
 */ 
uint8_t sha204h_mac(struct sha204h_mac_in_out param)
{
	// Local Variables
	uint8_t temporary[SHA204_MSG_SIZE_MAC];
	uint8_t i;
	uint8_t *p_temp;
	
	// Check parameters
	if (	!param.response
			|| ((param.mode & ~MAC_MODE_MASK) != 0)
			|| (((param.mode & MAC_MODE_BLOCK1_TEMPKEY) == 0) && !param.key)
			|| (((param.mode & MAC_MODE_BLOCK2_TEMPKEY) == 0) && !param.challenge)
			|| (((param.mode & MAC_MODE_USE_TEMPKEY_MASK) != 0) && !param.temp_key)
			|| (((param.mode & MAC_MODE_INCLUDE_OTP_64) != 0) && !param.otp)
			|| (((param.mode & MAC_MODE_INCLUDE_OTP_88) != 0) && !param.otp)
			|| (((param.mode & MAC_MODE_INCLUDE_SN) != 0) && !param.sn) )
		return SHA204_BAD_PARAM;
	
	// Check TempKey fields validity if TempKey is used
	if (	((param.mode & MAC_MODE_USE_TEMPKEY_MASK) != 0) &&
			// TempKey.CheckFlag must be 0 and TempKey.Valid must be 1
			(  (param.temp_key->check_flag != 0)
			|| (param.temp_key->valid != 1) 
			// If either mode parameter bit 0 or bit 1 are set, mode parameter bit 2 must match temp_key.source_flag
			// Logical not (!) are used to evaluate the expression to TRUE/FALSE first before comparison (!=)
			|| (!(param.mode & MAC_MODE_SOURCE_FLAG_MATCH) != !(param.temp_key->source_flag)) ))
		return SHA204_CMD_FAIL;
	
	// Start calculation
	p_temp = temporary;
		
	// (1) first 32 bytes
	if (param.mode & MAC_MODE_BLOCK1_TEMPKEY) {
		memcpy(p_temp, param.temp_key->value, 32);    // use TempKey.Value
		p_temp += 32;
	} else {
		memcpy(p_temp, param.key, 32);                // use Key[KeyID]
		p_temp += 32;
	}
	
	// (2) second 32 bytes
	if (param.mode & MAC_MODE_BLOCK2_TEMPKEY) {
		memcpy(p_temp, param.temp_key->value, 32);    // use TempKey.Value
		p_temp += 32;
	} else {
		memcpy(p_temp, param.challenge, 32);          // use challenge
		p_temp += 32;
	}
	
	// (3) 1 byte opcode
	*p_temp++ = SHA204_MAC;
	
	// (4) 1 byte mode parameter
	*p_temp++ = param.mode;
	
	// (5) 2 bytes keyID
	*p_temp++ = param.key_id & 0xFF;
	*p_temp++ = (param.key_id >> 8) & 0xFF;
	
	// (6, 7) 8 bytes OTP[0:7] or 0x00's, 3 bytes OTP[8:10] or 0x00's
	if (param.mode & MAC_MODE_INCLUDE_OTP_88) {
		memcpy(p_temp, param.otp, 11);            // use OTP[0:10], Mode:5 is overridden
		p_temp += 11;
	} else {
		if (param.mode & MAC_MODE_INCLUDE_OTP_64) {
			memcpy(p_temp, param.otp, 8);         // use 8 bytes OTP[0:7] for (6)
			p_temp += 8;
		} else {
			for (i = 0; i < 8; i++) {             // use 8 bytes 0x00's for (6)
				*p_temp++ = 0x00;
			}
		}
		
		for (i = 0; i < 3; i++) {                 // use 3 bytes 0x00's for (7)
			*p_temp++ = 0x00;
		}
	}
	
	// (8) 1 byte SN[8] = 0xEE
	*p_temp++ = SHA204_SN_8;
	
	// (9) 4 bytes SN[4:7] or 0x00's
	if (param.mode & MAC_MODE_INCLUDE_SN) {
		memcpy(p_temp, &param.sn[4], 4);     //use SN[4:7] for (9)
		p_temp += 4;
	} else {
		for (i = 0; i < 4; i++) {            //use 0x00's for (9)
			*p_temp++ = 0x00;
		}
	}
	
	// (10) 2 bytes SN[0:1] = 0x0123
	*p_temp++ = SHA204_SN_0;
	*p_temp++ = SHA204_SN_1;
	
	// (11) 2 bytes SN[2:3] or 0x00's
	if (param.mode & MAC_MODE_INCLUDE_SN) {
		memcpy(p_temp, &param.sn[2], 2);     //use SN[2:3] for (11)
		p_temp += 2;
	} else {
		for (i = 0; i < 2; i++) {            //use 0x00's for (11)
			*p_temp++ = 0x00;
		}       
	}
	
	// This is the resulting MAC digest
	sha256(temporary, SHA204_MSG_SIZE_MAC, param.response);
	
	// Update TempKey fields
	param.temp_key->valid = 0;
	
	return SHA204_SUCCESS;
}

/** \brief This function generates an HMAC/SHA-256 digest of a key and other informations.
 *
 *         The resulting digest will match with those generated in the Device by HMAC opcode.
 *         The TempKey should be valid (temp_key.valid = 1) before executing this function.
 *
 * \param [in,out] param Structure for input/output parameters. Refer to sha204h_hmac_in_out.
 * \return status of the operation.
 */ 
uint8_t sha204h_hmac(struct sha204h_hmac_in_out param)
{
	// Local Variables
	uint8_t temporary[SHA204_MSG_SIZE_HMAC_INNER];
	uint8_t i;
	uint8_t *p_temp;
	
	// Check parameters
	if (	!param.response || !param.key || !param.temp_key
			|| ((param.mode & ~HMAC_MODE_MASK) != 0)
			|| (((param.mode & MAC_MODE_INCLUDE_OTP_64) != 0) && !param.otp)
			|| (((param.mode & MAC_MODE_INCLUDE_OTP_88) != 0) && !param.otp)
			|| (((param.mode & MAC_MODE_INCLUDE_SN) != 0) && !param.sn) )
		return SHA204_BAD_PARAM;
	
	// Check TempKey fields validity (TempKey is always used)
	if (	// TempKey.CheckFlag must be 0 and TempKey.Valid must be 1
			   (param.temp_key->check_flag != 0)
			|| (param.temp_key->valid != 1) 
			// The mode parameter bit 2 must match temp_key.source_flag
			// Logical not (!) are used to evaluate the expression to TRUE/FALSE first before comparison (!=)
			|| (!(param.mode & MAC_MODE_SOURCE_FLAG_MATCH) != !(param.temp_key->source_flag)) )
		return SHA204_CMD_FAIL;
		
	// Start first calculation (inner)
	p_temp = temporary;

	// Refer to fips-198a.pdf, length Key = 32 bytes, Blocksize = 512 bits = 64 bytes,
	//   so the Key must be padded with zeros	
	// XOR K0 with ipad, then append
	for (i = 0; i < 32; i++) {
		*p_temp++ = param.key[i] ^ 0x36;
	}

	// XOR the remaining zeros and append
	for (i = 0; i < 32; i++) {
		*p_temp++ = 0 ^ 0x36;
	}
	
	// Next append the stream of data 'text'
	// (1) first 32 bytes: zeros
	for (i = 0; i < 32; i++) {
		*p_temp++ = 0;
	}
	
	// (2) second 32 bytes: TempKey
	memcpy(p_temp, param.temp_key->value, 32);
	p_temp += 32;
	
	// (3) 1 byte opcode
	*p_temp++ = SHA204_HMAC;
	
	// (4) 1 byte mode parameter
	*p_temp++ = param.mode;
	
	// (5) 2 bytes keyID
	*p_temp++ = param.key_id & 0xFF;
	*p_temp++ = (param.key_id >> 8) & 0xFF;
	
	// (6, 7) 8 bytes OTP[0:7] or 0x00's, 3 bytes OTP[8:10] or 0x00's
	if (param.mode & MAC_MODE_INCLUDE_OTP_88) {
		memcpy(p_temp, param.otp, 11);            // use OTP[0:10], Mode:5 is overridden
		p_temp += 11;
	} else {
		if (param.mode & MAC_MODE_INCLUDE_OTP_64) {
			memcpy(p_temp, param.otp, 8);         // use 8 bytes OTP[0:7] for (6)
			p_temp += 8;
		} else {
			for (i = 0; i < 8; i++) {             // use 8 bytes 0x00's for (6)
				*p_temp++ = 0x00;
			}
		}
		
		for (i = 0; i < 3; i++) {                 // use 3 bytes 0x00's for (7)
			*p_temp++ = 0x00;
		}
	}	

	// (8) 1 byte SN[8] = 0xEE
	*p_temp++ = SHA204_SN_8;

	// (9) 4 bytes SN[4:7] or 0x00's
	if (param.mode & MAC_MODE_INCLUDE_SN) {
		memcpy(p_temp, &param.sn[4], 4);     //use SN[4:7] for (9)
		p_temp += 4;
	} else {
		for (i = 0; i < 4; i++) {            //use 0x00's for (9)
			*p_temp++ = 0x00; 
		}
	}

	// (10) 2 bytes SN[0:1] = 0x0123
	*p_temp++ = SHA204_SN_0;
	*p_temp++ = SHA204_SN_1;

	// (11) 2 bytes SN[2:3] or 0x00's
	if (param.mode & MAC_MODE_INCLUDE_SN) {
		memcpy(p_temp, &param.sn[2], 2);     //use SN[2:3] for (11)
		p_temp += 2;
	} else {
		for (i = 0; i < 2; i++) {            //use 0x00's for (11)
			*p_temp++ = 0x00;
		}
	}
	
	// This is now H((K0^ipad):text), use param.response for temporary storage
	sha256(temporary, SHA204_MSG_SIZE_HMAC_INNER, param.response);
	
	
	// Start second calculation (outer)
	p_temp = temporary;
	
	// XOR K0 with opad, then append
	for (i = 0; i < 32; i++) {
		*p_temp++ = param.key[i] ^ 0x5C;
	}
	
	// XOR the remaining zeros and append
	for (i = 0; i < 32; i++) {
		*p_temp++ = 0 ^ 0x5C;
	}	
	
	// Append result from last calculation H((K0^ipad):text)
	memcpy(p_temp, param.response, 32);
	p_temp += 32;
	
	// This is the resulting HMAC	
	sha256(temporary, SHA204_MSG_SIZE_HMAC_OUTER, param.response);	

	// Update TempKey fields
	param.temp_key->valid = 0;
	
	return SHA204_SUCCESS;
}


/** \brief This function combines current TempKey with a stored value.
 *
 *         The stored value can be a data slot, OTP page, configuration zone, or hardware transport key.
 *         The TempKey generated by this function will match with the TempKey in the Device generated by GenDig opcode.
 *         The TempKey should be valid (temp_key.valid = 1) before executing this function.
 *         To use this function, Application first executes GenDig command in the Device, with a chosen stored value.
 *         This stored value must be known by the Application, and is passed to GenDig calculation function.
 *         The function calculates new TempKey, and returns it.
 *
 * \param [in,out] param Structure for input/output parameters. Refer to sha204h_gen_dig_in_out.
 * \return status of the operation.
 */ 
uint8_t sha204h_gen_dig(struct sha204h_gen_dig_in_out param)
{
	// Local Variables
	uint8_t temporary[SHA204_MSG_SIZE_GEN_DIG];
	uint8_t i;
	uint8_t *p_temp;

	// Check parameters
	if (	!param.stored_value || !param.temp_key
			|| ((param.zone != GENDIG_ZONE_OTP)
			    && (param.zone != GENDIG_ZONE_DATA)
				&& (param.zone != GENDIG_ZONE_CONFIG)) )
		return SHA204_BAD_PARAM;
	
	// Check TempKey fields validity (TempKey is always used)
	if (	// TempKey.CheckFlag must be 0 and TempKey.Valid must be 1
			   (param.temp_key->check_flag != 0)
			|| (param.temp_key->valid != 1) )
		return SHA204_CMD_FAIL;

	
	// Start calculation
	p_temp = temporary;
	
	// (1) 32 bytes inputKey
	//     (Config[KeyID] or OTP[KeyID] or Data.slot[KeyID] or TransportKey[KeyID])
	memcpy(p_temp, param.stored_value, 32);
	p_temp += 32;
	
	// (2) 1 byte Opcode
	*p_temp++ = SHA204_GENDIG;
	
	// (3) 1 byte Param1 (zone)
	*p_temp++ = param.zone;
	
	// (4) 2 bytes Param2 (keyID)
	*p_temp++ = param.key_id & 0xFF;
	*p_temp++ = (param.key_id >> 8) & 0xFF;
	
	// (5) 1 byte SN[8] = 0xEE
	*p_temp++ = SHA204_SN_8;
	
	// (6) 2 bytes SN[0:1] = 0x0123
	*p_temp++ = SHA204_SN_0;
	*p_temp++ = SHA204_SN_1;
	
	// (7) 25 bytes 0x00's
	for (i = 0; i < 25; i++) {
		*p_temp++ = 0x00;
	}
	
	// (8) 32 bytes TempKey
	memcpy(p_temp, param.temp_key->value, 32);
	
	// This is the new TempKey
	sha256(temporary, SHA204_MSG_SIZE_GEN_DIG, param.temp_key->value);

	
	// Update TempKey fields
	param.temp_key->valid = 1;
	
	if ((param.zone == GENDIG_ZONE_DATA) && (param.key_id <= 15)) {
		param.temp_key->gen_data = 1;
		param.temp_key->key_id = (param.key_id & 0xF);    // mask lower 4-bit only
	} else {
		param.temp_key->gen_data = 0;
		param.temp_key->key_id = 0;
	}
	
	return SHA204_SUCCESS;
}


/** \brief This function combines current value of a key with the TempKey.
 *
 *         Used in conjunction with DeriveKey command, the key derived by this function will match with the key in the Device.
 *         Two kind of operations are supported:
 *         - Roll Key operation, target_key and parent_key parameters should be set to point to the same location (TargetKey).
 *         - Create Key operation, target_key should be set to point to TargetKey, parent_key should be set to point to ParentKey.
 *         After executing this function, initial value of target_key will be overwritten with the derived key.
 *         The TempKey should be valid (temp_key.valid = 1) before executing this function.
 *
 * \param [in,out] param Structure for input/output parameters. Refer to sha204h_derive_key_in_out.
 * \return status of the operation.
 */ 
uint8_t sha204h_derive_key(struct sha204h_derive_key_in_out param)
{
	// Local Variables
	uint8_t temporary[SHA204_MSG_SIZE_DERIVE_KEY];
	uint8_t i;
	uint8_t *p_temp;

	// Check parameters			
	if (	!param.parent_key || !param.target_key || !param.temp_key
			|| ((param.random & ~DERIVE_KEY_RANDOM_FLAG) != 0)
			|| (param.target_key_id > SHA204_KEY_ID_MAX) )
		return SHA204_BAD_PARAM;
		
	// Check TempKey fields validity (TempKey is always used)
	if (	// TempKey.CheckFlag must be 0 and TempKey.Valid must be 1
			   (param.temp_key->check_flag != 0)
			|| (param.temp_key->valid != 1) 
			// The random parameter bit 2 must match temp_key.source_flag
			// Logical not (!) are used to evaluate the expression to TRUE/FALSE first before comparison (!=)
			|| (!(param.random & DERIVE_KEY_RANDOM_FLAG) != !(param.temp_key->source_flag)) )
		return SHA204_CMD_FAIL;	
	
	
	// Start calculation
	p_temp = temporary;
	
	// (1) 32 bytes parent key
	memcpy(p_temp, param.parent_key, 32);
	p_temp += 32;

	// (2) 1 byte Opcode
	*p_temp++ = SHA204_DERIVE_KEY;
	
	// (3) 1 byte Param1 (random)
	*p_temp++ = param.random;
	
	// (4) 2 bytes Param2 (keyID)
	*p_temp++ = param.target_key_id & 0xFF;
	*p_temp++ = (param.target_key_id >> 8) & 0xFF;
	
	// (5) 1 byte SN[8] = 0xEE
	*p_temp++ = SHA204_SN_8;
	
	// (6) 2 bytes SN[0:1] = 0x0123
	*p_temp++ = SHA204_SN_0;
	*p_temp++ = SHA204_SN_1;
	
	// (7) 25 bytes 0x00's
	for (i = 0; i < 25; i++) {
		*p_temp++ = 0x00;
	}
	
	// (8) 32 bytes tempKey
	memcpy(p_temp, param.temp_key->value, 32);
	p_temp += 32;
	
	// This is the derived key
	sha256(temporary, SHA204_MSG_SIZE_DERIVE_KEY, param.target_key);	

	
	// Update TempKey fields
	param.temp_key->valid = 0;	
	
	return SHA204_SUCCESS;
}


/** \brief This function calculates input MAC for DeriveKey opcode.
 *
 *         If SlotConfig[TargetKey].Bit15 is set, DeriveKey command needs an input MAC.
 *
 * \param [in,out] param Structure for input/output parameters. Refer to sha204h_derive_key_mac_in_out.
 * \return status of the operation.
 */ 
uint8_t sha204h_derive_key_mac(struct sha204h_derive_key_mac_in_out param)
{
	// Local Variables
	uint8_t temporary[SHA204_MSG_SIZE_DERIVE_KEY_MAC];
	uint8_t *p_temp;

	// Check parameters			
	if (	!param.parent_key || !param.mac
			|| ((param.random & ~DERIVE_KEY_RANDOM_FLAG) != 0)
			|| (param.target_key_id > SHA204_KEY_ID_MAX) )
		return SHA204_BAD_PARAM;

	// Start calculation		
	p_temp = temporary;
	
	// (1) 32 bytes parent key
	memcpy(p_temp, param.parent_key, 32);
	p_temp += 32;
	
	// (2) 1 byte Opcode
	*p_temp++ = SHA204_DERIVE_KEY;
	
	// (3) 1 byte Param1 (random)
	*p_temp++ = param.random;
	
	// (4) 2 bytes Param2 (keyID)
	*p_temp++ = param.target_key_id & 0xFF;
	*p_temp++ = (param.target_key_id >> 8) & 0xFF;
	
	// (5) 1 byte SN[8] = 0xEE
	*p_temp++ = SHA204_SN_8;
	
	// (6) 2 bytes SN[0:1] = 0x0123
	*p_temp++ = SHA204_SN_0;
	*p_temp++ = SHA204_SN_1;
	
	// This is the input MAC for DeriveKey command
	sha256(temporary, SHA204_MSG_SIZE_DERIVE_KEY_MAC, param.mac);

	return SHA204_SUCCESS;
}


/** \brief This function encrypts 32-byte cleartext data to be written using Write opcode, and optionally calculates input MAC.
 *
 *         To use this function, first the nonce must be valid and synchronized between Device and Application.
 *         Application executes GenDig command in the Device, using parent key. If Data zone has been locked, this is specified by SlotConfig.WriteKey. The Device updates its TempKey.
 *         Application then updates its own TempKey using GenDig calculation function, using the same key.
 *         Application passes the cleartext data to encryption function.
 *         If input MAC is needed, application must pass a valid pointer to buffer in the "mac" parameter. If input MAC is not needed, application can pass NULL pointer in "mac" parameter.
 *         The function encrypts the data and optionally calculate input MAC, returns it to Application.
 *         Using this encrypted data and input MAC, Application executes Write command in the Device. Device validates the MAC, then decrypts and writes the data.
 *         The encryption function does not check whether the TempKey has been generated by correct ParentKey for the corresponding zone.
 *         Therefore to get a correct result, after Data/OTP locked, Application has to make sure that prior GenDig calculation was done using correct ParentKey.
 *
 * \param [in,out] param Structure for input/output parameters. Refer to sha204h_encrypt_in_out.
 * \return status of the operation.
 */ 
uint8_t sha204h_encrypt(struct sha204h_encrypt_in_out param)
{
	// Local Variables
	uint8_t temporary[SHA204_MSG_SIZE_ENCRYPT_MAC];
	uint8_t i;
	uint8_t *p_temp;

	// Check parameters
	if (!param.data || !param.temp_key || ((param.zone & ~WRITE_ZONE_MASK) != 0))
		return SHA204_BAD_PARAM;
		
	// Check TempKey fields validity
	// Note that temp_key.key_id is not checked,
	//   we cannot make sure if the key used in previous GenDig IS equal to
	//   the key pointed by SlotConfig.WriteKey in the device.
	if (	// TempKey.CheckFlag must be 0
			(param.temp_key->check_flag != 0)
			// TempKey.Valid must be 1
			|| (param.temp_key->valid != 1) 
			// TempKey.GenData must be 1
			|| (param.temp_key->gen_data != 1)
			// TempKey.SourceFlag must be 0 (random)
			|| (param.temp_key->source_flag != 0) )
		return SHA204_CMD_FAIL;
	
	// If the pointer *mac is provided by the caller then calculate input MAC
	if (param.mac) {
		// Start calculation
		p_temp = temporary;
		
		// (1) 32 bytes parent key
		memcpy(p_temp, param.temp_key->value, 32);
		p_temp += 32;

		// (2) 1 byte Opcode
		*p_temp++ = SHA204_WRITE;
		
		// (3) 1 byte Param1 (zone)
		*p_temp++ = param.zone;
		
		// (4) 2 bytes Param2 (address)
		*p_temp++ = param.address & 0xFF;
		*p_temp++ = (param.address >> 8) & 0xFF;
		
		// (5) 1 byte SN[8] = 0xEE
		*p_temp++ = SHA204_SN_8;
		
		// (6) 2 bytes SN[0:1] = 0x0123
		*p_temp++ = SHA204_SN_0;
		*p_temp++ = SHA204_SN_1;
		
		// (7) 25 bytes 0x00's
		for (i = 0; i < 25; i++) {
			*p_temp++ = 0x00;
		}
		
		// (8) 32 bytes data
		memcpy(p_temp, param.data, 32);	
		
		// This is the input MAC
		sha256(temporary, SHA204_MSG_SIZE_ENCRYPT_MAC, param.mac);
	}
	
	
	// Encrypt by XOR-ing Data with the TempKey
	for (i = 0; i < 32; i++) {
		param.data[i] ^= param.temp_key->value[i];
	}
	
	// Update TempKey fields
	param.temp_key->valid = 0;
	
	return SHA204_SUCCESS;
}


/** \brief This function decrypts 32-byte encrypted data (Contents) from Read opcode.
 *
 *         To use this function, first the nonce must be valid and synchronized between Device and Application.
 *         Application executes GenDig command in the Device, using key specified by SlotConfig.ReadKey. The Device updates its TempKey.
 *         Application then updates its own TempKey using GenDig calculation function, using the same key.
 *         Application executes Read command in the Device to a user zone configured with EncryptRead.
 *         The Device encrypts 32-byte zone contents, and outputs it to the host.
 *         Application passes this encrypted data to decryption function. The function decrypts the data, and returns it.
 *         TempKey must be updated by GenDig using a ParentKey as specified by SlotConfig.ReadKey before executing this function.
 *         The decryption function does not check whether the TempKey has been generated by correct ParentKey for the corresponding zone.
 *         Therefore to get a correct result, Application has to make sure that prior GenDig calculation was done using correct ParentKey.
 *
 * \param [in,out] param Structure for input/output parameters. Refer to sha204h_decrypt_in_out.
 * \return status of the operation.
 */ 
uint8_t sha204h_decrypt(struct sha204h_decrypt_in_out param)
{
	// Local Variables
	uint8_t i;
	
	// Check parameters
	if (!param.data || !param.temp_key)
		return SHA204_BAD_PARAM;
	
	// Check TempKey fields validity
	// Note that temp_key.key_id is not checked,
	//   we cannot make sure if the key used in previous GenDig IS equal to
	//   the key pointed by SlotConfig.ReadKey in the device.
	if (	// TempKey.CheckFlag must be 0
			(param.temp_key->check_flag != 0)
			// TempKey.Valid must be 1
			|| (param.temp_key->valid != 1) 
			// TempKey.GenData must be 1
			|| (param.temp_key->gen_data != 1)
			// TempKey.SourceFlag must be 0 (random)
			|| (param.temp_key->source_flag != 0) )
		return SHA204_CMD_FAIL;
	
	// Decrypt by XOR-ing Data with the TempKey
	for (i = 0; i < 32; i++) {
		param.data[i] ^= param.temp_key->value[i];
	}
	
	// Update TempKey fields
	param.temp_key->valid = 0;
	
	return SHA204_SUCCESS;	
}


/** \brief This function calculates CRC.
 *
 *         crc_register is initialized with *crc, so it can be chained to calculate CRC from large array of data.
 *         For the first calculation or calculation without chaining, crc[0] and crc[1] values must be initialized to 0 by the caller.
 *
 * \param[in] length number of bytes in buffer
 * \param[in] data pointer to data for which CRC should be calculated
 * \param[out] crc pointer to 16-bit CRC
 */
void sha204h_calculate_crc_chain(uint8_t length, uint8_t *data, uint8_t *crc)
{
	uint8_t counter;
	uint16_t crc_register = 0;
	uint16_t polynom = 0x8005;
	uint8_t shift_register;
	uint8_t data_bit, crc_bit;
	
	crc_register = (((uint16_t) crc[0]) & 0x00FF) | (((uint16_t) crc[1]) << 8);
	
	for (counter = 0; counter < length; counter++) {
	  for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1) {
		 data_bit = (data[counter] & shift_register) ? 1 : 0;
		 crc_bit = crc_register >> 15;

		 // Shift CRC to the left by 1.
		 crc_register <<= 1;

		 if ((data_bit ^ crc_bit) != 0)
			crc_register ^= polynom;
	  }
	}
	
	crc[0] = (uint8_t) (crc_register & 0x00FF);
	crc[1] = (uint8_t) (crc_register >> 8);
}
//========================================================================================================================
//atsha204_device_personalization.c

uint8_t atsha204_device_personalization(void) 
{
	static uint8_t sha204_lib_return = SHA204_SUCCESS;
	static uint8_t transmit_buffer[SHA204_CMD_SIZE_MAX];
	static uint8_t response_buffer[SHA204_RSP_SIZE_MAX]; 

	struct sha204_write_parameters write_parameters;	
	struct sha204_read_parameters read_parameters;
	struct sha204_lock_parameters lock_parameters;
	

	// Wake the device, validate its presence and put it back to sleep.
	//sha204_lib_return |= atsha204_wakeup_and_validate_device();
	sha204_lib_return |= sha204c_wakeup(wakeup_response_buffer);

	if(SHA204_SUCCESS != sha204_lib_return)
	{
		return sha204_lib_return;
	}

	/*!
	 *	*** ENTER PERSONALIZATON PREFERENCES INTO THE CONFIGURATION MEMORY ***
	 */
	//-----------tony comment:ATSHA204 side operate---------------------------
	//tony comment: personalization step 1-----	 
	// Device Operation Parameters
	write_parameters.tx_buffer = transmit_buffer;
	write_parameters.rx_buffer = response_buffer;
	write_parameters.zone = SHA204_ZONE_CONFIG;
	write_parameters.mac = NULL;

	memset(response_buffer, 0, sizeof(response_buffer));
	
	write_parameters.address = 4 * DEVICE_MODES_ADDRESS;//change double word address to byte word address
	write_parameters.new_value = &DEVICE_MODES[0];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();

	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 2-----
	// *** SLOT CONFIGURATION ***
	// Slots 0 and 1
	write_parameters.address = 4 * SLOT_CONFIG_0_1_ADDRESS;
	write_parameters.new_value = SLOT_CONFIG_00_01;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	// Slots 2 and 3
	write_parameters.address = 4 * SLOT_CONFIG_2_3_ADDRESS;
	write_parameters.new_value = SLOT_CONFIG_02_03;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	// Slots 4 and 5
	write_parameters.address = 4 * SLOT_CONFIG_4_5_ADDRESS;
	write_parameters.new_value = SLOT_CONFIG_04_05;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	// Slots 6 and 7
	write_parameters.address = 4 * SLOT_CONFIG_6_7_ADDRESS;
	write_parameters.new_value = SLOT_CONFIG_06_07;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	// Slots 8 and 9
	write_parameters.address = 4 * SLOT_CONFIG_8_9_ADDRESS;
	write_parameters.new_value = SLOT_CONFIG_08_09;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();		

	// Slots 10 and 11
	write_parameters.address = 4 * SLOT_CONFIG_10_11_ADDRESS;
	write_parameters.new_value = SLOT_CONFIG_10_11;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();		

	// Slots 12 and 13
	write_parameters.address = 4 * SLOT_CONFIG_12_13_ADDRESS;
	write_parameters.new_value = SLOT_CONFIG_12_13;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();

	// Slots 14 and 15
	write_parameters.address = 4 * SLOT_CONFIG_14_15_ADDRESS;
	write_parameters.new_value = SLOT_CONFIG_14_15;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	// *** USE FLAG and UPDATE COUNT Region
	// Slots 0 and 1
	write_parameters.address = 4 * SLOT_0_1_USE_UPDATE_ADDRESS;
	write_parameters.new_value = SLOT_0_1_USE_UPDATE;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	
	
	// Slots 2 and 3
	write_parameters.address = 4 * SLOT_2_3_USE_UPDATE_ADDRESS;
	write_parameters.new_value = SLOT_2_3_USE_UPDATE;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	// Slots 4 and 5
	write_parameters.address = 4 * SLOT_4_5_USE_UPDATE_ADDRESS;
	write_parameters.new_value = SLOT_4_5_USE_UPDATE;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	


	// Slots 6 and 7
	write_parameters.address = 4 * SLOT_6_7_USE_UPDATE_ADDRESS;
	write_parameters.new_value = SLOT_6_7_USE_UPDATE;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();

	
	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 3-----
	// *** LAST KEY USE Region ***
	// First word
	write_parameters.address = 4* (LAST_KEY_USE_ADDRESS + 0);
	write_parameters.new_value = &LAST_KEY_USE[0];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	
	
	// Second word
	write_parameters.address = 4 * (LAST_KEY_USE_ADDRESS+1);
	write_parameters.new_value = &LAST_KEY_USE[4];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	// Third word
	write_parameters.address = 4 * (LAST_KEY_USE_ADDRESS + 2);
	write_parameters.new_value = &LAST_KEY_USE[8];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	
	
	// Fourth word
	write_parameters.address = 4 * (LAST_KEY_USE_ADDRESS + 3);
	write_parameters.new_value = &LAST_KEY_USE[12];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 4-----	
	//*	*** OPTIONAL READ and VERIFY ****
	// The read data is contained in the response buffer.  Note that the buffer does not accumulate data. It's content is refreshed on each read so break and inspect buffer after each command execution.
	sha204_lib_return |= sha204p_wakeup();

	read_parameters.tx_buffer = transmit_buffer;
	read_parameters.rx_buffer = response_buffer;
	read_parameters.zone = SHA204_ZONE_CONFIG | SHA204_ZONE_COUNT_FLAG;
	read_parameters.address = 4 * CONFIG_BLOCK_0_ADDRESS;

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 32 bytes	

	read_parameters.address = 4 * CONFIG_BLOCK_1_ADDRESS;

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 32 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 0);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 1);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	
	
	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address =4 *  (CONFIG_BLOCK_2_ADDRESS + 2);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 3);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 4);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 5);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 5-----	
	/*!
	 *	*** LOCK CONFIG ***
	 **********************
	 *
	 *	Forever lock the custom configuration from future modification.  Writing of Data or OTP regions requires prior execution of this command.
	 */
	 lock_parameters.tx_buffer = transmit_buffer;
	 lock_parameters.rx_buffer = response_buffer;
	 lock_parameters.zone = LOCK_ZONE_NO_CRC;
	 lock_parameters.summary = LOCK_PARAM2_NO_CRC;
	 
	 sha204_lib_return |= sha204m_lock(&lock_parameters);

	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 6-----	
	/*!
	 *	*** WRITE INITIAL DATA TO DATA SLOTS ***
	 *******************************************
	 *
	 *  Write initial content to data slots.  This is the only opportunity to to write non-modifiable information e.g. model numbers and certain keys.
	 */
	write_parameters.tx_buffer = transmit_buffer;
	write_parameters.rx_buffer = response_buffer;
	write_parameters.zone = SHA204_ZONE_DATA | SHA204_ZONE_COUNT_FLAG;
	write_parameters.mac = NULL;

	// Write initial content for slot 0
	write_parameters.address = 4 * SLOT_0_ADDRESS;
	write_parameters.new_value = SLOT_00_CONTENT;	
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 1	
	write_parameters.address = 4 * SLOT_1_ADDRESS;
	write_parameters.new_value = SLOT_01_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 2
	write_parameters.address = 4 * SLOT_2_ADDRESS;
	write_parameters.new_value = SLOT_02_CONTENT;	
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 3	
	write_parameters.address = 4 * SLOT_3_ADDRESS;
	write_parameters.new_value = SLOT_03_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 4
	write_parameters.address = 4 * SLOT_4_ADDRESS;
	write_parameters.new_value = SLOT_04_CONTENT;	
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 5	
	write_parameters.address = 4 * SLOT_5_ADDRESS;
	write_parameters.new_value = SLOT_05_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 6
	write_parameters.address = 4 * SLOT_6_ADDRESS;
	write_parameters.new_value = SLOT_06_CONTENT;	

	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 7	
	write_parameters.address = 4 * SLOT_7_ADDRESS;
	write_parameters.new_value = SLOT_07_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();	

	// Write initial content for slot 8
	write_parameters.address = 4 * SLOT_8_ADDRESS;
	write_parameters.new_value = SLOT_08_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 9	
	write_parameters.address = 4 * SLOT_9_ADDRESS;
	write_parameters.new_value = SLOT_09_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();			

	// Write initial content for slot 10	
	write_parameters.address = 4 * SLOT_10_ADDRESS;
	write_parameters.new_value = SLOT_10_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 11
	write_parameters.address = 4 * SLOT_11_ADDRESS;
	write_parameters.new_value = SLOT_11_CONTENT;	
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 12	
	write_parameters.address = 4 * SLOT_12_ADDRESS;
	write_parameters.new_value = SLOT_12_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 13
	write_parameters.address = 4 * SLOT_13_ADDRESS;
	write_parameters.new_value = SLOT_13_CONTENT;	
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 14	
	write_parameters.address = 4 * SLOT_14_ADDRESS;
	write_parameters.new_value = SLOT_14_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();	

	// Write initial content for slot 15
	write_parameters.address = 4 *SLOT_15_ADDRESS;
	write_parameters.new_value = SLOT_15_CONTENT;	

	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();	

	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 7-----		
	/*!
	 *	*** WRITE INITIAL OTP DATA INTO THE OTP REGION ***
	 *****************************************************
	 *  Write initial information to the OTP region.  This is the only opportunity to do so.  After locking data and OTP regions, write accesses to this region will be controlled by 
	 *  custom access privileges defined in the configuration region.
	 */
	write_parameters.zone = SHA204_ZONE_OTP| SHA204_ZONE_COUNT_FLAG; 
	write_parameters.address = 4 * OTP_BLOCK_0_ADDRESS;
	write_parameters.new_value = &OTP[0];
	
	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();	

	write_parameters.address = 4 * OTP_BLOCK_1_ADDRESS;
	write_parameters.new_value = &OTP[8];	
	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();	
	
	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 8-----	
	/*!
	 *	*** LOCK VALUE ***
	 *********************
	 *
	 *	Forever lock the data and OTP regions.  After lock data, access to these regions will be controlled by access rights defined in the configuration region.
	 */
	 lock_parameters.zone = LOCK_ZONE_NO_CONFIG|LOCK_ZONE_NO_CRC;
	 lock_parameters.summary = LOCK_PARAM2_NO_CRC;
	 
	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_lock(&lock_parameters);	
	sha204_lib_return |= sha204p_sleep();		
	/*!
	 *	*** VERIFY SUCCESSFUL COMPLETION OF THE PERSONALIZATION PROCESS ***
	 **********************************************************************
	 *
	 * Check that all functions executed without errors and that the chip is actually locked.
	 */	 
	 
	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 9-----		
	// Read lock bytes and verify successful lock
	#if 0
	read_parameters.tx_buffer = transmit_buffer;
	read_parameters.rx_buffer = response_buffer;	
	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * EXTRA_SELECTOR_LOCK_ADDRESS;	

	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_read(&read_parameters);
	sha204_lib_return |= sha204p_sleep();		
	
	sha204_lib_return |= response_buffer[3] /* LockValue */ & response_buffer[4] /* LockConfig */; 
	#endif

	return sha204_lib_return;
}


//========================================================================================================================
//enc_write.c enc_read.c enc_mac.c 
uint8_t atsha204_enc_write(uint16_t slot_to_write, uint8_t* clear_data, uint16_t key_id, uint8_t* secret_key, uint8_t* NumIn) 
{	
	static uint8_t sha204_lib_return = SHA204_SUCCESS;			//!< Function execution status, initialized to SUCCES and bitmasked with error codes as needed.
	uint8_t transmit_buffer[SHA204_CMD_SIZE_MAX];	//!< Transmit data buffer
	uint8_t response_buffer[SHA204_RSP_SIZE_MAX];	//!< Receive data buffer
//	uint8_t soft_digest [32];						//!< Buffer to hold software calculated digest
	uint8_t mac[32];								//!< Buffer to hold required authentication input MAC from host
	uint8_t enc_data[32];							//!< Buffer to hold the host computed encrypted data to write
	struct sha204h_nonce_in_out mcu_nonce_parameters;      //!< Parameter for nonce helper function
	struct sha204h_temp_key mcu_tempkey;				//!< tempkey parameter for nonce and mac helper function	struct sha204h_gen_dig_in_out gendig_param;		//!< Parameter for gendig helper function
	struct sha204h_gen_dig_in_out mcu_gendig_param;		//!< Parameter for gendig helper function
	struct sha204h_encrypt_in_out mcu_enc_param;		//!< Parameter for encrypt helper function	

	struct sha204_nonce_parameters nonce_parameters;
	struct sha204_gen_dig_parameters gen_dig_parameters;
	struct sha204_write_parameters write_parameters;
	
	//tony comment: encrypt write step 1
	// Wake the device, validate its presence and put it back to sleep.
	//sha204_lib_return |= atsha204_wakeup_and_validate_device();
	sha204_lib_return |= sha204c_wakeup(wakeup_response_buffer);
	
	sha204_lib_return |= sha204p_wakeup();

	if(SHA204_SUCCESS!=sha204_lib_return)
	{
		return sha204_lib_return;
	}

	//-----------tony comment:ATSHA204 side operate------
	//tony comment: encrypt write step 2-----ATSH204 side calulate tempkey and return RN(Random Number)
	// Execute the nonce command - validates TempKey flag.
	nonce_parameters.tx_buffer = transmit_buffer;
	nonce_parameters.rx_buffer = response_buffer;
	nonce_parameters.mode = NONCE_MODE_NO_SEED_UPDATE;
	nonce_parameters.num_in = NumIn;
	
	sha204_lib_return |= sha204m_nonce(&nonce_parameters);

	//-----------tony comment:MCU side operate------
	//tony comment: encrypt write step 3-----MCU side calculate tempkey
	// Prepare parameters and nonce in host.
	// Initialize parameter for helper function
	mcu_nonce_parameters.mode = NONCE_MODE_NO_SEED_UPDATE;
	mcu_nonce_parameters.num_in = NumIn;
	mcu_nonce_parameters.rand_out = &response_buffer[1];//RN(Random Number)
	mcu_nonce_parameters.temp_key = &mcu_tempkey;

	sha204_lib_return |= sha204h_nonce(mcu_nonce_parameters);

	//-----------tony comment:ATSHA204 side operate------
	//tony comment: encrypt write step 4-----ATSH204 side claulate tempkey useing tempkey and key[key_id] as input for sha256 
	// Execute GenDig command in device to prepare TempKey
	gen_dig_parameters.tx_buffer = transmit_buffer;
	gen_dig_parameters.rx_buffer = response_buffer;	
	gen_dig_parameters.zone = GENDIG_ZONE_DATA;
	gen_dig_parameters.key_id = key_id;
	gen_dig_parameters.other_data = NULL;

	sha204_lib_return |= sha204m_gen_dig(&gen_dig_parameters);
	
	//-----------tony comment:MCU side operate------
	//tony comment: encrypt write step 5-----MCU side calculate tempkey useing tempkey and key[key_id] as input for sha256 
	// Prepare host software to compute equivalent GenDig information
	// Initialize parameter for helper function
	mcu_gendig_param.zone = GENDIG_ZONE_DATA;
	mcu_gendig_param.key_id = key_id;
	mcu_gendig_param.stored_value = secret_key;
	mcu_gendig_param.temp_key = &mcu_tempkey;

	sha204_lib_return |= sha204h_gen_dig(mcu_gendig_param);

	//-----------tony comment:MCU side operate------
	//tony comment: encrypt write step 6-----
	memcpy(enc_data, clear_data, 32);
	
	mcu_enc_param.zone = SHA204_ZONE_DATA|WRITE_ZONE_MODE_32_BYTES;
	mcu_enc_param.address = slot_to_write;
	mcu_enc_param.data = enc_data;
	mcu_enc_param.mac = mac;
	mcu_enc_param.temp_key = &mcu_tempkey;
	sha204_lib_return |= sha204h_encrypt(mcu_enc_param);

	//-----------tony comment:ATSHA204 side operate------
	//tony comment: encrypt read step 7----- 
	// Device Operation Parameters
	write_parameters.tx_buffer = transmit_buffer;
	write_parameters.rx_buffer = response_buffer;
	write_parameters.zone = SHA204_ZONE_DATA|WRITE_ZONE_MODE_32_BYTES;
	write_parameters.mac = mac;
	
	write_parameters.address = 4 * slot_to_write;//change double word address to byte word address
	write_parameters.new_value = enc_data;
	
	// Observe the data being sent to confirm it is encrypted.
	sha204_lib_return |= sha204m_write(&write_parameters);//write 32 bytes

	//-----------tony comment:ATSHA204 side operate------
	//tony comment: encrypt read step 8----- 
	// Send Sleep command.
	mdelay(100);
	//delay_ms(100);
	sha204_lib_return |= sha204p_sleep();	
	
	// Compare the data 
	//sha204_lib_return |= memcmp(clear_data,&response_buffer[1], 32);

	return sha204_lib_return;
}

uint8_t atsha204_enc_read(uint16_t slot_to_read, uint8_t* clear_data, uint16_t key_id, uint8_t* secret_key, uint8_t* NumIn) 
{
	static uint8_t sha204_lib_return = SHA204_SUCCESS;			//!< Function execution status, initialized to SUCCES and bitmasked with error codes as needed.
	uint8_t transmit_buffer[SHA204_CMD_SIZE_MAX];	//!< Transmit data buffer
	uint8_t response_buffer[SHA204_RSP_SIZE_MAX];	//!< Receive data buffer
	uint8_t dec_data[32];							//!< Buffer to hold the hostdecrypted data
	struct sha204h_nonce_in_out mcu_nonce_param;		//!< Parameter for nonce helper function
	struct sha204h_gen_dig_in_out mcu_gendig_param;		//!< Parameter for gendig helper function
	struct sha204h_temp_key mcu_tempkey;				//!< Tempkey parameter for nonce and mac helper function
	struct sha204h_decrypt_in_out mcu_dec_param;		//!< Parameter for decrypt helper function

	struct sha204_nonce_parameters nonce_parameters;
	struct sha204_gen_dig_parameters gen_dig_parameters;
	struct sha204_read_parameters read_parameters;

	
	//tony comment: encrypt read step 1
	// Wake the device, validate its presence and put it back to sleep.
	//sha204_lib_return |= atsha204_wakeup_and_validate_device();
	sha204_lib_return |= sha204c_wakeup(wakeup_response_buffer);
	sha204_lib_return |= sha204p_wakeup();

	if(SHA204_SUCCESS!=sha204_lib_return)
	{
		return sha204_lib_return;
	}

	//-----------tony comment:ATSHA204 side operate------
	//tony comment: encrypt read step 2-----ATSH204 side calulate tempkey and return RN(Random Number)	
	// Execute the nonce command - validates TempKey flag.
	nonce_parameters.tx_buffer = transmit_buffer;
	nonce_parameters.rx_buffer = response_buffer;
	nonce_parameters.mode = NONCE_MODE_NO_SEED_UPDATE;
	nonce_parameters.num_in = NumIn;
	
	sha204_lib_return |= sha204m_nonce(&nonce_parameters);

	//-----------tony comment:MCU side operate------
	//tony comment: encrypt read step 3-----MCU side calculate tempkey
	// Initialize parameter for helper function
	// Prepare parameters and nonce in host.
	mcu_nonce_param.mode = NONCE_MODE_NO_SEED_UPDATE;
	mcu_nonce_param.num_in = NumIn;	
	mcu_nonce_param.rand_out = &response_buffer[1];	
	mcu_nonce_param.temp_key = &mcu_tempkey;
	sha204_lib_return |= sha204h_nonce(mcu_nonce_param);

	//-----------tony comment:ATSHA204 side operate------
	//tony comment: encrypt read step 4-----ATSH204 side claulate tempkey useing tempkey and key[key_id] as input for sha256 	
	// Execute GenDig command in device to prepare TempKey
	gen_dig_parameters.tx_buffer = transmit_buffer;
	gen_dig_parameters.rx_buffer = response_buffer;	
	gen_dig_parameters.zone = GENDIG_ZONE_DATA;
	gen_dig_parameters.key_id = key_id;
	gen_dig_parameters.other_data = NULL;

	sha204_lib_return |= sha204m_gen_dig(&gen_dig_parameters);

	if(SHA204_CMD_FAIL == sha204_lib_return)
	{
      sha204p_sleep();	
		return sha204_lib_return;
	}
	
	//-----------tony comment:MCU side operate------
	//tony comment: encrypt read step 5-----MCU side calculate tempkey useing tempkey and key[key_id] as input for sha256 
	// Prepare host software to compute equivalent GenDig information
	// Initialize parameter for helper function
	mcu_gendig_param.zone = GENDIG_ZONE_DATA;
	mcu_gendig_param.key_id = key_id;
	mcu_gendig_param.stored_value = secret_key;
	mcu_gendig_param.temp_key = &mcu_tempkey;

	sha204_lib_return |= sha204h_gen_dig(mcu_gendig_param);

	//-----------tony comment:SHA204 side operate------
	//tony comment: encrypt read step 6-----
	//struct sha204_read_parameters read_parameters;
	read_parameters.tx_buffer = transmit_buffer;
	read_parameters.rx_buffer = response_buffer;
	read_parameters.zone = SHA204_ZONE_DATA|SHA204_ZONE_COUNT_FLAG;
	read_parameters.address = 4 * slot_to_read;
	
	sha204_lib_return |= sha204m_read(&read_parameters);	//read 32 bytes	
	if(SHA204_CMD_FAIL == sha204_lib_return)
	{
      sha204p_sleep();	
		return sha204_lib_return;
	}


	//tony comment: encrypt read step 7	
	// Capture the read data and store in buffers for decryption and reference
	memcpy(dec_data, &response_buffer[1], 32);

	//tony comment: encrypt read step 8
	// Prepare host helper parameter and decrypt the data
	mcu_dec_param.data = dec_data;
	mcu_dec_param.temp_key = &mcu_tempkey;
	sha204_lib_return |= sha204h_decrypt(mcu_dec_param);	
	
	//tony comment: encrypt read step 9	
	// Compare the data 
	sha204_lib_return |= memcmp(dec_data,clear_data, 32);	

	//tony comment: encrypt read step 10
	sha204p_sleep();

	return sha204_lib_return;
}


//************************************
// Method:    atsha204_mac
// FullName:  atsha204_mac : Performs a Nonce, sends a host challenge, receives and validates the response.
// Access:    public 
// Returns:   uint8_t
// Qualifier:
// Parameter: uint16_t key_id
//					The ATSHA204 key ID for the key to use in this MAC operation
// Parameter: uint8_t key[32]
//					The actual key value.  An authentic host system will know this value.
//					Because the host may not be secure, unprotected storage of this key becomes a source of vulnerability.  Atmel
//					advices operation in a secure MCU or use of another ATSHA204 device in a host system to protect the key AND
//					also securely perform host operations.
//
// Parameter: uint8_t NumIn[NONCE_NUMIN_SIZE_PASSTHROUGH]
//					This is a 20-byte or 32-byte NumIn parameter of the Nonce command.  It is advisable to make this a system
//					provided varying value.
//************************************
uint8_t atsha204_mac(uint16_t key_id,uint8_t* secret_key, uint8_t* NumIn, uint8_t* challenge) 
{
	int i;
	static uint8_t sha204_lib_return = SHA204_SUCCESS;			//!< Function execution status, initialized to SUCCES and bitmasked with error codes as needed.
	uint8_t transmit_buffer[SHA204_CMD_SIZE_MAX];	//!< Transmit data buffer
	uint8_t response_buffer[SHA204_RSP_SIZE_MAX];	//!< Receive data buffer
	uint8_t soft_digest [32];						//!< Software calculated digest
	struct sha204h_nonce_in_out nonce_param;		//!< Parameter for nonce helper function
	struct sha204h_mac_in_out mac_param;			//!< Parameter for mac helper function
	struct sha204h_temp_key tempkey;				//!< tempkey parameter for nonce and mac helper function

	struct sha204_mac_parameters mac;
	struct sha204_nonce_parameters nonce_parameters;

	//-----------tony comment:MCU side operate------
	//tony comment: MAC step 1	
	// Wake the device, validate its presence and put it back to sleep.
	//sha204_lib_return |= atsha204_wakeup_and_validate_device();
	sha204_lib_return |= sha204c_wakeup(wakeup_response_buffer);
	
	sha204_lib_return |= sha204p_wakeup();
	if(SHA204_SUCCESS!=sha204_lib_return)
	{
		printk("sha204 wakeup failed\n");
		return sha204_lib_return;
	}

	//-----------tony comment:ATSHA204 side operate------
	//tony comment: MAC step 2-----ATSH204 side calulate tempkey and return RN(Random Number)
	// Execute the nonce command - precedes all MAC commands.
	nonce_parameters.tx_buffer = transmit_buffer;
	nonce_parameters.rx_buffer = response_buffer;
	nonce_parameters.mode = NONCE_MODE_NO_SEED_UPDATE;
	nonce_parameters.num_in = NumIn;
	
	sha204_lib_return |= sha204m_nonce(&nonce_parameters);

	//-----------tony comment:MCU side operate------
	//tony comment: MAC step 3-----MCU side calculate tempkey
	// Initialize parameter for helper function
	// Initialize parameter for helper function
	nonce_param.mode = NONCE_MODE_NO_SEED_UPDATE;
	nonce_param.num_in = NumIn;	
	nonce_param.rand_out = &response_buffer[1];	
	nonce_param.temp_key = &tempkey;
	sha204_lib_return |= sha204h_nonce(nonce_param);

	
	//-----------tony comment:ATSHA204 side operate------
	//tony comment: MAC step 4-----ATSHA204 MAC
	// Execute the MAC command which constitutes sending a challenge. Successful execution will yield a result that contains the "Challenge Response" to be validated later in this function.
	mac.mode = MAC_MODE_BLOCK2_TEMPKEY;
	mac.key_id = key_id;
	mac.challenge = challenge;
	mac.tx_buffer = transmit_buffer;
	mac.rx_buffer = response_buffer;

	sha204_lib_return = sha204m_mac(&mac);

	//-----------tony comment:MCU side operate------
	//tony comment: MAC step 5-----MCU MAC
	// Collect required information needed by a host system to calculate the expected challenge response in software, then perform the calculation.
	//mac_param.mode = MAC_MODE_BLOCK1_TEMPKEY|MAC_MODE_BLOCK2_TEMPKEY;
	mac_param.mode = MAC_MODE_BLOCK2_TEMPKEY;
	mac_param.key_id = key_id;
	mac_param.challenge = challenge;
	mac_param.key = secret_key;
	mac_param.otp = NULL;
	mac_param.sn = NULL;
	mac_param.response = soft_digest;
	mac_param.temp_key = &tempkey;
	sha204_lib_return |= sha204h_mac(mac_param);
	
#if 1
	for (i = 0; i < 32; i++)
	{
		printk("soft_digest[%d] = %x, response_buffer[%d] = %x\n", i, soft_digest[i], i, response_buffer[1 + i]);
	}
#endif
	
	
	//-----------tony comment:ATSHA204 side operate------
	//tony comment: MAC step 6-----MCU MAC
	// Send Sleep command.
	sha204_lib_return |= sha204p_sleep();	
	
	// Moment of truth!  Compare the chip generated digest found in 'response_buffer' with the host software calculated digest found in 'soft_digest'.
	sha204_lib_return |= memcmp(soft_digest,&response_buffer[1],32);

	return sha204_lib_return;
}

//========================================================================================================================





uint8_t atsha204_slot02_personalization(void) 
{
	static uint8_t sha204_lib_return = SHA204_SUCCESS;
	static uint8_t transmit_buffer[SHA204_CMD_SIZE_MAX];
	static uint8_t response_buffer[SHA204_RSP_SIZE_MAX]; 

	struct sha204_write_parameters write_parameters;	
	struct sha204_read_parameters read_parameters;
	struct sha204_lock_parameters lock_parameters;
	

	// Wake the device, validate its presence and put it back to sleep.
	//sha204_lib_return |= atsha204_wakeup_and_validate_device();
	sha204_lib_return |= sha204c_wakeup(wakeup_response_buffer);

	if(SHA204_SUCCESS != sha204_lib_return)
	{
		return sha204_lib_return;
	}

	/*!
	 *	*** ENTER PERSONALIZATON PREFERENCES INTO THE CONFIGURATION MEMORY ***
	 */
	//-----------tony comment:ATSHA204 side operate---------------------------
	//tony comment: personalization step 1-----	 
	// Device Operation Parameters
	write_parameters.tx_buffer = transmit_buffer;
	write_parameters.rx_buffer = response_buffer;
	write_parameters.zone = SHA204_ZONE_CONFIG;
	write_parameters.mac = NULL;

	memset(response_buffer, 0, sizeof(response_buffer));
	
	write_parameters.address = 4 * DEVICE_MODES_ADDRESS;//change double word address to byte word address
	write_parameters.new_value = &DEVICE_MODES[0];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();

	// Slots 2 and 3
	write_parameters.address = 4 * SLOT_CONFIG_2_3_ADDRESS;
	write_parameters.new_value = SLOT_CONFIG_02_03;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	


	// *** USE FLAG and UPDATE COUNT Region
	
	// Slots 2 and 3
	write_parameters.address = 4 * SLOT_2_3_USE_UPDATE_ADDRESS;
	write_parameters.new_value = SLOT_2_3_USE_UPDATE;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	
	
	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 3-----
	// *** LAST KEY USE Region ***
	// First word
	write_parameters.address = 4* (LAST_KEY_USE_ADDRESS + 0);
	write_parameters.new_value = &LAST_KEY_USE[0];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	
	
	// Second word
	write_parameters.address = 4 * (LAST_KEY_USE_ADDRESS+1);
	write_parameters.new_value = &LAST_KEY_USE[4];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	// Third word
	write_parameters.address = 4 * (LAST_KEY_USE_ADDRESS + 2);
	write_parameters.new_value = &LAST_KEY_USE[8];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	
	
	// Fourth word
	write_parameters.address = 4 * (LAST_KEY_USE_ADDRESS + 3);
	write_parameters.new_value = &LAST_KEY_USE[12];
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);//write 4 bytes
	sha204_lib_return |= sha204p_sleep();	

	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 4-----	
	//*	*** OPTIONAL READ and VERIFY ****
	// The read data is contained in the response buffer.  Note that the buffer does not accumulate data. It's content is refreshed on each read so break and inspect buffer after each command execution.
	sha204_lib_return |= sha204p_wakeup();

	read_parameters.tx_buffer = transmit_buffer;
	read_parameters.rx_buffer = response_buffer;
	read_parameters.zone = SHA204_ZONE_CONFIG | SHA204_ZONE_COUNT_FLAG;
	read_parameters.address = 4 * CONFIG_BLOCK_0_ADDRESS;

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 32 bytes	

	read_parameters.address = 4 * CONFIG_BLOCK_1_ADDRESS;

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 32 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 0);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 1);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	
	
	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address =4 *  (CONFIG_BLOCK_2_ADDRESS + 2);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 3);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 4);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * (CONFIG_BLOCK_2_ADDRESS + 5);

	sha204_lib_return |= sha204m_read(&read_parameters);	//read 4 bytes	

	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 5-----	
	/*!
	 *	*** LOCK CONFIG ***
	 **********************
	 *
	 *	Forever lock the custom configuration from future modification.  Writing of Data or OTP regions requires prior execution of this command.
	 */
	 lock_parameters.tx_buffer = transmit_buffer;
	 lock_parameters.rx_buffer = response_buffer;
	 lock_parameters.zone = LOCK_ZONE_NO_CRC;
	 lock_parameters.summary = LOCK_PARAM2_NO_CRC;
	 
	 sha204_lib_return |= sha204m_lock(&lock_parameters);

	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 6-----	
	/*!
	 *	*** WRITE INITIAL DATA TO DATA SLOTS ***
	 *******************************************
	 *
	 *  Write initial content to data slots.  This is the only opportunity to to write non-modifiable information e.g. model numbers and certain keys.
	 */
	write_parameters.tx_buffer = transmit_buffer;
	write_parameters.rx_buffer = response_buffer;
	write_parameters.zone = SHA204_ZONE_DATA | SHA204_ZONE_COUNT_FLAG;
	write_parameters.mac = NULL;
	

	// Write initial content for slot 2
	write_parameters.address = 4 * SLOT_2_ADDRESS;
	write_parameters.new_value = SLOT_02_CONTENT;	
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		

	// Write initial content for slot 3	
	write_parameters.address = 4 * SLOT_3_ADDRESS;
	write_parameters.new_value = SLOT_03_CONTENT;
	
	sha204_lib_return |= sha204p_wakeup();
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();		


	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 7-----		
	/*!
	 *	*** WRITE INITIAL OTP DATA INTO THE OTP REGION ***
	 *****************************************************
	 *  Write initial information to the OTP region.  This is the only opportunity to do so.  After locking data and OTP regions, write accesses to this region will be controlled by 
	 *  custom access privileges defined in the configuration region.
	 */
	write_parameters.zone = SHA204_ZONE_OTP| SHA204_ZONE_COUNT_FLAG; 
	write_parameters.address = 4 * OTP_BLOCK_0_ADDRESS;
	write_parameters.new_value = &OTP[0];
	
	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();	

	write_parameters.address = 4 * OTP_BLOCK_1_ADDRESS;
	write_parameters.new_value = &OTP[8];	
	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_write(&write_parameters);
	sha204_lib_return |= sha204p_sleep();	
	
	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 8-----	
	/*!
	 *	*** LOCK VALUE ***
	 *********************
	 *
	 *	Forever lock the data and OTP regions.  After lock data, access to these regions will be controlled by access rights defined in the configuration region.
	 */
	 lock_parameters.zone = LOCK_ZONE_NO_CONFIG|LOCK_ZONE_NO_CRC;
	 lock_parameters.summary = LOCK_PARAM2_NO_CRC;
	 
	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_lock(&lock_parameters);	
	sha204_lib_return |= sha204p_sleep();		
	/*!
	 *	*** VERIFY SUCCESSFUL COMPLETION OF THE PERSONALIZATION PROCESS ***
	 **********************************************************************
	 *
	 * Check that all functions executed without errors and that the chip is actually locked.
	 */	 
	 
	//-----------tony comment:ATSHA204 side operate----------------------------
	//tony comment: personalization step 9-----		
	// Read lock bytes and verify successful lock
	#if 0
	read_parameters.tx_buffer = transmit_buffer;
	read_parameters.rx_buffer = response_buffer;	
	read_parameters.zone = SHA204_ZONE_CONFIG;
	read_parameters.address = 4 * EXTRA_SELECTOR_LOCK_ADDRESS;	

	sha204_lib_return |= sha204p_wakeup();	
	sha204_lib_return |= sha204m_read(&read_parameters);
	sha204_lib_return |= sha204p_sleep();		
	
	sha204_lib_return |= response_buffer[3] /* LockValue */ & response_buffer[4] /* LockConfig */; 
	#endif

	return sha204_lib_return;
}

static int result = 0;

static ssize_t serial_number_show(struct device *dev,struct device_attribute *attr,char* buf)
{
	if (result)
		return sprintf(buf,"%s\n", "success!");
	else
		return sprintf(buf,"%s\n", "fail!");
}

static ssize_t serial_number_store(struct device *dev,struct device_attribute *attr, const char* buf,size_t size)
{
	int retval;
	//int key_id = 2;

	result = 0;

	retval = atsha204_device_personalization();
	if (retval == 0)
		result = 1;

	return size;
}

static DEVICE_ATTR(SerialNumber, 0666, serial_number_show, serial_number_store);

static int sha204_probe(struct platform_device *dev)
{
	int retval;
	int key_id = 2;
	//we need use key_id 2, if not,reset!
	char key[] = {0x14, 0x15, 0x63, 0x37, 0x28, 0x45, 0x73, 0x94, 0x51, 0x34,
				 0x61, 0x92, 0x79, 0x3b, 0xec, 0xc4, 0x29, 0xfc, 0xdf, 0x7d,
				 0x6c, 0xaa, 0x76, 0x23, 0x85, 0x12, 0x1d, 0x4e, 0x53, 0x8e,
				 0xe1, 0xd3};
	
	char num_in[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10,
				     0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20};

	char challenge[] = {0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7, 0xf6,
						0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10,
						0x6c, 0xaa, 0x76, 0x23, 0x85, 0x12, 0x1d, 0x4e, 0x23, 0x8e,
						0xe1, 0xd3};
						
	retval = atsha204_mac(key_id, key, num_in, challenge);
	//printk("sha204_mac retval = %d\n", retval);
	if (retval != 0)
	{
		//reset  
		printk("sha204_module reset ...\n");
		//mub_update1(0x0101, 0x05);	
	}

	retval=device_create_file(&dev->dev, &dev_attr_SerialNumber);
	if (retval)
		printk("could not create file attrbute: SerialNumber\n");

	return 0;
}

static int sha204_remove(struct platform_device *dev)
{

	device_remove_file(&dev->dev,&dev_attr_SerialNumber);

	return 0;
}

static struct platform_device sha204_device = {
	.name = "sha204",
	.id = 0,
    .dev = {
        .coherent_dma_mask = 0xffffffffUL	
    },
};

static struct platform_driver sha204_driver = {
	.driver = {
		.name = "sha204",
		.owner = THIS_MODULE,
	},
	.probe	= sha204_probe,
	.remove = sha204_remove,
};

static int __init sha204_init(void)
{
	int ret = 0;
	printk("sha204 init entry ...\n");
	ret = platform_driver_register(&sha204_driver);
	if (!ret)
	{
		ret = platform_device_register(&sha204_device);
		if (ret)
			platform_driver_unregister(&sha204_driver);
	}

	return ret;
}

static void __exit sha204_exit(void)
{
	platform_device_unregister(&sha204_device);
	platform_driver_unregister(&sha204_driver);
}

module_init(sha204_init);
module_exit(sha204_exit);

MODULE_LICENSE("GPL");
