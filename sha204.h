#ifndef __SYSTEM_AT88_H__
#define __SYSTEM_AT88_H__


#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/spi/ads7846.h>
#include <linux/utsname.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/times.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
//#include <mach/hardware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

typedef unsigned char uint8_t;
typedef unsigned char uint8;
typedef unsigned int  uint16;
typedef unsigned long uint32;


//===================================================================
//software_IIC.h

#define GPIO_OUT_LOW    0
#define GPIO_OUT_HIGH   1

#define ACK    0
#define NACK   1

#define IMX_GPIO_NR(a, b) (a << b)
//GPIO3_22
#define SDA_PIN  IMX_GPIO_NR(3, 22)
//GPIO3_20
#define SCL_PIN  IMX_GPIO_NR(3, 20)

//IO方向设置
#define SDA_IN()  gpio_direction_input(SDA_PIN)  //输入，上/下拉输入
#define SDA_OUT() //gpio_direction_output(SDA_PIN, 1)  //输出，推完输出

//IO操作函数	 
#define IIC_SCL(out_voltage_level)    {if (out_voltage_level) gpio_direction_output(SCL_PIN, 1); else gpio_direction_output(SCL_PIN, 0);}  //SCL
#define IIC_SDA(out_voltage_level)    {if (out_voltage_level) gpio_direction_output(SDA_PIN,1); else gpio_direction_output(SDA_PIN, 0);}  //SDA
#define READ_SDA()    gpio_get_value(SDA_PIN) //输入SDA 



// error codes for physical hardware dependent module
// Codes in the range 0x00 to 0xF7 are shared between physical interfaces (SWI, TWI, SPI).
// Codes in the range 0xF8 to 0xFF are special for the particular interface.
#define I2C_FUNCTION_RETCODE_SUCCESS     ((uint8_t) 0x00) //!< Communication with device succeeded.
#define I2C_FUNCTION_RETCODE_COMM_FAIL   ((uint8_t) 0xF0) //!< Communication with device failed.
#define I2C_FUNCTION_RETCODE_TIMEOUT     ((uint8_t) 0xF1) //!< Communication timed out.
#define I2C_FUNCTION_RETCODE_NACK        ((uint8_t) 0xF8) //!< TWI nack

void SCL_out_low(void);
void SCL_out_high(void);

//SDA  --------_______
//SCL  __---------_____
void software_IIC_start(void);

//SDA_____------____
//SCL___------______
void software_IIC_stop(void);
bool software_IIC_read_ack(void);
void software_IIC_send_ack_nack(bool ack_nack);

void software_IIC_send_byte(uint8_t data);

uint8_t software_IIC_read_byte(void);

#define SOFTWARE_IIC_RESTART() software_IIC_start();



uint8_t i2c_send_bytes(uint8_t count,uint8_t *data);
uint8_t i2c_receive_byte(uint8_t *data);
uint8_t i2c_receive_bytes(uint8_t count,uint8_t *data);

//=============================================================
//sha204_config.h

#define SHA204_I2C

#define SHA204_I2C_DEFAULT_ADDRESS  (0xC8)

#define SHA204_CHECK_PARAMETERS



/** \brief number of command / response retries
 *
 * If communication is lost, re-synchronization includes waiting for the
 * longest possible execution time of a command.
 * This adds a #SHA204_COMMAND_EXEC_MAX delay to every retry.
 * Every increment of the number of retries increases the time
 * the library is spending in the retry loop by #SHA204_COMMAND_EXEC_MAX.
 */
#define SHA204_RETRY_COUNT             (2)

//////////////////////////////////////////////////////////////////////////
///////////// definitions specific to interface //////////////////////////
//////////////////////////////////////////////////////////////////////////

//! Define this if you are using GPIO for SWI communication.
//#define SHA204_SWI_BITBANG

//! Define this if you are using a UART for SWI communication.
//#define SHA204_SWI_UART

//! Define this if you are using TWI communication.
//#define SHA204_TWI

////////////////////////////// GPIO configurations //////////////////////////////
#ifdef SHA204_SWI_BITBANG

//! #START_PULSE_TIME_OUT in us instead of loop counts
#   define SWI_RECEIVE_TIME_OUT      ((uint16_t) 163)

//! It takes 312.5 us to send a byte (9 single-wire bits / 230400 Baud * 8 flag bits).
#   define SWI_US_PER_BYTE           ((uint16_t) 313)
#endif


////////////////////////////// UART configurations //////////////////////////////
#ifdef SHA204_SWI_UART

//! receive timeout in us instead of loop counts
#   define SWI_RECEIVE_TIME_OUT      ((uint16_t) 153)

//! It takes 312.5 us to send a byte (9 single-wire bits / 230400 Baud * 8 flag bits).
#   define SWI_US_PER_BYTE           ((uint16_t) 313)

//! SWI response timeout is the sum of receive timeout and the time it takes to send the TX flag.
#   ifndef SHA204_RESPONSE_TIMEOUT
#      define SHA204_RESPONSE_TIMEOUT   ((uint16_t) SWI_RECEIVE_TIME_OUT + SWI_US_PER_BYTE)
#   endif

#endif


///////// SWI configurations common for GPIO and UART implementation ////////////////////////
#if defined(SHA204_SWI_BITBANG) || defined(SHA204_SWI_UART)

//! delay before sending a transmit flag in the synchronization routine
#   define SHA204_SYNC_TIMEOUT       ((uint8_t) 85)

#   ifndef SHA204_RESPONSE_TIMEOUT
//! SWI response timeout is the sum of receive timeout and the time it takes to send the TX flag.
#      define SHA204_RESPONSE_TIMEOUT   ((uint16_t) SWI_RECEIVE_TIME_OUT + SWI_US_PER_BYTE)
#   endif

#endif


////////////////////////////// TWI configurations //////////////////////////////
#ifdef SHA204_I2C

/** \brief For I2C, the response polling time is the time
 *         it takes to send the I2C address.
 *
 *         This value is used to timeout when waiting for a response.
 */
#   ifndef SHA204_RESPONSE_TIMEOUT
#      define SHA204_RESPONSE_TIMEOUT     ((uint16_t) 37)
#   endif

#endif
//================================================================
//sha204_physical.h
#define SHA204_RSP_SIZE_MIN          ((uint8_t)  4)  //!< minimum number of bytes in response
#define SHA204_RSP_SIZE_MAX          ((uint8_t) 35)  //!< maximum size of response packet

#define SHA204_BUFFER_POS_COUNT      (0)             //!< buffer index of count byte in command or response
#define SHA204_BUFFER_POS_DATA       (1)             //!< buffer index of data in response

//! width of Wakeup pulse in 10 us units
#define SHA204_WAKEUP_PULSE_WIDTH    (6)

//! delay between Wakeup pulse and communication in ms
#define SHA204_WAKEUP_DELAY          (3)
/**
 * \defgroup sha204_communication_physical_group SHA204 Service - hardware dependent communication functions
 *
 * @{
 */
void    sha204p_init(void);
void    sha204p_set_device_id(uint8_t id);
uint8_t sha204p_send_command(uint8_t count, uint8_t *command);
uint8_t sha204p_receive_response(uint8_t size, uint8_t *response);
uint8_t sha204p_wakeup(void);
uint8_t sha204p_idle(void);
uint8_t sha204p_sleep(void);
uint8_t sha204p_reset_io(void);
uint8_t sha204p_resync(uint8_t size, uint8_t *response);

//==============================================================
//sha204_comm.h
#define SHA204_COMMAND_EXEC_MAX      (69)

//! minimum number of bytes in command (from count byte to second CRC byte)
#define SHA204_CMD_SIZE_MIN          ((uint8_t)  7)

//! maximum size of command packet (CheckMac)
#define SHA204_CMD_SIZE_MAX          ((uint8_t) 84)

//! number of CRC bytes
#define SHA204_CRC_SIZE              ((uint8_t)  2)

//! buffer index of status byte in status response
#define SHA204_BUFFER_POS_STATUS     (1)

//! buffer index of first data byte in data response
#define SHA204_BUFFER_POS_DATA       (1)

//! status byte after wake-up
#define SHA204_STATUS_BYTE_WAKEUP    ((uint8_t) 0x11)

//! command parse error
#define SHA204_STATUS_BYTE_PARSE     ((uint8_t) 0x03)

//! command execution error
#define SHA204_STATUS_BYTE_EXEC      ((uint8_t) 0x0F)

//! communication error
#define SHA204_STATUS_BYTE_COMM      ((uint8_t) 0xFF)

/** 
 * \brief This structure contains the parameters for the \ref sha204c_send_and_receive function.
 */
struct sha204_send_and_receive_parameters {
	uint8_t *tx_buffer;         //!< pointer to send buffer
	uint8_t rx_size;            //!< size of receive buffer
	uint8_t *rx_buffer;         //!< pointer to receive buffer
	uint8_t poll_delay;         //!< how long to wait before polling for response-ready
	uint8_t poll_timeout;       //!< how long to poll before timing out
};

/**
 * \defgroup sha204_communication_group SHA204 Service - hardware independent communication functions
 * @{
 */

uint8_t sha204c_resync(uint8_t size, uint8_t *response);

void sha204c_calculate_crc(uint8_t length, uint8_t *data, uint8_t *crc);
uint8_t sha204c_wakeup(uint8_t *response);
uint8_t sha204c_send_and_receive(struct sha204_send_and_receive_parameters *args);

//=====================================================================================
//atsha204_wakeup_and_validate_device.h

uint8_t atsha204_wakeup_and_validate_device(void);


//=============================================================================
//atsha204_defines.h
#define NONCE_PARAM2					((uint16_t) 0x0000)		//nonce param2. always zero
#define HMAC_MODE_EXCLUDE_OTHER_DATA	((uint8_t) 0x00)		//!< HMAC mode excluded other data
#define HMAC_MODE_INCLUDE_OTP_88		((uint8_t) 0x10)		//!< HMAC mode bit   4: include first 88 OTP bits
#define HMAC_MODE_INCLUDE_OTP_64		((uint8_t) 0x20)		//!< HMAC mode bit   5: include first 64 OTP bits
#define HMAC_MODE_INCLUDE_SN			((uint8_t) 0x40)		//!< HMAC mode bit   6: include serial number
#define DERIVE_KEY_RANDOM_NONCE			((uint8_t) 0x00)		//Derive key mode using random nonce
#define MAC_MODE_NO_TEMPKEY				((uint8_t) 0x00)		//MAC mode using internal key and challenge to get MAC result
#define LOCK_PARAM2_NO_CRC				((uint16_t) 0x0000)		//Lock mode : not using checksum to validate the data written
#define CHECKMAC_PASSWORD_MODE			((uint8_t) 0X01)		//CheckMac mode : password check operation

/*!
 *	*** DEVICE Modes Address ***
 */
#define DEVICE_MODES_ADDRESS			((uint16_t) 0x0004)   //Double word address
#define DEVICE_MODES_BYTE_SIZE			(4)			


//Key ID in 16 bit boundaries
#define KEY_ID_0						((uint16_t) 0x0000)
#define KEY_ID_1						((uint16_t) 0x0001)
#define KEY_ID_2						((uint16_t) 0x0002)
#define KEY_ID_3						((uint16_t) 0x0003)
#define KEY_ID_4						((uint16_t) 0x0004)
#define KEY_ID_5						((uint16_t) 0x0005)
#define KEY_ID_6						((uint16_t) 0x0006)
#define KEY_ID_7						((uint16_t) 0x0007)
#define KEY_ID_8						((uint16_t) 0x0008)
#define KEY_ID_9						((uint16_t) 0x0009)
#define KEY_ID_10						((uint16_t) 0x000A)
#define KEY_ID_11						((uint16_t) 0x000B)
#define KEY_ID_12						((uint16_t) 0x000C)
#define KEY_ID_13						((uint16_t) 0x000D)
#define KEY_ID_14						((uint16_t) 0x000E)
#define KEY_ID_15						((uint16_t) 0x000F)

//Slot configuration address
#define SLOT_CONFIG_0_1_ADDRESS			((uint16_t) 0x0005)
#define SLOT_CONFIG_2_3_ADDRESS			((uint16_t) 0x0006)
#define SLOT_CONFIG_4_5_ADDRESS			((uint16_t) 0x0007)
#define SLOT_CONFIG_6_7_ADDRESS			((uint16_t) 0x0008)
#define SLOT_CONFIG_8_9_ADDRESS			((uint16_t) 0x0009)
#define SLOT_CONFIG_10_11_ADDRESS		((uint16_t) 0x000A)
#define SLOT_CONFIG_12_13_ADDRESS		((uint16_t) 0x000B)
#define SLOT_CONFIG_14_15_ADDRESS		((uint16_t) 0x000C)

//Slot key address
#define SLOT_0_ADDRESS					((uint16_t) 0x0000) //Word Address 
#define SLOT_1_ADDRESS					((uint16_t) 0x0008)
#define SLOT_2_ADDRESS					((uint16_t) 0x0010)
#define SLOT_3_ADDRESS					((uint16_t) 0x0018)
#define SLOT_4_ADDRESS					((uint16_t) 0x0020)
#define SLOT_5_ADDRESS					((uint16_t) 0x0028)
#define SLOT_6_ADDRESS					((uint16_t) 0x0030)
#define SLOT_7_ADDRESS					((uint16_t) 0x0038)
#define SLOT_8_ADDRESS					((uint16_t) 0x0040)
#define SLOT_9_ADDRESS					((uint16_t) 0x0048)
#define SLOT_10_ADDRESS					((uint16_t) 0x0050)
#define SLOT_11_ADDRESS					((uint16_t) 0x0058)
#define SLOT_12_ADDRESS					((uint16_t) 0x0060)
#define SLOT_13_ADDRESS					((uint16_t) 0x0068)
#define SLOT_14_ADDRESS					((uint16_t) 0x0070)
#define SLOT_15_ADDRESS					((uint16_t) 0x0078)


/*!
 * *** Read granularity and address specifiers ***
 */
#define CONFIG_READ_SHORT				((uint8_t)0x00)
#define CONFIG_READ_LONG				((uint8_t)0x80)

#define OTP_READ_SHORT					((uint8_t)0x01)
#define OTP_READ_LONG					((uint8_t)0x81)
#define OTP_BLOCK_0_ADDRESS				((uint16_t)0x0000)			//!< Base address of the first 32 bytes of the OTP region
#define OTP_BLOCK_1_ADDRESS				((uint16_t)0x0008)			//!< Base address of the second 32 bytes of the OTP region

#define DATA_READ_SHORT					((uint8_t)0x02)
#define DATA_READ_LONG					((uint8_t)0x82)

#define CONFIG_BLOCK_0_ADDRESS			((uint16_t)0x0000)
#define CONFIG_BLOCK_1_ADDRESS			((uint16_t)0x0008)
#define CONFIG_BLOCK_2_ADDRESS			((uint16_t)0x0010)


/*!
 * Word base addresses for UseFlag and UpdateCount bits 
 *	Even bytes address UseFlag
 *  Odd bytes address UpdateCount
 */
#define SLOT_0_1_USE_UPDATE_ADDRESS		((uint16_t) 0x000D)		// Word 13
#define SLOT_2_3_USE_UPDATE_ADDRESS		((uint16_t) 0x000E)		// Word 14
#define SLOT_4_5_USE_UPDATE_ADDRESS		((uint16_t) 0x000F)		// Word 15
#define SLOT_6_7_USE_UPDATE_ADDRESS		((uint16_t) 0x0010)		// Word 16

/*!
 *	*** LAST KEY USE ADDRESS AND SIZE ***
 */
#define LAST_KEY_USE_ADDRESS			((uint16_t) 0X0011)		// double Word address,word 17
#define LAST_KEY_USE_BYTE_SIZE			((uint8_t) 0x10)		// 16 bytes
/*!
 *	*** USER EXTRA, SELECTOR, and LOCK bytes address
 */
#define EXTRA_SELECTOR_LOCK_ADDRESS		((uint16_t) 0x0015)		// Word 21

//write parameter (additional)
#define WRITE_BUFFER_SIZE_SHORT			(4)						//buffer size for 4 bytes write
#define WRITE_BUFFER_SIZE_LONG			(32)					//buffer size for 32 bytes write
#define WRITE_DATA_START_IDX			(5)						//index for the first data in write buffer
#define WRITE_DATA_END_IDX_4_BYTES		(9)						//index for the last data in 4 bytes write buffer
#define WRITE_DATA_END_IDX_32_BYTES		(37)					//index for the last data in 32 bytes write buffer
#define WRITE_ZONE_MODE_32_BYTES        ((uint8_t) 0x80)		//!< write mode: 32 bytes

//read parameter (additional)
#define READ_BUFFER_SIZE_SHORT			(4)						//buffer size for 4 bytes read
#define READ_BUFFER_SIZE_LONG			(32)					//buffer size for 32 bytes write
#define READ_DATA_START_IDX				(1)						//index for the first data in read buffer
#define READ_DATA_END_IDX_4_BYTES		(5)						//index for the last data in 4 bytes read buffer
#define READ_DATA_END_IDX_32_BYTES		(33)					//index for the last data in 32 bytes write buffer


//====================================================================================
//atsha204_device_configuration.h

/*!
 *	*** DEVICE INFORMATION  ***
 *
 *	The first 16 bytes of the configuration region holds non-modifiable device information.
 */

/*!
 *  Device Operation Modes and Parameters
 * 
 *	Configure the I2C Address, Temperature sensor offset, OTP mode, and Selector Mode bytes
 */	

//													 { I2C_Address,	TempOffset,	   OTPmode,	SelectorMode}; 
uint8_t DEVICE_MODES[WRITE_BUFFER_SIZE_LONG] = {	      0xC9,       0x00,       0x55,         0x00};								


/*!
 *  *** SLOT CONFIGURATION ***
 *
 * Configure the access restrictions governing each slot
 *
 */
//													       { LSB,  MSB,    LSB,  MSB};				
//uint8_t SLOT_CONFIG_00_01[WRITE_BUFFER_SIZE_SHORT] = {0x0F, 0x20,   0xAF, 0x80};  /* Access rights for slots 0 and 1   */
uint8_t SLOT_CONFIG_00_01[WRITE_BUFFER_SIZE_SHORT] = {0x0F, 0x20,   0x8F, 0x80};  /* Access rights for slots 0 and 1   *///2014-6-12 tony comment:原厂提供的配置有问题

//uint8_t SLOT_CONFIG_02_03[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0x80,   0xAF, 0x80};  /* Access rights for slots 2 and 3   */
uint8_t SLOT_CONFIG_02_03[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0x80,   0x8F, 0x80};  /* Access rights for slots 2 and 3   *///2014-6-12tony comment :修改限制次数

//uint8_t SLOT_CONFIG_04_05[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0xB1,   0x8F, 0xA2};  /* Access rights for slots 4 and 5   */
uint8_t SLOT_CONFIG_04_05[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0x80,   0x8F, 0xA2};  /* Access rights for slots 4 and 5   *///2014-6-12tony comment :修改限制次数

uint8_t SLOT_CONFIG_06_07[WRITE_BUFFER_SIZE_SHORT] = {0xC3, 0x44,   0xCF, 0x00};  /* Access rights for slots 6 and 7   */
uint8_t SLOT_CONFIG_08_09[WRITE_BUFFER_SIZE_SHORT] = {0x0F, 0x00,   0xC1, 0x00};  /* Access rights for slots 8 and 9   */
uint8_t SLOT_CONFIG_10_11[WRITE_BUFFER_SIZE_SHORT] = {0x0F, 0x42,   0x8F, 0xB2};  /* Access rights for slots 10 and 11 */
uint8_t SLOT_CONFIG_12_13[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0x30,   0x8F, 0xA2};  /* Access rights for slots 12 and 13 */
uint8_t SLOT_CONFIG_14_15[WRITE_BUFFER_SIZE_SHORT] = {0x8F, 0x20,   0xF0, 0x80};  /* Access rights for slots 14 and 15 */

/*!
 *	*** Cconfugre USE FLAGS AND UPDATE COUNT bytes ***
 *
 * Use flags and update counts apply restrictions to enable limits on key usage.
 *	- For each 4-byte word:
 *		- Byte 0 is the UseFlag byte for the lower slot
 *		- Byte 1 is the UpdateCount byte for the lower slot
 *		- Byte 2 is the UseFlag byte for the upper slot
 *		- Byte 3 is the UpdateCount byte for the upper slot
 */
uint8_t SLOT_0_1_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0xFF,  0xFF, 0xFF};

//uint8_t SLOT_2_3_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0xFF,  0x1F, 0xFF};
uint8_t SLOT_2_3_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0xFF,  0xFF, 0xFF};//2014-6-12tony comment :修改限制次数
//uint8_t SLOT_4_5_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0x07,  0xFF, 0x3F};
uint8_t SLOT_4_5_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0xFF,  0xFF, 0x3F};//2014-6-12tony comment :修改限制次数

uint8_t SLOT_6_7_USE_UPDATE[WRITE_BUFFER_SIZE_SHORT] = {0xFF, 0xFF,  0xFF, 0xFF};


/*!
 *	***	LAST KEY USE ***
 *
 *	Control limited use for KeyID 15.  Factory defaults are 0xFF
 *
 */
uint8_t LAST_KEY_USE [LAST_KEY_USE_BYTE_SIZE] = {	0x00, 0x00, 0x00, 0x00,		// Bytes 68 - 71
																									0x00, 0x00, 0x00, 0x00,		// Bytes 72 - 75
																									0x00, 0x00, 0x00, 0x80,		// Bytes 76 - 79
																									0xFF, 0xFF, 0xFF, 0xFF		// Bytes 80 - 83																				
																								};

/*!
 *	***	Configure USER EXTRA and SELECTOR bytes ***
 *
 *	- Byte 0 configures UserExtra
 *	- Byte 1 configures Selector
 *	- Bytes 2 and 3 are ignored as they can only be modified via LOCK commands.
 *
 * These bytes are modifiable only through UpdateExtra and Lock commands only
 */

/*!
 *	*** INITIAL OTP CONTENT ***
 *
 *	512 Bits in total
 */
uint8_t OTP[2 * WRITE_BUFFER_SIZE_LONG] = {	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
												0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
											};

/*!
 *	*** INITIAL SLOT CONTENT ***
 *
 *	The initial slot content can be keys or data depending on custom security configuration.
 * 
 * The slots in this example are populated with values easy to remember for easy illustration.
 * Atmel strongly advices use of random values preferably from the high quality on-chip RNG 
 * (random number generator) for initial key values.
 *
 */

uint8_t SLOT_00_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	'A', 't', 'm', 'e' , 'l' , ' ', 'W', 'i',
														'd', 'g', 'e', 't', ' ', 'M', 'o', 'd',
														'e', 'l', ' ', '5', '5', '5', '5', 0x00,
														0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
													};

uint8_t SLOT_01_CONTENT [WRITE_BUFFER_SIZE_LONG] = {0x13, 0x13, 0xf3, 0x47, 0x22, 0x85, 0x83, 0x90, 0x71, 0x01,
													0x81, 0x72, 0x77, 0x7c, 0xec, 0x34, 0x91, 0xcc, 0xd3, 0x5d,
													0x6c, 0xaa, 0x76, 0x23, 0x85, 0x12, 0x1d, 0x4e, 0x23, 0x8e,
													0xe1, 0xd3};

uint8_t SLOT_02_CONTENT [WRITE_BUFFER_SIZE_LONG] = {0x14, 0x15, 0x63, 0x37, 0x28, 0x45, 0x73, 0x94, 0x51, 0x34,
													0x61, 0x92, 0x79, 0x3b, 0xec, 0xc4, 0x29, 0xfc, 0xdf, 0x7d,
													0x6c, 0xaa, 0x76, 0x23, 0x85, 0x12, 0x1d, 0x4e, 0x53, 0x8e,
													0xe1, 0xd3};

uint8_t SLOT_03_CONTENT [WRITE_BUFFER_SIZE_LONG] = {0x13, 0x13, 0xf3, 0x47, 0x22, 0x85, 0x83, 0x90, 0x71, 0x01,
													0x81, 0x72, 0x77, 0x7c, 0xec, 0x34, 0x91, 0xcc, 0xd3, 0x5d,
													0x6c, 0xaa, 0x76, 0x23, 0x85, 0x12, 0x1d, 0x4e, 0x23, 0x8e,
													0xe1, 0xd3};

uint8_t SLOT_04_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
															0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
															0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
															0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04
														};

uint8_t SLOT_05_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
															0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
															0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
															0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05
														};

uint8_t SLOT_06_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
															0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
															0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,
															0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06
														};

uint8_t SLOT_07_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
															0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
															0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
															0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07
														};

uint8_t SLOT_08_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
															0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
															0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
															0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08
														};

 uint8_t SLOT_09_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
															0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
															0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
															0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09
														};

uint8_t SLOT_10_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0xA,
															0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0xA,
															0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0xA,
															0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0xA
														};

uint8_t SLOT_11_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
															0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
															0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
															0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B
														};

uint8_t SLOT_12_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
															0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
															0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
															0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C
														};

uint8_t SLOT_13_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D,
															0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D,
															0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D,
															0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D
														};

uint8_t SLOT_14_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E,
															0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E,
															0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E,
															0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E
														};

uint8_t SLOT_15_CONTENT [WRITE_BUFFER_SIZE_LONG] = {	0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
															0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
															0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
															0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
														};



//===========================================================
//SHA204_COMMAND_MARSHALING_H

/** \todo Create doxygen groups. */

//////////////////////////////////////////////////////////////////////
// command op-code definitions
#define SHA204_CHECKMAC                 ((uint8_t) 0x28)       //!< CheckMac command op-code
#define SHA204_DERIVE_KEY               ((uint8_t) 0x1C)       //!< DeriveKey command op-code
#define SHA204_DEVREV                   ((uint8_t) 0x30)       //!< DevRev command op-code
#define SHA204_GENDIG                   ((uint8_t) 0x15)       //!< GenDig command op-code
#define SHA204_HMAC                     ((uint8_t) 0x11)       //!< HMAC command op-code
#define SHA204_LOCK                     ((uint8_t) 0x17)       //!< Lock command op-code
#define SHA204_MAC                      ((uint8_t) 0x08)       //!< MAC command op-code
#define SHA204_NONCE                    ((uint8_t) 0x16)       //!< Nonce command op-code
#define SHA204_PAUSE                    ((uint8_t) 0x01)       //!< Pause command op-code
#define SHA204_RANDOM                   ((uint8_t) 0x1B)       //!< Random command op-code
#define SHA204_READ                     ((uint8_t) 0x02)       //!< Read command op-code
#define SHA204_UPDATE_EXTRA             ((uint8_t) 0x20)       //!< UpdateExtra command op-code
#define SHA204_WRITE                    ((uint8_t) 0x12)       //!< Write command op-code


//////////////////////////////////////////////////////////////////////
// packet size definitions
#define SHA204_RSP_SIZE_VAL             ((uint8_t)  7)         //!< size of response packet containing four bytes of data

//////////////////////////////////////////////////////////////////////
// parameter range definitions
#define SHA204_KEY_ID_MAX               ((uint8_t) 15)         //!< maximum value for key id
#define SHA204_OTP_BLOCK_MAX            ((uint8_t)  1)         //!< maximum value for OTP block

//////////////////////////////////////////////////////////////////////
// definitions for command packet indexes common to all commands
#define SHA204_COUNT_IDX                ( 0)                   //!< command packet index for count
#define SHA204_OPCODE_IDX               ( 1)                   //!< command packet index for op-code
#define SHA204_PARAM1_IDX               ( 2)                   //!< command packet index for first parameter
#define SHA204_PARAM2_IDX               ( 3)                   //!< command packet index for second parameter
#define SHA204_DATA_IDX                 ( 5)                   //!< command packet index for second parameter

//////////////////////////////////////////////////////////////////////
// zone definitions
#define SHA204_ZONE_CONFIG              ((uint8_t)  0x00)      //!< Configuration zone
#define SHA204_ZONE_OTP                 ((uint8_t)  0x01)      //!< OTP (One Time Programming) zone
#define SHA204_ZONE_DATA                ((uint8_t)  0x02)      //!< Data zone
#define SHA204_ZONE_MASK                ((uint8_t)  0x03)      //!< Zone mask
#define SHA204_ZONE_COUNT_FLAG          ((uint8_t)  0x80)      //!< Zone bit 7 set: Access 32 bytes, otherwise 4 bytes.
#define SHA204_ZONE_ACCESS_4            ((uint8_t)     4)      //!< Read or write 4 bytes.
#define SHA204_ZONE_ACCESS_32           ((uint8_t)    32)      //!< Read or write 32 bytes.
#define SHA204_ADDRESS_MASK_CONFIG      (         0x001F)      //!< Address bits 5 to 7 are 0 for Configuration zone.
#define SHA204_ADDRESS_MASK_OTP         (         0x000F)      //!< Address bits 4 to 7 are 0 for OTP zone.
#define SHA204_ADDRESS_MASK             (         0x007F)    //!< Address bit 7 to 15 are always 0.

//////////////////////////////////////////////////////////////////////
// CheckMAC command definitions
#define CHECKMAC_MODE_IDX               SHA204_PARAM1_IDX      //!< CheckMAC command index for mode
#define CHECKMAC_KEYID_IDX              SHA204_PARAM2_IDX      //!< CheckMAC command index for key identifier
#define CHECKMAC_CLIENT_CHALLENGE_IDX   SHA204_DATA_IDX        //!< CheckMAC command index for client challenge
#define CHECKMAC_CLIENT_RESPONSE_IDX    (37)                   //!< CheckMAC command index for client response
#define CHECKMAC_DATA_IDX               (69)                   //!< CheckMAC command index for other data
#define CHECKMAC_COUNT                  (84)                   //!< CheckMAC command packet size
#define CHECKMAC_MODE_MASK              ((uint8_t) 0x27)       //!< CheckMAC mode bits 3, 4, 6, and 7 are 0.
#define CHECKMAC_CLIENT_CHALLENGE_SIZE  (32)                   //!< CheckMAC size of client challenge
#define CHECKMAC_CLIENT_RESPONSE_SIZE   (32)                   //!< CheckMAC size of client response
#define CHECKMAC_OTHER_DATA_SIZE        (13)                   //!< CheckMAC size of "other data"
#define CHECKMAC_CLIENT_COMMAND_SIZE    ( 4)                   //!< CheckMAC size of client command header size inside "other data"

//////////////////////////////////////////////////////////////////////
// DeriveKey command definitions
#define DERIVE_KEY_RANDOM_IDX           SHA204_PARAM1_IDX      //!< DeriveKey command index for random bit
#define DERIVE_KEY_TARGETKEY_IDX        SHA204_PARAM2_IDX      //!< DeriveKey command index for target slot
#define DERIVE_KEY_MAC_IDX              SHA204_DATA_IDX        //!< DeriveKey command index for optional MAC
#define DERIVE_KEY_COUNT_SMALL          SHA204_CMD_SIZE_MIN    //!< DeriveKey command packet size without MAC
#define DERIVE_KEY_COUNT_LARGE          (39)                   //!< DeriveKey command packet size with MAC
#define DERIVE_KEY_RANDOM_FLAG          ((uint8_t) 4)          //!< DeriveKey 1. parameter
#define DERIVE_KEY_MAC_SIZE             (32)                   //!< DeriveKey MAC size

//////////////////////////////////////////////////////////////////////
// DevRev command definitions
#define DEVREV_PARAM1_IDX               SHA204_PARAM1_IDX      //!< DevRev command index for 1. parameter (ignored)
#define DEVREV_PARAM2_IDX               SHA204_PARAM2_IDX      //!< DevRev command index for 2. parameter (ignored)
#define DEVREV_COUNT                    SHA204_CMD_SIZE_MIN    //!< DevRev command packet size

//////////////////////////////////////////////////////////////////////
// GenDig command definitions
#define GENDIG_ZONE_IDX                 SHA204_PARAM1_IDX      //!< GenDig command index for zone
#define GENDIG_KEYID_IDX                SHA204_PARAM2_IDX      //!< GenDig command index for key id
#define GENDIG_DATA_IDX                 SHA204_DATA_IDX        //!< GenDig command index for optional data
#define GENDIG_COUNT                    SHA204_CMD_SIZE_MIN    //!< GenDig command packet size without "other data"
#define GENDIG_COUNT_DATA               (11)                   //!< GenDig command packet size with "other data"
#define GENDIG_OTHER_DATA_SIZE          (4)                    //!< GenDig size of "other data"
#define GENDIG_ZONE_OTP                 ((uint8_t) 1)          //!< GenDig zone id OTP
#define GENDIG_ZONE_DATA                ((uint8_t) 2)          //!< GenDig zone id data

//////////////////////////////////////////////////////////////////////
// HMAC command definitions
#define HMAC_MODE_IDX                   SHA204_PARAM1_IDX      //!< HMAC command index for mode
#define HMAC_KEYID_IDX                  SHA204_PARAM2_IDX      //!< HMAC command index for key id
#define HMAC_COUNT                      SHA204_CMD_SIZE_MIN    //!< HMAC command packet size
#define HMAC_MODE_MASK                  ((uint8_t) 0x74)       //!< HMAC mode bits 0, 1, 3, and 7 are 0.

//////////////////////////////////////////////////////////////////////
// Lock command definitions
#define LOCK_ZONE_IDX                   SHA204_PARAM1_IDX      //!< Lock command index for zone
#define LOCK_SUMMARY_IDX                SHA204_PARAM2_IDX      //!< Lock command index for summary
#define LOCK_COUNT                      SHA204_CMD_SIZE_MIN    //!< Lock command packet size
#define LOCK_ZONE_NO_CONFIG             ((uint8_t) 0x01)       //!< Lock zone is OTP or Data
#define LOCK_ZONE_NO_CRC                ((uint8_t) 0x80)       //!< Lock command: Ignore summary.
#define LOCK_ZONE_MASK                  (0x81)                 //!< Lock parameter 1 bits 2 to 6 are 0.

//////////////////////////////////////////////////////////////////////
// Mac command definitions
#define MAC_MODE_IDX                    SHA204_PARAM1_IDX      //!< MAC command index for mode
#define MAC_KEYID_IDX                   SHA204_PARAM2_IDX      //!< MAC command index for key id
#define MAC_CHALLENGE_IDX               SHA204_DATA_IDX        //!< MAC command index for optional challenge
#define MAC_COUNT_SHORT                 SHA204_CMD_SIZE_MIN    //!< MAC command packet size without challenge
#define MAC_COUNT_LONG                  (39)                   //!< MAC command packet size with challenge
#define MAC_MODE_BLOCK2_TEMPKEY         ((uint8_t) 0x01)       //!< MAC mode bit   0: second SHA block from TempKey
#define MAC_MODE_BLOCK1_TEMPKEY         ((uint8_t) 0x02)       //!< MAC mode bit   1: first SHA block from TempKey
#define MAC_MODE_SOURCE_FLAG_MATCH      ((uint8_t) 0x04)       //!< MAC mode bit   2: match TempKey.SourceFlag
#define MAC_MODE_PASSTHROUGH            ((uint8_t) 0x07)       //!< MAC mode bit 0-2: pass-through mode
#define MAC_MODE_INCLUDE_OTP_88         ((uint8_t) 0x10)       //!< MAC mode bit   4: include first 88 OTP bits
#define MAC_MODE_INCLUDE_OTP_64         ((uint8_t) 0x20)       //!< MAC mode bit   5: include first 64 OTP bits
#define MAC_MODE_INCLUDE_SN             ((uint8_t) 0x50)       //!< MAC mode bit   6: include serial number
#define MAC_CHALLENGE_SIZE              (32)                   //!< MAC size of challenge
#define MAC_MODE_MASK                   ((uint8_t) 0x77)       //!< MAC mode bits 3 and 7 are 0.


//////////////////////////////////////////////////////////////////////
// Nonce command definitions
#define NONCE_MODE_IDX                  SHA204_PARAM1_IDX      //!< Nonce command index for mode
#define NONCE_PARAM2_IDX                SHA204_PARAM2_IDX      //!< Nonce command index for 2. parameter
#define NONCE_INPUT_IDX                 SHA204_DATA_IDX        //!< Nonce command index for input data
#define NONCE_COUNT_SHORT               (27)                   //!< Nonce command packet size for 20 bytes of data
#define NONCE_COUNT_LONG                (39)                   //!< Nonce command packet size for 32 bytes of data
#define NONCE_MODE_MASK                 ((uint8_t) 3)          //!< Nonce mode bits 2 to 7 are 0.
#define NONCE_MODE_SEED_UPDATE          ((uint8_t) 0x00)       //!< Nonce mode: update seed
#define NONCE_MODE_NO_SEED_UPDATE       ((uint8_t) 0x01)       //!< Nonce mode: do not update seed
#define NONCE_MODE_INVALID              ((uint8_t) 0x02)       //!< Nonce mode 2 is invalid.
#define NONCE_MODE_PASSTHROUGH          ((uint8_t) 0x03)       //!< Nonce mode: pass-through
#define NONCE_NUMIN_SIZE                (20)                   //!< Nonce data length
#define NONCE_NUMIN_SIZE_PASSTHROUGH    (32)                   //!< Nonce data length in pass-through mode (mode = 3)

//////////////////////////////////////////////////////////////////////
// Pause command definitions
#define PAUSE_SELECT_IDX                SHA204_PARAM1_IDX      //!< Pause command index for Selector
#define PAUSE_PARAM2_IDX                SHA204_PARAM2_IDX      //!< Pause command index for 2. parameter
#define PAUSE_COUNT                     SHA204_CMD_SIZE_MIN    //!< Pause command packet size

//////////////////////////////////////////////////////////////////////
// Random command definitions
#define RANDOM_MODE_IDX                 SHA204_PARAM1_IDX      //!< Random command index for mode
#define RANDOM_PARAM2_IDX               SHA204_PARAM2_IDX      //!< Random command index for 2. parameter
#define RANDOM_COUNT                    SHA204_CMD_SIZE_MIN    //!< Random command packet size
#define RANDOM_SEED_UPDATE              ((uint8_t) 0x00)       //!< Random mode for automatic seed update
#define RANDOM_NO_SEED_UPDATE           ((uint8_t) 0x01)       //!< Random mode for no seed update

//////////////////////////////////////////////////////////////////////
// Read command definitions
#define READ_ZONE_IDX                   SHA204_PARAM1_IDX      //!< Read command index for zone
#define READ_ADDR_IDX                   SHA204_PARAM2_IDX      //!< Read command index for address
#define READ_COUNT                      SHA204_CMD_SIZE_MIN    //!< Read command packet size
#define READ_ZONE_MASK                  ((uint8_t) 0x83)       //!< Read zone bits 2 to 6 are 0.
#define READ_ZONE_MODE_32_BYTES         ((uint8_t) 0x80)       //!< Read mode: 32 bytes

//////////////////////////////////////////////////////////////////////
// UpdateExtra command definitions
#define UPDATE_MODE_IDX                  SHA204_PARAM1_IDX     //!< UpdateExtra command index for mode
#define UPDATE_VALUE_IDX                 SHA204_PARAM2_IDX     //!< UpdateExtra command index for new value
#define UPDATE_COUNT                     SHA204_CMD_SIZE_MIN   //!< UpdateExtra command packet size
#define UPDATE_CONFIG_BYTE_86            ((uint8_t) 0x01)      //!< UpdateExtra mode: update Config byte 86

//////////////////////////////////////////////////////////////////////
// Write command definitions
#define WRITE_ZONE_IDX                  SHA204_PARAM1_IDX      //!< Write command index for zone
#define WRITE_ADDR_IDX                  SHA204_PARAM2_IDX      //!< Write command index for address
#define WRITE_VALUE_IDX                 SHA204_DATA_IDX        //!< Write command index for data
#define WRITE_MAC_VS_IDX                ( 9)                   //!< Write command index for MAC following short data
#define WRITE_MAC_VL_IDX                (37)                   //!< Write command index for MAC following long data
#define WRITE_COUNT_SHORT               (11)                   //!< Write command packet size with short data and no MAC
#define WRITE_COUNT_LONG                (39)                   //!< Write command packet size with long data and no MAC
#define WRITE_COUNT_SHORT_MAC           (43)                   //!< Write command packet size with short data and MAC
#define WRITE_COUNT_LONG_MAC            (71)                   //!< Write command packet size with long data and MAC
#define WRITE_MAC_SIZE                  (32)                   //!< Write MAC size
// EA: #define WRITE_ZONE_MASK                 ((uint8_t) 0xC1)       //!< Write zone bits 2 to 5 are 0.
#define WRITE_ZONE_MASK                 ((uint8_t) 0xC3)       //!< Write zone bits 2 to 5 are 0.
#define WRITE_ZONE_WITH_MAC             ((uint8_t) 0x40)       //!< Write zone bit 6: write encrypted with MAC

//////////////////////////////////////////////////////////////////////
// Response size definitions
#define CHECKMAC_RSP_SIZE               SHA204_RSP_SIZE_MIN    //!< response size of DeriveKey command
#define DERIVE_KEY_RSP_SIZE             SHA204_RSP_SIZE_MIN    //!< response size of DeriveKey command
#define DEVREV_RSP_SIZE                 SHA204_RSP_SIZE_VAL    //!< response size of DevRev command returns 4 bytes
#define GENDIG_RSP_SIZE                 SHA204_RSP_SIZE_MIN    //!< response size of GenDig command
#define HMAC_RSP_SIZE                   SHA204_RSP_SIZE_MAX    //!< response size of HMAC command
#define LOCK_RSP_SIZE                   SHA204_RSP_SIZE_MIN    //!< response size of Lock command
#define MAC_RSP_SIZE                    SHA204_RSP_SIZE_MAX    //!< response size of MAC command
#define NONCE_RSP_SIZE_SHORT            SHA204_RSP_SIZE_MIN    //!< response size of Nonce command with mode[0:1] = 3
#define NONCE_RSP_SIZE_LONG             SHA204_RSP_SIZE_MAX    //!< response size of Nonce command
#define PAUSE_RSP_SIZE                  SHA204_RSP_SIZE_MIN    //!< response size of Pause command
#define RANDOM_RSP_SIZE                 SHA204_RSP_SIZE_MAX    //!< response size of Random command
#define READ_4_RSP_SIZE                 SHA204_RSP_SIZE_VAL    //!< response size of Read command when reading 4 bytes
#define READ_32_RSP_SIZE                SHA204_RSP_SIZE_MAX    //!< response size of Read command when reading 32 bytes
#define UPDATE_RSP_SIZE                 SHA204_RSP_SIZE_MIN    //!< response size of UpdateExtra command
#define WRITE_RSP_SIZE                  SHA204_RSP_SIZE_MIN    //!< response size of Write command

//////////////////////////////////////////////////////////////////////
// command timing definitions for typical execution times (ms)
//! CheckMAC typical command delay
#define CHECKMAC_DELAY                  (12)

//! DeriveKey typical command delay
#define DERIVE_KEY_DELAY                (14)

//! DevRev typical command delay
#define DEVREV_DELAY                    ( 1)  // 0.4 rounded up

//! GenDig typical command delay
#define GENDIG_DELAY                    (11)

//! HMAC typical command delay
#define HMAC_DELAY                      (27)

//! Lock typical command delay
#define LOCK_DELAY                      ( 5)

//! MAC typical command delay
#define MAC_DELAY                       (12)

//! Nonce typical command delay
#define NONCE_DELAY                     (22)

//! Pause typical command delay
#define PAUSE_DELAY                     ( 1)  // 0.4 rounded up

//! Random typical command delay
#define RANDOM_DELAY                    (11)

//! Read typical command delay
#define READ_DELAY                      ( 1)  // 0.4 rounded up

//! UpdateExtra typical command delay
#define UPDATE_DELAY                    ( 8)

//! Write typical command delay
#define WRITE_DELAY                     ( 4)

//////////////////////////////////////////////////////////////////////
// command timing definitions for maximum execution times (ms)
//! CheckMAC maximum execution time
#define CHECKMAC_EXEC_MAX                (38)

//! DeriveKey maximum execution time
#define DERIVE_KEY_EXEC_MAX              (62)

//! DevRev maximum execution time
#define DEVREV_EXEC_MAX                  ( 2)

//! GenDig maximum execution time
#define GENDIG_EXEC_MAX                  (43)

//! HMAC maximum execution time
#define HMAC_EXEC_MAX                    (69)

//! Lock maximum execution time
#define LOCK_EXEC_MAX                    (24)

//! MAC maximum execution time
#define MAC_EXEC_MAX                     (35)

//! Nonce maximum execution time
#define NONCE_EXEC_MAX                   (60)

//! Pause maximum execution time
#define PAUSE_EXEC_MAX                   ( 2)

//! Random maximum execution time
#define RANDOM_EXEC_MAX                  (50)

//! Read maximum execution time
#define READ_EXEC_MAX                    ( 4)

//! UpdateExtra maximum execution time
#define UPDATE_EXEC_MAX                  (12)

//! Write maximum execution time
#define WRITE_EXEC_MAX                   (42)

//////////////////////////////////////////////////////////////////////

/** 
 * \brief This structure contains the parameters for the \ref sha204m_check_mac function.
 */
struct sha204_check_mac_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t mode;              //!< what to include in the MAC calculation
   uint8_t key_id;            //!< what key to use for the MAC calculation
   uint8_t *client_challenge; //!< pointer to challenge that host had sent to client
   uint8_t *client_response;  //!< pointer to challenge response received from client 
   uint8_t *other_data;       //!< pointer to 13 bytes of data that were used by client to calculate MAC
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_derive_key function.
 */
struct sha204_derive_key_parameters {
   uint8_t *tx_buffer;   	//!< pointer to send buffer
   uint8_t *rx_buffer;   	//!< pointer to receive buffer
   uint8_t use_random;   	//!< true if source for TempKey was random number
   uint8_t target_key;   	//!< slot where derived key should be stored
   uint8_t *mac;	     	   //!< pointer to MAC for this command
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_dev_rev function.
 */
struct sha204_dev_rev_parameters {
   uint8_t *tx_buffer;   	//!< pointer to send buffer
   uint8_t *rx_buffer;		//!< pointer to receive buffer
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_gen_dig function.
 */
struct sha204_gen_dig_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t zone;              //!< what zone (config, OTP, or data) to use in the digest calculation
   uint8_t key_id;            //!< what key or OTP block to use for the digest calculation
   uint8_t *other_data;       //!< pointer to four bytes of data to use for the digest calculation, only needed when key is CheckMac only key 
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_hmac function.
 */
struct sha204_hmac_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t mode;              //!< what to include in the HMAC calculation
   uint16_t key_id;           //!< what key to use for the HMAC calculation
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_lock function.
 */
struct sha204_lock_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t zone;              //!< what zone (config, OTP, or data) to lock
   uint16_t summary;          //!< CRC over the zone to be locked
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_mac function.
 */
struct sha204_mac_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t mode;              //!< what to include in the MAC calculation
   uint16_t key_id;           //!< what key to use for the MAC calculation
   uint8_t *challenge;        //!< pointer to 32 bytes of challenge data to be sent to client
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_nonce function.
 */
struct sha204_nonce_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t mode;              //!< what TempKey should be loaded with
   uint8_t *num_in;           //!< pointer to 20 bytes of input or 32 bytes of pass-through data
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_pause function.
 */
struct sha204_pause_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t selector;          //!< which device not to set into Idle mode (single-wire interface only)
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_random function.
 */
struct sha204_random_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t mode;              //!< true if existing EEPROM seed should be used
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_read function.
 */
struct sha204_read_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t zone;              //!< what zone (config, OTP, or data) to read from and how many bytes (4 or 32)
   uint16_t address;          //!< what address to read from
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_update_extra function.
 */
struct sha204_update_extra_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t mode;              //!< config byte address = 84 + mode (0 or 1)
   uint8_t new_value;         //!< value to write
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_write function.
 */
struct sha204_write_parameters {
   uint8_t *tx_buffer;        //!< pointer to send buffer
   uint8_t *rx_buffer;        //!< pointer to receive buffer
   uint8_t zone;              //!< what zone (config, OTP, or data) to write to, how many bytes (4 or 32), and whether data are encrypted
   uint16_t address;          //!< what address to write to
   uint8_t *new_value;        //!< pointer to 4 or 32 bytes of data to be written
   uint8_t *mac;              //!< pointer to MAC of this command (null if zone is unlocked) 
};

/** 
 * \brief This structure contains the parameters for the \ref sha204m_execute function.
 */
struct sha204_command_parameters {
	uint8_t op_code;      //!< command code
	uint8_t param_1;      //!< parameter 1
	uint16_t param_2;     //!< parameter 2
	uint8_t data_len_1;   //!< length of data field 1
	uint8_t data_len_2;   //!< length of data field 2
	uint8_t data_len_3;   //!< length of data field 3
	uint8_t *data_1;      //!< pointer to data field 1
	uint8_t *data_2;      //!< pointer to data field 2
	uint8_t *data_3;      //!< pointer to data field 3
	uint8_t *tx_buffer;   //!< pointer to send buffer
	uint8_t *rx_buffer;   //!< pointer to receive buffer
	uint8_t tx_size;      //!< size of supplied send buffer
	uint8_t rx_size;      //!< size of supplied receive buffer
};
	
/**
 * \defgroup sha204_command_marshaling_group SHA204 Service - command marshaling functions
 *
 * @{
 */
uint8_t sha204m_check_mac(struct sha204_check_mac_parameters *args);
uint8_t sha204m_derive_key(struct sha204_derive_key_parameters *args);
uint8_t sha204m_dev_rev(struct sha204_dev_rev_parameters *args);
uint8_t sha204m_gen_dig(struct sha204_gen_dig_parameters *args);
uint8_t sha204m_hmac(struct sha204_hmac_parameters *args);
uint8_t sha204m_lock(struct sha204_lock_parameters *args);
uint8_t sha204m_mac(struct sha204_mac_parameters *args);
uint8_t sha204m_nonce(struct sha204_nonce_parameters *args);
uint8_t sha204m_pause(struct sha204_pause_parameters *args);
uint8_t sha204m_random(struct sha204_random_parameters *args);
uint8_t sha204m_read(struct sha204_read_parameters *args);
uint8_t sha204m_update_extra(struct sha204_update_extra_parameters *args);
uint8_t sha204m_write(struct sha204_write_parameters *args);
uint8_t sha204m_execute(struct sha204_command_parameters *args);




//========================================================================================
//SHA256.h
#define SHA224_DIGEST_SIZE ( 224 / 8)
#define SHA256_DIGEST_SIZE ( 256 / 8)

#define SHA256_BLOCK_SIZE  ( 512 / 8)
#define SHA224_BLOCK_SIZE  SHA256_BLOCK_SIZE


typedef struct {
    uint32 tot_len;
    uint32 len;
    uint8 block[2 * SHA256_BLOCK_SIZE];
    uint32 h[8];
} sha256_ctx;

void sha256_init(sha256_ctx * ctx);
void sha256_update(sha256_ctx *ctx, const uint8 *message, uint32 len);
void sha256_final(sha256_ctx *ctx, uint8 *digest);

void sha256_noPad(sha256_ctx *ctx, uint8 *digest);

void sha256(const uint8 *message, uint32 len, uint8 *digest);


//==========================================================================================
//SHA204_LIB_RETURN_CODES_H
#define SHA204_SUCCESS              ((uint8_t)  0x00) //!< Function succeeded.
#define SHA204_PARSE_ERROR          ((uint8_t)  0xD2) //!< response status byte indicates parsing error
#define SHA204_CMD_FAIL             ((uint8_t)  0xD3) //!< response status byte indicates command execution error
#define SHA204_STATUS_CRC           ((uint8_t)  0xD4) //!< response status byte indicates CRC error
#define SHA204_FUNC_FAIL            ((uint8_t)  0xE0) //!< Function could not execute due to incorrect condition / state.
#define SHA204_BAD_PARAM            ((uint8_t)  0xE2) //!< bad argument (out of range, null pointer, etc.)
#define SHA204_INVALID_SIZE         ((uint8_t)  0xE4) //!< Count value is out of range or greater than buffer size.
#define SHA204_BAD_CRC              ((uint8_t)  0xE5) //!< incorrect CRC received
#define SHA204_RX_FAIL              ((uint8_t)  0xE6) //!< Timed out while waiting for response. Number of bytes received is > 0.
#define SHA204_RX_NO_RESPONSE       ((uint8_t)  0xE7) //!< Not an error while the Command layer is polling for a command response.
#define SHA204_RESYNC_WITH_WAKEUP   ((uint8_t)  0xE8) //!< re-synchronization succeeded, but only after generating a Wake-up

#define SHA204_COMM_FAIL            ((uint8_t)  0xF0) //!< Communication with device failed. Same as in hardware dependent modules.
#define SHA204_TIMEOUT              ((uint8_t)  0xF1) //!< Timed out while waiting for response. Number of bytes received is 0.



//==========================================================================================

//-------------------
// Macro definitions
//-------------------
// SHA-256 message sizes
#define SHA204_MSG_SIZE_NONCE            (55)  // (32+20+1+1+1)
#define SHA204_MSG_SIZE_MAC              (88)  // (32+32+1+1+2+8+3+1+4+2+2)
#define SHA204_MSG_SIZE_HMAC_INNER       (152) // (32+32+32+32+1+1+2+8+3+1+4+2+2)
#define SHA204_MSG_SIZE_HMAC_OUTER       (96)  // (32+32+32)
#define SHA204_MSG_SIZE_GEN_DIG          (96)  // (32+1+1+2+1+2+25+32)
#define SHA204_MSG_SIZE_DERIVE_KEY       (96)  // (32+1+1+2+1+2+25+32)
#define SHA204_MSG_SIZE_DERIVE_KEY_MAC   (39)  // (32+1+1+2+1+2)
#define SHA204_MSG_SIZE_ENCRYPT_MAC      (96)  // (32+1+1+2+1+2+25+32)

// SN[0:1] and SN[8]
#define SHA204_SN_0               (0x01)
#define SHA204_SN_1               (0x23)
#define SHA204_SN_8               (0xEE)

// Command parameter definitions (that is not defined in sha204_comm_marshaling.h)
#define MAC_MODE_USE_TEMPKEY_MASK    ((uint8_t) 0x03)
#ifndef GENDIG_ZONE_CONFIG
	#define GENDIG_ZONE_CONFIG       ((uint8_t) 0)
#endif //GENDIG_ZONE_CONFIG


//-----------------------
// Structure for TempKey
//-----------------------
/** \struct sha204h_temp_key
 *  \brief Structure to hold TempKey fields
 *  \var sha204h_temp_key::value
 *       \brief The value of TempKey. Nonce (from nonce command) or Digest (from GenDig command) 
 *  \var sha204h_temp_key::key_id
 *       \brief If TempKey was generated by GenDig (see the GenData and CheckFlag bits), these bits indicate which key was used in its computation.
 *  \var sha204h_temp_key::source_flag
 *       \brief The source of the randomness in TempKey: 0=Rand, 1=Input.
 *  \var sha204h_temp_key::gen_data
 *       \brief Indicates if TempKey has been generated by GenDig using Data zone.
 *  \var sha204h_temp_key::check_flag
 *       \brief Not used in the library.
 *  \var sha204h_temp_key::valid
 *       \brief Indicates if the information in TempKey is valid.
 */
struct sha204h_temp_key {
	uint8_t value[32];
	uint8_t key_id :4;
	uint8_t source_flag :1;
	uint8_t gen_data :1;
	uint8_t check_flag :1;
	uint8_t valid :1;
};


//----------------------------------------
// Structures for input/output parameters
//----------------------------------------
/** \struct sha204h_nonce_in_out
 *  \brief Input/output parameters for function sha204h_nonce().
 *  \var sha204h_nonce_in_out::mode
 *       \brief [in] Mode parameter used in Nonce command (Param1).
 *  \var sha204h_nonce_in_out::num_in
 *       \brief [in] Pointer to 20-byte NumIn data used in Nonce command.
 *  \var sha204h_nonce_in_out::rand_out
 *       \brief [in] Pointer to 32-byte RandOut data from Nonce command.
 *  \var sha204h_nonce_in_out::temp_key
 *       \brief [in,out] Pointer to TempKey structure.
 */
struct sha204h_nonce_in_out {
	uint8_t mode; 
	uint8_t *num_in;
	uint8_t *rand_out;
	struct sha204h_temp_key *temp_key;
};


/** \struct sha204h_mac_in_out
 *  \brief Input/output parameters for function sha204h_mac().
 *  \var sha204h_mac_in_out::mode
 *       \brief [in] Mode parameter used in MAC command (Param1).
 *  \var sha204h_mac_in_out::key_id
 *       \brief [in] KeyID parameter used in MAC command (Param2).
 *  \var sha204h_mac_in_out::challenge
 *       \brief [in] Pointer to 32-byte Challenge data used in MAC command, depending on mode.
 *  \var sha204h_mac_in_out::key
 *       \brief [in] Pointer to 32-byte key used to generate MAC digest.
 *  \var sha204h_mac_in_out::otp
 *       \brief [in] Pointer to 11-byte OTP, optionally included in MAC digest, depending on mode.
 *  \var sha204h_mac_in_out::sn
 *       \brief [in] Pointer to 9-byte SN, optionally included in MAC digest, depending on mode.
 *  \var sha204h_mac_in_out::response
 *       \brief [out] Pointer to 32-byte SHA-256 digest (MAC).
 *  \var sha204h_mac_in_out::temp_key
 *       \brief [in,out] Pointer to TempKey structure.
 */
struct sha204h_mac_in_out {
	uint8_t mode;
	uint16_t key_id;
	uint8_t *challenge;
	uint8_t *key;
	uint8_t *otp;
	uint8_t *sn;
	uint8_t *response;
	struct sha204h_temp_key *temp_key;
};


/** \struct sha204h_hmac_in_out
 *  \brief Input/output parameters for function sha204h_hmac().
 *  \var sha204h_hmac_in_out::mode
 *       \brief [in] Mode parameter used in HMAC command (Param1).
 *  \var sha204h_hmac_in_out::key_id
 *       \brief [in] KeyID parameter used in HMAC command (Param2).
 *  \var sha204h_hmac_in_out::key
 *       \brief [in] Pointer to 32-byte key used to generate HMAC digest.
 *  \var sha204h_hmac_in_out::otp
 *       \brief [in] Pointer to 11-byte OTP, optionally included in HMAC digest, depending on mode.
 *  \var sha204h_hmac_in_out::sn
 *       \brief [in] Pointer to 9-byte SN, optionally included in HMAC digest, depending on mode.
 *  \var sha204h_hmac_in_out::response
 *       \brief [out] Pointer to 32-byte SHA-256 HMAC digest.
 *  \var sha204h_hmac_in_out::temp_key
 *       \brief [in,out] Pointer to TempKey structure.
 */
struct sha204h_hmac_in_out {
	uint8_t mode;
	uint16_t key_id;
	uint8_t *key;
	uint8_t *otp;
	uint8_t *sn;
	uint8_t *response;
	struct sha204h_temp_key *temp_key;
};


/** \struct sha204h_gen_dig_in_out
 *  \brief Input/output parameters for function sha204h_gen_dig().
 *  \var sha204h_gen_dig_in_out::zone
 *       \brief [in] Zone parameter used in GenDig command (Param1).
 *  \var sha204h_gen_dig_in_out::key_id
 *       \brief [in] KeyID parameter used in GenDig command (Param2).
 *  \var sha204h_gen_dig_in_out::stored_value
 *       \brief [in] Pointer to 32-byte stored value, can be a data slot, OTP page, configuration zone, or hardware transport key.
 *  \var sha204h_gen_dig_in_out::temp_key
 *       \brief [in,out] Pointer to TempKey structure.
 */
struct sha204h_gen_dig_in_out {
	uint8_t zone;
	uint16_t key_id;
	uint8_t *stored_value;
	struct sha204h_temp_key *temp_key;
};


/** \struct sha204h_derive_key_in_out
 *  \brief Input/output parameters for function sha204h_derive_key().
 *  \var sha204h_derive_key_in_out::random
 *       \brief [in] Random parameter used in DeriveKey command (Param1).
 *  \var sha204h_derive_key_in_out::target_key_id
 *       \brief [in] KeyID to be derived, TargetKey parameter used in DeriveKey command (Param2).
 *  \var sha204h_derive_key_in_out::parent_key
 *       \brief [in] Pointer to 32-byte ParentKey. Set equal to target_key if Roll Key operation is intended.
 *  \var sha204h_derive_key_in_out::target_key
 *       \brief [out] Pointer to 32-byte TargetKey.
 *  \var sha204h_derive_key_in_out::temp_key
 *       \brief [in,out] Pointer to TempKey structure.
 */
struct sha204h_derive_key_in_out {
	uint8_t random;
	uint16_t target_key_id;
	uint8_t *parent_key;
	uint8_t *target_key;
	struct sha204h_temp_key *temp_key;
};


/** \struct sha204h_derive_key_mac_in_out
 *  \brief Input/output parameters for function sha204h_derive_key_mac().
 *  \var sha204h_derive_key_mac_in_out::random
 *       \brief [in] Random parameter used in DeriveKey command (Param1).
 *  \var sha204h_derive_key_mac_in_out::target_key_id
 *       \brief [in] KeyID to be derived, TargetKey parameter used in DeriveKey command (Param2).
 *  \var sha204h_derive_key_mac_in_out::parent_key
 *       \brief [in] Pointer to 32-byte ParentKey. ParentKey here is always SlotConfig[TargetKey].WriteKey, regardless whether the operation is Roll or Create.
 *  \var sha204h_derive_key_mac_in_out::mac
 *       \brief [out] Pointer to 32-byte Mac.
 */
struct sha204h_derive_key_mac_in_out {
	uint8_t random;
	uint16_t target_key_id;
	uint8_t *parent_key;
	uint8_t *mac;
};


/** \struct sha204h_encrypt_in_out
 *  \brief Input/output parameters for function sha204h_encrypt().
 *  \var sha204h_encrypt_in_out::zone
 *       \brief [in] Zone parameter used in Write (Param1).
 *  \var sha204h_encrypt_in_out::address
 *       \brief [in] Address parameter used in Write command (Param2).
 *  \var sha204h_encrypt_in_out::data
 *       \brief [in,out] Pointer to 32-byte data. Input cleartext data, output encrypted data to Write command (Value field).
 *  \var sha204h_encrypt_in_out::mac
 *       \brief [out] Pointer to 32-byte Mac. Can be set to NULL if input MAC is not required by the Write command (write to OTP, unlocked user zone).
 *  \var sha204h_encrypt_in_out::temp_key
 *       \brief [in,out] Pointer to TempKey structure.
 */
struct sha204h_encrypt_in_out {
	uint8_t zone;
	uint16_t address;
	uint8_t *data;
	uint8_t *mac;
	struct sha204h_temp_key *temp_key;
};


/** \struct sha204h_decrypt_in_out
 *  \brief Input/output parameters for function sha204h_decrypt().
 *  \var sha204h_decrypt_in_out::data
 *       \brief [in,out] Pointer to 32-byte data. Input encrypted data from Read command (Contents field), output decrypted.
 *  \var sha204h_decrypt_in_out::temp_key
 *       \brief [in,out] Pointer to TempKey structure.
 */
struct sha204h_decrypt_in_out {
	uint8_t *data;
	struct sha204h_temp_key *temp_key;
};


//---------------------
// Function prototypes
//---------------------
uint8_t sha204h_nonce(struct sha204h_nonce_in_out param);
uint8_t sha204h_mac(struct sha204h_mac_in_out param);
uint8_t sha204h_hmac(struct sha204h_hmac_in_out param);
uint8_t sha204h_gen_dig(struct sha204h_gen_dig_in_out param);
uint8_t sha204h_derive_key(struct sha204h_derive_key_in_out param);
uint8_t sha204h_derive_key_mac(struct sha204h_derive_key_mac_in_out param);
uint8_t sha204h_encrypt(struct sha204h_encrypt_in_out param);
uint8_t sha204h_decrypt(struct sha204h_decrypt_in_out param);
void sha204h_calculate_crc_chain(uint8_t length, uint8_t *data, uint8_t *crc);


uint8_t atsha204_enc_write(uint16_t slot_to_write, uint8_t* clear_data, uint16_t key_id, uint8_t* secret_key, uint8_t* NumIn) ;
uint8_t atsha204_enc_read(uint16_t slot_to_read, uint8_t* clear_data, uint16_t key_id, uint8_t* secret_key, uint8_t* NumIn);
uint8_t atsha204_mac(uint16_t key_id,uint8_t* secret_key, uint8_t* NumIn, uint8_t* challenge);



#endif
