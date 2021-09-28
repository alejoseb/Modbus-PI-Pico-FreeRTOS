/*
 * Modbus.h
 *
 *  Created on: May 5, 2020
 *      Author: Alejandro Mera
 */

#ifndef THIRD_PARTY_MODBUS_INC_MODBUS_H_
#define THIRD_PARTY_MODBUS_INC_MODBUS_H_

#ifdef __cplusplus
    extern "C" {
#endif

//#include <CMSIS_RTOS_V2/cmsis_os.h>
#include <inttypes.h>
//#include "main.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "ModbusConfig.h"





typedef QueueHandle_t SemaphoreHandle_t;




/// Priority values.
typedef enum {
  osPriorityNone          =  0,         ///< No priority (not initialized).
  osPriorityIdle          =  1,         ///< Reserved for Idle thread.
  osPriorityLow           =  8,         ///< Priority: low
  osPriorityLow1          =  8+1,       ///< Priority: low + 1
  osPriorityLow2          =  8+2,       ///< Priority: low + 2
  osPriorityLow3          =  8+3,       ///< Priority: low + 3
  osPriorityLow4          =  8+4,       ///< Priority: low + 4
  osPriorityLow5          =  8+5,       ///< Priority: low + 5
  osPriorityLow6          =  8+6,       ///< Priority: low + 6
  osPriorityLow7          =  8+7,       ///< Priority: low + 7
  osPriorityBelowNormal   = 16,         ///< Priority: below normal
  osPriorityBelowNormal1  = 16+1,       ///< Priority: below normal + 1
  osPriorityBelowNormal2  = 16+2,       ///< Priority: below normal + 2
  osPriorityBelowNormal3  = 16+3,       ///< Priority: below normal + 3
  osPriorityBelowNormal4  = 16+4,       ///< Priority: below normal + 4
  osPriorityBelowNormal5  = 16+5,       ///< Priority: below normal + 5
  osPriorityBelowNormal6  = 16+6,       ///< Priority: below normal + 6
  osPriorityBelowNormal7  = 16+7,       ///< Priority: below normal + 7
  osPriorityNormal        = 24,         ///< Priority: normal
  osPriorityNormal1       = 24+1,       ///< Priority: normal + 1
  osPriorityNormal2       = 24+2,       ///< Priority: normal + 2
  osPriorityNormal3       = 24+3,       ///< Priority: normal + 3
  osPriorityNormal4       = 24+4,       ///< Priority: normal + 4
  osPriorityNormal5       = 24+5,       ///< Priority: normal + 5
  osPriorityNormal6       = 24+6,       ///< Priority: normal + 6
  osPriorityNormal7       = 24+7,       ///< Priority: normal + 7
  osPriorityAboveNormal   = 32,         ///< Priority: above normal
  osPriorityAboveNormal1  = 32+1,       ///< Priority: above normal + 1
  osPriorityAboveNormal2  = 32+2,       ///< Priority: above normal + 2
  osPriorityAboveNormal3  = 32+3,       ///< Priority: above normal + 3
  osPriorityAboveNormal4  = 32+4,       ///< Priority: above normal + 4
  osPriorityAboveNormal5  = 32+5,       ///< Priority: above normal + 5
  osPriorityAboveNormal6  = 32+6,       ///< Priority: above normal + 6
  osPriorityAboveNormal7  = 32+7,       ///< Priority: above normal + 7
  osPriorityHigh          = 40,         ///< Priority: high
  osPriorityHigh1         = 40+1,       ///< Priority: high + 1
  osPriorityHigh2         = 40+2,       ///< Priority: high + 2
  osPriorityHigh3         = 40+3,       ///< Priority: high + 3
  osPriorityHigh4         = 40+4,       ///< Priority: high + 4
  osPriorityHigh5         = 40+5,       ///< Priority: high + 5
  osPriorityHigh6         = 40+6,       ///< Priority: high + 6
  osPriorityHigh7         = 40+7,       ///< Priority: high + 7
  osPriorityRealtime      = 48,         ///< Priority: realtime
  osPriorityRealtime1     = 48+1,       ///< Priority: realtime + 1
  osPriorityRealtime2     = 48+2,       ///< Priority: realtime + 2
  osPriorityRealtime3     = 48+3,       ///< Priority: realtime + 3
  osPriorityRealtime4     = 48+4,       ///< Priority: realtime + 4
  osPriorityRealtime5     = 48+5,       ///< Priority: realtime + 5
  osPriorityRealtime6     = 48+6,       ///< Priority: realtime + 6
  osPriorityRealtime7     = 48+7,       ///< Priority: realtime + 7
  osPriorityISR           = 56,         ///< Reserved for ISR deferred thread.
  osPriorityError         = -1,         ///< System cannot determine priority or illegal priority.
  osPriorityReserved      = 0x7FFFFFFF  ///< Prevents enum down-size compiler optimization.
} osPriority_t;



typedef enum
{
    USART_HW = 1,
    USB_CDC_HW = 2,
    TCP_HW = 3,
}mb_hardware_t ;


typedef enum
{
    MB_SLAVE = 3,
    MB_MASTER = 4
}mb_masterslave_t ;



/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for Master or for Slave.
 *
 * @see also fctsupported
 * @see also modbus_t
 */
typedef enum MB_FC
{
    MB_FC_READ_COILS               = 1,	 /*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUT      = 2,	 /*!< FCT=2 -> read digital inputs */
    MB_FC_READ_REGISTERS           = 3,	 /*!< FCT=3 -> read registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER      = 4,	 /*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_COIL               = 5,	 /*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_REGISTER           = 6,	 /*!< FCT=6 -> write single register */
    MB_FC_WRITE_MULTIPLE_COILS     = 15, /*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16	 /*!< FCT=16 -> write multiple registers */
}mb_functioncode_t;


typedef struct
{
uint8_t uxBuffer[MAX_BUFFER];
uint8_t u8start;
uint8_t u8end;
uint8_t u8available;
bool    overflow;
}modbusRingBuffer_t;




/**
 * @enum MESSAGE
 * @brief
 * Indexes to telegram frame positions
 */
typedef enum MESSAGE
{
    ID                             = 0, //!< ID field
    FUNC, //!< Function code position
    ADD_HI, //!< Address high byte
    ADD_LO, //!< Address low byte
    NB_HI, //!< Number of coils or registers high byte
    NB_LO, //!< Number of coils or registers low byte
    BYTE_CNT  //!< byte counter
}mb_message_t;

typedef enum COM_STATES
{
    COM_IDLE                     = 0,
    COM_WAITING                  = 1
}mb_com_state_t;

typedef enum ERR_LIST
{
    ERR_NOT_MASTER                = -1,
    ERR_POLLING                   = -2,
    ERR_BUFF_OVERFLOW             = -3,
    ERR_BAD_CRC                   = -4,
    ERR_EXCEPTION                 = -5,
    ERR_BAD_SIZE                  = -6,
    ERR_BAD_ADDRESS               = -7,
    ERR_TIME_OUT		          = -8,
    ERR_BAD_SLAVE_ID		      = -9

}mb_errot_t;

enum
{
    //NO_REPLY = 255,
    EXC_FUNC_CODE = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE = 4
};

typedef union {
	uint8_t  u8[4];
	uint16_t u16[2];
	uint32_t u32;

} bytesFields ;





/**
 * @struct modbus_t
 * @brief
 * Master query structure:
 * This structure contains all the necessary fields to make the Master generate a Modbus query.
 * A Master may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */
typedef struct
{
    uint8_t u8id;          /*!< Slave address between 1 and 247. 0 means broadcast */
    mb_functioncode_t u8fct;         /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
    uint16_t u16RegAdd;    /*!< Address of the first register to access at slave/s */
    uint16_t u16CoilsNo;   /*!< Number of coils or registers to access */
    uint16_t *u16reg;     /*!< Pointer to memory image in master */
    uint32_t *u32CurrentTask; /*!< Pointer to the task that will receive notifications from Modbus */
#if ENABLE_TCP ==1
    uint32_t   xIpAddress;
    uint16_t u16Port;
#endif
}
modbus_t;




/**
 * @struct modbusHandler_t
 * @brief
 * Modbus handler structure
 * Contains all the variables required for Modbus daemon operation
 */
typedef struct
{

	mb_masterslave_t uModbusType;
	uart_inst_t *port; //HAL Serial Port handler
	uint8_t u8id; //!< 0=master, 1..247=slave number
	uint16_t *EN_Port; //!< flow control pin: 0=USB or RS-232 mode, >1=RS-485 mode
	uint16_t EN_Pin;  //!< flow control pin: 0=USB or RS-232 mode, >1=RS-485 mode
	mb_errot_t i8lastError;
	uint8_t u8Buffer[MAX_BUFFER]; //Modbus buffer for communication
	uint8_t u8BufferSize;
	uint8_t u8lastRec;
	uint16_t *u16regs;
	uint16_t u16InCnt, u16OutCnt, u16errCnt; //keep statistics of Modbus traffic
	uint16_t u16timeOut;
	uint32_t u32time, u32timeOut;
	uint16_t u16regsize;
	uint8_t dataRX;
	int8_t i8state;

	//FreeRTOS components

	//Queue Modbus Telegram
	QueueHandle_t QueueTelegramHandle;

	//Task Modbus slave
	TaskHandle_t myTaskModbusAHandle;
	//Timer RX Modbus
	TimerHandle_t xTimerT35;
	//Timer MasterTimeout
	TimerHandle_t xTimerTimeout;
	//Semaphore for Modbus data
	SemaphoreHandle_t ModBusSphrHandle;
	// RX ring buffer for USART
	modbusRingBuffer_t xBufferRX;
	// type of hardware  TCP, USB CDC, USART
	mb_hardware_t xTypeHW;

}
modbusHandler_t;


enum
{
    RESPONSE_SIZE = 6,
    EXCEPTION_SIZE = 3,
    CHECKSUM_SIZE = 2
};




extern modbusHandler_t *mHandlers[MAX_M_HANDLERS];

void ModbusStart(modbusHandler_t * modH);
void ModbusInit(modbusHandler_t * modH);
void setTimeOut( uint16_t u16timeOut); //!<write communication watch-dog timer
uint16_t getTimeOut(); //!<get communication watch-dog timer value
bool getTimeOutState(); //!<get communication watch-dog timer state
void ModbusQuery(modbusHandler_t * modH, modbus_t telegram ); // put a query in the queue tail
void ModbusQueryInject(modbusHandler_t * modH, modbus_t telegram); //put a query in the queue head
void setID( uint8_t u8id ); //!<write new ID for the slave
void setTxendPinOverTime( uint32_t u32overTime );
void ModbusEnd(); //!<finish any communication and release serial communication port
void StartTaskModbusSlave(void *argument); //slave
void StartTaskModbusMaster(void *argument); //master
uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length);


//Function prototypes for ModbusRingBuffer
void RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val); // adds a byte to the ring buffer
uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer); // gets all the available bytes into buffer and return the number of bytes read
uint8_t RingGetNBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber); // gets uNumber of bytes from ring buffer, returns the actual number of bytes read
uint8_t RingCountBytes(modbusRingBuffer_t *xRingBuffer); // return the number of available bytes
void RingClear(modbusRingBuffer_t *xRingBuffer); // flushes the ring buffer

//Function prototypes for ModbusRingBuffer
void RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val); // adds a byte to the ring buffer
uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer); // gets all the available bytes into buffer and return the number of bytes read
uint8_t RingGetNBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber); // gets uNumber of bytes from ring buffer, returns the actual number of bytes read
uint8_t RingCountBytes(modbusRingBuffer_t *xRingBuffer); // return the number of available bytes
void RingClear(modbusRingBuffer_t *xRingBuffer); // flushes the ring buffer

#ifdef __cplusplus
    }
#endif

#endif /* THIRD_PARTY_MODBUS_INC_MODBUS_H_ */
