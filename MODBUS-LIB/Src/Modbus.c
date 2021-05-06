/*
 * Modbus.c
 *  Modbus RTU Master and Slave library for STM32 CUBE with FreeRTOS
 *  Created on: May 5, 2020
 *      Author: Alejandro Mera
 *      Adapted from https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Modbus.h"
#include "timers.h"
#include "semphr.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

#define lowByte(w) ((w) & 0xff)
#define highByte(w) ((w) >> 8)

#define STACK_SIZE 256


static void vTimerCallbackT35(TimerHandle_t *pxTimer);
static void vTimerCallbackTimeout(TimerHandle_t *pxTimer);
static void StartTaskModbusSlave(void *argument);

uint8_t numberHandlers = 0;






/* Ring Buffer functions */
// This function must be called only after disabling USART RX interrupt or inside of the RX interrupt
void RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val)
{

	xRingBuffer->uxBuffer[xRingBuffer->u8end] = u8Val;
	xRingBuffer->u8end = (xRingBuffer->u8end + 1) % MAX_BUFFER;
	if (xRingBuffer->u8available == MAX_BUFFER)
	{
		xRingBuffer->overflow = true;
		xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
	}
	else
	{
		xRingBuffer->overflow = false;
		xRingBuffer->u8available++;
	}

}

// This function must be called only after disabling USART RX interrupt
uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer)
{
	return RingGetNBytes(xRingBuffer, buffer, xRingBuffer->u8available);
}

// This function must be called only after disabling USART RX interrupt
uint8_t RingGetNBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber)
{
	uint8_t uCounter;
	if(xRingBuffer->u8available == 0  || uNumber == 0 ) return 0;
	if(uNumber > MAX_BUFFER) return 0;

	for(uCounter = 0; uCounter < uNumber && uCounter< xRingBuffer->u8available ; uCounter++)
	{
		buffer[uCounter] = xRingBuffer->uxBuffer[xRingBuffer->u8start];
		xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
	}
	xRingBuffer->u8available = xRingBuffer->u8available - uCounter;
	xRingBuffer->overflow = false;
	RingClear(xRingBuffer);

	return uCounter;
}

uint8_t RingCountBytes(modbusRingBuffer_t *xRingBuffer)
{
return xRingBuffer->u8available;
}

void RingClear(modbusRingBuffer_t *xRingBuffer)
{
xRingBuffer->u8start = 0;
xRingBuffer->u8end = 0;
xRingBuffer->u8available = 0;
xRingBuffer->overflow = false;
}

/* End of Ring Buffer functions */




/**
 * @brief
 * Initialization for a Master/Slave.
 *
 * For hardware serial through USB/RS232C/RS485 set port to Serial, Serial1,
 * Serial2, or Serial3. (Numbered hardware serial ports are only available on
 * some boards.)
 *
 * For software serial through RS232C/RS485 set port to a SoftwareSerial object
 * that you have already constructed.
 *
 * ModbusRtu needs a pin for flow control only for RS485 mode. Pins 0 and 1
 * cannot be used.
 *
 * First call begin() on your serial port, and then start up ModbusRtu by
 * calling start(). You can choose the line speed and other port parameters
 * by passing the appropriate values to the port's begin() function.
 *
 * @param u8id   node address 0=master, 1..247=slave
 * @param port   serial port used
 * @param EN_Port_v port for txen RS-485
 * @param EN_Pin_v pin for txen RS-485 (NULL means RS232C mode)
 * @ingroup setup
 */
void ModbusInit(modbusHandler_t * modH)
{

  if (numberHandlers < MAX_M_HANDLERS)
  {
	  //Create QueueModbus //Queue Modbus RX
	  //modH->QueueModbusHandle = osMessageQueueNew (MAX_BUFFER, sizeof(uint8_t), &QueueModbus_attributes);
    RingClear(&modH->xBufferRX);

	
	  if(modH->uModbusType == MB_SLAVE)
	  {
		  //Create Modbus task slave
	  	  //modH->myTaskModbusAHandle = osThreadNew(StartTaskModbusSlave, modH, &myTaskModbusA_attributes);

  	  	  TaskHandle_t hTask;
  	  	  UBaseType_t prio;

  	  	  //prio = osPriorityNormal;
  	  	  prio = osPriorityLow;
		  if ((prio < osPriorityIdle) || (prio > osPriorityISR)) {
			  //TODO error message
			  //return "error";
			  while(1);
		  }

		
			  if (xTaskCreate ((TaskFunction_t)StartTaskModbusSlave, "myTaskModbusA", (uint16_t)(STACK_SIZE), modH, prio, &hTask) != pdPASS) {
				  hTask = NULL;
			  }
		

          modH->myTaskModbusAHandle = hTask;


	  }
	  
	  else
	  {
		  while(1); //Error Modbus type not supported choose a valid Type
	  }
	  //Create Semaphore DataRX
	  //vSemaphoreCreateBinary(SemaphoreDataRX);
	  //Create timer T35

	  modH->xTimerT35 = xTimerCreate("TimerT35",         // Just a text name, not used by the kernel.
		  	  	  	  	  	  	  	5 ,     // The timer period in ticks.
                                    pdFALSE,         // The timers will auto-reload themselves when they expire.
									( void * )modH->xTimerT35,     // Assign each timer a unique id equal to its array index.
                                    (TimerCallbackFunction_t) vTimerCallbackT35     // Each timer calls the same callback when it expires.
                                    );

	  //modH->ModBusSphrHandle = osSemaphoreNew(1, 1, &ModBusSphr_attributes);

	  SemaphoreHandle_t hSemaphore;
	  hSemaphore = xSemaphoreCreateBinary();
	  if ((hSemaphore != NULL)) {
        if (xSemaphoreGive (hSemaphore) != pdPASS) {
          vSemaphoreDelete (hSemaphore);
          hSemaphore = NULL;
        }
      }
      #if (configQUEUE_REGISTRY_SIZE > 0)
      vQueueAddToRegistry (hSemaphore, "ModBusSphr");
      #endif

  	  modH->ModBusSphrHandle = hSemaphore;

  	  mHandlers[numberHandlers] = modH;

	  numberHandlers++;
  }
  else
  {
	  while(1); //error no more Modbus handlers supported
  }

}


void StartTaskModbusSlave(void *argument)
{

  modbusHandler_t *modH =  (modbusHandler_t *)argument;


  for(;;)
  {

	   vTaskDelay(1000);
  }
  
}

void vTimerCallbackT35(TimerHandle_t *pxTimer)
{
	//Notify that a steam has just arrived
	int i;
	//TimerHandle_t aux;
	for(i = 0; i < numberHandlers; i++)
	{

		if( (TimerHandle_t *)mHandlers[i]->xTimerT35 ==  pxTimer ){
			if(mHandlers[i]->uModbusType == MB_MASTER)
			{
				xTimerStop(mHandlers[i]->xTimerTimeout,0);
			}
			xTaskNotify(mHandlers[i]->myTaskModbusAHandle, 0, eSetValueWithOverwrite);
		}

	}
}

void vTimerCallbackTimeout(TimerHandle_t *pxTimer)
{
	//Notify that a steam has just arrived
	int i;
	//TimerHandle_t aux;
	for(i = 0; i < numberHandlers; i++)
	{

		if( (TimerHandle_t *)mHandlers[i]->xTimerTimeout ==  pxTimer ){
				xTaskNotify(mHandlers[i]->myTaskModbusAHandle, ERR_TIME_OUT, eSetValueWithOverwrite);
		}

	}

}