#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "GPIO.hpp"
#include "Modbus.h"
#include <array>



modbusHandler_t ModbusH;
uint16_t ModbusDATA[8];
modbusHandler_t ModbusH2;
uint16_t ModbusDATA2[8];



static pico_cpp::GPIO_Pin ledPin(25,pico_cpp::PinType::Output);
void vTaskCode( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. 
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
    */
    modbus_t telegram[2];

    uint32_t u32NotificationValue;

    telegram[0].u8id = 1; // slave address
    telegram[0].u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS; // function code (this one is registers read)
    //telegram[0].u16RegAdd = 0x160; // start address in slave
    telegram[0].u16RegAdd = 0x0; // start address in slave
    telegram[0].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[0].u16reg = ModbusDATA; // pointer to a memory array in the Arduino


    telegram[1].u8id = 1; // slave address
    telegram[1].u8fct = MB_FC_READ_REGISTERS; // function code (this one is registers read)
    //telegram[0].u16RegAdd = 0x160; // start address in slave
    telegram[1].u16RegAdd = 0x0; // start address in slave
    telegram[1].u16CoilsNo = 8; // number of elements (coils or registers) to read
    telegram[1].u16reg = ModbusDATA; // pointer to a memory array in the Arduino
    
    for( ;; )
    {
            
            //if(xSemaphoreTake(ModbusH.ModBusSphrHandle , 500) == pdTRUE)
            //{
            //    
            //    gpio_put(PICO_DEFAULT_LED_PIN, ModbusDATA[0] & 0x01 );                
            //    xSemaphoreGive( ModbusH.ModBusSphrHandle );               
            //}
            
            
	        ModbusQuery(&ModbusH, telegram[1]); // make a query
	        u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
	        if(u32NotificationValue)
	        {
	  	    //handle error
		    //  while(1);
	        }
            vTaskDelay(100);
            ModbusDATA[0] = ModbusDATA[1]++;
            
        
	        ModbusQuery(&ModbusH, telegram[0]); // make a query
	        u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
	        if(u32NotificationValue)
	        {
	  	    //handle error
		    //  while(1);
	        }
            vTaskDelay(100);

                                  
    }
}



#define UART0_TX_PIN 0
#define UART0_RX_PIN 1

#define UART1_TX_PIN 4
#define UART1_RX_PIN 5


void initSerial()
{
    uart_init(uart0, 115200);
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(uart0, false);


    uart_init(uart1, 115200);
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(uart1, false);
}




int main() {


BaseType_t xReturned;
TaskHandle_t xHandle = NULL;

initSerial();


/* Create the task, storing the handle. */
    xReturned = xTaskCreate(
                    vTaskCode,       /* Function that implements the task. */
                    "Blinky task",   /* Text name for the task. */
                    512,             /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    tskIDLE_PRIORITY,/* Priority at which the task is created. */
                    &xHandle );   


  //ModbusH.uModbusType = MB_SLAVE;
  ModbusH.uModbusType = MB_MASTER;
  ModbusH.port = uart1;
  ModbusH.u8id = 0; // For master it must be 0
  ModbusH.u16timeOut = 1000;
  // ModbusH.EN_Port = NULL;
  ModbusH.EN_Port = (uint16_t *) 1; //enables the RS485 ChipSelect
  ModbusH.EN_Pin = 3; //Pi controlling RS485 ChipSelect
  ModbusH.u16regs = ModbusDATA;
  ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
  ModbusH.xTypeHW = USART_HW;
  //Initialize Modbus library
  ModbusInit(&ModbusH);
  //Start capturing traffic on serial Port
  ModbusStart(&ModbusH);




  ModbusH2.uModbusType = MB_SLAVE;
// ModbusH.uModbusType = MB_MASTER;
  ModbusH2.port = uart0;
  ModbusH2.u8id = 1; // For master it must be 0
  ModbusH2.u16timeOut = 1000;
  // ModbusH.EN_Port = NULL;
  ModbusH2.EN_Port = NULL; //enables the RS485 ChipSelect
  ModbusH2.EN_Pin = 2; //Pi controlling RS485 ChipSelect
  ModbusH2.u16regs = ModbusDATA2;
  ModbusH2.u16regsize= sizeof(ModbusDATA2)/sizeof(ModbusDATA2[0]);
  ModbusH2.xTypeHW = USART_HW;
  //Initialize Modbus library
  ModbusInit(&ModbusH2);
  //Start capturing traffic on serial Port
  ModbusStart(&ModbusH2);

  vTaskStartScheduler();
  while(1)
  {
      configASSERT(0);    /* We should never get here */
  }

}