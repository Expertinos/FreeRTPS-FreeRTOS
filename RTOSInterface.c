#include "freertps/RTOSInterface.h"

//TODO mestrado. Patterns names in FreeRTOS
static void RTOSListenTask(void * parameters);
static void RTOSDiscotickTask(void * parameters);
static void RTOSSystemInitTask(void * parameters);

void vApplicationMallocFailedHook( void ){}
void vApplicationIdleHook( void ){}
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName ){}
void vApplicationTickHook( void ){}
void vApplicationPingReplyHook( ePingReplyStatus_t eStatus, uint16_t usIdentifier ){}

//TODO: Put priority in sub function
//TODO: Do sub to other types
//TODO: How would be a good way to define type in subs
static void RTOSSystemInitTask( void * parameters )
{
  freertps_system_init();
  setup();
  frudp_disco_start();

  xTaskCreate( RTOSListenTask, "RTOSListenTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
  xTaskCreate( RTOSDiscotickTask, "RTOSDiscotickTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );

  vTaskSuspend( NULL );
  while( true ){}
}

//Task responsible for send device discovery messages
static void RTOSDiscotickTask( void * parameters )
{
  while( true )
  {
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    frudp_disco_tick();
  }
}

//Task responsible for decode incoming RTPS messages 
static void RTOSListenTask( void * parameters )
{
  while( true )
  {
    vTaskDelay( 1 / portTICK_PERIOD_MS );
    frudp_listen();
  }
}

//FreeRTOS+UDP init callback is called when when the UDP system has started successful
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
  led_on();
  xTaskCreate( RTOSSystemInitTask, "RTOSSystemInitTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
}