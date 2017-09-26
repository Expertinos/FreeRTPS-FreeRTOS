#include <stdlib.h>
#include <stdbool.h>
#include "freertps/freertps.h"
#include "freertps/udp.h"
#include "freertps/sub.h"

bool g_freertps_init_complete;

void freertps_perish_if(bool b, const char *msg)
{
  if (b)
  {
    #ifdef PRINT_DEBUG
    FREERTPS_FATAL("%s\n", msg);
    #endif
    exit(1);
  }
}

//alterei
//TODO: ( mestrado ), Add size of the queue, how to determine the buffer size, how to determine priority
//Advise from FreeRTOS: If you are using FreeRTOS-MPU then it is recommended to use xTaskCreateRestricted() in place of xTaskCreate().

void freertps_create_sub( const char *topic_name, const char *type_name, TaskFunction_t pvTaskCode, const int size )
//void freertps_create_sub(const char *topic_name, const char *type_name, freertps_msg_cb_t msg_cb)
{
  QueueHandle_t xQueue = xQueueCreate( size, sizeof( Message ) );

  if( xQueue != NULL )
  {    
    if( xTaskCreate( pvTaskCode, topic_name, configMINIMAL_STACK_SIZE, xQueue, tskIDLE_PRIORITY + 1, NULL ) == pdPASS )
    {
      frudp_add_user_sub( topic_name, type_name, xQueue );
      return true;
    }
  }

  return false;
}

frudp_pub_t *freertps_create_pub( const char *topic_name, const char *type_name )
{
  return frudp_create_user_pub( topic_name, type_name );
}

bool freertps_publish(frudp_pub_t *pub, const uint8_t *msg, const uint32_t msg_len)
{
  // todo: other physical layers...
  return frudp_publish_user_msg(pub, msg, msg_len);
}

void freertps_start(void)
{
  // todo: other physical layers...
  frudp_disco_start();
}
