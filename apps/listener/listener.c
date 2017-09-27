#include <stdio.h>
#include "freertps/freertps.h"
#include "std_msgs/string.h"
#include "freertps/RTOSInterface.h"

//Topic chatter, string, subscriber task
static void chatterSubTask( void *parameters );

//Device ethernet MAC address
uint8_t ucMACAddress[ 6 ] = { 0x2C, 0x4D, 0x59, 0x01, 0x23, 0x50 };
//Desired IP parameter if DHCP do not work
static const uint8_t ucIPAddress[ 4 ]        = { 192, 168, 2, 150 };
static const uint8_t ucNetMask[ 4 ]          = { 255, 255, 255, 0 };
static const uint8_t ucGatewayAddress[ 4 ]   = { 192, 168, 2, 0 };
static const uint8_t ucDNSServerAddress[ 4 ] = { 208, 67, 222, 222 };

//Main function
int main( void )
{
	//Do necessary clock configuration before FreeRTOS_IPInit
	//Start FreeRTOS+UDP stack
  FreeRTOS_IPInit( ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, ucMACAddress );
  //Start operating system scheduler
  vTaskStartScheduler();
  //Infinite loop, the program must not reach this loop if ok
  for(;;){}
  return 0;
}

//Function to setup pubs, subs and others tasks
//Pub handles must be declared outside this function
//For pubs, use: pub = freertps_create_pub( topic name, type name );
//For subs, use: freertps_create_sub( topic name, type name, handle task, receive queue size );
void setup( void )
{
  //Add here peripheral init function, subscribers, publishers and general tasks

  //Pubs here. Example pub = freertps_create_pub( topicName, typeName );

  //Subs here. Example freertps_create_sub( topicName, typeName, handlerTask, dataQueueSize );
  freertps_create_sub( "chatter", std_msgs__string__type.rtps_typename, chatterSubTask, 10 );

  //General tasks here
}


//Function to receive messages from chatter topic
static void chatterSubTask( void *parameters )
{
  //Queue responsible for handle receiver information on "chatter" topic
  QueueHandle_t sQueue = ( QueueHandle_t )parameters;
  //Variables to handle the data
  uint8_t  msg[ 128 ] = { 0 };
  char     data[ 128 ] = { 0 };
  uint32_t length;
  
  while( true )
  {
    //portMAX_DELAY tells that xQueueReceive will be blocked until data arrive in sQueue
    if( xQueueReceive( sQueue, msg, portMAX_DELAY ) == pdPASS )
    {
      //Led toggle to see that data has been arrived
      led_toggle();
      //Getting the received string string size
			length = *( ( uint32_t * )msg );
      //Getting the string data
			for( int i = 0; i < length && i < sizeof( data ) - 1; i++ )
			{
			  data[ i ] = ( ( uint8_t * )msg )[ 4 + i ];
			}
    }
  }
}