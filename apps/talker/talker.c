#include <stdio.h>
#include "freertps/freertps.h"
#include "std_msgs/string.h"
#include "freertps/RTOSInterface.h"

//Chatter publisher task
static void pubTask( void *parameters );

//Voltage publisher variable
frudp_pub_t *pub;

//Device ethernet MAC address
uint8_t ucMACAddress[ 6 ] = { 0x2C, 0x4D, 0x59, 0x01, 0x23, 0x49 };
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
  pub = freertps_create_pub( "chatter", std_msgs__string__type.rtps_typename );

  //Subs here. Example freertps_create_sub( topicName, typeName, handlerTask, dataQueueSize );

  //General tasks here. ctrlTask with greater priority
  xTaskCreate( pubTask, "pubTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
}

//Voltage publisher task
static void pubTask( void *parameters )
{
	//FreeRTPS publisher variables
	struct std_msgs__string msg;
	char data_buf[ 50 ] = { 0 };
	uint8_t cdr[ 50 ] = { 0 };
	int cdr_len;
	int cont;

  msg.data = data_buf;

  while( true )
  {
  	//Fomating the string
  	snprintf( msg.data, sizeof( data_buf ), "Sending: %d", cont++ );
  	cdr_len = serialize_std_msgs__string( &msg, cdr, sizeof( cdr ) );
    //Publish string information, cdr, on publisher "pub", which send data to topic "chatter"
    freertps_publish( pub, cdr, cdr_len );
    //Task period
    vTaskDelay( 250 / portTICK_PERIOD_MS );
  }
}