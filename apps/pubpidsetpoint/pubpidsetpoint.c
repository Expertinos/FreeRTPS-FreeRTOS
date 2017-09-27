#include <stdio.h>
#include "freertps/freertps.h"
#include "std_msgs/uint32.h"
#include "freertps/RTOSInterface.h"

//Voltage setpoint publisher task
static void pubTask( void *parameters );

//Voltage setpoint publisher variable
frudp_pub_t *pub;

//Device ethernet MAC address
uint8_t ucMACAddress[ 6 ] = { 0x2C, 0x4D, 0x59, 0x01, 0x23, 0x52 };
//Desired IP parameter if DHCP do not work
static const uint8_t ucIPAddress[ 4 ]        = { 192, 168, 2, 152 };
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
  pub = freertps_create_pub( "desiredVolt", std_msgs__uint32__type.rtps_typename );

  //Subs here. Example freertps_create_sub( topicName, typeName, handlerTask, dataQueueSize );

  //General tasks here. ctrlTask with greater priority
  xTaskCreate( pubTask, "pubTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
}

//Uint32 publisher task
static void pubTask( void *parameters )
{
  //FreeRTPS publisher variables
  uint8_t cdr[ 20 ] = { 0 };
  int cdr_len;
  struct std_msgs__uint32 digital12bitsVoltage;

  //A discrete sin wave oscilating from -1.5 to 1.5 in 20 steps
  double voltageSetpoints[ 20 ] = { 0, 0.463, 0.881, 1.213, 1.426, 1.5, 1.427, 1.2145, 0.883, 0.4655, 0, -0.463, -0.881, -1.213, -1.426, -1.5, -1.427, -1.2145, -0.883, -0.4655 };
  //Variable to store the actual position of array to read
  int pos = 0;

  while( true )
  {
    //Blink led each time that loop start
    led_toggle();
    //Setting the new position on the array to get the next value
    pos = ( pos + 1 ) % 20;
    //Offset the value and multiplying for 1365, getting a final value from 0 to 4096
    digital12bitsVoltage.data = ( uint32_t )( ( voltageSetpoints[ pos ] + 1.5 ) * 1365.0 );
    //serialize the data
    cdr_len = serialize_std_msgs__uint32( &digital12bitsVoltage, cdr, sizeof( cdr ) );
    //Publish the data
    freertps_publish( pub, cdr, cdr_len );
    //Task period
    vTaskDelay( 50 / portTICK_PERIOD_MS );
  }
}