#include <stdio.h>
#include "freertps/freertps.h"
#include "std_msgs/uint32.h"
#include "freertps/RTOSInterface.h"

//If use PID is set to 1 the program use closed loop control, if not the program use open loop control
#define usePID 0

//PID control task
static void ctrlTask( void *parameters );
//Voltage publisher task
static void pubCurVoltTask( void *parameters );
//Topic setPoint, uint32, subscriber task
static void desiredVoltSubTask( void *parameters );

//PWM init function
void initPWM();
//ADC init function
void initADC();
//Current ADC value function
uint16_t getADCValue();

//Voltage publisher variable
frudp_pub_t *pub;
//PID voltage setpoint variable. Start with 2048 what means 1.5 volts
uint32_t setPoint = 2048.0;

//Device ethernet MAC address
uint8_t ucMACAddress[ 6 ] = { 0x2C, 0x4D, 0x59, 0x01, 0x23, 0x51 };
//Desired IP parameter if DHCP do not work
static const uint8_t ucIPAddress[ 4 ]        = { 192, 168, 2, 151 };
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

  //Init ADC and PWM peripheral
  initADC();
  initPWM();

  //Pubs here. Example pub = freertps_create_pub( topicName, typeName );
  pub = freertps_create_pub( "currentVolt", std_msgs__uint32__type.rtps_typename );

  //Subs here. Example freertps_create_sub( topicName, typeName, handlerTask, dataQueueSize );
  freertps_create_sub( "desiredVolt", std_msgs__uint32__type.rtps_typename, desiredVoltSubTask, 10 );

  //General tasks here. ctrlTask with greater priority
  xTaskCreate( pubCurVoltTask, "pubCurVoltTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
  xTaskCreate( ctrlTask, "ctrlTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
}

//PID control task
static void ctrlTask( void *parameters )
{
  //PID variables
  double y0, y1, y2, e0, e1, e2;
  const double sampleTime = 0.01;
  const double kp = 0.36;
  const double ki = 0.01 * 2.0 / sampleTime;
  const double kd = 0.0025 * sampleTime / 2.0;

  //Init the variables
  e0 = e1 = e2 = y0 = y1 = y2 = 0;

  //PID loop
  while( true )
  {
#if usePID == 1
    //Doing the calculations
    y2 = y1;
    y1 = y0;
    e2 = e1;
    e1 = e0;
    e0 = ( double )( setPoint ) - ( double )( getADCValue() );
    y0 = y2 + kp * ( e0 - e2 ) + ki * ( e0 + 2 * e1 + e2 ) + kd * ( e0 - 2 * e1 + e2 );
#else
    //Considering that we received a 12 bit value, multiply it by 4 
    y0 = setPoint << 4;
#endif

    //Saturating at maximum value
    if( y0 > ( double )( 0xFFFE ) )
      y0 = ( double )( 0xFFFF );
    //Saturating at minimum value
    if( y0 < ( double )( 0x0001 ) )
      y0 = ( double )( 0x0000 );

    //Setting the new PWM value to the controled system
    TIM4->CCR4 = ( uint16_t )( y0 );
    //Delay sampleTime milliseconds to get the correct PID time control
    vTaskDelay( ( sampleTime * 1000.0 ) / portTICK_PERIOD_MS );
  }
}

//Voltage publisher task
static void pubCurVoltTask( void *parameters )
{
  //FreeRTPS publisher variables
  uint8_t cdr[ 20 ] = { 0 };
  int cdr_len;
  struct std_msgs__uint32 voltage;

  while( true )
  {
    //Getting the current voltage in digital value
    voltage.data = getADCValue();
    //serialize the data
    cdr_len = serialize_std_msgs__uint32( &voltage, cdr, sizeof( cdr ) );
    //Publish the data
    freertps_publish( pub, cdr, cdr_len );
    //Task period
    vTaskDelay( 250 / portTICK_PERIOD_MS );
  }
}

//Function to receive messages from setPoint topic
static void desiredVoltSubTask( void *parameters )
{
  //Queue responsible for handle receiver information on "setPoint" topic
  QueueHandle_t sQueue = ( QueueHandle_t )parameters;
  //Variables to handle the data
  uint8_t  msg[ 128 ] = { 0 };
  
  while( true )
  {
    //portMAX_DELAY tells that xQueueReceive will be blocked until data arrive in sQueue
    if( xQueueReceive( sQueue, msg, portMAX_DELAY ) == pdPASS )
    {
      //Led toggle to see that data has been arrived
      led_toggle();
      //Getting the received setpoint
			setPoint = *( ( uint32_t* )msg );
    }
  }
}

//Init ADC1 - Channel 11 - Pin C11
void initADC()
{
  //Enable PORT C clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  //Enable clocl for AD1
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  //Enale port C as analog
  GPIOC->MODER |= GPIO_MODER_MODER1_1 + GPIO_MODER_MODER1_0;
  //Set ADC1 with channel 11
  ADC1->SQR3 |= 0x0000000B;
  //Enable AD1
  ADC1->CR2 |= ADC_CR2_ADON;
}

//Init PWM - TIMER 4 - Channel 4 - PIN D15
void initPWM()
{
  //Enable PORT D clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  //Enale port D as alternate function (PWM)
  GPIOD->MODER |= GPIO_MODER_MODER15_1;
  //Alternate function for Pin D15 (TIMER 4 PWM Channel 4)
  GPIOD->AFR[ 1 ] |= 0x20000000;
  //Enable TIMER 4, used to PWM
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  //Timer 4 Auto-reload value
  TIM4->ARR |= 0x0000FFFF;
  //Set Timer 4 as PWM
  TIM4->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
  //TIMER 4 compare enable
  TIM4->CCER |= TIM_CCER_CC4E;
  //Enable TIMER 4
  TIM4->CR1 |= TIM_CR1_CEN;
}

//Routine to get current ADC1 value
uint16_t getADCValue()
{
  //Start AD1 conversion
  ADC1->CR2 |= ADC_CR2_SWSTART;
  //Wait for the end of AD1 conversion
  while( !( ADC1->SR & ADC_SR_EOC ) );
  //Get the conversion result
  return ADC1->DR;
}