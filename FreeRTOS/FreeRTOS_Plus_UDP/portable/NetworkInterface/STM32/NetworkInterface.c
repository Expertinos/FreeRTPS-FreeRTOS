/*
 * FreeRTOS+UDP V1.0.4 (C) 2014 Real Time Engineers ltd.
 * All rights reserved
 *
 * This file is part of the FreeRTOS+UDP distribution.  The FreeRTOS+UDP license
 * terms are different to the FreeRTOS license terms.
 *
 * FreeRTOS+UDP uses a dual license model that allows the software to be used 
 * under a standard GPL open source license, or a commercial license.  The 
 * standard GPL license (unlike the modified GPL license under which FreeRTOS 
 * itself is distributed) requires that all software statically linked with 
 * FreeRTOS+UDP is also distributed under the same GPL V2 license terms.  
 * Details of both license options follow:
 *
 * - Open source licensing -
 * FreeRTOS+UDP is a free download and may be used, modified, evaluated and
 * distributed without charge provided the user adheres to version two of the
 * GNU General Public License (GPL) and does not remove the copyright notice or
 * this text.  The GPL V2 text is available on the gnu.org web site, and on the
 * following URL: http://www.FreeRTOS.org/gpl-2.0.txt.
 *
 * - Commercial licensing -
 * Businesses and individuals that for commercial or other reasons cannot comply
 * with the terms of the GPL V2 license must obtain a commercial license before 
 * incorporating FreeRTOS+UDP into proprietary software for distribution in any 
 * form.  Commercial licenses can be purchased from http://shop.freertos.org/udp 
 * and do not require any source files to be changed.
 *
 * FreeRTOS+UDP is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+UDP unless you agree that you use the software 'as is'.
 * FreeRTOS+UDP is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/udp
 *
 */

/* Standard includes. */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware abstraction. */


/* FreeRTOS+UDP includes. */
#include "FreeRTOS_UDP_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Sockets.h"
#include "NetworkBufferManagement.h"

/* Driver includes. */
#include "metal/systime.h"
#include "metal/delay.h"

/* Demo includes. */
#include "NetworkInterface.h"

/* When a packet is ready to be sent, if it cannot be sent immediately then the
task performing the transmit will block for niTX_BUFFER_FREE_WAIT
milliseconds.  It will do this a maximum of niMAX_TX_ATTEMPTS before giving
up. */
#define niTX_BUFFER_FREE_WAIT	( ( TickType_t ) 2UL / portTICK_RATE_MS )
#define niMAX_TX_ATTEMPTS		( 5 )

/*-----------------------------------------------------------*/

/*
 * A deferred interrupt handler task that processes
 */
static void prvEMACHandlerTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* The queue used to communicate Ethernet events with the IP task. */
extern xQueueHandle xNetworkEventQueue;

/* The semaphore used to wake the deferred interrupt handler task when an Rx
interrupt is received. */
static xSemaphoreHandle xEMACRxEventSemaphore = NULL;
/*-----------------------------------------------------------*/


#ifndef ENET_PHY_ADDR
#  define ENET_PHY_ADDR 0x00
#endif

#define ENET_NBUF     1500
#define ENET_DMA_NRXD   7
#define ENET_DMA_NTXD   7


typedef struct enet_dma_desc
{
  volatile uint32_t des0;
  volatile uint32_t des1;
  volatile uint32_t des2;
  volatile uint32_t des3;
} enet_dma_desc_t;

#define ALIGN4 __attribute__((aligned(4)));

static volatile enet_dma_desc_t g_enet_dma_rx_desc[ENET_DMA_NRXD] ALIGN4;
static volatile enet_dma_desc_t g_enet_dma_tx_desc[ENET_DMA_NTXD] ALIGN4;
static volatile uint8_t g_enet_dma_rx_buf[ENET_DMA_NRXD][ENET_NBUF] ALIGN4;
static volatile uint8_t g_enet_dma_tx_buf[ENET_DMA_NTXD][ENET_NBUF] ALIGN4;
static volatile enet_dma_desc_t *g_enet_dma_rx_next_desc = &g_enet_dma_rx_desc[0];
static volatile enet_dma_desc_t *g_enet_dma_tx_next_desc = &g_enet_dma_tx_desc[0];

uint16_t enet_read_phy_reg( const uint8_t reg_idx )
{
	while( ETH->MACMIIAR & ETH_MACMIIAR_MB ){}
	ETH->MACMIIAR = ( ENET_PHY_ADDR << 11 ) | ( ( reg_idx & 0x1F ) << 6 ) | ETH_MACMIIAR_CR_Div102 | ETH_MACMIIAR_MB;
	while( ETH->MACMIIAR & ETH_MACMIIAR_MB ){}
	return ETH->MACMIIDR & 0xFFFF;
}
/*-----------------------------------------------------------*/


void enet_write_phy_reg( const uint8_t reg_idx, const uint16_t reg_val )
{
  while( ETH->MACMIIAR & ETH_MACMIIAR_MB ){}
  ETH->MACMIIDR = reg_val;
  ETH->MACMIIAR = ( ENET_PHY_ADDR << 11 ) | ( ( reg_idx & 0x1F ) << 6 ) | ETH_MACMIIAR_CR_Div102 | ETH_MACMIIAR_MW | ETH_MACMIIAR_MB;
  while( ETH->MACMIIAR & ETH_MACMIIAR_MB ){}
  enet_read_phy_reg( reg_idx );
}
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceInitialise( void )
{
	BaseType_t xReturn;
	enet_mac_init_pins();

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST;

	for( volatile int i = 0; i < 1000; i++ ){}
	for( volatile int i = 0; i < 1000; i++ ){}

#ifndef ENET_USE_MII
	SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL; // set the MAC in RMII mode
#endif

	for( volatile int i = 0; i < 100000; i++ ){}
	RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACRXEN | RCC_AHB1ENR_ETHMACTXEN | RCC_AHB1ENR_ETHMACEN;
	for( volatile int i = 0; i < 100000; i++ ){}
	RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETHMACRST;
	for( volatile int i = 0; i < 100000; i++ ){}
	RCC->AHB1RSTR |= RCC_AHB1RSTR_ETHMACRST;
	for( volatile int i = 0; i < 100000; i++ ){}
	RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETHMACRST;
	for( volatile int i = 0; i < 100000; i++ ){}

	ETH->DMABMR |= ETH_DMABMR_SR;
	for( volatile int i = 0; i < 100000; i++ ){}
	while( ETH->DMABMR & ETH_DMABMR_SR ){}
	for( volatile int i = 0; i < 100000; i++ ){}
	ETH->DMAOMR |= ETH_DMAOMR_FTF;
	while( ETH->DMAOMR & ETH_DMAOMR_FTF ){}

	ETH->MACCR |= 0x02000000 | ETH_MACCR_FES | ETH_MACCR_DM | ETH_MACCR_IPCO | ETH_MACCR_APCS;
	ETH->MACFFR |= ETH_MACFFR_RA;

	/* 1 mS delay*/
	for( int i = 0; i < configCPU_CLOCK_HZ / 1000; i++ )
	{
	  asm( "nop" );
	}

	while( enet_read_phy_reg( 0 ) == 0xffff ){}

	for( int i = 0; i < ENET_DMA_NTXD; i++ )
	{
		g_enet_dma_tx_desc[i].des0 = 0x00100000 | 0x00c00000;
		g_enet_dma_tx_desc[i].des1 = 0;
		g_enet_dma_tx_desc[i].des2 = ( uint32_t )&g_enet_dma_tx_buf[ i ][ 0 ];

		if( i < ENET_DMA_NTXD - 1 )
		{
			g_enet_dma_tx_desc[ i ].des3 = ( uint32_t )&g_enet_dma_tx_desc[ i + 1 ];
		}
		else
		{
			g_enet_dma_tx_desc[ i ].des3 = ( uint32_t )&g_enet_dma_tx_desc[ 0 ];
		}
	}

	for( int i = 0; i < ENET_DMA_NRXD; i++ )
	{
		g_enet_dma_rx_desc[ i ].des0 = 0x80000000;
		g_enet_dma_rx_desc[ i ].des1 = 0x00004000 | ENET_NBUF;
		g_enet_dma_rx_desc[ i ].des2 = (uint32_t)&g_enet_dma_rx_buf[ i ][ 0 ];

		if( i < ENET_DMA_NRXD - 1 )
		{
			g_enet_dma_rx_desc[ i ].des3 = ( uint32_t )&g_enet_dma_rx_desc[ i + 1 ];
		}
		else
		{
			g_enet_dma_rx_desc[ i ].des3 = ( uint32_t )&g_enet_dma_rx_desc[ 0 ];
		}
	}

	ETH->DMATDLAR = ( uint32_t )&g_enet_dma_tx_desc[ 0 ];
	ETH->DMARDLAR = ( uint32_t )&g_enet_dma_rx_desc[ 0 ];
	ETH->DMAOMR = ETH_DMAOMR_TSF;
	ETH->DMABMR |= ETH_DMABMR_AAB;

	ETH->DMAIER = ETH_DMAIER_NISE | ETH_DMAIER_RIE;
	ETH->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;

	//How to know if the hardware was not initilizes corretly
	if( 1 )
	{
		if( xEMACRxEventSemaphore == NULL )
		{
			vSemaphoreCreateBinary( xEMACRxEventSemaphore );
		}

		configASSERT( xEMACRxEventSemaphore );

		/* The handler task is created at the highest possible priority to
		ensure the interrupt handler can return directly to it. */

		xTaskCreate( prvEMACHandlerTask, "EMAC", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL );

		/* Enable the interrupt and set its priority to the minimum
		interrupt priority.  */
		NVIC_SetPriority( ETH_IRQn, 5 );
		NVIC_EnableIRQ( ETH_IRQn );
		ETH->DMAOMR |= ETH_DMAOMR_ST | ETH_DMAOMR_SR;

		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceOutput( xNetworkBufferDescriptor_t * const pxNetworkBuffer )
{
	BaseType_t xReturn = pdFAIL;
	int32_t x;
	volatile uint32_t tps;

	/* Attempt to obtain access to a Tx buffer. */
	for( x = 0; x < niMAX_TX_ATTEMPTS; x++ )
	{
		/* Verify if the DMA descriptor are free to send a frame */
		if( !( g_enet_dma_tx_next_desc->des0 & 0x80000000 ) )
		{
			/* Will the data fit in the Tx buffer? */
			if( pxNetworkBuffer->xDataLength < ENET_NBUF ) /*_RB_ The size needs to come from FreeRTOSIPConfig.h. */
			{
				/* Assign the buffer to the Tx descriptor that is now known to
				be free. */
				memcpy( g_enet_dma_tx_next_desc->des2, pxNetworkBuffer->pucEthernetBuffer, pxNetworkBuffer->xDataLength );
				g_enet_dma_tx_next_desc->des1  = ( uint32_t )pxNetworkBuffer->xDataLength;

				/* Initiate the Tx. */
				g_enet_dma_tx_next_desc->des0 |= 0x30000000 | 0x80000000;

				/*delay*/
				for( int i = 0; i < configCPU_CLOCK_HZ / 100000000; i++ )
				{
				  asm( "nop" );
				}

				tps = ETH->DMASR & ETH_DMASR_TPS;

				/* Verify if the DMA is suspended.If it is, put it to run again and transmit the descriptors*/				
				if( tps == ETH_DMASR_TPS_Suspended )
				{
					ETH->DMATPDR = 0;
				}

				/* Set DMA descriptor to the next transmition descriptor */
				g_enet_dma_tx_next_desc = (enet_dma_desc_t *)g_enet_dma_tx_next_desc->des3;

				/* The Tx has been initiated. */
				xReturn = pdTRUE;
			}
			break;
		}
		else
		{
			vTaskDelay( niTX_BUFFER_FREE_WAIT );
		}
	}

	/* Finished with the network buffer. */
	vNetworkBufferRelease( pxNetworkBuffer );

	return xReturn;
}
/*-----------------------------------------------------------*/

void eth_vector( void )
{
	volatile uint32_t dmasr = ETH->DMASR;
	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	
	ETH->DMASR = dmasr;

	if( dmasr & ETH_DMASR_RS )
	{
		xSemaphoreGiveFromISR( xEMACRxEventSemaphore, &xHigherPriorityTaskWoken );
	}

	/* ulInterruptCause is used for convenience here.  A context switch is
	wanted, but coding portEND_SWITCHING_ISR( 1 ) would likely result in a
	compiler warning. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

static void prvEMACHandlerTask( void *pvParameters )
{
	xNetworkBufferDescriptor_t *pxNetworkBuffer;
	size_t xBytesReceived;
	xIPStackEvent_t xRxEvent;

	( void )pvParameters;
	configASSERT( xEMACRxEventSemaphore );

	for( ;; )
	{
		/* Wait for the Ethernet MAC interrupt to indicate that another packet
        has been received.  It is assumed xEMACRxEventSemaphore is a counting
        semaphore (to count the Rx events) and that the semaphore has already
        been created. */
		while( xSemaphoreTake( xEMACRxEventSemaphore, portMAX_DELAY ) == pdFALSE );

		/* At least one packet has been received. */
		while( !( g_enet_dma_rx_next_desc->des0 & 0x80000000 ) )
		{
			/* See how much data was received. */
			xBytesReceived = ( size_t )( ( g_enet_dma_rx_next_desc->des0 & 0x3FFF0000 ) >> 16 );

			if( xBytesReceived > 0 )
			{
				/* Allocate a network buffer descriptor that references an Ethernet
				buffer large enough to hold the received data. */
				pxNetworkBuffer = pxNetworkBufferGet( xBytesReceived, ( TickType_t ) 0 );

				if( pxNetworkBuffer != NULL )
				{
					/* pxNetworkBuffer->pucEthernetBuffer now points to an Ethernet
					buffer large enough to hold the received data.  Copy the
					received data into pcNetworkBuffer->pucEthernetBuffer. */
					memcpy( pxNetworkBuffer->pucEthernetBuffer, g_enet_dma_rx_next_desc->des2, xBytesReceived );
					pxNetworkBuffer->xDataLength = xBytesReceived;

	                /* See if the data contained in the received Ethernet frame needs
	                to be processed. */
	                if( eConsiderFrameForProcessing( pxNetworkBuffer->pucEthernetBuffer ) == eProcessBuffer )
	                {
	                    /* The event about to be sent to the IP stack is an Rx event. */
	                    xRxEvent.eEventType = eEthernetRxEvent;

	                    /* pvData is used to point to the network buffer descriptor that
	                    references the received data. */
	                    xRxEvent.pvData = pxNetworkBuffer;

	                    /* Send the data to the IP stack. */
	                    if( xQueueSendToBack( xNetworkEventQueue, &xRxEvent, 0 ) == pdFALSE )
	                    {
	                        /* The buffer could not be sent to the IP task so the buffer
	                        must be released. */
	                        vNetworkBufferRelease( pxNetworkBuffer );
	                    }
	                    else
	                    {
	                        /* The message was successfully sent to the IP stack. */
	                    }
	                }
	                else
	                {
	                    /* The Ethernet frame can be dropped, but the Ethernet buffer
	                    must be released. */
	                    vNetworkBufferRelease( pxNetworkBuffer );
	                }
				}
				else
				{
					/* RX event lost */
				}
			}

			g_enet_dma_rx_next_desc->des0 |= 0x80000000;
			g_enet_dma_rx_next_desc = ( enet_dma_desc_t * )g_enet_dma_rx_next_desc->des3;
		}
	}
}
/*-----------------------------------------------------------*/
