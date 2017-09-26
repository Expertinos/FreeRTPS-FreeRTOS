#ifndef FREERTPS_H
#define FREERTPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

// NOTE: the prefix freertps_udp_ is too long to type, so it will often
// be shortened to frudp_

typedef void (*freertps_msg_cb_t)(const void *msg);

#include "freertps/udp.h"
#include "freertps/config.h"
#include "freertps/time.h"
#include "freertps/ports.h"
#include "freertps/locator.h"
#include "freertps/disco.h"
#include "freertps/system.h"
#include "freertps/pub.h"
#include "freertps/sub.h"


#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


typedef union rtps_active_psms
{
    uint32_t val;
    struct rtps_active_psms_mask
    {
      uint32_t udp : 1;
      uint32_t ser : 1;
    } s;
} __attribute__((packed)) rtps_active_psms_t;

extern union rtps_active_psms g_rtps_active_psms;
extern bool g_freertps_init_complete;

//alterei
void freertps_create_sub( const char *topic_name, const char *type_name, TaskFunction_t pvTaskCode, const int size );
//void freertps_create_sub(const char *topic_name, const char *type_name, freertps_msg_cb_t msg_cb);
frudp_pub_t *freertps_create_pub( const char *topic_name, const char *type_name );
bool freertps_publish( frudp_pub_t *pub, const uint8_t *msg, const uint32_t msg_len );
void freertps_start( void );

#ifdef __cplusplus
}
#endif

#endif
