#ifndef FREERTPS_CONFIG_H
#define FREERTPS_CONFIG_H

#include <stdint.h>
#include "freertps/id.h"


// default multicast group is 239.255.0.1
#define FRUDP_DEFAULT_MCAST_GROUP 0xEFFF0001
//#define FRUDP_DEFAULT_MCAST_GROUP 0xFFFFFFFF
//#define FRUDP_DOMAIN_ID  0

extern uint8_t ucMACAddress[ 6 ];
extern volatile int FRUDP_IP4_ADDR;

#define FRUDP_MAX_PUBS 15
#define FRUDP_MAX_SUBS 15
#define FRUDP_MAX_READERS 15
#define FRUDP_MAX_WRITERS 15
#define FRUDP_DISCO_MAX_PARTS 10

#define FRUDP_MAX_TOPIC_NAME_LEN 128
#define FRUDP_MAX_TYPE_NAME_LEN  128

typedef struct
{
  frudp_guid_prefix_t guid_prefix;
  int participant_id;
  uint32_t domain_id;
  uint32_t unicast_addr;
} frudp_config_t;
extern frudp_config_t g_frudp_config;

#define VERBOSE_MSG_RX
//#define VERBOSE_HEARTBEAT
//#define VERBOSE_DATA
//#define VERBOSE_ACKNACK
//#define VERBOSE_GAP

//#define VERBOSE_TX_ACKNACK
//#define SEDP_VERBOSE

#endif
