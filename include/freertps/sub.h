#ifndef SUBSCRIPTION_H
#define SUBSCRIPTION_H

#include "freertps/id.h"
#include "freertps/udp.h"
#include "freertps/freertps.h"
#include "freertps/config.h"
#include "FreeRTOS.h"
#include "queue.h"

// create userland UDP subscriptions. people should call the
// freertps_create_subscription() from userland code though, to be agnostic
// to the physical layer

// could be dangerous to hold onto string pointers... they need to be
// stored in the caller's constant memory. maybe revisit this at some point,
// with a #define switch somewhere to have it use more memory for string
// buffers, etc.

//alterei
void frudp_add_user_sub(const char *topic_name, const char *type_name, QueueHandle_t *xQueue);
//void frudp_add_user_sub(const char *topic_name, const char *type_name, freertps_msg_cb_t msg_cb);

// this is the private subscribe function used internally... should be hidden
// eventually.
/*
bool frudp_subscribe(const frudp_entity_id_t reader_id,
                     const frudp_entity_id_t writer_id,
                     const frudp_rx_data_cb_t data_cb,
                     const freertps_msg_cb_t msg_cb);
*/

//alterei
uint8_t Message[ 128 ];

typedef struct
{
  const char *topic_name;
  const char *type_name;
  frudp_eid_t reader_eid;
  frudp_rx_data_cb_t data_cb;
  bool msg_cb;
  bool reliable;
  QueueHandle_t *xQueue;
} frudp_sub_t;

void frudp_add_sub(const frudp_sub_t *s);

extern frudp_sub_t g_frudp_subs[FRUDP_MAX_SUBS];
extern uint32_t g_frudp_num_subs;

typedef struct
{
  bool reliable;
  frudp_guid_t writer_guid;
  frudp_eid_t reader_eid;
  frudp_sn_t max_rx_sn;
  frudp_rx_data_cb_t data_cb;
  bool msg_cb;
  QueueHandle_t *xQueue;
} frudp_reader_t;

// not great to have these freely available. someday hide these.
extern frudp_reader_t g_frudp_readers[FRUDP_MAX_READERS];
extern uint32_t g_frudp_num_readers;

int frudp_add_reader(const frudp_reader_t *reader);

void frudp_print_readers(void);

#endif
