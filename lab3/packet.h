#ifndef __PACKET_H__
#define __PACKET_H__

#include <stdint.h>

#define GATE_ID   0
#define NUM_NODES 4

typedef enum {
  packet_type_sensor,
  packet_type_network,
  packet_type_config
} packet_type;

typedef struct {
  uint16_t    pkt_num;
  uint16_t    reading;
} sensor_packet_t;

typedef struct {
  uint16_t    pkt_num;
  uint16_t    neighbors[NUM_NODES];
} network_packet_t;

typedef enum {
  config_type_sensor,
  config_type_network
} config_type;

typedef struct {
  uint16_t    pkt_num;
  config_type cfg_type;
  nrk_time_t  period;
} config_packet_t;

typedef struct {
  packet_type pkt_type;
  uint8_t     sender;
  uint8_t     forwarder;
  uint8_t     receiver;
} header_t;

typedef struct {
  header_t header;
  union {
    sensor_packet_t  sensor_packet;
    network_packet_t network_packet;
    config_packet_t  config_packet;
  } packet;
} packet_t;

#define SENSOR_PKT_LEN							\
  (sizeof(sensor_packet_t) + sizeof(header_t))
#define NETWORK_PKT_LEN							\
  (sizeof(network_packet_t) + sizeof(header_t))
#define CONFIG_PKT_LEN							\
  (sizeof(config_packet_t) + sizeof(header_t))


#endif // __PACKET_H__
