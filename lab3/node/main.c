/******************************************************************************
 *  Nano-RK, a real-time operating system for sensor networks.
 *  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
 *  All rights reserved.
 *
 *  This is the Open Source Version of Nano-RK included as part of a Dual
 *  Licensing Model. If you are unsure which license to use please refer to:
 *  http://www.nanork.org/nano-RK/wiki/Licensing
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2.0 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *******************************************************************************/

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <bmac.h>
#include <nrk_error.h>
#include <string.h>

#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>
#include <nrk_cfg.h>

#include <../debug.h>
#include <../packet.h>

#undef  NRK_APP_STACKSIZE
#define NRK_APP_STACKSIZE 512

#define NODE_ID    0x0003

void nrk_create_taskset ();
void nrk_register_drivers();
void whacky_task (void);

nrk_task_type SEND_SENSOR_TASK;
nrk_task_type SEND_NETWORK_TASK;
nrk_task_type RECEIVE_TASK;

NRK_STK send_sensor_task_stack[NRK_APP_STACKSIZE];
NRK_STK send_network_task_stack[NRK_APP_STACKSIZE];
NRK_STK receive_task_stack[NRK_APP_STACKSIZE];

uint16_t sensor_packet_number[NUM_NODES] = {0};
uint16_t network_packet_number[NUM_NODES] = {0};
uint16_t config_packet_number[NUM_NODES]  = {0};

union {
  uint8_t  buffer[RF_MAX_PAYLOAD_SIZE];
  packet_t packet; 
} tx_sensor_buf, tx_network_buf, rx_buf;

uint16_t adj_matrix[NUM_NODES][NUM_NODES] = {{0}, {0}, {0}, {0}};

#define DEFAULT_PERIOD_S  3
#define DEFAULT_PERIOD_NS 0

nrk_time_t sensor_period  = {DEFAULT_PERIOD_S, DEFAULT_PERIOD_NS};
nrk_time_t network_period = {DEFAULT_PERIOD_S, DEFAULT_PERIOD_NS};
nrk_sem_t* my_sem;



int main ()
{
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);

  nrk_init ();

  nrk_led_clr (0);
  nrk_led_clr (1);
  nrk_led_clr (2);
  nrk_led_clr (3);

  nrk_time_set (0, 0);

  bmac_task_config ();

  nrk_register_drivers();
  nrk_create_taskset ();
  my_sem = nrk_sem_create(1,4);
  if (my_sem == NULL){
        nrk_kprintf(PSTR("Error creating semaphore.\r\n"));
  }
  nrk_start ();

  return 0;
}

/* 
	This function sends a message containing the light sensor reading
	from that particular node.
	
 */
void send_sensor_task(void)
{
  uint8_t          fd;
  sensor_packet_t *sensor_packet = &tx_sensor_buf.packet.packet.sensor_packet;
  int8_t           val;

  tx_sensor_buf.packet.pkt_type = packet_type_sensor;
  tx_sensor_buf.packet.sender   = NODE_ID;
  tx_sensor_buf.packet.receiver = GATE_ID;

  sensor_packet->pkt_num = 1;

  while (!bmac_started()) {
    nrk_wait(sensor_period); //FIXME: some shorter amount of time?
  }

  fd = nrk_open(FIREFLY_SENSOR_BASIC,READ);
  if (fd == NRK_ERROR) {
    nrk_kprintf(PSTR("Failed to open sensor driver\r\n"));
  }
  
  val = nrk_set_status(fd, SENSOR_SELECT, LIGHT);
  if (val != NRK_OK) {
    nrk_kprintf(PSTR("Failed to set to light reading\r\n"));
  }

  while (1) {
    val = nrk_read(fd,
		   (uint8_t*)&sensor_packet->reading,
		   sizeof(sensor_packet->reading)
		   );
    if (val == NRK_ERROR) {
      DBG_KPRINTF("Could not read light!\r\n");
    }
    printf("reading %d\r\n", sensor_packet->reading) ;
    val = bmac_tx_pkt((uint8_t*)&tx_sensor_buf, SENSOR_PKT_LEN);
    if (val != NRK_OK) {
      DBG_KPRINTF("Could not Transmit!\r\n");
    }

    sensor_packet->pkt_num++;

    nrk_wait(sensor_period);
  }
}

/* 
	This sends a network message meant for the gateway. Updates array
	of neighbors
	
 */
void send_network_task(void)
{
  network_packet_t *network_packet = &tx_network_buf.packet.packet.network_packet;
  int8_t            val;

  tx_network_buf.packet.pkt_type = packet_type_sensor;
  tx_network_buf.packet.sender   = NODE_ID;
  tx_network_buf.packet.receiver = GATE_ID;

  network_packet->pkt_num = 1;

  while (!bmac_started()) {
    nrk_wait(network_period); //FIXME: some shorter amount of time?
  }

  while (1) {
    memcpy(network_packet->neighbors,
	   adj_matrix[NODE_ID],
	   NUM_NODES*sizeof(int16_t)
	   );
    
	//Send the message
    val = bmac_tx_pkt((uint8_t*)&tx_network_buf, NETWORK_PKT_LEN);
    if (val != NRK_OK) {
      DBG_KPRINTF("Could not Transmit!\r\n");
    }
	
	//increment packet number before sending again.
    network_packet->pkt_num++;

    nrk_wait(network_period);
  }
}

void receive_task(void)
{
  uint8_t i=0;
  uint8_t len =0;
  int8_t rssi = 0;
  printf("Receive Task PID = %d\r\n", nrk_get_pid());
  bmac_init(17);
  bmac_rx_pkt_set_buffer((uint8_t *)rx_buf.buffer, sizeof(rx_buf));
  
  while(1){
	//nrk_kprintf(PSTR("I'm RECEIVER\r\n"));
  	packet_t *local;
	uint8_t val=0;
	uint8_t rval=0;
	while(bmac_rx_pkt_ready()){
	  local = (packet_t*) bmac_rx_pkt_get(&len, &rssi);
	  uint8_t route=0;
    	  switch (local->pkt_type) {
    	  	case packet_type_sensor:
			  // check packet num
			  if (local->packet.sensor_packet.pkt_num <= sensor_packet_number[local->sender]){
				bmac_rx_pkt_release ();
				continue;}
			  sensor_packet_number[local->sender] = local->packet.sensor_packet.pkt_num;
			  // update rssi no matter what
				// using semaphore
				rval = nrk_sem_pend(my_sem);
				if (rval== NRK_ERROR) nrk_kprintf(PSTR("Error pending\r\n"));
				adj_matrix[NODE_ID][local->forwarder] = rssi; 
				rval = nrk_sem_post(my_sem);
				if (rval== NRK_ERROR) nrk_kprintf(PSTR("Error posting\r\n"));
			  // print for gateway only
			  if (NODE_ID == GATE_ID && local->receiver == NODE_ID){
				printf("Get Reading %d from node %d\r\n", local->packet.sensor_packet.reading, local->sender);
			  }
			  
			  else{route=1;}
			  bmac_rx_pkt_release ();
			  break;

    	  	case packet_type_network:
			  // check packet num
			  if (local->packet.network_packet.pkt_num <= network_packet_number[local->sender]){
				bmac_rx_pkt_release ();
				continue;}
			  network_packet_number[local->sender] = local->packet.network_packet.pkt_num;
			  // update rssi no matter what
				// using semaphore
				rval = nrk_sem_pend(my_sem);
				if (rval== NRK_ERROR) nrk_kprintf(PSTR("Error pending\r\n"));
				adj_matrix[NODE_ID][local->forwarder] = rssi; 
				rval = nrk_sem_post(my_sem);
				if (rval== NRK_ERROR) nrk_kprintf(PSTR("Error posting\r\n"));
			  // check receiver
			  if (local->receiver == NODE_ID){
				// always store the info for now, in fact only gateway needs this
				for (i=0;i<NUM_NODES;i++){
					adj_matrix[local->sender][i] = local->packet.network_packet.neighbors[i];
				}		
			  }
			  else{route=1;}
			  bmac_rx_pkt_release ();
			  break;

    	  	case packet_type_config:
			  // check packet num
			  if (local->packet.config_packet.pkt_num <= config_packet_number[local->sender]){
				bmac_rx_pkt_release ();
				continue;}
			  config_packet_number[local->sender] = local->packet.config_packet.pkt_num;
			  // check receiver   
			  if (local->receiver == NODE_ID){
				adj_matrix[local->sender][NODE_ID] = rssi;
				if(local->packet.config_packet.cfg_type==config_type_sensor){
				// using semaphore
					rval = nrk_sem_pend(my_sem);
					if (rval== NRK_ERROR) nrk_kprintf(PSTR("Error pending\r\n"));
					sensor_period = local->packet.config_packet.period;
					rval = nrk_sem_post(my_sem);
					if (rval== NRK_ERROR) nrk_kprintf(PSTR("Error posting\r\n"));
				}
				else if(local->packet.config_packet.cfg_type==config_type_network){
				// using semaphore
					rval = nrk_sem_pend(my_sem);
					if (rval== NRK_ERROR) nrk_kprintf(PSTR("Error pending\r\n"));
					network_period = local->packet.config_packet.period;
					rval = nrk_sem_post(my_sem);
					if (rval== NRK_ERROR) nrk_kprintf(PSTR("Error posting\r\n"));
				}
				else{
					printf("Error: unidentified cfg_type %d\r\n", local->packet.config_packet.cfg_type);
				}
			  }
			  else{route=1;}
			  bmac_rx_pkt_release ();
			  break;
		  
		default:
		  nrk_kprintf(PSTR("Unidentified packet type.\r\n"));
		  bmac_rx_pkt_release ();
		  break;
	  }
	  //
	  if (route==1){//broadcast the packet
	  	val = bmac_tx_pkt((uint8_t*)&local, sizeof(local));
		if (val==NRK_OK){
			nrk_kprintf(PSTR("Packet Transfered Success.\r\n"));}
		else{
			nrk_kprintf(PSTR("Packet Transfered Failed.\r\n"));}
	  }
	}
  nrk_wait_until_next_period();
  }//end of while
}

void nrk_create_taskset ()
{
  // TODO change naming and values
  SEND_SENSOR_TASK.task = send_sensor_task;
  nrk_task_set_stk(&SEND_SENSOR_TASK, send_sensor_task_stack, NRK_APP_STACKSIZE);
  SEND_SENSOR_TASK.prio = 2;
  SEND_SENSOR_TASK.FirstActivation = TRUE;
  SEND_SENSOR_TASK.Type = BASIC_TASK;
  SEND_SENSOR_TASK.SchType = PREEMPTIVE;
  SEND_SENSOR_TASK.period.secs = 5;
  SEND_SENSOR_TASK.period.nano_secs = 0;
  SEND_SENSOR_TASK.cpu_reserve.secs = 3;
  SEND_SENSOR_TASK.cpu_reserve.nano_secs = 0;
  SEND_SENSOR_TASK.offset.secs = 0;
  SEND_SENSOR_TASK.offset.nano_secs = 0;

  nrk_activate_task(&SEND_SENSOR_TASK);

  SEND_NETWORK_TASK.task = send_sensor_task;
  nrk_task_set_stk(&SEND_NETWORK_TASK, send_network_task_stack, NRK_APP_STACKSIZE);
  SEND_NETWORK_TASK.prio = 3;
  SEND_NETWORK_TASK.FirstActivation = TRUE;
  SEND_NETWORK_TASK.Type = BASIC_TASK;
  SEND_NETWORK_TASK.SchType = PREEMPTIVE;
  SEND_NETWORK_TASK.period.secs = 5;
  SEND_NETWORK_TASK.period.nano_secs = 0;
  SEND_NETWORK_TASK.cpu_reserve.secs = 3;
  SEND_NETWORK_TASK.cpu_reserve.nano_secs = 0;
  SEND_NETWORK_TASK.offset.secs = 0;
  SEND_NETWORK_TASK.offset.nano_secs = 0;

  nrk_activate_task(&SEND_NETWORK_TASK);

  RECEIVE_TASK.task = receive_task;
  nrk_task_set_stk(&RECEIVE_TASK, receive_task_stack, NRK_APP_STACKSIZE);
  RECEIVE_TASK.prio = 1;
  RECEIVE_TASK.FirstActivation = TRUE;
  RECEIVE_TASK.Type = BASIC_TASK;
  RECEIVE_TASK.SchType = PREEMPTIVE;
  RECEIVE_TASK.period.secs = 5;
  RECEIVE_TASK.period.nano_secs = 0;
  RECEIVE_TASK.cpu_reserve.secs = 3;
  RECEIVE_TASK.cpu_reserve.nano_secs = 0;
  RECEIVE_TASK.offset.secs = 0;
  RECEIVE_TASK.offset.nano_secs = 0;

  nrk_activate_task(&RECEIVE_TASK);

  printf ("Create done\r\n");
}
void nrk_register_drivers()
{
  int8_t val;

  // Register the Basic FireFly Sensor device driver
  // Make sure to add: 
  //     #define NRK_MAX_DRIVER_CNT  
  //     in nrk_cfg.h
  // Make sure to add: 
  //     SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/ff_basic_sensor.c
  //     in makefile
  val=nrk_register_driver( &dev_manager_ff3_sensors,FIREFLY_SENSOR_BASIC);
  if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to load my ADC driver\r\n") );

}
