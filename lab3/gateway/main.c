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

#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>
#include <nrk_cfg.h>

//#include <pthread.h>
#include <../debug.h>
#include <../packet.h>

#undef  NRK_APP_STACKSIZE
#define NRK_APP_STACKSIZE 512
#define NODE_ID    0x0000
#define GATE_ID    0x0000
#define LAST_NODE		2
void nrk_create_taskset ();
void nrk_register_drivers();

nrk_task_type RECEIVE_TASK;

NRK_STK receive_stack[NRK_APP_STACKSIZE];
nrk_sem_t *my_sem;


union {
  uint8_t  buffer[RF_MAX_PAYLOAD_SIZE];
  packet_t packet; 
} tx_config_buf, rx_buf;

uint16_t adj_matrix[NUM_NODES][NUM_NODES] = {0};
uint16_t sensor_packet_number[NUM_NODES] = {0};
uint16_t network_packet_number[NUM_NODES] = {0};
uint8_t neightbor_list[NUM_NODES] = {0};
nrk_time_t sensor_period, network_period;

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
  //create semaphore
  my_sem = nrk_sem_create(1,4);
  if (my_sem == NULL){
	nrk_kprintf(PSTR("Error creating semaphore.\r\n"));
  }

  nrk_start ();

  return 0;
}

#define MAX_TIMEOUTS 3


/*
	Will return data into global array neighbor_list.
	
	The node at index 0 (representing the gateway) will
	contain the ID of the gateway's closest neighbor.
	
	A node graph of G <- 3 <- 1 <- 2 will look like:
	
	neighbor_list[0] = 3
	neighbor_list[1] = 1
	neighbor_list[2] = 2
	neighbor_list[3] = 0;
*/
void find_neighbors(void)
{
	uint8_t max_rssi = 0;
	//look for first node closest to gateway
	for(int x = 1; x < NUM_NODES; x++)
	{
		if(adj_matrix[x][0] > max_rssi)
		{
			neighbor_list[0] = x;
			next_neighbor = x;
			max_rssi = adj_matrix[x][0];
		}
	}
	max_rssi = 0;
	//look for next node
	for(int x = 1; x < NUM_NODES; x++)
	{
		if(adj_matrix[x][next_neighbor] > max_rssi && x != next_neighbor)
		{
			neighbor_list[1] = x;
			max_rssi = adj_matrix[x][next_neighbor];
			next_neighbor = x;
		}
	}
	max_rssi = 0;
	//Look for last node
	for(int x = 1; x < NUM_NODES; x++)
	{
		if(adj_matrix[x][next_neighbor] > max_rssi && x != neighbor_list[0] && x != neighbor_list[1])
		{
			neighbor_list[2] = x;
			max_rssi = adj_matrix[x][next_neighbor];
			next_neighbor = x;
		}
	}
	//make the last node's neighbor set to 0
	neighbor_list[3] = 0;
	return;
}

/* 
	When we receive a task, we check to ake sure that packet numbers match up in order
	to preserve an updated data. Then, we distinguish what type of task it is
	and act accordingly:
	Sensor - 
	Network - 
	Config - Update sensor and network update periods
	
 */
void receive_task(void)
{
  uint8_t i=0;
  uint8_t len =0;
  int8_t rssi = 0;
  nrk_sig_t uart_rx_signal;
  char	option;
  //packet to hold configuration message to be sent
  packet_t *config;
  //time variable to set configuration periods
  nrk_time_t config_period;
  config_period.nano_secs = 0;
  config_period.secs = 5;

  //Initialize config packet to be sent
  config->packet.pkt_type = packet_type_config;
  config->packet.sender = GATE_ID;
  //We make sure to send to the last node in the network
  //Also known as the farthest node from the gateway.
  config->packet.receiver = neighbor_list[LAST_NODE];
  config->packet.config_packet.pkt_num = 1;
  config->packet.config_packet.period = config_period;

  printf("Receive Task PID = %d\r\n", nrk_get_pid());
  bmac_init(17);
  bmac_rx_pkt_set_buffer((uint8_t *)rx_buf.buffer, sizeof(rx_buf));
  
  nrk_kprintf(PSTR("Press 'r' to refresh information, press 's' to set sensor period, and press 'n' to set network period.\r\n" ));
  
  while(1){
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
				if (rval== NRK_ERROR) nrk_kprintf(PSR("Error pending\r\n"));
				adj_matrix[NODE_ID][local->forwarder] = rssi; 
				rval = nrk_sem_post(my_sem);
				if (rval== NRK_ERROR) nrk_kprintf(PSR("Error posting\r\n"));
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
				if (rval== NRK_ERROR) nrk_kprintf(PSR("Error pending\r\n"));
				adj_matrix[NODE_ID][local->forwarder] = rssi; 
				rval = nrk_sem_post(my_sem);
				if (rval== NRK_ERROR) nrk_kprintf(PSR("Error posting\r\n"));
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
					if (rval== NRK_ERROR) nrk_kprintf(PSR("Error pending\r\n"));
					sensor_period = local->packet.config_packet.period;
					rval = nrk_sem_post(my_sem);
					if (rval== NRK_ERROR) nrk_kprintf(PSR("Error posting\r\n"));
				}
				else if(local->packet.config_packet.cfg_type==config_type_network){
				// using semaphore
					rval = nrk_sem_pend(my_sem);
					if (rval== NRK_ERROR) nrk_kprintf(PSR("Error pending\r\n"));
					network_period = local->packet.config_packet.period;
					rval = nrk_sem_post(my_sem);
					if (rval== NRK_ERROR) nrk_kprintf(PSR("Error posting\r\n"));
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
	  
	  if (route==1){//broadcast the packet
	  	val = bmac_tx_pkt((uint8_t*)&local, sizeof(local));
		if (val==NRK_OK){
			nrk_kprintf(PSTR("Packet Transfered Success.\r\n"));}
		else{
			nrk_kprintf(PSTR("Packet Transfered Failed.\r\n"));}
	  }
	  //Check Keyboard for inputs
	  
	  if(nrk_uart_data_ready(NRK_DEFAULT_UART))
		option=getchar();
		int input_val;
		
	  switch(option){
		case 's': nrk_kprintf(PSTR("Please input number (in seconds) and press enter.\r\n" ));
			config->packet.config_packet.cfg_type = config_type_sensor;
			do{
			if(nrk_uart_data_ready(NRK_DEFAULT_UART))
			  option = getchar();
			  input_val *= 10;
			  input_val += atoi(option);
			else nrk_event_wait(SIG(uart_rx_signal));
			  } while(option!='\n');
			  
			if(input_val > 0 && input_val < 30){
			  config_period.secs = input_val;
  		      config.packet.config_packet.period = config_period;
			}
		    break;
		case 'n': nrk_kprintf(PSTR("Please input number (in seconds) and press enter.\r\n" ));
			local->packet.config_packet.cfg_type = config_type_network;
			do{
			if(nrk_uart_data_ready(NRK_DEFAULT_UART))
			  input_val *= 10;
			  input_val += atoi(getchar());
			else nrk_event_wait(SIG(uart_rx_signal));
			  } while(option!='\n');
			  
			if(input_val > 0 && input_val < 30){
			  config_period.secs = input_val;
			  config.packet.config_packet.period = config_period;
			  }
			break;
		case'r': nrk_kprintf(PSTR("Most recent network structure (lower numbers mean node is farther away from the gateway):\r\n" ));
			find_neighbors();
			for(int x = 0; x < NUM_NODES; x++){
			nrk_kprintf("%d. Node %d \r\n", x, neighbor_list[x]);
			}
			break;
		default: nrk_kprintf(PSTR("Command not reconized!\r\n" ));
			break;
	  }
	  
	}
  }
}

void nrk_create_taskset ()
{
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
