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
#define NODE_ID    0x0003

void nrk_create_taskset ();
void nrk_register_drivers();

nrk_task_type SEND_SENSOR_TASK;
nrk_task_type SEND_NETWORK_TASK;
nrk_task_type RECEIVE_TASK;

NRK_STK send_sensor_stack[NRK_APP_STACKSIZE];
NRK_STK send_network_stack[NRK_APP_STACKSIZE];
NRK_STK receive_stack[NRK_APP_STACKSIZE];
nrk_sem_t *my_sem;


union {
  uint8_t  buffer[RF_MAX_PAYLOAD_SIZE];
  packet_t packet; 
} tx_sensor_buf, tx_network_buf, rx_buf;

uint16_t adj_matrix[NUM_NODES][NUM_NODES] = {0};
uint16_t sensor_packet_number[NUM_NODES] = {0};
uint16_t network_packet_number[NUM_NODES] = {0};
uint16_t config_packet_number[NUM_NODES] = {0};
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


void send_sensor_task(void)
{
  //tx_sensor_buf.packet.sensor_packet.pkt_num = 0;
}

void send_network_task(void)
{
}

/*
	Will return data into global array neighbor_list.
	
	The node at index 0 (representing the gateway) will
	contain the ID of the gateway's closest neighbor.
	
	A node graph of G <- 3 <- 1 <- 2 will look like:
	
	neighbor_list[0] = 3
	neighbor_list[3] = 1
	neighbor_list[1] = 2
	neighbor_list[2] = 0;
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
		if(adj_matrix[x][first] > max_rssi && x != first)
		{
			neighbor_list[next_neighbor] = x;
			next_neighbor = x;
			max_rssi = adj_matrix[x][first];
		}
	}
	max_rssi = 0;
	//Look for last node
	for(int x = 1; x < NUM_NODES; x++)
	{
		if(adj_matrix[x][second] > max_rssi && x != first && x != second)
		{
			neighbor_list[next_neighbor] = x;
			next_neighbor = x;
			max_rssi = adj_matrix[x][third];
		}
	}
	//make the last node's neighbor set to 0
	neighbor_list[next_neighbor] = 0;
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
			do{
			if(nrk_uart_data_ready(NRK_DEFAULT_UART))
			  input_val *= 10;
			  input_val += atoi(getchar());
			else nrk_event_wait(SIG(uart_rx_signal));
			  } while(option!='\n');
			sned_config_task(input_val);
			break;
		case 'n': nrk_kprintf(PSTR("Please input number (in seconds) and press enter.\r\n" ));
			do{
			if(nrk_uart_data_ready(NRK_DEFAULT_UART))
			  input_val *= 10;
			  input_val += atoi(getchar());
			else nrk_event_wait(SIG(uart_rx_signal));
			  } while(option!='\n');
			break;
		case'r':
			break;
		default: nrk_kprintf(PSTR("Command not reconized!\r\n" ));
			break;
	  }
	  
	}
	//When not receiving packets, look for keyboard commands
  }
}

#if 0
void whacky_task(void)
{
  uint8_t    i, len, fd;
  int8_t     rssi, val;
  packet_t  *local_buf;
  uint16_t   light;
  uint32_t   master_time, response_time;
  nrk_time_t start_time, end_time;

  //
  // Initialize the constant fields for tx_buf
  // 
  tx_buf.packet.sender        = MAC_ADDR;
  tx_buf.packet.receiver      = MASTER_ID;   
  
  printf ("whacky_task PID=%d\r\n", nrk_get_pid ());
  
  // Open ADC device as read 
  //fd=nrk_open(FIREFLY_SENSOR_BASIC,READ);
  if(fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open sensor driver\r\n"));
  
  val = nrk_set_status(fd, SENSOR_SELECT, LIGHT);

  // init bmac on channel 17
  bmac_init(17);
  
  // This sets the next RX buffer.
  // This can be called at anytime before releasing the packet
  // if you wish to do a zero-copy buffer switch
  bmac_rx_pkt_set_buffer((uint8_t*)&rx_buf, sizeof(rx_buf));

  while (1) {
    DBG_KPRINTF("Waiting for a Packet\r\n");

    // Get the RX packet 
    nrk_led_set (ORANGE_LED);

    val = bmac_wait_until_rx_pkt ();
    if (val != NRK_OK) {
      DBG_KPRINTF("Could not receive packet\r\n");
      continue;
    }

    local_buf = (packet_t*)bmac_rx_pkt_get(&len, &rssi);
    
    DBG_PRINTF("Got RX packet len=%d RSSI=%d\r\n", len, rssi);

    DBG_PRINTF("sender: %d, recipient: %d, round: %d, type: %d, response: %d\r\n",
	       local_buf->sender,
	       local_buf->receiver,
	       local_buf->round,
	       local_buf->msg_type,
	       local_buf->response_time
	       );

    nrk_led_clr (ORANGE_LED);

    if (len != PACKET_LEN) {
      DBG_PRINTF("packet of length %d\r\n", len);
      bmac_rx_pkt_release ();
      continue;
    }
    
    if (local_buf->receiver != MAC_ADDR) {
      DBG_PRINTF("intended recipient is %d\r\n", local_buf->receiver);
      bmac_rx_pkt_release ();
      continue;
    }
    
    if (local_buf->sender != MASTER_ID) {
      DBG_PRINTF("sender is %d\r\n", local_buf->sender);
      bmac_rx_pkt_release ();
      continue;
    }
    
    //
    // Old packet
    //
    if (local_buf->round <= last_round) {
      DBG_PRINTF("stale packet: %d\n", local_buf->round);
      bmac_rx_pkt_release ();
      continue;
    }

    master_time = local_buf->response_time;
    last_round  = local_buf->round;
    
    tx_buf.packet.round    = last_round;
    tx_buf.packet.msg_type = packet_type_ack;

    DBG_PRINTF("sending\nsender: %d, recipient: %d, round: %d, type: %d, response: %d\r\n",
	       tx_buf.packet.sender,
	       tx_buf.packet.receiver,
	       tx_buf.packet.round,
	       tx_buf.packet.msg_type,
	       tx_buf.packet.response_time
	       );
      val = bmac_tx_pkt((uint8_t*)&tx_buf, PACKET_LEN);
      if (val != NRK_OK) {
	DBG_KPRINTF("Could not Transmit!\r\n");
	continue;
      }
      // Task gets control again after TX complete
      nrk_kprintf (PSTR ("Tx task sent data!\r\n"));
    nrk_led_set (RED_LED);
    
    //Start time when light turns on.
    nrk_time_get(&start_time);
    
    while (1) {
      //Checking our response time limit.
      nrk_time_get(&end_time);
      response_time = ((end_time.secs-start_time.secs)*1000+
		       (end_time.nano_secs-start_time.nano_secs)/1000000);
      //if we've reached our limit, then exit the loop.
      if(response_time >= 10000) break;
      
      //Checking light value for hit node.
      val = nrk_read(fd,  (uint8_t*)&light, sizeof(light));

      if(light >= WHACK_THRESHOLD) {
	DBG_PRINTF("Light: %u\r\n", light);	
	break;
      }

      //Checking to see if the master may not have gotten our ACK
      while (bmac_rx_pkt_ready()) {
	//Store the packet and check to see if its for the node
	local_buf = (packet_t*)bmac_rx_pkt_get(&len, &rssi);
	
	if (local_buf->receiver == MAC_ADDR &&
	    local_buf->sender   == MASTER_ID) {
	  //At this point, packet is set to be ACK still
	  val = bmac_tx_pkt((uint8_t*)&tx_buf, PACKET_LEN);
	  if (val != NRK_OK) {
	    DBG_KPRINTF("Could not Transmit!\r\n");
	  }
	}

	bmac_rx_pkt_release ();
      }
    }    
    
    nrk_led_clr (RED_LED);
    
    tx_buf.packet.msg_type      = packet_type_done;
    
    if (light >= WHACK_THRESHOLD) {
      tx_buf.packet.response_time = 1;
    } else {
      tx_buf.packet.response_time = 0;
    }
    
    nrk_time_t wait_time;
    uint8_t    acked = 0;

    wait_time.secs      = WAIT_TIME_S;
    wait_time.nano_secs = WAIT_TIME_NS;
    
    for (i = 0; i < MAX_TIMEOUTS && !acked; i++) {
      val = bmac_tx_pkt((uint8_t*)&tx_buf, PACKET_LEN);
      if (val != NRK_OK) {
	DBG_KPRINTF("can't send packet\r\n");
      }
      
      val = nrk_wait(wait_time);
      if (val != NRK_OK) {
	DBG_KPRINTF("Wait failed?!\r\n");
	continue;
      }
      
      while (bmac_rx_pkt_ready()) {
	local_buf = (packet_t*)bmac_rx_pkt_get (&len, &rssi);

	if (len != PACKET_LEN) {
	  DBG_KPRINTF("length mismatch, who sent this packet?!\r\n");
	  bmac_rx_pkt_release ();
	  continue;
	}
	
	if (local_buf->sender != MASTER_ID) {
	  DBG_KPRINTF("Dropping slave packet\r\n");
	  bmac_rx_pkt_release ();
	  continue;
	}
	
	if (local_buf->receiver != MAC_ADDR) {
	  DBG_KPRINTF("Dropping, not intended recipient\r\n");
	  bmac_rx_pkt_release ();
	  continue;
	}
	
	if (local_buf->round < last_round) {
	  DBG_KPRINTF("Dropping stale packet\r\n");
	  bmac_rx_pkt_release ();
	  continue;
	}

	if (local_buf->msg_type != packet_type_ack) {
	  DBG_PRINTF("Master did not ack: %d\r\n", local_buf->msg_type);
	  bmac_rx_pkt_release ();
	  continue;
	}
	
	DBG_KPRINTF("ACK RECEIVED\r\n");

	acked = 1;
	break;
      }
    } 
  }
}
#endif

void nrk_create_taskset ()
{
  // TODO change naming and values

  SEND_SENSOR_TASK.task = send_sensor_task;
  nrk_task_set_stk( &WHACKY_TASK, whacky_task_stack, NRK_APP_STACKSIZE);
  WHACKY_TASK.prio = 2;
  WHACKY_TASK.FirstActivation = TRUE;
  WHACKY_TASK.Type = BASIC_TASK;
  WHACKY_TASK.SchType = PREEMPTIVE;
  WHACKY_TASK.period.secs = 10;
  WHACKY_TASK.period.nano_secs = 0;
  WHACKY_TASK.cpu_reserve.secs = 5;
  WHACKY_TASK.cpu_reserve.nano_secs = 0;
  WHACKY_TASK.offset.secs = 0;
  WHACKY_TASK.offset.nano_secs = 0;


  WHACKY_TASK.task = whacky_task;
  nrk_task_set_stk( &WHACKY_TASK, whacky_task_stack, NRK_APP_STACKSIZE);
  WHACKY_TASK.prio = 2;
  WHACKY_TASK.FirstActivation = TRUE;
  WHACKY_TASK.Type = BASIC_TASK;
  WHACKY_TASK.SchType = PREEMPTIVE;
  WHACKY_TASK.period.secs = 10;
  WHACKY_TASK.period.nano_secs = 0;
  WHACKY_TASK.cpu_reserve.secs = 5;
  WHACKY_TASK.cpu_reserve.nano_secs = 0;
  WHACKY_TASK.offset.secs = 0;
  WHACKY_TASK.offset.nano_secs = 0;

  WHACKY_TASK.task = whacky_task;
  nrk_task_set_stk( &WHACKY_TASK, whacky_task_stack, NRK_APP_STACKSIZE);
  WHACKY_TASK.prio = 2;
  WHACKY_TASK.FirstActivation = TRUE;
  WHACKY_TASK.Type = BASIC_TASK;
  WHACKY_TASK.SchType = PREEMPTIVE;
  WHACKY_TASK.period.secs = 10;
  WHACKY_TASK.period.nano_secs = 0;
  WHACKY_TASK.cpu_reserve.secs = 5;
  WHACKY_TASK.cpu_reserve.nano_secs = 0;
  WHACKY_TASK.offset.secs = 0;
  WHACKY_TASK.offset.nano_secs = 0;

  nrk_activate_task (&WHACKY_TASK);

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
