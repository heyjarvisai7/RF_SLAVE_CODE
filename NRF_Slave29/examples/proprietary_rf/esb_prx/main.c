/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "nrf_esb.h"

#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

void pingPacket(void);
uint32_t esb_init1( void );
uint32_t esb_init( void );
uint32_t set_slave_adress(uint8_t slaveid,uint8_t *base_address);
void scan_device(uint8_t length);



uint8_t led_nr;
uint8_t Fwrd_Dirct = 0;
uint8_t Rvsr_Dirct = 0;

nrf_esb_payload_t rx_payload;

static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */
#define PACKET_HEADER_SIZE                 sizeof(Packet_Header)
#define START_BYTE                         PACKET_HEADER_SIZE

#define ARRAY_SIZE 4
#define MAX_CIRCLE 4
#define MAX_NEIGHBORS 10
#define MIN_CIRCLE 0

#define     FORWARD              1
#define     BACKWORD             0

#define     POS_PACKET_TYPE     0
#define     POS_DIRECTION       sizeof(header.packet_type)
#define     POS_LENGTH          POS_DIRECTION + sizeof(header.Direction) 
#define     POS_CIRCLE_ARRAY    POS_LENGTH + sizeof(header.length)
#define     POS_DIRTY_FLAG      POS_CIRCLE_ARRAY + sizeof(header.circle_array)

#define     PING_PACKET         1
#define     DATA_PACKET         0

#define     MAX_NODES            255
#define     MIN_NODES            0


#define check_direction(arrptr) \
    (((arrptr)[POS_DIRECTION]  == 1) ? (Fwrd_Dirct = 1) : (Rvsr_Dirct = 1))



uint8_t arrays[MAX_CIRCLE][ARRAY_SIZE] =
{
    {0x0A, 0x0A, 0x0A, 0x0A},   // ARRAY_1
    {0x0B, 0x0B, 0x0B, 0x0B},   // ARRAY_2
    {0x0C, 0x0C, 0x0C, 0x0C},   // ARRAY_3
    {0x0D, 0x0D, 0x0D, 0x0D}    // ARRAY_4
};




uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
uint8_t base_addr_1[4] = {0xC1, 0xC1, 0xC1, 0xC1};
uint8_t addr_prefix[8] = {0x77};


uint8_t received_data = 0;


typedef struct
{
    uint16_t node_id;       // Neighbor ID
    int8_t   rssi;          // Signal strength
    //uint8_t  hop_count;     // Distance to sink
    //uint32_t last_seen;     // Timestamp
    //uint8_t  valid;         // Entry active
    uint8_t dirtyflag;
} neighbor_t;
 
neighbor_t Nxt_neighbor_table[MAX_NEIGHBORS];
neighbor_t Prev_neighbor_table[MAX_NEIGHBORS];


typedef struct
{
    uint8_t node_id;
    uint8_t circle_no;
    //uint8_t  hop_count;
    //uint8_t  seq;
} adv_packet_t;

adv_packet_t advertisment_pcket;
#if 0
struct __attribute__((packed)) Packet_Header
{
uint8_t circle_array[MAX_CIRCLE];
uint16_t length;
uint8_t Direction;
uint8_t reserved;
uint8_t packet_type; 
};
#else


struct __attribute__((packed)) Packet_Header
{
      uint8_t packet_type; 
      uint8_t Direction;
      uint16_t length;
      uint8_t circle_array[MAX_CIRCLE];
      uint8_t dirtyflag;
      uint8_t reserved;
};
#endif


struct Packet_Header Packet_Header;
uint8_t selected_pipe = 0;
uint8_t volatile Mater_Data_received = 0;
uint8_t volatile Mater_Data_Sent = 0;
uint8_t volatile Slave_Data_received = 0;
uint8_t volatile Slave_Data_Sent = 0;
uint8_t itr ;
uint16_t length = 0;
uint8_t i = 0;
uint8_t sent_bytes_count = 0;
nrf_esb_payload_t rx_payload;
uint8_t Circle_No = 0;
uint8_t Current_Circle = 2;
uint8_t Send_Data_To_Nxt_Slave = 0;

uint8_t uartbuff[2048];
uint8_t Ack_Txing = 0;
uint8_t Actd[15];
uint8_t *circlearr;
uint8_t Nxt_table_index;
uint8_t Prev_table_index;
uint8_t neighbour_no;


struct Packet_Header header;


void select_slave(uint8_t slave_id)
{
    // slave_id must match the pipe number configured on the slave
    selected_pipe = slave_id;
}

void sort_neighbors_by_rssi(neighbor_t *table, int n)
{
    for (int i = 1; i < n; i++)
    {
        neighbor_t key = table[i];
        int j = i - 1;

        while (j >= 0 && table[j].rssi < key.rssi)
        {
            table[j + 1] = table[j];
            j--;
        }
        table[j + 1] = key;
    }
}


void Construct_DLMS_Packet(void)
{
   
  if(circlearr[Current_Circle] == addr_prefix[0])
  {
   memcpy(&header, rx_payload.data,sizeof(header));
  if(rx_payload.data[START_BYTE] == 0x7E)
    {
            if((rx_payload.data[START_BYTE + 2]))
            {
                    length = (((rx_payload.data[START_BYTE +1] & 0x07) << 8) | rx_payload.data[START_BYTE +2]);
                    length += 2;
            }
            i = 0;

    }
    if(length  > NRF_ESB_MAX_PAYLOAD_LENGTH - START_BYTE)
    {
      //received = 0;

      memcpy(uartbuff+sent_bytes_count,rx_payload.data + START_BYTE,rx_payload.length - START_BYTE);
     // tx_payload.length = rx_payload.length;
      sent_bytes_count += NRF_ESB_MAX_PAYLOAD_LENGTH - START_BYTE;
      length -= NRF_ESB_MAX_PAYLOAD_LENGTH -START_BYTE;
    }
    else
    {
      //memset(tx_payload.data,'/0',strlen(tx_payload.data));
     // memcpy(tx_payload.data+sent_bytes_count,rx_payload.data,rx_payload.length);
      memcpy(uartbuff+sent_bytes_count,rx_payload.data + START_BYTE,rx_payload.length - START_BYTE);
    //  tx_payload.length = rx_payload.length;
      sent_bytes_count += rx_payload.length - START_BYTE;
      if(uartbuff[sent_bytes_count-1] == 0x7E)
      {
           if(header.length == sent_bytes_count)
           {
               sent_bytes_count = 0;
               header.Direction = BACKWORD;// reverse direction;
              

               //send data to uart
           }
      }   
      else
      {
        
         Mater_Data_received = 0;//when haft data come
      }
      
    }
  }
                    
  NRF_LOG_FLUSH();
}
void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
             Ack_Txing = 1;
             NRF_LOG_FLUSH();
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
             neighbour_no++;
             NRF_LOG_FLUSH();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
                      {
                          if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
                          {
                             circlearr = rx_payload.data;
                             if(circlearr[POS_PACKET_TYPE] == 0)
                             {
                                  if(circlearr[(POS_CIRCLE_ARRAY+ Current_Circle ) + 1] != 0)
                                 {
                                     check_direction(circlearr);
                                 }
                                 else
                                 {
                                    Construct_DLMS_Packet();
                                 }
                             }
                             pingPacket();
                          }  
                      }break;
           
#if 0      
            if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {


            #if 0
                //for(uint8_t i = 0;i < MAX_CIRCLE ; i++)
                //{
                
                    if(rx_payload.data[Current_Circle + 1] != 0) //&& rx_payload.data[Current_Circle] == addr_prefix[0])
                    {
                        nrf_esb_stop_rx();
                        memcpy(tx_payload.data+sent_bytes_count,rx_payload.data,rx_payload.length);
                        tx_payload.length = rx_payload.length;
                        tx_payload.noack = false;
                        Send_Data_To_Nxt_Slave = 1;
                        set_slave_adress(rx_payload.data[Current_Circle + 1],arrays[Current_Circle + 1]);
                    }
                //}
 
            }
            #endif
                // Example Debug print]
#if 1
                Circle_No = 0;
                int8_t i,j =0;
                
                memcpy(&header, rx_payload.data, PACKET_HEADER_SIZE); // header copying from rx_payload.data

     
                for( i = 0;i < MAX_CIRCLE; i++)
                {
#if 0 // DD
                    if(rx_payload.data[i] == 0)
                    {
                      if(i != 0)
                      {
                         Circle_No = i - 1;
                      }
                      break;
                    }
#else 
                    if(header.circle_array[i] == 0)
                    {
                      if(i != 0)
                      {
                         Circle_No = i - 1;
                      }
                      break;
                    }
#endif
                    else
                    {
                      Circle_No = MAX_CIRCLE;
                    }
                }

                if(Circle_No > Current_Circle)
                {
                     nrf_esb_stop_rx();
                     memcpy(tx_payload.data,rx_payload.data,rx_payload.length);
                     tx_payload.length = rx_payload.length;
                     tx_payload.noack = false;
                     Send_Data_To_Nxt_Slave = 1;
                     #if 0
                     set_slave_adress(rx_payload.data[Current_Circle + 1],arrays[Current_Circle + 1]);
                     #else
                     set_slave_adress(header.circle_array[Current_Circle + 1],arrays[Current_Circle + 1]);
                     #endif
                }
                #if 0
                else if((Circle_No == Current_Circle) && (rx_payload.data[Circle_No] != 0))
                #else
                else if((Circle_No == Current_Circle) && (header.circle_array[Circle_No] != 0))
                {
                   Construct_DLMS_Packet();
                } 
                #endif
                else
                {
                    for(j = Circle_No ; j < MAX_CIRCLE ; j++)
                    {
                    #if 0
                        if(rx_payload.data[j] != 0)
                        {
                            nrf_esb_stop_rx();
                            if(j+1 >= 4)
                            {
                                //send data to dcu       
                            }
                            else
                            {
                               set_slave_adress(rx_payload.data[j+1],arrays[MAX_CIRCLE - j]);
                            }
                            rx_payload.data[j] = 0;
                            memcpy(tx_payload.data+sent_bytes_count,rx_payload.data,rx_payload.length);
                            tx_payload.length = rx_payload.length;
                            tx_payload.noack = false;
                            Send_Data_To_Nxt_Slave = 1;
                        }
                        #else // DD
                        if(header.circle_array[j] != 0)
                        {
                            nrf_esb_stop_rx();
                            if(j+1 >= 4)
                            {
                                //send data to dcu  
                                NRF_LOG_INFO("Sending to DCU");
                                NRF_LOG_FLUSH(); 
                            }
                            else
                            {
                            #if 0
                               set_slave_adress(rx_payload.data[j+1],arrays[MAX_CIRCLE - j]);
                               #else
                               set_slave_adress(header.circle_array[j+1],arrays[MAX_CIRCLE - j]);
                               #endif
                            }
                            #if 0
                            rx_payload.data[j] = 0;
                            #else
                            header.circle_array[j] = 0;
                            #endif
                            memcpy(tx_payload.data+sent_bytes_count,rx_payload.data,rx_payload.length);
                            tx_payload.length = rx_payload.length;
                            tx_payload.noack = false;
                            Send_Data_To_Nxt_Slave = 1;
                        }
                        #endif
                    }
                } 
            }
            #endif
#endif
    
        //}break;
    }
}

void pingPacket(void)
{
          if(circlearr[POS_PACKET_TYPE] == 1  && circlearr[POS_LENGTH] == 0)
          {
                memcpy(&advertisment_pcket,rx_payload.data + sizeof(header), sizeof(advertisment_pcket));
                if(rx_payload.rssi > 0 && Nxt_table_index < MAX_NEIGHBORS && advertisment_pcket.circle_no == Current_Circle +1)
                {
                      Nxt_neighbor_table[Nxt_table_index].node_id = advertisment_pcket.node_id;
                      Nxt_neighbor_table[Nxt_table_index].rssi = rx_payload.rssi;
                      Nxt_table_index++;
                }
                if(rx_payload.rssi > 0 && Prev_table_index < MAX_NEIGHBORS && advertisment_pcket.circle_no == Current_Circle - 1)
                {
                      Prev_neighbor_table[Prev_table_index].node_id = advertisment_pcket.node_id;
                      Prev_neighbor_table[Prev_table_index].rssi = rx_payload.rssi;
                      Prev_table_index++;
                }
                neighbour_no++;
                Ack_Txing = 0;

          }
          if(circlearr[POS_PACKET_TYPE] == 1  && circlearr[POS_LENGTH] == 1)
          {
                advertisment_pcket.circle_no = Current_Circle;
                advertisment_pcket.node_id = addr_prefix[0];
                memset(&header,0,sizeof(header));
                header.packet_type = 1;
                if(circlearr[(POS_CIRCLE_ARRAY+ Current_Circle) - 1] != 0)
                {
                       header.Direction = BACKWORD;
                }
                else
                {
                       header.Direction = FORWARD;
                }
                memcpy(tx_payload.data,&header,sizeof(header));
                memcpy(tx_payload.data + sizeof(header), &advertisment_pcket, sizeof(advertisment_pcket));
                tx_payload.length = sizeof(header) + sizeof(advertisment_pcket);
                check_direction(tx_payload.data);
          }
}

void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    bsp_board_init(BSP_INIT_LEDS);
}

void nrf_set_txmode(void)
{
    uint32_t err_code;
     nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.payload_length           = 8;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);
    //VERIFY_SUCCESS(err_code);
}


void scan_device(uint8_t length)
{
    uint8_t i, k = 0, l = 0;
 
     for(i = 0 ;i <= 0xFF ; i++)
     {
          NRF_LOG_DEBUG("Transmitting packet %02x", tx_payload.data[1]);
          NRF_LOG_FLUSH();
 
          tx_payload.noack = false;
          if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
          {
              nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
              nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
              nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
              nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
              tx_payload.data[1]++;
          }
          else
          {
              NRF_LOG_INFO("Sending packet failed");
              NRF_LOG_FLUSH();
          }
 
          nrf_delay_us(50000);

 
          if(Ack_Txing == 1)
          {
              Ack_Txing = 0;
              Actd[k++] = i;
          }
 
         if(k >= length)
         {
            break;
         }

          set_slave_adress(i, arrays[Current_Circle - 1]);
     }
 
     NRF_LOG_INFO("Scan done");
     NRF_LOG_FLUSH();
 
}


uint32_t esb_init( void )
{

    uint32_t err_code;

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;


    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(arrays[Current_Circle]);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 1);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

uint32_t set_slave_adress(uint8_t slaveid,uint8_t *base_address)
{

    uint32_t err_code;

    err_code = nrf_esb_set_base_address_0(base_address);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(&slaveid, 1);
    VERIFY_SUCCESS(err_code);
    nrf_esb_flush_tx();
    nrf_esb_flush_rx();


    return err_code;
}

#define	POW_CNTRL_PIN	7
uint8_t autohealing = 0;
uint8_t Diagnostic_Test = 0;

void doDiagnosticTest(void)
{
     Nxt_table_index = 0;
         Prev_table_index = 0;
         uint8_t done_rx_start = 1;
         if(Current_Circle == MIN_CIRCLE)
         {
             
             for(neighbour_no = MIN_NODES; neighbour_no < MAX_NODES ;)
             {
                
                 if(Ack_Txing == 0)
                 {
                   nrf_esb_stop_rx();
                   set_slave_adress( neighbour_no ,arrays[Current_Circle + 1]);
                   memset(&header,0,sizeof(header));
                   header.packet_type = 1;
                   header.length = 1;
                   header.circle_array[Current_Circle] = addr_prefix[0];
                 
                   memcpy(tx_payload.data,&header,sizeof(header));
                   memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                   tx_payload.length = (sizeof(header) + sizeof("HAI"));
                   done_rx_start = 1;
                    if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                    {
                        nrf_delay_us(50000); //to send the data 
                      //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
                        
                    }
                }
                if(done_rx_start == 1 && Ack_Txing == 1)
                {
                      done_rx_start = 0;
                         nrf_esb_flush_tx();
                        nrf_esb_flush_rx();
                      set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                        nrf_esb_start_rx();
                }

             }
               Diagnostic_Test = 0;

         }
         else if(Current_Circle == MAX_CIRCLE)
         {
             
             for(neighbour_no = MIN_NODES; neighbour_no < MAX_NODES ;)
             {
              if(Ack_Txing == 0)
                 {
                   nrf_esb_stop_rx();
                   set_slave_adress( neighbour_no ,arrays[Current_Circle - 1]);
                   memset(&header,0,sizeof(header));
                   header.packet_type = 1;
                   header.length = 1;
                   header.circle_array[Current_Circle] = addr_prefix[0];
                   done_rx_start = 1;
                   memcpy(tx_payload.data,&header,sizeof(header));
                   memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                   tx_payload.length = (sizeof(header) + sizeof("HAI"));
                    if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                    {
                        nrf_delay_us(50000); //to send the data 
                      //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
                       // nrf_esb_start_rx();
                    }
                }
                if(done_rx_start == 1 && Ack_Txing == 1)
                {
                      done_rx_start = 0;
                         nrf_esb_flush_tx();
                        nrf_esb_flush_rx();
                      set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                        nrf_esb_start_rx();
                }

             }
             Diagnostic_Test = 0;
         }
        else 
         {
             
             for( neighbour_no = MIN_NODES; neighbour_no < MAX_NODES ;)
             {
                    if(Ack_Txing == 0)
                    {
                          nrf_esb_stop_rx();
                          set_slave_adress( neighbour_no ,arrays[Current_Circle - 1]);
                          memset(&header,0,sizeof(header));
                          header.packet_type = 1;
                          header.length = 1;
                          header.circle_array[Current_Circle] = addr_prefix[0];
                          done_rx_start = 1;
                          memcpy(tx_payload.data,&header,sizeof(header));
                          memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                          tx_payload.length = (sizeof(header) + sizeof("HAI"));

                          if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                          {
                              nrf_delay_us(50000); //to send the data 
                              //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
                              //nrf_esb_start_rx();
                          }
                    }

                     if(done_rx_start == 1 && Ack_Txing == 1)
                    {
                        done_rx_start = 0;
                        //nrf_esb_stop_rx();
                           nrf_esb_flush_tx();
                          nrf_esb_flush_rx();
                        set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                        nrf_esb_start_rx();
                    }
                    

             }
            for( neighbour_no = MIN_NODES; neighbour_no < MAX_NODES ;)
             {
                    if(Ack_Txing == 0)
                    {
                          nrf_esb_stop_rx();
                          set_slave_adress( neighbour_no ,arrays[Current_Circle + 1]);
                          memset(&header,0,sizeof(header));
                          header.packet_type = 1;
                          header.length = 1;
                          header.circle_array[Current_Circle] = addr_prefix[0];
                          done_rx_start = 1;
                          memcpy(tx_payload.data,&header,sizeof(header));
                          memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                          tx_payload.length = (sizeof(header) + sizeof("HAI"));
                          if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                          {
                          nrf_delay_us(50000); //to send the data 
                          //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
                          //nrf_esb_start_rx();
                          }
                    }
                    if(done_rx_start == 1 && Ack_Txing == 1)
                    {
                        done_rx_start = 0;
                           nrf_esb_flush_tx();
                          nrf_esb_flush_rx();
                        set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                        nrf_esb_start_rx();
                    }

             }
             Diagnostic_Test = 0;
         }
}

void sendDataToNextSlave(void)
{
          //nrf_delay_us(50000);
          // nrf_esb_stop_rx();
          // nrf_delay_us(50000);
          Send_Data_To_Nxt_Slave = 0; 
          if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
          {
              nrf_delay_us(50000); //to send the data 
              set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
              nrf_esb_start_rx();
          }
          else
          {
              NRF_LOG_INFO("FAILED to send the data");
          }
}

void sendDataBidirectional(void)
{
        if(Fwrd_Dirct == 1)
        {
            Fwrd_Dirct = 0;
            nrf_delay_us(50000);
            nrf_esb_stop_rx();
            if(header.packet_type == 0)
            {
                memcpy(tx_payload.data,rx_payload.data,rx_payload.length);
                tx_payload.length = rx_payload.length;
                
            }
            tx_payload.noack = false;
            Send_Data_To_Nxt_Slave = 1;
            set_slave_adress(rx_payload.data[(POS_CIRCLE_ARRAY+ Current_Circle ) + 1],arrays[Current_Circle + 1]);
        }
        if(Rvsr_Dirct == 1)
        {
            Rvsr_Dirct = 0;
            nrf_delay_us(50000);
            nrf_esb_stop_rx();
            if(header.packet_type == 0)
            {
                memcpy(tx_payload.data,rx_payload.data,rx_payload.length);
                tx_payload.length = rx_payload.length;
                
            }
            tx_payload.noack = false;
            Send_Data_To_Nxt_Slave = 1;

            if (Current_Circle == 0)    
            {
                  // send the response to DCU s

            }
  
            else    set_slave_adress(rx_payload.data[(POS_CIRCLE_ARRAY+ Current_Circle ) - 1],arrays[Current_Circle  - 1]);
        }
}

void main(void)
{
    uint32_t err_code;
    uint8_t check_nerby[PACKET_HEADER_SIZE];

    gpio_init();
    nrf_gpio_cfg_output(POW_CNTRL_PIN);
    nrf_gpio_pin_set(POW_CNTRL_PIN);

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    clocks_start();

     
    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("Enhanced ShockBurst Receiver Example started.");

    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);
  
    //nrf_esb_start_rx();
    while(true)
    {

          sendDataBidirectional();

          if(Send_Data_To_Nxt_Slave == 1)
          {
              sendDataToNextSlave();
          }
          if(Diagnostic_Test == 1)
          {
              Diagnostic_Test = 0;
        
              doDiagnosticTest();
          }

         #if 0
         Nxt_table_index = 0;
         Prev_table_index = 0;
         uint8_t done_rx_start = 1;
         if(Current_Circle == MIN_CIRCLE)
         {
             
             for(neighbour_no = 0; neighbour_no < 255 ;)
             {
                
                 if(Ack_Txing == 0)
                 {
                   nrf_esb_stop_rx();
                   set_slave_adress( neighbour_no ,arrays[Current_Circle + 1]);
                   memset(&header,0,sizeof(header));
                   header.packet_type = 1;
                   header.length = 1;
                   header.circle_array[Current_Circle] = addr_prefix[0];
                   done_rx_start = 1;
                   memcpy(tx_payload.data,&header,sizeof(header));
                   memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                   tx_payload.length = (sizeof(header) + sizeof("HAI"));
                   done_rx_start = 1;
                    if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                    {
                        nrf_delay_us(50000); //to send the data 
                      //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
                        
                    }
                }
                else if(done_rx_start == 1)
                {
                      done_rx_start = 0;
                      //   nrf_esb_flush_tx();
                      //  nrf_esb_flush_rx();
                      set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                        nrf_esb_start_rx();
                }

             }
         }
         else if(Current_Circle == MAX_CIRCLE)
         {
             
             for(neighbour_no = 0; neighbour_no < 255 ;)
             {
              if(Ack_Txing == 0)
                 {
                   nrf_esb_stop_rx();
                   set_slave_adress( neighbour_no ,arrays[Current_Circle - 1]);
                   memset(&header,0,sizeof(header));
                   header.packet_type = 1;
                   header.length = 1;
                   header.circle_array[Current_Circle] = addr_prefix[0];
                   done_rx_start = 1;
                   memcpy(tx_payload.data,&header,sizeof(header));
                   memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                   tx_payload.length = (sizeof(header) + sizeof("HAI"));
                    if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                    {
                        nrf_delay_us(50000); //to send the data 
                      //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
                       // nrf_esb_start_rx();
                    }
                }
                else if(done_rx_start == 1)
                {
                      done_rx_start = 0;
                      //   nrf_esb_flush_tx();
                      //  nrf_esb_flush_rx();
                      set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                        nrf_esb_start_rx();
                }

             }
         }
        else 
         {
             
             for( neighbour_no = 0; neighbour_no < 255 ;)
             {
                    if(Ack_Txing == 0)
                    {
                          nrf_esb_stop_rx();
                          set_slave_adress( neighbour_no ,arrays[Current_Circle - 1]);
                          memset(&header,0,sizeof(header));
                          header.packet_type = 1;
                          header.length = 1;
                          header.circle_array[Current_Circle] = addr_prefix[0];
                          done_rx_start = 1;
                          memcpy(tx_payload.data,&header,sizeof(header));
                          memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                          tx_payload.length = (sizeof(header) + sizeof("HAI"));

                          if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                          {
                              nrf_delay_us(50000); //to send the data 
                              //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
                              //nrf_esb_start_rx();
                          }
                    }

                    else if(done_rx_start == 1)
                    {
                        done_rx_start = 0;
                        //nrf_esb_stop_rx();
                          // nrf_esb_flush_tx();
                          //nrf_esb_flush_rx();
                        set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                        nrf_esb_start_rx();
                    }
                    

             }
            for( neighbour_no = 0; neighbour_no < 255 ;)
             {
                    if(Ack_Txing == 0)
                    {
                          nrf_esb_stop_rx();
                          set_slave_adress( neighbour_no ,arrays[Current_Circle + 1]);
                          memset(&header,0,sizeof(header));
                          header.packet_type = 1;
                          header.length = 1;
                          header.circle_array[Current_Circle] = addr_prefix[0];
                          done_rx_start = 1;
                          memcpy(tx_payload.data,&header,sizeof(header));
                          memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                          tx_payload.length = (sizeof(header) + sizeof("HAI"));
                          if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                          {
                          nrf_delay_us(50000); //to send the data 
                          //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
                          //nrf_esb_start_rx();
                          }
                    }
                    else if(done_rx_start == 1)
                    {
                        done_rx_start = 0;
                           nrf_esb_flush_tx();
                          nrf_esb_flush_rx();
                        set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                        nrf_esb_start_rx();
                    }

             }
         }
      }
      #endif

    
    if (NRF_LOG_PROCESS() == false)
    {
        __WFE();
    }
        
}
}
//}
/*lint -restore */
