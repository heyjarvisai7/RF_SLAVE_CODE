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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);

static nrf_esb_payload_t        rx_payload;
uint8_t receieved = 0;

#define MAX_CIRCLE    4


#define PACKET_HEADER_SIZE                 sizeof(Packet_Header)

struct __attribute__((packed)) Packet_Header
{
uint8_t circle_array[MAX_CIRCLE];
uint16_t length;
uint8_t zone;
uint8_t reserved; 
};

struct Packet_Header Packet_Header;


void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
            receieved++;
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_DEBUG("TX FAILED EVENT");
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
           // NRF_LOG_INFO("RX RECEIVED EVENT");
           //NRF_LOG_INFO("Receiving packet: %02x", rx_payload.data[1]);
           //NRF_LOG_FLUSH();
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                 //NRF_LOG_INFO("RX RECEIVED PAYLOAD");
                 //NRF_LOG_FLUSH();

                if (rx_payload.length > 0)
                {

            //                (void) nrf_esb_flush_tx();
            //(void) nrf_esb_start_tx();
                  
                    //NRF_LOG_INFO("RX RECEIVED PAYLOAD");
                     NRF_LOG_INFO("Receiving packet: %02x", rx_payload.data[1]);
                    // NRF_LOG_FLUSH();
                }
            }
            //NRF_LOG_INFO("Receiving packet: %02x", rx_payload.data[1]);
            //NRF_LOG_FLUSH();
            break;
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
    nrf_gpio_range_cfg_output(8, 15);
    bsp_board_init(BSP_INIT_LEDS);
}



uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0x0A, 0x0A, 0x0A, 0x0A};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7};

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 1);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

uint8_t  open_request[20] = {0xE7,0xE6, 0x00,0x00,0x00,0x09,0x00,0x00,0x7E, 0xA0, 0x07, 0x03,0x21, 0x93, 0x0F, 0x01, 0x7E};
//uint8_t  open_request1[20] = {};

uint8_t  open_request1[20] = {0x7E, 0xA0, 0x07, 0x03,0x21, 0x93, 0x0F, 0x01, 0x7E};
uint8_t  addr[4] = {0xE7,0xE6, 0x00,0x00};

int main(void)
         {
    ret_code_t err_code;

    struct Packet_Header hdr;
    uint32_t nextBytes = 0;

    gpio_init();

    err_code = NRF_LOG_INIT(NULL);  
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    clocks_start();

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("Enhanced ShockBurst Transmitter Example started.");
    

    while (true)
    {
      if(receieved == 0)
      {
          receieved =1;
      
         // memcpy(tx_payload.data , open_request,17);// strlen(open_request));
#if 1
         memcpy(hdr.circle_array, addr, MAX_CIRCLE);
         hdr.length     = 0x09;
         hdr.zone       = 0;
         hdr.reserved   = 0;
       
         memcpy(tx_payload.data, &hdr, sizeof(hdr));
         nextBytes += sizeof(hdr);
         memcpy(tx_payload.data + nextBytes, open_request1, 0x09);
         nextBytes += hdr.length;
          
          tx_payload.length = nextBytes;//strlen(open_request);
#else
          memcpy(tx_payload.data, open_request, 17);
          tx_payload.length = 17;
#endif

            for(int i =0;i < 1 ;i++)
        {
            NRF_LOG_DEBUG("Transmitting packet %02x", tx_payload.data[1]);

            tx_payload.noack = false;

                if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                {
                    // Toggle one of the LEDs.
                    nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
                    nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
                    nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
                    nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
                    //tx_payload.data[1]++;
                }
                

                else
                {
                    NRF_LOG_WARNING("Sending packet failed");
                }
       

            nrf_delay_us(50000);
            
            NRF_LOG_FLUSH();
        }
    }
    #if 0
          if(receieved == 2)
      {
          receieved =0;
          memcpy(tx_payload.data , open_request1, strlen(open_request1));
          
          tx_payload.length = strlen(open_request1);

            for(int i =0;i < 1 ;i++)
        {
            NRF_LOG_DEBUG("Transmitting packet %02x", tx_payload.data[1]);

            tx_payload.noack = false;

                if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                {
                    // Toggle one of the LEDs.
                    nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
                    nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
                    nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
                    nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
                    //tx_payload.data[1]++;
                }
                

                else
                {
                    NRF_LOG_WARNING("Sending packet failed");
                }
       

            nrf_delay_us(50000);
            
            NRF_LOG_FLUSH();
                //APP_ERROR_CHECK(err_code);
                                 err_code = nrf_esb_start_rx();
        }
    }
#endif
      //                nrf_esb_flush_tx();
      //nrf_esb_flush_rx();
    //err_code = esb_init();

         //APP_ERROR_CHECK(err_code);
         }
    }



         
         //while(1)
         //{
         //}
//    }
//}
