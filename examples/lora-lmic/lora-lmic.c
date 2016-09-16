/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/ctimer.h"
#include "apps/lora/lora.h"
#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
PROCESS(lora_app_process, "LoRa client process");
AUTOSTART_PROCESSES(&lora_app_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lora_app_process, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  printf("LoRa client starting...\n");

  static const uint8_t NWKSKEY[16] = { 0x3D, 0xDC, 0x46, 0xA9, 0x7E, 0x70, 0xB6, 0x79, 0x00, 0x71, 0x02, 0x40, 0x9E, 0x2A, 0xCE, 0x2E};

  // LoRaWAN AppSKey, application session key
  static const uint8_t APPSKEY[16] = { 0x49, 0x54, 0x7E, 0x02, 0x3A, 0x4F, 0xFA, 0xFD, 0x22, 0xDF, 0x9E, 0x5E, 0xBA, 0xFC, 0x25, 0xEA};

  // LoRaWAN end-device address (DevAddr)
  static const uint32_t DEVADDR =  0x008EE225; // <-- Change this address for every node

  printf("Configure LoRa details...\n");
  lora_set_auth_details(DEVADDR, NWKSKEY, APPSKEY);

  static uint8_t txdata[] = {0xab};
  static bool result;
  result = lora_send(txdata, sizeof(txdata));

  printf("TX result: %d\n", result);

  static struct etimer timer;
  etimer_set(&timer, CLOCK_SECOND * 10);
  while(1) {
    PROCESS_YIELD();

     result = lora_send(txdata, sizeof(txdata));

     printf("TX result: %d\n", result);

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
