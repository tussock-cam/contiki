#ifndef CONTIKI_LORA_H
#define CONTIKI_LORA_H

#include <stdint.h>
#include <stdbool.h>

void lora_set_auth_details(uint32_t addr, const uint8_t* network_key, const uint8_t* app_key);
bool lora_send(const uint8_t* data, uint8_t len);

#endif
