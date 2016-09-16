#include "lora.h"
#include "dev/rfm9x/lmic/lmic.h"

void lora_set_auth_details(uint32_t addr, const uint8_t* network_key, const uint8_t* app_key)
{
	LMIC_setSession(1, addr, network_key, app_key);
}

bool lora_send(const uint8_t* data, uint8_t len)
{
	return LMIC_setTxData2(1, data, len, 1) == 0;
}
