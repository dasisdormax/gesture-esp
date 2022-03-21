#ifndef wireless_h
#define wireless_h

#include "common.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
//#include "esp_smartconfig.h"


void wireless_setup();
void wireless_init();
void wireless_reset();
void wireless_send(char *buf);
bool wireless_is_configured();

#endif