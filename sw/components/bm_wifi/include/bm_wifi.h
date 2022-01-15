#ifndef BLINKMOJT_WIFI_H
#define BLINKMOJT_WIFI_H

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"

#define BM_WIFI_SSID CONFIG_BM_WIFI_SSID
#define BM_WIFI_PASS CONFIG_BM_WIFI_PASSWORD

extern EventGroupHandle_t wifi_event_group;
extern const int CONNECTED_BIT;
extern const int WIFI_FAIL_BIT;

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
void wifi_init(void);

#endif
