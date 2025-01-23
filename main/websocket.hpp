#ifndef WEBSOCKET_HPP
#define WEBSOCKET_HPP
#include "esp_websocket_client.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "cJSON.h"
// #include "globals.hpp"
extern float incline[3];//Inclinaciones
extern float coordinates[3];//Coordenadas
extern bool spin;//Girar?
extern int gait;//Tipo de paso

// #include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_websocket_client.h"
#include "esp_event.h"
#include "camfunctions.hpp"

extern float incline[3];//Inclinaciones
extern float coordinates[3];//Coordenadas
extern bool spin;//Girar?
extern int gait;//Tipo de paso
void websocket_start();
static void shutdown_signaler(TimerHandle_t xTimer);
void websocket_handler(void *handler_args, esp_event_base_t base,int32_t event_id, void *event_data);
#endif;