#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

/** DEFINES **/
#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1
#define MAX_FAILURES 10

bool is_wifi_connected();
void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_event_handler(void* arg,esp_event_base_t event_base, int32_t event_id, void* event_data);
esp_err_t connect_wifi();