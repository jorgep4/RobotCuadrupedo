#include "wififunctions.hpp"
static EventGroupHandle_t wifi_event_group;
int intentos=0;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


esp_err_t connect_wifi(){
    int estado = WIFI_FAILURE;

	// Inicializa interfaz wifi del esp32
	ESP_ERROR_CHECK(esp_netif_init());

	//Inicia el bucle por defecto en el esp32
	ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Crea STA
    esp_netif_create_default_wifi_sta();

    //Configuración de la conexión wifi
    wifi_init_config_t config= WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    
    //Grupo tareas wifi
    wifi_event_group=xEventGroupCreate();

    // Instancia a Controlador del WiFi (Obtener datos)
    esp_event_handler_instance_t wifi_handler_event_instance;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &wifi_handler_event_instance));
          
    // Instancia a controlador de gestion de IPs                                                  // Instancia a Controlador del WiFi (Obtener datos)
    esp_event_handler_instance_t got_ip_event_instance;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &ip_event_handler,
                                                        NULL,
                                                        &got_ip_event_instance));
    //Parametrod WIfi
    wifi_config_t configuracion_wifi={
        .sta={
            .ssid="MOVISTAR_1DB8", //MOVISTAR_1DB8 "TP-Link_171A"
            .password="m9Pk4kbcb7cJeAWigngg",  //m9Pk4kbcb7cJeAWigngg "24633925"
            .threshold={
                .authmode=WIFI_AUTH_WPA2_PSK
            },
            .pmf_cfg={
                .capable =true,
                .required=false,
            },
        },
    };
    //Modo STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    //Configuración
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&configuracion_wifi));
        ESP_LOGI("WIFI","CONEXION OK");

    //Empezar Wifi
    ESP_ERROR_CHECK(esp_wifi_start());



    //Esperar respuesta
    EventBits_t wifi_response= xEventGroupWaitBits(wifi_event_group,
        WIFI_SUCCESS | WIFI_FAILURE,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY  );

    if (wifi_response & WIFI_SUCCESS){
        ESP_LOGI("WIFI","CONEXION OK");
        estado=WIFI_SUCCESS;
    }
    else if(wifi_response & WIFI_FAILURE){
        ESP_LOGI("WIFI","CONEXION NOK");
        estado=WIFI_FAILURE;
    }
    else{
        ESP_LOGI("WIFI","ERROR INESPERADO");
        estado=WIFI_FAILURE;
    }

    //Liberar recursos
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler_event_instance));
          
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT,IP_EVENT_STA_GOT_IP,got_ip_event_instance));
                                                        
    vEventGroupDelete(wifi_event_group);
    return estado;

}


void wifi_event_handler(void* arg,esp_event_base_t event_base, int32_t event_id, void* event_data){
    if(event_base ==WIFI_EVENT && event_id ==WIFI_EVENT_STA_START){
        ESP_LOGI("WIFI","Conectando");
        esp_wifi_connect();
    }
    else if(event_base ==WIFI_EVENT && event_id ==WIFI_EVENT_STA_DISCONNECTED){
        if(intentos<MAX_FAILURES){
            ESP_LOGI("WIFI","REINTENTAR");
            esp_wifi_connect();
            intentos++;
        }
        else{
            xEventGroupSetBits(wifi_event_group,WIFI_FAILURE);
        }
    }
// else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
//         ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
//         intentos = 0;
//         xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
//     }
}

void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if(event_base ==IP_EVENT && event_id ==IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t* event=( ip_event_got_ip_t*)event_data;
        ESP_LOGI("WIFI","IP"IPSTR, IP2STR(&event->ip_info.ip));
        intentos=0;
        xEventGroupSetBits(wifi_event_group,WIFI_SUCCESS);
    }
}

bool is_wifi_connected(){
    wifi_ap_record_t wifi_info;
    if(esp_wifi_sta_get_ap_info(&wifi_info)==ESP_OK){
        return true;
    }
    else{
        return false;
    }
}