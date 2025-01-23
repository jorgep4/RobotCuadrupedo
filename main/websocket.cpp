#include "websocket.hpp"
float incline[3]={0,0,0};
float coordinates[3]={0,0,-0.15};//Coordenadas
bool spin=0;//Girar?
int gait=0;//Tipo de paso

static TimerHandle_t shutdown_signal_timer;
static SemaphoreHandle_t shutdown_sema;
void websocket_handler(void *handler_args, esp_event_base_t base,int32_t event_id, void *event_data){
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI("WS", "WEBSOCKET_EVENT_CONNECTED");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI("WS", "WEBSOCKET_EVENT_DISCONNECTED");
        break;
    case WEBSOCKET_EVENT_DATA:{
        ESP_LOGI("WS", "WEBSOCKET_EVENT_DATA");
        ESP_LOGI("WS", "Received opcode=%d", data->op_code);
        ESP_LOGW("WS", "Received=%.*s", data->data_len, (char *)data->data_ptr);
        ESP_LOGW("WS", "Total payload length=%d, data_len=%d, current payload offset=%d\r\n", data->payload_len, data->data_len, data->payload_offset);
        // Parsear el string JSON
        cJSON *json = cJSON_Parse((char *)data->data_ptr);
        if (json == NULL) {
        printf("Error al parsear JSON.\n");
        }
        else{
        char *jsonFormatted = cJSON_Print(json);
        printf("JSON parseado:\n%s\n", jsonFormatted);
        free(jsonFormatted);
          // Buscar la etiqueta "gait"
    cJSON *etiqueta_gait = cJSON_GetObjectItemCaseSensitive(json, "gait");
    if (cJSON_IsNumber(etiqueta_gait)) {
        // Si la etiqueta "gait" existe y su valor es un número, obtener el valor numérico
        gait = etiqueta_gait->valueint;
        printf("Valor de la etiqueta 'gait': %d\n", gait);
    } else {
        printf("La etiqueta 'gait' no se encontró o su valor no es un número.\n");
    }
    printf("GAIT %d",gait);

    // Buscar la etiqueta "joystickData"
    cJSON *etiqueta_joystickData = cJSON_GetObjectItemCaseSensitive(json, "joystickData");
    if (cJSON_IsArray(etiqueta_joystickData)) {
        // Si la etiqueta "joystickData" es un array, procesar cada elemento del array
        int num_elementos = cJSON_GetArraySize(etiqueta_joystickData);
        for (int i = 0; i < num_elementos; i++) {
            // Obtener el elemento actual del array
            cJSON *elemento = cJSON_GetArrayItem(etiqueta_joystickData, i);
            // Verificar si el elemento es un objeto
            if (cJSON_IsObject(elemento)) {
                // Obtener los valores de las etiquetas "x", "y" y "active" del objeto
                cJSON *x_etiqueta = cJSON_GetObjectItemCaseSensitive(elemento, "x");
                cJSON *y_etiqueta = cJSON_GetObjectItemCaseSensitive(elemento, "y");
                cJSON *active_etiqueta = cJSON_GetObjectItemCaseSensitive(elemento, "active");
                // Verificar si los valores son números
                // if (cJSON_IsNumber(x_etiqueta) && cJSON_IsNumber(y_etiqueta) && cJSON_IsBool(active_etiqueta)) {
                double x_valor = x_etiqueta->valuedouble;
                double y_valor = y_etiqueta->valuedouble;
                bool active_valor = cJSON_IsTrue(active_etiqueta);
                printf("Coord trad %f %f",x_valor,y_valor);
                //Obtener i y asociar a que coordenadas/inclinacion toca
                switch(i){
                    case 0:                //0 Movimiento x-y
                    spin=false;
                    coordinates[0]=0.05*x_valor/2; //CHEQUEAR VMAX
                    coordinates[1]=0.05*y_valor/2;
                    coordinates[2]=-0.135;
                    printf("Coord trad %f %f",coordinates[0],coordinates[1]);
                    break;
                    case 1:                       //1 PITCH ROLL
                    spin=false;
                    incline[0]=0.05*x_valor/2.0; //CHEQUEAR VMAX
                    incline[1]=0.05*y_valor/2.0;
                    break;
                    case 2:
                    spin=false;
                    coordinates[0]=0.05*x_valor/2.0;
                    break;
                    // case 2: //YAW
                    //  incline[2]=0.05*y_valor/2.0; //CONVERTIR A GRADOS
                    // break;
                    case 3:
                    coordinates[1]=0.05*y_valor/2.0;
                    spin=true;
                    break;
                }
                //2 YAW
                //3 EJE Z
                //
                // Imprimir los valores obtenidos
               // printf("Elemento %d - x: %d, y: %d, active: %s\n", i+1, x_valor, y_valor, active_valor ? "true" : "false");
                
            } else {
                printf("El elemento  no es un objeto.\n");
            }
        }
    } else {
        printf("La etiqueta 'joystickData' no es un array o no se encontró en el objeto JSON.\n");
    }
        }
    


        // // Liberar memoria utilizada por el JSON
        cJSON_Delete(json);
        
        // // PROCESAR DATOS RECIBIDOS SI HAY DATOS LLAMAR A FUNCIONES DE MOVIMIENTO
        // xTimerReset(shutdown_signal_timer, portMAX_DELAY);
        break;
    }
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGI("WS", "WEBSOCKET_EVENT_ERROR");
        break;
    }
}
static void shutdown_signaler(TimerHandle_t xTimer)
{
    ESP_LOGI("WS", "No data received for %d seconds, signaling shutdown", 10);
    xSemaphoreGive(shutdown_sema);
}

void websocket_start(){
    //Iniciar camara
   caminit();
        
    esp_websocket_client_config_t websocket_cfg = {};

    shutdown_signal_timer = xTimerCreate("Websocket shutdown timer", 1000 * 1000 / portTICK_PERIOD_MS,
                                         pdFALSE, NULL, shutdown_signaler);
    //shutdown_sema = xSemaphoreCreateBinary();
    //websocket_cfg.uri="ws";
    websocket_cfg.host="192.168.1.41";
    websocket_cfg.port=8885;

    esp_websocket_client_handle_t client =esp_websocket_client_init(&websocket_cfg);
    
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_handler, (void *)client);

    esp_websocket_client_start(client);
    
    //xTimerStart(shutdown_signal_timer, portMAX_DELAY);
      while(1){
    camera_fb_t * fb=esp_camera_fb_get();
    if (!fb){
        ESP_LOGI("CAM","FRAME NOT AQUIRED");
        esp_camera_fb_return(fb);
        return;
    }
    //ESP_LOGI("CAM","FRAME AQUIRED");
    esp_websocket_client_send_bin(client,(const char *)fb->buf,fb->len,portMAX_DELAY);
    esp_camera_fb_return(fb);
     vTaskDelay(pdMS_TO_TICKS(120));

     }

   // xSemaphoreTake(shutdown_sema, portMAX_DELAY);

    

    //Obtenr imagen y enviar imagen // PASAR A OTRA FUNCION PARA ENVIAR
    // camera_fb_t * fb=esp_camera_fb_get();
    // if (!fb){
    //     ESP_LOGE("CAM","FRAME NOT AQUIRED");
    //     return;
    // }

    // // if esp_weboscket_client_is_connected(client)
    // esp_websocket_client_send_bin(client,(const char *)fb->buf,fb->len,portMAX_DELAY);
    // xSemaphoreTake(shutdown_sema, portMAX_DELAY);


    // //Condicion de stop
    // esp_websocket_client_stop(client);
    // ESP_LOGI("WS", "Websocket Stopped");
    // esp_websocket_client_destroy(client);



    
}

