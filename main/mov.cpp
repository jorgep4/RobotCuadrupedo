#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "stdio.h"
#include "esp_log.h"
#include "PCA9685.hpp"
#include "MPU6050.hpp"
#include "i2cfunctions.hpp"
#include <math.h>
#include "robotfunctions.hpp"
#include "websocket.hpp"
#include "wififunctions.hpp"
#include "nvs_flash.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

TaskHandle_t myTaskHandle1 = NULL;
TaskHandle_t myTaskHandle2 = NULL;



void task_websocket(void *ignore){
    //Evitar reinicios Guru Meditation Core (problemas de potencia)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    //Inicializar NVS para poder utilizar el wifi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Si no hay páginas libres o hay una versión nueva encontrada, borrar el almacenamiento NVS
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    //Iniciar WIFI
    connect_wifi();
    vTaskDelay(pdMS_TO_TICKS(5000));
    //Iniciar WEBSOCKET
    websocket_start();
  

}
void task_robot(void *ignore){
 //Inicializar puerto serie
  esp_log_level_set("mpu6050", ESP_LOG_INFO);
 // Inicializar I2C
  i2c_init();
  //Inicializar MPU6050 y pca
    mpu6050 mpu;
    pca9685 pca;
    bool err;
       uint16_t freq=50;
   err=pca.pcaReset();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    // ESP_LOGI("PCA","Error reset %d",err);
    // vTaskDelay(pdMS_TO_TICKS(5000));
   err=pca.setfrequency(freq);
   // ESP_LOGI("PCA","Error reset %d",err);
    // if (mpu.iniciar()==0){
    //     ESP_LOGE("mpu6050","Inicialización incorrecta");
    // }
    // else{
    //     ESP_LOGI("mpu6050","Inicialización correcta");
    // }

    //Inicializar PCA9685 solo si hace falta cambiar frecuencia --Apartado de momento
   err=pca.turnoff();
   while(1){
    movimiento();
    vTaskDelay(pdMS_TO_TICKS(10));
   }


    // err=pca.pwm_control(3,0,0);
   //  uint16_t myDataOn;
   //  uint16_t myDataOff;
    // err=pca.pwm_get(3,&myDataOn,&myDataOff);
    // ESP_LOGI("PCA","Error reset %d",err);
    // vTaskDelay(pdMS_TO_TICKS(5000));
    // uint16_t pwmTable[2] = {3,3};
   //  uint16_t grados;

   //     float coordinates[3];
   //     coordinates[0]=0.00;
   //     coordinates[1]=0;
   //     coordinates[2]=-0.15;
   //     float coordinates_r[3];
   //      coordinates_r[0]=0;
   //     coordinates_r[1]=0;
   //     coordinates_r[2]=0;
      //  float theta[3]={0,0,0};
      //  float theta1[3]={0,0,0};
      //  float theta2[3]={0,0,0};
      //  float theta3[3]={0,0,0};
      //  float result[3];
  

}

extern "C" {void app_main(){

  xTaskCreate(task_robot, "task_robot", 4096, (void* ) 0, 10, &myTaskHandle1);
//   xTaskCreate(task_websocket, "task_websocket", 4096, (void* ) 0, 10, &myTaskHandle1);
//   xTaskCreatePinnedToCore(task_websocket, "task_websocket", 4096, NULL,10, &myTaskHandle2, 1);
  }
}
   