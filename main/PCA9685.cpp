#include "PCA9685.hpp"
#include "i2cfunctions.hpp"
#include <stdio.h>
#include "math.h"
#include "esp_log.h"



bool pca9685::pcaReset(){

    return (send_bytes(PCA9685_ADDR,MODE_REG_1,RESET));

}


bool pca9685::pwm_control(uint8_t puerto,uint16_t time_on, uint16_t time_off){  //uint8_t port,uint16_t time_on, uint16_t time_off

    uint8_t reg_to_write= START_REG_PWM + LED_MULTIPLYER * puerto;
    // send_bytes(PCA9685_ADDR,reg_to_write,time_on);
    // reg_to_write= LED0_OFF_L + LED_MULTIPLYER * puerto;
    // send_bytes(PCA9685_ADDR,reg_to_write,time_off);
    return (send_two_words(PCA9685_ADDR,time_on,time_off,reg_to_write));

}
bool pca9685::pwm_get(uint8_t num, uint16_t* dataOn, uint16_t* dataOff){
   
    uint8_t readPWMValueOn0;
    uint8_t readPWMValueOn1;
    uint8_t readPWMValueOff0;
    uint8_t readPWMValueOff1;
    uint8_t pinAddress = 6 + 4 * num;
    int ret=0;
    ret = read_bytes_1(PCA9685_ADDR, &readPWMValueOn0, &readPWMValueOn1,pinAddress);

    if (ret != ESP_OK) {
        return ret;
    }
    pinAddress = LED0_OFF_L + LED_MULTIPLYER * num;

    ret = read_bytes_1(PCA9685_ADDR, &readPWMValueOff0, &readPWMValueOff1,pinAddress);

    if (ret != ESP_OK) {
        return ret;
    }

    *dataOn = (readPWMValueOn1 << 8) | readPWMValueOn0;
    *dataOff = (readPWMValueOff1 << 8) | readPWMValueOff0;
    printf("OWError reset %hu\n",*dataOn);
    printf("OWError reset %hu\n",*dataOff);

    return ret;

   

}

bool pca9685::turnoff(){  //uint8_t port,uint16_t time_on, uint16_t time_off

    uint16_t ton=0;
    uint16_t toff=4096;
    uint8_t alloff=0xFA;
    return (send_two_words(PCA9685_ADDR,ton,toff,alloff));

}
bool pca9685::turnon(){  //uint8_t port,uint16_t time_on, uint16_t time_off

    uint16_t ton=4096;
    uint16_t toff=0;
    uint8_t alloff=0xFA;
    return (send_two_words(PCA9685_ADDR,ton,toff,alloff));

}



bool pca9685::setfrequency(uint16_t freq){

    esp_err_t ret;
    int err=0;

    // Send to sleep
    ret = send_bytes(PCA9685_ADDR,MODE_REG_1,0x10);
    printf("eee %d",err);
    if (ret != 1) {
        return ret;
    }

    // Set prescaler
    // calculation on page 25 of datasheet
    uint8_t prescale_val = round((CLOCK_FREQ / 4096 / (0.9*freq)) - 1+0.5);
    ret = send_bytes(PCA9685_ADDR,PRE_SCALE,prescale_val);
    printf("eee %d",err);
    if (ret != 1) {
        return ret;
    }

    // reset again
    err=pcaReset();
    printf("eee %d",err);

    // Send to sleep again
    ret = send_bytes(PCA9685_ADDR,MODE_REG_1,0x10);
    if (ret != 1) {
        return ret;
    }

    // wait
    vTaskDelay(5/portTICK_PERIOD_MS);

    // Write 0xa0 for auto increment LED0_x after received cmd
    ret = send_bytes(PCA9685_ADDR,MODE_REG_1,0xA0);
    if (ret != 1) {
        return ret;
    }
    return ret;

}

