#include <cstdint>
#define PCA9685_ADDR 0x40
#define RESET 0x80
#define START_REG_PWM 0x6
#define MODE_REG_1 0x00
#define LED0_OFF_L 0x8
#define ALLOFF 0xFA
#define CLOCK_FREQ      25000000.0  /*!< 25MHz default osc clock */
#define PRE_SCALE 0xFE
#define LED_MULTIPLYER 4



class pca9685{
    public:
        bool pwm_control(uint8_t puerto,uint16_t time_on, uint16_t time_off);
        bool turnoff();
        bool pcaReset();
        bool setfrequency(uint16_t freq);
        bool pwm_get(uint8_t num, uint16_t* dataOn, uint16_t* dataOff);
        bool turnon();
};