#include "driver/i2c.h"
#define i2c_master_port (i2c_port_t)0x00

void i2c_init();

bool send_bytes(uint16_t addr,uint8_t reg,uint8_t data);

bool read_bytes(uint8_t addr,uint8_t reg,uint8_t *buf,uint32_t len);

bool send_two_words(uint16_t addr,uint16_t data_on,uint16_t data_off,uint8_t reg);

bool read_bytes_1(uint16_t addr,uint8_t* data_on,uint8_t* data_off,uint8_t reg);