#include "i2cfunctions.hpp"


bool send_bytes(uint16_t addr,uint8_t reg,uint8_t data){

    //Crear handler comunicación I2C, se crea una cola con los comandos
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    //Iniciar comunicacion
    i2c_master_start(cmd);

    // Activar escritura
    i2c_master_write_byte(cmd,(addr<<1) | I2C_MASTER_WRITE,I2C_MASTER_NACK);//ACK

    // Elegir registro a escribir 
    i2c_master_write_byte(cmd,reg,I2C_MASTER_NACK);

    //Escribir datos
    i2c_master_write_byte(cmd, data,I2C_MASTER_NACK);

    // Acabar
    i2c_master_stop(cmd);

    // Empezar a enviar comandos de la cola
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);

    // Fin link i2c
    i2c_cmd_link_delete(cmd);

    //vTaskDelay(50 / portTICK_PERIOD_MS);

    if (ret == ESP_FAIL) {
        return false;
    }
    return true;
}



bool read_bytes(uint8_t addr,uint8_t reg,uint8_t *buf,uint32_t len){
    //Crear handler comunicación I2C, se crea una cola con los comandos
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    //Iniciar comunicacion
    i2c_master_start(cmd);

    // Activar escritura
    i2c_master_write_byte(cmd,addr<<1,I2C_MASTER_NACK);//ACK

    // Elegir registro a leer
    i2c_master_write_byte(cmd,reg,I2C_MASTER_NACK);

    // Acabar
    i2c_master_stop(cmd);

    // Empezar a enviar comandos de la cola
    int ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);

    // Fin link i2c
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_FAIL) {
        return false;
    }

    // Crear handler
    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    // Poner en modo lectura
    i2c_master_write_byte(cmd, addr << 1 | 1, I2C_MASTER_NACK);

    //Leer
    while(len) {
        i2c_master_read_byte(cmd, buf,I2C_MASTER_NACK);
        buf++;
        len--;
    }
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_FAIL) {
        return false;
    }

    return true;

}

bool send_two_words(uint16_t addr,uint16_t data_on,uint16_t data_off,uint8_t reg){

    //Crear handler comunicación I2C, se crea una cola con los comandos
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    //Iniciar comunicacion
    i2c_master_start(cmd);

    // Activar escritura
    i2c_master_write_byte(cmd,(addr<<1) | I2C_MASTER_WRITE,I2C_MASTER_NACK);//ACK

    // Elegir registro a escribir 
    i2c_master_write_byte(cmd,reg,I2C_MASTER_NACK);

    //Escribir datos
    i2c_master_write_byte(cmd, data_on & 0xff,I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, data_on >> 8,I2C_MASTER_NACK);
    i2c_master_write_byte(cmd, data_off & 0xff,I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, data_off >> 8,I2C_MASTER_NACK);

    // Acabar
    i2c_master_stop(cmd);

    // Empezar a enviar comandos de la cola
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);

    // Fin link i2c
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_FAIL) {
        return false;
    }

    return true;
}

void i2c_init(){
    i2c_config_t conf;
    i2c_port_t port;
    port= I2C_NUM_0;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 14;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 15;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(port, &conf);
    i2c_driver_install(port, conf.mode, 0, 0, 0);


}

bool read_bytes_1(uint16_t addr,uint8_t* data_on,uint8_t* data_off,uint8_t reg){
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) , I2C_MASTER_NACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, I2C_MASTER_NACK);
    i2c_master_read_byte(cmd, data_on, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data_off, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}