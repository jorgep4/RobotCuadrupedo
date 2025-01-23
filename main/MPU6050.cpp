#include "MPU6050.hpp"
#include "i2cfunctions.hpp"

#define ACCEL_SCALE 2 // 2G 4G 8G 16G 
#define GYRO_SCALE 250 // º/s

#if ACCEL_SCALE == 2
    #define sens_accel (float)(16384)
#elif ACCEL_SCALE == 4
    #define sens_accel (float)(8192)
#elif ACCEL_SCALE == 8
    #define sens_accel (float)(4096)
#elif ACCEL_SCALE == 16
    #define sens_accel (float)(2048)
#endif

#if GYRO_SCALE == 250
    #define sens_gyro (float)(131.0)
#elif GYRO_SCALE == 500
    #define sens_gyro (float) (65.5)
#elif GYRO_SCALE == 1000
    #define sens_gyro (float) (32.8)
#elif GYRO_SCALEE == 2000
    #define sens_gyro (float) (16.4)
#endif


bool mpu6050::iniciar(){

    if(send_bytes(MPU6050_ADDRESS,DEVICE_MODE,0x00)==0){
        return false;
    }
    if(send_bytes(MPU6050_ADDRESS,CONFIG_REG,0x07)==0){
        return false;
    }
    if(send_bytes(MPU6050_ADDRESS,SAMPLE_CONFIG,0x07)==0){
        return false;
    }
    if(send_bytes(MPU6050_ADDRESS,GYRO_CONFIG,0x18)==0){
        return false;
    }
    if(send_bytes(MPU6050_ADDRESS,ACCEL_CONFIG,0x01)==0){
        return false;
    }
    return true;

}
 
float mpu6050::accel_x(){

    uint8_t value_read[2];
    read_bytes(MPU6050_ADDRESS,ACCEL_X_REG,value_read,2);
    float value_read_full =( value_read[0]<<8 | value_read[1])/sens_accel;
    return value_read_full;
    
}

float mpu6050::accel_y(){

    uint8_t value_read[0];
    read_bytes(MPU6050_ADDRESS,ACCEL_Y_REG,value_read,2);
    short value_read_full = value_read[0] << 8 | value_read[1];
    return (float)value_read_full / sens_accel;

}

float mpu6050::accel_z(){

    uint8_t value_read[2];
    read_bytes(MPU6050_ADDRESS,ACCEL_Z_REG,value_read,2);
    float value_read_full = (value_read[0]<<8 | value_read[1])/sens_accel;
    return value_read_full;

}

float mpu6050::gyro_x(){

    uint8_t value_read[2];
    read_bytes(MPU6050_ADDRESS,GYRO_X_REG,value_read,2);
    float value_read_full = (value_read[0]<<8 | value_read[1])/sens_gyro;
    return value_read_full;

}

float mpu6050::gyro_y(){

    uint8_t value_read[2];
    read_bytes(MPU6050_ADDRESS,GYRO_Y_REG,value_read,2);
    float value_read_full = (value_read[0]<<8 | value_read[1])/sens_gyro;
    return value_read_full;

}

float mpu6050::gyro_z(){

    uint8_t value_read[2];
    read_bytes(MPU6050_ADDRESS,GYRO_Z_REG,value_read,2);
    float value_read_full = (value_read[0]<<8 | value_read[1])/sens_gyro;
    return value_read_full;

}

float mpu6050::temperature(){

    uint8_t value_read[2];
    read_bytes(MPU6050_ADDRESS,TEMP_REG,value_read,2);
    float value_read_full = (value_read[0]<<8 | value_read[1]);
    return value_read_full;

}
