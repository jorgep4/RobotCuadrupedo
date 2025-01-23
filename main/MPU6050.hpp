#define MPU6050_ADDRESS 0x68
#define ACCEL_X_REG 0x3B
#define ACCEL_Y_REG 0x3D
#define ACCEL_Z_REG 0x3F
#define ACCEL_X_REG 0x3B
#define GYRO_X_REG 0x43
#define GYRO_Y_REG 0x45
#define GYRO_Z_REG 0x47
#define TEMP_REG 0x41
#define DEVICE_MODE 0x6B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define SAMPLE_CONFIG 0x19
#define CONFIG_REG 0x1A


class mpu6050 {
    public:
        bool iniciar();
        float accel_x();
        float accel_y();
        float accel_z();
        float gyro_x();
        float gyro_y();
        float gyro_z();
        float temperature();
};