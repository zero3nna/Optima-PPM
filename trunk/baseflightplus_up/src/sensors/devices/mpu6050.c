#include "board.h"
#include "drivers/i2c.h"

#define MPU6050_ADDRESS         0x68

#define MPU_RA_AUX_VDDIO        0x00
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

//                          Accelerometer (Fs = 1kHz)   Gyroscope 
//                          Bandwidth (Hz)  Delay (ms)  Bandwidth (Hz)  Delay (ms)  Fs (kHz)
#define DLPF_CFG_0 0   //   260             0           256             0.98        8
#define DLPF_CFG_1 1   //   184             2.0         188             1.9         1
#define DLPF_CFG_2 2   //   94              3.0         98              2.8         1
#define DLPF_CFG_3 3   //   44              4.9         42              4.8         1
#define DLPF_CFG_4 4   //   21              8.5         20              8.3         1
#define DLPF_CFG_5 5   //   10              13.8        10              13.4        1
#define DLPF_CFG_6 6   //   5               19.0        5               18.6        1

#define AFS_SEL_2G    0     // Â± 2g     - 16384 LSB/mg
#define AFS_SEL_4G    1     // Â± 4g     - 8192 LSB/mg
#define AFS_SEL_8G    2     // Â± 8g     - 4096 LSB/mg
#define AFS_SEL_16G   3     // Â± 16g    - 2048 LSB/mg

#define FS_SEL_250    0     // Â± 250 Â°/s
#define FS_SEL_500    1     // Â± 500 Â°/s
#define FS_SEL_1000   2     // Â± 1000 Â°/s
#define FS_SEL_2000   3     // Â± 2000 Â°/s

#define CLK_SEL_INTERNAL    0 // Internal 8MHz oscillator
#define CLK_SEL_PLL_GX      1 // PLL with X Gyro reference
#define CLK_SEL_PLL_GY      2 // PLL with Y Gyro reference
#define CLK_SEL_PLL_GZ      3 // PLL with Z Gyro reference
#define CLK_SEL_PLL_EXT32   4 // PLL with external 32.768kHz reference
#define CLK_SEL_PLL_EXT19   5 // PLL with external 19.2MHz reference
#define CLK_SEL_STOP        7 // Stops the clock and keeps the timing generator in reset

#define H_RESET             (1<<7)

///////////////////////////////////////////////////////////////////////////////
// MPU3050/6050 Variables
///////////////////////////////////////////////////////////////////////////////

// MPU3050 14.375 LSBs per dps at Â±2000 Âº/s
// scale factor to get rad/s: (1/14.375*PI/180) = 0.00121414208834388144
#define MPU6050_GYRO_SCALE_FACTOR     0.00121414208834388144f

void mpu6050AccelRead(int16_t *accValues)
{
    uint8_t buf[6];

    // Get data from device
    i2cRead(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H, 6, buf);

    accValues[XAXIS] = ((buf[0] << 8) | buf[1]);
    accValues[YAXIS] = -((buf[2] << 8) | buf[3]);
    accValues[ZAXIS] = -((buf[4] << 8) | buf[5]);
}

void mpu6050GyroRead(int16_t *gyroValues)
{
    uint8_t buf[6];

    // Get data from device
    i2cRead(MPU6050_ADDRESS, MPU_RA_GYRO_XOUT_H, 6, buf);

    gyroValues[XAXIS] = ((buf[0] << 8) | buf[1]);
    gyroValues[YAXIS] = ((buf[2] << 8) | buf[3]);
    gyroValues[ZAXIS] = -((buf[4] << 8) | buf[5]);
}

void mpu6050TempRead(int16_t *temperature)
{
    uint8_t buf[2];

    // Get data from device
    i2cRead(MPU6050_ADDRESS, MPU_RA_TEMP_OUT_H, 2, buf);

    *temperature = (buf[0] << 8) | buf[1];
}

uint8_t mpu6050Detect(gyro_t *gyro, accel_t *accel)
{
    uint8_t data;
    
    if(! i2cRead(MPU6050_ADDRESS, MPU_RA_WHO_AM_I, 1, &data) || data != MPU6050_ADDRESS ) {
        return false;
    }

    gyro->init = mpu6050GyroInit;
    gyro->read = mpu6050GyroRead;
    gyro->temperature = mpu6050TempRead;
    
    accel->init = mpu6050AccelInit;
    accel->read = mpu6050AccelRead;
    
    return true;
}

void mpu6050AccelInit(void)
{
    uint8_t i = 0;

    for(i = 0; i < 3; i++) {
        sensors.accelScaleFactor[i] = ACCEL_1G / 4096.0f; 
    }
}

void mpu6050GyroInit(void)
{
    uint8_t i;

    // datasheet page 13 says 30ms.
    delay(35);

    i2cWrite(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, H_RESET);         //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(5);

    //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_SMPLRT_DIV, 7);         
    
    //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, CLK_SEL_PLL_GZ);         

    //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_CONFIG, DLPF_CFG_0);     

    // Set Gyro to Â± 2000 Â°/s
    i2cWrite(MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, FS_SEL_2000 << 3);

    // Set Accel to 8G
    // notice- mpu-6000 RevC use different scaling, hopefully we don't care ;)
    i2cWrite(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, AFS_SEL_8G << 3);
    
    for(i = 0; i < 3; ++i) {
        sensors.gyroScaleFactor[i] = MPU6050_GYRO_SCALE_FACTOR;
    }
}