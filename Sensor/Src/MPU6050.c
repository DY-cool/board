//
// Created by 10171 on 2022/1/2.
//

#include "MPU6050.h"
#include "i2c.h"
#include "ROS_ShareWare.h"


/*-----------------------------------------------------------

 -----------------------------------------------------------*/
void MPU6050_Init(MPU6050_Struct *MPU6050)
{
    uint8_t I2c1_SenData;

    HAL_Delay(100);

    I2c1_SenData = 0x02;
    I2C1_Tx_Flag = 0;
    HAL_I2C_Mem_Write_IT(&hi2c1, MPU6050_DevAddr, MPU6050_RA_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &I2c1_SenData, 1);//ÉèÖÃÊ±ÖÓ
    while(!I2C1_Tx_Flag){};

    I2c1_SenData = 0x18;
    I2C1_Tx_Flag = 0;
    HAL_I2C_Mem_Write_IT(&hi2c1, MPU6050_DevAddr, MPU6050_RA_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &I2c1_SenData, 1);//ÍÓÂÝÒÇ×î´óÁ¿³Ì +-2000¶ÈÃ¿Ãë
    while(!I2C1_Tx_Flag){};

    I2c1_SenData = 0x00;
    I2C1_Tx_Flag = 0;
    HAL_I2C_Mem_Write_IT(&hi2c1, MPU6050_DevAddr, MPU6050_RA_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &I2c1_SenData, 1);//¼ÓËÙ¶È¶È×î´óÁ¿³Ì +-2g
    while(!I2C1_Tx_Flag){};

    I2c1_SenData = 0x02;
    I2C1_Tx_Flag = 0;
    HAL_I2C_Mem_Write_IT(&hi2c1, MPU6050_DevAddr, MPU6050_RA_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &I2c1_SenData, 1);//½øÈë¹¤×÷×´Ì¬
    while(!I2C1_Tx_Flag){};

    MPU6050->Accel_XRaw = 0;//
    MPU6050->Accel_YRaw = 0;//
    MPU6050->Accel_ZRaw = 0;//
    MPU6050->TemperatureRaw = 0;//
    MPU6050->Gyro_XRaw = 0;//
    MPU6050->Gyro_YRaw = 0;//
    MPU6050->Gyro_ZRaw = 0;//

    MPU6050->Accel_X = 0;//
    MPU6050->Accel_Y = 0;//
    MPU6050->Accel_Z = 0;//

    MPU6050->Gyro_X = 0;//
    MPU6050->Gyro_Y = 0;//
    MPU6050->Gyro_Z = 0;//

    MPU6050->Temperature = 0;
}


/*-----------------------------------------------------------

 -----------------------------------------------------------*/
void MPU6050_Update(MPU6050_Struct *MPU6050)
{
    MPU6050->Accel_XRaw = (MPU6050->Buf[0]<<8) + MPU6050->Buf[1];
    MPU6050->Accel_YRaw = (MPU6050->Buf[2]<<8) + MPU6050->Buf[3];
    MPU6050->Accel_ZRaw = (MPU6050->Buf[4]<<8) + MPU6050->Buf[5];
    MPU6050->TemperatureRaw = (MPU6050->Buf[6]<<8) + MPU6050->Buf[7];
    MPU6050->Temperature = 36.53 + (float)MPU6050->TemperatureRaw / 340;
    MPU6050->Gyro_XRaw = (MPU6050->Buf[8]<<8) + MPU6050->Buf[9];
    MPU6050->Gyro_YRaw = (MPU6050->Buf[10]<<8) + MPU6050->Buf[11];//
    MPU6050->Gyro_ZRaw = (MPU6050->Buf[12]<<8) + MPU6050->Buf[13];//
}

/*-----------------------------------------------------------

 -----------------------------------------------------------*/
void MPU6050_UpdateAccel(MPU6050_Struct *MPU6050)
{
    MPU6050->Accel_X = (float)MPU6050->Accel_XRaw / 1638.375f;//
    MPU6050->Accel_Y = (float)MPU6050->Accel_YRaw / 1638.375f;//
    MPU6050->Accel_Z = (float)MPU6050->Accel_ZRaw / 1638.375f;//
}

/*-----------------------------------------------------------

 -----------------------------------------------------------*/
void MPU6050_UpdateGyro(MPU6050_Struct *MPU6050)
{
    MPU6050->Gyro_X = (float)MPU6050->Gyro_XRaw / 938.72f;//
    MPU6050->Gyro_Y = (float)MPU6050->Gyro_YRaw / 938.72f;//
    MPU6050->Gyro_Z = (float)MPU6050->Gyro_ZRaw / 938.72f;//
}

