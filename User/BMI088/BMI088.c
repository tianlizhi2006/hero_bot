#include "BMI088.h"

#include "spi.h"

/* 陀螺仪 ±2000dps：灵敏度 16.384 LSB/dps */
#define BMI088_GYRO_LSB_PER_DPS   16.384f
/* 加速度计 ±24g：灵敏度 1365.333 LSB/g */
#define BMI088_ACCEL_LSB_PER_G    1365.333f
/* 度转弧度 */
#define DEG2RAD  0.01745329252f

/* 陀螺仪零偏（rad/s），上电静止校准得到 */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

/* 校准采样次数 */
#define CALIB_SAMPLES  1000

/* 写单个寄存器（最高位为0表示写） */
static void BMI088_Gyro_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;
    tx[1] = val;

    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    GYRO_CS_HIGH();
}

/* BMI088 陀螺仪初始化：设置量程退出挂起模式 */
void BMI088_Gyro_Init(void)
{
    /* 软复位（写 0xB6 到寄存器 0x14） */
    BMI088_Gyro_WriteReg(0x14, 0xB6);
    HAL_Delay(30);

    /* 设置量程为 ±2000dps（写 0x00 到 GYRO_RANGE 0x0F，并使陀螺仪退出挂起模式） */
		//量程最大值是-32768 ~ +32767
    BMI088_Gyro_WriteReg(0x0F, 0x00);

    /* 设置带宽：ODR 2000Hz（0x10 复位默认值即为 0x80） */
    BMI088_Gyro_WriteReg(0x10, 0x80);

    /* 关键：写 GYRO_LPM1 (0x11) = 0x00，让陀螺仪退出挂起(suspend)模式，进入正常测量模式。
       软复位/上电后陀螺仪默认处于挂起模式，此时 RATE 寄存器(0x02~0x07)不会更新。 */
    BMI088_Gyro_WriteReg(0x11, 0x00);
}

uint8_t BMI088_Gyro_ReadReg(uint8_t *reg_arr,uint8_t i)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg_arr[i] | 0x80;
    tx[1] = 0x00;

    GYRO_CS_LOW();

    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);

    GYRO_CS_HIGH();

    return rx[1];
}

void BMI088_Gyro_ReadReg_all(int16_t *Data )
{
		uint8_t reg[6] = {0x02,0x03,0x04,0x05,0x06,0x07};
        uint8_t data_temp[6];
        for(int i = 0; i < 6; i++)
        {
            data_temp[i] = BMI088_Gyro_ReadReg(reg, i);
        }
        // 将高八位与低八位整合
        Data[0] = (int16_t)(data_temp[1] << 8) | data_temp[0];
        Data[1] = (int16_t)(data_temp[3] << 8) | data_temp[2];
        Data[2] = (int16_t)(data_temp[5] << 8) | data_temp[4];
}

/* 读取陀螺仪三轴角速度，单位 rad/s，并做零偏补偿 */
void BMI088_Gyro_ReadAll(Vector3f_t *gyro)
{
    int16_t raw[3];
    BMI088_Gyro_ReadReg_all(raw);

    gyro->x = (raw[0] / BMI088_GYRO_LSB_PER_DPS) * DEG2RAD - gyro_bias_x;
    gyro->y = (raw[1] / BMI088_GYRO_LSB_PER_DPS) * DEG2RAD - gyro_bias_y;
    gyro->z = (raw[2] / BMI088_GYRO_LSB_PER_DPS) * DEG2RAD - gyro_bias_z;
}

/* 写单个寄存器（最高位为0表示写） */
static void BMI088_Accel_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;
    tx[1] = val;

    ACCEL_CS_LOW();
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    ACCEL_CS_HIGH();
}

/* BMI088 加速度计初始化：使能加速度计并设置量程与带宽 */
void BMI088_Accel_Init(void)
{
    /* 软复位（写 0xB6 到寄存器 0x7E） */
    BMI088_Accel_WriteReg(0x7E, 0xB6);
    HAL_Delay(30);

    /* 使能加速度计（ACC_PWR_CTRL 0x7D = 0x04） */
    BMI088_Accel_WriteReg(0x7D, 0x04);
    HAL_Delay(30);

    /* 关闭挂起模式（ACC_PWR_CONF 0x7C = 0x00） */
    BMI088_Accel_WriteReg(0x7C, 0x00);
    HAL_Delay(30);

    /* 设置带宽/采样率：ODR 1600Hz（ACC_CONF 0x40 = 0xAC） */
    BMI088_Accel_WriteReg(0x40, 0xAC);

    /* 设置量程为 ±24g（ACC_RANGE 0x41 = 0x03）
       量程最大值是 -32768 ~ +32767 */
    BMI088_Accel_WriteReg(0x41, 0x03);
}

uint8_t BMI088_Accel_ReadReg(uint8_t *reg_arr, uint8_t i)
{
    uint8_t tx[3];
    uint8_t rx[3];

    /* 加速度计读取需要多一个 dummy 字节，数据在第 3 个字节返回 */
    tx[0] = reg_arr[i] | 0x80;
    tx[1] = 0x00;
    tx[2] = 0x00;

    ACCEL_CS_LOW();

    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3, 100);

    ACCEL_CS_HIGH();

    return rx[2];
}

void BMI088_Accel_ReadReg_all(int16_t *Data)
{
    uint8_t reg[6] = {0x12,0x13,0x14,0x15,0x16,0x17};
    uint8_t data_temp[6];
    for (int i = 0; i < 6; i++)
    {
        data_temp[i] = BMI088_Accel_ReadReg(reg, i);
    }
    // 将高八位与低八位整合（低字节在前）
    Data[0] = (int16_t)((data_temp[1] << 8) | data_temp[0]);
    Data[1] = (int16_t)((data_temp[3] << 8) | data_temp[2]);
    Data[2] = (int16_t)((data_temp[5] << 8) | data_temp[4]);
}

/* 读取加速度计三轴数据，单位 g */
void BMI088_Accel_ReadAll(Vector3f_t *acc)
{
    int16_t raw[3];
    BMI088_Accel_ReadReg_all(raw);

    acc->x = raw[0] / BMI088_ACCEL_LSB_PER_G;
    acc->y = raw[1] / BMI088_ACCEL_LSB_PER_G;
    acc->z = raw[2] / BMI088_ACCEL_LSB_PER_G;
}


/* 初始化陀螺仪与加速度计 */
void BMI088_Init(void)
{
    BMI088_Gyro_Init();
    BMI088_Accel_Init();
}

/* 上电静止校准：多次采样求陀螺仪零偏（rad/s） */
void BMI088_Calibrate(void)
{
    int16_t g_raw[3];
    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;

    HAL_Delay(100);    /* 等待传感器稳定 */

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        BMI088_Gyro_ReadReg_all(g_raw);
        gx_sum += g_raw[0];
        gy_sum += g_raw[1];
        gz_sum += g_raw[2];
        HAL_Delay(1);
    }

    gyro_bias_x = (gx_sum / CALIB_SAMPLES) / BMI088_GYRO_LSB_PER_DPS * DEG2RAD;
    gyro_bias_y = (gy_sum / CALIB_SAMPLES) / BMI088_GYRO_LSB_PER_DPS * DEG2RAD;
    gyro_bias_z = (gz_sum / CALIB_SAMPLES) / BMI088_GYRO_LSB_PER_DPS * DEG2RAD;
}
