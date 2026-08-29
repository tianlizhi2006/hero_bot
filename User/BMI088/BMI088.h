#include "main.h"
#ifndef __BMI088_H__
#define __BMI088_H__


#define GYRO_CS_LOW()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
#define GYRO_CS_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)

#define ACCEL_CS_LOW()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)
#define ACCEL_CS_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)


/* 三维向量：陀螺仪 rad/s，加速度 g */
typedef struct
{
    float x;
    float y;
    float z;
} Vector3f_t;


void BMI088_Gyro_Init(void);
uint8_t BMI088_Gyro_ReadReg(uint8_t *reg_arr,uint8_t i);
void BMI088_Gyro_ReadReg_all(int16_t *Data );

void BMI088_Accel_Init(void);
uint8_t BMI088_Accel_ReadReg(uint8_t *reg_arr, uint8_t i);
void BMI088_Accel_ReadReg_all(int16_t *Data);

void BMI088_Init(void);
void BMI088_Calibrate(void);
void BMI088_Gyro_ReadAll(Vector3f_t *gyro);
void BMI088_Accel_ReadAll(Vector3f_t *acc);
#endif
