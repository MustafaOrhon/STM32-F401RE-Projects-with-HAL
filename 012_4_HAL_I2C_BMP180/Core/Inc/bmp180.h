/*
 * bmp180.h
 *
 *  Created on: Jan 24, 2023
 *      Author: Mustafa ORHON
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;
/*BMP180 DEVICE ADDRESS*/
#define BMP180_DeviceWriteRegisterAddress 0xEE
#define BMP180_DeviceReadRegisterAddress 0xEF

#define BMP180_Calibration_Value_Lenght 22




void BMP180_Init(void);
void BMP180_GetCalibration(void);
void BMP180_GetCalibration_Value(void);
void BMP180_Get_Uncompansated_Temp(void);
float BMP180_Get_Temp(void);
void BMP180_Get_Uncompansated_Pressure(void);
float BMP180_Get_Pressure(void);
#endif /* INC_BMP180_H_ */
