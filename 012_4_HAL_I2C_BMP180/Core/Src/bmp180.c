/*
 * bmp180.c
 *
 *  Created on: Jan 24, 2023
 *      Author: Mustafa ORHON
 */

#include "bmp180.h"
/* Calibration Values*/
int16_t AC1;
int16_t AC2;
int16_t AC3;
uint16_t AC4;
uint16_t AC5;
uint16_t AC6;
int16_t B1;
int16_t B2;
int16_t MB;
int16_t MC;
int16_t MD;

/* Temp and Pressure Values */
int16_t unCompensatedTemp;
int32_t unCompensatedPressure;
float temperature;
float pressure;
int32_t tX1,tX2,tB5;
uint32_t pX1,pX2,pX3,pB3,pB7,pB4,pB6;
void BMP180_Init()
{
   if(HAL_I2C_IsDeviceReady(&hi2c1,BMP180_DeviceWriteRegisterAddress, 1, 100000) != HAL_OK)
   {
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
   }

   BMP180_GetCalibration_Value();
}
void BMP180_GetCalibration()
{

}
void BMP180_GetCalibration_Value()
{
  int a = 0;
  uint8_t calibBuff[BMP180_Calibration_Value_Lenght] = {0};
  HAL_I2C_Mem_Read(&hi2c1, BMP180_DeviceReadRegisterAddress,0xAA,1, calibBuff,BMP180_Calibration_Value_Lenght, 10000);

    AC1 = (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    AC2 = (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    AC3 = (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    AC4 = (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    AC5 = (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    AC6 = (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    B1 =  (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    B2 =  (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    MB =  (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    MC =  (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
    MD =  (int16_t *)((calibBuff[a]<<8) | calibBuff[a+1]); a+=2;
            if((AC1==0x00 )| (AC1==0xFFFF))
    		 {
    			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    			 while(1);
    		 }
    		 if((AC2==0x00) | (AC2==0xFFFF))
    		 {
    				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    				 while(1);
    			 }

    		 if((AC3==0x00 )| (AC3==0xFFFF))
    		 {
    				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    				 while(1);
    			 }

    		 if((AC4==0x00) | (AC4==0xFFFF))
    		 {
    				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    				 while(1);
    			 }

    		 if((AC5==0x00) | (AC5==0xFFFF))
    		 {
    				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    				 while(1);
    			 }

    		 if((AC6==0x00) | (AC6==0xFFFF))
    		 {
    				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    				 while(1);
    			 }

    		 if((B2==0x00 )| (B2==0xFFFF))
    		 {
    				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    				 while(1);
    			 }

    		 if((MB==0x00) | (MB==0xFFFF))
    		 {
    				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    				 while(1);
    			 }

    		 if( (MC==0x00) | (MC==0xFFFF))
    		 {
    				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    				 while(1);
    			 }
    		 if((MD==0x00 )| ( MD==0xFFFF))
    		 {
    				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    				 while(1);
    			 }

}
void BMP180_Get_Uncompansated_Temp(void)
{
	uint8_t wData[1];
	uint8_t rData[2] = {0};
	wData[0] = 0x2E;

	HAL_I2C_Mem_Write(&hi2c1, BMP180_DeviceWriteRegisterAddress, 0xF4, 1, wData, 1, 100000);
	HAL_Delay(5); // Wait 5 MS
	HAL_I2C_Mem_Read(&hi2c1,BMP180_DeviceReadRegisterAddress, 0xF6,1, rData, 2, 100000);
	unCompensatedTemp = (int16_t *) ((rData[0]<<8) | rData[1]);
}
float BMP180_Get_Temp(void)
{
	BMP180_Get_Uncompansated_Temp();
    tX1 = (((int32_t)unCompensatedTemp-(int32_t)AC6) * (int32_t)AC5) / 32768;
    tX2 = ((int32_t)MC * 2048) / (tX1 + (int32_t) MD);
    if((tX1==0 )|(tX2==0))
    	  {
    		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,SET);
    		 while(1);
    	  }

    tB5 = tX1+tX2;
    temperature = (float) (tB5+8) / 16 * 0.1;
    return temperature;
}
void BMP180_Get_Uncompansated_Pressure(void)
{
	uint8_t wData[1];
	uint8_t rData[3] = {0};
	wData[0] = 0x34 | (0x03<<6); // oss value set to ultra high res.

	HAL_I2C_Mem_Write(&hi2c1, BMP180_DeviceWriteRegisterAddress,0xF4, 1, wData, 1, 100000);
	HAL_Delay(26); // Wait 26 MS
	HAL_I2C_Mem_Read(&hi2c1,BMP180_DeviceReadRegisterAddress, 0xF6,1, rData, 3, 100000);

	unCompensatedPressure = (rData[0]<<16 | rData[1] <<8 | rData[2]) >> (8 - (uint8_t)(0x03));

}
float BMP180_Get_Pressure(void)
{
	BMP180_Get_Uncompansated_Pressure();

	pB6 = tB5 - 4000;
	pX1 = (B2 * (pB6 * pB6 / 4096)) / 2048;
    pX2 = AC2 * pB6 / 2048;
    pX3 = pX1 + pX2;
    pB3 = (((AC1 * 4 + pX3) << (uint8_t)(0x03)) + 2) / 4;
    pX1 = AC3 * pB6 / 8192;
    pX2 = (B1 * (pB6 * pB6 / 4096)) / 65536;
    pX3 = ((pX1+pX2) + 2) / 4;
    pB4 = AC4 * (unsigned long)(pX3 + 32768) / 32768;
    pB7 = ((unsigned long)unCompensatedPressure - pB3) * (50000 >> (uint8_t)0x03);
    if(pB7 < 0x80000000)
    { pressure = (pB7 * 2) /pB4;}
    else
    {
    	pressure = (pB7 / pB4) * 2;
    }
    pX1 = (pressure / 256) * (pressure / 256);
    pX1 = (pX1 * 3038) / 65536;
    pX2 = (-7357 * pressure ) / 65536;
    pressure = pressure + (pX1 + pX2 + 3791) / 16; // hPa
    pressure = pressure * 0.00000986923;
    return pressure;
}

