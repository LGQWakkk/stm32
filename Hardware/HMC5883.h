#ifndef __HMC5883_H
#define __HMC5883_H

#include <stdint.h>

#define HMC5883ADDR 0x3c

#define HMC5883_SCL_PIN GPIO_Pin_0
#define HMC5883_SDA_PIN GPIO_Pin_1
#define HMC5883_IIC_PORT GPIOA


void HMC5883_I2C_Init(void);
void HMC5883_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t HMC5883_ReadReg(uint8_t RegAddress);

void HMC5883_I2C_W_SCL(uint8_t BitValue);
void HMC5883_I2C_W_SDA(uint8_t BitValue);
uint8_t HMC5883_I2C_R_SDA(void);
void HMC5883_I2C_Start(void);
void HMC5883_I2C_Stop(void);
void HMC5883_I2C_SendByte(uint8_t Byte);
uint8_t HMC5883_I2C_ReceiveByte(void);
void HMC5883_I2C_SendAck(uint8_t AckBit);
uint8_t HMC5883_I2C_ReceiveAck(void);

void HMC5883_SingleMeasure(void);
void HMC5883_ContinuousMeasure(void);
void HMC5883_IdleMode(void);

void HMC5883_GetSignedData(short* databuf);
void HMC5883_SetGain(uint8_t gain_value);

//Reg address
#define CFGA_REG 	0x00
#define CFGB_REG 	0x01
#define MODE_REG 	0x02
#define X_MSB_REG 	0x03
#define X_LSB_REG 	0x04
#define Z_MSB_REG	0x05
#define Z_LSB_REG 	0x06
#define Y_MSB_REG 	0x07
#define Y_LSB_REG 	0x08
#define SR_REG 		0x09
#define ID_A_REG 	0x0A
#define ID_B_REG	0x0B
#define ID_C_REG	0x0C

//Gain Value
#define GAIN_1370 	0x00
#define GAIN_1090 	0x20
#define GAIN_820	0x40
#define GAIN_660	0x60
#define GAIN_440	0x80
#define GAIN_390	0xA0
#define GAIN_330	0xC0
#define GAIN_230	0xE0

#endif