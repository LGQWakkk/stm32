#include "HMC5883.h"
#include "stm32f10x.h" 
#include "Delay.h"


//HMC5883 IIC 
//SCL PA0  SDA PA1
void HMC5883_I2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	//注意这里面的SCL SDA都是开漏输出，在读取数据的时候需要注意原理！
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = HMC5883_SCL_PIN | HMC5883_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, HMC5883_SCL_PIN | HMC5883_SDA_PIN);
}

void HMC5883_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	HMC5883_I2C_Start();
	HMC5883_I2C_SendByte(HMC5883ADDR);
	HMC5883_I2C_ReceiveAck();
	HMC5883_I2C_SendByte(RegAddress);
	HMC5883_I2C_ReceiveAck();
	HMC5883_I2C_SendByte(Data);
	HMC5883_I2C_ReceiveAck();
	HMC5883_I2C_Stop();
}

uint8_t HMC5883_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	HMC5883_I2C_Start();
	HMC5883_I2C_SendByte(HMC5883ADDR);
	HMC5883_I2C_ReceiveAck();
	HMC5883_I2C_SendByte(RegAddress);
	HMC5883_I2C_ReceiveAck();
	
	HMC5883_I2C_Start();
	HMC5883_I2C_SendByte(HMC5883ADDR | 0x01);
	HMC5883_I2C_ReceiveAck();
	Data = HMC5883_I2C_ReceiveByte();
	HMC5883_I2C_SendAck(1);
	HMC5883_I2C_Stop();
	
	return Data;
}

//Measurement Control
void HMC5883_SingleMeasure(void){
	HMC5883_WriteReg(MODE_REG,0x01);
}

void HMC5883_ContinuousMeasure(void){
	HMC5883_WriteReg(MODE_REG,0x00);
}

void HMC5883_IdleMode(void){
	HMC5883_WriteReg(MODE_REG,0x02);
}

void HMC5883_GetSignedData(short* databuf)
{
	uint16_t tmp=0;
	short* p_short=(short*)(&tmp);
	tmp=HMC5883_ReadReg(X_MSB_REG);
	tmp=tmp<<8;
	tmp=tmp|HMC5883_ReadReg(X_LSB_REG);
	databuf[0]=*p_short;
	tmp=0;
	
	tmp=HMC5883_ReadReg(Z_MSB_REG);
	tmp=tmp<<8;
	tmp=tmp|HMC5883_ReadReg(Z_LSB_REG);
	databuf[2]=*p_short;
	tmp=0;
	
	tmp=HMC5883_ReadReg(Y_MSB_REG);
	tmp=tmp<<8;
	tmp=tmp|HMC5883_ReadReg(Y_LSB_REG);
	databuf[1]=*p_short;
	tmp=0;
}

void HMC5883_SetGain(uint8_t gain_value)
{
	HMC5883_WriteReg(CFGB_REG,gain_value);
}

//软件IIC驱动

void HMC5883_I2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(HMC5883_IIC_PORT, HMC5883_SCL_PIN, (BitAction)BitValue);
	Delay_us(10);
}

void HMC5883_I2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(HMC5883_IIC_PORT, HMC5883_SDA_PIN, (BitAction)BitValue);
	Delay_us(10);
}

uint8_t HMC5883_I2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(HMC5883_IIC_PORT, HMC5883_SDA_PIN);
	Delay_us(10);
	return BitValue;
}


void HMC5883_I2C_Start(void)
{
	HMC5883_I2C_W_SDA(1);
	HMC5883_I2C_W_SCL(1);
	HMC5883_I2C_W_SDA(0);
	HMC5883_I2C_W_SCL(0);
}

void HMC5883_I2C_Stop(void)
{
	HMC5883_I2C_W_SDA(0);
	HMC5883_I2C_W_SCL(1);
	HMC5883_I2C_W_SDA(1);
}

void HMC5883_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		HMC5883_I2C_W_SDA(Byte & (0x80 >> i));
		HMC5883_I2C_W_SCL(1);
		HMC5883_I2C_W_SCL(0);
	}
}

uint8_t HMC5883_I2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	HMC5883_I2C_W_SDA(1);
	for (i = 0; i < 8; i ++)
	{
		HMC5883_I2C_W_SCL(1);
		if (HMC5883_I2C_R_SDA() == 1){Byte |= (0x80 >> i);}
		HMC5883_I2C_W_SCL(0);
	}
	return Byte;
}

void HMC5883_I2C_SendAck(uint8_t AckBit)
{
	HMC5883_I2C_W_SDA(AckBit);
	HMC5883_I2C_W_SCL(1);
	HMC5883_I2C_W_SCL(0);
}

uint8_t HMC5883_I2C_ReceiveAck(void)
{
	uint8_t AckBit;
	HMC5883_I2C_W_SDA(1);
	HMC5883_I2C_W_SCL(1);
	AckBit = HMC5883_I2C_R_SDA();
	HMC5883_I2C_W_SCL(0);
	return AckBit;
}

