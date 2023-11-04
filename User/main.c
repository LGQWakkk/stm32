//20231104 @Wakkk
//STM32F103C6T6解析SBUS协议
#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MC7RB.h"  

//项目基本情况：
//使用USART1 PA10作为SBUS信号输入
//使用0.96寸 SSD1306 OLED 进行显示 模拟IIC
//SCL PB8   SDA PB9

extern uint8_t uart_sbus_ok;//SBUS数据就绪(单次接收完毕) 全局变量

int main(void)
{
	uint8_t i=0;
	uint16_t chbuf[6]={0};//6个通道数据存储数组
	OLED_Init();//OLED初始化
	uart_init(100000);//100k baudrate SBUS(UART)初始化
	while(1){
		if(uart_sbus_ok){//数据就绪可以读取
			uart_sbus_ok=0;
			Save_6CH(chbuf);//使用Save_6CH函数向chbuf写入通道数据
			rxbuf_clear();//清空串口缓冲区
			//OLED显示通道数据
			OLED_ShowNum(1,1,chbuf[0],4);
			OLED_ShowNum(2,1,chbuf[1],4);
			OLED_ShowNum(3,1,chbuf[2],4);
			OLED_ShowNum(1,8,chbuf[3],4);
			OLED_ShowNum(2,8,chbuf[4],4);
			OLED_ShowNum(3,8,chbuf[5],4);
		}
	}
}
