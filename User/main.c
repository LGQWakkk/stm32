//20231104 @Wakkk
//STM32F103C6T6����SBUSЭ��
#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MC7RB.h"  

//��Ŀ���������
//ʹ��USART1 PA10��ΪSBUS�ź�����
//ʹ��0.96�� SSD1306 OLED ������ʾ ģ��IIC
//SCL PB8   SDA PB9

extern uint8_t uart_sbus_ok;//SBUS���ݾ���(���ν������) ȫ�ֱ���

int main(void)
{
	uint8_t i=0;
	uint16_t chbuf[6]={0};//6��ͨ�����ݴ洢����
	OLED_Init();//OLED��ʼ��
	uart_init(100000);//100k baudrate SBUS(UART)��ʼ��
	while(1){
		if(uart_sbus_ok){//���ݾ������Զ�ȡ
			uart_sbus_ok=0;
			Save_6CH(chbuf);//ʹ��Save_6CH������chbufд��ͨ������
			rxbuf_clear();//��մ��ڻ�����
			//OLED��ʾͨ������
			OLED_ShowNum(1,1,chbuf[0],4);
			OLED_ShowNum(2,1,chbuf[1],4);
			OLED_ShowNum(3,1,chbuf[2],4);
			OLED_ShowNum(1,8,chbuf[3],4);
			OLED_ShowNum(2,8,chbuf[4],4);
			OLED_ShowNum(3,8,chbuf[5],4);
		}
	}
}
