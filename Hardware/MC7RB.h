#ifndef __MC7RB_H
#define __MC7RB_H
#include <stdint.h>

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define SBUS_MAX_NUM 			25  	//SBUSÿ�δ������25�ֽ�

extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART_RX_STA;         		//����״̬���	
void uart_init(uint32_t bound);

void Save_6CH(uint16_t* chbuf);
void rxbuf_clear(void);


#endif

