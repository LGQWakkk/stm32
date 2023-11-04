#include "MC7RB.h"  
#include "stm32f10x.h"

uint8_t USART_RX_BUF[USART_REC_LEN]; 
uint8_t uart_rx_num=0;
uint8_t startbyte_detected=0;
uint8_t uart_sbus_ok=0;

void uart_init(uint32_t bound){
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
    //USART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
    //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//����λ8bit
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//2��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;//żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}

void USART1_IRQHandler(void)
{
	uint8_t Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//RXNE:�����Ѿ���ȫ����������ݼĴ��������Զ�����
	{
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		if(startbyte_detected==0&&Res==0x0f){
			startbyte_detected=1;
			uart_rx_num=0;
			USART_RX_BUF[uart_rx_num]=Res;
			uart_rx_num++;
		}else if(startbyte_detected==1&&(uart_rx_num<(SBUS_MAX_NUM-1))){
			USART_RX_BUF[uart_rx_num]=Res;
			uart_rx_num++;
		}else if(startbyte_detected==1&&(uart_rx_num==(SBUS_MAX_NUM-1))){
			uart_sbus_ok=1;
			uart_rx_num=0;
		}else{
			uart_rx_num=0;
			uart_sbus_ok=0;
		}  		 
	} 
} 

//���ݽ�������
//�����ڽ��ջ����������ݶ�ȡ��ת����ͨ����ʽ
//ͨ��ֵ����uint16_t chbuf[6];
void Save_6CH(uint16_t* chbuf)
{	//CH1
	uint16_t tmp=0;
	tmp=(USART_RX_BUF[2]&0x07)<<8;
	tmp=tmp|USART_RX_BUF[1];
	chbuf[0]=tmp;
	//CH2
	tmp=0;
	tmp=(USART_RX_BUF[3]&0x3f)<<5;
	tmp=tmp|(USART_RX_BUF[2]>>3);
	chbuf[1]=tmp;
	//CH3
	tmp=0;
	tmp=(USART_RX_BUF[5]&0x01)<<10;
	tmp=tmp|(USART_RX_BUF[4]<<2);
	tmp=tmp|(USART_RX_BUF[3]>>6);
	chbuf[2]=tmp;
	//CH4
	tmp=0;
	tmp=(USART_RX_BUF[6]&0x0f)<<7;
	tmp=tmp|(USART_RX_BUF[5]>>1);
	chbuf[3]=tmp;
	//CH5
	tmp=0;
	tmp=(USART_RX_BUF[7]&0x7f)<<4;
	tmp=tmp|(USART_RX_BUF[6]>>4);
	chbuf[4]=tmp;
	//CH6
	tmp=0;
	tmp=(USART_RX_BUF[9]&0x03)<<9;
	tmp=tmp|(USART_RX_BUF[8]<<1);
	tmp=tmp|(USART_RX_BUF[7]>>7);
	chbuf[5]=tmp;
}

//��մ��ڻ�����
void rxbuf_clear(void)
{
	uint8_t i=0;
	for(i=0;i<25;i++){
		USART_RX_BUF[i]=0;
	}
}

