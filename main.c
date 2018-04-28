/**
  ******************************************************************************
  * @file    Dht11.h
  * @author  bokeer
  * @version V1.0
  * @date    11-March-2018
  * @brief   stm32F103RC 
  ******************************************************************************
  * @attention
  *
  * /* 定义芯片类型 STM32F10X_HD 
	*keil options for target c/c++ compiler
	*preprocessor symbols
	*#define STM32F10X_HD
	*
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "Source/TFT_demo.h"
#include "Source/Lcd_Driver.h"
#include "Source/GUI.h"
#include "Source/delay.h"
#include "Source/Dht11.h"

#include <string.h>
#include <stdio.h>


void RTC_Configuration(void);
void DMA_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(void);
void CAN_Configuration(void);
uint8_t can_tx(char *tx);
uint8_t can_rx(char *rx);
void usart_send_string(char *data);

uint8_t state = 0;
uint32_t time = 0;
uint8_t cannum=0;
int main(){
	uint8_t sysclksouce=RCC_GetSYSCLKSource();
	uint8_t res=0;

	/***
	initialize 
	*/
	NVIC_Configuration();
  DMA_Configuration();
	RTC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	CAN_Configuration();
	
	delay_init(72);
	Lcd_Init();
	Lcd_Clear(GRAY0);
	while(1){
		char Txbuffer[5];
		char Rxbuffer[5];
		
		/*if(can_tx(Txbuffer)){
			Gui_DrawFont_Num32(10,10,RED,GRAY0,1);
		}else
		{
			Gui_DrawFont_Num32(10,10,RED,GRAY0,0);
		}
		
		if(can_rx(Rxbuffer)){
			Gui_DrawFont_Num32(50,10,RED,GRAY0,1);
		}else
		{
			Gui_DrawFont_Num32(50,10,RED,GRAY0,0);
		}
		
		delay_ms(100);
		*/
		u8 buffer[5];
		
		float hum;
		float temp;
		char str[5] ;
		if (dht11_read_data(buffer) == 0)
		{
				hum = buffer[0] + buffer[1] / 10;
			
				temp = buffer[2] + buffer[3] / 10;
			for(int i=0;i<5;i++){
				Txbuffer[i]=buffer[i];
			}
			can_tx(Txbuffer);
		  sprintf(str,"%f", temp);
			//p = ecvt(temp,ndig,&dec,&sign);
			Lcd_Clear(GRAY0);
			Gui_DrawFont_GBK16(5,8,BLUE,GRAY0,"temp:");
			Gui_DrawFont_GBK16(45,8,BLUE,GRAY0,str);
			sprintf(str,"%f", hum);
			Gui_DrawFont_GBK16(5,30,BLUE,GRAY0,"hum:");
			Gui_DrawFont_GBK16(45,30,BLUE,GRAY0,str);
		}else{
			Lcd_Clear(GRAY0);
			Gui_DrawFont_GBK16(5,8,BLUE,GRAY0,"temp:");
			Gui_DrawFont_GBK16(45,8,BLUE,GRAY0,"-");
			sprintf(str,"%f", hum);
			Gui_DrawFont_GBK16(5,30,BLUE,GRAY0,"hum:");
			Gui_DrawFont_GBK16(45,30,BLUE,GRAY0,"-");
			
		}
	  if(can_rx(Rxbuffer)){
			for(int i=0;i<5;i++){
				buffer[i]=Rxbuffer[i];
			}
			hum = buffer[0] + buffer[1] / 10;
			
				temp = buffer[2] + buffer[3] / 10;
			sprintf(str,"%f", temp);
			Gui_DrawFont_GBK16(5,50,BLUE,GRAY0,"temp:");
			Gui_DrawFont_GBK16(45,50,BLUE,GRAY0,str);
			sprintf(str,"%f", hum);
			Gui_DrawFont_GBK16(5,70,BLUE,GRAY0,"hum:");
			Gui_DrawFont_GBK16(45,70,BLUE,GRAY0,str);
		}else{
			Gui_DrawFont_GBK16(5,50,BLUE,GRAY0,"temp:");
			Gui_DrawFont_GBK16(45,50,BLUE,GRAY0,"-");
			Gui_DrawFont_GBK16(5,70,BLUE,GRAY0,"hum:");
			Gui_DrawFont_GBK16(45,70,BLUE,GRAY0,"-");
		}
		delay_ms(300);
		//Lcd_Clear(GRAY0);
		
		//Gui_DrawFont_GBK16(16,10,BLUE,GRAY0,"temp:");
		
		//Test_Demo();
		
	}
}

void RTC_Configuration(void){
	/**
  * @brief  Configures the RTC Clock
  * @note   
  * @param 
  */
	/*使能PWR和BKP时钟,RTC依赖PWR和BKP*/ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE); 
	/*使能备份域寄存器*/ 
	PWR_BackupAccessCmd(ENABLE); 
	/*对备份域进行软件复位*/ 
	BKP_DeInit(); 
	/* 使能低速外部时钟 LSE */ 
	RCC_LSEConfig(RCC_LSE_ON); 
	/* 等待LSE起振稳定 */ 
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) 
	{} 
	/* 选择LSE作为 RTC 外设的时钟*/ 
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); 
	/* 使能RTC时钟 */ 
	RCC_RTCCLKCmd(ENABLE); 
	/* 等待RTC寄存器与APB1同步*/ 
	RTC_WaitForSynchro(); 
	/* 等待对RTC的写操作完成*/ 
	RTC_WaitForLastTask(); 
	/* 使能RTC秒中断 */ 
	RTC_ITConfig(RTC_IT_SEC, ENABLE); 
	
	RTC_WaitForLastTask(); 
	/* 设置RT 时钟分频: 使RTC定时周期为1秒  
	RTC 周期 = RTCCLK/RTC_PR = (32.768 KHz)/(PrescalerValue+1)
	RTC 周期 = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
	RTC_SetPrescaler(32767);  
	 
	RTC_WaitForLastTask(); 
 
}
void DMA_Configuration(void){
	DMA_InitTypeDef DMA_InitStructure;
	//DMA设置：
	//设置DMA源：内存地址&串口数据寄存器地址
	//方向：内存-->外设
	//每次传输位：8bit
	//传输大小DMA_BufferSize=SENDBUFF_SIZE
	//地址自增模式：外设地址不增，内存地址自增1
	//DMA模式：一次传输，非循环
	//优先级：中
	DMA_StructInit(&DMA_InitStructure);
	DMA_DeInit(DMA1_Channel4);//串口1的DMA传输通道是通道4
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_BASE;
	//DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendBuff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//外设作为DMA的目的端
	//DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;//传输大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal（只传送一次）, DMA_Mode_Circular （不停地传送）
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//(DMA传送优先级为中等)
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
}
void GPIO_Configuration(void){
		/**
  * @brief  Configures the GPIO
  * @note   
  * @param 
  */
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD  , ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*配置USART1 Tx PA9 Rx PA10 引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*配置remap CAN1 Tx PB9 Rx PB8 引脚*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);
}
void NVIC_Configuration(void){
	/**
  * @brief  Configures the nvic
  * @note   
  * @param 
  */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void USART_Configuration(void){
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	USART_InitTypeDef USART_InitStructure;
	
  USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ClockInitTypeDef USART_ClockInitStructure;
	USART_ClockStructInit(&USART_ClockInitStructure);
	USART_ClockInit(USART1,&USART_ClockInitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1, ENABLE);
}
void usart_send_string(char *data){
    char *p = data;
    while (p < (data + strlen(data)))
    {
			  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
        USART_SendData(USART1, *p++);
		}
		
}
void CAN_Configuration(void){
	uint8_t i=0;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CAN_StructInit(&CAN_InitStructure);
	
	CAN_DeInit(CAN1);
	
	
	CAN_InitStructure.CAN_TTCM=DISABLE;//禁止时间触发通信模式
  CAN_InitStructure.CAN_ABOM=DISABLE;//软件对CAN_MCR寄存器的INRQ位进行置1随后清0后，一旦硬件检测
                                     //到128次11位连续的隐性位，就退出离线状态。
 
  CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过清除CAN_MCR寄存器的SLEEP位，由软件唤醒
 
  CAN_InitStructure.CAN_NART=DISABLE;//DISABLE;CAN报文只被发送1次，不管发送的结果如何（成功、出错或仲裁丢失）
 
  CAN_InitStructure.CAN_RFLM=DISABLE;//在接收溢出时FIFO未被锁定，当接收FIFO的报文未被读出，下一个收到的报文会覆盖原有
                                                            //的报文
 
  CAN_InitStructure.CAN_TXFP=DISABLE;//发送FIFO优先级由报文的标识符来决定
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal; //CAN硬件工作在正常模式
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;//重新同步跳跃宽度1个时间单位
  CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;//时间段1为8个时间单位
  CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;//时间段2为7个时间单位
  CAN_InitStructure.CAN_Prescaler = 9; //(pclk1/((1+8+7)*9)) = 36Mhz/16/9 = 250Kbits设定了一个时间单位的长度9
  CAN_Init(CAN1,&CAN_InitStructure);
 
  /* CAN filter init 过滤器初始化*/
  CAN_FilterInitStructure.CAN_FilterNumber=0;//指定了待初始化的过滤器
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//指定了过滤器将被初始化到的模式为标识符屏蔽位模式
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//给出了过滤器位宽1个32位过滤器
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//用来设定过滤器标识符（32位位宽时为其高段位，16位位宽时为第一个）
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;//用来设定过滤器标识符（32位位宽时为其低段位，16位位宽时为第二个
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//用来设定过滤器屏蔽标识符或者过滤器标识符（32位位宽时为其高段位，16位位宽时为第一个
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;//用来设定过滤器屏蔽标识符或者过滤器标识符（32位位宽时为其低段位，16位位宽时为第二个
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;//设定了指向过滤器的FIFO0
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//使能过滤器
  CAN_FilterInit(&CAN_FilterInitStructure);
  /* CAN FIFO0 message pending interrupt enable */
  //CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);//使能指定的CAN中断
	
	
}
uint8_t can_tx(char *tx){
	CanTxMsg txmsg;
	
	uint8_t TransmitMailbox;
	txmsg.StdId=0x01;
  txmsg.RTR=CAN_RTR_Data;
  txmsg.IDE=CAN_Id_Standard;
  txmsg.DLC=5;
  //txmsg.Data[0]=0x01;
  //txmsg.Data[1]=0x02;
	for(int i=0;i<5;i++){
		txmsg.Data[i]=tx[i];
	}
	TransmitMailbox=CAN_Transmit(CAN1,&txmsg);
	uint16_t i=0;
	
  while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&(i!=0xFFF)){
		i++;
	}
	if(i==0xFFF)return ERROR;
  
	//if (rxmsg.StdId!=0x01) return ERROR;

  //if (rxmsg.IDE!=CAN_ID_STD) return ERROR;

  //if (rxmsg.DLC!=2) return ERROR;

 // if ((rxmsg.Data[0]<<8|rxmsg.Data[1])!=0xCAFE) return ERROR;
  
  return SUCCESS; /* Test Passed */
}
uint8_t can_rx(char *rx){
	CanRxMsg rxmsg;
	
	if(CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;
	
  
  CAN_Receive(CAN1,CAN_FIFO0, &rxmsg);
	for(int i=0;i<5;i++){
		rx[i]=rxmsg.Data[i];
	}
	return SUCCESS; 
}

void RTC_IRQHandler(void){ 
	if (RTC_GetITStatus(RTC_IT_SEC) != RESET) 
	{ 
		/* 清除秒中断标志 */ 
		RTC_ClearITPendingBit(RTC_IT_SEC); 
		RTC_WaitForLastTask(); 
	} 
	state=~state;
	GPIO_WriteBit(GPIOA,GPIO_Pin_8,~state);
	time++;
	
}
void USART1_IRQHandler(void){ 
	int temp=0;
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE)!=RESET){
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
	temp=USART_ReceiveData(USART1);
		
	
		
	char *buff="receive num is :";
	
	usart_send_string(buff);
		
	//while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
	USART_SendData(USART1,temp);
	
		
	char *buff2="\r\n";
	usart_send_string(buff2);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	}	
}
