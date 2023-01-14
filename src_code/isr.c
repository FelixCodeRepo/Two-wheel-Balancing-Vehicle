/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���       ��isr.c
 * ����         ���жϴ�������
**********************************************************************************/
#include "common.h"
#include "include.h"
#include  "isr.h"
#include "process.h"
#include"calculation.h"
#include"VisualScope.h"
#include "LQ12864.h"
u8 TIME1flag_1s   = 0 ;
u8 TIME1flag_100ms = 0 ;
u8 flag_1ms=0; 
//UART0�жϵ��α���
float  kprxdata,kirxdata,kdrxdata;
int num222=0;
char data_receive[25];
//����ͷ���ñ���
extern void run();
extern s32 GRAVITY_OFFSET;
extern float GYROSCOPE_ANGLE_RATIO;
extern float g_Speedgoal,AmSpeed;//�����ڲ�
extern float TurnP,TurnD;
void PORTD_IRQHandler()    
{ 
  if(PORTD_ISFR & (1 << 13))//PTD13�����ж�
  {
    PORTD_ISFR |=(1 << 13);//д1���жϱ�־λ
 }
}
void PORTC_IRQHandler() 
{
   if(PORTC_ISFR &(1 << 8))//PTC8�����ж�
  {
    PORTC_ISFR  |= (1 << 8);//д1���жϱ�־λ
  }
}
/*************************************************************************
*  �������ƣ�DMA_CH4_Handler
*  ����˵����DMAͨ��4���жϷ�����
*  ����˵�����ǲɼ�����ͷ���ݣ�������λ����ͷAD���ݣ����Բɼ���300���㡣
             ���ñ�־λ�ܹ���ʱ���ơ�
*************************************************************************/
signed long int DMA_Over_Flg = 0 ;     //�вɼ���ɱ�־λ
volatile u32 dma_int_count;
void DMA_CH4_Handler(void)
{
    DMA_IRQ_CLEAN(DMA_CH4) ;
    DMA_IRQ_DIS(DMA_CH4);
    DMA_DIS(DMA_CH4);
}
void PIT0_IRQHandler(void)//1ms�ж�
{
  PIT_Flag_Clear(PIT0);//���жϱ�־λ 
  //flag_1ms=1;          //1ms�жϱ�־
  run();          //ֱ������
}
/*************************************************************************
*  �������ƣ�PIT1_IRQHandler
*  ����˵����PIT1��ʱ�жϷ�����
*************************************************************************/
void PIT1_IRQHandler(void)
{
   static u8 Time1Count = 0 ;  
   PIT_Flag_Clear(PIT1);       //���жϱ�־λ
   TIME1flag_100ms =1 ;
   if(++Time1Count == 10)
   {
     TIME1flag_1s = 1 ;
     Time1Count = 0 ;
   }  
}
/*************************************************************************
*  �������ƣ�USART0_IRQHandler
*  ����˵��������0 �ж� ���� ������
*************************************************************************/
int fasong1=0;
int num_value=0;
extern s32  g_SpeedControlIntegral; 
//��Ҫ����PID������λ��ʹ�ã��ڲ����س��������½��е��Σ����㡢��ʡʱ��
//�ɵ���ֱ��PD�������ٶ�PI����������PD��������������ƫ��Ŀ���ٶ�
//ʹС���ڲ�ͣ����������Լ�����������Ѱ�����Ų���
void USART0_IRQHandler(void)
{ 
 data_receive[num_value++]=uart_getchar(UART0);//����λ�����͵����ݴ浽������
   if(num_value==25&&data_receive[24]=='u') //ֱ��    
  {    
    num_value=0;  
    uart_sendStr (UART0, "1");//����Ӧ���źţ���ʾ���յ�������
    pid_adjust();
    ANGLE_CONTROL_P=kprxdata;
    ANGLE_CONTROL_D=kdrxdata;
   kprxdata=kirxdata=kdrxdata=0;//�������㴦����������ۼ����
  }
  else if(num_value==25&&data_receive[24]=='v') //�ٶ�
  {    
    num_value=0; 
    uart_sendStr (UART0, "2");//����Ӧ���źţ���ʾ���յ�������
    pid_adjust();
    SpeedKP=kprxdata;
    SpeedKI=kirxdata;
    kprxdata=kirxdata=kdrxdata=0;//�������㴦����������ۼ����
  }
  else if(num_value==25&&data_receive[24]=='w') //����
  {    
    num_value=0;    
    uart_sendStr (UART0, "3");//����Ӧ���źţ���ʾ���յ�������
    pid_adjust();    
    TurnP=kprxdata;
    TurnD=kdrxdata;
    kprxdata=kirxdata=kdrxdata=0;//�������㴦����������ۼ����
  }
  else if(num_value==25&&data_receive[24]=='t') //����
  {    
    num_value=0;    
    uart_sendStr(UART0, "4");//����Ӧ���źţ���ʾ���յ�������
    pid_adjust();    
    g_Speedgoal=kdrxdata;
    kprxdata=kirxdata=kdrxdata=0;
  }
 if(num_value>=25)//���������һ�㲻�����˺���
   {
     num_value=0;//����������
     for(int i=0;i<25;i++)
       data_receive[num_value]=0;
   }
}


/////////////////////���������²��ÿ�������//////////////////////////////////////////
/////////////////////���������²��ÿ�������//////////////////////////////////////////
/////////////////////���������²��ÿ�������//////////////////////////////////////////
/*************************************************************************
*  �������ƣ�SysTick_Handler
*  ����˵����ϵͳ�δ�ʱ���жϷ�����
*  �޸�ʱ�䣺2012-2-18    �Ѳ���
*  ��    ע��ucos���õõ�
*************************************************************************/
void SysTick_Handler(void)
{
    //    OSIntEnter();
    //    OSTimeTick();
    //    OSIntExit();
}
/*************************************************************************
*  �������ƣ�HardFault_Handler
*  ����˵����Ӳ���Ϸ��жϷ�����
*  �޸�ʱ�䣺2012-2-4    �Ѳ���
*  ��    ע��������LED��˸��ָʾ������Ӳ���Ϸ�
*************************************************************************/
void HardFault_Handler(void)
{
    while (1)
    {
        printf("\n****Ӳ���Ϸô���!!!*****\r\n\n");
    }
}

/*************************************************************************
*  �������ƣ�PendSV_Handler
*  ����˵����PendSV��������ϵͳ���ã��жϷ�����
*  �޸�ʱ�䣺2012-2-15    �Ѳ���
*  ��    ע��uC/OS�����л�����
*************************************************************************/
void PendSV_Handler(void)
{
}
/*************************************************************************
*  �������ƣ�PORTA_IRQHandler
*  ����˵����PORTA�˿��жϷ�����
*  �޸�ʱ�䣺2012-1-25    �Ѳ���
*  ��    ע�����ź���Ҫ�Լ���ʼ�������
*************************************************************************/
void PORTA_IRQHandler()
{
}
/*************************************************************************
*  �������ƣ�FTM0_IRQHandler
*  ����˵����FTM0���벶׽�жϷ�����
*  �޸�ʱ�䣺2012-2-25
*  ��    ע�����ź���Ҫ�����Լ���ʼ�����޸ģ��ο����еĴ�������Լ��Ĺ���
*************************************************************************/
void FTM0_IRQHandler()
{
}
/*************************************************************************
*  �������ƣ�DMA_CH4_Handler
*  ����˵����DMAͨ��4���жϷ�����
*  ����˵�����ǲɼ�����ͷ���ݣ�������λ����ͷAD���ݣ����Բɼ���300���㡣
             ���ñ�־λ�ܹ���ʱ���ơ�
*************************************************************************/

void DMA_CH0_Handler(void)
{
    DMA_IRQ_CLEAN(DMA_CH0);                             //���ͨ�������жϱ�־λ    (���������ٴν����ж�)
    DMA_EN(DMA_CH0);                                    //ʹ��ͨ��CHn Ӳ������      (�������ܼ�������DMA����)
}
/*************************************************************************
*  �������ƣ�LPT_Handler
*  ����˵����LPTͨ��4���жϷ�����
*************************************************************************/
volatile u32 LPT_INT_count=0;    
void  LPT_Handler(void)
{
    LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;   //���LPTMR�Ƚϱ�־
    LPT_INT_count++;                    //�ж������1
}
u8 TIME0flag_1ms=0,TIME0flag_5ms=0,TIME0flag_10ms=0,TIME0flag_20ms=0;
