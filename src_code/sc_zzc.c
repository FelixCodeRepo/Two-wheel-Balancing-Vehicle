/*************************************************************
             7620����ͷ��������
            ��������
����ͷ   
          ���ݿ�    d0--d7
          ���ж�    b3
          ���ж�    a5 
          pclk      e6
����    ������256000
         rx      c15 
         tx      c14
*///////////////////////////////////////////////
#include "common.h"
#include "image.h"
#include "HW_GPIO.h"
extern int image_get,a;    //ͼ��ɼ���ɱ�־
void main (void)
{    
    gpio_init();        //��ʼ��
    uart_init();
    dma_init();
    ftm_init();
    
    DisableInterrupts;
    enable_irq(PORTB_IRQn);    //��
    enable_irq(PORTA_IRQn);    //��
    EnableInterrupts;

    
    
    while(1)
    {
      
      
      if(image_get==1) //
      {
        //����ͼ����λ�� ����С��������ʱ���˺���Ӧ��ע�͵�
//        image_get=0;   //����ͼ����λ��ʱ���˴�Ӧ��ע�͵�
//        chuli_image();
//        send_image(); 
        
        
        a=0;
      
    } 
}
}