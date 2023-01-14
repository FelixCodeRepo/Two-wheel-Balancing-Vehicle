/*************************************************************
             7620摄像头整车例程
            引脚配置
摄像头   
          数据口    d0--d7
          行中断    b3
          场中断    a5 
          pclk      e6
串口    波特率256000
         rx      c15 
         tx      c14
*///////////////////////////////////////////////
#include "common.h"
#include "image.h"
#include "HW_GPIO.h"
extern int image_get,a;    //图像采集完成标志
void main (void)
{    
    gpio_init();        //初始化
    uart_init();
    dma_init();
    ftm_init();
    
    DisableInterrupts;
    enable_irq(PORTB_IRQn);    //行
    enable_irq(PORTA_IRQn);    //场
    EnableInterrupts;

    
    
    while(1)
    {
      
      
      if(image_get==1) //
      {
        //发送图像到上位机 ，让小车正常跑时，此函数应该注释掉
//        image_get=0;   //发送图像到上位机时，此处应该注释掉
//        chuli_image();
//        send_image(); 
        
        
        a=0;
      
    } 
}
}