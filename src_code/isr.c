/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：isr.c
 * 描述         ：中断处理例程
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
//UART0中断调参变量
float  kprxdata,kirxdata,kdrxdata;
int num222=0;
char data_receive[25];
//摄像头采用变量
extern void run();
extern s32 GRAVITY_OFFSET;
extern float GYROSCOPE_ANGLE_RATIO;
extern float g_Speedgoal,AmSpeed;//用于内部
extern float TurnP,TurnD;
void PORTD_IRQHandler()    
{ 
  if(PORTD_ISFR & (1 << 13))//PTD13触发中断
  {
    PORTD_ISFR |=(1 << 13);//写1清中断标志位
 }
}
void PORTC_IRQHandler() 
{
   if(PORTC_ISFR &(1 << 8))//PTC8触发中断
  {
    PORTC_ISFR  |= (1 << 8);//写1清中断标志位
  }
}
/*************************************************************************
*  函数名称：DMA_CH4_Handler
*  功能说明：DMA通道4的中断服务函数
*  参数说明：是采集摄像头数据，本数据位摄像头AD数据，可以采集到300个点。
             设置标志位能够及时搬移。
*************************************************************************/
signed long int DMA_Over_Flg = 0 ;     //行采集完成标志位
volatile u32 dma_int_count;
void DMA_CH4_Handler(void)
{
    DMA_IRQ_CLEAN(DMA_CH4) ;
    DMA_IRQ_DIS(DMA_CH4);
    DMA_DIS(DMA_CH4);
}
void PIT0_IRQHandler(void)//1ms中断
{
  PIT_Flag_Clear(PIT0);//清中断标志位 
  //flag_1ms=1;          //1ms中断标志
  run();          //直立函数
}
/*************************************************************************
*  函数名称：PIT1_IRQHandler
*  功能说明：PIT1定时中断服务函数
*************************************************************************/
void PIT1_IRQHandler(void)
{
   static u8 Time1Count = 0 ;  
   PIT_Flag_Clear(PIT1);       //清中断标志位
   TIME1flag_100ms =1 ;
   if(++Time1Count == 10)
   {
     TIME1flag_1s = 1 ;
     Time1Count = 0 ;
   }  
}
/*************************************************************************
*  函数名称：USART0_IRQHandler
*  功能说明：串口0 中断 接收 服务函数
*************************************************************************/
int fasong1=0;
int num_value=0;
extern s32  g_SpeedControlIntegral; 
//需要配套PID调参上位机使用，在不下载程序的情况下进行调参，方便、节省时间
//可调节直立PD参数、速度PI参数、方向PD参数、陀螺仪零偏及目标速度
//使小车在不停车的情况下自己调整参数，寻找最优参数
void USART0_IRQHandler(void)
{ 
 data_receive[num_value++]=uart_getchar(UART0);//将上位机发送的数据存到数组中
   if(num_value==25&&data_receive[24]=='u') //直立    
  {    
    num_value=0;  
    uart_sendStr (UART0, "1");//发送应答信号，表示接收到了数据
    pid_adjust();
    ANGLE_CONTROL_P=kprxdata;
    ANGLE_CONTROL_D=kdrxdata;
   kprxdata=kirxdata=kdrxdata=0;//进行清零处理，否则会有累加误差
  }
  else if(num_value==25&&data_receive[24]=='v') //速度
  {    
    num_value=0; 
    uart_sendStr (UART0, "2");//发送应答信号，表示接收到了数据
    pid_adjust();
    SpeedKP=kprxdata;
    SpeedKI=kirxdata;
    kprxdata=kirxdata=kdrxdata=0;//进行清零处理，否则会有累加误差
  }
  else if(num_value==25&&data_receive[24]=='w') //方向
  {    
    num_value=0;    
    uart_sendStr (UART0, "3");//发送应答信号，表示接收到了数据
    pid_adjust();    
    TurnP=kprxdata;
    TurnD=kdrxdata;
    kprxdata=kirxdata=kdrxdata=0;//进行清零处理，否则会有累加误差
  }
  else if(num_value==25&&data_receive[24]=='t') //其他
  {    
    num_value=0;    
    uart_sendStr(UART0, "4");//发送应答信号，表示接收到了数据
    pid_adjust();    
    g_Speedgoal=kdrxdata;
    kprxdata=kirxdata=kdrxdata=0;
  }
 if(num_value>=25)//正常情况下一般不会进入此函数
   {
     num_value=0;//超出，清零
     for(int i=0;i<25;i++)
       data_receive[num_value]=0;
   }
}


/////////////////////！！！以下不用看！！！//////////////////////////////////////////
/////////////////////！！！以下不用看！！！//////////////////////////////////////////
/////////////////////！！！以下不用看！！！//////////////////////////////////////////
/*************************************************************************
*  函数名称：SysTick_Handler
*  功能说明：系统滴答定时器中断服务函数
*  修改时间：2012-2-18    已测试
*  备    注：ucos里用得到
*************************************************************************/
void SysTick_Handler(void)
{
    //    OSIntEnter();
    //    OSTimeTick();
    //    OSIntExit();
}
/*************************************************************************
*  函数名称：HardFault_Handler
*  功能说明：硬件上访中断服务函数
*  修改时间：2012-2-4    已测试
*  备    注：可以用LED闪烁来指示发生了硬件上访
*************************************************************************/
void HardFault_Handler(void)
{
    while (1)
    {
        printf("\n****硬件上访错误!!!*****\r\n\n");
    }
}

/*************************************************************************
*  函数名称：PendSV_Handler
*  功能说明：PendSV（可悬起系统调用）中断服务函数
*  修改时间：2012-2-15    已测试
*  备    注：uC/OS用来切换任务
*************************************************************************/
void PendSV_Handler(void)
{
}
/*************************************************************************
*  函数名称：PORTA_IRQHandler
*  功能说明：PORTA端口中断服务函数
*  修改时间：2012-1-25    已测试
*  备    注：引脚号需要自己初始化来清除
*************************************************************************/
void PORTA_IRQHandler()
{
}
/*************************************************************************
*  函数名称：FTM0_IRQHandler
*  功能说明：FTM0输入捕捉中断服务函数
*  修改时间：2012-2-25
*  备    注：引脚号需要根据自己初始化来修改，参考现有的代码添加自己的功能
*************************************************************************/
void FTM0_IRQHandler()
{
}
/*************************************************************************
*  函数名称：DMA_CH4_Handler
*  功能说明：DMA通道4的中断服务函数
*  参数说明：是采集摄像头数据，本数据位摄像头AD数据，可以采集到300个点。
             设置标志位能够及时搬移。
*************************************************************************/

void DMA_CH0_Handler(void)
{
    DMA_IRQ_CLEAN(DMA_CH0);                             //清除通道传输中断标志位    (这样才能再次进入中断)
    DMA_EN(DMA_CH0);                                    //使能通道CHn 硬件请求      (这样才能继续触发DMA传输)
}
/*************************************************************************
*  函数名称：LPT_Handler
*  功能说明：LPT通道4的中断服务函数
*************************************************************************/
volatile u32 LPT_INT_count=0;    
void  LPT_Handler(void)
{
    LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;   //清除LPTMR比较标志
    LPT_INT_count++;                    //中断溢出加1
}
u8 TIME0flag_1ms=0,TIME0flag_5ms=0,TIME0flag_10ms=0,TIME0flag_20ms=0;
