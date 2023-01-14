#include "include.h"
#include "calculation.h"
#include "VisualScope.h"
#include "process.h"
#include "LQ12864.h"

/*******************************************************************************************************
接口说明：PWM：PTC1，PTC2，PTC3，PTC4
         直立陀螺仪：PTB1
         转向陀螺仪：PTB5
         加速度计：PTB2
         
        串口：RX--PTC3   TX--PTC4
        编码器：正交解码左轮：A相---PTA8  B相---PTA9
                正交解码右轮：A相---PTA10 B相---PTA11
带有的是需要修改的参数  关于下面的P、I、D最好先清零！！！即ANGLE_CONTROL_P=0，ANGLE_CONTROL_D=0
SpeedKP=0，SpeedKI=0，TurnP=0，TurnD=0
直立滤波参数的调节在calculation.c中
注意：串口发送函数在不需要使用时要屏蔽掉，否则无法直立！VisualScope虚拟示波器使用时要以兼容方式打开，win95！
**********************************************************************************************************/
//PID参数
/*float  ANGLE_CONTROL_P=110;     //直立P
float  ANGLE_CONTROL_D=3.1;     //直立D
float SpeedKP =25;              //速度P
float SpeedKI =2;              //速度I
float TurnP=94;                //方向P
float TurnD=8.4;                //方向D
float AmSpeed=0;  //目标速度 用于外部       设置速度时在这里设置
float g_Speedgoal=0;//用于内部 自加速使用，这个不用动！！！*/
float  ANGLE_CONTROL_P=0;//*     //直立P
float  ANGLE_CONTROL_D=0;//*     //直立D
float SpeedKP =0;//*              //速度P
float SpeedKI =0;//*               //速度I
float TurnP=0;//*                 //方向P
float TurnD=0;//*                //方向D
float AmSpeed=0;//*   //目标速度 用于外部       设置速度时在这里设置
float g_Speedgoal=0;//用于内部 自加速使用，这个不用动！！！
//时间标志位
extern u8 TIME0flag_5ms,TIME0flag_10ms,TIME0flag_20ms;
extern u8 TIME1flag_100ms,flag_1ms ;
extern u8 TIME1flag_1s;  //PT1口1s定时标志位
s32 ATimeCount=0;//100ms进入标志  由20个5ms构成，速度PID时使用
s32 TimeCount=0; //1ms中断标志
//角度传感器
s32 GYROSCOPE_OFFSET,GYROSCOPE_turn_OFFSET;//陀螺仪静止时的零点 
s32 AD_ACC_Z;//加速度计的Z轴
s32 AD_GYRO;//平衡陀螺仪
s32 AD_GYRO_turn;//转向陀螺仪
s32 AAngPWM=0,LastAAngPWM=0,AAngPeriodCount=0,MotorAAngPWM=0;
//速度变量
float g_SpeedControlOutNew,g_SpeedControlOutOld;
s16 SpeedPeriodCount=0 ;
extern s16 g_nLeftMotorPulseSigma,g_nRightMotorPulseSigma,lm,rm;
int sudu_xianshi=0;
s32 MotorSpeedPWM=0;
s32 PWMout;
int start_flag=0,stop_jiasu=0;
//CCD变量
//int ccd_count=0;
//s16 TurnPeriodCount=0;
//s32 MotorTurnPWM=0;
//s16 TurnPWMOUT=0;
//s16 LastTurnPWMOUT=0;
void run();//直立主函数
void qibu();//起步不能一下把速度加上去，速度要慢慢加
void main()
{
   DisableInterrupts; //禁止总中断 
   uart_init(UART0, 9600);//初始化UART1，RX--PTC3  TX--PTC4
   AngleAcceleration_init() ;//AD初始化
   FTM1_QUAD_Iint();//正交解码测速  A相---PTA8  B相---PTA9
   FTM2_QUAD_Iint();//正交解码测速  A相---PTA10 B相---PTA11
  // oled_init();//oled初始化
   //CCD_init (); //CCD初始化 
   //gpio_init (PORTA , 17, GPO,HIGH); //程序运行灯
   pit_init_ms(PIT0, 1);  //初始化PIT0，定时时间为： 1ms
   pit_init_ms(PIT1, 100);//初始化PIT1，定时时间为： 100ms 
   FTM_init(); //PWM初始化
   
   GYROSCOPE_OFFSET = ad_ave(ADC1, AD9, ADC_12bit, 200);      //开机自动采集零偏       直立
   GYROSCOPE_turn_OFFSET = ad_ave(ADC1, AD11, ADC_12bit, 200); //开机自动采集零偏       转向
   
   EnableInterrupts;//开总中断  
   uart_irq_EN(UART1);
   while(1)
   { 
      //display_lvbo_jiaodu();//用于看滤波图像时调用
      //display_suducaiji(); //显示速度脉冲的
   } 
}
void run()//直立函数 在isr.c中的1ms中断中调用  PIT0_IRQHandler
{    
  TimeCount++ ; 
  SpeedPeriodCount++;
  //TurnPeriodCount ++ ;
  AAngPeriodCount ++ ;
  //MotorTurnPWM  = TurnPWMOut(TurnPWMOUT,LastTurnPWMOUT,TurnPeriodCount) ;
  MotorSpeedPWM = SpeedPWMOut(g_SpeedControlOutNew ,g_SpeedControlOutOld,SpeedPeriodCount);  
  MotorAAngPWM  = AAangPWMOut(AAngPWM, LastAAngPWM, AAngPeriodCount);  
  Checkcarstate();//开启停止判断
  if(TimeCount>=5)//读速度  5us
  {
    TimeCount=0;
    GetMotorPulse();//读速度脉冲
  }  
  else if(TimeCount == 1)//读取AD值
  {       
     //AD_GYRO_turn =ad_ave(ADC1,AD11,ADC_12bit,7);       //陀螺仪  转向
     AD_ACC_Z = ad_ave(ADC0,AD12,ADC_12bit,6);           //加速度计
     AD_GYRO = ad_ave(ADC1,AD9, ADC_12bit,4);            //陀螺仪   直立
  }
  else if(TimeCount == 2)//20us 直立滤波，控制
  { 
    AAngPeriodCount  = 0;
    AngleCalculate();//计算加速值和角度值
    LastAAngPWM = AAngPWM ;      
    AAngPWM = AngleControl();     //计算平衡电机速度 
   // PWMout=MotorSpeedOut(AAngPWM,MotorSpeedPWM,MotorTurnPWM);//电机输出  调速度控制时使用
    PWMout=MotorSpeedOut(AAngPWM,0,0);//电机输出  刚开始用的时候可以把MotorSpeedPWM和MotorTurnPWM都设为0 
                                      //先调直立，再加速度MotorSpeedPWM，最后加方向MotorTurnPWM
  }
  else if(TimeCount == 3)//5us  速度PI调节 
  {
     ATimeCount ++ ;
     if(ATimeCount >= 20)//20*5=100ms进行一次速度PID调节
     {
       ATimeCount=0;
       sudu_xianshi=1;//采集完速度的标志，用于向上位机发送速度
       lm=g_nLeftMotorPulseSigma;
       rm=g_nRightMotorPulseSigma;
       SpeedPID() ; 
       SpeedPeriodCount = 0 ;
       qibu();//起步  作用：不可以一下就把速度加上，要缓慢加
     }
   } 
  /*else if(TimeCount == 4)// 采集与处理
  {
    ccd_count++;
    if(ccd_count>=4)//20ms进一次
    {
        ccd_count=0;
      //  process();//处理函数
        LastTurnPWMOUT = TurnPWMOUT ;
        TurnPWM() ;
        TurnPeriodCount = 0 ;
    }
  }  
  if(TIME1flag_1s == 1)//程序运行灯
  {
    TIME1flag_1s = 0 ;
    PTA17_OUT = ~PTA17_OUT ; 
  }
*/
}


void qibu()//这一部分认真看一下，也可以根据自己的想法改一下
{
  if(AmSpeed!=0)//在目标速度不为0时才运行 
  {
   start_flag++;
   if(start_flag<2)//静止2S才出发  可根据规则修改
     g_Speedgoal=0; 
   else if(start_flag>=2)
   {
     if(g_Speedgoal<AmSpeed&&stop_jiasu==0)//然后逐渐加速 
     { 
         g_Speedgoal+=30;//单次加速值，值大加速能力强，比赛前可设两组值，一组快速加速，用于应对起步时是长
                         //直道的情况，另一组为慢加速，用于应对起步时就是弯道的情况
     } 
     else if(g_Speedgoal>=AmSpeed&&stop_jiasu==0) //内部速度高于设置速度，停止大幅度加速，进行小幅度调整加速
     { 
         g_Speedgoal=AmSpeed; 
         stop_jiasu=1;
     }                                
     start_flag=30;  //终止静止2s计数 
     if(stop_jiasu==1)        //车模之后的目标速度调整主要在此函数中
     {
       if(g_Speedgoal<AmSpeed)//然后逐渐加速 
       { 
         g_Speedgoal+=10;//
       } 
        else if(g_Speedgoal>=AmSpeed) 
       { 
         g_Speedgoal=AmSpeed; 
       }    
     }
   }
  }
}
