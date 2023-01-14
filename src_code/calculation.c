#include "common.h"
#include "include.h"
#include "calculation.h"
#include"VisualScope.h"
#include "LQ12864.h"
s16 g_nDirectionControlFlag = 0,g_nWaitCarStandCount = 0,g_nCarControlFlag = 0 ;
s16 g_nSpeedControlFlag = 0,g_nAngleControlFlag = 0,g_fAngleControlOut = 0;
double fGyroscopeAngleIntegral ;//陀螺仪倾角积分
double g_fCarAngle,g_fGyroscopeAngleSpeed,g_fGravityAngle,fDeltaValue ;  //g_fGravityAngle加速度计倾角   g_fCarAngle陀螺仪倾角
s16 g_nLeftMotorPulse,g_nRightMotorPulse,g_nLeftMotorPulseSigma,g_nRightMotorPulseSigma,lm,rm;
extern int sudu_xianshi;
extern float OutData[4];
u8 Stop_flag=0;//停车标志
s32  leftMotorPWM ,RingtMotorPWM ;
s32 LeftMotorOut ,RinghtMotorOut ;
float fValue;   
extern s32 GYROSCOPE_OFFSET;
//直立部分调节下面带有//*的参数
//正常的话只需要调节以下两句的参数，其他几个可以不调
s32 GRAVITY_OFFSET=1824;//*    //加速度计平衡参数，改变大小可改变直立时的平衡位置，这个需要车直立时把平衡系数填入
//也可以自己试，值大或值小，小车会往某个方向跑，达不到平衡位置，可以自己去试，也可以使用IAR的watch功能，看下
//小车处于平衡位置时加速度计的值，然后填在这里！一定要是平衡位置！可以自己用手扶车去试
float GYROSCOPE_ANGLE_RATIO=0.23;//*     //陀螺仪:"归一化比例系数"  根据图像调
#define GYROSCOPE_ANGLE_SIGMA_FREQUENCY	 200   //陀螺仪积分时间  
#define GRAVITY_ADJUST_TIME_CONSTANT	1 //*   //加速度计时间补偿系数  3
#define GRAVITY_ANGLE_RATIO  0.08//*           //加速度计Z轴:"归一化比例系数" B-A=C,180/C=
void AngleCalculate(void) 
{
  //加速度计倾角 =(加速度计AD值 - 加速度计Z轴偏移量)*加速度计Z轴"归一化比例系数" 0.1345
    g_fGravityAngle = ((AD_ACC_Z -GRAVITY_OFFSET  ) * GRAVITY_ANGLE_RATIO);
              //GRAVITY_ANGLE_RATIO = 90°/(Acce_Z_POS_45 - Acce_Z_NAG_45) = 0.09879 放大1000倍
    g_fGyroscopeAngleSpeed = (AD_GYRO - GYROSCOPE_OFFSET) *GYROSCOPE_ANGLE_RATIO;// 0.1870  
    //陀螺仪倾角速度=     ( 陀螺仪AD值-陀螺仪偏移量)*陀螺仪"归一化比例系数"    0.1832//  减小能消过冲0.1363 
    //陀螺仪倾角 = 陀螺仪倾角积分                                      
    g_fCarAngle = fGyroscopeAngleIntegral;      
    fDeltaValue = (g_fGravityAngle - g_fCarAngle)/GRAVITY_ADJUST_TIME_CONSTANT;//补偿量(1-4的数字，小数也可以)
    //角度的误差信号=(            -           ) /tg      加速度计时间补偿系数 （减小能使融合角度曲线下降）
//tg越大，角度输出跟踪z轴输出越慢，但可以有效抑制重力加速度计上的噪声，
//tg过大，就会放大陀螺仪输出误差2 7  
                                                         //200      //陀螺仪积分时间 (1000 / CONTROL_PERIOD)  // 提升波形的斜率
    fGyroscopeAngleIntegral += (g_fGyroscopeAngleSpeed + fDeltaValue) /GYROSCOPE_ANGLE_SIGMA_FREQUENCY; //  19;
//进行积分                                                            积分时间小，受加速度计影响大                      
}
/*********************************************************** 
函数名称：AngleControl
***********************************************************/
s32 AngleControl(void) 
{
   if(g_nAngleControlFlag == 0) 
   {
     g_fAngleControlOut = 0;
     return g_fAngleControlOut ;
   }   
   fValue = g_fCarAngle * ANGLE_CONTROL_P + g_fGyroscopeAngleSpeed * ANGLE_CONTROL_D;     //角度控制系数       
   if(fValue > ANGLE_CONTROL_OUT_MAX)      fValue = ANGLE_CONTROL_OUT_MAX;
   else if(fValue < ANGLE_CONTROL_OUT_MIN) fValue = ANGLE_CONTROL_OUT_MIN;
   g_fAngleControlOut = fValue; 
   return g_fAngleControlOut ;
}
/*********************************************************** 
函数名称：AAangPWMOut
函数功能：
入口参数：NewspeedPWM      当前的电机输出PWM
          LastspeedPWM     上次电机输出PWM
          PeriodCount      平滑周期
***********************************************************/
#define AANGPERIODFAV  (5) 
s32 AAangPWMOut(s32 NewAangPWM ,s32 LastAangPWM,s32 PeriodCount)
{
    s32  AangPWMfav,AangOUT ;
    AangPWMfav = NewAangPWM - LastAangPWM ;
    AangOUT = AangPWMfav *(PeriodCount+1)/AANGPERIODFAV + LastAangPWM ;
    return AangOUT ; 
}
/*********************************************************** 
函数名称：MotorSpeedOut
函数功能：
入口参数：TurnPosition          计算黑线位置
          TurnMidPosition       黑线中点值
          *AngerCout            角度传感器AD采集次数
          *AngerValue           采集的AD值 
***********************************************************/
s32 MotorSpeedOut(s32 anglePWM ,s32 speedPWM ,s32 turnPWM)
{
  s32 speedAvPWM ,turnAvPWM ;
  speedAvPWM = speedPWM  ;
  turnAvPWM = turnPWM/TURNTIMCOUNT ;
  LeftMotorOut = anglePWM -speedAvPWM - turnAvPWM ;//直立、速度、方向三者融合，其中速度与直立的符合是相反的，参考官方方案
  RinghtMotorOut = anglePWM - speedAvPWM + turnAvPWM ;
  
   if(LeftMotorOut > MOTOR_OUT_MAX)//简单限幅
    LeftMotorOut = MOTOR_OUT_MAX ;
  else if(LeftMotorOut < MOTOR_OUT_MIN)
    LeftMotorOut = MOTOR_OUT_MIN ; 
  if(RinghtMotorOut > MOTOR_OUT_MAX)
    RinghtMotorOut = MOTOR_OUT_MAX ;
  else if(RinghtMotorOut < MOTOR_OUT_MIN)
    RinghtMotorOut = MOTOR_OUT_MIN ;
   
  if(LeftMotorOut >= 0)
  {  
     leftMotorPWM = LeftMotorOut ;
     leftMotorPWM += 0 ;
     FTM_CnV_REG(FTMx[FTM0], CH2) = leftMotorPWM;
     gpio_set(PORTE, 6, 0);
     gpio_set(PORTE, 7, 1);
  }
  else 
  {
     leftMotorPWM = 0-LeftMotorOut ;
     leftMotorPWM += 0;
     FTM_CnV_REG(FTMx[FTM0], CH2) = leftMotorPWM;
     gpio_set(PORTE, 6, 1);
     gpio_set(PORTE, 7, 0);
  }
   if(RinghtMotorOut >= 0)
   {
     RingtMotorPWM = RinghtMotorOut;
     RingtMotorPWM += 0;
     FTM_CnV_REG(FTMx[FTM0], CH3) = RingtMotorPWM;
     gpio_set(PORTE, 10, 0);
     gpio_set(PORTE, 11, 1);
   }
   else
   {
     RingtMotorPWM = 0-RinghtMotorOut ;
     RingtMotorPWM += 0;    
     FTM_CnV_REG(FTMx[FTM0], CH3) = RingtMotorPWM;
     gpio_set(PORTE, 10, 1);
     gpio_set(PORTE, 11, 0);
   }
      
   return ((LeftMotorOut +RinghtMotorOut)/2);
}
/* ===================================================================
** SpeedPID
   输入：speedCount采集车速，AmSpeed 目标车速  ；  
   输出 ：SpeedPWMOUT  计算车速 ；
** ===================================================================*/  
s32  DSpeed ,LastSpeedCut0, g_SpeedControlIntegral=0,SpeedDifference0=0;;      
float SpeedPWMKP ,SpeedPWMKI ;
void SpeedPID()//都正 前冲  都负 后冲  结论 
{    
   LastSpeedCut0 = (g_nLeftMotorPulseSigma+g_nRightMotorPulseSigma)/2 ;//将左右轮的速度取平均
   g_nLeftMotorPulseSigma=g_nRightMotorPulseSigma=0;//然后清零
   if(g_nSpeedControlFlag == 0) //如果加了停车遥控，有时命令停车时还不挺，需要将积分项清零
   {
      SpeedPWMKI = 0 ;
      SpeedPWMKP = 0 ;
      LastSpeedCut0 = DSpeed ;
      g_SpeedControlIntegral=0;//积分项
    }  
    DSpeed =g_Speedgoal;//内部速度
    SpeedDifference0 =LastSpeedCut0-DSpeed  ;//与实际速度做差
     
    SpeedPWMKP = SpeedKP*SpeedDifference0;
    SpeedPWMKI = SpeedKI*SpeedDifference0;
    g_SpeedControlIntegral+=SpeedPWMKI; 
    if(g_SpeedControlIntegral  >=Pre_SPEED_OUT_MAX_I)  
    {
      g_SpeedControlIntegral=Pre_SPEED_OUT_MAX_I; 
    } 
    if(g_SpeedControlIntegral <= Pre_SPEED_OUT_MIN_I) 
    { 
      g_SpeedControlIntegral=Pre_SPEED_OUT_MIN_I;
    } 
    g_SpeedControlOutOld=g_SpeedControlOutNew; 
    g_SpeedControlOutNew=SpeedPWMKP+g_SpeedControlIntegral; 
    if(SpeedPWMKP>KPPLUSMAX)//30000
    {
      SpeedPWMKP = KPPLUSMAX;
    }else if (SpeedPWMKP <KPNEGATIVEMAX)
    {
      SpeedPWMKP = KPNEGATIVEMAX;                       
    } 
    if(SpeedPWMKI > KIPLUSMAX)//30000
    {
      SpeedPWMKI = KIPLUSMAX;
    } else if(SpeedPWMKI < KINEGATIVEMAX)
    {
      SpeedPWMKI = KINEGATIVEMAX;
    } 
}
/*********************************************************** 
函数名称：SpeedPWMOut
函数功能：
入口参数：NewspeedPWM      当前的电机输出PWM
          LastspeedPWM     上次电机输出PWM
          PeriodCount      平滑周期
出口参数：无 
备 注： 
***********************************************************/
#define SPEEDPERIODFAV  100 //速度控制周期
s32 SpeedPWMOut(s32 NewspeedPWM ,s32 LastspeedPWM,s32 PeriodCount)
{
    s32  speedPWMfav,SpeedOUT ;
    speedPWMfav = NewspeedPWM - LastspeedPWM ;
    SpeedOUT = speedPWMfav *(PeriodCount+1)/SPEEDPERIODFAV + LastspeedPWM ;   
    return SpeedOUT ; 
}
void GetMotorPulse(void) //正交解码，可以读出正负
{      
   g_nRightMotorPulse = FTM2_CNT  ; //可直接读取正负值，变量类型要为s16类型！！！ 
   FTM2_CNT = 0; 
   g_nLeftMotorPulse = FTM1_CNT  ;
   FTM1_CNT = 0; 
   if(g_nLeftMotorPulse>0)//让两个轮子向前一致，否则就会有一正一负的情况	
     g_nLeftMotorPulse = -g_nLeftMotorPulse; 
   else 
     g_nLeftMotorPulse = -g_nLeftMotorPulse; 
   g_nRightMotorPulseSigma+=g_nRightMotorPulse;
   g_nLeftMotorPulseSigma +=g_nLeftMotorPulse;
}

void display_lvbo_jiaodu()//发送到虚拟示波器
{
  OutData[0] = g_fGravityAngle ;   //g_fGravityAngle加速度计倾角
  OutData[1] = g_fCarAngle;        //g_fCarAngle陀螺仪倾角
  OutData[2]=g_fGyroscopeAngleSpeed;   //陀螺仪倾角速度
 // OutData[3]=angle_dot;
  OutPut_Data();
}

void uart(int fs)
{
 if(fs<0)
  {
    fs=-fs;
    uart_sendStr(UART0,"-");  
  }
  uart_putchar(UART0,fs/1000+0x30);
  uart_putchar(UART0,fs%1000/100+0x30);
  uart_putchar(UART0,fs%100/10+0x30);
  uart_putchar(UART0,fs%10+0x30);
  uart_sendStr(UART0," ");
}
void display_suducaiji()
{
  if(sudu_xianshi==1)
  {
    sudu_xianshi=0;
    uart_sendStr(UART0,"L:");
    uart(lm);
    uart_sendStr(UART0,"R:");
    uart(rm);
    uart_putchar(UART0,'\n');
  }
}
void display_OLED()//1.2mssuoyou  now:290us
{ 
    LCD_P6x8Str(1,1,"JSU:");
    Dis_num(28,1,GRAVITY_OFFSET);//加计零偏 定值
   // LCD_P6x8Str(1,2,"A_Z:");
    //Dis_num(28,2,AD_ACC_Z);
    LCD_P6x8Str(64,1,"LP:");
    Dis_num(96,1,GYROSCOPE_OFFSET);//陀螺仪零偏  自检
   // LCD_P6x8Str(64,2,"GYRO:");
   // Dis_num(96,2,AD_GYRO);
    
    //LCD_P6x8Str(1,3,"TLxs:");
   // Dis_float(28,3,GYROSCOPE_ANGLE_RATIO);
   
   // LCD_P6x8Str(1,7,"RMB:");
   // Dis_num(28,7,g_nRightMotorPulseSigma);
   // LCD_P6x8Str(64,7,"LMB:");
   // Dis_num(96,7,g_nLeftMotorPulseSigma);
   //  LCD_CLS();
}

/*********************************************************** 
函数名称：SEEPINP_init
函数功能：速度脉冲计数器初始化程序
入口参数：
出口参数：无 
备 注： 
***********************************************************/
void FTM_init(void)
{
    /* FTM_PWM_init(FTM0 , CH0, 10000,0);                              mod = (clk_hz >> 16 ) / freq ;
                                                                       for(sc_ps = 0; (mod >> sc_ps) >= 1; sc_ps++);
                                                                         if(freq < 1000)sc_ps++;   
                                                                          mod = (clk_hz >> sc_ps) / freq;  //频率设置因子,clk_hz = 25000000   */
    //FTM_PWM_init(FTM0 , CH1, 10000,0);     //电机占空比设置初始化   MOD =3125 ;  占空比 = duty /(MOD +1 ) ;FTM_CnV_REG(FTMx[ftmn], ch) = cv;
    FTM_PWM_init(FTM0 , CH2, 10000,0);     //电机占空比设置初始化   MOD =3125 ;  占空比 = duty /(MOD +1 ) ;FTM_CnV_REG(FTMx[ftmn], ch) = cv;
    FTM_PWM_init(FTM0 , CH3, 10000,0);     //电机占空比设置初始化   MOD =3125 ;  占空比 = duty /(MOD +1 ) ;FTM_CnV_REG(FTMx[ftmn], ch) = cv;
    
    gpio_init(PORTE , 6, GPO, 0);
    gpio_init(PORTE , 7, GPO, 0);
    gpio_init(PORTE , 10, GPO, 0);
    gpio_init(PORTE , 11, GPO, 0);
}
/*********************************************************** 
函数名称：AngleAcceleration_init   
函数功能：加速度角度传感器初始化程序
入口参数：
出口参数：无 
备 注： 
***********************************************************/
void AngleAcceleration_init()
{
   adc_init(ADC1, AD9) ;     //PTB1    z轴 10 11 16
   adc_init(ADC0, AD12) ;     //PTB2   直立陀螺仪
   adc_init(ADC1, AD11) ;     //PTB5  转向陀螺仪
}
void CarControlStart() 
{
  CAR_CONTROL_SET;
  ANGLE_CONTROL_START;
  SPEED_CONTROL_START;
  DIRECTION_CONTROL_START;
}
//------------------------------------------------------------------------------
void CarControlStop() 
{
  CAR_CONTROL_CLEAR;
  ANGLE_CONTROL_STOP;
  SPEED_CONTROL_STOP;
  DIRECTION_CONTROL_STOP;
}
//------------------------------------------------------------------------------
void WaitCarStand() 
{
  if(g_nCarControlFlag == 1) return;//260
  if(g_fCarAngle > CAR_STAND_ANGLE_MIN && g_fCarAngle < CAR_STAND_ANGLE_MAX &&  
     g_fGravityAngle > CAR_STAND_ANGLE_MIN &&g_fGravityAngle < CAR_STAND_ANGLE_MAX)    
  {
    CarControlStart();         
  } 	
}
//------------------------------------------------------------------------------	
void CheckCarStand() 
{
  if(g_nCarControlFlag == 0) return;       
  if(g_fCarAngle >= CAR_FAILURE_ANGLE_MAX ||
     g_fCarAngle <= CAR_FAILURE_ANGLE_MIN) 
  {
      CarControlStop();
      return;
  }
}
void Checkcarstate()
{
  TIME0flag_1ms++;//每ms检测一次
  if(TIME0flag_1ms==1)
  {
    TIME0flag_1ms=0;
    if((g_fCarAngle != 0 )&& (g_fGravityAngle != 0 ))//开启停止判断 
    {
       WaitCarStand();
       CheckCarStand();
    }
   // if(PTD9_IN==1)//无线遥控  
    {
    //   Stop_flag=1;
    }
    if(Stop_flag==1)//该标志可用于停车
    {    
     CarControlStop();
    }
  }
}
