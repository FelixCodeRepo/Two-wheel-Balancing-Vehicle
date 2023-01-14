#include "common.h"
#include "include.h"
#include "calculation.h"
#include "process.h"
#include"VisualScope.h"
#include "LQ12864.h"

extern s32 GYROSCOPE_turn_OFFSET;//陀螺仪零点  
extern s32 AD_GYRO_turn;//转向陀螺仪AD值
extern s16 TurnPWMOUT ,LastTurnPWMOUT;//转向PWM
extern float TurnP,TurnD;
int  leftline, rightline; //左右线
s32 TurnPosition=0;//实际位置
s32 TurnMidPosition=64;//图像中心位置
void CCD_init (void)
{       
  gpio_init (PORTD ,0, GPO,HIGH);//SI ----PTD0  可自定义
  gpio_init (PORTD ,1, GPO,HIGH);//CLK----PTD1  可自定义
  adc_init(ADC0, AD9) ;//ad         A0-----PTB1  可修改，但必须有AD功能！
}

void process()//图像简单处理函数
{
 
}
/*********************************************************** 
函数名称：TurnPWM
函数功能：根据转向值和角度传感器计算转向值
入口参数：TurnPosition          计算黑线位置
          TurnMidPosition       黑线中点值
          *AngerCout            角度传感器AD采集次数
          *AngerValue           采集的AD值
***********************************************************/
s16 direction,error=0;
void TurnPWM()//最好安装转向陀螺仪！
{
    direction = TurnPosition - TurnMidPosition ;//求出偏差
    //TurnPWMOUT = TurnP*direction+TurnD*(AD_GYRO_turn - GYROSCOPE_turn_OFFSET ) ;//有转向陀螺仪,如转向陀螺仪的正负号方向安反了，车转弯更加不平稳
    TurnPWMOUT = TurnP*direction+TurnD*error; //无转向陀螺仪  
    error=direction;
}
/*********************************************************** 
函数名称：TurnPWMOut
函数功能：
入口参数：NewspeedPWM      当前的电机输出PWM
          LastspeedPWM     上次电机输出PWM
          PeriodCount      平滑周期
***********************************************************/ 
#define TURNPERIODFAV  (20) 
s16 TurnPWMOut(s16 NewturnPWM ,s16 LastturnPWM,s16 PeriodCount)
{
  s16  turnPWMfav ;
  s16  turnOUT ;
  turnPWMfav = NewturnPWM - LastturnPWM ;
  turnOUT = turnPWMfav *(PeriodCount)/TURNPERIODFAV + LastturnPWM ;
  return turnOUT ; 
}
