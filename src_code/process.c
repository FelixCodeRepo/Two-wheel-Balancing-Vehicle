#include "common.h"
#include "include.h"
#include "calculation.h"
#include "process.h"
#include"VisualScope.h"
#include "LQ12864.h"

extern s32 GYROSCOPE_turn_OFFSET;//���������  
extern s32 AD_GYRO_turn;//ת��������ADֵ
extern s16 TurnPWMOUT ,LastTurnPWMOUT;//ת��PWM
extern float TurnP,TurnD;
int  leftline, rightline; //������
s32 TurnPosition=0;//ʵ��λ��
s32 TurnMidPosition=64;//ͼ������λ��
void CCD_init (void)
{       
  gpio_init (PORTD ,0, GPO,HIGH);//SI ----PTD0  ���Զ���
  gpio_init (PORTD ,1, GPO,HIGH);//CLK----PTD1  ���Զ���
  adc_init(ADC0, AD9) ;//ad         A0-----PTB1  ���޸ģ���������AD���ܣ�
}

void process()//ͼ��򵥴�����
{
 
}
/*********************************************************** 
�������ƣ�TurnPWM
�������ܣ�����ת��ֵ�ͽǶȴ���������ת��ֵ
��ڲ�����TurnPosition          �������λ��
          TurnMidPosition       �����е�ֵ
          *AngerCout            �Ƕȴ�����AD�ɼ�����
          *AngerValue           �ɼ���ADֵ
***********************************************************/
s16 direction,error=0;
void TurnPWM()//��ð�װת�������ǣ�
{
    direction = TurnPosition - TurnMidPosition ;//���ƫ��
    //TurnPWMOUT = TurnP*direction+TurnD*(AD_GYRO_turn - GYROSCOPE_turn_OFFSET ) ;//��ת��������,��ת�������ǵ������ŷ��򰲷��ˣ���ת����Ӳ�ƽ��
    TurnPWMOUT = TurnP*direction+TurnD*error; //��ת��������  
    error=direction;
}
/*********************************************************** 
�������ƣ�TurnPWMOut
�������ܣ�
��ڲ�����NewspeedPWM      ��ǰ�ĵ�����PWM
          LastspeedPWM     �ϴε�����PWM
          PeriodCount      ƽ������
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
