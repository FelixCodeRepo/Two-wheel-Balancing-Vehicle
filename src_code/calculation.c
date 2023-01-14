#include "common.h"
#include "include.h"
#include "calculation.h"
#include"VisualScope.h"
#include "LQ12864.h"
s16 g_nDirectionControlFlag = 0,g_nWaitCarStandCount = 0,g_nCarControlFlag = 0 ;
s16 g_nSpeedControlFlag = 0,g_nAngleControlFlag = 0,g_fAngleControlOut = 0;
double fGyroscopeAngleIntegral ;//��������ǻ���
double g_fCarAngle,g_fGyroscopeAngleSpeed,g_fGravityAngle,fDeltaValue ;  //g_fGravityAngle���ٶȼ����   g_fCarAngle���������
s16 g_nLeftMotorPulse,g_nRightMotorPulse,g_nLeftMotorPulseSigma,g_nRightMotorPulseSigma,lm,rm;
extern int sudu_xianshi;
extern float OutData[4];
u8 Stop_flag=0;//ͣ����־
s32  leftMotorPWM ,RingtMotorPWM ;
s32 LeftMotorOut ,RinghtMotorOut ;
float fValue;   
extern s32 GYROSCOPE_OFFSET;
//ֱ�����ֵ����������//*�Ĳ���
//�����Ļ�ֻ��Ҫ������������Ĳ����������������Բ���
s32 GRAVITY_OFFSET=1824;//*    //���ٶȼ�ƽ��������ı��С�ɸı�ֱ��ʱ��ƽ��λ�ã������Ҫ��ֱ��ʱ��ƽ��ϵ������
//Ҳ�����Լ��ԣ�ֵ���ֵС��С������ĳ�������ܣ��ﲻ��ƽ��λ�ã������Լ�ȥ�ԣ�Ҳ����ʹ��IAR��watch���ܣ�����
//С������ƽ��λ��ʱ���ٶȼƵ�ֵ��Ȼ���������һ��Ҫ��ƽ��λ�ã������Լ����ַ���ȥ��
float GYROSCOPE_ANGLE_RATIO=0.23;//*     //������:"��һ������ϵ��"  ����ͼ���
#define GYROSCOPE_ANGLE_SIGMA_FREQUENCY	 200   //�����ǻ���ʱ��  
#define GRAVITY_ADJUST_TIME_CONSTANT	1 //*   //���ٶȼ�ʱ�䲹��ϵ��  3
#define GRAVITY_ANGLE_RATIO  0.08//*           //���ٶȼ�Z��:"��һ������ϵ��" B-A=C,180/C=
void AngleCalculate(void) 
{
  //���ٶȼ���� =(���ٶȼ�ADֵ - ���ٶȼ�Z��ƫ����)*���ٶȼ�Z��"��һ������ϵ��" 0.1345
    g_fGravityAngle = ((AD_ACC_Z -GRAVITY_OFFSET  ) * GRAVITY_ANGLE_RATIO);
              //GRAVITY_ANGLE_RATIO = 90��/(Acce_Z_POS_45 - Acce_Z_NAG_45) = 0.09879 �Ŵ�1000��
    g_fGyroscopeAngleSpeed = (AD_GYRO - GYROSCOPE_OFFSET) *GYROSCOPE_ANGLE_RATIO;// 0.1870  
    //����������ٶ�=     ( ������ADֵ-������ƫ����)*������"��һ������ϵ��"    0.1832//  ��С��������0.1363 
    //��������� = ��������ǻ���                                      
    g_fCarAngle = fGyroscopeAngleIntegral;      
    fDeltaValue = (g_fGravityAngle - g_fCarAngle)/GRAVITY_ADJUST_TIME_CONSTANT;//������(1-4�����֣�С��Ҳ����)
    //�Ƕȵ�����ź�=(            -           ) /tg      ���ٶȼ�ʱ�䲹��ϵ�� ����С��ʹ�ںϽǶ������½���
//tgԽ�󣬽Ƕ��������z�����Խ������������Ч�����������ٶȼ��ϵ�������
//tg���󣬾ͻ�Ŵ�������������2 7  
                                                         //200      //�����ǻ���ʱ�� (1000 / CONTROL_PERIOD)  // �������ε�б��
    fGyroscopeAngleIntegral += (g_fGyroscopeAngleSpeed + fDeltaValue) /GYROSCOPE_ANGLE_SIGMA_FREQUENCY; //  19;
//���л���                                                            ����ʱ��С���ܼ��ٶȼ�Ӱ���                      
}
/*********************************************************** 
�������ƣ�AngleControl
***********************************************************/
s32 AngleControl(void) 
{
   if(g_nAngleControlFlag == 0) 
   {
     g_fAngleControlOut = 0;
     return g_fAngleControlOut ;
   }   
   fValue = g_fCarAngle * ANGLE_CONTROL_P + g_fGyroscopeAngleSpeed * ANGLE_CONTROL_D;     //�Ƕȿ���ϵ��       
   if(fValue > ANGLE_CONTROL_OUT_MAX)      fValue = ANGLE_CONTROL_OUT_MAX;
   else if(fValue < ANGLE_CONTROL_OUT_MIN) fValue = ANGLE_CONTROL_OUT_MIN;
   g_fAngleControlOut = fValue; 
   return g_fAngleControlOut ;
}
/*********************************************************** 
�������ƣ�AAangPWMOut
�������ܣ�
��ڲ�����NewspeedPWM      ��ǰ�ĵ�����PWM
          LastspeedPWM     �ϴε�����PWM
          PeriodCount      ƽ������
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
�������ƣ�MotorSpeedOut
�������ܣ�
��ڲ�����TurnPosition          �������λ��
          TurnMidPosition       �����е�ֵ
          *AngerCout            �Ƕȴ�����AD�ɼ�����
          *AngerValue           �ɼ���ADֵ 
***********************************************************/
s32 MotorSpeedOut(s32 anglePWM ,s32 speedPWM ,s32 turnPWM)
{
  s32 speedAvPWM ,turnAvPWM ;
  speedAvPWM = speedPWM  ;
  turnAvPWM = turnPWM/TURNTIMCOUNT ;
  LeftMotorOut = anglePWM -speedAvPWM - turnAvPWM ;//ֱ�����ٶȡ����������ںϣ������ٶ���ֱ���ķ������෴�ģ��ο��ٷ�����
  RinghtMotorOut = anglePWM - speedAvPWM + turnAvPWM ;
  
   if(LeftMotorOut > MOTOR_OUT_MAX)//���޷�
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
   ���룺speedCount�ɼ����٣�AmSpeed Ŀ�공��  ��  
   ��� ��SpeedPWMOUT  ���㳵�� ��
** ===================================================================*/  
s32  DSpeed ,LastSpeedCut0, g_SpeedControlIntegral=0,SpeedDifference0=0;;      
float SpeedPWMKP ,SpeedPWMKI ;
void SpeedPID()//���� ǰ��  ���� ���  ���� 
{    
   LastSpeedCut0 = (g_nLeftMotorPulseSigma+g_nRightMotorPulseSigma)/2 ;//�������ֵ��ٶ�ȡƽ��
   g_nLeftMotorPulseSigma=g_nRightMotorPulseSigma=0;//Ȼ������
   if(g_nSpeedControlFlag == 0) //�������ͣ��ң�أ���ʱ����ͣ��ʱ����ͦ����Ҫ������������
   {
      SpeedPWMKI = 0 ;
      SpeedPWMKP = 0 ;
      LastSpeedCut0 = DSpeed ;
      g_SpeedControlIntegral=0;//������
    }  
    DSpeed =g_Speedgoal;//�ڲ��ٶ�
    SpeedDifference0 =LastSpeedCut0-DSpeed  ;//��ʵ���ٶ�����
     
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
�������ƣ�SpeedPWMOut
�������ܣ�
��ڲ�����NewspeedPWM      ��ǰ�ĵ�����PWM
          LastspeedPWM     �ϴε�����PWM
          PeriodCount      ƽ������
���ڲ������� 
�� ע�� 
***********************************************************/
#define SPEEDPERIODFAV  100 //�ٶȿ�������
s32 SpeedPWMOut(s32 NewspeedPWM ,s32 LastspeedPWM,s32 PeriodCount)
{
    s32  speedPWMfav,SpeedOUT ;
    speedPWMfav = NewspeedPWM - LastspeedPWM ;
    SpeedOUT = speedPWMfav *(PeriodCount+1)/SPEEDPERIODFAV + LastspeedPWM ;   
    return SpeedOUT ; 
}
void GetMotorPulse(void) //�������룬���Զ�������
{      
   g_nRightMotorPulse = FTM2_CNT  ; //��ֱ�Ӷ�ȡ����ֵ����������ҪΪs16���ͣ����� 
   FTM2_CNT = 0; 
   g_nLeftMotorPulse = FTM1_CNT  ;
   FTM1_CNT = 0; 
   if(g_nLeftMotorPulse>0)//������������ǰһ�£�����ͻ���һ��һ�������	
     g_nLeftMotorPulse = -g_nLeftMotorPulse; 
   else 
     g_nLeftMotorPulse = -g_nLeftMotorPulse; 
   g_nRightMotorPulseSigma+=g_nRightMotorPulse;
   g_nLeftMotorPulseSigma +=g_nLeftMotorPulse;
}

void display_lvbo_jiaodu()//���͵�����ʾ����
{
  OutData[0] = g_fGravityAngle ;   //g_fGravityAngle���ٶȼ����
  OutData[1] = g_fCarAngle;        //g_fCarAngle���������
  OutData[2]=g_fGyroscopeAngleSpeed;   //����������ٶ�
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
    Dis_num(28,1,GRAVITY_OFFSET);//�Ӽ���ƫ ��ֵ
   // LCD_P6x8Str(1,2,"A_Z:");
    //Dis_num(28,2,AD_ACC_Z);
    LCD_P6x8Str(64,1,"LP:");
    Dis_num(96,1,GYROSCOPE_OFFSET);//��������ƫ  �Լ�
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
�������ƣ�SEEPINP_init
�������ܣ��ٶ������������ʼ������
��ڲ�����
���ڲ������� 
�� ע�� 
***********************************************************/
void FTM_init(void)
{
    /* FTM_PWM_init(FTM0 , CH0, 10000,0);                              mod = (clk_hz >> 16 ) / freq ;
                                                                       for(sc_ps = 0; (mod >> sc_ps) >= 1; sc_ps++);
                                                                         if(freq < 1000)sc_ps++;   
                                                                          mod = (clk_hz >> sc_ps) / freq;  //Ƶ����������,clk_hz = 25000000   */
    //FTM_PWM_init(FTM0 , CH1, 10000,0);     //���ռ�ձ����ó�ʼ��   MOD =3125 ;  ռ�ձ� = duty /(MOD +1 ) ;FTM_CnV_REG(FTMx[ftmn], ch) = cv;
    FTM_PWM_init(FTM0 , CH2, 10000,0);     //���ռ�ձ����ó�ʼ��   MOD =3125 ;  ռ�ձ� = duty /(MOD +1 ) ;FTM_CnV_REG(FTMx[ftmn], ch) = cv;
    FTM_PWM_init(FTM0 , CH3, 10000,0);     //���ռ�ձ����ó�ʼ��   MOD =3125 ;  ռ�ձ� = duty /(MOD +1 ) ;FTM_CnV_REG(FTMx[ftmn], ch) = cv;
    
    gpio_init(PORTE , 6, GPO, 0);
    gpio_init(PORTE , 7, GPO, 0);
    gpio_init(PORTE , 10, GPO, 0);
    gpio_init(PORTE , 11, GPO, 0);
}
/*********************************************************** 
�������ƣ�AngleAcceleration_init   
�������ܣ����ٶȽǶȴ�������ʼ������
��ڲ�����
���ڲ������� 
�� ע�� 
***********************************************************/
void AngleAcceleration_init()
{
   adc_init(ADC1, AD9) ;     //PTB1    z�� 10 11 16
   adc_init(ADC0, AD12) ;     //PTB2   ֱ��������
   adc_init(ADC1, AD11) ;     //PTB5  ת��������
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
  TIME0flag_1ms++;//ÿms���һ��
  if(TIME0flag_1ms==1)
  {
    TIME0flag_1ms=0;
    if((g_fCarAngle != 0 )&& (g_fGravityAngle != 0 ))//����ֹͣ�ж� 
    {
       WaitCarStand();
       CheckCarStand();
    }
   // if(PTD9_IN==1)//����ң��  
    {
    //   Stop_flag=1;
    }
    if(Stop_flag==1)//�ñ�־������ͣ��
    {    
     CarControlStop();
    }
  }
}
