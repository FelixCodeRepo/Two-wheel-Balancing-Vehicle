#include "include.h"
#include "calculation.h"
#include "VisualScope.h"
#include "process.h"
#include "LQ12864.h"

/*******************************************************************************************************
�ӿ�˵����PWM��PTC1��PTC2��PTC3��PTC4
         ֱ�������ǣ�PTB1
         ת�������ǣ�PTB5
         ���ٶȼƣ�PTB2
         
        ���ڣ�RX--PTC3   TX--PTC4
        �������������������֣�A��---PTA8  B��---PTA9
                �����������֣�A��---PTA10 B��---PTA11
���е�����Ҫ�޸ĵĲ���  ���������P��I��D��������㣡������ANGLE_CONTROL_P=0��ANGLE_CONTROL_D=0
SpeedKP=0��SpeedKI=0��TurnP=0��TurnD=0
ֱ���˲������ĵ�����calculation.c��
ע�⣺���ڷ��ͺ����ڲ���Ҫʹ��ʱҪ���ε��������޷�ֱ����VisualScope����ʾ����ʹ��ʱҪ�Լ��ݷ�ʽ�򿪣�win95��
**********************************************************************************************************/
//PID����
/*float  ANGLE_CONTROL_P=110;     //ֱ��P
float  ANGLE_CONTROL_D=3.1;     //ֱ��D
float SpeedKP =25;              //�ٶ�P
float SpeedKI =2;              //�ٶ�I
float TurnP=94;                //����P
float TurnD=8.4;                //����D
float AmSpeed=0;  //Ŀ���ٶ� �����ⲿ       �����ٶ�ʱ����������
float g_Speedgoal=0;//�����ڲ� �Լ���ʹ�ã�������ö�������*/
float  ANGLE_CONTROL_P=0;//*     //ֱ��P
float  ANGLE_CONTROL_D=0;//*     //ֱ��D
float SpeedKP =0;//*              //�ٶ�P
float SpeedKI =0;//*               //�ٶ�I
float TurnP=0;//*                 //����P
float TurnD=0;//*                //����D
float AmSpeed=0;//*   //Ŀ���ٶ� �����ⲿ       �����ٶ�ʱ����������
float g_Speedgoal=0;//�����ڲ� �Լ���ʹ�ã�������ö�������
//ʱ���־λ
extern u8 TIME0flag_5ms,TIME0flag_10ms,TIME0flag_20ms;
extern u8 TIME1flag_100ms,flag_1ms ;
extern u8 TIME1flag_1s;  //PT1��1s��ʱ��־λ
s32 ATimeCount=0;//100ms�����־  ��20��5ms���ɣ��ٶ�PIDʱʹ��
s32 TimeCount=0; //1ms�жϱ�־
//�Ƕȴ�����
s32 GYROSCOPE_OFFSET,GYROSCOPE_turn_OFFSET;//�����Ǿ�ֹʱ����� 
s32 AD_ACC_Z;//���ٶȼƵ�Z��
s32 AD_GYRO;//ƽ��������
s32 AD_GYRO_turn;//ת��������
s32 AAngPWM=0,LastAAngPWM=0,AAngPeriodCount=0,MotorAAngPWM=0;
//�ٶȱ���
float g_SpeedControlOutNew,g_SpeedControlOutOld;
s16 SpeedPeriodCount=0 ;
extern s16 g_nLeftMotorPulseSigma,g_nRightMotorPulseSigma,lm,rm;
int sudu_xianshi=0;
s32 MotorSpeedPWM=0;
s32 PWMout;
int start_flag=0,stop_jiasu=0;
//CCD����
//int ccd_count=0;
//s16 TurnPeriodCount=0;
//s32 MotorTurnPWM=0;
//s16 TurnPWMOUT=0;
//s16 LastTurnPWMOUT=0;
void run();//ֱ��������
void qibu();//�𲽲���һ�°��ٶȼ���ȥ���ٶ�Ҫ������
void main()
{
   DisableInterrupts; //��ֹ���ж� 
   uart_init(UART0, 9600);//��ʼ��UART1��RX--PTC3  TX--PTC4
   AngleAcceleration_init() ;//AD��ʼ��
   FTM1_QUAD_Iint();//�����������  A��---PTA8  B��---PTA9
   FTM2_QUAD_Iint();//�����������  A��---PTA10 B��---PTA11
  // oled_init();//oled��ʼ��
   //CCD_init (); //CCD��ʼ�� 
   //gpio_init (PORTA , 17, GPO,HIGH); //�������е�
   pit_init_ms(PIT0, 1);  //��ʼ��PIT0����ʱʱ��Ϊ�� 1ms
   pit_init_ms(PIT1, 100);//��ʼ��PIT1����ʱʱ��Ϊ�� 100ms 
   FTM_init(); //PWM��ʼ��
   
   GYROSCOPE_OFFSET = ad_ave(ADC1, AD9, ADC_12bit, 200);      //�����Զ��ɼ���ƫ       ֱ��
   GYROSCOPE_turn_OFFSET = ad_ave(ADC1, AD11, ADC_12bit, 200); //�����Զ��ɼ���ƫ       ת��
   
   EnableInterrupts;//�����ж�  
   uart_irq_EN(UART1);
   while(1)
   { 
      //display_lvbo_jiaodu();//���ڿ��˲�ͼ��ʱ����
      //display_suducaiji(); //��ʾ�ٶ������
   } 
}
void run()//ֱ������ ��isr.c�е�1ms�ж��е���  PIT0_IRQHandler
{    
  TimeCount++ ; 
  SpeedPeriodCount++;
  //TurnPeriodCount ++ ;
  AAngPeriodCount ++ ;
  //MotorTurnPWM  = TurnPWMOut(TurnPWMOUT,LastTurnPWMOUT,TurnPeriodCount) ;
  MotorSpeedPWM = SpeedPWMOut(g_SpeedControlOutNew ,g_SpeedControlOutOld,SpeedPeriodCount);  
  MotorAAngPWM  = AAangPWMOut(AAngPWM, LastAAngPWM, AAngPeriodCount);  
  Checkcarstate();//����ֹͣ�ж�
  if(TimeCount>=5)//���ٶ�  5us
  {
    TimeCount=0;
    GetMotorPulse();//���ٶ�����
  }  
  else if(TimeCount == 1)//��ȡADֵ
  {       
     //AD_GYRO_turn =ad_ave(ADC1,AD11,ADC_12bit,7);       //������  ת��
     AD_ACC_Z = ad_ave(ADC0,AD12,ADC_12bit,6);           //���ٶȼ�
     AD_GYRO = ad_ave(ADC1,AD9, ADC_12bit,4);            //������   ֱ��
  }
  else if(TimeCount == 2)//20us ֱ���˲�������
  { 
    AAngPeriodCount  = 0;
    AngleCalculate();//�������ֵ�ͽǶ�ֵ
    LastAAngPWM = AAngPWM ;      
    AAngPWM = AngleControl();     //����ƽ�����ٶ� 
   // PWMout=MotorSpeedOut(AAngPWM,MotorSpeedPWM,MotorTurnPWM);//������  ���ٶȿ���ʱʹ��
    PWMout=MotorSpeedOut(AAngPWM,0,0);//������  �տ�ʼ�õ�ʱ����԰�MotorSpeedPWM��MotorTurnPWM����Ϊ0 
                                      //�ȵ�ֱ�����ټ��ٶ�MotorSpeedPWM�����ӷ���MotorTurnPWM
  }
  else if(TimeCount == 3)//5us  �ٶ�PI���� 
  {
     ATimeCount ++ ;
     if(ATimeCount >= 20)//20*5=100ms����һ���ٶ�PID����
     {
       ATimeCount=0;
       sudu_xianshi=1;//�ɼ����ٶȵı�־����������λ�������ٶ�
       lm=g_nLeftMotorPulseSigma;
       rm=g_nRightMotorPulseSigma;
       SpeedPID() ; 
       SpeedPeriodCount = 0 ;
       qibu();//��  ���ã�������һ�¾Ͱ��ٶȼ��ϣ�Ҫ������
     }
   } 
  /*else if(TimeCount == 4)// �ɼ��봦��
  {
    ccd_count++;
    if(ccd_count>=4)//20ms��һ��
    {
        ccd_count=0;
      //  process();//������
        LastTurnPWMOUT = TurnPWMOUT ;
        TurnPWM() ;
        TurnPeriodCount = 0 ;
    }
  }  
  if(TIME1flag_1s == 1)//�������е�
  {
    TIME1flag_1s = 0 ;
    PTA17_OUT = ~PTA17_OUT ; 
  }
*/
}


void qibu()//��һ�������濴һ�£�Ҳ���Ը����Լ����뷨��һ��
{
  if(AmSpeed!=0)//��Ŀ���ٶȲ�Ϊ0ʱ������ 
  {
   start_flag++;
   if(start_flag<2)//��ֹ2S�ų���  �ɸ��ݹ����޸�
     g_Speedgoal=0; 
   else if(start_flag>=2)
   {
     if(g_Speedgoal<AmSpeed&&stop_jiasu==0)//Ȼ���𽥼��� 
     { 
         g_Speedgoal+=30;//���μ���ֵ��ֵ���������ǿ������ǰ��������ֵ��һ����ټ��٣�����Ӧ����ʱ�ǳ�
                         //ֱ�����������һ��Ϊ�����٣�����Ӧ����ʱ������������
     } 
     else if(g_Speedgoal>=AmSpeed&&stop_jiasu==0) //�ڲ��ٶȸ��������ٶȣ�ֹͣ����ȼ��٣�����С���ȵ�������
     { 
         g_Speedgoal=AmSpeed; 
         stop_jiasu=1;
     }                                
     start_flag=30;  //��ֹ��ֹ2s���� 
     if(stop_jiasu==1)        //��ģ֮���Ŀ���ٶȵ�����Ҫ�ڴ˺�����
     {
       if(g_Speedgoal<AmSpeed)//Ȼ���𽥼��� 
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
