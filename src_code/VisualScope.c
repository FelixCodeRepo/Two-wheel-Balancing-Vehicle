#include"VisualScope.h"
#include "include.h"
#include"calculation.h"
#include"isr.h"
extern float kprxdata,kirxdata,kdrxdata ;
extern char data_receive[25]; //收到的数据存到数组中
float OutData[4] = { 0 };
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;
    for (i=0;i<CRC_CNT; i++)
		{      
        CRC_Temp ^= Buf[i];//逐位异或  去取反
        for (j=0;j<8;j++) 
				{
            if (CRC_Temp & 0x01)//如果地位为1
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
// 		Buf[8] = CRC_Temp & 0x00ff;  //将第9个数据赋上CRC的低字节
//     Buf[9] = CRC_Temp >>8;//第10数据上是CRC的高字节
    return(CRC_Temp);
}

void OutPut_Data(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
  {   
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i];  
  } 
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  } 
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256; 
  for(i=0;i<10;i++)
  uart_putchar(UART0,databuf[i]);
}

//PID调参用到的函数
float power(float n)
{
  float value_z=1;
  if(n>=0)
  for(int i=0;i<n;i++)
    value_z*=10;
  else 
    for(int i=0;i>n;i--)
    value_z/=10;
  return value_z;
}
void pid_adjust()
{
  for(int i=0;i<=7;i++)
  {
    kprxdata+=data_receive[i]*(power(3-i));
  }
   for(int i=8;i<=15;i++)
  {
    kirxdata+=data_receive[i]*(power(11-i));
  }
  for(int i=16;i<=23;i++)
  {
    kdrxdata+=data_receive[i]*(power(19-i));
  }
   
}
/*
s32 row=0;
char image_tuxiang[10000];
void SD()
{
    FIL fdst;  //文件
    FATFS fs;  //文件系统
    uint32 size;
    int res;
    //int i,j;
    char *str = "山东交通学院";
    //char *huiche_flag="\nAA";
   // u8 buff[BUFF_SIZE];
   row=0;
  //  for(size = 0; size < BUFF_SIZE; size++)
    {
    //  buff[size] = 0;
    }
    f_mount(0, &fs);//SD卡系统初始化
    res = f_open(&fdst, "0:/FireDemo.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);  //打开文件，如果没有就创建，带读写打开
    if( res == FR_DISK_ERR)
    {
       // printf( "\n没插入SD卡？？\n" );
        return;
    }
    else if ( res == FR_OK )
    {
        //printf( "\n文件打开成功 \n" );
    }
    else
    {
       // printf("\n返回值异常");
    }
    f_puts((image_tuxiang), &fdst); //往文件里写入字符串str   
    if(size > BUFF_SIZE)size = BUFF_SIZE;   //防止溢出
  //  f_lseek(&fdst, 0);                      //把指针指向文件顶部
   // f_read (&fdst, buff, size, (UINT *)&sizetmp);   //读取
    f_close(&fdst);                         //关闭文件
}*/    