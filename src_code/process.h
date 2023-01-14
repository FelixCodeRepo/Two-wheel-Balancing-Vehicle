#ifndef  process_H
#define  process_H  
void StartIntegration(void) ;   
void ImageCapture(uint8_t * ImageData) ;
void SendHex(uint8_t hex) ;
void SamplingDelay(void) ;
void CCD_init(void) ;
void SendImageData(uint8_t * ImageData) ;
void CCD_init (); 
s16 TurnPWMOut(s16 NewturnPWM ,s16 LastturnPWM,s16 PeriodCount);
void TurnPWM();
void process();
void erzhihua();
void tuxiang();
void fasongerzhihua();
#endif