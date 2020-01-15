#ifndef VARIABLE_H_
#define VARIABLE_H_

char MIR1[100];
char uart[14];
char uart2[50];
char buff[100];

#define SYSCLK      150E6   /* 150MHz */
#define TBCLK       150E6   /* 150MHz */
#define PWMCARRIER  20E3    /* 20kHz */

float32 Cpu_Clk;
float32 Timer_Prd;
Uint16 i = 0, M_i = 0, E_i = 0, D_i = 0;

int a = 0;
int Flash_bit=0;
int flag = 0;
int torque = 0;

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

char RxBuff[16];
char RxBuff2[30];
char Receivedbuff;
int len=0;
int protocol_len=14;
int Enable_num = 1;


// 통신 변수 선언.
Uint16 TimerCount = 0, MotorCount = 0, TimerCount_2 = 0;

float32 usec_delay=100000;

#endif /* VARIABLE_H_ */
