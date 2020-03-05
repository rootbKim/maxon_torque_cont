#ifndef VARIABLE_H_
#define VARIABLE_H_

char MIR1[100];
char uart[18];
char buff[100];
char uart_buff[8];

#define SYSCLK      150E6   /* 150MHz */
#define TBCLK       150E6   /* 150MHz */
#define PWMCARRIER  20E3    /* 20kHz */

float32 Cpu_Clk;
float32 Timer_Prd;
Uint16 i = 0, M_i = 0, E_i = 0, D_i = 0;

int a = 0;
int Flash_bit=0;
int flag = 0;
long torque = 0;
double torque_offset = 0;
double max_motor_torque = 0.75;
unsigned int gear_ratio = 60;
double torque_fourier = 0;
int stuff_position = 0;
int stuff_position2 = 0;
int buff_i = 0;
int stuff_i = 0;
unsigned short decimal2hexadecimal[8] = {0, };

double DegTimer = 0;
double Encoder_deg_time = 0;
double E_vel_deg_time = 0;
double Position_error = 0;
double Kp = 0;

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

char RxBuff[16];
char RxBuff2[30];
char Receivedbuff;
int len=0;
int protocol_len=14;
int Enable_num = 1;

Uint32 Encoder[10], Encoder_sum = 0, Encoder_cnt = 0;
double Encoder_deg_new = 0;
double Encoder_deg_old = 0;
double Encoder_deg = 0;
Uint32 Encoder_revcnt = 0;
double Encoder_vel = 0;
double Encoder_vel_old = 0;
double Encoder_acc = 0;
double E_vel_deg_new = 0;
double E_vel_deg_old = 0;
double EV_Buff[20];
double ED_Buff[20];
int V_i=0;
double ED_mva = 0;
double ED_mva_old = 0;
double EV_mva = 0;
double EV_mva_old = 0;

double move_dis = 0;
unsigned int mode_num = 0;
double velocity = 0;
double R_velocity = 0;
double tablet_velocity = 0;
double under_velocity = 0;
float break_duty = 0;

// Newton_2km
double       a0 =      -0.2778  ;
double       a1 =       -2.473  ;
double       b1 =        -1.49  ;
double       a2 =      -0.7655  ;
double       b2 =      -0.2943  ;
double       a3 =      -0.3182  ;
double       b3 =       0.1401  ;
double       a4 =      -0.1315  ;
double       b4 =      -0.1185  ;
double       w =        0.0349  ;

// Encoder_deg_time
double       ae0 =       178.8  ;
double       ae1 =      -127.6  ;
double       be1 =      -95.55  ;
double       ae2 =      -65.83  ;
double       be2 =        18.9  ;
double       ae3 =      -6.494  ;
double       be3 =        33.9  ;
double       ae4 =       9.825  ;
double       be4 =       15.11  ;
double       ae5 =       8.628  ;
double       be5 =       0.712  ;
double       ae6 =       3.074  ;
double       be6 =      -2.617  ;
double       ae7 =     -0.0374  ;
double       be7 =      -1.438  ;
double       ae8 =      -0.327  ;
double       be8 =     -0.3416  ;
double       we =       2.0048  ;
/*
double       a0 =      -0.805  ;
double       a1 =      -13.87  ;
double       b1 =      -7.798  ;
double       a2 =     -0.3677  ;
double       b2 =     -1.1566  ;
double       a3 =    -0.07837  ;
double       b3 =      0.1774  ;
double       w =       0.0349  ;
*/

// 통신 변수 선언.
Uint16 TimerCount = 0, MotorCount = 0, TimerCount_2 = 0;

float32 usec_delay=100000;

#endif /* VARIABLE_H_ */
