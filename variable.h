#ifndef VARIABLE_H_
#define VARIABLE_H_

char MAXON[100], UT[100], BT[50];
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
int flag2 = 0;
int Enable_bit = 0;
int gain_bit = 1;
int ratio_bit = 0;
long long torque = 0;
//double torque_offset = 5.5; // 마찰력 보상(200310)
double torque_scale = 4.3;
double max_motor_torque = 0.75;
unsigned int gear_ratio = 60;
double torque_fourier_1 = 0;	// 1km/h fourier
double torque_fourier_3 = 0;	// 3km/h fourier
double torque_interpolation = 0;
double torque_buffer = 0;
double mass_torque = 0;
int mass = 70;
int stuff_position = 0;
int stuff_position2 = 0;
int buff_i = 0;
int stuff_i = 0;
unsigned short decimal2hexadecimal[8] = {0, };

double DegTimer = 0;
double BaseDegTimer = 2.142;  // 2km/h
double SetDegTimer = 0;
double Encoder_deg_time = 0;
double Encoder_deg_time_old = 0;
double Encoder_vel_deg = 0;
double Encoder_acc_deg = 0;
Uint32 time_Encoder_revcnt = 0;
double E_vel_deg_time = 0;
double Position_error = 0;
double Vel_error = 0;
double Acc_error = 0;
double integrator = 0;
double Kp = 0.32;
double Kd = 0.265;
double Kv = 0;
double Ka = 0;
double Ki = 0;
double Kp_term = 0;
double Kd_term = 0;

double target_gain = 0;
double ratio_target_gain = 0;
double ratio_gain = 0;
double current_gain = 1;
double gain_step = 0.1;

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
double degree_offset = 2;
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

// Mass Torque 100%
double a0 = 107.4;
double a1 = -0.008972;
double b1 = -0.02227;
double a2 = 8.443;
double b2 = -4.911;
double a3 = 0.0004001;
double b3 = -0.001289;
double w = 0.01745;

// Newton_1km
double a0_1 = -0.08009;
double a1_1 = -0.7486;
double b1_1 = -2.089;
double a2_1 = -0.2192;
double b2_1 = -0.2818;
double a3_1 = -0.03954;
double b3_1 = 0.02539;
double a4_1 = -0.03543;
double b4_1 = -0.02386;
double w_1 = 0.0349;

// Newton_3km
double a0_3 = -0.6832;
double a1_3 = -5.218;
double b1_3 = -0.1314;
double a2_3 = -1.608;
double b2_3 = -0.1195;
double a3_3 = -0.6634;
double b3_3 = 0.441;
double a4_3 = -0.3319;
double b4_3 = -0.2368;
double w_3 = 0.0349;

// Encoder_deg_time
double ae0 = 178.8;
double ae1 = -127.6;
double be1 = -95.55;
double ae2 = -65.83;
double be2 = 18.9;
double ae3 = -6.494;
double be3 = 33.9;
double ae4 = 9.825;
double be4 = 15.11;
double ae5 = 8.628;
double be5 = 0.712;
double ae6 = 3.074;
double be6 = -2.617;
double ae7 = -0.0374;
double be7 = -1.438;
double ae8 = -0.327;
double be8 = -0.3416;
double we = 0;
double w_base =   2.0048  ; // 2km/h

// Mode2 Kv parameter
double av0 = 85.96;
double av1 = -3.327;
double bv1 = -17.33;
double av2 = 0.4339;
double bv2 = -2.847;
double av3 = -0.2428;
double bv3 = -0.7497;
double av4 = 0.2398;
double bv4 = -0.2869;
double wv = 0.0349;

// Mode2 Ka parameter
double aa0 = -0.1624;
double aa1 = -51.82;
double ba1 = 9.128;
double aa2 = -15.3;
double ba2 = 2.012;
double aa3 = -6.466;
double ba3 = 4.658;
double aa4 = -2.93;
double ba4 = -1.633;
double wa = 0.0349;

// 통신 변수 선언.
Uint16 TimerCount = 0, TimerCount1 = 0, TimerCount2 = 0, MotorCount = 0, TimerCount_2 = 0;

float32 usec_delay=100000;

#endif /* VARIABLE_H_ */
