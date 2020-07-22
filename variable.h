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
Uint16 i = 0, M_i = 0, E_i = 0, D_i = 0, En_i = 0;

int a = 0;
int b = 0;

int Flash_bit=0;
double flag = 0;
int flag2 = 0;
int gain_bit = 1;
int ratio_bit = 0;
long long torque = 0;
long long vel = 0;
//double torque_offset = 5.5; // ������ ����(200310)
double torque_scale = 4.3;
double max_motor_torque = 0.75;
unsigned int gear_ratio = 60;
double torque_fourier_1 = 0;   // 1km/h fourier
double torque_fourier_3 = 0;   // 3km/h fourier
double torque_interpolation = 0;
double torque_buffer = 0;
double mass_torque = 0;
double torque_dynamics = 0;
int mass = 70;
int stuff_position = 0;
int stuff_position2 = 0;
int buff_i = 0;
int stuff_i = 0;
unsigned short decimal2hexadecimal[8] = {0, };

double DegTimer = 0;
double DegTimer_old = 0;
double BaseDegTimer = 2.133;  // 2km/h
double SetDegTimer = 0;
double time_degree = 0;
double Encoder_deg_time = 0;
double Encoder_deg_time_old = 0;
double Encoder_vel_deg = 0;
double Encoder_acc_deg = 0;
Uint32 time_Encoder_revcnt = 0;
double E_vel_deg_time = 0;
double Position_error = 0;
double Position_error_mvg = 0;
double Vel_error = 0;
double Acc_error = 0;
double Kp = 0.8;   // 1.5
double Kd = 0.028;  // 0.13 // 0.075
double Kv = 0.03;	// 0.03
double Ka = 0.005;	// 0.005
double Kp_term = 0;
double Kd_term = 0;

double target_gain = 1;
double ratio_target_gain = 0;
double ratio_gain = 0;
double target_buff_gain = 1;
double current_gain = 1;
double gain_step = 0.2;
double velocity_mode3 = 0;
double active_ratio_gain = 1;

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

char RxBuff[16];
char RxBuff2[30];
char Receivedbuff;
char Receivedbuff2;
int len=0;
int protocol_len=14;
int Enable_num = 1;

Uint32 Encoder[10], Encoder_old[10], Encoder_sum = 0, Encoder_cnt = 0;
double Encoder_deg_new = 0;
double Encoder_deg_mvg = 0;
double Encoder_deg_mvg2 = 0;
double Encoder_deg_buff = 0;
double degree_offset = 15;
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
double E_Buff[5];
int V_i=0;
double ED_mva = 0;
double ED_mva_old = 0;
double EV_mva = 0;
double EV_mva_old = 0;

double move_dis = 0;
unsigned int mode_num = 0;
int velocity_flag = 0;
double velocity = 0;
double R_velocity = 0;
double tablet_velocity = 0;
double under_velocity = 0;
float break_duty = 0;
unsigned int break_num = 0;
unsigned int break_bit1 = 0;
unsigned int break_bit2 = 0;

unsigned int Type_sel = 0;
unsigned int pause_bit = 0;
int init_bit = 0;
int init_flag = 0;
int init_flag2 = 0;
int Play_the_game = 0;
double training_timer = 0;
unsigned int time_now = 0;
double target_dis = 0;
int pause_finish = 0;
double target_time = 0;

// Mass Torque 50%
double a0 = 52.7;
double a1 = -0.004401;
double b1 = -0.01093;
double a2 = 4.141;
double b2 = -2.41;
double a3 = 0.0001963;
double b3 = -0.0006322;
double w = 0.01745;

// Newton_1km
double a0_1 = -0.07869;
double a1_1 = -0.745;
double b1_1 = -2.119;
double a2_1 = -0.174;
double b2_1 = -0.3115;
double a3_1 = 0.02393;
double b3_1 = -0.07717;
double a4_1 = 0.009776;
double b4_1 = 0.001882;
double w_1 = 0.0349;

// Newton_3km
double a0_3 = -0.708;
double a1_3 = -5.15;
double b1_3 = -0.2478;
double a2_3 = -1.163;
double b2_3 = -0.3155;
double a3_3 = -0.03081;
double b3_3 = -0.4578;
double a4_3 = 0.05215;
double b4_3 = 0.002418;
double w_3 = 0.0349;

// Encoder_deg_time
double ae0 = 177.1;
double ae1 = -135;
double be1 = -88.77;
double ae2 = -64.29;
double be2 = 27.26;
double ae3 = -0.6759;
double be3 = 35.81;
double ae4 = 13.35;
double be4 = 13.03;
double ae5 = 8.794;
double be5 = -1.764;
double ae6 = 1.916;
double be6 = -3.579;
double ae7 = -0.7842;
double be7 = -1.22;
double ae8 = -0.4171;
double be8 = -0.003043;
double we = 0;
double w_base = 1.9848  ; // 2km/h

// time_set 1km/h
double at0 = 0;
double at1 = 0;
double bt1 = 0;
double at2 = 0;
double bt2 = 0;
double at3 = 0;
double bt3 = 0;
double at4 = 0;
double bt4 = 0;
double at5 = 0;
double bt5 = 0;
double at6 = 0;
double bt6 = 0;
double at7 = 0;
double bt7 = 0;
double at8 = 0;
double bt8 = 0;
double wt = 0.01186;
double at0_base = 2.205;
double at1_base = -1.608;
double bt1_base = -1.021;
double at2_base = -0.7358;
double bt2_base = 0.3511;
double at3_base = -0.1215;
double bt3_base = 0.424;
double at4_base = 0.1683;
double bt4_base = 0.1405;
double at5_base = 0.1024;
double bt5_base = -0.03226;
double at6_base = 0.006741;
double bt6_base = -0.05135;
double at7_base = -0.01159;
double bt7_base = -0.01325;
double at8_base = -0.005134;
double bt8_base = 0.0005766;


// Mode2 Kv parameter
double av0 = 86.25;
double av1 = -2.805;
double bv1 = -17.16;
double av2 = 0.7057;
double bv2 = -2.39;
double av3 = 0.5507;
double bv3 = -0.09314;
double av4 = 0.05724;
double bv4 = 0.08293;
double wv = 0.03491;

// Mode2 Ka parameter
double aa0 = 0.0008013;
double aa1 = -51.25;
double ba1 = 7.753;
double aa2 = -12.19;
double ba2 = 0.6664;
double aa3 = -1.037;
double ba3 = -2.598;
double aa4 = 0.3467;
double ba4 = -0.4306;
double wa = 0.03491;

double torque_degree1 = 360;	// 345
double torque_degree2 = 180;	//165
double torque_degree_offset = 80;
double torque_inflection_point = 0;
double torque_inflection_gain = 5;
double theta = 0;
double Radian = 0;
double torque_intention = 0;

// ��� ���� ����.
Uint16 TimerCount = 0, TimerCount1 = 0, TimerCount2 = 0, MotorCount = 0, TimerCount_2 = 0;

int slow_start_timer = 0;
int initial_timer = 0;
int time = 1000;
unsigned int leg_num = 0;
unsigned int start_bit = 0;
unsigned int motor_bit = 0;
unsigned int end_bit = 0;

unsigned int move_distance_1000 = 0;
unsigned int move_distance_100 = 0;
unsigned int move_distance_10 = 0;
unsigned int move_distance_1 = 0;

unsigned int time_now_min_10 = 0;
unsigned int time_now_min_1 = 0;
unsigned int time_now_sec_10 = 0;
unsigned int time_now_sec_1 = 0;

unsigned int Train_num = 0;
unsigned int Train_target = 0;

unsigned int break_timer = 0;
unsigned int break_time_now = 0;

unsigned int break_time_100 = 0;
unsigned int break_time_10 = 0;
unsigned int break_time_1 = 0;

float32 usec_delay = 100000;

#endif /* VARIABLE_H_ */
