#include "DSP28x_Project.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "variable.h"

#pragma CODE_SECTION(Encoder_define,"ramfuncs")
#pragma CODE_SECTION(Uart_transmit,"ramfuncs")
#pragma CODE_SECTION(UART_Put_String,"ramfuncs")
#pragma CODE_SECTION(BT_transmit,"ramfuncs")
#pragma CODE_SECTION(TrainAbnormalPerson,"ramfuncs")
#pragma CODE_SECTION(UpdateInformation,"ramfuncs")

// CPU timer0 선언 //
interrupt void cpu_timer0_isr(void);

//------------------함수------------------//
void Initialize_motor(int init_bit, int pause_bit, int end_bit);
int Robot_Initialize();
void clear_variable();
void MetabolizeRehabilitationRobot();
int ConnectBluetooth();
void OutputPWM();
void TrainAbnormalPerson();
void UpdateInformation();
int IsStart();
int IsEnd();
void BeNormal();
int IsPause();
int Type_Check_fun();
int Start_breaking();
void Reg_setting_fun();
void Encoder_define();
void Encoder_position_renew();
void Encoder_value_calculation();
void break_time();

void Motor_Enable1();
void Motor_Enable2();
unsigned short* decimal2hex(long torque);
unsigned short CalcFieldCRC(unsigned short* pDataArray, unsigned short ArrayLength);
void torque_fourier_constant(double target_gain);
void Torque_Calculate();
void Timer_set();

//------------------함수------------------//
void InitEPwm1Module(void);

// 통신 설정을 위한 함수 //
void scia_echoback_init(void);
void scia_fifo_init(void);
void scib_echoback_init(void);
void scib_fifo_init(void);
void scic_echoback_init(void);
void scic_fifo_init(void);

// App 전송 //
void BT_transmit();
void BT_Put_String(char* BT_string);

// Data 전송을 위한 함수
void Uart_transmit();
void UART_Put_String(char* Uart_string);

interrupt void scicTxFifoIsr(void);
interrupt void sciaRxFifoIsr(void);

//--------------main 함수------------------//

void main(void) {
	// Step 1. Disable Global Interrupt
	DINT;

	// Step 2. 시스템 컨트롤 초기화:
	InitSysCtrl();

	// FLASH 영역의 함수를 빠른 속도를 위해 RAM으로 구동시키기 위해 선언한 함수
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();

	// Step 3. 인터럽트 초기화:
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();

	// GPIO Pin을 밑의 기능을 사용하기 위해 재배치
	InitSciaGpio();
	InitScibGpio();///////////////////////////////////////////////////////////////////////////////////////////////문제가 있을수도 있음
	InitScicGpio();
	InitAdc();
	InitEPwm1Gpio();

	// Vector table을 내가 사용하기 위한 기능으로 배치
	Reg_setting_fun();

	// PWM 초기화 함수
	InitEPwm1Module();

	// CPU Timer 초기화
	InitCpuTimers();
	Cpu_Clk = 150;          // 현재 시스템 클럭을 설정 (MHz 단위)
	Timer_Prd = 1000;      // 타이머 주기 설정 (usec 단위) // 200 Hz -> 5000
	ConfigCpuTimer(&CpuTimer0, Cpu_Clk, Timer_Prd);

	// CPU Timer0 시작
	StartCpuTimer0();

	// CPU Timer0 인터럽트 활성화
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;         // PIE 인터럽트(TINT0) 활성화
	PieCtrlRegs.PIEIER8.bit.INTx6 = 1;
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		  // SCIRXB
	IER = IER | M_INT1 | M_INT8 | M_INT9;              // CPU 인터럽트(INT1), SCIRXB  활성화

	// 통신함수 초기화
	scia_fifo_init();      // Initialize the SCI FIFO
	scia_echoback_init();  // Initalize SCI for echoback
	scib_fifo_init();      // Initialize the SCI FIFO
	scib_echoback_init();  // Initalize SCI for echoback
	scic_fifo_init();      // Initialize the SCI FIFO
	scic_echoback_init();  // Initalize SCI for echoback

	//버퍼 비우기
	for (i = 0; i < 20; i++)
		EV_Buff[i] = 0;
	for (i = 0; i < 16; i++)
		RxBuff[i] = 0;
	for (i = 0; i < 50; i++)
		BT[i] = 0;
	EINT;
	// Enable Global interrupt INTM
	ERTM;
	// Enable Global realtime interrupt DBGM

	GpioDataRegs.GPBDAT.bit.GPIO48 = 1;
	DELAY_US(usec_delay);
	GpioDataRegs.GPBDAT.bit.GPIO51 = 0;
	DELAY_US(usec_delay);

	// IDLE loop. Just sit and loop forever :
	//--------------------------------------------------------------------------------------------

	for (;;) {

	}
}

// 메인함수 끝.

//함수시작
void Reg_setting_fun() {
	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
	PieVectTable.SCITXINTC = &scicTxFifoIsr;
	SysCtrlRegs.HISPCP.bit.HSPCLK = 1;

	//엔코더 입력 핀 사용 설정
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
	GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;

	//led 제어 사용 설정
	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;

	//led 제어 사용 설정
	GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;

	// PWM 1B를 사용하기 위해 GPIO1을 Pull-up 시킴
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;

	// PWM 1B를 사용하기 위해 MUX Pin 배치
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;

	EDIS;
}

void Initialize_motor(int init_bit, int pause_bit, int end_bit)
{
	if(pause_bit == 1 || end_bit == 1)
	{
		Kp = 1.5;
		Kd = 0.075;
	}
	else if(pause_bit == 0)
	{
		Kp = 0.3;
		Kd = 0.075;
	}
	Kp = 0.3;
	Kd = 0.075;

	if(init_bit == 0)
	{
		Timer_set();
		torque_fourier_1 = a0_1 + a1_1 * cos(Encoder_deg_new * w_1)
			+ b1_1 * sin(Encoder_deg_new * w_1)
			+ a2_1 * cos(2 * Encoder_deg_new * w_1)
			+ b2_1 * sin(2 * Encoder_deg_new * w_1)
			+ a3_1 * cos(3 * Encoder_deg_new * w_1)
			+ b3_1 * sin(3 * Encoder_deg_new * w_1)
			+ a4_1 * cos(4 * Encoder_deg_new * w_1)
			+ b4_1 * sin(4 * Encoder_deg_new * w_1);

		torque_interpolation = current_gain * torque_fourier_1;

		Position_error = E_vel_deg_time - E_vel_deg_new;
		torque_buffer = torque_interpolation * torque_scale + Kp * Position_error - Kd * Encoder_vel; // + integrator;

		Kp_term = Kp * Position_error;
		Kd_term = Kd * Encoder_vel;
		torque = torque_buffer * 1000;
		if (torque <= 0)
			torque = 0;
		if (torque >= 45000)
		{
			torque = 44900;
				flag2++;
		}

		torque = torque / gear_ratio; // 감속비 60
		torque = (torque / max_motor_torque);	// 모터 정격 토크 = 0.75
		Torque_Calculate();
		ScicRegs.SCIFFTX.bit.TXFFIENA = 1;

	}
	else if(init_bit == 1)
	{
		if(leg_num == 1) DegTimer = 0.7798;
		if(leg_num == 2) DegTimer = 2.8527;
//		DegTimer = (Encoder_deg_new / 360) * 4.284;
		torque = 0;
		torque_interpolation = 0;
		mass_torque = 0;
		Position_error = 0;
		Vel_error = 0;
		Acc_error = 0;
		Encoder_vel_deg = 0;
		Encoder_acc_deg = 0;
		Encoder_deg_time = 0;
		E_vel_deg_time = 0;
		Encoder_revcnt = 0;
		time_Encoder_revcnt = 0;
		Encoder_vel = 0;
		Encoder_acc = 0;
		current_gain = 1;
		gain_bit = 1;
		Kp = 1.5;
		Kd = 0.075;
		Torque_Calculate();
		ScicRegs.SCIFFTX.bit.TXFFIENA = 1;
	}
}

int Robot_Initialize() {
	if (break_bit1 == 1 && break_bit2 == 0)
	{
		break_time();
	}

	//환측다리=오른발이면
	if (leg_num == 1) {
		if (!init_bit) {
			break_duty = 1;	//브레이크 OFF
			gain_bit = 1;
			current_gain = 0.6;
			Initialize_motor(init_bit, pause_bit, end_bit);

			if (Encoder_deg_new >= 45 && Encoder_deg_new <= 55) {
				break_duty = 0;
				velocity = 0;
				init_bit = 1;
				Initialize_motor(init_bit, pause_bit, end_bit);
				if(end_bit == 1) clear_variable();
				if(pause_finish == 0)	Play_the_game = 0;
				if (pause_finish == 0 && pause_bit)
					pause_finish = 1;
				return 1;
			}
			return 0;
		}
		else
			return 1;
	}
	//왼발이면
	else if (leg_num == 2) {
		if (!init_bit) {
			break_duty = 1;
			gain_bit = 1;
			current_gain = 0.6;
			Initialize_motor(init_bit, pause_bit, end_bit);

			if (Encoder_deg_new >= 225 && Encoder_deg_new <= 235) {
				break_duty = 0;
				velocity = 0;
				init_bit = 1;
				Initialize_motor(init_bit, pause_bit, end_bit);
				if(end_bit == 1) clear_variable();
				if(pause_finish == 0)	Play_the_game = 0;
				if (pause_finish == 0 && pause_bit)
					pause_finish = 1;
				return 1;
			}
			return 0;
		}
		else
			return 1;
	}
	return 0;
}

void clear_variable() {
	leg_num = 0;
	start_bit = 0;	//시작비트
	training_timer = 0;	//타이머0
	Play_the_game = 0;
	move_dis = 0;	//이동거리
	target_time = 0;	//목표시간
	time_now = 0;	//보행시간
	pause_bit = 0;	//일시정지비트
	pause_finish = 0;
	target_gain = 0;	//목표게인
	end_bit = 0;	//종료비트
	break_timer = 0;	//
	ratio_gain = 0;
	mode_num = 0;
	Type_sel = 0;
	target_dis = 0;
	E_vel_deg_new = 0;
	Encoder_revcnt = 0;
	velocity = 0;
	R_velocity = 0;
	tablet_velocity = 0;
	EV_mva = 0;
	init_bit = 0;
	break_bit1 = 0;
	break_bit2 = 0;
	motor_bit = 0;
	initial_timer = 0;

	slow_start_timer = 0;
	break_time_now = 0;
	Train_num = 0;
	Train_target = 0;
	time_now_min_10 = 0;
	time_now_min_1 = 0;
	time_now_sec_10 = 0;
	time_now_sec_1 = 0;
	break_time_10 = 0;
	break_time_1 = 0;

	torque = 0;
	torque_interpolation = 0;
	mass_torque = 0;
	Position_error = 0;
	Vel_error = 0;
	Acc_error = 0;
	DegTimer = 0;
	Encoder_vel_deg = 0;
	Encoder_acc_deg = 0;
	Encoder_deg_time = 0;
	time_Encoder_revcnt = 0;
	Encoder_vel = 0;
	Encoder_acc = 0;
	current_gain = 1;
	gain_bit = 1;

	for (i = 0; i < 16; i++) RxBuff[i] = 0;
	for (i = 0; i < 20; i++) EV_Buff[i] = 0;
	for (i = 0; i < 20; i++) ED_Buff[i] = 0;
	for (i = 0; i < 50; i++) BT[i] = 0;
}

void InitEPwm1Module(void) {
	/* Setup TBCLK */
	EPwm1Regs.TBPRD = (150E6 / 20E3) - 1; /* Set Timer Period */
	EPwm1Regs.TBCTR = 0; /* Clear Counter */

	/* Set Compare values */
	EPwm1Regs.CMPA.half.CMPA = ((EPwm1Regs.TBPRD + 1) >> 1); /* Set Compare A value to 50% */
	EPwm1Regs.CMPB = ((EPwm1Regs.TBPRD + 1) >> 1);
	/* Setup counter mode */
	EPwm1Regs.TBCTL.bit.CTRMODE = 0; /* Count Up (Asymmetric) */
	EPwm1Regs.TBPHS.half.TBPHS = 0; /* Phase is 0 */
	EPwm1Regs.TBCTL.bit.PHSEN = 0; /* Disable phase loading */
	EPwm1Regs.TBCTL.bit.PRDLD = 0; /* Period Register is loaded from its shadow when CNTR=Zero */
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; /* Clock ratio to SYSCLKOUT */
	EPwm1Regs.TBCTL.bit.CLKDIV = 0; /* TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) */

	/* Setup shadowing */
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0; /* Enable Shadowing */
	EPwm1Regs.CMPCTL.bit.LOADAMODE = 0; /* Load on CNTR=Zero */

	/* Set actions */
	EPwm1Regs.AQCTLA.bit.ZRO = 2; /* Set EPWM1A on CNTR=Zero */
	EPwm1Regs.AQCTLA.bit.CAU = 1; /* Clear EPWM1A on event A, up count */
	EPwm1Regs.AQCTLB.bit.ZRO = 2;
	EPwm1Regs.AQCTLB.bit.CBU = 1;

	/* Set Interrupts */
	EPwm1Regs.ETSEL.bit.INTSEL = 1; /* Select INT on CNTR=Zero */
	EPwm1Regs.ETSEL.bit.INTEN = 1; /* Enable INT */
	EPwm1Regs.ETPS.bit.INTPRD = 1; /* Generate INT on 1st event */
}

void scia_echoback_init() {
	SciaRegs.SCICTL1.bit.SWRESET = 0;
	SciaRegs.SCICCR.bit.SCICHAR = 7; // 1 stop bit, No loopback, No parity, 8 char bits,
	SciaRegs.SCICTL1.bit.RXENA = 1;    // SCI 송신기능 Enable
	SciaRegs.SCICTL1.bit.TXENA = 1;    // async mode, idle-line protocol
	SciaRegs.SCICTL2.all = 0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA = 1;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

#if (CPU_FRQ_150MHZ)
	SciaRegs.SCIHBAUD = 0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
	SciaRegs.SCILBAUD = 0x0028;
#endif
#if (CPU_FRQ_100MHZ)
	SciaRegs.SCIHBAUD = 0x0001;  // 9600 baud @LSPCLK = 20MHz.
	SciaRegs.SCILBAUD = 0x0044;
#endif
	SciaRegs.SCICTL1.bit.SWRESET = 1;
	//SciaRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
	//SciaRegs.SCILBAUD    =0x0079;
	//SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scia_fifo_init() {
	SciaRegs.SCIFFTX.all = 0xE040;
	SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
	SciaRegs.SCIFFCT.all = 0x0;
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
	SciaRegs.SCIFFRX.bit.RXFFIL = 1;
}

void scib_echoback_init() {
	ScibRegs.SCICTL1.bit.SWRESET = 0;
	ScibRegs.SCICCR.bit.SCICHAR = 7; // 1 stop bit, No loopback, No parity, 8 char bits,
	ScibRegs.SCICTL1.bit.RXENA = 1;    // SCI 송신기능 Enable
	ScibRegs.SCICTL1.bit.TXENA = 1;    // async mode, idle-line protocol
	ScibRegs.SCICTL2.all = 0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA = 1;
	ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

#if (CPU_FRQ_150MHZ)
	ScibRegs.SCIHBAUD = 0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
	ScibRegs.SCILBAUD = 0x0028;
#endif
#if (CPU_FRQ_100MHZ)
	ScibRegs.SCIHBAUD = 0x0001;  // 9600 baud @LSPCLK = 20MHz.
	ScibRegs.SCILBAUD = 0x0044;
#endif
	ScibRegs.SCICTL1.bit.SWRESET = 1;
	//ScibRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
	//ScibRegs.SCILBAUD    =0x0079;
	//ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void scib_fifo_init() {
	ScibRegs.SCIFFTX.all = 0xE040;
	ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
	ScibRegs.SCIFFCT.all = 0x0;
	ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	ScibRegs.SCIFFRX.bit.RXFFIENA = 1;
	ScibRegs.SCIFFRX.bit.RXFFIL = 1;
}

void scic_echoback_init() {
	ScicRegs.SCICTL1.bit.SWRESET = 0;
	ScicRegs.SCICCR.bit.SCICHAR = 7;
	ScicRegs.SCICTL1.bit.RXENA = 1;
	ScicRegs.SCICTL1.bit.TXENA = 1;
	ScicRegs.SCICTL2.all = 0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA = 1;
	ScicRegs.SCICTL2.bit.RXBKINTENA = 1;

#if (CPU_FRQ_150MHZ)
	ScicRegs.SCIHBAUD = 0x0000;  // 9600 baud @LSPCLK = 37.5MHz.
	ScicRegs.SCILBAUD = 0x0028;
#endif
#if (CPU_FRQ_100MHZ)
	ScicRegs.SCIHBAUD = 0x0001;  // 9600 baud @LSPCLK = 20MHz.
	ScicRegs.SCILBAUD = 0x0044;
#endif
	ScicRegs.SCICTL1.bit.SWRESET = 1;
	//ScibRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
	//ScibRegs.SCILBAUD    =0x0079;
	//ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}
void scic_fifo_init() {
	//   ScicRegs.SCIFFTX.all = 0xE040;
	ScicRegs.SCIFFTX.bit.SCIFFENA = 1;
	ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
	ScicRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
	ScicRegs.SCIFFTX.bit.TXFFIENA = 1;
	ScicRegs.SCIFFTX.bit.TXFFIL = 0;

	ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
	ScicRegs.SCIFFCT.all = 0x0;
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
	ScicRegs.SCIFFRX.bit.RXFFIENA = 1;
	ScicRegs.SCIFFRX.bit.RXFFIL = 1;
}

void BT_Put_String(char* BT_string) {
	while (*BT_string != 0) {
		SciaRegs.SCITXBUF = *BT_string++;
		while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {
		}
	}
}

void UART_Put_String(char* Uart_string) {
	while (*Uart_string != 0) {
		ScibRegs.SCITXBUF = *Uart_string++;
		while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {
		}
	}
}

void BT_transmit() {
		sprintf(BT, "!s%d.%dt%d%d%d%dd%d%d%d%d?\n\0",
				(int) velocity, (int) under_velocity,
				time_now_min_10, time_now_min_1, time_now_sec_10, time_now_sec_1,
				move_distance_1000, move_distance_100, move_distance_10,move_distance_1);

		BT_Put_String(BT);
}

void Uart_transmit() {
	//	sprintf(UT, "%ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\n\0", (long) (10000 * torque), (long) (10000 * torque_interpolation), (long) (10000 * torque_buffer), (long) (10000 * Position_error), (long) (10000 * Encoder_deg_time), (long) (10000 * Encoder_deg_new), (long) (10000 * time_Encoder_revcnt), (long) (10000 * Encoder_revcnt), (long) (10000 * Kp), (long) (10000 * velocity));
	//	sprintf(UT, "%lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %d, %d\n\0", (long long) (10000 * torque), (long long) (10000 * torque_buffer), (long long) (10000 * torque_interpolation), (long long) (10000 * mass_torque), (long long) (10000 * Position_error), (long long) (10000 * Encoder_deg_time), (long long) (10000 * Encoder_deg_new), (long long) (10000 * Encoder_vel), (long long) (10000 * Kp_term), (long long) (10000 * Kd_term), (long long) (100 * velocity), (long long) (10000 * current_gain), (long long) (10000 * tablet_velocity), V_i, flag);
	sprintf(UT, "%lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld\n\0", (long long)(10000 * torque), (long long)(10000 * torque_buffer), (long long)(10000 * torque_interpolation), (long long)(10000 * mass_torque), (long long)(10000 * Encoder_deg_new), (long long)(10000 * Encoder_vel_deg), (long long)(10000 * Encoder_vel), (long long)(10000 * Encoder_acc_deg), (long long)(10000 * Encoder_acc), (long long)(10000 * Vel_error), (long long)(10000 * Acc_error), (long long)(100 * velocity), (long long)(10000 * current_gain), (long long)(10000 * ratio_gain));
	//	sprintf(UT, "%ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\n\0", (long) (10000 * E_vel_deg_new), (long) (10000 * ED_mva), (long) (10000 * Encoder_vel), (long) (10000 * EV_mva), (long) (10000 * Encoder_acc), (long) (10000 * R_velocity), (long) (10000 * tablet_velocity), (long) (V_i), (long) (10000 * Encoder_deg_new));
	UART_Put_String(UT);
}

interrupt void scicTxFifoIsr(void) {
	for (len = 0; len < protocol_len; len++)
		ScicRegs.SCITXBUF = uart[len];


	ScicRegs.SCIFFTX.bit.TXFFIENA = 0;
	ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

}
/*
// 응답 코드 확인
interrupt void scicRxFifoIsr(void) {

   Receivedbuff2 = ScicRegs.SCIRXBUF.bit.RXDT;

   if (b == 0)
   {
	  if (Receivedbuff2 == 0x90)
	  {
		 RxBuff2[b] = Receivedbuff2;
		 b++;
	  }
	  else
	  {
		 RxBuff2[13] = 0;
		 b = 0;
	  }
   }
   else if (b == 1)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 2)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 3)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 4)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 5)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 6)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 7)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 8)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 9)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 10)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 11)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
	  if(RxBuff2[3] == 0x02){
		 b = 0;
	  }
   }
   else if (b == 12)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
   }
   else if (b == 13)
   {
	  RxBuff2[b] = Receivedbuff2;
	  b++;
	  if(RxBuff2[3] == 0x04){
		 b = 0;
	  }
   }
   sprintf(RxBuff2, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c", RxBuff2[0], RxBuff2[1], RxBuff2[2], RxBuff2[3], RxBuff2[4], RxBuff2[5], RxBuff2[6], RxBuff2[7], RxBuff2[8], RxBuff2[9], RxBuff2[10], RxBuff2[11], RxBuff2[12], RxBuff2[13]);

   ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;         // Clear Overflow flag
   ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;         // Clear Interrupt flag
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;      // Acknowledge interrupt to PIE
}
*/
interrupt void sciaRxFifoIsr(void) {

	Receivedbuff = SciaRegs.SCIRXBUF.bit.RXDT;

	if (a == 0)
	{
		if (Receivedbuff == '!')
		{
			RxBuff[a] = Receivedbuff;
			a++;
		}
		else
		{
			RxBuff[6] = 0;
			a = 0;
		}
	}
	else if (a == 1)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 2)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 3)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 4)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 5)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}
	else if (a == 6)
	{
		RxBuff[a] = Receivedbuff;
		a++;
	}

	if (Receivedbuff == '?') {
		RxBuff[a] = Receivedbuff;
		a = 0;
	}

	if (RxBuff[0] == '!' && RxBuff[1] == 'S' && RxBuff[2] == '?') {
		if (leg_num != 0 && mode_num != 0) {
			if (Type_sel == 1 || Type_sel == 2) {
				start_bit = 1;
				RxBuff[6] = 0;
			}
			else {
				start_bit = 0;
				RxBuff[6] = 0;
			}
		}

		else {
			start_bit = 0;
			RxBuff[6] = 0;
		}
	}

	else if (RxBuff[0] == '!' && RxBuff[1] == 'E' && RxBuff[2] == '?') {
		end_bit = 1;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'P' && RxBuff[2] == '1'
		&& RxBuff[3] == '?') {
		pause_bit = 1;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'P' && RxBuff[2] == '2'
		&& RxBuff[3] == '?') {
		pause_bit = 0;
		pause_finish = 0;
		RxBuff[6] = 0;
		pause_finish = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'M' && RxBuff[2] == '1'
		&& RxBuff[3] == '?') {
		mode_num = 1;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'M' && RxBuff[2] == '2'
		&& RxBuff[3] == '?') {
		target_gain = 0;
		mode_num = 2;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'M' && RxBuff[2] == '3'
		&& RxBuff[3] == '?') {
		mode_num = 3;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'H' && RxBuff[2] == 'R'
		&& RxBuff[3] == '?') {
		leg_num = 1;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'H' && RxBuff[2] == 'L'
		&& RxBuff[3] == '?') {
		leg_num = 2;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'D' && RxBuff[6] == '?') {
		target_dis = atof(&RxBuff[2]);
		target_dis = target_dis * 0.01;

		Type_sel = 1;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'F' && RxBuff[6] == '?') {
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'T' && RxBuff[6] == '?') {
		target_time = atof(&RxBuff[2]);
		break_num = ((int)target_time) % 10;
		Train_target = ((int)(target_time / 10)) % 10;
		target_time = (int)(target_time / 100);
		target_time = (int)target_time * 60;

		Type_sel = 2;
		RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'G' && RxBuff[4] == '?') {
		if (mode_num == 1) {
			target_gain = atof(&RxBuff[2]);
			target_gain = target_gain * 0.1;
			RxBuff[6] = 0;
		}
		else RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'R' && RxBuff[4] == '?') {
		if (mode_num == 2) {
			target_gain = atof(&RxBuff[2]);
			target_gain = target_gain * 0.1 * ratio_gain;
			target_gain = (int)(target_gain * 10);
			target_gain = target_gain / 10;
			RxBuff[6] = 0;
		}
		else RxBuff[6] = 0;
	}
	else if (RxBuff[0] == '!' && RxBuff[1] == 'K' && RxBuff[4] == '?') {
		if (mode_num == 2) {
			ratio_gain = atof(&RxBuff[2]);
			ratio_gain = ratio_gain * 0.01;
			RxBuff[6] = 0;
		}
		else RxBuff[6] = 0;
	}
	else RxBuff[6] = 0;

	SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;			// Clear Overflow flag
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;			// Clear Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;		// Acknowledge interrupt to PIE

}
// LCD 없이 모터제어를 위한 통신프로토콜
void Encoder_position_renew() {
	Encoder[0] = GpioDataRegs.GPADAT.bit.GPIO5;
	Encoder[1] = GpioDataRegs.GPBDAT.bit.GPIO37;
	Encoder[2] = GpioDataRegs.GPADAT.bit.GPIO25;
	Encoder[3] = GpioDataRegs.GPADAT.bit.GPIO27;
	Encoder[4] = GpioDataRegs.GPADAT.bit.GPIO12;
	Encoder[5] = GpioDataRegs.GPADAT.bit.GPIO14;
	Encoder[6] = GpioDataRegs.GPBDAT.bit.GPIO32;
	Encoder[7] = GpioDataRegs.GPADAT.bit.GPIO7;
	Encoder[8] = GpioDataRegs.GPADAT.bit.GPIO9;
	Encoder[9] = GpioDataRegs.GPADAT.bit.GPIO11;
}

void Encoder_value_calculation()
{
	Encoder_sum = 0;

	for (Encoder_cnt = 0; Encoder_cnt < 10; Encoder_cnt++) {
		Encoder_sum += Encoder[Encoder_cnt] << Encoder_cnt; // Encoder_sum 은 0-1024 Pulse까지의 수를 Count해줌.
	}

	Encoder_deg_new = 360 - (double)Encoder_sum * 0.3515625; // Encoder값 갱신. 1024 Pulse를 0 - 360 deg로 바꿔줌.
	Encoder_deg_new = Encoder_deg_new - degree_offset;
	if (Encoder_deg_new < 0) Encoder_deg_new = Encoder_deg_new + 360;
	if (Encoder_deg_old - Encoder_deg_new >= 250 && (Play_the_game == 1 || (leg_num !=0 && init_bit == 0))) // 각속도 구할 때 갑자기 100도이상 차이나면 360 -> 0 도로 된것을 알아내는 조건
		Encoder_revcnt++; // 회전수 체크
	if (Encoder_deg_old - Encoder_deg_new <= -250 && (Play_the_game == 1 || (leg_num !=0 && init_bit == 0)))
	Encoder_revcnt--;

	E_vel_deg_new = Encoder_revcnt * 360 + Encoder_deg_new;
	move_dis = 0.001 * E_vel_deg_new * 1190 / 360; // m단위
}

void Moving_avg_degree()
{
	if (D_i < 19)
	{
		ED_Buff[D_i] = E_vel_deg_new; // 각도 buff 저장
		D_i++;
	}

	else if (D_i == 19)
	{
		ED_Buff[D_i] = E_vel_deg_new;
		ED_mva = 0.05 * (ED_Buff[0] + ED_Buff[1] + ED_Buff[2] + ED_Buff[3] + ED_Buff[4] + ED_Buff[5] + ED_Buff[6] + ED_Buff[7] + ED_Buff[8] + ED_Buff[9] +
			ED_Buff[10] + ED_Buff[11] + ED_Buff[12] + ED_Buff[13] + ED_Buff[14] + ED_Buff[15] + ED_Buff[16] + ED_Buff[17] + ED_Buff[18] + ED_Buff[19]); // 각도 moving avg
		for (i = 0; i < 19; i++)
		{
			ED_Buff[i] = ED_Buff[i + 1]; // 각도 buff renew
		}

		D_i = 19;
	}

	Encoder_vel = (ED_mva - ED_mva_old) * 100; // 각속도 계산

	if (E_i < 19)
	{
		EV_Buff[E_i] = Encoder_vel; // 각속도 buff 저장
		E_i++;
	}

	else if (E_i == 19)
	{
		EV_Buff[E_i] = Encoder_vel;
		EV_mva = 0.05 * (EV_Buff[0] + EV_Buff[1] + EV_Buff[2] + EV_Buff[3] + EV_Buff[4] + EV_Buff[5] + EV_Buff[6] + EV_Buff[7] + EV_Buff[8] + EV_Buff[9] +
			EV_Buff[10] + EV_Buff[11] + EV_Buff[12] + EV_Buff[13] + EV_Buff[14] + EV_Buff[15] + EV_Buff[16] + EV_Buff[17] + EV_Buff[18] + EV_Buff[19]); // 각속도 moving avg
		for (i = 0; i < 19; i++)
		{
			EV_Buff[i] = EV_Buff[i + 1]; // 각속도 buff renew
		}

		E_i = 19;
	}

	Encoder_acc = (EV_mva - EV_mva_old) * 100;

	if (Encoder_deg_old - Encoder_deg_new <= -250)
	{
		flag++;
	}
	if (Encoder_deg_old - Encoder_deg_new >= 250)
	{
		if (flag >= 1)
		{
			flag = 0;
		}
		else
		{
			R_velocity = tablet_velocity / (double)(V_i);
			tablet_velocity = 0;
			V_i = 0;
		}
	}
	if (Encoder_deg_new > Encoder_deg_old + 0.1)
	{
		tablet_velocity += (Encoder_vel);
		V_i++;
	}
}

void Encoder_define() {
	Encoder_deg_old = Encoder_deg_new; // 이전 Encoder값을 저장
	E_vel_deg_old = E_vel_deg_new;
	ED_mva_old = ED_mva;
	EV_mva_old = EV_mva;

	// Encoder Digital Input 값 받기
	Encoder_position_renew();
	Encoder_value_calculation();
	if (Play_the_game)	Moving_avg_degree();
	//각속도-->보행속도
	velocity = R_velocity * 0.0119;
	under_velocity = velocity * 10 - ((int)velocity) * 10;
}

void MetabolizeRehabilitationRobot() {

	++TimerCount;
	++TimerCount1;

	//속도, 토크값 컴에서확인
	if (TimerCount1 == 10)
	{
		TimerCount1 = 0;
		Encoder_define();
	}

	if (Play_the_game == 1) Timer_set();
	if (gain_bit)	torque_fourier_constant(current_gain);

	// MATLAB 2 -> 100Hz Bluetooth 40 -> 5Hz
	if (TimerCount == 200) {
		TimerCount = 0;
		Flash_bit = !Flash_bit;

		GpioDataRegs.GPBDAT.bit.GPIO48 = Flash_bit;
		if (start_bit && (!end_bit))
			if (Play_the_game) {
				BT_transmit();
			}
	}
}

void Timer_set() {

	DegTimer = DegTimer + 0.001;

	Encoder_deg_time = ae0 + ae1 * cos(DegTimer * we)
		+ be1 * sin(DegTimer * we)
		+ ae2 * cos(2 * DegTimer * we)
		+ be2 * sin(2 * DegTimer * we)
		+ ae3 * cos(3 * DegTimer * we)
		+ be3 * sin(3 * DegTimer * we)
		+ ae4 * cos(4 * DegTimer * we)
		+ be4 * sin(4 * DegTimer * we)
		+ ae5 * cos(5 * DegTimer * we)
		+ be5 * sin(5 * DegTimer * we)
		+ ae6 * cos(6 * DegTimer * we)
		+ be6 * sin(6 * DegTimer * we)
		+ ae7 * cos(7 * DegTimer * we)
		+ be7 * sin(7 * DegTimer * we)
		+ ae8 * cos(8 * DegTimer * we)
		+ be8 * sin(8 * DegTimer * we);

	if (DegTimer >= SetDegTimer)
	{
		DegTimer = 0;
		if ((int)(target_gain * 10) == (int)(current_gain * 10))
		{
			gain_bit = 1;
			current_gain = target_gain;
		}
		else if ((int)(target_gain * 10) > (int)(current_gain * 10))
		{
			gain_bit = 1;
			current_gain = current_gain + gain_step;
		}
		else if ((int)(target_gain * 10) < (int)(current_gain * 10))
		{
			gain_bit = 1;
			current_gain = current_gain - gain_step;
		}
	}

	if (Encoder_deg_time_old - Encoder_deg_time >= 250) // 각속도 구할 때 갑자기 100도이상 차이나면 360 -> 0 도로 된것을 알아내는 조건
		time_Encoder_revcnt++; // 회전수 체크

	E_vel_deg_time = time_Encoder_revcnt * 360 + Encoder_deg_time;
	Encoder_deg_time_old = Encoder_deg_time;
}

void torque_fourier_constant(double current_gain)
{
	int current_num = current_gain * 10;
	double base_num = (double)(current_num) / 20;
	gain_bit = 0;
	we = w_base * base_num;
	SetDegTimer = BaseDegTimer / base_num;
}

int ConnectBluetooth() {
	if (leg_num == 1 || leg_num == 2)
		return 1;
	else
		return 0;
}

// 초기 모터 Enable
void Motor_Enable1()
{
	if(leg_num == 0)
	{
		uart[0] = 0x90;
		uart[1] = 0x02;
		uart[2] = 0x68;
		uart[3] = 0x04;
		uart[4] = 0x01;
		uart[5] = 0x40;
		uart[6] = 0x60;
		uart[7] = 0x00;
		uart[8] = 0x06;
		uart[9] = 0x00;
		uart[10] = 0x00;
		uart[11] = 0x00;
		uart[12] = 0x22;
		uart[13] = 0x99;
		ScicRegs.SCIFFTX.bit.TXFFIENA = 1;
	}
}
// 초기 모터 Enable
void Motor_Enable2()
{
	if(motor_bit == 0)
	{
		uart[0] = 0x90;
		uart[1] = 0x02;
		uart[2] = 0x68;
		uart[3] = 0x04;
		uart[4] = 0x01;
		uart[5] = 0x40;
		uart[6] = 0x60;
		uart[7] = 0x00;
		uart[8] = 0x0f;
		uart[9] = 0x00;
		uart[10] = 0x00;
		uart[11] = 0x00;
		uart[12] = 0xb3;
		uart[13] = 0x07;
		ScicRegs.SCIFFTX.bit.TXFFIENA = 1;
	}
}
// 토크 값 변환 및 CRC 계산
void Torque_Calculate()
{
	unsigned short* hexadecimal = decimal2hex(torque);
	unsigned short DataArray[6];
	unsigned short CRC = 0;

	protocol_len = 14;

	if (torque >= 0)
	{
		uart[0] = 0x90;
		uart[1] = 0x02;
		uart[2] = 0x68;
		uart[3] = 0x04;
		uart[4] = 0x01;
		uart[5] = 0x71;
		uart[6] = 0x60;
		uart[7] = 0x00;
		uart[8] = (hexadecimal[1] << 4) | hexadecimal[0];
		uart[9] = (hexadecimal[3] << 4) | hexadecimal[2];
		uart[10] = (hexadecimal[5] << 4) | hexadecimal[4];
		uart[11] = (hexadecimal[7] << 4) | hexadecimal[6];

		DataArray[0] = (uart[3] << 8) + uart[2];
		DataArray[1] = (uart[5] << 8) + uart[4];
		DataArray[2] = (uart[7] << 8) + uart[6];
		DataArray[3] = (uart[9] << 8) + uart[8];
		DataArray[4] = (uart[11] << 8) + uart[10];
		DataArray[5] = 0x0000;

		CRC = CalcFieldCRC(DataArray, 6);

		uart[12] = (CRC & 0xff);
		uart[13] = (CRC & 0xff00) >> 8;

		if (uart[8] == 0x90 || uart[9] == 0x90 || uart[10] == 0x90 || uart[11] == 0x90 || uart[12] == 0x90 || uart[13] == 0x90)
		{
			for (stuff_i = 8; stuff_i < 14; stuff_i++)
			{
				stuff_position++;

				if (uart[stuff_i] == 0x90)
				{
					uart_buff[buff_i] = 0x90;
					buff_i++;
					uart_buff[buff_i] = 0x90;
					buff_i++;

					protocol_len++;

					if (stuff_position2 == 0)
					{
						stuff_position2 = stuff_position + 7;
					}
				}
				else if (uart[stuff_i] != 0x90 && stuff_position2 != 0)
				{
					uart_buff[buff_i] = uart[stuff_i];
					buff_i++;
				}
			}
			for (stuff_i = stuff_position2; stuff_i < (stuff_position2 + buff_i); stuff_i++)
			{
				uart[stuff_i] = uart_buff[(stuff_i - stuff_position2)];
			}

			stuff_position = 0;
			stuff_position2 = 0;
			buff_i = 0;
		}
	}
	else if (torque < 0)
	{
		uart[0] = 0x90;
		uart[1] = 0x02;
		uart[2] = 0x68;
		uart[3] = 0x04;
		uart[4] = 0x01;
		uart[5] = 0x71;
		uart[6] = 0x60;
		uart[7] = 0x00;
		uart[8] = -((hexadecimal[1] << 4) | hexadecimal[0]) & 0xff;
		uart[9] = (-((hexadecimal[3] << 4) | hexadecimal[2]) - 1) & 0xff;
		uart[10] = (-((hexadecimal[5] << 4) | hexadecimal[4]) - 1) & 0xff;
		uart[11] = (-((hexadecimal[7] << 4) | hexadecimal[6]) - 1) & 0xff;

		DataArray[0] = (uart[3] << 8) + uart[2];
		DataArray[1] = (uart[5] << 8) + uart[4];
		DataArray[2] = (uart[7] << 8) + uart[6];
		DataArray[3] = (uart[9] << 8) + uart[8];
		DataArray[4] = (uart[11] << 8) + uart[10];
		DataArray[5] = 0x0000;

		CRC = CalcFieldCRC(DataArray, 6);

		uart[12] = (CRC & 0xff);
		uart[13] = (CRC & 0xff00) >> 8;

		if (uart[8] == 0x90 || uart[9] == 0x90 || uart[10] == 0x90 || uart[11] == 0x90 || uart[12] == 0x90 || uart[13] == 0x90)
		{
			for (stuff_i = 8; stuff_i < 14; stuff_i++)
			{
				stuff_position++;

				if (uart[stuff_i] == 0x90)
				{
					uart_buff[buff_i] = 0x90;
					buff_i++;
					uart_buff[buff_i] = 0x90;
					buff_i++;

					if (stuff_position2 == 0)
					{
						stuff_position2 = stuff_position + 7;
					}
				}
				else if (uart[stuff_i] != 0x90 && stuff_position2 != 0)
				{
					uart_buff[buff_i] = uart[stuff_i];
					buff_i++;
				}
			}
			for (stuff_i = stuff_position2; stuff_i < (stuff_position2 + buff_i); stuff_i++)
			{
				uart[stuff_i] = uart_buff[(stuff_i - stuff_position2)];
			}

			stuff_position = 0;
			stuff_position2 = 0;
			buff_i = 0;
		}
	}
}

// hex값으로 변환
unsigned short* decimal2hex(long torque)
{
	int position = 0;
	long decimal = torque;

	for (i = 0; i < 8; i++)
	{
		decimal2hexadecimal[i] = 0;
	}

	if (decimal < 0) decimal = -decimal;

	while (1)
	{
		int mod = decimal % 16;
		decimal2hexadecimal[position] = mod;
		decimal = decimal / 16;
		position++;

		if (decimal == 0)
			break;
	}

	return decimal2hexadecimal;
}

// CRC 계산
unsigned short CalcFieldCRC(unsigned short* pDataArray, unsigned short ArrayLength)
{
	unsigned short shifter;
	unsigned short c;
	unsigned short carry;
	unsigned short CRC = 0;

	while (ArrayLength--)
	{
		shifter = 0x8000;      //Initialize BitX to Bit15
		c = *pDataArray++;      //Copy next DataWord to c
		do
		{
			carry = CRC & 0x8000;      //Check if Bit15 of CRC is set
			CRC <<= 1;               //CRC = CRC * 2
			if (c & shifter) CRC++;      //CRC = CRC + 1, if BitX is set in c
			if (carry) CRC ^= 0x1021;   //CRC = CRC XOR G(x), if carry is true
			shifter >>= 1;            //Set BitX to next lower Bit, shifter = shifter/2
		} while (shifter);
	}

	return CRC;
}

void OutputPWM() {
	if (break_duty >= 1)
		break_duty = 1;
	else if (break_duty <= 0)
		break_duty = 0;

	// Brake의 Duty를 조절하는 함수. Brake Duty는 0.0 ~ 1.0 사이어야 함.
	EPwm1Regs.TBPRD = (150E6 / 20E3) - 1;
	EPwm1Regs.CMPB = EPwm1Regs.TBPRD * break_duty;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

int IsStart() {
	return start_bit;
}

int IsPause() {
	if (pause_bit) {
		if (IsEnd()) {
			BeNormal();
		}
		if (!pause_finish) {
			if (init_bit == 1)
			{
				DegTimer = (Encoder_deg_new / 360) * 7.14;
				init_bit = 0;
			}
			Robot_Initialize();
			return 1;
		}
		else
			return 1;
	}
	return 0;
}

void IncreaseTime() {
	++training_timer;
	// 훈련시간 확인 알려주는것
	if (training_timer == 1000) {
		training_timer = 0;
		++time_now;
	}
}

void TrainAbnormalPerson() {

	switch (mode_num) {
	case 1:
		break_duty = 1;
		torque_fourier_1 = a0_1 + a1_1 * cos(Encoder_deg_new * w_1)
			+ b1_1 * sin(Encoder_deg_new * w_1)
			+ a2_1 * cos(2 * Encoder_deg_new * w_1)
			+ b2_1 * sin(2 * Encoder_deg_new * w_1)
			+ a3_1 * cos(3 * Encoder_deg_new * w_1)
			+ b3_1 * sin(3 * Encoder_deg_new * w_1)
			+ a4_1 * cos(4 * Encoder_deg_new * w_1)
			+ b4_1 * sin(4 * Encoder_deg_new * w_1);
		torque_fourier_3 = a0_3 + a1_3 * cos(Encoder_deg_new * w_3)
			+ b1_3 * sin(Encoder_deg_new * w_3)
			+ a2_3 * cos(2 * Encoder_deg_new * w_3)
			+ b2_3 * sin(2 * Encoder_deg_new * w_3)
			+ a3_3 * cos(3 * Encoder_deg_new * w_3)
			+ b3_3 * sin(3 * Encoder_deg_new * w_3)
			+ a4_3 * cos(4 * Encoder_deg_new * w_3)
			+ b4_3 * sin(4 * Encoder_deg_new * w_3);
		if (current_gain >= 1)	torque_interpolation = ((3 - current_gain) / 2) * torque_fourier_1 + ((current_gain - 1) / 2) * torque_fourier_3;
		else if (current_gain < 1) torque_interpolation = current_gain * torque_fourier_1;

		mass_torque = a0 + a1 * cos(Encoder_deg_new * w)  // 최대 200kg;
			+ b1 * sin(Encoder_deg_new * w)
			+ a2 * cos(2 * Encoder_deg_new * w)
			+ b2 * sin(2 * Encoder_deg_new * w)
			+ a3 * cos(3 * Encoder_deg_new * w)
			+ b3 * sin(3 * Encoder_deg_new * w);
		mass_torque = (double)mass * 0.005 * mass_torque;

		Position_error = E_vel_deg_time - E_vel_deg_new;
		torque_buffer = torque_interpolation * torque_scale + mass_torque + Kp * Position_error - Kd * Encoder_vel; // + integrator;
		Kp_term = Kp * Position_error;
		Kd_term = Kd * Encoder_vel;
		torque = torque_buffer * 1000;
		if (torque <= 0)
			torque = 0;
		if (torque >= 45000)
		{
			torque = 44900;
			flag2++;
		}

		torque = torque / gear_ratio; // 감속비 60
		torque = (torque / max_motor_torque);	// 모터 정격 토크 = 0.75
		Torque_Calculate();
		ScicRegs.SCIFFTX.bit.TXFFIENA = 1;

		break;

	case 2:
		flag2 = 2;
		break_duty = 1;

		torque_fourier_1 = a0_1 + a1_1 * cos(Encoder_deg_new * w_1)
			+ b1_1 * sin(Encoder_deg_new * w_1)
			+ a2_1 * cos(2 * Encoder_deg_new * w_1)
			+ b2_1 * sin(2 * Encoder_deg_new * w_1)
			+ a3_1 * cos(3 * Encoder_deg_new * w_1)
			+ b3_1 * sin(3 * Encoder_deg_new * w_1)
			+ a4_1 * cos(4 * Encoder_deg_new * w_1)
			+ b4_1 * sin(4 * Encoder_deg_new * w_1);
		torque_fourier_3 = a0_3 + a1_3 * cos(Encoder_deg_new * w_3)
			+ b1_3 * sin(Encoder_deg_new * w_3)
			+ a2_3 * cos(2 * Encoder_deg_new * w_3)
			+ b2_3 * sin(2 * Encoder_deg_new * w_3)
			+ a3_3 * cos(3 * Encoder_deg_new * w_3)
			+ b3_3 * sin(3 * Encoder_deg_new * w_3)
			+ a4_3 * cos(4 * Encoder_deg_new * w_3)
			+ b4_3 * sin(4 * Encoder_deg_new * w_3);

		Encoder_vel_deg = av0 + av1 * cos(Encoder_deg_new * wv)
			+ bv1 * sin(Encoder_deg_new * wv)
			+ av2 * cos(2 * Encoder_deg_new * wv)
			+ bv2 * sin(2 * Encoder_deg_new * wv)
			+ av3 * cos(3 * Encoder_deg_new * wv)
			+ bv3 * sin(3 * Encoder_deg_new * wv)
			+ av4 * cos(4 * Encoder_deg_new * wv)
			+ bv4 * sin(4 * Encoder_deg_new * wv);

		Encoder_acc_deg = aa0 + aa1 * cos(Encoder_deg_new * wa)
			+ ba1 * sin(Encoder_deg_new * wa)
			+ aa2 * cos(2 * Encoder_deg_new * wa)
			+ ba2 * sin(2 * Encoder_deg_new * wa)
			+ aa3 * cos(3 * Encoder_deg_new * wa)
			+ ba3 * sin(3 * Encoder_deg_new * wa)
			+ aa4 * cos(4 * Encoder_deg_new * wa)
			+ ba4 * sin(4 * Encoder_deg_new * wa);

		Encoder_vel_deg = Encoder_vel_deg * current_gain;
		Encoder_acc_deg = Encoder_acc_deg * current_gain * current_gain;

		if (current_gain >= 1)	torque_interpolation = ((3 - current_gain) / 2) * torque_fourier_1 + ((current_gain - 1) / 2) * torque_fourier_3;
		else if (current_gain < 1) torque_interpolation = current_gain * torque_fourier_1;

		mass_torque = a0 + a1 * cos(Encoder_deg_new * w)  // 최대 200kg;
			+ b1 * sin(Encoder_deg_new * w)
			+ a2 * cos(2 * Encoder_deg_new * w)
			+ b2 * sin(2 * Encoder_deg_new * w)
			+ a3 * cos(3 * Encoder_deg_new * w)
			+ b3 * sin(3 * Encoder_deg_new * w);
		mass_torque = (double)mass * 0.005 * mass_torque;

		Vel_error = Encoder_vel - Encoder_vel_deg;
		Acc_error = Encoder_acc - Encoder_acc_deg;

		if (Vel_error <= 0) Vel_error = 0;
		if (Acc_error <= 0) Acc_error = 0;

		torque_buffer = torque_interpolation * torque_scale + mass_torque * 0.5 * ratio_gain + Kv * Vel_error + Ka * Acc_error;

		torque = torque_buffer * 1000;
		if (torque <= 0)
			torque = 0;
		if (torque >= 45000)
		{
			torque = 44900;
			flag2++;
		}

		torque = torque / gear_ratio; // 감속비 60
		torque = (torque / max_motor_torque);	// 모터 정격 토크 = 0.75
		Torque_Calculate();
		time_Encoder_revcnt = Encoder_revcnt;
		Encoder_deg_time = Encoder_deg_new;
		E_vel_deg_time = E_vel_deg_new;
		ScicRegs.SCIFFTX.bit.TXFFIENA = 1;

		break;

	case 3:

		break_duty = 1;

		break;
	}
}

void UpdateInformation() {
	//시간변수 업데이트
	time_now_sec_1 = time_now % 60;
	time_now_sec_1 = time_now_sec_1 % 10;

	time_now_sec_10 = time_now % 60;
	time_now_sec_10 = time_now_sec_10 / 10;

	time_now_min_1 = time_now / 60;
	time_now_min_1 = time_now_min_1 % 60;
	time_now_min_1 = time_now_min_1 % 10;

	time_now_min_10 = time_now / 60;
	time_now_min_10 = time_now_min_10 % 60;
	time_now_min_10 = time_now_min_10 / 10;

	//거리변수 업데이트
	move_distance_1000 = move_distance_100 = move_distance_10 = move_distance_1 = move_dis;
	move_distance_1000 = move_distance_1000 / 1000;
	move_distance_100 = move_distance_100 % 1000;
	move_distance_100 = move_distance_100 / 100;
	move_distance_10 = move_distance_10 % 100;
	move_distance_10 = move_distance_10 / 10;
	move_distance_1 = move_distance_1 % 10;
}

int IsEnd() {
	return end_bit;
}

void BeNormal() {
	if (init_bit)
	{
		DegTimer = (Encoder_deg_new / 360) * 7.14;
		init_bit = 0;
	}
	Robot_Initialize();
}

int Type_Check_fun() {
	if (Type_sel == 1) {
		if (move_dis > target_dis) {
			end_bit = 1;
			sprintf(BT, "!e?");
			BT_Put_String(BT);
			return 1;
		}
	}

	else if (Type_sel == 2) {
		if (time_now >= target_time) {
			if ((Train_target - Train_num) == 1) {
				end_bit = 1;
				sprintf(BT, "!e?");
				BT_Put_String(BT);
			}
			else {
				init_bit = 0;
				break_bit1 = 1;
				if (break_bit1 == 1 && break_bit2 == 1)
				{
					break_time();
				}
				break_bit2 = Robot_Initialize();

				if (IsEnd()) {
					BeNormal();
				}
				return 1;
			}
		}
	}
	return 0;

}

void break_time() {
	++break_timer;
	// 휴식시간 확인 알려주는것
	if (break_timer == 1000) {
		break_timer = 0;
		++break_time_now;
	}
	if (break_time_now >= 60 * break_num) {
		time_now = 0;
		break_time_now = 0;
		break_timer = 0;
		break_bit1 = 0;
		break_bit2 = 0;
		++Train_num;
	}
	break_time_100 = break_time_now / 100;
	break_time_10 = (break_time_now - break_time_100 * 100) / 10;
	break_time_1 = (break_time_now - break_time_100 * 100) / break_time_now % 10;
}

int Initial_breaking(){
	if ((initial_timer < time) && (motor_bit == 0))   //1000hz 1ms
	{
		++initial_timer;
		return 0;
	}
	else if (initial_timer >= time) {
		initial_timer = time+1;
		motor_bit = 1;
		return 1;
	}
	else return 0;
}
int Start_breaking() {
	if ((slow_start_timer < 5000) && (start_bit == 1))   //1000hz 5ms
	{
		++slow_start_timer;
		return 0;
	}
	else if (slow_start_timer >= 5000) {
		slow_start_timer = 5001;
		Play_the_game = 1;
		return 1;
	}
	else return 0;
}

interrupt void cpu_timer0_isr(void) // cpu timer 현재 제어주파수 100Hz
{

	MetabolizeRehabilitationRobot();

	Motor_Enable1();

	if (!ConnectBluetooth())
	{
		DegTimer = (Encoder_deg_new / 360) * 7.11;
		SetDegTimer = 7.11; // 0.6km/h SetDegTimer
		goto RETURN;
	}

	Motor_Enable2();


	if (!Initial_breaking())
	{
		goto RETURN;
	}

	if (!Robot_Initialize())
	{
		goto RETURN;
	}

	if (!IsStart()) goto RETURN;

	if (IsPause()) goto RETURN;

	if (Type_Check_fun()) goto RETURN;

	if (!Start_breaking())
		goto RETURN;
	IncreaseTime();
	TrainAbnormalPerson();
	UpdateInformation();

	if (IsEnd())
		BeNormal();

	RETURN: OutputPWM();

	OutputPWM();
}

//============================================================================================
