#include "DSP28x_Project.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "variable.h"

#pragma CODE_SECTION(Uart_transmit,"ramfuncs")
#pragma CODE_SECTION(UART_Put_String,"ramfuncs")
#pragma CODE_SECTION(BT_transmit,"ramfuncs")
#pragma CODE_SECTION(MAXON_transmit,"ramfuncs")
#pragma CODE_SECTION(MAXON_Put_String,"ramfuncs")
#pragma CODE_SECTION(TrainAbnormalPerson,"ramfuncs")

// CPU timer0 선언 //
interrupt void cpu_timer0_isr(void);

//------------------함수------------------//
void MetabolizeRehabilitationRobot();
void Reg_setting_fun();

// 통신 설정을 위한 함수 //
void scia_echoback_init(void);
void scia_fifo_init(void);
void scib_echoback_init(void);
void scib_fifo_init(void);
void scic_echoback_init(void);
void scic_fifo_init(void);

void MetabolizeRehabilitationRobot();
void Encoder_position_renew();
void Encoder_value_calculation();
void Moving_avg_degree();
void Encoder_define();

void BT_transmit();
void BT_Put_String(char *BT_string);
void Uart_transmit();
void UART_Put_String(char *Uart_string);
void MAXON_transmit();
void MAXON_Put_String(char *MAXON_string);

void Motor_Enable();
unsigned short* decimal2hex(long torque);
unsigned short CalcFieldCRC(unsigned short* pDataArray, unsigned short ArrayLength);
void torque_fourier_constant(double target_gain);
void Timer_set();

void InitEPwm1Module(void);
void OutputPWM();
interrupt void scicRxFifoIsr(void);

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

   InitEPwm1Module();

   // CPU Timer 초기화
   InitCpuTimers();
   Cpu_Clk = 150;          // 현재 시스템 클럭을 설정 (MHz 단위)
   Timer_Prd = 5000;      // 타이머 주기 설정 (usec 단위) // 200 Hz -> 5000
   ConfigCpuTimer(&CpuTimer0, Cpu_Clk, Timer_Prd);

   // CPU Timer0 시작
   StartCpuTimer0();

   // CPU Timer0 인터럽트 활성화
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;         // PIE 인터럽트(TINT0) 활성화
   PieCtrlRegs.PIEIER8.bit.INTx5 = 1;        // SCIRXC
   IER = IER | M_INT1 | M_INT8;              // CPU 인터럽트(INT1), SCIRXC  활성화

   // 통신함수 초기화
	scia_fifo_init();      // Initialize the SCI FIFO
	scia_echoback_init();  // Initalize SCI for echoback
	scib_fifo_init();      // Initialize the SCI FIFO
	scib_echoback_init();  // Initalize SCI for echoback
	scic_fifo_init();      // Initialize the SCI FIFO
	scic_echoback_init();  // Initalize SCI for echoback

   for (i = 0; i < 100; i++)
	   MAXON[i] = 0;

   EINT;
   // Enable Global interrupt INTM
   ERTM;
   // Enable Global realtime interrupt DBGM

   DELAY_US(usec_delay);

   GpioDataRegs.GPBDAT.bit.GPIO48 = 1;
   DELAY_US(usec_delay);
   GpioDataRegs.GPBDAT.bit.GPIO51 = 0;
   DELAY_US(usec_delay);

// IDLE loop. Just sit and loop forever :
//--------------------------------------------------------------------------------------------

   for (;;) {

   }
}

// 핀 설정
void Reg_setting_fun() {
   EALLOW;
   PieVectTable.TINT0 = &cpu_timer0_isr;
   PieVectTable.SCIRXINTC = &scicRxFifoIsr;

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

// 통신 설정
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
	SciaRegs.SCIHBAUD =0x0001;  // 9600 baud @LSPCLK = 20MHz.
	SciaRegs.SCILBAUD =0x0044;
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
	ScibRegs.SCIHBAUD =0x0001;  // 9600 baud @LSPCLK = 20MHz.
	ScibRegs.SCILBAUD =0x0044;
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
   ScicRegs.SCIHBAUD =0x0001;  // 9600 baud @LSPCLK = 20MHz.
   ScicRegs.SCILBAUD =0x0044;
#endif
   ScicRegs.SCICTL1.bit.SWRESET = 1;
   //ScibRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 37.5MHz.
   //ScibRegs.SCILBAUD    =0x0079;
   //ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}
void scic_fifo_init() {
   ScicRegs.SCIFFTX.all = 0xE040;
   ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
   ScicRegs.SCIFFCT.all = 0x0;
   ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
   ScicRegs.SCIFFRX.bit.RXFFIENA = 1;
   ScicRegs.SCIFFRX.bit.RXFFIL = 1;
}

void MetabolizeRehabilitationRobot() {

	++TimerCount; //40 -> 2
	++TimerCount1; //40 -> 2

	if(TimerCount1 == 2)
	{
		TimerCount1 = 0;
		Encoder_define();
	}
	if(mode_num == 1) Timer_set();

	// MATLAB 2 -> 200Hz 40 -> 5Hz
	if (TimerCount == 40) {
		TimerCount = 0;
//		if(mode_num == 1)
//		{
//			Uart_transmit();
//		}
		Flash_bit=!Flash_bit;
		GpioDataRegs.GPBDAT.bit.GPIO48 = Flash_bit;
	}
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

	Encoder_deg_new = 360 - (double) Encoder_sum * 0.3515625; // Encoder값 갱신. 1024 Pulse를 0 - 360 deg로 바꿔줌.
	if (Encoder_deg_old - Encoder_deg_new >= 250 && mode_num == 1) // 각속도 구할 때 갑자기 100도이상 차이나면 360 -> 0 도로 된것을 알아내는 조건
		Encoder_revcnt++; // 회전수 체크
	if (Encoder_deg_old - Encoder_deg_new <= -250 && mode_num == 1)
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
		for(i=0; i<19; i++)
		{
			ED_Buff[i]=ED_Buff[i+1]; // 각도 buff renew
		}

		D_i=19;
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
		for(i=0; i<19; i++)
		{
			EV_Buff[i]=EV_Buff[i+1]; // 각속도 buff renew
		}

		E_i=19;
	}

	Encoder_acc = (EV_mva - EV_mva_old) * 100;

	if (Encoder_deg_old - Encoder_deg_new >= 250)
	{
		R_velocity = tablet_velocity / (double) V_i;
		tablet_velocity = 0;
		V_i = 0;
	}
	if (Encoder_deg_new > Encoder_deg_old + 0.1)
	{
		tablet_velocity += Encoder_vel;
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
	if(mode_num == 1)	Moving_avg_degree();
	//각속도-->보행속도
		velocity = R_velocity * 0.0119;
		under_velocity = velocity * 10 - ((int) velocity) * 10;
}

void BT_Put_String(char *BT_string) {
	while (*BT_string != 0) {
		SciaRegs.SCITXBUF = *BT_string++;
		while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {
		}
	}
}

void UART_Put_String(char *Uart_string) {
	while (*Uart_string != 0) {
		ScibRegs.SCITXBUF = *Uart_string++;
		while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {
		}
	}
}

// 포로토콜 전송
void MAXON_Put_String(char *MAXON_string) {
   for(len = 0; len<protocol_len; len++) {
      ScicRegs.SCITXBUF = *MAXON_string++;
      while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {
      }
   }

}
void MAXON_transmit() {
   MAXON_Put_String(uart);
}

void BT_transmit() {
/*	sprintf(BT, "!s%d.%dt%d%d%d%dd%d%d%d%d?\n\0",
			(int) velocity, (int) under_velocity,
			time_now_min_10, time_now_min_1, time_now_sec_10, time_now_sec_1,
			move_distance_1000, move_distance_100, move_distance_10,move_distance_1);

	BT_Put_String(BT);*/
}

void Uart_transmit() {
//	sprintf(UT, "%ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\n\0", (long) (10000 * torque), (long) (10000 * torque_interpolation), (long) (10000 * torque_buffer), (long) (10000 * Position_error), (long) (10000 * Encoder_deg_time), (long) (10000 * Encoder_deg_new), (long) (10000 * time_Encoder_revcnt), (long) (10000 * Encoder_revcnt), (long) (10000 * Kp), (long) (10000 * velocity));
	sprintf(UT, "%lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %d\n\0", (long long) (10000 * torque), (long long) (10000 * torque_interpolation), (long long) (10000 * mass_torque), (long long) (10000 * Position_error), (long long) (10000 * Encoder_deg_time), (long long) (10000 * Encoder_deg_new), (long long) (10000 * Encoder_vel), (long long) (10000 * Kp_term), (long long) (10000 * Kd_term), (long long) (10000 * velocity), (long long) (10000 * current_gain), (long long) (10000 * tablet_velocity), V_i);
//	sprintf(UT, "%ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\n\0", (long) (10000 * E_vel_deg_new), (long) (10000 * ED_mva), (long) (10000 * Encoder_vel), (long) (10000 * EV_mva), (long) (10000 * Encoder_acc), (long) (10000 * R_velocity), (long) (10000 * tablet_velocity), (long) (V_i), (long) (10000 * Encoder_deg_new));
	UART_Put_String(UT);
}

// 응답 코드 확인
interrupt void scicRxFifoIsr(void) {

   Receivedbuff = ScicRegs.SCIRXBUF.bit.RXDT;

   if (a == 0)
   {
      if (Receivedbuff == 0x90)
      {
         RxBuff[a] = Receivedbuff;
         a++;
      }
      else
      {
         RxBuff[13] = 0;
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
   else if (a == 7)
   {
      RxBuff[a] = Receivedbuff;
      a++;
   }
   else if (a == 8)
   {
      RxBuff[a] = Receivedbuff;
      a++;
   }
   else if (a == 9)
   {
      RxBuff[a] = Receivedbuff;
      a++;
   }
   else if (a == 10)
   {
      RxBuff[a] = Receivedbuff;
      a++;
   }
   else if (a == 11)
   {
      RxBuff[a] = Receivedbuff;
      a++;
      if(RxBuff[3] == 0x02){
         a = 0;
      }
   }
   else if (a == 12)
   {
      RxBuff[a] = Receivedbuff;
      a++;
   }
   else if (a == 13)
   {
      RxBuff[a] = Receivedbuff;
      a++;
      if(RxBuff[3] == 0x04){
         a = 0;
      }
   }
   sprintf(RxBuff2, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c", RxBuff[0], RxBuff[1], RxBuff[2], RxBuff[3], RxBuff[4], RxBuff[5], RxBuff[6], RxBuff[7], RxBuff[8], RxBuff[9], RxBuff[10], RxBuff[11], RxBuff[12], RxBuff[13]);

   ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;         // Clear Overflow flag
   ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;         // Clear Interrupt flag
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;      // Acknowledge interrupt to PIE
}
// RxBuff2로 응답 코드 확인

// 초기 모터 Enable
void Motor_Enable()
{
		switch(Enable_num){
		case 1:
			sprintf(uart, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x68, 0x04, 0x01, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00, 0x22, 0x99);
			MAXON_transmit();
			Enable_num = 2;
			break;
		case 2:
			sprintf(uart, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x68, 0x04, 0x01, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00, 0xb3, 0x07);
			MAXON_transmit();
			Enable_num = 3;
			break;
		case 3:
			sprintf(uart, "%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x60, 0x02, 0x01, 0x51, 0x31, 0x00, 0x51, 0x80);
			MAXON_transmit();
			Enable_num = 4;
			break;
		case 4:
			sprintf(uart, "%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x60, 0x02, 0x01, 0x51, 0x31, 0x01, 0x60, 0xb3);
			MAXON_transmit();
			Enable_num = 5;
			break;
		case 5:
			sprintf(uart, "%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x60, 0x02, 0x01, 0x51, 0x31, 0x02, 0x33, 0xe6);
			MAXON_transmit();
			Enable_num = 6;
			break;
		case 6:
			sprintf(uart, "%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x60, 0x02, 0x01, 0x51, 0x31, 0x03, 0x02, 0xd5);
			MAXON_transmit();
			Enable_num = 0;
			Enable_bit = 0;
			break;
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

		if(uart[8] == 0x90 || uart[9] == 0x90 || uart[10] == 0x90 || uart[11] == 0x90 || uart[12] == 0x90 || uart[13] == 0x90)
		{
			for(stuff_i=8;stuff_i<14;stuff_i++)
			{
				stuff_position++;

				if(uart[stuff_i] == 0x90)
				{
					uart_buff[buff_i] = 0x90;
					buff_i++;
					uart_buff[buff_i] = 0x90;
					buff_i++;

					protocol_len++;

					if(stuff_position2 == 0)
					{
						stuff_position2 = stuff_position+7;
					}
				}
				else if(uart[stuff_i] != 0x90 && stuff_position2 != 0)
				{
					uart_buff[buff_i] = uart[stuff_i];
					buff_i++;
				}
			}
			for(stuff_i = stuff_position2; stuff_i < (stuff_position2+buff_i); stuff_i++)
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
		uart[9] = (-((hexadecimal[3] << 4) | hexadecimal[2]) - 1) &0xff;
		uart[10] = (-((hexadecimal[5] << 4) | hexadecimal[4]) - 1) &0xff;
		uart[11] = (-((hexadecimal[7] << 4) | hexadecimal[6]) - 1) &0xff;

		DataArray[0] = (uart[3] << 8) + uart[2];
		DataArray[1] = (uart[5] << 8) + uart[4];
		DataArray[2] = (uart[7] << 8) + uart[6];
		DataArray[3] = (uart[9] << 8) + uart[8];
		DataArray[4] = (uart[11] << 8) + uart[10];
		DataArray[5] = 0x0000;

		CRC = CalcFieldCRC(DataArray, 6);

		uart[12] = (CRC & 0xff);
		uart[13] = (CRC & 0xff00) >> 8;

		if(uart[8] == 0x90 || uart[9] == 0x90 || uart[10] == 0x90 || uart[11] == 0x90 || uart[12] == 0x90 || uart[13] == 0x90)
		{
			for(stuff_i=8;stuff_i<14;stuff_i++)
			{
				stuff_position++;

				if(uart[stuff_i] == 0x90)
				{
					uart_buff[buff_i] = 0x90;
					buff_i++;
					uart_buff[buff_i] = 0x90;
					buff_i++;

					if(stuff_position2 == 0)
					{
						stuff_position2 = stuff_position+7;
					}
				}
				else if(uart[stuff_i] != 0x90 && stuff_position2 != 0)
				{
					uart_buff[buff_i] = uart[stuff_i];
					buff_i++;
				}
			}
			for(stuff_i = stuff_position2; stuff_i < (stuff_position2+buff_i); stuff_i++)
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

	for(i=0; i<8; i++)
	{
		decimal2hexadecimal[i] = 0;
	}

	if(decimal < 0) decimal = -decimal;

	while(1)
	{
		int mod = decimal % 16;
		decimal2hexadecimal[position] = mod;
		decimal = decimal / 16;
		position++;

		if(decimal == 0)
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

   while(ArrayLength--)
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
}

void torque_fourier_constant(double current_gain)
{
	int current_num = current_gain*10;
	double base_num = (double)(current_num) / 20;
	gain_bit = 0;
	we = w_base * base_num;
	SetDegTimer = BaseDegTimer / base_num;
}

void Timer_set() {

	DegTimer = DegTimer + 0.005;

	if(DegTimer >= SetDegTimer)
	{
		DegTimer = 0;
		if((int)(target_gain*10) == (int)(current_gain*10))
		{
			gain_bit = 1;
			current_gain = target_gain;
		}
		else if((int)(target_gain*10) > (int)(current_gain*10))
		{
			gain_bit = 1;
			current_gain = current_gain + gain_step;
		}
		else if((int)(target_gain*10) < (int)(current_gain*10))
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
		if(target_gain >= 1)	torque_interpolation = ((3-target_gain)/2) * torque_fourier_1 + ((target_gain-1)/2) * torque_fourier_3;
		else if(target_gain < 1) torque_interpolation = target_gain * torque_fourier_1;

		mass_torque = a0 + a1 * cos(Encoder_deg_new * w)  // 최대 200kg;
		+ b1 * sin(Encoder_deg_new * w)
		+ a2 * cos(2 * Encoder_deg_new * w)
		+ b2 * sin(2 * Encoder_deg_new * w)
		+ a3 * cos(3 * Encoder_deg_new * w)
		+ b3 * sin(3 * Encoder_deg_new * w);
		mass_torque = (double)mass *0.005 * mass_torque;

		if(flag == 1)
		{
			torque_interpolation = 0; // Test
			mass_torque = 0;
		}

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

		Position_error = E_vel_deg_time - E_vel_deg_new;
//		if(time_Encoder_revcnt > Encoder_revcnt) Position_error = Position_error + 360;
//		else if(time_Encoder_revcnt < Encoder_revcnt) Position_error = Position_error - 360;

//		integrator += Ki * Position_error * 0.005;
//		if(integrator >= 10)	integrator = 10;
//		else if(integrator <= -10)	integrator = -10;
		torque_buffer = (torque_interpolation + mass_torque) * torque_scale + Kp * Position_error - Kd * Encoder_vel; // + integrator;
		Kp_term = Kp*Position_error;
		Kd_term = Kd*Encoder_vel;
		torque = torque_buffer * 1000;
		if(torque <= 0)
			torque = 0;
		if(torque >= 45000)
		{
			torque = 44900;
			flag2++;
		}

		torque = torque / gear_ratio; // 감속비 60
		torque = (torque / max_motor_torque);	// 모터 정격 토크 = 0.75
		Torque_Calculate();
		MAXON_transmit();

		break;
	case 2:

		torque = 0;
		DegTimer = 0;
		Encoder_revcnt = 0;
		time_Encoder_revcnt = 0;
		Encoder_vel = 0;
		integrator = 0;
		current_gain = 1;
		gain_bit = 1;
		for(D_i = 0; i<20; i++)	ED_Buff[D_i] = 0;
		if(Encoder_deg_new >=0 && Encoder_deg_new <= 1.0) break_duty = 0;
		else break_duty = 1;
		Torque_Calculate();
		MAXON_transmit();

		break;
	case 3:

		break;
	}
}

// timer 인터럽트
interrupt void cpu_timer0_isr(void) // cpu timer 현재 제어주파수 200Hz
{
	MetabolizeRehabilitationRobot();

	if(Enable_bit)	Motor_Enable();
	if(gain_bit)	torque_fourier_constant(current_gain);

	TrainAbnormalPerson();
	OutputPWM();

	TimerCount2++;
	if (TimerCount2 == 40) {
		TimerCount2 = 0;
		if(mode_num == 1)
		{
			Uart_transmit();
		}
	}
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//============================================================================================
