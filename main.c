#include "DSP28x_Project.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "variable.h"

#pragma CODE_SECTION(MIR_transmit,"ramfuncs")
#pragma CODE_SECTION(MIR_Put_String,"ramfuncs")

// CPU timer0 선언 //
interrupt void cpu_timer0_isr(void);

//------------------함수------------------//
void MetabolizeRehabilitationRobot();
void Reg_setting_fun();

// 통신 설정을 위한 함수 //
void scic_echoback_init(void);
void scic_fifo_init(void);

// Motor를 LCD 없이 동작시키기 위한 통신
void MIR_transmit();
void MIR_Put_String(char *MIR_string);
void Motor_Enable();
unsigned short* decimal2hex(int torque);
unsigned short CalcFieldCRC(unsigned short* pDataArray, unsigned short ArrayLength);

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
   scic_fifo_init();      // Initialize the SCI FIFO
   scic_echoback_init();  // Initalize SCI for echoback

   for (i = 0; i < 100; i++)
      MIR1[i] = 0;

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

// 메인함수 끝.

//함수시작
void Reg_setting_fun() {
   EALLOW;
   PieVectTable.TINT0 = &cpu_timer0_isr;
   PieVectTable.SCIRXINTC = &scicRxFifoIsr;

   SysCtrlRegs.HISPCP.bit.HSPCLK = 1;

   //led 제어 사용 설정
   GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;
   GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;

   //led 제어 사용 설정
   GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;
   GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;

   EDIS;
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

void MIR_Put_String(char *MIR_string) {
   for(len = 0; len<protocol_len; len++) {
      ScicRegs.SCITXBUF = *MIR_string++;
      while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {
      }
   }

}

void MIR_transmit() {
   MIR_Put_String(MIR1);
}

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

   sprintf(RxBuff2, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", RxBuff[0], RxBuff[1], RxBuff[2], RxBuff[3], RxBuff[4], RxBuff[5], RxBuff[6], RxBuff[7], RxBuff[8], RxBuff[9], RxBuff[10], RxBuff[11], RxBuff[12], RxBuff[13]);

   ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;         // Clear Overflow flag
   ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;         // Clear Interrupt flag
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;      // Acknowledge interrupt to PIE

}

void MetabolizeRehabilitationRobot() {

   ++TimerCount; //40 -> 2
   ++TimerCount_2;

   //속도, 토크값 컴에서확인
   if (TimerCount_2 == 2) {
      TimerCount_2 = 0;
            MIR_transmit();
   }
   // MATLAB 2 -> 100Hz Bluetooth 40 -> 5Hz
   if (TimerCount == 40) {
      TimerCount = 0;
      Flash_bit=!Flash_bit;

      GpioDataRegs.GPBDAT.bit.GPIO48 = Flash_bit;
   }
}

void Motor_Enable()
{
	switch(Enable_num){
	case 1:
	       sprintf(MIR1, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x68, 0x04, 0x01, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00, 0x22, 0x99);
	       MIR_transmit();
	       Enable_num = 2;
	       break;
	case 2:
		   sprintf(MIR1, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x68, 0x04, 0x01, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00, 0xb3, 0x07);
		   MIR_transmit();
		   Enable_num = 3;
		   break;
	case 3:
		   sprintf(MIR1, "%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x60, 0x02, 0x01, 0x51, 0x31, 0x00, 0x51, 0x80);
		   MIR_transmit();
		   Enable_num = 4;
		   break;
	case 4:
		   sprintf(MIR1, "%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x60, 0x02, 0x01, 0x51, 0x31, 0x01, 0x60, 0xb3);
		   MIR_transmit();
		   Enable_num = 5;
		   break;
	case 5:
		   sprintf(MIR1, "%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x60, 0x02, 0x01, 0x51, 0x31, 0x02, 0x33, 0xe6);
		   MIR_transmit();
		   Enable_num = 6;
		   break;
	case 6:
		   sprintf(MIR1, "%c%c%c%c%c%c%c%c%c%c", 0x90, 0x02, 0x60, 0x02, 0x01, 0x51, 0x31, 0x03, 0x02, 0xd5);
		   MIR_transmit();
		   Enable_num = 0;
		   break;
	}
}

void Torque_Calculate()
{
	unsigned short* hexadecimal = decimal2hex(torque);
	unsigned short DataArray[6];
	unsigned short CRC = 0;

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
		uart[13] = (CRC & (0xff << 8)) >> 8;

		if(uart[8] == 0x90 || uart[9] == 0x90 || uart[10] == 0x90 || uart[11] == 0x90)
		{
			for(stuff_i=8;stuff_i<12;stuff_i++)
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
				else if(uart[stuff_i] != 0x90)
				{
					uart_buff[buff_i] = uart[stuff_i];
					buff_i++;
				}
			}
			for(stuff_i = 13; stuff_i > 11; stuff_i--)
			{
				uart[stuff_i+(buff_i-4)] = uart[stuff_i];
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
		torque = -torque;

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
		uart[13] = (CRC & (0xff << 8)) >> 8;

		if(uart[8] == 0x90 || uart[9] == 0x90 || uart[10] == 0x90 || uart[11] == 0x90)
		{
			for(stuff_i=8;stuff_i<12;stuff_i++)
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
				else if(uart[stuff_i] != 0x90)
				{
					uart_buff[buff_i] = uart[stuff_i];
					buff_i++;
				}
			}
			for(stuff_i = 13; stuff_i > 11; stuff_i--)
			{
				uart[stuff_i+(buff_i-4)] = uart[stuff_i];
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

unsigned short* decimal2hex(int torque)
{
	unsigned short hexadecimal[8] = {0, };
	int position = 0;
	int decimal = torque;

	while(1)
	{
		int mod = decimal % 16;
		hexadecimal[position] = mod;
		decimal = decimal / 16;
		position++;

		if(decimal == 0)
			break;
	}

	return hexadecimal;
}
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

interrupt void cpu_timer0_isr(void) // cpu timer 현재 제어주파수 100Hz
{

//   MetabolizeRehabilitationRobot();

	Motor_Enable();

	Torque_Calculate();

   if(flag == 1)
   {
      sprintf(MIR1, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c", uart[0], uart[1], uart[2], uart[3], uart[4], uart[5], uart[6], uart[7], uart[8], uart[9], uart[10], uart[11], uart[12], uart[13]);
      MIR_transmit();
      flag = 0;
   }

   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//============================================================================================
