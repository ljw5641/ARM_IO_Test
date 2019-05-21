#include "board.h"
#include "AT91SAM7x.h"
#include "myLIB.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <pio/pio.h>
#include <aic/aic.h>
#include <utility\trace.h>
#include <cs8900a/cs8900a.h>

//For Switch
#define LEFT	1
#define RIGHT	2

unsigned int  Key_Count=0,Pre_Key_Data=0;
unsigned char Switch_Check(void);
unsigned char Port_Flag=0;
unsigned int  Count=0; 

int ms = 0;
int s = 0;
int m = 0;
int h = 0;
//============================================================================

//  Function  : PIT Interrupt

//============================================================================

void Isr_PIT(void)
{
    volatile unsigned int pit_pivr;
	if((rPIT_SR & 1) != 0)  //The Periodic Interval timer has reached PIV since the last read of PIT_PIVR
    {
		pit_pivr = rPIT_PIVR;    //Reads Periodic Interval Timer Value Register - Clears PITS in PIT_SR
//		Count++;
//		if(Count==100)
//		{
//			Count=0;
			if(Port_Flag==0)
			{
				rPIO_SODR_B=(LED1|LED2|LED3);
 
				Port_Flag=1;
			}
			else
			{	
				rPIO_CODR_B=(LED1|LED2|LED3);
				Port_Flag=0;
			}	
//		}		
	}
}

 
void PIT_Interrupt_Setup(void) 
{
	unsigned int	tmp=0;

    rAIC_IECR = (1<<1);

	// System Advanced Interrupt Controller

    rAIC_SMR1 = (1<<5) +  (7<<0);  //Edge Trigger, Prior 7
    rAIC_SVR1 = (unsigned)Isr_PIT;

	// System Periodic Interval Timer (PIT) Mode Register (MR) 

	// PITEN(24) - Periodic Interval Timer Enabled

	// unsigned int PITEN = (1<<24);

	// PITIEN(25) - Periodic Interval Timer Interrupt Enabled
	
	// unsigned int PITIEN = (1<<25);

	// PIV(19:0) - Periodic Interval Time

	// will be compared with 20-bit CPIV (Counter of Periodic Interval Timer)

    tmp=(48000000/16/100)&0xFFFFF;         // T=30Hz

	// unsigned int PIV = 0;

	// PIV = (48000000/16/100)&0xFFFFF;

    rPIT_MR=(1<<25)+(1<<24)+(tmp<<0);      // Enable PIT, Disable Interrupt

	// rPIT_MR = PITIEN + PITEN + PIV;
}

#define ULTRASONIC_TRIGGER PA0
#define ULTRASONIC_ECHO PA1

void Port_Setup(void)
{
	// PMC (Power Management Clock) enables peripheral clocks
	AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOB );
	AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA );

	// Enable PIO in output mode: Port A 0-7
	//AT91F_PIO_CfgOutput( AT91C_BASE_PIOA,  PORTA);

	// LED (Port B: 28-30)
	AT91F_PIO_CfgOutput( AT91C_BASE_PIOB, LED1|LED2|LED3 ); // output mode
	AT91F_PIO_CfgPullup( AT91C_BASE_PIOB, LED1|LED2|LED3 ); // pull-up

	// Switch (Port A: 8,9)
	AT91F_PIO_CfgInput( AT91C_BASE_PIOA, SW1|SW2 ); // output mode
	AT91F_PIO_CfgPullup( AT91C_BASE_PIOA, SW1|SW2 ); // pull-up

	// Ultra
	AT91F_PIO_CfgOutput(AT91C_BASE_PIOA, ULTRASONIC_TRIGGER);
	AT91F_PIO_CfgInput(AT91C_BASE_PIOA, ULTRASONIC_ECHO);
	AT91F_PIO_CfgPullup(AT91C_BASE_PIOA, ULTRASONIC_TRIGGER|ULTRASONIC_ECHO ); // pull-up

//AT91F_PIO_SetOutput(AT91C_BASE_PIOA, (1<<13));
//AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, (1<<13));
}

 

/*
void Read_For_Setup_CMOS(void)
{
	//Read address Reset
	rPIO_CODR_A=FIFO_RD_RST;			
	rPIO_SODR_A=FIFO_RD;
	rPIO_CODR_A=FIFO_RD;
	rPIO_SODR_A=FIFO_RD;
	rPIO_CODR_A=FIFO_RD;
	rPIO_SODR_A=FIFO_RD_RST;
	
	//Write On		
	//rPIO_SODR_A=XCLK_ON;
	rPIO_SODR_A=HREF_SYNC;

	//Until Wait Low
	while((rPIO_PDSR_A & 0x00000004)){}
	//Until Wait High
	while(!(rPIO_PDSR_A & 0x00000004)){}

	//Write Off		
	//rPIO_CODR_A=HREF_SYNC;
	//CS_LOW
	rPIO_CODR_A=FIFO_CS;
}


void CMOS_Read_Clk(void)
{
	rPIO_SODR_A=FIFO_RD;
	rPIO_CODR_A=FIFO_RD;
}
*/

 

//-----------------------------------------------------------------------------

/// Main Procedure

//-----------------------------------------------------------------------------                                         

 

// PIT interrupt service routine

volatile unsigned int ten_us_count = 0;

void PIT_ISR()
{
	// Clear PITS
	AT91F_PITGetPIVR(AT91C_BASE_PITC);

	// increase ten_us_count
	ten_us_count++;
}

// Initialize PIT and interrupt

void PIT_initiailize()
{
	// enable peripheral clock for PIT
	AT91F_PITC_CfgPMC();

	// set the period to be every 1 msec in 48MHz
	AT91F_PITInit(AT91C_BASE_PITC, 1, 48);

	// PIV (Periodic Interval Value) = 3000 clocks = 1 msec

	// MCK/16 = 48,000,000 / 16 = 3,000,000 clocks/sec

	// = 3000 clocks / 1msec

	// = 30 clocks / 10 micro second

	AT91F_PITSetPIV(AT91C_BASE_PITC, 30-1);

	// disable PIT periodic interrupt for now

	AT91F_PITDisableInt(AT91C_BASE_PITC);

	// interrupt handler initializatioin

	AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_SYS, 7, 1, PIT_ISR);

	// enable the PIT interrupt

	AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_SYS);
}

 

 

 

/*// delay in ms by PIT interrupt

void HW_delay_ms(unsigned int ms)
{
	// special case
	if(ms == 0) return;

	// start time
	ms_count = 0;

	// enable PIT interrupt
	AT91F_PITEnableInt(AT91C_BASE_PITC);

	// wait for ms
	while(ms_count < ms);

	// disable PIT interrupt
	AT91F_PITDisableInt(AT91C_BASE_PITC);
}*/

// delay in micro seconds by PIT interrupt
void HW_delay_10us(unsigned int ten_us)
{
	// special case
	if(ten_us == 0) return;

	// start time
	ten_us_count = 0;

	// enable PIT interrupt
	AT91F_PITEnableInt(AT91C_BASE_PITC);

	// wait for ten_us
	while(ten_us_count < ten_us);

	// disable PIT interrupt
	AT91F_PITDisableInt(AT91C_BASE_PITC);
} 


// PIO interrupt service routine

void PIO_ISR()
{
	// Reset the stop watch
	ms = 0;
	s = 0;
	m = 0;
	h = 0;
}  

void Interrupt_setup()
{
	// Use SW1 as an input
	AT91F_PIO_InputFilterEnable(AT91C_BASE_PIOA, SW1);

	// set interrupt to SW1
	AT91F_PIO_InterruptEnable(AT91C_BASE_PIOA, SW1);

	// Set Callback funtion
	AT91F_AIC_ConfigureIt(AT91C_BASE_AIC,AT91C_ID_PIOA, 7, 1, PIO_ISR);

	// Enable AIC
	//AT91F_AIC_EnableIt(AT91C_BASE_AIC,AT91C_ID_PIOA);
}

void TC_initialize()
{
  // Enable peripheral clock in PMC for Timer Counter 0
  AT91F_TC0_CfgPMC();

  // Configure Timer Counter 0 with Software Trigger Effect on TIOA

  // TIMER_CLOCK1: MCK/2 = 48,000,000 / 2 = 24,000,000 clocks/sec = 24 clocks/us
  // TIMER_CLOCK3: MCK/32 = 48,000,000 / 32 = 1,500,000 clocks/sec = 1.5 clock/us

  //TC_Configure(AT91C_BASE_TC0, AT91C_TC_CLKS_TIMER_DIV1_CLOCK | AT91C_TC_ASWTRG); 
  TC_Configure(AT91C_BASE_TC0, AT91C_TC_CLKS_TIMER_DIV3_CLOCK | AT91C_TC_ASWTRG); 
}


int main()
{
  int n =0;

  // Port set up
  Port_Setup();

  // UART 
  DBG_Init();
  Uart_Printf("Ultrasound - Test\n\r");

  // PIT setup
  PIT_initiailize();

  // Timer counter
  TC_initialize();

  while(1) 
  {
    Uart_Printf("iter = %d : ", n);

    //LEC ON
    //rPIO_SODR_B=(LED1|LED2|LED3);

	//[Polling]
	rPIO_CODR_A=(ULTRASONIC_TRIGGER|ULTRASONIC_ECHO);

	//[1] set trigger pin (PA0) on
	rPIO_SODR_A=(ULTRASONIC_TRIGGER);

	//[2] wait for 10us
	HW_delay_10us(1);

	//[3] set trigger pin, echo pin off
	rPIO_CODR_A=(ULTRASONIC_TRIGGER);


	//[4] listen to Echo pin (PA1) if it's on 
	while(1)
	{
		if(AT91F_PIO_IsInputSet(AT91C_BASE_PIOA,ULTRASONIC_ECHO))
		{
			break;
		}
	}

	//[5] start the timer
	TC_Start(AT91C_BASE_TC0);

	//[6] listen to Echo pin if it's off
	while(1)
	{
		if(!AT91F_PIO_IsInputSet(AT91C_BASE_PIOA,ULTRASONIC_ECHO))
		{
			break;
		}
	}
	
	//[7] stop the timer
	TC_Stop(AT91C_BASE_TC0);
	
	//[8] TC_CV -> cm
	// TC_CV = 24 clocks / us
	// Ultrasonic = 1cm / 58us
	// 58/24 cm / clock
	//Uart_Printf("\t%lf cm\n\r", (double)(AT91C_BASE_TC0->TC_CV)/(58.0*24.0)); // TIMER_CLOCK1

	// TC_CV = 1.5 clocks / us
	// Ultrasonic = 1cm / 58us
	// 1/(1.5*58) cm / clock
	Uart_Printf("\t%lf cm\n\r", (double)(AT91C_BASE_TC0->TC_CV)/(58.0*1.5)); // TIMER_CLOCK3

	

	
	

    // 타이머시작
    /*TC_Start(AT91C_BASE_TC0);

	// 딜레이
    HW_delay_10us(1);

    // 타이머스탑
    TC_Stop(AT91C_BASE_TC0);

    //오버플로우?
    if (AT91C_BASE_TC0-> TC_SR & AT91C_TC_COVFS)
    {
      Uart_Printf("Overflow - ");
    }
    else
    {
      Uart_Printf("Normal - ");
    }

    // 타이머값
    Uart_Printf("\tStop TC1 = %u clocks = %lf ms\n\r", AT91C_BASE_TC0->TC_CV , (double)AT91C_BASE_TC0->TC_CV/48000.0); //\tStop , (double)AT91C_BASE_TC0->TC_CV/48000.0

    rPIO_CODR_B=(LED1|LED2|LED3);
    */
    HW_delay_10us(50000);
    n++;
  } 
}