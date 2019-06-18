/*
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

unsigned int 	Key_Count=0,Pre_Key_Data=0;
unsigned char Switch_Check(void);
unsigned char Port_Flag=0;
unsigned int	Count=0; 
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
void 	PIT_Interrupt_Setup(void) 
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


void Port_Setup(void)
{
	// PMC (Power Management Clock) enables peripheral clocks
	AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOB );
	AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA );
	
	// Enable PIO in output mode: Port A 0-7
	AT91F_PIO_CfgOutput( AT91C_BASE_PIOA,  PORTA);

	// LED (Port B: 28-30)
	AT91F_PIO_CfgOutput( AT91C_BASE_PIOB, LED1|LED2|LED3 ); // output mode
	AT91F_PIO_CfgPullup( AT91C_BASE_PIOB, LED1|LED2|LED3 ); // pull-up

	// Switch (Port A: 8,9)
	AT91F_PIO_CfgInput( AT91C_BASE_PIOA, SW1|SW2 ); // output mode
	AT91F_PIO_CfgPullup( AT91C_BASE_PIOA, SW1|SW2 ); // pull-up

//AT91F_PIO_SetOutput(AT91C_BASE_PIOA, (1<<13));
//AT91F_PIO_ClearOutput(AT91C_BASE_PIOA, (1<<13));
	
}

unsigned char Switch_Check(void)
{
unsigned char Result=0;

	if(!(rPIO_PDSR_A & SW1)) Result=LEFT;
	else if(!(rPIO_PDSR_A & SW2)) Result=RIGHT;
	
	
	if(Pre_Key_Data==Result) Key_Count++;
	else Key_Count=0;
	
	Pre_Key_Data=Result;
	return	Result;
}
//-----------------------------------------------------------------------------
/// Main Procedure
//-----------------------------------------------------------------------------
                   
int main()
{
	int i = 0;
  	Port_Setup();
	
	while(1) 
	{
		// LED off
		rPIO_CODR_B=(LED1);
		for(i = 0; i < 10; ++i) Delay(100000);

		// LED on
		rPIO_SODR_B=(LED1);
		for(i = 0; i < 10; ++i) Delay(100000);
	}	
}*/


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
#include <pwmc/pwmc.h>

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

/*void Interrupt_setup()
{
	// Use SW1 as an input
	AT91F_PIO_InputFilterEnable(AT91C_BASE_PIOA, SW1);
	// set interrupt to SW1
	AT91F_PIO_InterruptEnable(AT91C_BASE_PIOA, SW1);
	// Set Callback funtion
	AT91F_AIC_ConfigureIt(AT91C_BASE_AIC,AT91C_ID_PIOA, 7, 1, PIO_ISR);
	// Enable AIC
	//AT91F_AIC_EnableIt(AT91C_BASE_AIC,AT91C_ID_PIOA);
} */


__inline void set_SMR (
	AT91PS_AIC pAic,  // \arg pointer to the AIC registers
	unsigned int irq_id,     // \arg interrupt number to initialize
	unsigned int priority,   // \arg priority to give to the interrupt
	unsigned int src_type)   // \arg activation and sense of activation
{
    unsigned int mask ;

    mask = 0x1 << irq_id ;
    
    //* Disable the interrupt on the interrupt controller
    pAic->AIC_IDCR = mask ;
    
    //* Store the Source Mode Register
    pAic->AIC_SMR[irq_id] = src_type | priority  ;
    
    //* Clear the interrupt on the interrupt controller
    pAic->AIC_ICCR = mask ;
}

void Ultra_Interrupt_setup();
int mode = 0;
int trigger_ultrasonic = 0;
void PIO_ISR()
{
	// Reset the stop watch
	/*ms = 0;
	s = 0;
	m = 0;
	h = 0;*/
	unsigned int priority = 7;
	
	if(mode == 0)
	{
		trigger_ultrasonic = 0;
		Uart_Printf("PIO_ISR - mode == 0 - Echo on\n");
		
	    AT91F_AIC_DisableIt(AT91C_BASE_AIC,AT91C_ID_PIOA);
		Uart_Printf("PIO_ISR - mode == 0 - AT91F_AIC_DisableIt\n");
		
		TC_Start(AT91C_BASE_TC0);
		Uart_Printf("PIO_ISR - mode == 0 - TC_Start\n");
		
		rAIC_SMR2 = (AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE|priority);
		Uart_Printf("PIO_ISR - mode == 0 - AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE\n");
		Uart_Printf("%i\n",rAIC_SMR2);
		mode = 1;
		Uart_Printf("PIO_ISR - mode == 0 - mode\n");
		
		AT91F_AIC_EnableIt(AT91C_BASE_AIC,AT91C_ID_PIOA);
		Uart_Printf("PIO_ISR - mode == 0 - AT91F_AIC_EnableIt\n");
	}
	else if(mode == 1)
	{
		Uart_Printf("PIO_ISR - mode == 1 - Echo off\n");
		
	    AT91F_AIC_DisableIt(AT91C_BASE_AIC,AT91C_ID_PIOA);
		Uart_Printf("PIO_ISR - mode == 1 - AT91F_AIC_DisableIt\n");
		
		TC_Stop(AT91C_BASE_TC0);
		Uart_Printf("PIO_ISR - mode == 1 - TC_Stop\n");
		
		Uart_Printf("\t%lf cm\n\r", (double)(AT91C_BASE_TC0->TC_CV)/(58.0*1.5));
		
		rAIC_SMR2 = (AT91C_AIC_SRCTYPE_POSITIVE_EDGE|priority);
		//rAIC_SMR2 = ((unsigned int)1|priority);
		//rAIC_SMR2 = (1|priority);
		//Ultra_Interrupt_setup();
		//set_SMR(AT91C_BASE_AIC, AT91C_ID_PIOA, priority, AT91C_AIC_SRCTYPE_POSITIVE_EDGE);
		Uart_Printf("PIO_ISR - AT91C_AIC_SRCTYPE_POSITIVE_EDGE\n");

		AT91F_AIC_EnableIt(AT91C_BASE_AIC,AT91C_ID_PIOA);
		Uart_Printf("PIO_ISR - mode == 1 - AT91F_AIC_EnableIt\n");
		
		mode = 0;
		Uart_Printf("PIO_ISR - mode = 0\n");
		
		trigger_ultrasonic = 1;
		Uart_Printf("PIO_ISR - mode == 1 - trigger_ultrasonic\n");
		
		/*
		//HW_delay_10us(6000);
		//Uart_Printf("PIO_ISR - HW_delay_10us(6000)\n");
		
		//[1] set trigger pin (PA0) on
  		rPIO_SODR_A=(ULTRASONIC_TRIGGER);
		Uart_Printf("PIO_ISR - rPIO_SODR_A\n");
  		//[2] wait for 10us
  		HW_delay_10us(1);
		Uart_Printf("PIO_ISR - HW_delay_10us(1)\n");
  		//[3] set trigger pin, echo pin off6
  		rPIO_CODR_A=(ULTRASONIC_TRIGGER);
		Uart_Printf("PIO_ISR - mode == 1 - rPIO_CODR_A\n");
		*/
	}
} 

void Ultra_Interrupt_setup()
{  	
	unsigned int priority = 7;
	
	Uart_Printf("Ultra_Interrupt_setup - 0\n");
	AT91F_AIC_DisableIt(AT91C_BASE_AIC,AT91C_ID_PIOA);
	
	// Use SW1 as an input
  	Uart_Printf("Ultra_Interrupt_setup - 1\n");
	AT91F_PIO_InputFilterEnable(AT91C_BASE_PIOA, ULTRASONIC_ECHO);

	// set interrupt to SW1
  	Uart_Printf("Ultra_Interrupt_setup - 2\n");
	AT91F_PIO_InterruptEnable(AT91C_BASE_PIOA, ULTRASONIC_ECHO);

	// Set Callback funtion
  	Uart_Printf("Ultra_Interrupt_setup - 3\n");
	//AT91F_AIC_ConfigureIt(AT91C_BASE_AIC,AT91C_ID_PIOA, priority, AT91C_AIC_SRCTYPE_POSITIVE_EDGE, PIO_ISR);
	AT91F_AIC_ConfigureIt(AT91C_BASE_AIC, AT91C_ID_PIOA, priority, 1, PIO_ISR);

	// Enable AIC
  	Uart_Printf("Ultra_Interrupt_setup - 4\n");
	AT91F_AIC_EnableIt(AT91C_BASE_AIC, AT91C_ID_PIOA);
	
  	Uart_Printf("Ultra_Interrupt_setup - 5\n");
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


#define PWM_FREQUENCY    20000
#define MAX_DUTY_CYCLE   50
#define MIN_DUTY_CYCLE   0
#define CHANNEL_PWM_LED1 1
#define CHANNEL_PWM_LED2 2
#define PIN_PWMC_LED1  {LED1, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT}

// B 19,20,21,22 / 27,28,29,30
const Pin pins[] =
{
	PINS_DBGU,
	PIN_PWMC_LED1,
};

int i=0;
int main()
{
	
	
	while(true)
	{
	
		// PIO Set up
		PIO_Configure(pins, PIO_LISTSIZE(pins));
	
		// Enable PWMC
		AT91F_PWMC_CfgPMC();
	
		// Clock Setting
		PWMC_ConfigureClocks(PWM_FREQUENCY*MAX_DUTY_CYCLE, 0, BOARD_MCK);
	
		// Channel
		PWMC_ConfigureChannel(CHANNEL_PWM_LED1, AT91C_PWMC_CPRE_MCKA, 0, 0);
	
	
		// Period
		PWMC_SetPeriod(CHANNEL_PWM_LED1, MAX_DUTY_CYCLE);
	
		for(i=MAX_DUTY_CYCLE;i<=MIN_DUTY_CYCLE;i--)
		{
			// Duty cycle
			PWMC_SetDutyCycle(CHANNEL_PWM_LED1,i);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
		}
		
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		
		for(i=MIN_DUTY_CYCLE;i<=MAX_DUTY_CYCLE;i++)
		{
			// Duty cycle
			PWMC_SetDutyCycle(CHANNEL_PWM_LED1,i);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
			for(i = 0; i < 10; ++i) Delay(100000);
		}
		
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		for(i = 0; i < 10; ++i) Delay(100000);
		
	
		// Enable Channel
		PWMC_EnableChannel(CHANNEL_PWM_LED1);
	}
	
	return 0;
}


