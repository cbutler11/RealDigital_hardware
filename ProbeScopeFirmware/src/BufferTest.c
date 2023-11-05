/*
 * Copyright 2022 NXP
 * NXP confidential.
 * This software is owned or controlled by NXP and may only be used strictly
 * in accordance with the applicable license terms.  By expressly accepting
 * such terms or by downloading, installing, activating and/or otherwise using
 * the software, you are agreeing that you have read, and that you agree to
 * comply with and are bound by, such license terms.  If you do not agree to
 * be bound by the applicable license terms, then you may not retain, install,
 * activate or otherwise use the software.
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

//#if defined (__MULTICORE_MASTER_SLAVE_M0APP) | defined (__MULTICORE_MASTER_SLAVE_M0SUB)
//#include "cr_start_m0.h"
//#endif

#include <cr_section_macros.h>

uint32_t* sample_pointer;

//********** Circular Buffer header and global variables **********//
#include "SampleBuffer.h"

struct CircularBuffer* samples;

//********** USB Rom header and variables **********//
#include <stdio.h>
#include <string.h>
#include "libusbdev.h"

//********** HSADC variables **********//
#define HSADC_CLOCK_RATE (80 * 1000000)
#define HSADC_DESC_NOPOWERDOWN 0

//********** GPDMA definitions and variables **********//
#include "GPDMA.h"

//#define FREQTIMER0M0 1000
//#define PACKET_BUFFER_SIZE 1024//4096
//#define LUSB_DATA_PENDING _BIT(0)

#define DMA_CH 7
//uint8_t ch_no;
//#define VADC_DMA_WRITE  7
//#define VADC_DMA_READ   8

//static uint8_t g_rxBuff[PACKET_BUFFER_SIZE];
static uint8_t g_txBuff[8192]; //4096
static uint8_t g_txBuff1[8192];
static uint8_t g_txBuff2[8192];
static uint8_t g_txBuff3[8192];
//volatile uint32_t samplevector[32000];
volatile uint32_t numsamples = 0;
volatile bool unhandledReq = false;
volatile bool usbDumpConnected = false;
volatile uint8_t pendingChunks = 0;
volatile uint8_t count = 0;

//********** ADC functions **********//
/* Base clock lookup table used for finding best HSADC clock rate. Only the
   base clock sources in this table are used as a possible source for the
   HSASC peripheral clock before being divided. Add or remove sources as
   needed, but an exact frequency may not be available unless it's set up
   on an unused PLL or clock input. */
static const CHIP_CGU_CLKIN_T adcBaseClkSources[] = {
	CLKIN_IRC,			/* Usually 12MHz */
	CLKIN_CLKIN,		/* External clock in rate */
	CLKIN_CRYSTAL,		/* Usually 12MHz */
	CLKIN_AUDIOPLL,		/* Unknown, will be 0 if not configured */
	CLKIN_MAINPLL		/* Usually 204MHz, may be too fast to use with a divider */
};

/* Periodic sample rate in Hz */
#define SAMPLERATE (5)

uint32_t freqHSADC = 0;
uint32_t stored_last_0 = 0;

void setupADC()
{

//**************************************** Our Newly constructed ADC Code ****************************************//

	//configure HSADC clock to 80MHz
	Chip_USB0_Init();
	Chip_Clock_SetDivider(CLK_IDIV_A,CLKIN_USBPLL,2); //480 MHz -> 240 MHz
	Chip_Clock_SetDivider(CLK_IDIV_B,CLKIN_IDIVA,3);  //240 MHz -> 80 MHz
	Chip_Clock_SetBaseClock(CLK_BASE_ADCHS, CLKIN_IDIVB, true, false); /* Source ADHCS base clock from DIV_B */
	freqHSADC = Chip_HSADC_GetBaseClockRate(LPC_ADCHS);
	Chip_Clock_EnableOpts(CLK_MX_ADCHS, true, true, 1);
	Chip_Clock_Enable(CLK_ADCHS);

	/* Show the actual HSADC clock rate */
	freqHSADC = Chip_HSADC_GetBaseClockRate(LPC_ADCHS);

	//disable attenuator on AI_1
//	SET_PIN(PIN_ATT_AI1_5K,0);
//	SET_PIN(PIN_ATT_AI1_10K,0);

	//Reset all Interrupts
	NVIC_DisableIRQ(ADCHS_IRQn);
	LPC_ADCHS->INTS[0].CLR_EN = 0x7f; // disable Interrupt0
	LPC_ADCHS->INTS[0].CLR_STAT = 0x7f; // clear Interrupt-Status
	while(LPC_ADCHS->INTS[0].STATUS & 0x7d); // wait for status to clear, have to exclude FIFO_EMPTY (bit 1)

	LPC_ADCHS->INTS[1].CLR_EN = 0x7f;
	LPC_ADCHS->INTS[1].CLR_STAT = 0x7f;
	while(LPC_ADCHS->INTS[1].STATUS & 0x7d); // wait for status to clear, have to exclude FIFO_EMPTY (bit 1)

	// Make sure the HSADC is not powered down
	LPC_ADCHS->POWER_DOWN = (0 << 0);        /* PD_CTRL:      0=disable power down, 1=enable power down */

	// Clear FIFO
	LPC_ADCHS->FLUSH = 1;

	// FIFO Settings      0= 1 sample packed into 32 bit, 1= 2 samples packed into 32 bit */
	LPC_ADCHS->FIFO_CFG = (1 << 0) | /* PACKED */
						  (8 << 1);  /* FIFO_LEVEL */

	//select table 0, descriptor 1
	LPC_ADCHS->DSCR_STS = (1 << 1)| 0;

	// Select both positive and negative DC biasing for input 2
	Chip_HSADC_SetACDCBias(LPC_ADCHS, 2, HSADC_CHANNEL_DCBIAS, HSADC_CHANNEL_NODCBIAS);

	LPC_ADCHS->THR[0] = 0x000 << 0 | 0xFFF << 16;//Default
	LPC_ADCHS->THR[1] = 0x000 << 0 | 0xFFF << 16;//Default


	LPC_ADCHS->CONFIG =  /* configuration register */
	    (1 << 0) |       /* TRIGGER_MASK:     0=triggers off, 1=SW trigger, 2=EXT trigger, 3=both triggers */
	    (0 << 2) |       /* TRIGGER_MODE:     0=rising, 1=falling, 2=low, 3=high external trigger */
	    (0 << 4) |       /* TRIGGER_SYNC:     0=no sync, 1=sync external trigger input */
	    (0 << 5) |       /* CHANNEL_ID_EN:    0=don't add, 1=add channel id to FIFO output data */
	    (0x90 << 6);     /* RECOVERY_TIME:    ADC recovery time from power down, default is 0x90 */

	/* Setup data format for 2's complement and update clock settings. This function
	   should be called whenever a clock change is made to the HSADC */
	Chip_HSADC_SetPowerSpeed(LPC_ADCHS, false);

	//use two descriptors to be able to run at the full 80MSPS
	/* Set descriptor 0 to take a measurement at every clock and branch to itself*/
	LPC_ADCHS->DESCRIPTOR[0][0] = (1 << 24) /* RESET_TIMER*/
							    | (0 << 22) /* THRESH*/
							    | (0 << 8) /* MATCH*/
							    | (1 << 6) /* BRANCH to First*/;
	/* Set descriptor 1 to take a measurement after 0x9A clocks and branch to first descriptor*/
	LPC_ADCHS->DESCRIPTOR[0][1] = (1 << 31) /* UPDATE TABLE*/
							    | (1 << 24) /* RESET_TIMER*/
							    | (0 << 22) /* THRESH*/
							    | (0x9A << 8) /* MATCH*/
							    | (0x01 << 6) /* BRANCH to first*/;

	//Enable HSADC power
	Chip_HSADC_EnablePower(LPC_ADCHS);

	// Enable interrupts
	NVIC_EnableIRQ(ADCHS_IRQn);
//	Chip_HSADC_UpdateDescTable(LPC_ADCHS, 0);

//	//clear interrupt stats
//	Chip_HSADC_ClearIntStatus(LPC_ADCHS, 0, Chip_HSADC_GetEnabledInts(LPC_ADCHS, 0));
//	Chip_HSADC_EnableInts(LPC_ADCHS, 0 ,(HSADC_INT0_DSCR_DONE | HSADC_INT0_FIFO_FULL));

	//start HSADC
//	Chip_HSADC_SWTrigger(LPC_ADCHS);
}

//********** DMA functions **********//
void setupGPDMA()
{
//	GPDMA_init();
//	GPDMA_InitADV(sample_pointer);
	//**************************************** Our DMA Code ****************************************//
	Chip_GPDMA_Init(LPC_GPDMA);

	NVIC_DisableIRQ(DMA_IRQn);

	// Clear all DMA interrupt and error flag
	LPC_GPDMA->INTTCCLEAR = 0xFF; //clears channel terminal count interrupt
	LPC_GPDMA->INTERRCLR = 0xFF;  //clears channel error interrupt.

	LPC_GPDMA->CONFIG =   0x01;
	while( !(LPC_GPDMA->CONFIG & 0x01) );

	for (int i = 0; i < 6; i++) {};
	// Set source and destination address
	LPC_GPDMA->CH[DMA_CH].SRCADDR  =  (uint32_t) &LPC_ADCHS->FIFO_OUTPUT[0];
	LPC_GPDMA->CH[DMA_CH].DESTADDR = sample_pointer;


	//Provide the GPDMA with a linked list to facilitate transfer
	LPC_GPDMA->CH[DMA_CH].LLI = 0;

	LPC_GPDMA->CH[DMA_CH].CONTROL  = 4094   // transfer size
								   | (0x0 << 12)  // src burst size // use 2 or 3?
								   | (0x0 << 15)  // dst burst size // use 2 or 3?
								   | (0x2 << 18)  // src transfer width
								   | (0x2 << 21)  // dst transfer width
								   | (0x1 << 24)  // src AHB master select (24)
								   | (0x0 << 25)  // dst AHB master select (25)
								   | (0x0 << 26)  // src increment: 0, src address not increment after each trans
								   | (0x1 << 27)  // dst increment: 1, dst address     increment after each trans
								   | (0x1 << 31); // terminal count interrupt enable bit: 1, enabled

	LPC_GPDMA->CH[DMA_CH].CONFIG   = (0x8 << 1)   // src peripheral: set to 8   - HSADC
								   | (0x0 << 6)   // dst peripheral: no setting - memory
								   | (0x6 << 11)  // flow control: peripheral to memory - DMA control
								   | (0x2 << 14)  // IE  - interrupt error mask
								   | (0x1 << 15)  // ITC - terminal count interrupt mask
								   | (0x0 << 16)  // lock: when set, this bit enables locked transfer
								   | (0x1 << 18); // Halt: 1, enable DMA requests; 0, ignore further src DMA req

//	//Enable Interrupt for DMA
	NVIC_SetPriority(DMA_IRQn,0x00);
	NVIC_ClearPendingIRQ(DMA_IRQn);
	NVIC_EnableIRQ(DMA_IRQn);
//
//	//Enable Channel
	LPC_GPDMA->CH[DMA_CH].CONFIG |= (0x1 << 0);
}

void startSampling()
{
	//start HSADC
	Chip_HSADC_SWTrigger(LPC_ADCHS);



	for (int i = 0; i < 8; i++) {

			LPC_GPDMA->CH[DMA_CH].DESTADDR = sample_pointer + 4094*i;
			GPDMA_ADC2Mtransfer(sample_pointer + 4094*i, 1);

			//start DMA
			LPC_GPDMA->CH[DMA_CH].CONFIG = (0x1 << 0); // enable bit, 1 enable, 0 disable

			// wait for DMA transfer to complete
			while(LPC_GPDMA->INTTCSTAT == 1);


	}



	Chip_HSADC_FlushFIFO(LPC_ADCHS);
	uint32_t sts = Chip_HSADC_GetFIFOLevel(LPC_ADCHS);
	Chip_HSADC_DeInit(LPC_ADCHS); //shut down HSADC
	Chip_GPDMA_DeInit(LPC_GPDMA); //shut down GPDMA
}

//********** USBD ROM functions **********//
static void setupUSB()
{
	libusbdev_init(USB_STACK_MEM_BASE, USB_STACK_MEM_SIZE);

	// wait until host is connected
	while (libusbdev_Connected() == 0)
	{
		// Sleep until next IRQ happens
		__WFI();
	}
}

//********** Device basic functions **********//
static void setupHardware()
{
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();

	// Initializes GPIO
	Chip_GPIO_Init(LPC_GPIO_PORT);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 7);

	// Initialize USB
	setupUSB();

	// Initialize ADC
	setupADC();

	//samples = (struct CircularBuffer*) malloc(sizeof(struct CircularBuffer));
	sample_pointer = (uint32_t*) malloc(32752*sizeof(uint32_t));

	setupGPDMA();
	//cBufferInit(samples, 32000);
}

static void sendSample()
{
	for (int i = 0; i < 4094; i++)
	{
		while (libusbdev_QueueSendDone() != 0)
		{
			//wait for last send to finish
			__WFI();
		}
		//send data point
		libusbdev_QueueSendReq((uint8_t *) &(samples->buffer[(samples->head + i) % 32000]), 32);
	}
}

void samplingADC(void)
{
	LPC_ADCHS->DSCR_STS = ((0 << 1)
						| (0 << 0)); //Descriptor table 0
	LPC_ADCHS->TRIGGER = 1; //SW trigger ADC
}

//********** Interrupt service Handler Functions **********//
void TIMER0_IRQHandler(void) //Not Used
{
	if (Chip_TIMER_MatchPending(LPC_TIMER0, 0))
	{
		LPC_TIMER0->IR = TIMER_IR_CLR(0);
		LPC_ADCHS->TRIGGER = 1;
	}
}

void ADCHS_IRQHandler(void) //Not Used
{
	uint32_t sts;
	// Get ADC interrupt status on group 0 (TEST)
	sts = Chip_HSADC_GetIntStatus(LPC_ADCHS, 0) & Chip_HSADC_GetEnabledInts(LPC_ADCHS, 0);
	// Clear group 0 interrupt statuses
	Chip_HSADC_ClearIntStatus(LPC_ADCHS, 0, sts);
}

void DMA_IRQHandler(void)
{
	__asm("nop");
	//stop hsadc
	LPC_GPDMA-> CH [DMA_CH].CONFIG&=~(0x1 << 0);

}

//********** Main **********//
int main(void)
{

#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
#if defined (__MULTICORE_MASTER) || defined (__MULTICORE_NONE)
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
#endif
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif

    // Start M0APP slave processor
#if defined (__MULTICORE_MASTER_SLAVE_M0APP)
    cr_start_m0(SLAVE_M0APP,&__core_m0app_START__);
#endif

    // Start M0SUB slave processor
#if defined (__MULTICORE_MASTER_SLAVE_M0SUB)
    cr_start_m0(SLAVE_M0SUB,&__core_m0sub_START__);
#endif

    // TODO: insert code here
    setupHardware();
    LPC_GPDMA->SYNC = 1;

    //add line for usb read request user input prompt run once mode
	samplingADC();
	startSampling();
	//USB send
	sendSample();

    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
        i++ ;
        // "Dummy" NOP to allow source level single
        // stepping of tight while() loop
        __asm volatile ("nop");
    }
    return 0 ;
}
