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



int run;

//********** pins for GPIO components: ************//

//uint8_t* MCU_SC_GAIN = LPC_GPIO_PORT->B[0][12];
//uint8_t* MCU_SC_DC_n = LPC_GPIO_PORT->B[0][13];
//uint8_t* MCU_SC_DC_p = LPC_GPIO_PORT->B[0][15];

//********** USB read in Buffer *******************//

uint8_t usb_in[8];

//********** Arrays for storing samples ***********//

uint8_t sample_pointer0[16380];
uint8_t sample_pointer1[16380];
uint8_t sample_pointer2[16380];
uint8_t sample_pointer3[16380];
uint8_t sample_pointer4[16380];
uint8_t sample_pointer5[16380];
//uint8_t sample_pointer6[16380];
//uint8_t sample_pointer7[16380];

//********** USB Rom header and variables **********//
#include <stdio.h>
#include <string.h>
#include "libusbdev.h"

//********** HSADC variables **********//
#define HSADC_CLOCK_RATE (80 * 1000000)
#define HSADC_DESC_NOPOWERDOWN 0

//********** GPDMA definitions and variables **********//
#include "GPDMA.h"
#define DMA_CH 7

static GPDMA_vector DMA_List[4];

volatile uint32_t numsamples = 0;
volatile bool unhandledReq = false;
volatile bool usbDumpConnected = false;
volatile uint8_t pendingChunks = 0;
volatile uint8_t count = 0;

//********** ADC functions **********//

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
//	Chip_Clock_SetDivider(CLK_IDIV_C,CLKIN_IDIVB,2); //80MHz -> 40 MHz
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
	Chip_HSADC_SetACDCBias(LPC_ADCHS, 0, HSADC_CHANNEL_DCBIAS, HSADC_CHANNEL_NODCBIAS);

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
	Chip_HSADC_SetPowerSpeed(LPC_ADCHS, true);

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

	DMA_List[0].DstAddr = (uint32_t) sample_pointer0;
	DMA_List[1].DstAddr = (uint32_t) sample_pointer1;
	DMA_List[2].DstAddr = (uint32_t) sample_pointer2;
	DMA_List[3].DstAddr = (uint32_t) sample_pointer3;
	DMA_List[4].DstAddr = (uint32_t) sample_pointer4;
	DMA_List[5].DstAddr = (uint32_t) sample_pointer5;
//	DMA_List[6].DstAddr = (uint32_t) sample_pointer6;
//	DMA_List[7].DstAddr = (uint32_t) sample_pointer7;

	for (int i = 0; i < 6; i++) {
		    DMA_List[i].SrcAddr = (uint32_t) &LPC_ADCHS->FIFO_OUTPUT[0];
		    DMA_List[i].NextLLI = (uint32_t)(&DMA_List[(i+1) % 6]);
		    DMA_List[i].Control =  (4095 << 0) |     	   // Transfersize (does not matter when flow control is handled by peripheral)
		                           (0x0 << 12)  |          // Source Burst Size
		                           (0x0 << 15)  |          // Destination Burst Size
		                           (0x2 << 18)  |          // Source width // 32 bit width
		                           (0x2 << 21)  |          // Destination width   // 32 bits
		                           (0x1 << 24)  |          // Source AHB master 0 / 1
		                           (0x1 << 25)  |          // Dest AHB master 0 / 1
		                           (0x0 << 26)  |          // Source increment(LAST Sample)
		                           (0x1 << 27)  |          // Destination increment
		                           (0x0UL << 31);          // Terminal count interrupt disabled
	};

	LPC_GPDMA->CH[DMA_CH].SRCADDR  =  DMA_List[0].SrcAddr;
	LPC_GPDMA->CH[DMA_CH].DESTADDR = DMA_List[0].DstAddr;
	LPC_GPDMA->CH[DMA_CH].CONTROL = DMA_List[0].Control;
	LPC_GPDMA->CH[DMA_CH].LLI  = (uint32_t)&(DMA_List[1]);

	LPC_GPDMA->CH[DMA_CH].CONFIG   = (0x1)		  // enable bit
								   | (0x8 << 1)   // src peripheral: set to 8   - HSADC
								   | (0x0 << 6)   // dst peripheral: no setting - memory
								   | (0x2 << 11)  // flow control: peripheral to memory - DMA control
								   | (0x1 << 14)  // IE  - interrupt error mask
								   | (0x1 << 15)  // ITC - terminal count interrupt mask
								   | (0x0 << 16)  // lock: when set, this bit enables locked transfer
								   | (0x1 << 18); // Halt: 1, enable DMA requests; 0, ignore further src DMA req

	//Enable Interrupt for DMA
	NVIC_SetPriority(DMA_IRQn,0x00);
	NVIC_ClearPendingIRQ(DMA_IRQn);
	NVIC_EnableIRQ(DMA_IRQn);
}

void startSampling()
{
	//start HSADC
	Chip_HSADC_SWTrigger(LPC_ADCHS);

	//start DMA
	LPC_GPDMA->CH[DMA_CH].CONFIG = (0x1 << 0); // enable bit, 1 enable, 0 disable

	for (int i = 0; i < 4095; i++) {
		// wait for DMA transfer to complete
		while(LPC_GPDMA->INTTCSTAT == 1) {};
	}

	Chip_HSADC_FlushFIFO(LPC_ADCHS);
	uint32_t sts = Chip_HSADC_GetFIFOLevel(LPC_ADCHS);
//	Chip_HSADC_DeInit(LPC_ADCHS); //shut down HSADC
//	Chip_GPDMA_DeInit(LPC_GPDMA); //shut down GPDMA
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

	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 12);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 13);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 15);

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 12, false);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 13, true);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, true);


	// Initialize USB
	setupUSB();

	// Initialize ADC
	setupADC();

	setupGPDMA();
	//cBufferInit(samples, 32000);
}

static void sendSample()
{
	while (libusbdev_QueueSendDone() != 0) {};
	while (libusbdev_QueueSendReq(sample_pointer0, 16380) != 0) {};

	while (libusbdev_QueueSendDone() != 0){};
	while (libusbdev_QueueSendReq(sample_pointer1, 16380) != 0){};

	while (libusbdev_QueueSendDone() != 0){};
	while (libusbdev_QueueSendReq(sample_pointer2, 16380) != 0){};

	while (libusbdev_QueueSendDone() != 0){};
	while (libusbdev_QueueSendReq(sample_pointer3, 16380) != 0){};
}

void samplingADC(void)
{
	LPC_ADCHS->DSCR_STS = ((0 << 1)
						| (0 << 0)); //Descriptor table 0
	LPC_ADCHS->TRIGGER = 1; //SW trigger ADC
}

void configureDevice(void) {
	//usb_in[0] = run mode (single = 0, continuous = 1)
	//usb_in[1] = trigger mode (0 = unconditional, 1 = use trigger)
	//usb_in[3:2] = trigger configuration settings (bits 11:0 = comparison value, bit 12 = 0 falling, 1 rising)
	//usb_in[4] = attenuation mode (1x = 0, 10x = 1)

	if (usb_in[4] == 1) {
		//set attenuation to 1x by using relay 2 to bypass analog gain module
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 12, true);
	} else {
		//set attenuation to 10x
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 12, false);
	}

	//usb_in[5] = coupling mode (AC = 0, DC = 1)
	if (usb_in[5] == 1) {
		//set analog input coupling mode to DC
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 13, false);
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, false);

		// Disable DC Biasing
		Chip_HSADC_SetACDCBias(LPC_ADCHS, 0, HSADC_CHANNEL_NODCBIAS, HSADC_CHANNEL_NODCBIAS);
	} else {
		//set analog input coupling mode to AC
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 13, true);
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, true);

		// Enable DC biasing
		Chip_HSADC_SetACDCBias(LPC_ADCHS, 0, HSADC_CHANNEL_DCBIAS, HSADC_CHANNEL_DCBIAS);
	}
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
    run = 0;

    while(1) {
    	if (libusbdev_QueueReadReq(usb_in, 8) != -1) {
          configureDevice();
          run = 1;
    	}

    	if (run == 1) {
    		//add line for usb read request user input prompt run once mode
    		samplingADC();
    		startSampling();
    		//USB send
    		sendSample();

    		//set run to 0 if single run mode
    		if (usb_in[0] == 0) {
    			run = 0;
    		}
    	}
    }

    Chip_HSADC_DeInit(LPC_ADCHS); //shut down HSADC
   	Chip_GPDMA_DeInit(LPC_GPDMA); //shut down GPDMA


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
