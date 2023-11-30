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
//********** USB Input Array **********************//
uint8_t usb_in[8];

//********** Arrays for storing samples ***********//	`
uint8_t sampleBuffer_0[16352];
uint8_t sampleBuffer_1[16352];
uint8_t sampleBuffer_2[16352];
uint8_t sampleBuffer_3[16352];

uint8_t sampleBuffer_4[16352];
uint8_t sampleBuffer_5[16352];
__SECTION(bss, RAM2) uint8_t sampleBuffer_6[16352];
//__SECTION(bss, RAM2) uint8_t sampleBuffer_7[16380];

//variable to store last transfer address
uint32_t last_transfer;

//********** USB Rom header and variables **********//
#include <stdio.h>
#include <string.h>
#include "libusbdev.h"

//******************** HSADC variables ********************//
#define HSADC_CLOCK_RATE (80 * 1000000)
#define HSADC_DESC_NOPOWERDOWN 0

//********** GPDMA definitions and variables **********//

#define DMA_CH 7

typedef struct{
	uint32_t SrcAddr; /**< Source Address */
	uint32_t DstAddr; /**< Destination address */
	uint32_t NextLLI; /**< Next LLI address, otherwise set to '0' */
	uint32_t Control; /**< GPDMA Control of this LLI */
}GPDMA_vector;

static GPDMA_vector DMA_List[8];

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

	// setting up the ADC clock speed
	Chip_USB0_Init(); /* Initialize the USB0 PLL to 480 MHz */ //try changing this to USB1?
	Chip_Clock_SetDivider(CLK_IDIV_A, CLKIN_USBPLL, 2); /* Source DIV_A from USB0PLL, and set divider to 2 (Max div value supported is 4)  [IN 480 MHz; OUT 240 MHz */
	Chip_Clock_SetDivider(CLK_IDIV_B, CLKIN_IDIVA, 3); /* Source DIV_B from DIV_A, [IN 240 MHz; OUT 80 MHz */
//	Chip_Clock_SetDivider(CLK_IDIV_C,CLKIN_IDIVB,2); //80 MHz -> 40 MHz
	Chip_Clock_SetBaseClock(CLK_BASE_ADCHS, CLKIN_IDIVB, true, false); /* Source ADHCS base clock from DIV_B */
	freqHSADC = Chip_HSADC_GetBaseClockRate(LPC_ADCHS);
	Chip_Clock_EnableOpts(CLK_MX_ADCHS, true, true, 1);
	Chip_Clock_Enable(CLK_ADCHS);

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
	LPC_ADCHS->FIFO_CFG = (1 << 0) | /* UNPACKED */
						  (8 << 1);  /* FIFO_LEVEL */

	LPC_ADCHS->DSCR_STS = (0 << 0) |   /* ACT_TABLE:        0=table 0 is active, 1=table 1 is active */
						  (1 << 1);    /* ACT_DESCRIPTOR:   ID of the descriptor that is active */

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

	/*Set descriptor 0 to take a measurement at every clock and branch to itself*/
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
}

//********** DMA functions **********//
void setupGPDMA()
{
//**************************************** Our DMA Code ****************************************//
	Chip_GPDMA_Init(LPC_GPDMA);

	NVIC_DisableIRQ(DMA_IRQn);

	// Clear all DMA interrupt and error flag
	LPC_GPDMA->INTTCCLEAR = 0xFF; //clears channel terminal count interrupt
	LPC_GPDMA->INTERRCLR = 0xFF;  //clears channel error interrupt.

	LPC_GPDMA->CONFIG = 0x01;
	while( !(LPC_GPDMA->CONFIG & 0x01) );

	// setting up dataBuffer list destination address
	DMA_List[0].DstAddr = (uint32_t) sampleBuffer_0;
	DMA_List[1].DstAddr = (uint32_t) sampleBuffer_0;
	DMA_List[2].DstAddr = (uint32_t) sampleBuffer_1;
	DMA_List[3].DstAddr = (uint32_t) sampleBuffer_2;
	DMA_List[4].DstAddr = (uint32_t) sampleBuffer_3;
	DMA_List[5].DstAddr = (uint32_t) sampleBuffer_4;
	DMA_List[6].DstAddr = (uint32_t) sampleBuffer_5;
	DMA_List[7].DstAddr = (uint32_t) sampleBuffer_6;

	for (int i = 0; i < 8; i++)
	{
		// Set source and destination address
		DMA_List[i].SrcAddr = (uint32_t) &LPC_ADCHS->FIFO_OUTPUT[0];
		DMA_List[i].NextLLI = (uint32_t) (&DMA_List[(i + 1) % 7]);
		DMA_List[i].Control  = (4088 << 0) // transfer size (does not matter when flow control is handled by peripheral)
							 | (0x0 << 12)  // src burst size of 1
							 | (0x0 << 15)  // dst burst size of 1
							 | (0x2 << 18)  // src transfer width
							 | (0x2 << 21)  // dst transfer width
							 | (0x1 << 24)  // src AHB master select (24)
							 | (0x1 << 25)  // dst AHB master select (25)
							 | (0x0 << 26)  // src increment: 0, src address not increment after each trans
							 | (0x1 << 27)  // dst increment: 1, dst address     increment after each trans
							 | (0x0UL << 31); // terminal count interrupt enable bit: 1, enabled
	}

	DMA_List[7].NextLLI = (uint32_t) (&DMA_List[1]);

	LPC_GPDMA->CH[DMA_CH].SRCADDR  = DMA_List[0].SrcAddr;
	LPC_GPDMA->CH[DMA_CH].DESTADDR = DMA_List[0].DstAddr;
	LPC_GPDMA->CH[DMA_CH].CONTROL  = DMA_List[0].Control;
	LPC_GPDMA->CH[DMA_CH].LLI      = (uint32_t)(&DMA_List[1]);  // must be pointing to the second LLI as the first is used when initializing

	LPC_GPDMA->CH[DMA_CH].CONFIG = (0x1 << 0) // enable bit: 1 enable, 0 disable
								 | (0x8 << 1)   // src peripheral: set to 8   - HSADC
								 | (0x0 << 6)   // dst peripheral: no setting - memory
								 | (0x2 << 11)  // flow control: peripheral to memory - DMA control //used to be 0x6
								 | (0x1 << 14)  // IE  - interrupt error mask //used to be 0x2
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

	//start DMA
	LPC_GPDMA->CH[DMA_CH].CONFIG = (0x1 << 0); // enable bit, 1 enable, 0 disable

	// check how long we have to give DMA to store all data in the buffer
	// also check if the number of samples also increases along with the delay
	for(int i = 0; i < 16280; i++)
	{
		// wait for DMA transfer to complete
		while(LPC_GPDMA->INTTCSTAT == 1);
	}

	Chip_HSADC_FlushFIFO(LPC_ADCHS);
	//uint32_t sts = Chip_HSADC_GetFIFOLevel(LPC_ADCHS);
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

void samplingADC(void)
{
	LPC_ADCHS->DSCR_STS = ((0 << 1) | (0 << 0)); //Descriptor table 0
	LPC_ADCHS->TRIGGER = 1; //SW trigger ADC
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

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 12, true);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 13, true);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, 15, true);


	// Initialize USB
	setupUSB();

	// Initialize ADC
	setupADC();

	//Initialize GPDMA
	setupGPDMA();

	samplingADC();

	//start HSADC
	Chip_HSADC_SWTrigger(LPC_ADCHS);

}

static void sendSample()
{
	while (libusbdev_QueueSendDone() != 0){};
	while (libusbdev_SendInterrupt(1) != 0) {};

	while (libusbdev_QueueSendReq((uint8_t*) &last_transfer, 4) != 0){};

	while (libusbdev_QueueSendDone() != 0){};

	while (libusbdev_QueueSendReq(sampleBuffer_0, 16352) != 0){};

	while (libusbdev_QueueSendDone() != 0){};
//	libusbdev_SendInterrupt(1);
	while (libusbdev_QueueSendReq(sampleBuffer_1, 16352) != 0){};

	while (libusbdev_QueueSendDone() != 0){};
//	libusbdev_SendInterrupt(1);
	while (libusbdev_QueueSendReq(sampleBuffer_2, 16352) != 0){};

	while (libusbdev_QueueSendDone() != 0){};
//	libusbdev_SendInterrupt(1);
	while (libusbdev_QueueSendReq(sampleBuffer_3, 16352) != 0){};

	while (libusbdev_QueueSendDone() != 0){};
//	libusbdev_SendInterrupt(1);
	while (libusbdev_QueueSendReq(sampleBuffer_4, 16352) != 0){};

	while (libusbdev_QueueSendDone() != 0){};
//	libusbdev_SendInterrupt(1);
	while (libusbdev_QueueSendReq(sampleBuffer_5, 16352) != 0){};

	while (libusbdev_QueueSendDone() != 0){};
//	libusbdev_SendInterrupt(1);
	while (libusbdev_QueueSendReq(sampleBuffer_6, 16352) != 0){};

//	while (libusbdev_QueueSendDone() != 0){};
//	while (libusbdev_QueueSendReq(sampleBuffer_7, 16352) != 0){};
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
//	uint32_t actualLLI;
//	static bool on1, on2;
//
//	//If USB is in dump mode
//	if (usbDumpConnected == true)
//	{
//		actualLLI = LPC_GPDMA->CH[0].LLI; //Look at LLI in order to know what is the previous full USB buffer and send to PC
//		if (actualLLI == (uint32_t) &arrayLLI[0]) {
//			while (libusbdev_QueueSendDone() != 0);
//			while (libusbdev_QueueSendReq(g_txBuff2, 8192) != 0);
//		}
//		if (actualLLI == (uint32_t) &arrayLLI[1]) {
//			while (libusbdev_QueueSendDone() != 0);
//			while (libusbdev_QueueSendReq(g_txBuff3, 8192) != 0);
//		}
//		if (actualLLI == (uint32_t) &arrayLLI[2]) {
//			while (libusbdev_QueueSendDone() != 0);
//			while (libusbdev_QueueSendReq(g_txBuff, 8192) != 0);
//		}
//		if (actualLLI == (uint32_t) &arrayLLI[3]) {
//			while (libusbdev_QueueSendDone() != 0);
//			while (libusbdev_QueueSendReq(g_txBuff1, 8192) != 0);
//		}
//	}
//	LPC_GPDMA->INTTCCLEAR = LPC_GPDMA->INTTCSTAT;
//	//GPDMA_capture(&g_txBuff[0], 128); //Restart DMA operation
//	//Board_LED_Set(0,true);

//**************************************** DMA IRQ handler test code ****************************************//
	__asm("nop");
	//stop hsadc
	LPC_GPDMA-> CH [DMA_CH].CONFIG&=~(0x1 << 0);
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

void calculateIndexLast(void){
	if (last_transfer >= (uint32_t)sampleBuffer_0 && last_transfer < ((uint32_t)sampleBuffer_0 + 16352)){
		last_transfer -= (uint32_t)sampleBuffer_0;
		last_transfer /= 2;
		return;
	}
	if (last_transfer >= (uint32_t)sampleBuffer_1 && last_transfer < ((uint32_t)sampleBuffer_1 + 16352)){
		last_transfer -= (uint32_t)sampleBuffer_1;
		last_transfer += 16352;
		last_transfer /= 2;
		return;
	}
	if (last_transfer >= (uint32_t)sampleBuffer_2 && last_transfer < ((uint32_t)sampleBuffer_2 + 16352)){
		last_transfer -= (uint32_t)sampleBuffer_2;
		last_transfer += 2*16352;
		last_transfer /= 2;
		return;
	}
	if (last_transfer >= (uint32_t)sampleBuffer_3 && last_transfer < ((uint32_t)sampleBuffer_3 + 16352)){
		last_transfer -= (uint32_t)sampleBuffer_3;
		last_transfer += 3*16352;
		last_transfer /= 2;
		return;
	}
	if (last_transfer >= (uint32_t)sampleBuffer_4 && last_transfer < ((uint32_t)sampleBuffer_4 + 16352)){
		last_transfer -= (uint32_t)sampleBuffer_4;
		last_transfer += 4*16352;
		last_transfer /= 2;
		return;
	}
	if (last_transfer >= (uint32_t)sampleBuffer_5 && last_transfer < ((uint32_t)sampleBuffer_5 + 16352)){
		last_transfer -= (uint32_t)sampleBuffer_5;
		last_transfer += 5*16352;
		last_transfer /= 2;
		return;
	}
	if (last_transfer >= (uint32_t)sampleBuffer_6 && last_transfer < ((uint32_t)sampleBuffer_6 + 16352)){
		last_transfer -= (uint32_t)sampleBuffer_6;
		last_transfer += 6*16352;
		last_transfer /= 2;
		return;
	}
	last_transfer = -1;
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
    Chip_GPDMA_ChannelCmd(LPC_GPDMA, DMA_CH, ENABLE);
    //start DMA
    LPC_GPDMA->SYNC = 1;
    LPC_GPDMA->CH[DMA_CH].CONFIG = (0x1 << 0); // enable bit, 1 enable, 0 disable
    while(1) {
    	//Start DMA transfer


		//request Run Command from USB
		libusbdev_QueueReadReq(&(usb_in[0]), (uint32_t) 1);
		while (libusbdev_QueueReadDone() == -1) {};

		//If run, stop the sampling and send buffers over usb
		if (usb_in[0] == 1) {
			// sampling data via ADC
			//startSampling();
			//Stop DMA transfer
			while(LPC_GPDMA->INTTCSTAT == 1);
		    LPC_GPDMA->CH[DMA_CH].CONFIG = (0x0 << 0); // enable bit, 1 enable, 0 disable

		    last_transfer = LPC_GPDMA->CH[DMA_CH].DESTADDR;
		    calculateIndexLast();
			//send data over USB
			sendSample();
			LPC_GPDMA->CH[DMA_CH].CONFIG = (0x1 << 0); // enable bit, 1 enable, 0 disable

		}
    }

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
