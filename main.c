/**
  ******************************************************************************
  * @file    main.c
  * @author  J.Shankarappa
  * @version V0.0.1
  * @date    5-April-2013
  * @brief   Blink LED Program for STM32L-Discovery Board
  ******************************************************************************
  */

#include "stm32l1xx.h"
#include "discover_board.h"
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_tim.h>
#include <stm32l1xx_adc.h>
#include <misc.h>
#include "resources.h"
#include "algorithm.h"

TIM_TimeBaseInitTypeDef 	TTB;
DAC_InitTypeDef         	DAC_InitStructure;
GPIO_InitTypeDef			GPIO_InitStructure;
NVIC_InitTypeDef         	DACNVIC_InitStructure;


#define ARRAYSIZE 3*4
#define ADC1_DR    ((uint32_t)0x4001244C)
volatile uint16_t ADC_values[ARRAYSIZE];
volatile uint32_t status = 0;

volatile uint16_t R =5730;

#define MAX32 (uint32_t) 4294967295


// 1st sine
volatile uint32_t accumulator1=0;
volatile uint16_t accumulator1angle=0;
volatile uint16_t accumulator1step=0;
volatile uint32_t accumulator1r=35737418;
volatile double VoltValue1=0.0;


//2nd sine
volatile uint32_t accumulator2=0;
volatile uint16_t accumulator2angle=0;
volatile uint16_t accumulator2step=0;
volatile uint32_t accumulator2r=25737418;
volatile double VoltValue2=0.0;

//3rd sine
volatile uint32_t accumulator3=0;
volatile uint16_t accumulator3angle=0;
volatile uint16_t accumulator3step=0;
volatile uint32_t accumulator3r=35737418;
volatile double VoltValue3=0.0;
// for DAC output data (12-bit) from 0 to 4095
volatile uint16_t DAC1OutputData ;

volatile uint16_t lutindex;

void TimingDelay_Decrement(void);  // used in SysTick_Handler function
void Delay(__IO uint32_t nTime);

void InitDAC();
void InitADC();
void InitDMA();
static __IO uint32_t TimingDelay;

int main(void)
{
	/* At this stage the micro-controller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32l1xx_md.S) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32l1xx.c file
	 */

	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable Peripheral Clocks
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    // Configure GPIO Pins
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);


    //SysTick_Config(SystemCoreClock / 1000);  // SysTick for every  1-miliSec

    GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
    /*
    Delay(100000);
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
    Delay(100000);
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
    Delay(100000);
	*/
    lutindex=0;

    InitDAC();
    InitADC();
    InitDMA();

    ADC_values[0]=12345670;
    //Enable DMA1 Channel transfer
    DMA_Cmd(DMA1_Channel1, ENABLE);
    //Start ADC1 Software Conversion
    ADC_SoftwareStartConv(ADC1);

  // endless loop
    while(1) {

    	//GPIO_TOGGLE(GPIOC,GPIO_Pin_8);
    	//GPIO_TOGGLE(GPIOC,GPIO_Pin_9);

    	//Delay(100);
    	/*
    	lutindex++;
    	if (lutindex>=1024)
		{
    		lutindex-=1024;

    	}

    	DAC1OutputData=Sine1024_12bit[lutindex];

    	DAC_SetChannel1Data(DAC_Align_12b_R,DAC1OutputData);
    	*/
    }
}



/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
	if( TimingDelay != 0x00 ) {
		TimingDelay--;
	}
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
	  TimingDelay = nTime;

	  while(TimingDelay != 0)
		  ;
}

void InitDAC()
{
	/* DAC  clock enable */

			  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

			  /* TIM2 for DAC */
			  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 ,ENABLE);

			  /* for output PINS */
			  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);


			  // Configure PA.04/05 (DAC) as output -------------------------
			  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
			  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
			  	GPIO_Init(GPIOA, &GPIO_InitStructure);

			  /* Fill DAC InitStructure */

			  	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
			 	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
			 	//DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
			 	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
			 	//DAC_InitStructure.DAC_WaveGeneration =DAC_WaveGeneration_Triangle;
			 	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

			 	  //(+) Enable the DAC channel using DAC_Cmd()
			 	DAC_Cmd(DAC_Channel_1, ENABLE);


				  //TTB.TIM_ClockDivision = TIM_CKD_DIV1;
		        TTB.TIM_CounterMode = TIM_CounterMode_Up;
		        //TTB.TIM_RepetitionCounter = 0;
		        TTB.TIM_ClockDivision = 0;
		        TTB.TIM_Prescaler = 10; //  4800 kHz // was 300-1
		        TTB.TIM_Period = 10; //1Hz; // was 10-1

		        TIM_TimeBaseInit(TIM2, &TTB);
		        TIM_Cmd(TIM2, ENABLE);


		         /* http://visualgdb.com/tutorials/arm/stm32/timers/ */
		        TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		        TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

		          /* http://forbot.pl/blog/artykuly/programowanie/kurs-stm32-7-liczniki-timery-w-praktyce-pwm-id8459 */

		          // use as interrupt
		        NVIC_SetPriority(TIM2_IRQn, 0);
		        NVIC_EnableIRQ (TIM2_IRQn);
		        DACNVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
		       // DACNVIC_InitStructure.NVIC_IRQChannelPriority = 0;
		        DACNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		        NVIC_Init(&DACNVIC_InitStructure);
}


void InitADC()
{

		  //--Enable ADC1 and GPIOA--
		     // RCC_APB1PeriphClockCmd(RCC_APB1Periph_ADC1 | RCC_APB1Periph_GPIOA, ENABLE);
		      RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
		      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
		      GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins
		      //==Configure ADC pins (PA0 -> Channel 0 to PA7 -> Channel 7) as analog inputs==
		      GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
		      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
		      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		      GPIO_Init(GPIOA, &GPIO_InitStructure);

		      ADC_InitTypeDef ADC_InitStructure;
		      //ADC1 configuration

		     // ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		      //We will convert multiple channels
		      ADC_InitStructure.ADC_ScanConvMode = ENABLE;
		      //select continuous conversion mode
		      ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//!
		      //select no external triggering
		      ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		      //right 12-bit data alignment in ADC data register
		      ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		      //8 channels conversion
		      ADC_InitStructure.ADC_NbrOfConversion = 3;
		      //load structure values to control and status registers
		      ADC_Init(ADC1, &ADC_InitStructure);
		      //wake up temperature sensor
		      //ADC_TempSensorVrefintCmd(ENABLE);
		      //configure each channel
		      /*
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_41Cycles5);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_41Cycles5);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_41Cycles5);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_41Cycles5);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_41Cycles5);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 7, ADC_SampleTime_41Cycles5);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_41Cycles5);
		      */
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_0,1, ADC_SampleTime_48Cycles);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_1,2, ADC_SampleTime_48Cycles);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_2,3, ADC_SampleTime_48Cycles);

		      //Enable ADC1
		      ADC_Cmd(ADC1, ENABLE);
		      //enable DMA for ADC
		      ADC_DMACmd(ADC1, ENABLE);
		      //Enable ADC1 reset calibration register
		      //ADC_ResetCalibration(ADC1);
		      //Check the end of ADC1 reset calibration register
		      //while(ADC_GetResetCalibrationStatus(ADC1));
		      //Start ADC1 calibration
		      //ADC_StartCalibration(ADC1);
		      //Check the end of ADC1 calibration
		      //while(ADC_GetCalibrationStatus(ADC1));

}

void InitDMA()
{
	//enable DMA1 clock
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	    //create DMA structure
	    DMA_InitTypeDef  DMA_InitStructure;
	    //reset DMA1 channe1 to default values;
	    DMA_DeInit(DMA1_Channel1);
	    //channel will be used for memory to memory transfer
	    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	    //setting normal mode (non circular)
	    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	    //medium priority
	    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	    //source and destination data size word=32bit
	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	    //automatic memory destination increment enable.
	    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	    //source address increment disable
	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	    //Location assigned to peripheral register will be source
	    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	    //chunk of data to be transfered
	    DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
	    //source and destination start addresses
	    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR;
	    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_values;
	    //send values to DMA registers
	    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	    // Enable DMA1 Channel Transfer Complete interrupt
	    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	    DMA_Cmd(DMA1_Channel1, ENABLE); //Enable the DMA1 - Channel1
	    NVIC_InitTypeDef NVIC_InitStructure;
	    //Enable DMA1 channel IRQ Channel */
	    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);
}

//  handling TIM2 interrupt for DAC conversion

void TIM2_IRQHandler()
{



    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {


    			//DACtimer=TIM_GetCounter(TIM2);
    			/*	based on
    			 http://amarkham.com/?p=49
    			 */
    	 	 	  accumulator1+=accumulator1r;
    	    	  //  first 10 (32 -22) bits -> lut table index
    	    	  accumulator1angle=(uint16_t)(accumulator1>>22);
    	    	  /*
    	    	  if (accumulator1angle>=1024)
    	    	  {
    	    		  accumulator1angle-=1024;
    	    	  }
    	    	  */
    	    	  accumulator1step = Sine1024_12bit[accumulator1angle];

    	    	  accumulator2+=accumulator2r;
    	    	  //  first 10 (32 -22) bits -> lut table index
    	    	  accumulator2angle=(uint16_t)(accumulator2>>22);
    	    	  accumulator2step = Sine1024_12bit[accumulator2angle];

    	    	  accumulator3+=accumulator3r;
    	    	  //  first 10 (32 -22) bits -> lut table index
    	    	  accumulator3angle=(uint16_t)(accumulator3>>22);
    	    	  accumulator3step = Sine1024_12bit[accumulator3angle];

    	    	  {
    	    	  DAC1OutputData = (uint16_t)
    	    			  (accumulator1step+accumulator1step+accumulator1step)/3.0;
    	    	  }

    	    	  // sending 12-bits output signal
    	    	  DAC_SetChannel1Data(DAC_Align_12b_R,DAC1OutputData);
    	    	  // changing accumulator register in time ....


    	    	  accumulator1r+=R>>6;
    	    	 // accumulator2r-=R>>4;
    	    	  //accumulator3r+=R>>8;
    	    	  /*
    	    	  accumulator1r=(uint32_t)257374*
    	    	     	    	  	  		rangeScaleLinear(ADC_values[0],0,4095,10,5000);
				*/
    	    	  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);


    }

}

