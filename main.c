/**
  ******************************************************************************
  * @file    main.c
  * @author  K.Pudlowski
  * @version V0.0.1
  * @date    26-11-2016
  * @brief   DAC, ADC, audio purpose  code for STM32L-Discovery Board
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


#define ARRAYSIZE 3
#define ADC1_DR    ((uint32_t)0x4001244C)
#define ADC1_DR_ADDRESS    ((uint32_t)0x40012458)
volatile uint16_t ADC_values[ARRAYSIZE];
__IO uint16_t ADC_ConvertedValue;
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

void InitDAC(void);
void InitADCDMA(void);
void InitADCSingle(void);
void ADC_DMA_Config(void);

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
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);

    lutindex=0;

    /* Enable The HSI (16Mhz) */
    RCC_HSICmd(ENABLE);

    /* Check that HSI oscillator is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

    InitDAC();
    InitADCDMA();
    //InitADCSingle();
    //ADC_DMA_Config();
    //ADC_values[0]=2000;
  // endless loop
    while(1) {
    	/* ADC_SoftwareStartConv(ADC1);

    	     ADC_values[0] = ADC_GetConversionValue(ADC1); //Read ADC value
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

void InitDAC(void)
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
		        TTB.TIM_Prescaler = 1; //  4800 kHz // was 300-1
		        TTB.TIM_Period = 1; //1Hz; // was 10-1

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
		        DACNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		        DACNVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		        DACNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		        NVIC_Init(&DACNVIC_InitStructure);
}



void InitADCDMA(void)
{
	/*
	https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2Fcortex%5Fmx%5Fstm32%2FMultiple%20Channel%20ADC%20on%20STM32%20L1%20Discovery%20with%20CooCox&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=359
	*/
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

		  //Enable GPIOA--


		      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
		      GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins
		      //==Configure ADC pins (PA0 -> Channel 0 to PA1 -> Channel 1) as analog inputs==
		      GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
		      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3;
		      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		      GPIO_Init(GPIOA, &GPIO_InitStructure);



		      /* Enable the DMA Stream IRQ Channel */
		        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
		        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		        NVIC_Init(&NVIC_InitStructure);

		      //Enable ADC1
		      RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

		      //ADC1 configuration

		      /* Clear  ADC configuration   */
		       ADC_DeInit(ADC1);

		       ADC_CommonStructInit(&ADC_CommonInitStructure);
		       ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
		       ADC_CommonInit(&ADC_CommonInitStructure);


		       /* ADC Configuration */
		       ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		       ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Multiple channels
		       ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Continuously back-to-back, not triggered
		       ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Not triggered
		       ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T9_CC2; // Not used, valid placeholder
		       ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		       ADC_InitStructure.ADC_NbrOfConversion = ARRAYSIZE; // =3
		       ADC_Init(ADC1,&ADC_InitStructure);




		      //configure each channel

		      ADC_RegularChannelConfig(ADC1, ADC_Channel_0,1, ADC_SampleTime_192Cycles);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_1,2, ADC_SampleTime_192Cycles);
		      ADC_RegularChannelConfig(ADC1, ADC_Channel_2,3, ADC_SampleTime_192Cycles);


		      /* Enable ADC1 Power Down during Delay */
			  ADC_PowerDownCmd(ADC1, ADC_PowerDown_Idle_Delay, ENABLE);

			  /* Enable DMA1 clock */
			   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

			   /* DMA1 channel1 configuration */
			   DMA_DeInit(DMA1_Channel1);
			   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;// was  &ADC1->DR;
			   DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_values[0];
			   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
			   DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
			   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
			   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
			   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
			   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
			   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
			   DMA_Init(DMA1_Channel1, &DMA_InitStructure);

			   /* Enable DMA Stream Half / Transfer Complete interrupt */
			   DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);

			   /* Enable DMA1 channel1 */
			   DMA_Cmd(DMA1_Channel1, ENABLE);

			   /* Enable the request after last transfer for DMA Circular mode */
			   ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

			   /* Enable ADC1 DMA */
			   ADC_DMACmd(ADC1, ENABLE);

			   /* Enable ADC1 */
			   ADC_Cmd(ADC1, ENABLE);

			   /* Wait until ADC1 ON status */
			   while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);

			   /* Start ADC1 Software Conversion */
			   ADC_SoftwareStartConv(ADC1);

}


void InitADCSingle(void)
{

	 ADC_InitTypeDef ADC_InitStructure;

	 // pin configuration
	 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	 	GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins
	 		      //==Configure ADC pins (PA0 -> Channel 0 to PA1 -> Channel 1) as analog inputs==
	 	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
	 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3;
	 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	 	GPIO_Init(GPIOA, &GPIO_InitStructure);


    //ADC1 configuration

    /* Clear  ADC configuration   */
     ADC_DeInit(ADC1);

     /* ADC Configuration */
   	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // One channel
   	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Continuously back-to-back, not triggered
   	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Not triggered
   	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T9_CC2; // Not used, valid placeholder
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   	ADC_InitStructure.ADC_NbrOfConversion = 1; // =3
   	ADC_Init(ADC1,&ADC_InitStructure);

   	ADC_RegularChannelConfig(ADC1, ADC_Channel_0,1, ADC_SampleTime_192Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_0,1, ADC_SampleTime_192Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_0,1, ADC_SampleTime_192Cycles);
	   /* Enable ADC1 */
	   ADC_Cmd(ADC1, ENABLE);

	   /* Wait until ADC1 ON status */
	   while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);

	   /* Start ADC1 Software Conversion */
	   ADC_SoftwareStartConv(ADC1);

    // ADC_values[0] = ADC_GetConversionValue(ADC1); //Read ADC value
}


void ADC_DMA_Config(void)
{

	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
  /*------------------------ DMA1 configuration ------------------------------*/
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* DMA1 channel1 configuration */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /*----------------- ADC1 configuration with DMA enabled --------------------*/
  /* Enable the HSI oscillator */
  RCC_HSICmd(ENABLE);

#if defined (USE_STM32L152_EVAL)
  /* Enable GPIOB clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  /* Configure PA.12 (ADC Channel18) in analog mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

#elif defined (USE_STM32L152D_EVAL)

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    /* Configure PA.12 (ADC Channel18) in analog mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

  /* Check that HSI oscillator is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* ADC1 configuration */
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

#if defined (USE_STM32L152_EVAL)
  /* ADC1 regular channel18 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_192Cycles);
#elif defined (USE_STM32L152D_EVAL)
  /* ADC1 regular channel14 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_192Cycles);
#endif

  /* Enable the request after last transfer for DMA Circular mode */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait until the ADC1 is ready */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
  {
  }


  /* Enable the DMA Stream IRQ Channel */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConv(ADC1);




}



//  handling TIM2 interrupt for DAC conversion

void TIM2_IRQHandler()
{



    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {

    			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    			//DACtimer=TIM_GetCounter(TIM2);
    			/*	based on
    			 http://amarkham.com/?p=49
    			 */
    	 	 	  accumulator1+=accumulator1r;
    	    	  //  first 10 (32 -22) bits -> lut table index
    	    	  accumulator1angle=(uint16_t)(accumulator1>>22);

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
    	    	  DAC_SetChannel2Data(DAC_Align_12b_R,DAC1OutputData);



    	    	  // changing accumulator register in time ....

    	    	  /*
    	    	  accumulator1r+=R>>6;
    	    	  accumulator2r-=R>>4;
    	    	  accumulator3r+=R>>8;
				  */
    	    	  // reading value of pots from ADC_value table

    	    	  accumulator1r=(uint32_t)257374*
    	    	     	    	  	  		rangeScaleLinear(ADC_values[1],0,4095,200,5000);
    	    	  /*
    	    	  accumulator1r=(uint32_t)257374*
    	        	    	     	    	  	  		rangeScaleLinear(1000,0,4095,100,5000);
    	        	    	     	    	  	  		*/
    	    	  /*
    	    	  accumulator2r=(uint32_t)257374*
    	    	     	    	     	    rangeScaleLinear(ADC_values[1],0,4095,100,5000);

    	    	  accumulator3r=(uint32_t)257374*
    	    	     	    	     	    rangeScaleLinear(ADC_values[2],0,4095,100,5000);

				  */

    }

}
// DMA IQR handler for ADC conversions

void DMA1_Channel1_IRQHandler(void)
{
  /* Test on DMA Channel Half Transfer interrupt */
  if (DMA_GetITStatus(DMA1_IT_HT1))
  {
    /* Clear DMA Channel Half Transfer interrupt pending bit */
    DMA_ClearITPendingBit(DMA1_IT_HT1);
    //GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
    GPIO_ToggleBits(GPIOC,GPIO_Pin_8);

    // Add code here to process first half of buffer (ping)
  }

  /* Test on DMA Channel Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA1_IT_TC1))
  {
    /* Clear DMA Channel Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA1_IT_TC1);
   // GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
    GPIO_ToggleBits(GPIOC,GPIO_Pin_9);

    // Add code here to process second half of buffer (pong)
  }
}

