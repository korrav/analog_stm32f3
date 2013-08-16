#include "main.h"

#define ADC12_ADR 0x5000030A
#define ADC34_ADR 0x5000070A
#define SPI1_ADR	0x4001300C
#define SPI2_ADR	0x4000380C

__IO uint32_t TimingDelay = 0;

static void initial_spi(void);
static void initial_adc(void);
void initial_dma(void);
void initial_opa(void);
void initial_tim8(void);
void Delay(__IO uint32_t nTime);


int main(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	/* Setup SysTick Timer for 1 µsec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000000))
  { 
    /* Capture error */ 
    while (1)
    {}
  }
	initial_spi();
	initial_adc();
	initial_tim8();
	ADC_StartConversion(ADC1); 
	ADC_StartConversion(ADC2); 
	ADC_StartConversion(ADC3); 
	ADC_StartConversion(ADC4); 
	TIM_Cmd(TIM8, ENABLE);
	while(1) 
	{}
}

void initial_spi(void) {
	GPIO_InitTypeDef structGPIO;
	SPI_InitTypeDef structSPI;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	//configure GPIO
	structGPIO.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; //spi1
  structGPIO.GPIO_Mode = GPIO_Mode_AF;
	structGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	structGPIO.GPIO_OType = GPIO_OType_PP;
	structGPIO.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &structGPIO);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);
	structGPIO.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; //spi2
	GPIO_Init(GPIOB, &structGPIO);	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_5);
	//configure spi
	SPI_StructInit(&structSPI);
	structSPI.SPI_Direction = SPI_Direction_1Line_Tx;
	structSPI.SPI_Mode = SPI_Mode_Master;
	structSPI.SPI_DataSize = SPI_DataSize_12b;
	SPI_Init(SPI1, &structSPI);
	SPI_Init(SPI2, &structSPI);
	SPI_SSOutputCmd(SPI1, ENABLE);
	SPI_SSOutputCmd(SPI2, ENABLE);
	SPI_Cmd(SPI1, ENABLE);
	SPI_Cmd(SPI2, ENABLE);
// 	//configure interrupt
// 	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure); 
// 	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//   NVIC_Init(&NVIC_InitStructure); 
	return;
}

void initial_adc(void) {
	GPIO_InitTypeDef structGPIO;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div6);
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div6);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12 | RCC_AHBPeriph_ADC34 | RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD, ENABLE);
	//configure GPIO
	structGPIO.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
  structGPIO.GPIO_Mode = GPIO_Mode_AN;
  structGPIO.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &structGPIO);
	structGPIO.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_10;
	GPIO_Init(GPIOB, &structGPIO);
	structGPIO.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOC, &structGPIO);
	structGPIO.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_14;
	GPIO_Init(GPIOD, &structGPIO);
	//configure ADC
	/*power-on voltage reference*/
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	Delay(10);
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	Delay(10);
	ADC_VoltageRegulatorCmd(ADC3, ENABLE);
	Delay(10);
	ADC_VoltageRegulatorCmd(ADC4, ENABLE);
	Delay(10);
	/*calibration*/
	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1) != RESET );
	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC2);
  while(ADC_GetCalibrationStatus(ADC2) != RESET );
	ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC3);
  while(ADC_GetCalibrationStatus(ADC3) != RESET );
	ADC_SelectCalibrationMode(ADC4, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC4);
  while(ADC_GetCalibrationStatus(ADC4) != RESET );
	/*init logic*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_7; 
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_4; 
	ADC_Init(ADC3, &ADC_InitStructure);
	ADC_Init(ADC4, &ADC_InitStructure);
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_RegSimul;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC3, &ADC_CommonInitStructure);             
	ADC_CommonInit(ADC2, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC4, &ADC_CommonInitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_4Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_17, 1, ADC_SampleTime_4Cycles5);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_17, 1, ADC_SampleTime_4Cycles5);
	ADC_RegularChannelConfig(ADC4, ADC_Channel_17, 1, ADC_SampleTime_4Cycles5);
	
	ADC_RegularChannelSequencerLengthConfig(ADC1, 1);
	ADC_RegularChannelSequencerLengthConfig(ADC2, 1);
	ADC_RegularChannelSequencerLengthConfig(ADC3, 1);
	ADC_RegularChannelSequencerLengthConfig(ADC4, 1);
	
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_Cmd(ADC4, ENABLE);
	 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY) && !ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY) 
		 && !ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY) && !ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY));
	 
	initial_opa();
	initial_dma();
	return;
}

void initial_opa(void) {
	OPAMP_InitTypeDef OPAMP_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//OPA1
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO4;
  OPAMP_InitStructure.OPAMP_InvertingInput =  OPAMP_InvertingInput_PGA;
  OPAMP_Init(OPAMP_Selection_OPAMP1, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP1, OPAMP_OPAMP_PGAGain_2, OPAMP_PGAConnect_No);
	//OPA2
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO1;
	OPAMP_Init(OPAMP_Selection_OPAMP2, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP2, OPAMP_OPAMP_PGAGain_2, OPAMP_PGAConnect_No);
	//OPA3
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO4;
	OPAMP_Init(OPAMP_Selection_OPAMP2, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP2, OPAMP_OPAMP_PGAGain_2, OPAMP_PGAConnect_No);
	//OPA4
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO1;
	OPAMP_Init(OPAMP_Selection_OPAMP2, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP2, OPAMP_OPAMP_PGAGain_2, OPAMP_PGAConnect_No);
	
	OPAMP_Cmd(OPAMP_Selection_OPAMP1, ENABLE);
	OPAMP_Cmd(OPAMP_Selection_OPAMP2, ENABLE);
	OPAMP_Cmd(OPAMP_Selection_OPAMP3, ENABLE);
	OPAMP_Cmd(OPAMP_Selection_OPAMP4, ENABLE);
}

void initial_tim8(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_Period = 36000;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
// 	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);
//   TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	
	TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
	TIM_SelectOnePulseMode(TIM8, TIM_OPMode_Single);
	
}
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

// void TIM6_DAC_IRQHandler(void) {
// }

void initial_dma(void) {
	//for ADC12
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC12_ADR;
	DMA_InitStructure.DMA_MemoryBaseAddr = SPI1_ADR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc =  DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	//for ADC34
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC34_ADR;
	DMA_InitStructure.DMA_MemoryBaseAddr = SPI2_ADR;
  DMA_Init(DMA2_Channel5, &DMA_InitStructure);
	DMA_Cmd(DMA2_Channel5, ENABLE);
}
