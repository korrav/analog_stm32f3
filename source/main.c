#include "main.h"

#define SYSTEMTICK_PERIOD_MUS 100 //period of time update
#define ADC12_ADR 0x5000030A
#define ADC34_ADR 0x5000070A
#define SPI1_ADR	0x4001300C
#define SPI2_ADR	0x4000380C
#define SPI3_ADR	0x40003C0C
#define I2C_BUF_SIZE 100
#define I2C_SLAVE 1
#define I2C_OWN 6
#define TIME_MONITORING_PGA 100

__IO static uint64_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
volatile short* adc12 = (short*)0x5000030C;
volatile short* adc34 = (short*)0x5000070C;
volatile short* spi1 =  (short*)SPI1_ADR;
volatile short* spi2 =  (short*)SPI2_ADR;
volatile short* spi3 =  (short*)SPI3_ADR;

char test =0;

struct {
	int8_t bufr[I2C_BUF_SIZE];
	uint8_t numr;
	uint16_t is_tran;
} stat;

struct {
	int16_t gain[4];
	int8_t statusAdc;
	int16_t rec_message[4];
	char is_overload;
	char enable_monitoring;
	char mut;
} driver_stat;

struct {
	int8_t gain[4];
	int8_t statusTest;
} test_stat;

//temp variables for test
uint16_t spi3Start = 0;
uint16_t spi1Start = 0;
uint16_t cTran = 0;
uint16_t cSetFlag = 0;

static void initial_spi(void);
static void initial_adc(void);
void initial_gpio(void);
void initial_opa(void);
void initial_tim8(void);
void initial_i2c(void);
void initial_mco(void);
void start_ADC(void);
void stop_ADC(void);
void Delay(uint64_t nCount);
void Time_Update(void);
void i2c_hand(void);
void handl_i2c_message(void);
void i2c_write(uint8_t* pBuffer, uint8_t num);
void pga2505_write(void);
void orders_to_stop(void);
void orders_to_stop_test(void);
void start_test(void);
void monitoring_pga(void);
void setGainOut(int8_t* gain);
void clear_set_debug(void);

enum id_package_of_i2c { START_ADC, STOP_ADC, SET_GAIN, START_SPI0, START_SPI1, START_TEST, STOP_TEST, 
												CLEAR_SET_OVERLOAD, ENABLE_MONITORING, DISABLE_MONITORING};
enum status_ADC {RUN_ADC, HALT_ADC, DURING_STOP_ADC};
enum status_TEST {RUN_TEST, HALT_TEST, DURING_STOP_TEST};

int main(void) {
	int i = 0;
	driver_stat.statusAdc = HALT_ADC;
	for(i =0;  i < 4;i++) {
		driver_stat.gain[i] = 0;
		driver_stat.rec_message[i] = 0;
	}
	driver_stat.is_overload = 0;
	driver_stat.enable_monitoring = 0;
	driver_stat.mut = 0;
	test_stat.statusTest = HALT_TEST;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
		/* Setup SysTick Timer for 1 ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 10))
  { 
    /* Capture error */ 
    while (1)	
    {}
  }
	initial_spi();
	initial_gpio();
	initial_adc();
	initial_i2c();
	initial_mco();
	pga2505_write();
	driver_stat.enable_monitoring = 1;
	while(1) 
	{
		if(test_stat.statusTest != HALT_TEST) {
			*spi3 = 100 + i;
			__DSB();
			*spi3 = 200 + i;
			*spi1 = 300 + i;
			__DSB();
			*spi1 = 400 + i;
			i++;
			if( i > 100)
				i = 0;
			if(test_stat.statusTest == DURING_STOP_TEST)
				test_stat.statusTest = HALT_TEST;
			Delay(2);
		}
	}
}

void initial_spi(void) {
	GPIO_InitTypeDef structGPIO;
	SPI_InitTypeDef structSPI;
	//NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF, ENABLE);
	//configure GPIO
	structGPIO.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7; //spi1
  structGPIO.GPIO_Mode = GPIO_Mode_AF;
	structGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	structGPIO.GPIO_OType = GPIO_OType_PP;
	structGPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &structGPIO);
  structGPIO.GPIO_Pin = GPIO_Pin_4;	
	GPIO_Init(GPIOB, &structGPIO);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);
	structGPIO.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; //spi3
	structGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &structGPIO);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_6);
	structGPIO.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &structGPIO);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_6);
	structGPIO.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15; //spi2
	structGPIO.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &structGPIO);
	structGPIO.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOF, &structGPIO);
	structGPIO.GPIO_Pin = GPIO_Pin_15;
	structGPIO.GPIO_Mode = GPIO_Mode_OUT;
	structGPIO.GPIO_OType = GPIO_OType_PP;
	structGPIO.GPIO_PuPd = GPIO_PuPd_UP;
	structGPIO.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &structGPIO);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_5);
	//configure spi1 and spi3
	SPI_StructInit(&structSPI);
	structSPI.SPI_Direction = SPI_Direction_1Line_Tx;
	structSPI.SPI_Mode = SPI_Mode_Master;
	structSPI.SPI_DataSize = SPI_DataSize_16b;
	structSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4/*SPI_BaudRatePrescaler_32*/;
	structSPI.SPI_NSS = SPI_NSS_Hard;
	structSPI.SPI_CPOL = SPI_CPOL_Low;
	structSPI.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_Init(SPI1, &structSPI);
	structSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2 /*SPI_BaudRatePrescaler_32*/;
	SPI_Init(SPI3, &structSPI);
	SPI_SSOutputCmd(SPI1, ENABLE);
	SPI_SSOutputCmd(SPI3, ENABLE);
 	SPI_Cmd(SPI1, ENABLE);
 	SPI_Cmd(SPI3, ENABLE);
	//configure spi2
	GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET);
	structSPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	structSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_Init(SPI2, &structSPI);
	SPI_SSOutputCmd(SPI2, ENABLE);
	SPI_NSSPulseModeCmd(SPI2, DISABLE);
	SPI_Cmd(SPI2, ENABLE);
	return;
}

void initial_adc(void) {
	GPIO_InitTypeDef structGPIO;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div10 /*RCC_ADC12PLLCLK_Div128*/);
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div10 /*RCC_ADC34PLLCLK_Div128*/);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12 | RCC_AHBPeriph_ADC34 | RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD, ENABLE);
	//configure GPIO
	structGPIO.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_6;
  structGPIO.GPIO_Mode = GPIO_Mode_AN;
  structGPIO.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &structGPIO);
	structGPIO.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_10 | GPIO_Pin_1 | GPIO_Pin_12;
	GPIO_Init(GPIOB, &structGPIO);
	structGPIO.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOC, &structGPIO);
	structGPIO.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 /*| GPIO_Pin_14*/;
	GPIO_Init(GPIOD, &structGPIO);
	//configure ADC
	/*power-on voltage reference*/
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	Delay(100);
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	Delay(100);
	ADC_VoltageRegulatorCmd(ADC3, ENABLE);
	Delay(100);
	ADC_VoltageRegulatorCmd(ADC4, ENABLE);
	Delay(100);
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
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable /*ADC_ContinuousConvMode_Disable*/;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_4; 
	ADC_Init(ADC3, &ADC_InitStructure);
	ADC_Init(ADC4, &ADC_InitStructure);
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_RegSimul;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC3, &ADC_CommonInitStructure);             
	ADC_CommonInit(ADC2, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC4, &ADC_CommonInitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_1Cycles5);     //514.3 kHz
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC4, ADC_Channel_3, 1, ADC_SampleTime_1Cycles5);
	
	ADC_RegularChannelSequencerLengthConfig(ADC1, 1);
	ADC_RegularChannelSequencerLengthConfig(ADC2, 1);
	ADC_RegularChannelSequencerLengthConfig(ADC3, 1);
	ADC_RegularChannelSequencerLengthConfig(ADC4, 1);
	 
	initial_opa();
	//init NVIC
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	return;
}

void initial_opa(void) {
	OPAMP_InitTypeDef OPAMP_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//OPA1
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO4;
  OPAMP_InitStructure.OPAMP_InvertingInput =  OPAMP_InvertingInput_Vout;
  OPAMP_Init(OPAMP_Selection_OPAMP1, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP1, OPAMP_OPAMP_PGAGain_2, OPAMP_PGAConnect_No);
	//OPA2
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO3;
	OPAMP_Init(OPAMP_Selection_OPAMP2, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP2, OPAMP_OPAMP_PGAGain_2, OPAMP_PGAConnect_No);
	//OPA3
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO1;
	OPAMP_Init(OPAMP_Selection_OPAMP3, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP3, OPAMP_OPAMP_PGAGain_2, OPAMP_PGAConnect_No);
	//OPA4
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO1;
	OPAMP_Init(OPAMP_Selection_OPAMP4, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP4, OPAMP_OPAMP_PGAGain_2, OPAMP_PGAConnect_No);
	
	OPAMP_Cmd(OPAMP_Selection_OPAMP1, ENABLE);
	OPAMP_Cmd(OPAMP_Selection_OPAMP2, ENABLE);
	OPAMP_Cmd(OPAMP_Selection_OPAMP3, ENABLE);
	OPAMP_Cmd(OPAMP_Selection_OPAMP4, ENABLE);
}

void initial_tim8(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_Period = 36000;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
	TIM_SelectOnePulseMode(TIM8, TIM_OPMode_Single);
	
}

void start_ADC(void) {
	if(driver_stat.statusAdc != HALT_ADC)
		orders_to_stop();
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_Cmd(ADC4, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY) && !ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY) 
		 && !ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY) && !ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY));
	ADC_StartConversion(ADC1); 
	ADC_StartConversion(ADC2); 
	ADC_StartConversion(ADC3); 
	ADC_StartConversion(ADC4);
	initial_tim8();
	driver_stat.statusAdc = RUN_ADC;
	TIM_Cmd(TIM8, ENABLE);
	return;
}

void stop_ADC(void) {
	ADC_StopConversion(ADC1);
	ADC_StopConversion(ADC3);
	ADC_StopConversion(ADC2);
	ADC_StopConversion(ADC4);
	while((ADC1->CR & ADC_CR_ADSTP) || (ADC2->CR & ADC_CR_ADSTP) || (ADC3->CR & ADC_CR_ADSTP) || (ADC4->CR & ADC_CR_ADSTP));
	ADC_Cmd(ADC1, DISABLE);
	ADC_Cmd(ADC2, DISABLE);
	ADC_Cmd(ADC3, DISABLE);
	ADC_Cmd(ADC4, DISABLE);
	driver_stat.statusAdc = HALT_ADC;
	return;
}
void Delay(uint64_t nCount) {
  /* Capture the current local time */
  uint64_t timingdelay = LocalTime + nCount;  

  /* wait until the desired delay finish */  
  while(timingdelay > LocalTime);
	return;
}

void Time_Update() {
  LocalTime += SYSTEMTICK_PERIOD_MUS;
	if(!(LocalTime%TIME_MONITORING_PGA) && driver_stat.enable_monitoring == 1)
		monitoring_pga();
	return;
}

void adcInterrupt(void) {
	//ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
	*spi3 = adc12[0];
	//__DSB();
	*spi3 = adc12[1];
 	/*while(!(ADC3->ISR & ADC_IT_EOC));*/
	/*if(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET)
		cSetFlag++;*/
	/*ADC_ClearFlag(ADC3, ADC_FLAG_EOC);*/
	*spi1 = adc34[0];
	//__DSB();
	*spi1 = adc34[1];
	if(driver_stat.statusAdc == DURING_STOP_ADC)
		stop_ADC();
	return;
}

void initial_i2c(void) {
	GPIO_InitTypeDef structGPIO;
	I2C_InitTypeDef structI2c;
	NVIC_InitTypeDef NVIC_InitStructure;
	stat.numr =0;
	stat.is_tran = 0;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	structGPIO.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //i2c2
  structGPIO.GPIO_Mode = GPIO_Mode_AF;
	structGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	structGPIO.GPIO_OType = GPIO_OType_OD;
	structGPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &structGPIO);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);
	//init i2c
	structI2c.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	structI2c.I2C_DigitalFilter = 0;
	structI2c.I2C_Timing = 19 << 0 | 15 << 8 | 2 << 16 | 4 << 20;
	structI2c.I2C_Mode = I2C_Mode_I2C;
	structI2c.I2C_OwnAddress1 = I2C_OWN;
	structI2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	structI2c.I2C_Ack = I2C_Ack_Enable;
	I2C_Init(I2C2, &structI2c);
	//allowing mode fast mode plus driving capability for I2C2 pins
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
// 	SYSCFG_I2CFastModePlusConfig(SYSCFG_I2CFastModePlus_I2C2, ENABLE);
	//enable interrupt
	I2C_ITConfig(I2C2, I2C_IT_RXI | I2C_IT_TXI | I2C_IT_ADDRI | I2C_IT_STOPI, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	I2C_Cmd(I2C2, ENABLE);
}

void initial_mco(void) {
	GPIO_InitTypeDef structGPIO;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	structGPIO.GPIO_Pin = GPIO_Pin_8; //i2c2
  structGPIO.GPIO_Mode = GPIO_Mode_AF;
	structGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	structGPIO.GPIO_OType = GPIO_OType_PP;
	structGPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &structGPIO);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_0);
	RCC_MCOConfig(RCC_MCOSource_HSE);
}

void i2c_hand(void) {
	if(I2C_GetITStatus(I2C2, I2C_IT_ADDR) == SET) {
		handl_i2c_message();
		I2C_ClearFlag(I2C2, I2C_FLAG_ADDR);
		stat.is_tran = I2C_GetTransferDirection(I2C2);
		stat.numr = 0;
	}
	if(I2C_GetITStatus(I2C2, I2C_IT_STOPF) == SET) {
		handl_i2c_message();
		cTran++;
		I2C_ClearFlag(I2C2, I2C_FLAG_STOPF);
		stat.numr = 0;
	}
	if(I2C_GetITStatus(I2C2, I2C_IT_RXNE) == SET) {
		if(stat.numr < I2C_BUF_SIZE)
			stat.bufr[stat.numr++] = I2C_ReceiveData(I2C2);
	}
	return;
}

void handl_i2c_message(void) {
	if (stat.numr == 0)
		return;
	if(stat.is_tran == I2C_Direction_Transmitter) {
		switch (stat.bufr[0]) {
				case START_ADC:
					start_ADC();
					break;
				case STOP_ADC:
					orders_to_stop();
				break;
				case START_SPI0:
					*spi3 = 5;
						++spi3Start;
				break;
				case START_SPI1:
					*spi1 = 3;
					++spi1Start;
				break;
				case START_TEST:
					start_test();
				break;
				case STOP_TEST:
					orders_to_stop_test();
				case ENABLE_MONITORING:
					driver_stat.enable_monitoring = 1;
				break;
				case DISABLE_MONITORING:
					driver_stat.enable_monitoring = 0;
				break;
				case SET_GAIN:
					if(stat.numr == 5)
						setGainOut(&stat.bufr[1]);
				break;
				case CLEAR_SET_OVERLOAD:
					clear_set_debug();
				break;
		  }
		stat.numr = 0;
	}
	return;
}

void i2c_write(uint8_t* pBuffer, uint8_t num) {
	uint8_t i = 0;
	I2C_ITConfig(I2C2, I2C_IT_RXI | I2C_IT_TXI | I2C_IT_ADDRI | I2C_IT_STOPI, DISABLE);
	while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);
	I2C_TransferHandling(I2C2, I2C_SLAVE, num, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);
	for(; i < num; i++) {
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);
		I2C_SendData(I2C2, (uint8_t) I2C_SLAVE);
	}
	while(I2C_GetFlagStatus(I2C2, I2C_ISR_STOPF) == RESET); 
	I2C_ClearFlag(I2C2, I2C_ICR_STOPCF);
}

void pga2505_write(void) {
	int i =3;
	GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);
	for(; i >= 0; i--) {
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData16(SPI2, driver_stat.gain[i]);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
		SPI_I2S_ReceiveData16(SPI2);
	}
	while(SPI_GetTransmissionFIFOStatus(SPI2) != SPI_TransmissionFIFOStatus_Empty);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET);
	return;
}

//void 
void setGainOut(int8_t* gain) {
	uint8_t i;
	while(driver_stat.mut == 1){};
	driver_stat.mut = 1;
	for(i = 0; i < 4; i++)
		driver_stat.gain[i] = gain[i] & ~ 0x2000;
	pga2505_write();
	driver_stat.mut = 0;
	return;
}

void orders_to_stop(void) {
	if(driver_stat.statusAdc != HALT_ADC)
		driver_stat.statusAdc = DURING_STOP_ADC;
	return;
}

void orders_to_stop_test(void) {
		test_stat.statusTest = DURING_STOP_TEST;
	return;
}

void start_test(void) {
	test_stat.statusTest = RUN_TEST;
	return;
}

void initial_gpio(void) {
	GPIO_InitTypeDef structGPIO;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	//configure GPIO OUT
	structGPIO.GPIO_Pin = GPIO_Pin_12;
	structGPIO.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOE, &structGPIO);
}

void monitoring_pga(void) {
	int i = 3;
	char isResetPga = 0;
	if(driver_stat.mut)
		return;
	driver_stat.mut = 1;
	GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);
	for(; i >= 0; i--) {
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData16(SPI2, driver_stat.gain[i]);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
		if((driver_stat.rec_message[i] = SPI_I2S_ReceiveData16(SPI2)) != driver_stat.gain[i]) {
			isResetPga = 1;
		}
	}
	while(SPI_GetTransmissionFIFOStatus(SPI2) != SPI_TransmissionFIFOStatus_Empty);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET);
	if(isResetPga) {
		GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_SET);
		driver_stat.is_overload = 1;
	}
	driver_stat.mut = 0;
	return;
}

void clear_set_debug(void){
	GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_RESET);
	driver_stat.is_overload = 0;
}
