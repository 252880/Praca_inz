#include "driver_setup.h"

uint8_t bldc;
motor_t motor_L, motor_R;

motorConfigStuct_t motor_config = {0};

void motorsSetup(uint8_t actuation_type, uint8_t motor_type)
{
bldc = motor_type;

	LL_GPIO_InitTypeDef gpio_init = {0};

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	LL_TIM_DeInit(TIM3);
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM3, 999);
	LL_TIM_SetClockDivision(TIM3, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_GenerateEvent_UPDATE(TIM3);
	LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetCompareCH1(TIM3, 0);
	LL_TIM_OC_SetCompareCH3(TIM3, 0);


	if(motor_type == D_MOTOR_TYPE_STEPPER){
		LL_TIM_SetPrescaler(TIM3, 83);
		LL_TIM_OC_SetCompareCH1(TIM3, 500);
		LL_TIM_OC_SetCompareCH3(TIM3, 500);
		LL_TIM_OC_SetPolarity(TIM3,LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
		LL_TIM_OC_DisableFast(TIM3,LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
	}

	if(motor_type == D_MOTOR_TYPE_DC){
	LL_TIM_SetPrescaler(TIM3, 1);
	}

	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);

	if(actuation_type == D_MOTOR_CONTROL_TYPE_LPWM_RPWM)
	{
		LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
		LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
		LL_TIM_OC_SetCompareCH2(TIM3, 0);
		LL_TIM_OC_SetCompareCH4(TIM3, 0);
		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH4);
	}
	LL_TIM_EnableAllOutputs(TIM3);
	LL_TIM_EnableCounter(TIM3);

	//left motor out1 = gpio - PB4

	gpio_init.Pin = LL_GPIO_PIN_4;
	gpio_init.Pull = LL_GPIO_PULL_NO;
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_init.Alternate = LL_GPIO_AF_2;

	LL_GPIO_Init(GPIOB, &gpio_init);

	//left motor out2 = gpio = PB5
	if(actuation_type == D_MOTOR_CONTROL_TYPE_LPWM_RPWM)
	{
		//gpio - PB5
		gpio_init.Pin = LL_GPIO_PIN_5;
		gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
		gpio_init.Alternate = LL_GPIO_AF_2;
	}
	else //normal gpio mode
	{
		gpio_init.Pin = LL_GPIO_PIN_5;
		gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
		gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	}
	LL_GPIO_Init(GPIOB, &gpio_init);

	//right motor out1 = gpio - PB0
	gpio_init.Pin = LL_GPIO_PIN_0;
	gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_init.Alternate = LL_GPIO_AF_2;

	LL_GPIO_Init(GPIOB, &gpio_init);
	//right motor out2 = gpio - PB1
	if(actuation_type == D_MOTOR_CONTROL_TYPE_LPWM_RPWM)
	{
		//gpio - PB1
		gpio_init.Pin = LL_GPIO_PIN_1;
		gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
		gpio_init.Alternate = LL_GPIO_AF_2;
	}
	else //normal gpio mode
	{
		gpio_init.Pin = LL_GPIO_PIN_1;
		gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
		gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	}
	LL_GPIO_Init(GPIOB, &gpio_init);

	//configure motor structures
	motor_config.motor_type = motor_type;
	motor_config.actuation_type = actuation_type;
	motor_config.max_pwm_output = LL_TIM_GetAutoReload(TIM3);

	PWM_t pwm1 = {.timer = TIM3, .channel = LL_TIM_CHANNEL_CH1};
	PWM_t pwm2 = {.timer = TIM3, .channel = LL_TIM_CHANNEL_CH2};
	GPIO_t gpio = {.port = GPIOB, .pin = LL_GPIO_PIN_5};
	motor_Init(&motor_L, &motor_config, pwm1, pwm2, gpio);

	pwm1.channel = LL_TIM_CHANNEL_CH3;
	pwm2.channel = LL_TIM_CHANNEL_CH4;
	gpio.pin = LL_GPIO_PIN_1;
	motor_Init(&motor_R, &motor_config, pwm1, pwm2, gpio);
	return;

}

pidConfigStruct_t pid_config = {0};
pid_controller_t pid_L, pid_R;

void pidSetup(pidC inL, pidC outL, pidC inR, pidC outR, uint32_t bound)
{
	pid_config.inner_loop = inL;
	pid_config.outer_loop = outL;
	pid_config.output_bound = bound;
	pid_Init(&pid_L, &pid_config);

	pid_config.inner_loop = inR;
	pid_config.outer_loop = outR;
	pid_Init(&pid_R, &pid_config);
	return;
}

encoder_t enc_L, enc_R;

void encoderSetup(uint32_t enc_spr_L, uint32_t enc_spr_R)
{
	LL_TIM_InitTypeDef tim_init = {0};
	LL_GPIO_InitTypeDef gpio_init = {0};

	//encoder R
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/* encoder R gpio
	PA8   ------> TIM1_CH1
	PA9   ------> TIM1_CH2
	*/
	gpio_init.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
	gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_init.Pull = LL_GPIO_PULL_NO;
	gpio_init.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &gpio_init);

	LL_TIM_SetEncoderMode(TIM1, LL_TIM_ENCODERMODE_X4_TI12);
	LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV16_N5);
	LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_IC_SetActiveInput(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPrescaler(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV16_N5);
	LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
	tim_init.Prescaler = 0;
	tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
	tim_init.Autoreload = 65535;
	tim_init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	tim_init.RepetitionCounter = 0;
	LL_TIM_Init(TIM1, &tim_init);
	LL_TIM_EnableARRPreload(TIM1);
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM1);
	LL_TIM_EnableCounter(TIM1);

	//encoder L
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* encoder L gpio
	PA0   ------> TIM2_CH1
	PA1   ------> TIM2_CH2
	*/
	gpio_init.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
	gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio_init.Pull = LL_GPIO_PULL_NO;
	gpio_init.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOA, &gpio_init);

	LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
	LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV16_N5);
	LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV16_N5);
	LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
	tim_init.Prescaler = 0;
	tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
	tim_init.Autoreload = 65535;
	tim_init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM2, &tim_init);
	LL_TIM_EnableARRPreload(TIM2);
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM2);
	LL_TIM_EnableCounter(TIM2);

	encoder_Init(&enc_L, TIM2, enc_spr_L);
	encoder_Init(&enc_R, TIM1, enc_spr_R);
}

currentMeasurementConfigStruct_t curr_measure_config_L, curr_measure_config_R;
currentMeasurement_t curr_mot_L, curr_mot_R;

void currentMeasurementSetup(double gainAV)
{
	LL_ADC_InitTypeDef adc_init = {0};
	LL_ADC_REG_InitTypeDef adc_reg_init = {0};
	LL_ADC_CommonInitTypeDef adc_common_init = {0};

	LL_GPIO_InitTypeDef gpio_init = {0};

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/* current measurement adc gpio
	  PA5   ------> ADC1_IN5 - motor L current
	  PA4  ------> ADC1_IN4 - motor R current
	*/
	gpio_init.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_4;
	gpio_init.Mode = LL_GPIO_MODE_ANALOG;
	gpio_init.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &gpio_init);

	adc_init.Resolution = LL_ADC_RESOLUTION_12B;
	adc_init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	adc_init.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
	LL_ADC_Init(ADC1, &adc_init);


	adc_reg_init.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	adc_reg_init.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
	adc_reg_init.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	//adc_reg_init.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	adc_reg_init.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
	adc_reg_init.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
	LL_ADC_REG_Init(ADC1, &adc_reg_init);
	LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
	adc_common_init.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &adc_common_init);

	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_28CYCLES);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_28CYCLES);

	LL_ADC_Enable(ADC1);


	curr_measure_config_L.adc = ADC1;
	curr_measure_config_L.adc_channel = LL_ADC_CHANNEL_5;
	curr_measure_config_L.gainAV = gainAV;
	currentMeasurement_Init(&curr_mot_L, &curr_measure_config_L);
	curr_measure_config_R.adc = ADC1;
	curr_measure_config_R.adc_channel = LL_ADC_CHANNEL_4;
	curr_measure_config_R.gainAV = gainAV;
	currentMeasurement_Init(&curr_mot_R, &curr_measure_config_R);
	return;
}

ppm_t ppm;
ppmConfigStruct_t ppm_config;

void ppmSetup(uint8_t ppm_no_of_channels, uint16_t ppm_max_t_us, uint16_t ppm_min_t_us,  uint16_t max_vel, uint8_t rc_type)
{
	LL_EXTI_InitTypeDef exti_init = {0};
	LL_GPIO_InitTypeDef gpio_init = {0};

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	//ppm gpio PB12 - external interrupt
	gpio_init.Pin = LL_GPIO_PIN_12;
	gpio_init.Mode = LL_GPIO_MODE_INPUT;
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpio_init.Pull = LL_GPIO_PULL_UP;

	LL_GPIO_Init(GPIOB, &gpio_init);

	exti_init.Line_0_31 = LL_EXTI_LINE_12;
	exti_init.LineCommand = ENABLE;
	exti_init.Mode = LL_EXTI_MODE_IT;
	exti_init.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
	LL_EXTI_Init(&exti_init);
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE12);

	NVIC_SetPriority(EXTI15_10_IRQn, 7);

	// us timer
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
	LL_TIM_InitTypeDef tim_init;

	tim_init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	tim_init.Prescaler = 83;
	tim_init.Autoreload = 4294967295;
	tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;

	LL_TIM_Init(TIM5, &tim_init);
	LL_TIM_EnableCounter(TIM5);

	// ppm struct init
	GPIO_t ppm_gpio = {.port = GPIOB, .pin = LL_GPIO_PIN_12};
	ppm_config.no_of_channels = ppm_no_of_channels;
	ppm_config.gpio = ppm_gpio;
	ppm_config.trigger_state = 1;
	ppm_config.max_t_us = ppm_max_t_us;
	ppm_config.min_t_us = ppm_min_t_us;
	ppm_config.max_vel = max_vel;
	ppm_config.rc_type = rc_type;
	ppm_Init(&ppm, &ppm_config);

	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}


UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

DMA_HandleTypeDef hdma_usart1_rx;



void MX_USART1_UART_Init(uint8_t rc_type)
{

	 huart1.Instance = USART1;
	 huart1.Init.WordLength = UART_WORDLENGTH_8B;
	 huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	 huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	     huart2.Instance = USART2;
	 	 huart2.Init.WordLength = UART_WORDLENGTH_8B;
	 	 huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	 	 huart2.Init.OverSampling = UART_OVERSAMPLING_16;


	 	if(rc_type == RC_TYPE_IBUS){
  huart1.Init.BaudRate = 115200;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;}




  if(rc_type == RC_TYPE_SBUS){
  huart1.Init.BaudRate = 100000;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity =  UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_RX;}



  huart2.Init.BaudRate = 115200;
   huart2.Init.StopBits = UART_STOPBITS_1;
   huart2.Init.Parity = UART_PARITY_NONE;
   huart2.Init.Mode = UART_MODE_TX_RX;


  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }


}




/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART1)
  {

    /* Peripheral clock enable */
	  __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;



    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;



    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart1_rx);


  }

  if(huart->Instance==USART2)
  {


	    /* Peripheral clock enable */
		  __HAL_RCC_USART2_CLK_ENABLE();

	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    /**USART2 GPIO Configuration
	    PA2     ------> USART1_TX
	    PA3     ------> USART1_RX
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {

    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);


    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
}

  if(huart->Instance==USART2)
   {

     /* Peripheral clock disable */
     __HAL_RCC_USART2_CLK_DISABLE();

     /**USART1 GPIO Configuration
     PA2     ------> USART1_TX
     PA3     ------> USART1_RX
     */
     HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);


     /* USART1 DMA DeInit */
     HAL_DMA_DeInit(huart->hdmarx);
 }


}





void MX_DMA_Init()
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}






safetyControllerConfigStruct_t safety_config;
safetyController_t safety;

void safetyControllerSetup(int16_t l0_min, int16_t l0_max, int16_t l1_min, int16_t l1_max)
{
	//init gpio
	LL_GPIO_InitTypeDef gpio_init = {0};
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	gpio_init.Pin = LL_GPIO_PIN_10;
	gpio_init.Mode = LL_GPIO_MODE_INPUT;
	gpio_init.Pull = LL_GPIO_PULL_UP;
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
	LL_GPIO_Init(GPIOA, &gpio_init);
	gpio_init.Pin = LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOB, &gpio_init);
	gpio_init.Pin = LL_GPIO_PIN_13;
	gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
	gpio_init.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &gpio_init);

	GPIO_t l0_gpio = {GPIOA, LL_GPIO_PIN_10};
	safety_config.level0_maxRPM = l0_max;
	safety_config.level0_minRPM = l0_min;
	safety_config.level0_gpio = l0_gpio;
	GPIO_t l1_gpio = {GPIOB, LL_GPIO_PIN_15};
	safety_config.level1_maxRPM = l1_max;
	safety_config.level1_minRPM = l1_min;
	safety_config.level1_gpio = l1_gpio;
	GPIO_t MotPow_gpio = {GPIOB, LL_GPIO_PIN_13};
	safety_config.MotorPower_gpio = MotPow_gpio;

	SafetyController_Init(&safety, &safety_config);
}

inputSourceSelectorConfigStruct_t input_source_config;
inputSourceSelector_t input_source;

void inputSourceSelectorSetup()
{
	// input source gpio - PB14
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_GPIO_InitTypeDef gpio_init;
	gpio_init.Pin = LL_GPIO_PIN_14;
	gpio_init.Mode = LL_GPIO_MODE_INPUT;
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpio_init.Pull = LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(GPIOB, &gpio_init);

	input_source_config.safetyController = &safety;
	GPIO_t input_source_gpio = {.port = GPIOB, .pin = LL_GPIO_PIN_14};
	input_source_config.inputSourceGpio = input_source_gpio;
	input_source_config.ppm = &ppm;

	inputSourceSelector_Init(&input_source, &input_source_config);
}

void loopsSetup()
{
	LL_TIM_InitTypeDef tim_init = {0};
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM10);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM11);

	tim_init.Prescaler = 83;
	tim_init.Autoreload = 9999;
	tim_init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;

	LL_TIM_Init(TIM10, &tim_init);
	LL_TIM_Init(TIM11, &tim_init);


	LL_TIM_EnableIT_UPDATE(TIM10);
	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 8);
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	LL_TIM_EnableIT_UPDATE(TIM11);
	NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 8);
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

	LL_TIM_EnableCounter(TIM10);
	LL_TIM_EnableCounter(TIM11);
}

void driverSetup(driverConfig_t* config)
{
	motorsSetup(config->actuation_type,config->motor_type);
	encoderSetup(config->enc_cpr_L, config->enc_cpr_R);
	currentMeasurementSetup(config->curr_measure_gainAV);
	pidSetup(config->inner_loop_pid_L, config->outer_loop_pid_L,
			 config->inner_loop_pid_R, config->outer_loop_pid_R,
			 config->pid_output_bound);
	ppmSetup(config->ppm_no_of_channels, config->ppm_max_t_us, config->ppm_min_t_us, config->max_vel, config->rc_type);
	safetyControllerSetup(config->safety_level0_minRPM, config->safety_level0_maxRPM,
						  config->safety_level1_minRPM, config->safety_level1_maxRPM);
	inputSourceSelectorSetup();
	  MX_DMA_Init();
	  MX_USART1_UART_Init(config->rc_type);

	loopsSetup();
}
