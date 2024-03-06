
#include "main.h"



#include "usb_device.h"
#include "driver_setup.h"
#include "message_handler.h"

#include "pid.h"






#define D_LOOP_TIME_MS 10
void SystemClock_Config(void);




driverConfig_t d_conf =
{
		.motor_type = D_MOTOR_TYPE_BLDC,
		.actuation_type = D_MOTOR_CONTROL_TYPE_UART,
		.rc_type = RC_TYPE_IBUS,
		.enc_cpr_L = 512*14,
		.enc_cpr_R = 512*14,
		.pid_output_bound = 1000000,
		.inner_loop_pid_L = {5000, 20, 0 },
		.inner_loop_pid_R = {5000, 20, 0 },
		.outer_loop_pid_L = {5000, 20, 0 },
		.outer_loop_pid_R = {5000, 20, 0 },
		.curr_measure_gainAV = 1.40f,
		.ppm_min_t_us = 1000,
		.ppm_max_t_us = 2000,
		.ppm_no_of_channels = IBUS_USER_CHANNELS,
		.max_vel = 400,
		.safety_level0_maxRPM = 150,
		.safety_level0_minRPM = 0,
		.safety_level1_maxRPM = 150,
		.safety_level1_minRPM = 0,
};

messageHandler_t msg_handler;
inputFormat_t usb_input;

uint8_t driver_standby_flag = 0, new_message_flag = 0, driver_configured = 1;

uint8_t DataToSend[40]; // Tablica zawierajaca dane do wyslania
uint8_t MessageCounter = 0; // Licznik wyslanych wiadomosci
uint8_t MessageLength = 0;




uint16_t ibus_data[IBUS_USER_CHANNELS];

uint8_t byte;

uint16_t failsafe_status;
uint8_t buf[25];
uint16_t CH[18];

int main(void)
{

	  HAL_Init();
	  SystemClock_Config();


		  MX_USB_DEVICE_Init();
		  driverSetup(&d_conf);

		if(d_conf.motor_type == D_MOTOR_TYPE_BLDC)
		  HAL_UART_Receive_IT(commUART, &byte, sizeof(byte));




if(d_conf.rc_type == RC_TYPE_SBUS)
	  HAL_UART_Receive_DMA(&huart1, buf, 25);




if(d_conf.rc_type == RC_TYPE_IBUS){
	  ibus_init();}


	  while (1)
	  {


if(d_conf.rc_type == RC_TYPE_IBUS){
		  ibus_read(ibus_data);
		  ibus_soft_failsafe(ibus_data, 10);
	      HAL_Delay(10);
}

	 if(new_message_flag)
		  {
			  messageHandler_ApplyMessage(&msg_handler);
			  MessageLength = sprintf(DataToSend, " nr %d\n\r", MessageCounter);
			  			CDC_Transmit_FS(DataToSend, MessageLength);
			  new_message_flag = 0;
		  }
	  }
}

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLQ_DIV_7);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(84000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    while(1);
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == IBUS_UART && d_conf.rc_type == RC_TYPE_SBUS){
	if (buf[0] == 0x0F) {
		CH[0] = (buf[1] >> 0 | (buf[2] << 8)) & 0x07FF;
		CH[1] = (buf[2] >> 3 | (buf[3] << 5)) & 0x07FF;
		CH[2] = (buf[3] >> 6 | (buf[4] << 2) | buf[5] << 10) & 0x07FF;
		CH[3] = (buf[5] >> 1 | (buf[6] << 7)) & 0x07FF;
		CH[4] = (buf[6] >> 4 | (buf[7] << 4)) & 0x07FF;
		CH[5] = (buf[7] >> 7 | (buf[8] << 1) | buf[9] << 9) & 0x07FF;
		CH[6] = (buf[9] >> 2 | (buf[10] << 6)) & 0x07FF;
		CH[7] = (buf[10] >> 5 | (buf[11] << 3)) & 0x07FF;
		CH[8] = (buf[12] << 0 | (buf[13] << 8)) & 0x07FF;
		CH[9] = (buf[13] >> 3 | (buf[14] << 5)) & 0x07FF;
		CH[10] = (buf[14] >> 6 | (buf[15] << 2) | buf[16] << 10) & 0x07FF;
		CH[11] = (buf[16] >> 1 | (buf[17] << 7)) & 0x07FF;
		CH[12] = (buf[17] >> 4 | (buf[18] << 4)) & 0x07FF;
		CH[13] = (buf[18] >> 7 | (buf[19] << 1) | buf[20] << 9) & 0x07FF;
		CH[14] = (buf[20] >> 2 | (buf[21] << 6)) & 0x07FF;
		CH[15] = (buf[21] >> 5 | (buf[22] << 3)) & 0x07FF;

		if (buf[23] & (1 << 0)) {
			CH[16] = 1;
		} else {
			CH[16] = 0;
		}

		if (buf[23] & (1 << 1)) {
			CH[17] = 1;
		} else {
			CH[17] = 0;
		}

		// Failsafe
		failsafe_status = SBUS_SIGNAL_OK;
		if (buf[23] & (1 << 2)) {
			failsafe_status = SBUS_SIGNAL_LOST;
		}

		if (buf[23] & (1 << 3)) {
			failsafe_status = SBUS_SIGNAL_FAILSAFE;
		}

		//	SBUS_footer=buf[24];

	}}

	if(huart == IBUS_UART && d_conf.rc_type == RC_TYPE_IBUS)
			ibus_reset_failsafe();


	if (huart == commUART) { 	//sprawdzenie czy przyszło z właściwego uarta
		data_available = 1;
		//ponowne wywołanie oczekiwania na przerwania dzieje się po przetworzeniu danych w hoverserial.c (linijka 74 w hoverserial.c)
	}
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }

}

