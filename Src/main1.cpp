/*
 * main1.cpp
 *
 *  Created on: 2019/02/27
 *      Author: natadecoco01c
 */
#include <main1.h>
#include <array>
#include "SolenoidNode.hpp"
#include "CarrierNode.hpp"
#include "can.hpp"
#include "led.h"
#include "solenoid_driver.h"
#include "stm32f1xx_hal.h"

CAN_HandleTypeDef hcan;

#define CAN_MTU 8
TIM_HandleTypeDef htim2;
uint16_t now_pattern = 0b00000000;
uint8_t once = 0;
uint8_t order;
uint8_t endis = 0;

template<typename T> //����͊֐��e���v���[�g�Ƃ������̂炵��
static void can_unpack(const uint8_t (&buf)[CAN_MTU], T &data);
template<typename T>
static void can_pack(uint8_t (&buf)[CAN_MTU], const T data);

static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void SystemClock_Config(void);
static void _Error_Handler(void);
static void MX_TIM2_Init(void);

static SolenoidNode * const solenoidNode = new SolenoidNode();
//static CarrierNode * const carrierNode = new CarrierNode();

int Counter_1 = 0;
int Counter_2 = 0;

CAN_RxHeaderTypeDef rx_msg;
CAN_TxHeaderTypeDef tx_msg8;
CAN_TxHeaderTypeDef tx_msg9;

uint32_t status;
uint8_t tx_payload8[CAN_MTU];
uint8_t tx_payload9[CAN_MTU];
uint8_t timetosend = 0;
//static constexpr uint16_t id_handStatus = 0x101;
static constexpr uint16_t id_handCmd = 0x102;

uint8_t cmd[CAN_MTU]; //受信データ格納用

int main1(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_TIM2_Init();
	HAL_NVIC_SetPriority(SysTick_IRQn, 0U, 0U);
	SystemClock_Config();

	can_init();
	can_set_bitrate(CAN_BITRATE_500K);

	// turn on green LED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	// blink red LED for test
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	can_enable(); //CAN�̗L����

	// loop forever

	tx_msg8.StdId = 0x100;
	tx_msg8.RTR = CAN_RTR_DATA;
	tx_msg8.IDE = CAN_ID_STD;
	tx_msg8.DLC = 1;
	tx_msg9.StdId = 0x101;
	tx_msg9.RTR = CAN_RTR_DATA;
	tx_msg9.IDE = CAN_ID_STD;
	tx_msg9.DLC = 1;

//	uint32_t last_stat_time = HAL_GetTick();
//	const uint32_t stat_interval = 1000 / 100; //分母がヘルツに

	HAL_NVIC_EnableIRQ (TIM2_IRQn);
	HAL_TIM_Base_Start_IT(&htim2);

//	NVIC_SetPriority(EXTI9_5_IRQn, 1);
//	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1U, 1U);
//	NVIC_EnableIRQ (EXTI9_5_IRQn);
//	HAL_NVIC_EnableIRQ (EXTI9_5_IRQn);

//status = can_rx(&rx_msg, 3);

	//nhk�̕��ł͏��CAN����̐M�����瓮���Ă����ǁA�����͂����Ȃ蓮���o���̂ŁA�ŏ��ɏ�����(�L����)���K�v
//	solenoidNode->SetCommand(SolenoidCommands::reset_cmd);
//	solenoidNode->Control();
	while (1) {
		now_pattern = 0b00000000;
		solenoid_drive(now_pattern);
		status = can_rx(&rx_msg, cmd);
		if (status == HAL_OK) {
			// received can frame
			if (rx_msg.StdId == id_handCmd) {
				can_unpack(cmd, order);
				switch (order) {
				case 0x00: //disable
					//solenoid_disable();
					//solenoid_drive(0b00000000);
					endis = 0;
					once = 0;
					break;
				case 0x01: //enable
					//solenoid_enable();
					endis = 1;
					break;
				}

			}
		}
		while (endis == 1) {
//	solenoid_enable();
//	solenoid_drive(now_pattern);
			solenoid_drive(0x00);
//	HAL_Delay(1000);
//	solenoid_drive(0xff);

			//carrierNode->SetCommand(CarrierCommands::reset_cmd);
			//carrierNodse->Control();

			//solenoidNode->SetCommand(SolenoidCommands::apploaching_s_cmd); //三宝接近というか三宝つかむようのアームを下げる部分　0b0000 0010正論
			//solenoidNode->Control();
			now_pattern &= ~0b00000001;
			now_pattern |= 0b00100010;
			solenoid_drive(now_pattern);
			//solenoidNode->SetCommand(SolenoidCommands::apploaching_a_cmd); //0b0000 0001　負 0b0010 0000の正 三宝持つとこ開いとく為
			//solenoidNode->Control()

			while (1) {
				//solenoidNode->Control();
				solenoid_drive(now_pattern);
				//carrierNode->SetCommand(CarrierCommands::deliver_l_cmd);
				//carrierNode->Control();
//		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != GPIO_PIN_RESET) {
//			HAL_Delay(5);
//			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != GPIO_PIN_RESET) {
//				Counter_1++;
//				switch (Counter_1) {
//				case 1:
//					solenoidNode->SetCommand(SolenoidCommands::picked_s_cmd); //0b0000 0001 正　0b0010 0000 負　の組み合わせをしたい
//					solenoidNode->Control();
//					break;
//				case 2:
//					solenoidNode->SetCommand(SolenoidCommands::up_s_cmd); //0b0000 0010の負論理を取る　三宝を保持してアームを曲げるところまでがここまで
//					solenoidNode->Control();
//					Counter_1 = 0;
//					break;
//				}
//			}
//		}
//
//		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) != GPIO_PIN_RESET) {
//			HAL_Delay(5);
//			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) != GPIO_PIN_RESET) {
//				Counter_2++;
//				switch (Counter_2) {
//				case 1:
//					solenoidNode->SetCommand(
//							SolenoidCommands::apploaching_f_cmd); //0b0000 0100の負論理
//					solenoidNode->Control();
//					break;
//				case 2:
//					solenoidNode->SetCommand(SolenoidCommands::picked_f_1_cmd); //0b0000 1000 果物回収用のアームを下ろす
//					solenoidNode->Control();
//					break;
//				case 3:
//					solenoidNode->SetCommand(SolenoidCommands::picked_f_2_cmd); //0b0000 0100 果物回収のアームを閉じてボールつかむ
//					solenoidNode->Control();
//					break;
//				case 4:
//					solenoidNode->SetCommand(SolenoidCommands::up_f_1_cmd); //0b0000 1000の負論理　アームを上げる
//					solenoidNode->Control();
//					break;
//				case 5:
//					solenoidNode->SetCommand(SolenoidCommands::up_f_2_cmd); //0b0000 0100の負論理　ボールを離してレールへ
//					solenoidNode->Control();
//					Counter_2 = 0;
//					break;
//				}
//			}
//		}

//			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != 0)			//�O��c��
//					{
//				Counter_1++;
//				switch (Counter_1) {
//				case 1:
//					solenoidNode->SetCommand(SolenoidCommands::picked_s_cmd);//�����ɑ��肽���M�����L�q�H
//					solenoidNode->Control();
//					break;
//				case 2:
//					solenoidNode->SetCommand(SolenoidCommands::up_s_cmd);
//					solenoidNode->Control();
//					Counter_1 = 0;
//					break;
//				}
//			}
//
//			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) != 0)			//�؂̎��c��
//					{
//				Counter_2++;
//				switch (Counter_2) {
//				case 1:
//					solenoidNode->SetCommand(
//							SolenoidCommands::apploaching_f_cmd);
//					solenoidNode->Control();
//					break;
//				case 2:
//					solenoidNode->SetCommand(SolenoidCommands::picked_f_1_cmd);
//					solenoidNode->Control();
//					break;
//				case 3:
//					solenoidNode->SetCommand(SolenoidCommands::picked_f_2_cmd);
//					solenoidNode->Control();
//					break;
//				case 4:
//					solenoidNode->SetCommand(SolenoidCommands::up_f_1_cmd);
//					solenoidNode->Control();
//					break;
//				case 5:
//					solenoidNode->SetCommand(SolenoidCommands::up_f_2_cmd);
//					solenoidNode->Control();
//					Counter_2 = 0;
//					break;
//				}
//			}
//			last_ctrl_time = HAL_GetTick();
//		}

				status = can_rx(&rx_msg, cmd);
				if (status == HAL_OK) {
					// received can frame
					if (rx_msg.StdId == id_handCmd) {
						can_unpack(cmd, order);
						switch (order) {
						case 0x00: //disable
							//solenoid_disable();
							endis = 0;
							once = 0;
							break;
						case 0x01: //enable
							//solenoid_enable();
							endis = 1;
							break;
						case 0x02: //果物回収
							//					solenoidNode->SetCommand(SolenoidCommands::apploaching_f_cmd); //0b0000 0100の正論理
							//					solenoidNode->Control();
							now_pattern |= 0b00000100;
							solenoid_drive(now_pattern);
							HAL_Delay(1000);
							//					solenoidNode->SetCommand(SolenoidCommands::picked_f_1_cmd); //0b0000 1000 果物回収用のアームを下ろす
							//					solenoidNode->Control();
							now_pattern |= 0b00001000;
							solenoid_drive(now_pattern);
							HAL_Delay(1000);
							//					solenoidNode->SetCommand(SolenoidCommands::picked_f_2_cmd); //0b0000 0100 負 果物回収のアームを閉じてボールつかむ
							//					solenoidNode->Control();
							now_pattern &= ~0b00000100;
							solenoid_drive(now_pattern);
							HAL_Delay(1000);
							//					solenoidNode->SetCommand(SolenoidCommands::up_f_1_cmd); //0b0000 1000の負論理　アームを上げる
							//					solenoidNode->Control();
							now_pattern &= ~0b00001000;
							solenoid_drive(now_pattern);
							HAL_Delay(2000);
							//					solenoidNode->SetCommand(SolenoidCommands::up_f_2_cmd); //0b0000 0100の正論理　ボールを離してレールへ
							//					solenoidNode->Control();
							now_pattern |= 0b00000100;
							solenoid_drive(now_pattern);
							break;
						case 0x03: //親善奉納
//					solenoidNode->SetCommand(SolenoidCommands::apploaching_a_cmd); //0b0000 0001　負 0b0010 0000の正
//					solenoidNode->Control();
							now_pattern &= ~0b00000001;
							now_pattern |= 0b00100000;
							solenoid_drive(now_pattern);
							HAL_Delay(700); //もうちょい短くて良くない？

//					solenoidNode->SetCommand(SolenoidCommands::setting_s_cmd); //0b0001 0000 最後の押し出し　いえい
//					solenoidNode->Control();
							now_pattern |= 0b00010000;
							solenoid_drive(now_pattern);
							break;
						}
					}
				}

				if (endis != 1) {
					break;
				}

				if (timetosend == 1) {
					can_tx(&tx_msg8, tx_payload8);
//					HAL_Delay(1);
//					can_tx(&tx_msg9, tx_payload9);
					timetosend = 0;
 			}

				if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != GPIO_PIN_RESET)
						&& (once != 1)) {
					HAL_Delay(5);
					//if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != GPIO_PIN_RESET) {

//				solenoidNode->SetCommand(SolenoidCommands::picked_s_cmd); //0b0000 0001 正　0b0010 0000 負　の組み合わせをしたい
//				solenoidNode->Control();
					now_pattern &= ~0b00100000;
					now_pattern |= 0b00000001;
					solenoid_drive(now_pattern);
					HAL_Delay(700);
//				solenoidNode->SetCommand(SolenoidCommands::up_s_cmd); //0b0000 0010の負論理を取る　三宝を保持してアームを曲げるところまでがここまで
//				solenoidNode->Control();
					now_pattern &= ~0b00000010;
					solenoid_drive(now_pattern);

					once = 1;
					//}
				}

			}
			if (endis != 1) {
				break;
			}
		}
	}
}
extern "C" void TIM2_IRQHandler(void) //送信レート100
		{
	if (TIM2->SR & TIM_SR_UIF) {
		uint8_t sensor8_status;
//		uint8_t sensor9_status;
//		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) != GPIO_PIN_RESET) {
//			sensor9_status = 1;
//		} else {
//			sensor9_status = 0;
//		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != GPIO_PIN_RESET) {
			sensor8_status = 1;
		} else {
			sensor8_status = 0;
		}

		can_pack(tx_payload8,sensor8_status);
		timetosend =1;
//　can packの部分がどっか行っちゃった　使わないそうなのでコメントアウト

		TIM2->SR &= ~TIM_SR_UIF;
	}
}
/*extern "C" void EXTI9_5_IRQHandler(void) { //おそらくcounter君をつかった処理は上手く行かないんじゃないかと　三宝回収はセンサのみ
 //	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET) {
 //		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
 if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != GPIO_PIN_RESET) {
 HAL_Delay(5);
 if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != GPIO_PIN_RESET) {

 solenoidNode->SetCommand(SolenoidCommands::picked_s_cmd); //0b0000 0001 正　0b0010 0000 負　の組み合わせをしたい
 solenoidNode->Control();

 solenoidNode->SetCommand(SolenoidCommands::up_s_cmd); //0b0000 0010の負論理を取る　三宝を保持してアームを曲げるところまでがここまで
 solenoidNode->Control();

 }
 }
 //	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET) {
 //		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
 //
 //			solenoidNode->SetCommand(SolenoidCommands::apploaching_f_cmd); //0b0000 0100の負論理
 //			solenoidNode->Control();
 //
 //			solenoidNode->SetCommand(SolenoidCommands::picked_f_1_cmd); //0b0000 1000 果物回収用のアームを下ろす
 //			solenoidNode->Control();
 //
 //			solenoidNode->SetCommand(SolenoidCommands::picked_f_2_cmd); //0b0000 0100 果物回収のアームを閉じてボールつかむ
 //			solenoidNode->Control();
 //
 //			solenoidNode->SetCommand(SolenoidCommands::up_f_1_cmd); //0b0000 1000の負論理　アームを上げる
 //			solenoidNode->Control();
 //
 //			solenoidNode->SetCommand(SolenoidCommands::up_f_2_cmd); //0b0000 0100の負論理　ボールを離してレールへ
 //			solenoidNode->Control();
 //
 //		}
 //	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
 EXTI->PR |= EXTI_PR_PR9;
 EXTI->PR |= EXTI_PR_PR8;
 HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
 HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
 }*/
/* CAN init function */

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler();
	}
}

static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 48;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_4TQ; //1->4
	hcan.Init.TimeSeg2 = CAN_BS2_3TQ; //1->3
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE; //DISABLE->ENABLE
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		_Error_Handler();
	}

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
					| GPIO_PIN_15, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	/*Configure GPIO pins : PB0 PB1 PB12 PB13
	 PB14 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12 | GPIO_PIN_13
			| GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		_Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}
/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

