/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct package{
	uint8_t head;
	uint8_t length;
	uint8_t instruction;
	uint16_t parameter[6];
	uint16_t crc;
}package_typedef;

typedef struct error_package{
	uint8_t error_type;
	uint8_t error_info;
}error_package_typedef;

typedef struct via_point{
	uint16_t joint[4];
	uint16_t Chessboard;
}via_point_typedef;

typedef struct flag{
	uint8_t home_flag;
	uint8_t status_flag;
	uint8_t feedback_flag;
	uint8_t at_viapoint_flag ;
	uint8_t firsttime;
	uint8_t controlloop;
}flag_typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _motor_enable 0
#define _motor_disable 1
#define _dir_motor_ccw 0
#define _dir_motor_cw 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//system & flag
//test
float traj_T;
float traj_t;

uint8_t state = 0;
uint8_t input_package_flag = 0;
uint8_t command_finish = 0;
via_point_typedef via_point;

//communication
package_typedef process_package;
error_package_typedef error_package;
flag_typedef flag;
uint8_t input_package[17];
uint8_t output_package[50];
uint8_t package_count = 0;
uint8_t result_verify_package;

//movement
uint16_t Home_Set[4] = {0};
uint16_t Motor_Number[4] = {0,0,0,0};
uint16_t Dir_Motor[4] = {0};
uint16_t EncoderValue16[4]={0};  //J1 J2 J3 J4

//debug
uint16_t debug_count = 10;
uint16_t calculate_crc=0;
uint16_t crc_pack[10];
uint16_t count = 0;

//control

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void sethome();
static void Start_debug(uint16_t Parameter);
static void Update_Period(uint8_t Motor_Number[],double Freequency,float Duty_Cycle);
static uint16_t update_crc(uint16_t result, uint16_t *data_pack, uint16_t buff_size);
static uint8_t verify_package();
static void send_ack(uint8_t inst);
static void send_feedback(uint8_t inst);
static void send_status(uint8_t require_data);

static void send_error();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  //start PWM for motor J1 J2 J3 J4
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  //start Timer For encoder J1 J2 J3 J4
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  //start Timer For Control Loop
  HAL_TIM_Base_Start_IT(&htim6);
  //HAL_TIM_Base_Start(&htim7);
  //start DMA
  HAL_UART_Receive_DMA(&huart3, input_package, 17);
  HAL_UART_Receive_DMA(&huart5, input_package, 17);

  via_point.Chessboard = 1234;
  via_point.joint[0] = 123;
  via_point.joint[1] = 123;
  via_point.joint[2] = 123;
  via_point.joint[3] = 123;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(state){
	  	  case 0:
	  		 	 break;
	  	  case 99:
			  result_verify_package = verify_package();
			  if(result_verify_package){
				  state = process_package.instruction;
			  }
			  else{
				  //send error package
				  //send_error();
				  state = 0;
			  }
			  break;
		//set home
	  	  case 1:
			  send_ack(process_package.instruction);
			  sethome();
			  if(flag.home_flag){
				  send_ack(process_package.instruction);
				  flag.home_flag = 0;
				  state = 0;
			  }
			  break;
		//status mode
	  	  case 2:
	  		  send_ack(process_package.instruction);
	  		  send_status(process_package.parameter[0]);
	  		  if(flag.status_flag){
	  			send_ack(process_package.instruction);
	  			flag.status_flag = 0;
	  			state = 0;
	  		  }
	  		  break;
	  	//jog joint
	  	  case 3:
	  		if(flag.firsttime == 0){
	  			uint8_t moving_joint = 0;
				send_ack(process_package.instruction);
				if(process_package.parameter[0] & 0b00010000){
					flag.feedback_flag = 1;
				}
				moving_joint = process_package.parameter[0] % 0b00001111;
				for(int i = 0;i<4;i++){
					if((moving_joint >> i) == 1){
						via_point.joint[i] = process_package.parameter[1];
					}
				}
				flag.firsttime = 1;
	  		}
	  		if(flag.controlloop && flag.at_viapoint_flag == 0){

	  			//position control function

	  			//fortest
	  			flag.at_viapoint_flag = 1;

				if(flag.feedback_flag && count == 1000){
					send_feedback(process_package.instruction);
					count = 0;
				}
				count++;
				flag.controlloop = 0;
	  		}
	  		if(flag.at_viapoint_flag){
	  			send_ack(process_package.instruction);
	  			flag.at_viapoint_flag = 0;
	  			flag.feedback_flag = 0;
	  			flag.firsttime = 0 ;
	  			state = 0;
	  		}
	  		break;
	  	//linear jog
	  	  case 4:
	  		if(flag.firsttime == 0){
				send_ack(process_package.instruction);
				if(process_package.parameter[0] & 0b00010000){
					flag.feedback_flag = 1;
				}
				for(int i = 0;i<4;i++){
					via_point.joint[i] = process_package.parameter[i+1];
				}
				flag.firsttime = 1;
			}
	  		if(flag.controlloop){

				//position control function

	  			//fortest
	  			flag.at_viapoint_flag = 1;

				if(flag.feedback_flag){
					send_feedback(process_package.instruction);
				}
				flag.controlloop = 0;
			}
			if(flag.at_viapoint_flag){
				send_ack(process_package.instruction);
				flag.at_viapoint_flag = 0;
				flag.feedback_flag = 0;
				flag.firsttime = 0 ;
				state = 0;
			}
	  		break;
	  	//move
	  	  case 5:
			if(flag.firsttime == 0){
				send_ack(process_package.instruction);
				if(process_package.parameter[0] & 0b00010000){
					flag.feedback_flag = 1;
				}
				for(int i = 0;i<4;i++){
					via_point.joint[i] = process_package.parameter[i+1];
				}
				flag.firsttime = 1;
			}
			if(flag.controlloop){

				//casecade control function

				//fortest
				flag.at_viapoint_flag = 1;

				if(flag.feedback_flag){
					send_feedback(process_package.instruction);
				}
				flag.controlloop = 0;
			}
			if(flag.at_viapoint_flag){
				send_ack(process_package.instruction);
				flag.at_viapoint_flag = 0;
				flag.feedback_flag = 0;
				flag.firsttime = 0 ;
				state = 0;
			}
	  		break;
	  	//trajectory
	  	  case 6:
	  		package_count = 6;
	  		state = 0;
	  		break;
	  	//gripper
	  	  case 7:
	  		package_count = 7;
	  		state = 0;
	  		break;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//user function
void sethome(){
	flag.home_flag = 1;
}
void Update_Period(uint8_t Motor_Number[],double Freequency,float Duty_Cycle){

	/*******************************************************************************
	 **                                                                           **
	 **    Motor_Number : index 0 -> J1 , 1 -> J2 , 2 -> J3 , 3 -> J4             **
	 **    Freequency : Freequency that calculate from control or user command    **
	 **    Duty_Cycle : Duty cycle of pulse have value between 0 - 100            **
	 **                                                                           **
	 *******************************************************************************/

	uint32_t Period = 1000000/Freequency;
	uint32_t PWM = Period * Duty_Cycle / 100;

	if(Motor_Number[0] == 1){
		TIM13->ARR = Period; //update period
		TIM13->CCR1 = PWM;   //update duty cycle

	}
	else if(Motor_Number[1] == 1){
		TIM14->ARR = Period;
		TIM14->CCR1 = PWM;

	}
	else if(Motor_Number[2] == 1){
		TIM16->ARR = Period;
		TIM16->CCR1 = PWM;

	}
	else if(Motor_Number[3] == 1){
		TIM17->ARR = Period;
		TIM17->CCR1 = PWM;
	}
}
uint16_t update_crc(uint16_t result, uint16_t *data_pack, uint16_t buff_size){
	uint16_t i, j;
	uint16_t crc_table[256] = {
	        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
	        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
	        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
	        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
	        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
	        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
	        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
	        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
	        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
	        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
	        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
	        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
	        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
	        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
	        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
	        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
	        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
	        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
	        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
	        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
	        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
	        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
	        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
	        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
	        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
	        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
	        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	    };

	    for(j = 0; j <= buff_size-1; j++)
	    {
	        i = ((uint16_t)(result >> 8) ^ data_pack[j]) & 0xFF;
	        result = (result << 8) ^ crc_table[i];
	    }

	    return result;
}
uint8_t verify_package(){
	memset(process_package.parameter,0,sizeof(process_package.parameter));
	//debug mode
	if(process_package.instruction == 0x02){
		process_package.parameter[0] = input_package[3];
		process_package.crc = input_package[15] << 8 | input_package[16];
	}
	//set home
	else if(process_package.instruction == 0x01){
		process_package.crc = input_package[15] << 8 | input_package[16];
	}
	//jog joint
	else if(process_package.instruction == 0x03){
		process_package.parameter[0] = input_package[3];
		process_package.parameter[1] = input_package[4] << 8 | input_package[5];
		process_package.crc = input_package[15] << 8 | input_package[16];
	}
	//Cartesian jog, Move
	else if(process_package.instruction == 0x04 || process_package.instruction == 0x05){
		uint8_t index =1;
		process_package.parameter[0] = input_package[3];
		for(int i = 4;i <=10 ;i+=2){
			process_package.parameter[index] = input_package[i] << 8 | input_package[i+1];
			index++;
		}
		process_package.crc = input_package[15] << 8 | input_package[16];
	}
	//Trajectory move
	else if(process_package.instruction == 0x06){
		uint8_t index =0;
		for(int i = 3;i <=13 ;i+=2){
			process_package.parameter[index] = input_package[i] << 8 | input_package[i+1];
			index++;
		}
		process_package.crc = input_package[15] << 8 | input_package[16];
	}
	//crc check
	crc_pack[0] = process_package.head;
	crc_pack[1] = process_package.length;
	crc_pack[2] = process_package.instruction;
	for(int i = 3;i <=8 ;i++){
		crc_pack[i] = process_package.parameter[i-3];
	}
	calculate_crc = update_crc(0,crc_pack,sizeof(crc_pack)/2);
	if(process_package.crc == calculate_crc){
	  return 1;
	}
	else{
		error_package.error_type = 0x01;
		error_package.error_info = 0x01;
		return 0;
	}
}

void send_ack(uint8_t inst){
	memset(crc_pack,0,sizeof(crc_pack));
	memset(output_package,0,sizeof(output_package));
	output_package[0] = 0xFF;
	output_package[1] = 4;
	output_package[2] = (0x0A << 4) | inst;
	for(int  i = 0;i <= 3;i++){
			crc_pack[i] = output_package[i];
	}
	calculate_crc = update_crc(0,crc_pack,sizeof(crc_pack)/2);
	output_package[3] = calculate_crc / 256;
	output_package[4] = calculate_crc % 256;
	HAL_UART_Transmit(&huart3, output_package, 5,100);
}
void send_status(uint8_t require_data){
	uint8_t count = 2;
	memset(crc_pack,0,sizeof(crc_pack));
	memset(output_package,0,sizeof(output_package));

	output_package[0] = 0xFF;
	output_package[2] = 2;
	crc_pack[0] = 0xFF;
	crc_pack[2] = 1;

	if(((require_data & 128) >> 7) == 1){
		output_package[3] = via_point.Chessboard / 256;
		output_package[4] = via_point.Chessboard % 256;
		crc_pack[3] = via_point.Chessboard;
		count+=2;
	}
	if(((require_data & 64) >> 6) == 1){
		output_package[5] = via_point.joint[0] / 256;
		output_package[6] = via_point.joint[0] % 256;
		crc_pack[4] = via_point.joint[0];
		count+=2;
	}
	if(((require_data & 32) >> 5) == 1){
		output_package[7] = via_point.joint[1] / 256;
		output_package[8] = via_point.joint[1] % 256;
		crc_pack[5] = via_point.joint[1];
		count+=2;
	}
	if(((require_data & 16) >> 4) == 1){
		output_package[9] = via_point.joint[2] / 256;
		output_package[10] = via_point.joint[2] % 256;
		crc_pack[6] = via_point.joint[2];
		count+=2;
	}
	if(((require_data & 8) >> 3) == 1){
		output_package[11] = via_point.joint[3] / 256;
		output_package[12] = via_point.joint[3] % 256;
		crc_pack[7] = via_point.joint[3];
		count+=2;
	}
	if(((require_data & 4) >> 2) == 1){
		output_package[13] = 0 / 256;
		output_package[14] = 0 % 256;
		crc_pack[8] = 0;
		count+=2;
	}
	if(((require_data & 2) >> 1) == 1){
		output_package[15] = 0 / 256;
		output_package[16] = 0 % 256;
		crc_pack[9] = via_point.Chessboard;
		count+=2;
	}
	if((require_data & 1) == 1){
		output_package[17] = 0 / 256;
		output_package[18] = 0 % 256;
		crc_pack[10] = via_point.Chessboard;
		count+=2;
	}

	output_package[1] = count+2;
	crc_pack[1] = output_package[1];
	calculate_crc = update_crc(0, crc_pack, sizeof(crc_pack));
	output_package[count+1] = calculate_crc / 256;
	output_package[count+2] = calculate_crc % 256;
	HAL_UART_Transmit(&huart3, output_package, output_package[1]+1,100);
	flag.status_flag = 1;
}
void send_feedback(uint8_t inst){
	output_package[0] = 0xFF;
	output_package[2] = inst;
	crc_pack[0] = 0xFF;
	crc_pack[2] = inst;

	if(inst ==3 || inst == 4 || inst == 5){
		output_package[3] = TIM1->CNT / 256;
		output_package[4] = TIM1->CNT % 256;
		output_package[5] = TIM3->CNT / 256;
		output_package[6] = TIM3->CNT % 256;
		output_package[7] = TIM4->CNT / 256;
		output_package[8] = TIM4->CNT % 256;
		output_package[9] = TIM8->CNT / 256;
		output_package[10] = TIM8->CNT % 256;
		crc_pack[3] = TIM1->CNT;
		crc_pack[4] = TIM3->CNT;
		crc_pack[5] = TIM4->CNT;
		crc_pack[6] = TIM8->CNT;
		output_package[1] = 12;
		crc_pack[1] = output_package[1];
		calculate_crc = update_crc(0, crc_pack, sizeof(crc_pack));
		output_package[11] = calculate_crc / 256;
		output_package[12] = calculate_crc % 256;
		HAL_UART_Transmit(&huart3, output_package, 13,100);
		flag.feedback_flag = 1;
	}
	else if(inst == 6){
		output_package[3] = (uint8_t)(traj_T);
		output_package[4] = (uint8_t)(traj_t * 1000);
		output_package[5] = via_point.joint[0] / 256;
		output_package[6] = via_point.joint[0] % 256;
		output_package[7] = via_point.joint[1] / 256;
		output_package[8] = via_point.joint[1] % 256;
		output_package[9] = via_point.joint[2] / 256;
		output_package[10] = via_point.joint[2] % 256;
		output_package[11] = via_point.joint[3] / 256;
		output_package[12] = via_point.joint[3] % 256;
		crc_pack[3] = output_package[3];
		crc_pack[4] = output_package[4];
		crc_pack[5] = via_point.joint[0];
		crc_pack[6] = via_point.joint[1];
		crc_pack[7] = via_point.joint[2];
		crc_pack[8] = via_point.joint[3];
		output_package[1] = 14;
		crc_pack[1] = output_package[1];
		calculate_crc = update_crc(0, crc_pack, sizeof(crc_pack));
		output_package[13] = calculate_crc / 256;
		output_package[14] = calculate_crc % 256;
		HAL_UART_Transmit(&huart3, output_package, 15,100);
		flag.feedback_flag = 1;
	}
}
void send_error(){
	memset(crc_pack,0,sizeof(crc_pack));
	memset(output_package,0,sizeof(output_package));
	output_package[0] = 0xFF;
	output_package[1] = 5;
	output_package[2] = 0xEE;
	output_package[3] = (error_package.error_type << 4) | error_package.error_info;
	for(int  i = 0;i <= strlen(output_package);i++){
		crc_pack[i] = output_package[i];
	}
	calculate_crc = update_crc(0,crc_pack,sizeof(crc_pack)/2);
	output_package[4] = calculate_crc / 256;
	output_package[5] = calculate_crc % 256;
	HAL_UART_Transmit_DMA(&huart3, output_package, strlen(output_package));
}

//callback function

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_2){
		Home_Set[0] ^= 1;
	}
	else if(GPIO_Pin == GPIO_PIN_3){
		Home_Set[1] ^= 1;
	}
	else if(GPIO_Pin == GPIO_PIN_4){
		Home_Set[2] ^= 1;
	}
	else if(GPIO_Pin == GPIO_PIN_5){
		Home_Set[3] ^= 1;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart == &huart3){
		if(state == 0){
			process_package.head = input_package[0];
			process_package.length = input_package[1];
			process_package.instruction = input_package[2];
			state = 99;
		}
	}
	else if(huart == &huart5){
		if(state == 0){
			process_package.head = input_package[0];
			process_package.length = input_package[1];
			process_package.instruction = input_package[2];
			state = 99;
		}

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim6){
		//debug_count += 1;
		flag.controlloop = 1;
		//update variable for do control loop
	}
	else{
		debug_count = 99;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
