/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include "NRF24L01.h"

#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t ADC_value[2];

uint8_t TxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
uint8_t TxData[32];
uint8_t RxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
uint8_t RxData[32];

uint8_t data[32];

int row=0;
int col=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	/*******************************************************************************************************************************/

	//Switching variables

	int state0 = 0, flag0 = 0, Emergency_stop = 0, state1 = 0, flag1 = 0, comm_reset = 0, state2 = 0, flag2 = 0, Acc_mode = 0;
	int state3 = 0, flag3 = 0, Controller = 0, state4 = 0, flag4 = 0, Brake = 0, state5 = 0, flag5 = 0, step = 0;
	int stateR = 0,	Rx_reset = 0, flagR = 0;

	//ADC filter variables

	int M = 20;									//Filter length
	uint32_t x_raw = 0, y_raw = 0;
	//uint8_t MSG[80] = {'\0'};
	uint32_t xM1[M], yM1[M], xM2[M], yM2[M];

	//Controller variables

	double Jx = 0, Jy = 0, diff_x = 0, diff_y = 0;
	double x_out = 0, y_out = 0;
	double  dxdt[2], dydt[2], dx1dt[10], dy1dt[10];

	 double P = 4;
	 double I = 10;
	 double D = 1.6;

	double final_x = 0;
	double final_y = 0;

	//Received accelerometer data

	double Ax_1 = 0, Ay_1 = 0, Az_1 = 0;
	double Ax_2 = 0, Ay_2 = 0, Az_2 = 0;

	//LCD variables

	char buf[4];

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, ADC_value, 2);
  *ADC_value = HAL_ADC_GetValue(&hadc1);

  lcd_init ();
  lcd_send_string ("Initializing");
  lcd_put_cur(1, 0);
  lcd_send_string("Controller");
  HAL_Delay(2000);
  lcd_clear ();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

/*******************************************************************************************************************************/
/*******************************************************************************************************************************/

	  //Switching cases

/****************************************************************************************************************/

		//Emergency stop switch
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)){		//Read Emergency stop pin state
				  state0 = 1;
			  }
			  else{
				  state0 = 0;
			  }
		  if (state0 == 0 && Emergency_stop == 0 && flag0 == 0){		//Check that switch is pushed, switch state = off and iteration flag low  .0  .
				  Emergency_stop = 1;									//Set switch state = on
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
				  flag0 = 1;									//Set iteration flag
				}
				else if (state0 == 0 && Emergency_stop == 1 && flag0 == 0){	//Check that switch is pushed, switch state = on and iteration flag low
				  Emergency_stop = 0;										//Set switch state = off
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
				  flag0 = 1;											//Set iteration flag
				}
				else if (state0 == 1 && flag0 == 1){		//Check that switch is NOT pushed and iteration flag high
				  flag0 = 0;								//Reset iteration flag
				}

if (Emergency_stop == 0){
/****************************************************************************************************************/

	  //Wheel chair communication reset
	  	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)){		//Read reset pin state
	  	  		  state1 = 1;
	  	  	  }
	  	  	  else{
	  	  		  state1 = 0;
	  	  	  }
	  	  if (state1 == 0 && flag1 == 0){		//Check that switch is pushed, switch state = off and iteration flag low
	  		  	  comm_reset = 1;									//Set switch state = on
	  	  	      flag1 = 1;								//Set iteration flag
	  	  	    }
	  	  	    else if (state1 == 0 && comm_reset == 1 && flag1 == 0)		//Check that switch is pushed, switch state = on and iteration flag low
	  	  	    {
	  	  	      comm_reset = 0;										//Set switch state = off
	  	  	      flag1 = 1;											//Set iteration flag
	  	  	    }
	  	  	    else if (state1 == 1 && flag1 == 1){		//Check that switch is NOT pushed and iteration flag high
	  	  	      flag1 = 0;								//Reset iteration flag
	  	  	    }

	  	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)){		//Read reset pin state
	  		  	  		  stateR = 0;
	  		  	  	  }
	  		  	  	  else{
	  		  	  		  stateR = 1;
	  		  	  	  }
	  	  if (stateR == 0 && flagR == 0){		//Check that switch is pushed, switch state = off and iteration flag low
			  Rx_reset = 1;									//Set switch state = on
			  flagR = 1;								//Set iteration flag
			}
			else if (stateR == 0 && Rx_reset == 1 && flagR == 0)		//Check that switch is pushed, switch state = on and iteration flag low
			{
			  Rx_reset = 0;										//Set switch state = off
			  flagR = 1;											//Set iteration flag
			}
			else if (stateR == 1 && flagR == 1){		//Check that switch is NOT pushed and iteration flag high
			  flagR = 0;								//Reset iteration flag
			}


		//Acceleration mode select
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)){		//Read joy stick SW pin state
				  state2 = 1;
			  }
			  else{
				  state2 = 0;
			  }
		  if (state2 == 0 && Acc_mode == 0 && flag2 == 0){		//Check that switch is pushed, switch state = off and iteration flag low  .0  .
			  	  Acc_mode = 1;									//Set switch state = on
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
				  flag2 = 1;									//Set iteration flag
				}
				else if (state2 == 0 && Acc_mode == 1 && flag2 == 0){	//Check that switch is pushed, switch state = on and iteration flag low
				  Acc_mode = 0;										//Set switch state = off
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
				  flag2 = 1;											//Set iteration flag
				}
				else if (state2 == 1 && flag2 == 1){		//Check that switch is NOT pushed and iteration flag high
				  flag2 = 0;								//Reset iteration flag
				}

		//Controller status select
		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)){		//Read Controller status pin state
				  state3 = 1;
			  }
			  else{
				  state3 = 0;
			  }
		  if (state3 == 0 && Controller == 0 && flag3 == 0){		//Check that switch is pushed, switch state = off and iteration flag low  .0  .
			  	  Controller = 1;									//Set switch state = on
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
				  flag3 = 1;									//Set iteration flag
				}
				else if (state3 == 0 && Controller == 1 && flag3 == 0){	//Check that switch is pushed, switch state = on and iteration flag low
				  Controller = 0;										//Set switch state = off
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
				  flag3 = 1;											//Set iteration flag
				}
				else if (state3 == 1 && flag3 == 1){		//Check that switch is NOT pushed and iteration flag high
				  flag3 = 0;								//Reset iteration flag
				}

		 //Brake status select
		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)){		//Read Controller status pin state
				  state4 = 1;
			  }
			  else{
				  state4 = 0;
			  }
		  if (state4 == 0 && Brake == 0 && flag4 == 0){		//Check that switch is pushed, switch state = off and iteration flag low
			  	  Brake = 1;									//Set switch state = on
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
				  flag4 = 1;									//Set iteration flag
				}
				else if (state4 == 0 && Brake == 1 && flag4 == 0){	//Check that switch is pushed, switch state = on and iteration flag low
				  Brake = 0;										//Set switch state = off
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
				  flag4 = 1;											//Set iteration flag
				}
				else if (state4 == 1 && flag4 == 1){		//Check that switch is NOT pushed and iteration flag high
				  flag4 = 0;								//Reset iteration flag
				}

		//Step input status select
		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)){		//Read Controller status pin state
				  state5 = 1;
			  }
			  else{
				  state5 = 0;
			  }
		  if (state5 == 0 && step == 0 && flag5 == 0){		//Check that switch is pushed, switch state = off and iteration flag low  .0  .
			  	  step = 1;									//Set switch state = on
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
				  flag5 = 1;
				}
				else if (state5 == 0 && step == 1 && flag5 == 0){		//Check that switch is pushed, switch state = on and iteration flag low
				  step = 0;										//Set switch state = off
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
				  flag5 = 1;
				}
				else if (state5 == 1 && flag5 == 1){		//Check that switch is NOT pushed and iteration flag high
				  flag5 = 0;								//Reset iteration flag
				}

/*******************************************************************************************************************************/
/*******************************************************************************************************************************/

	  //ADC input to Moving average filter

		  if (Brake == 0){
			  x_raw = ADC_value[0];			//Get 'x-axis' ADC1 channel 0 raw data
			  y_raw = ADC_value[1];			//Get 'y-axis' ADC1 channel 1 raw data
		  }
		  if (Brake == 1){
			  x_raw = 2047.5;
			  y_raw = 2047.5;
		  }

	  int xtemp = 0, ytemp = 0;				//Declare temporary variables

	  for (int m = M - 1; m >= 0; m--){		//Use M-point arrays to store variable data
		  if (m > 0){						//For every instance other than the current time instant
			  xM1[m] = xM1[m-1];			//Shift values in array down by a single space
			  yM1[m] = yM1[m-1];
		  }
		  else if (m == 0){					//For only the current time instant
			  xM1[m] = x_raw;				//Store the value in the single space made by shifting other variables
			  yM1[m] = y_raw;
		  }

		  xtemp = xtemp + xM1[m];			//Sum of points
		  ytemp = ytemp + yM1[m];
	  }

	  for (int j = 0; j < 2; j++){			//Use 2-byte wide array to store previous and current output of filter
		  if (j < 1){						//For the previous instant
			  dxdt[j] = dxdt[j+1];			//Move the previous x-point to past position
			  dydt[j] = dydt[j+1];			//Move the previous y-point to past position
		  }
		  else if (j == 1){
			  dxdt[j] = xtemp / M;				//Calculate moving average
			  dydt[j] = ytemp / M;				//And move the point average to the current position
		  }

	  }

/*******************************************************************************************************************************/
/*******************************************************************************************************************************/

	  if (comm_reset == 1)
	  {
		  comm_reset = 0;						//Reset Communication reset flag
		  HAL_NVIC_SystemReset();				//Reset Main controller

	  }


	  //Controller

if (Controller == 1){

/************************************************************************************************/

	//Derivative control

	  diff_x = dxdt[1] - dxdt[0];			//Calculate the derivatives of the filtered inputs
	  diff_y = dydt[1] - dydt[0];

	  double xtemp2 = 0, ytemp2 = 0;		//Declare temporary variables

	  for (int j = 0; j < 10; j++){			//Filter derivative
	  		  if (j < 9){						//For the previous instants
	  			  dx1dt[j] = dx1dt[j+1];			//Move the previous x-point to past position
	  			  dy1dt[j] = dy1dt[j+1];			//Move the previous y-point to past position
	  		  }
	  		  else if (j == 9){
	  			  dx1dt[j] = diff_x;				//Add current derivative to array
	  			  dy1dt[j] = diff_y;
	  		  }
	  		  xtemp2 = xtemp2 + dx1dt[j];			//Calculate moving average
	  		  ytemp2 = ytemp2 + dy1dt[j];			//And move the point average to the current position
	  	  }

	  Jx = ( -50 / (1 + exp(xtemp2/(M*25))) ) + 25;
	  Jy = ( -50 / (1 + exp(ytemp2/(M*25))) ) + 25;

/************************************************************************************************/

	  //Integral controller

	  //X-axis Moving average filter 2 with derivative control

	  double xtemp1 = 0;		//Declare temporary variables

	  for (int m = M - 1; m >= 0; m--){		//Use M-point arrays to store variable data
		  if (m > 0){						//For every instance other than the current time instant
			  xM2[m] = xM2[m-1];			//Shift values in array down by a single space
		  }
		  else if (m == 0){					//For only the current time instant
			  xM2[m] = xtemp / M;				//Store the value in the single space made by shifting other variables
		  }
		  double x_control = ( P * exp(1/D*(m-I)) ) / ( (1+exp(1/D*(m-I)))*(1+exp(1/D*(m-I))) );
		  xtemp1 = xtemp1 + (xM2[m] * x_control);		//Sum of points to natural power of derivative
	  }

	  //Y-axis Moving average filter 3 with derivative control

	  double ytemp1 = 0;		//Declare temporary variables

	  for (int m = M - 1; m >= 0; m--){		//Use M-point arrays to store variable data
		  if (m > 0){						//For every instance other than the current time instant
			  yM2[m] = yM2[m-1];			//Shift values in array down by a single space
		  }
		  else if (m == 0){					//For only the current time instant
			  yM2[m] = ytemp / M;				//Store the value in the single space made by shifting other variables
		  }
		  double y_control = ( P * exp(1/D*(m-I)) ) / ( (1+exp(1/D*(m-I)))*(1+exp(1/D*(m-I))) );
		  ytemp1 = ytemp1 + (yM2[m] * y_control);		//Sum of points to natural power of derivative
	  }
		  x_out = xtemp1 / M;
		  y_out = ytemp1 / M;



/************************************************************************************************/

	//Proportion controller & combination

		final_x = 0.17435897 * ( 2*log(1+exp(0.02*(dxdt[1]-300))) + (-Jx) + x_out);			//Proportional & functional scaling of input
		final_y = 0.17435897 * ( 2*log(1+exp(0.02*(dydt[1]-300))) + (-Jy) + y_out);			//Sum of PID components

}
/*******************************************************************************************************************************/
}
			  //Switching functions

	else if (Emergency_stop == 1){
		final_x = 127.5;
		final_y = 127.5;
	  }

				  if (Controller == 0 && Emergency_stop == 0 && step == 0){
					  final_x = dxdt[1]/16.0588;
					  final_y = dydt[1]/16.0588;
				  }
				  if (step == 1 && Brake == 0 && Emergency_stop == 0){
					  final_x = 255;
					  final_y = 255;
				  }

/*******************************************************************************************************************************/
/*******************************************************************************************************************************/

	//Communication setup

				  TxData[0] = Emergency_stop;
				  TxData[1] = final_x;
				  TxData[2] = final_y;
				  TxData[3] = Acc_mode;
				  TxData[4] = Rx_reset;
				  TxData[5] = step;

				  if (Rx_reset == 1)
				  {
					  Rx_reset = 0;
				  }

				  NRF24_Init();
				  NRF24_TxMode(TxAddress, 10);
				  if (NRF24_Transmit(TxData) == 1)
					{
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

						NRF24_Init();
						NRF24_RxMode(RxAddress, 10);
						NRF24_ReadAll(data);

						if (isDataAvailable(1) == 1)
						  {
							NRF24_Receive(RxData);

							Ax_1 = (double)RxData[0];
							Ay_1 = (double)RxData[1];
							Az_1 = (double)RxData[2];

							Ax_2 = (double)RxData[3];
							Ay_2 = (double)RxData[4];
							Az_2 = (double)RxData[5];


							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

							//sprintf(MSG, "\r\nA_x1 = %d  A_y1 = %d  A_z1 = %d  G_x1 = %d  G_y1 = %d  G_z1 = %d\r\n\rA_x2 = %d  A_y2 = %d  A_z2 = %d  G_x2 = %d  G_y2 = %d  G_z2 = %d\r\n", (int)Ax_1, (int)Ay_1, (int)Az_1, (int)Gx_1, (int)Gy_1, (int)Gz_1, (int)Ax_2, (int)Ay_2, (int)Az_2, (int)Gx_2, (int)Gy_2, (int)Gz_2);
							//sprintf(MSG, "\r\nA_x1 = %d  A_y1 = %d  A_z1 = %d  \r\n\rA_x2 = %d  A_y2 = %d  A_z2 = %d\r\n", (int)Ax_1, (int)Ay_1, (int)Az_1, (int)Ax_2, (int)Ay_2, (int)Az_2);
							//HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 10);
						  }
					}

/*******************************************************************************************************************************/
/*******************************************************************************************************************************/

			//LCD screen

				  if (Ax_1 < 100){
					  lcd_put_cur(0, 2);
					  sprintf (buf, " ");
					  lcd_send_string (buf);
				  }
				  lcd_put_cur(0, 0);
				  sprintf (buf, "%d", (uint8_t)Ax_1);
				  lcd_send_string (buf);

				  if (Ay_1 < 100){
					  lcd_put_cur(0, 8);
					  sprintf (buf, " ");
					  lcd_send_string (buf);
				  }
				  lcd_put_cur(0, 5);
				  sprintf (buf, "|%d", (uint8_t)Ay_1);
				  lcd_send_string (buf);

				  if (Az_1 < 100){
					  lcd_put_cur(0, 14);
					  sprintf (buf, " ");
					  lcd_send_string (buf);
				  }
				  lcd_put_cur(0, 11);
				  sprintf (buf, "|%d", (uint8_t)Az_1);
				  lcd_send_string (buf);

				  if (Ax_2 < 100){
					  lcd_put_cur(1, 2);
					  sprintf (buf, " ");
					  lcd_send_string (buf);
				  }
				  lcd_put_cur(1, 0);
				  sprintf (buf, "%d", (uint8_t)Ax_2);
				  lcd_send_string (buf);

				  if (Ay_2 < 100){
					  lcd_put_cur(1, 8);
					  sprintf (buf, " ");
					  lcd_send_string (buf);
				  }
				  lcd_put_cur(1, 5);
				  sprintf (buf, "|%d", (uint8_t)Ay_2);
				  lcd_send_string (buf);

				  if (Az_2 < 100){
					  lcd_put_cur(1, 14);
					  sprintf (buf, " ");
					  lcd_send_string (buf);
				  }
				  lcd_put_cur(1, 11);
				  sprintf (buf, "|%d", (uint8_t)Az_2);
				  lcd_send_string (buf);





/*******************************************************************************************************************************/
/*******************************************************************************************************************************/


					if (Emergency_stop == 1){
						  HAL_Delay(750);
						  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
						  HAL_Delay(750);
						  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
					}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 70;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
