/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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


#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

__IO uint32_t     Transfer_Direction = 0;
__IO uint32_t     Xfer_Complete = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define sizeTableMoy 361


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */



//-------PV for i2c connection
uint8_t I2C_TxBuffer[20]="W\0\0L"; //buffer of i2c transmition
uint8_t I2C_RxBuffer[20]=""; //buffer of i2c reception
int dir=2; //type of command received (0->reception ; 1->transmission)




//-------PV for uart connection
uint8_t UART1_rxBuffer[110]; //buffer of IMU reception
uint8_t lidar_Rx[6800]; //buffer of lidar reception
uint8_t lidar_Tx[10]; //buffer of lidar transmition
char Wmode='F'; //imu reception mode
char Lmode='F'; //lidar reception mode



//-------PV for IMU reception
uint8_t AccData[8]; //acceleration data package
uint8_t AngVelData[8]; //angle velocity data package
uint8_t AngData[8]; //angle data package

int state=0; //state of byte acquisition
int Byte_Num=0; //number of byte received


//-------PV for Lidar reception
int StartLidar=1; //initialisation phase of lidar
int ReceiveInfo=0; //info received phase
int ReceiveSync=0; //synchronization received phase

uint8_t SyncByte=0; //value of syncbyte
uint8_t Checksum=0; //received checksum
uint8_t Check=0; //measured checksum

int stateLidar=0; //byte acquisition state
int indexLidar=0; //number of distance obtained in a data response
int startSample=1; //begining of new data acquisition
int S_bit=0, Sample=0; //value of Start bit (new angle of 360) and validation of sample

float Angle[3300]; //table of each angle measured
int Distance[3300]; // table of each distance measured in mm
int intDist=0, intAngle=0; //number of angle and distance obtained

uint8_t StartAngleLSB;  //lsb of start angle
uint8_t StartAngleMSB; //msb of start angle
uint16_t StartAngleByte; //Start angle in byte
uint16_t DistanceByte; //distance in byte

double StartAngle=0.0; //value of the new start angle
double LastAngle=0.0; //value of the last angle
double DiffAngle=0.0; //angle between two acquisition of data from lidar

float DistMoy[sizeTableMoy]; //mean distance for each degree (360)
int intMoy[sizeTableMoy]; //number of value obtained for each degree

double X=0, Y=0, Theta_N=0; //x and y calculated via lidar and real angle of the robot




//-------PV for Heartbeat
uint32_t beat = 0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void Call_UART(void);
void ClearBuffer(uint8_t* buffer, int size);
void Heartbeat(void);

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

	//sprintf(buf,"\r\n");
	//strcpy((char*)UART1_rxBuffer,buf);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */


  HAL_I2C_EnableListen_IT(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  if(Lmode=='T'){

	  //----get info command
	  lidar_Tx[0]=0xA5; //"¥";
	  lidar_Tx[1]=0x50; //"P";

	  HAL_UART_Transmit_IT(&huart2, lidar_Tx, 2);

	  while(StartLidar==1){ //waiting for transmission of info command

	  }

	  StartLidar=1;

	  while(ReceiveInfo==0){ //waiting for reception of info command
		  HAL_UART_Receive_IT(&huart2, lidar_Rx, 27);
	  }

	  //----express scan command (dense mode)
	  lidar_Tx[1]=0x82;
	  lidar_Tx[2]=0x05; //" ";
	  lidar_Tx[3]=0x00; //" ";
	  lidar_Tx[4]=0x00; //" ";
	  lidar_Tx[5]=0x00; //" ";
	  lidar_Tx[6]=0x00; //" ";
	  lidar_Tx[7]=0x00; //" ";
	  lidar_Tx[8]=0x22; //" ";


	  HAL_UART_Transmit_IT(&huart2, lidar_Tx, 9);

	  while(StartLidar==1){ //waiting for transmission of express scan command

	   }

	  StartLidar=1;

	  while(ReceiveSync==0){ //waiting for reception of express scan command synchronization bytes
		  HAL_UART_Receive_IT(&huart2, lidar_Rx, 7);
	  }
	  Lmode='F';
  }
  else{
	  Lmode='T';
  }

  //--------------start of main---------------
  while (1)
  {
	  Heartbeat(); //command of the led

	  //-------i2c request management
	  switch (dir){
	  	  case 0: //reception command received from master
	  		  dir=2;
	  		  HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, I2C_RxBuffer, sizeof(I2C_RxBuffer), I2C_FIRST_FRAME);
	  		  break;

	  	  case 1: //transmission command received from master
	  		  dir=2;
	  		  HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, I2C_TxBuffer, sizeof(I2C_TxBuffer),I2C_FIRST_FRAME);
	  		  break;

	  	  default:
	  		  break;
	  }


	  //-------uart reception management
	  if(Wmode=='F'){ //reception of IMU data
		  HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, sizeof(UART1_rxBuffer));
	  }
	  else if(Lmode=='F'){ //reception of lidar data
		  if(HAL_UART_Receive_IT(&huart2, lidar_Rx, sizeof(lidar_Rx))==HAL_OK){
		  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 64;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 80 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//-----UART connection management-----
//transmit info to uart port (used for lidar command only)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(StartLidar==1){ //initialisation of lidar
		StartLidar=0;
	}
}

//receiving and processing data from uart
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


	if (huart->Instance == USART1){ //------UART1 connection (IMU)------

		state=0; //initialization of state

		//Wmode='T';


		//Reception of Data in arrays
		for(int i=0;i<sizeof(UART1_rxBuffer);i++){
			switch (state){
				case 0:      //verification if data is received
					if((i<100) && UART1_rxBuffer[i]==0x55){
						state=1; //data received
					}
				  break;

				case 1:      //Verification of what type of data it is
					if(UART1_rxBuffer[i]==0x51){
								state=2; //acceleration data
							}
					else if(UART1_rxBuffer[i]==0x52){
						state=3; //angular velocity data
					}
					else if(UART1_rxBuffer[i]==0x53){
						state=4; //angle data
					}
					else{
						state=0; //Return to searching Data
					}
				 break;

				case 2: //Recuperation of Data in Acc Array
					if (Byte_Num<8){
						AccData[Byte_Num]=UART1_rxBuffer[i];
						Byte_Num++;
					}
					else if (Byte_Num==8){
						Byte_Num=0;
						state=0;
					}
				 break;

				case 3:  //Recuperation of Data in AngVel Array
					if (Byte_Num<8){
						AngVelData[Byte_Num]=UART1_rxBuffer[i];
						Byte_Num++;
					}
					else if (Byte_Num==8){
						Byte_Num=0;
						state=0;
					}
				 break;

				case 4:  //Recuperation of Data in Ang Array
					if (Byte_Num<8){
						AngData[Byte_Num]=UART1_rxBuffer[i];
						Byte_Num++;
					}
					else if (Byte_Num==8){
						Byte_Num=0;
						state=0;
					}
				 break;

				default:
					state=0;
					Byte_Num=0;
					break;
			}
		}


		//Inversion of Low bytes and High bytes in Data array

		int Value1;
		int Value2;
		int Value3;

		for(int j=0;j<4;j++){
			Value1=AccData[2*j];
			AccData[2*j]=AccData[2*j+1];
			AccData[2*j+1]=Value1;

			Value2=AngVelData[2*j];
			AngVelData[2*j]=AngVelData[2*j+1];
			AngVelData[2*j+1]=Value2;

			Value3=AngData[2*j];
			AngData[2*j]=AngData[2*j+1];
			AngData[2*j+1]=Value3;

		}

		//changing the value of Wz to transmit in the I2C transmit buffer
		I2C_TxBuffer[0]='W';
		I2C_TxBuffer[1]= (AngVelData[4]);
		I2C_TxBuffer[2]= (AngVelData[5]);

		//Lmode='F';
	}

	else if (huart->Instance == USART2){ //------UART2 connection (lidar)------


		Lmode='T';

		//Initialization of lidar
		if (StartLidar==1){
			if (ReceiveInfo==0){ //verification of info command
				ReceiveInfo=1;
			}
			else if (ReceiveSync==0){ //verification of scan request first package
				ReceiveSync=1;
				StartLidar=0;
			}
		}

		//processing data received after scan request
		else{

			//initialization of arrays and variables needed
			for(int k=0;k<intAngle;k++){
				Angle[k]=0;
				Distance[k]=0;
			}

			stateLidar=0;
			startSample=1;
			intAngle=0;
			intDist=0;


			//Reception of angle and distance Data in arrays
			for(int i=0; i<6800; i++){
				switch (stateLidar){
				case 0:      //verification if data is received and reception of checksum value
					if(((lidar_Rx[i] >> 4) == 0xa) && (6800-i >= 4)){
						SyncByte= (lidar_Rx[i] >> 4);
						Checksum= lidar_Rx[i] & (~0xa0);
						stateLidar=1;
					}
					break;

				case 1:     //end of verification if data is received and reception of checksum value
					if((lidar_Rx[i] >> 4) ==0x5){
						SyncByte= (SyncByte << 4) | (lidar_Rx[i] >> 4);
						Checksum= (Checksum | ((lidar_Rx[i] & (~0x50)) << 4));
						stateLidar=2;
					}
					else{ //if data isn't received then reboot of checksum and starting again state 0
						Checksum=0x00;
						SyncByte=0x00;
						stateLidar=0;
					}
					break;

				case 2:     //reception of angle LSB of current response package
					Check=0x0;
					StartAngleLSB = lidar_Rx[i];
					Check=Check ^ lidar_Rx[i];
					stateLidar = 3;
					break;

				case 3 :   //reception of angle MSB of current response package
					StartAngleMSB = lidar_Rx[i];
					StartAngleByte =((StartAngleMSB << 8 | StartAngleLSB));
					Check=Check ^ lidar_Rx[i];  //verification of the checksum from current response package

					LastAngle=StartAngle;

					StartAngle = (double)StartAngleByte;  //value of start Angle


					if(StartAngle >= 32768){ //verification of S bit (not used here)
						StartAngle = StartAngle/2.0;
						S_bit=1;
					}
					else{
						S_bit=0;
					}

					StartAngle = StartAngle/64.0;

					if(startSample==1 || Sample==0){
						startSample=0;
					}
					else{  //calculation of each angle from last response package
						DiffAngle=StartAngle - LastAngle;
						if(DiffAngle<0){
							DiffAngle+=360;
						}
						for(int j=0;j<40;j++){
							Angle[intAngle]= LastAngle + (DiffAngle/40)*j;
							if (Angle[intAngle]>360){
								Angle[intAngle]-=360;
							}
							intAngle++;
						}
					}

					indexLidar=0;
					if(6800-i >= 85){ //verification of data still to receive (if not enough then default state and wait for end of for)
						stateLidar = 4;
					}
					else{
						stateLidar=10; //default state
					}

					break;

				case 4: //reception of each distance LSB data from current response package
					DistanceByte = lidar_Rx[i];
					Check=Check ^ lidar_Rx[i];  //verification of the checksum from current response package

					stateLidar=5;


					break;

				case 5: //reception of each distance MSB data from current response package

					Check=Check ^ lidar_Rx[i];  //verification of the checksum from current response package

					DistanceByte = (lidar_Rx[i] << 8 | DistanceByte);
					Distance[intDist] = (int)(DistanceByte);  //value of current distance
					intDist++;

					if(indexLidar<39){  //verification of the end of current response package
						indexLidar++;
						stateLidar=4;
					}
					else{  //end of response package
						if(Check==Checksum){ //verification of checksum (if not good then sample not valid)
							Sample=1;
						}
						else{ //if sample not valid then deleting distance received from the current response package
							for(int j=0;j<40;j++){
								Distance[intDist-j] = '\0';
							}
							intDist-=40;
							Sample=0;
						}

						stateLidar=0;
					}


					break;

				default:
					break;

				}

			}


			//calculation of mean value of distance on each degree (not used, just for verification mean)
			//initialization of arrays
			for(int k = 0;k<sizeTableMoy;k++){
				DistMoy[k]=0;
				intMoy[k]=0;
			}

			//reception of distance values for each angle in an array
			for(int j=0;j<3300;j++){
				if(Distance[j]!=0){
					if((int)Angle[j]>=0 && (int)Angle[j]<=360)
					DistMoy[(int)(Angle[j])]+=(float)Distance[j]/1000.0;
					intMoy[(int)(Angle[j])]++;
				}
			}

			//calculation of mean value for each angle
			for(int n=0;n<sizeTableMoy;n++){
				if (intMoy[n]!=0){
					DistMoy[n]=DistMoy[n]/intMoy[n];
				}
			}




			//calculation of position via obstacle
			//initialization of variables
			double obstaclex; //distance x from obstacle
			double obstacley; //istance y from obstacle
			double newangle, dist;
			double x[4], y[4];  //value of x and y from each obstacle
			double AnglePlot[4], DistPlot[4]; //angle and distance arrays
			int intplot=0; //number of true obstacle
			for(int n=0;n<4;n++){ //initialization of arrays
				AnglePlot[n]=0;
				DistPlot[n]=0;
				x[n]=0;
				y[n]=0;
			}

			//verification of obstacle and putting it in array if obstacle is true
			for(int i = 0; i < indexLidar; i++){
				newangle = (double)Angle[i]-Theta_N*180/M_PI; //angle on the same position as the beginning
				dist=(double)Distance[i]/1000.0;  //value of distance
				if (newangle < 0) {
					newangle += 360;
				}
				if (90 < newangle && newangle < 180 && dist < 23) { //verification of first obstacle
					obstaclex = X - (cos(newangle-90)*dist);
					obstacley = Y - (sin(newangle-90)*dist);
					if (obstaclex > -2 && obstaclex < 0 && obstacley > -2 && obstacley < 0) { //verification if current obstacle is possible
						AnglePlot[0] = newangle;
						DistPlot[0] = dist;
					}
				}
				if (0 < newangle && newangle < 90 && dist < 23) { //verification of second obstacle
					obstaclex = X + (cos(newangle)*dist);
					obstacley = Y - (sin(newangle)*dist);
					if (obstaclex > 15.6 && obstaclex < 17.6 && obstacley > -2 && obstacley < 0) { //verification if current obstacle is possible
						AnglePlot[1] = newangle;
						DistPlot[1] = dist;
					}
				}
				if (270 < newangle && newangle < 360 && dist < 23) { //verification of third obstacle
					obstaclex = X + (cos(newangle-270)*dist);
					obstacley = Y + (sin(newangle-270)*dist);
					if (obstaclex > 15.6 && obstaclex < 17.6 && obstacley > 14 && obstacley < 16) { //verification if current obstacle is possible
						AnglePlot[2] = newangle;
						DistPlot[2] = dist;
					}
				}
				if (180 < newangle && newangle < 270 && dist < 23) { //verification of fourth obstacle
					obstaclex = X - (cos(newangle-180)*dist);
					obstacley = Y + (sin(newangle-180)*dist);
					if (obstaclex > -2 && obstaclex < 0 && obstacley > 14 && obstacley < 16) { //verification if current obstacle is possible
						AnglePlot[3] = newangle;
						DistPlot[3] = dist;
					}
				}
			}





			//calculation of each obstacle x and y value
			x[0] = DistPlot[0]*cos(AnglePlot[0]-90)-1;
			y[0] = DistPlot[0]*sin(AnglePlot[0]-90)-1;

			x[1] = 15.6-(DistPlot[1]*cos(AnglePlot[1])-1);
			y[1] = DistPlot[1]*sin(AnglePlot[1])-1;

			x[2] = 15.6-(DistPlot[2]*cos(AnglePlot[2]-270)-1);
			y[2] = 15-(DistPlot[2]*sin(AnglePlot[2]-270)-1);

			x[3] = DistPlot[3]*cos(AnglePlot[3]-180)-1;
			y[3] = 15-(DistPlot[3]*sin(AnglePlot[3]-180)-1);

			//verification of value of x and y for each obstacle
			for(int n=0;n<4;n++){
				if(DistPlot[n]!=0){
					X+=x[n];
					Y+=y[n];
					intplot++;
				}
			}

			//calculation of mean value of x and y (true position of robot)
			if(intplot!=0){
				X = (X/intplot)*1000.0;
				Y = (Y/intplot)*1000.0;
			}
			else{
				X=0;
				Y=0;
			}


			//changing the value of X and Y to transmit in the I2C transmit buffer
			I2C_TxBuffer[3]='L';
			I2C_TxBuffer[4]= (((unsigned long)X >> 8) & 0xff);
			I2C_TxBuffer[5]= ((unsigned long)X & 0xff);
			I2C_TxBuffer[6]= (((unsigned long)Y >> 8) & 0xff);
			I2C_TxBuffer[7]= ((unsigned long)Y & 0xff);

			Wmode='F';

		}

	}
}


//-----I2C connection management------
//command from master is received
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
	dir=TransferDirection; //request type (reception=0 ; transmission=1)
}

//activated when i2c transmit_IT is called
void HAL_I2C_SlaveTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	strcpy((char*)I2C_RxBuffer,""); //emptying the i2c rx buffer
}


//activated when i2c receive_IT is called
void HAL_I2C_SlaveRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	//Verification of information needed
	if (strcmp((char*)I2C_RxBuffer, "WL:") == 0){
		Theta_N = (double)((I2C_RxBuffer[3] << 8) & I2C_RxBuffer[4])/1000.0; //recuperation of angle theta measured by the ctrl stm32
	}
}


void Heartbeat() {
	if (HAL_GetTick() - beat > 500) { //verification of time for 500ms
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); //inversion of led (on/off)
		beat = HAL_GetTick(); //restarting the process
	}
}


//emptying a buffer
void ClearBuffer(uint8_t* buffer, int size){
	for (int i=0; i<size ; i++){
		buffer[i]='\0';
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
