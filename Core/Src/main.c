/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "TJ_MPU6050.h"
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

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__*/
//
//PUTCHAR_PROTOTYPE{
//		HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
//		return ch;
//}

//Ulrasonic
uint32_t echo1, echo2, echo3, echo4, delay;
float dis1, dis2, dis3, dis4;

float left, front, right, bottom;

//Motor A
uint32_t EncoderA;
float RPMA;


uint32_t input_capturesA[2] = {0};
uint32_t countA=1;
bool is_capture_doneA = false;

float RPMerrorA;
float RPMsetPointA = 10.00; //33

float DutyA;
int widthA=8000;//7200 10000
int midA;
int percentage;


//Motor B
uint32_t EncoderB;
float RPMB;

uint32_t input_capturesB[2] = {0};
uint32_t countB=1;
bool is_capture_doneB = false;

float RPMerrorB;
float RPMsetPointB = 10.00; //33

float DutyB;
int widthB=8000;//6000  9000
int midB;

//Test Pin
uint32_t pinIN;

//Proportional Term
float KpA = 180;//80.5; // 10;
float KpB = 180;//75.5;
//Integral Term
float IntegrationSumA;
float IntegrationSumB;
float KI = 0.01;
float Ia;
float Ib;

//Differences
uint32_t volatile capture_differenceA =0;

//StartUp
bool startFlag = false;

//Analyses
uint32_t analyses;

//Position States
int positionToDo;
int positionToDoPrevious;



//sensor cleaning
int pauseCheck;
float filterThresholdBottom = 60.0;
float filterThresholdRight = 60.0;
float filterThresholdLeft = 60.0;

//Filter Bottom
float ultraReadingsBottom[3];
uint8_t readingCountBottom = 1;
float bottomPast;

float pos0;
float pos1;


//Filter Right
float ultraReadingsRight[3];
uint8_t readingCountRight = 1;
float RightPast;

float posRight0;
float posRight1;

//Filter Left
float ultraReadingsLeft[3];
uint8_t readingCountLeft = 1;
float LeftPast;

float posLeft0;
float posLeft1;

//RightFlag
bool rightFlag;

//LeftFlag
bool LeftFlag;

//Top Flag
bool topFlag;

//Bottom Right Flag
bool bottomRightFlag = 0;

//Top Right Flag
bool topRightFlag = 0;

//Bottom Left Flag
bool bottomLeftFlag = 0;

//Top Left Flag
bool topLeftFlag = 0;

//Accelorometer and Gyroscope

//Gyro
float xValue, yValue, zValue;

float xValue_Cal = 0;
float yValue_Cal = 0;
float zValue_Cal = 0;
float initial_angle = 0.00;

//Accelorometer
float ACCxValue, ACCyValue, ACCzValue;

float ACCxValue_Cal = 0;
float ACCyValue_Cal = 0;
float ACCzValue_Cal = 0;

//Accelorometer and Gyroscope Timings
float tlast;
float tnow;
float dt;

//UART
char uartBuf[100];
uint8_t Rx_data[1];
uint8_t Tx_data[13] = "Hello World\r\n";


//Test Pin
uint32_t pinIN;

//Motor RPM
double timer2_cnt_freqA=0;
double timer2_cnt_resA=0;
double user_signal_time_periodA;
double user_signal_freqA=0;
char usr_msg[100];

uint32_t capture_differenceB =0;
double timer2_cnt_freqB=0;
double timer2_cnt_resB=0;
double user_signal_time_periodB =0;
double user_signal_freqB=0;
char usr_msgB[100];


//Contamination Sensor
uint32_t sensor1, sensor2, sensor3, sensor4, sensor5;
ADC_ChannelConfTypeDef sConfig = {0};

//Outside sensor Threshold
uint32_t outsideContaminationThreshold = 0;//1500; //220;
uint32_t move_up_down_time = 200;

//Accelerometer turn threshold
float accThreshold = -554.00;
float accThreshold_x = 45.00;

//Alignment counters

//Right Counter
uint16_t right_counter = 0;

//Middle Counter
uint16_t middle_counter = 0;

//Left Counter
uint16_t left_counter = 0;

//Frame Protection Threshold (cm)
uint32_t Frame_Threshold = 30.00;

//Condiotions Flag
uint32_t dayNight = 2; //0-Day 1-Night
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM13_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void readUltrasonicDistance();
void moveForward();
void moveReverse();
void turnLeftTrack();
void turnRightTrack();
void stop();
int positionScan();
void turnLeftTrackReverse();
void turnRightTrackReverse();
void UARTposition(int currentPosition);
void UARTposition2(int currentPosition);
void RPMCheck();
void filterBottom();
void filterRight();
void filterLeft();
void readContamination();
void UARTLeft(int LeftValue);
void UARTRight(int RightVALUE);
void UARTFront(int frontValue);
void UARTBottom(int BottomValue);
void UARTContaminationFound();
void UARTContaminationNOTFound();
void readAccelerometer();
void cleaningAction();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Receive_DMA(&huart2, Rx_data, 4);

	if (Rx_data[0] == 's') {
		HAL_UART_Transmit(&huart1, (uint8_t *)Tx_data, strlen(Tx_data), 10);

	}

}

RawData_Def myAcc_RawData, myGyro_RawData;

ScaledData_Def myAcc_ScaledData, myGyro_ScaledData;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMPUConfig;


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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_TIM13_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  //PWM
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);

  //Input Capture
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

  //Ulrasonic ECHO Capture and TRIG Start
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_Base_Start(&htim11);
  HAL_TIM_Base_Start(&htim11);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);


  HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_2);


  HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);


  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);


  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);

  TIM11->CCR1 = 3;

  //Start Condition for motors
  //motor A
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  //motor B
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);


  //UART RX DMA
  HAL_UART_Receive_DMA(&huart1, Rx_data, 1);


  //1.Initialize the MPU 6050 Module I2C interface
  MPU6050_Init(&hi2c2);
  //2. Configure the Accelerometer and Gyro Parameters
  myMPUConfig.Accel_Full_Scale = AFS_SEL_4g;
  myMPUConfig.Gyro_Full_Scale = FS_SEL_2000;
  myMPUConfig.ClockSource = Internal_8MHz;
  myMPUConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;    //Digital LPF
  myMPUConfig.Gyro_Full_Scale = FS_SEL_500;
  myMPUConfig.Sleep_Mode_Bit = 0;				 //0 - Normal mode || 1 - Sleep Mode
  MPU6050_Config(&myMPUConfig);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(startFlag == true){
		  //TurnLED on
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

		  //READ CURRENT POSITION FROM ULTRASONIC SENSORS
		  readUltrasonicDistance();

		  //DETERMINE CURRENT POSITION ON WINDOW
		  positionToDo = positionScan();

//		  pinIN = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);

//			readAccelerometer();
//			while(ACCxValue > accThreshold_x){
//				readAccelerometer();
//				//HAL_Delay(10);
//				//turnLeftTrackReverse();
//				//sprintf((char*)uartBuf, "Turning wheels %.2f\r\n", ACCzValue);
//				//HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
//				//turnLeftTrackReverse();
//			}

		  //readContamination();
		  switch (positionToDo) {

		  //BOTTOM RIGHT
			case 1:

				//STOP FOR 2.5 SEC
				stop();
				HAL_Delay(2500);

				//RE-READ ULTRASONIC SENSORS AND RECHECK POSITION
				readUltrasonicDistance();
				positionToDo = positionScan();
				if(positionToDo == 1){

					//UART CURRENT POSITION
					UARTposition(positionToDo);
////					UARTposition2(positionToDo);
//					UARTLeft(left);
//					UARTRight(right);
//					UARTFront(front);
//					UARTBottom(bottom);

					//SET AND CLEAR APPROPRIATE FLAGS
					bottomRightFlag = true;
					rightFlag = true;
					LeftFlag = false;
					topFlag = false;


					//Read Contamination Status
					readContamination();


					if(dayNight == 0){
						if( (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold) ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHERCK IF OTHER CORNERS HAVE BEEN LEFT OUT
							if( (topRightFlag == 0) || (bottomLeftFlag == 0) || (topLeftFlag == 0) ){
								moveForward();
								HAL_Delay(8000);
								stop();
							}else{
								stop();
							}
						}
					}else if (dayNight == 1) {
						if( (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold) ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHERCK IF OTHER CORNERS HAVE BEEN LEFT OUT
							if( (topRightFlag == 0) || (bottomLeftFlag == 0) || (topLeftFlag == 0) ){
								moveForward();
								HAL_Delay(8000);
								stop();
							}else{
								stop();
							}
						}
					}



				}
				break;

		   //RIGHT
			case 2:

				//UART CURRENT POSITION
				UARTposition(positionToDo);
////				UARTposition2(positionToDo);
//				UARTLeft(left);
//				UARTRight(right);
//				UARTFront(front);
//				UARTBottom(bottom);

				//SET AND CLEAR APPROPRIATE FLAGS
				rightFlag = true;
				LeftFlag = false;


				//Read Contamination Status
				readContamination();

				if(dayNight==0){
					if( (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold) ){
						//UART CONTAMINATION STATUS
						//UARTContaminationFound();
						//CONTAMINATION ACTIONS
						cleaningAction();
					}else{
						//UART CONTAMINATION STATUS
						//UARTContaminationNOTFound();
						//CHECK DIRECTION OF MOVEMENT

						//INCREMENT COUNTER
						right_counter++;
						if(right_counter == 100){
							right_counter=0;
							stop();
							HAL_Delay(250);
							//RE-ADJUST ALLIGNMENT
								readAccelerometer();
								if(ACCxValue > accThreshold_x){
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnLeftTrackReverse();
									}
								}else if(ACCxValue < accThreshold_x){
									while(ACCxValue < accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnRightTrackReverse();
									}
								}
						}

						if (topFlag == false) {
							moveForward();
						}else{
							moveReverse();
						}
				}

				}else if (dayNight==1) {
					if( (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold) ){
						//UART CONTAMINATION STATUS
						//UARTContaminationFound();
						//CONTAMINATION ACTIONS
						cleaningAction();
					}else{
						//UART CONTAMINATION STATUS
						//UARTContaminationNOTFound();
						//CHECK DIRECTION OF MOVEMENT

						//INCREMENT COUNTER
						right_counter++;
						if(right_counter == 100){
							right_counter=0;
							stop();
							HAL_Delay(250);
							//RE-ADJUST ALLIGNMENT
								readAccelerometer();
								if(ACCxValue > accThreshold_x){
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnLeftTrackReverse();
									}
								}else if(ACCxValue < accThreshold_x){
									while(ACCxValue < accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnRightTrackReverse();
									}
								}
						}

						if (topFlag == false) {
							moveForward();
						}else{
							moveReverse();
						}
				}
				}
				break;

		   //TOP RIGHT
			case 3:
				stop();
				HAL_Delay(2500);

				//RE-READ ULTRASONIC SENSORS AND RECHECK POSITION
				readUltrasonicDistance();
				positionToDo = positionScan();
				if(positionToDo == 3){

					//UART CURRENT POSITION
					UARTposition(positionToDo);
////					UARTposition2(positionToDo);
//					UARTLeft(left);
//					UARTRight(right);
//					UARTFront(front);
//					UARTBottom(bottom);

					//SET AND CLEAR APPROPRIATE FLAGS
					topRightFlag = true;
					rightFlag = true;
					topFlag = true;
					LeftFlag = false;


					//Read Contamination Status
					readContamination();
					if(dayNight==0){
						if(  (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold) ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHECK CORNERS ALREADY DONE OR NOT DONE

							if ((bottomRightFlag == true) && (bottomLeftFlag == false) && (topLeftFlag == false)) {
								//REVERSE
								moveReverse();
								HAL_Delay(4000);

								//TURN RIGHT TRACK REVERSE
								turnRightTrackReverse();
								HAL_Delay(8000);

								//REVERSE
								moveReverse();
								HAL_Delay(2000);

								//Turn LEFT TRACK REVERSE
								readAccelerometer();
								while(ACCyValue > accThreshold){
									readAccelerometer();
									turnLeftTrackReverse();
								}

								//REVERSE
								moveReverse();
								HAL_Delay(300);

							}else if (((bottomRightFlag == true) && (bottomLeftFlag == true) && (topLeftFlag == true))) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

							}else if ((bottomRightFlag == false)) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

							}
						}
					}else if (dayNight==1) {
						if(  (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold) ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHECK CORNERS ALREADY DONE OR NOT DONE

							if ((bottomRightFlag == true) && (bottomLeftFlag == false) && (topLeftFlag == false)) {
								//REVERSE
								moveReverse();
								HAL_Delay(4000);

								//TURN RIGHT TRACK REVERSE
								turnRightTrackReverse();
								HAL_Delay(8000);

								//REVERSE
								moveReverse();
								HAL_Delay(2000);

								//Turn LEFT TRACK REVERSE
								readAccelerometer();
								while(ACCyValue > accThreshold){
									readAccelerometer();
									turnLeftTrackReverse();
								}

								//REVERSE
								moveReverse();
								HAL_Delay(300);

							}else if (((bottomRightFlag == true) && (bottomLeftFlag == true) && (topLeftFlag == true))) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

							}else if ((bottomRightFlag == false)) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

							}
						}
					}


				}
				break;

		   //MIDDLE
			case 4:

				//UART CURRENT POSITION
				UARTposition(positionToDo);
////				UARTposition2(positionToDo);
//				UARTLeft(left);
//				UARTRight(right);
//				UARTFront(front);
//				UARTBottom(bottom);

				//Read Contamination Status
				readContamination();
				if(dayNight==0){
					if(  (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold) ){
						//UART CONTAMINATION STATUS
						//UARTContaminationFound();
						//CONTAMINATION ACTIONS
						cleaningAction();
					}else{
						//UART CONTAMINATION STATUS
						//UARTContaminationNOTFound();

						//INCREMENT COUNTER
						middle_counter++;
						if(middle_counter == 100){
							middle_counter=0;
							stop();
							HAL_Delay(250);
							//RE-ADJUST ALLIGNMENT
								readAccelerometer();
	//							if(ACCyValue > accThreshold){
	//								turnLeftTrack();
	//								turnRightTrack();
	//							}
								if(ACCxValue > accThreshold_x){
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnLeftTrackReverse();
									}
								}else if(ACCxValue < accThreshold_x){
									while(ACCxValue < accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnRightTrackReverse();
									}
								}
						}
						//CHECK DIRECTION OF MOVEMENT
						  if(topFlag == false){
							  moveForward();
						  }else{
							  moveReverse();
						  }
					}
				}else if (dayNight==1) {
					if(  (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold) ){
						//UART CONTAMINATION STATUS
						//UARTContaminationFound();
						//CONTAMINATION ACTIONS
						cleaningAction();
					}else{
						//UART CONTAMINATION STATUS
						//UARTContaminationNOTFound();

						//INCREMENT COUNTER
						middle_counter++;
						if(middle_counter == 100){
							middle_counter=0;
							stop();
							HAL_Delay(250);
							//RE-ADJUST ALLIGNMENT
								readAccelerometer();
	//							if(ACCyValue > accThreshold){
	//								turnLeftTrack();
	//								turnRightTrack();
	//							}
								if(ACCxValue > accThreshold_x){
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnLeftTrackReverse();
									}
								}else if(ACCxValue < accThreshold_x){
									while(ACCxValue < accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnRightTrackReverse();
									}
								}
						}
						//CHECK DIRECTION OF MOVEMENT
						  if(topFlag == false){
							  moveForward();
						  }else{
							  moveReverse();
						  }
					}
				}

				break;

		   //BOTTOM
			case 5:

				//STOP FOR 2.5 SEC
				stop();
				HAL_Delay(2500);

				//RE-READ ULTRASONIC SENSORS AND RECHECK POSITION
				readUltrasonicDistance();
				positionToDo = positionScan();
				if(positionToDo == 5){

					//UART CURRENT POSITION
					UARTposition(positionToDo);
////					UARTposition2(positionToDo);
//					UARTLeft(left);
//					UARTRight(right);
//					UARTFront(front);
//					UARTBottom(bottom);

					//SET AND CLEAR APPROPRIATE FLAGS
					topFlag = false;


					//Read Contamination Status
					readContamination();
					if(dayNight==0){
						if(  (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold) ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();

							//RE-ADJUST ALLIGNMENT
							readAccelerometer();
							if(ACCxValue > accThreshold_x){
								while(ACCxValue > accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnLeftTrackReverse();
								}
							}else if(ACCxValue < accThreshold_x){
								while(ACCxValue < accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnRightTrackReverse();
								}
							}

//							//STOP
//							stop();
//							HAL_Delay(200);

							//MOVE UP FROM BOTTOM FRAME
							moveForward();
							HAL_Delay(11000);
							stop();
						}
					}else if (dayNight==1) {
						if(  (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold) ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();

							//RE-ADJUST ALLIGNMENT
							readAccelerometer();
							if(ACCxValue > accThreshold_x){
								while(ACCxValue > accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnLeftTrackReverse();
								}
							}else if(ACCxValue < accThreshold_x){
								while(ACCxValue < accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnRightTrackReverse();
								}
							}

							//STOP
							stop();
							HAL_Delay(200);

							//MOVE UP FROM BOTTOM FRAME
							moveForward();
							HAL_Delay(8000);
							stop();
						}
					}

				}
				break;

		   //TOP
			case 6:

				//STOP FOR 2.5 SEC
				stop();
				HAL_Delay(2500);

				//RE-READ ULTRASONIC SENSORS AND RECHECK POSITION
				readUltrasonicDistance();
				positionToDo = positionScan();
				if(positionToDo == 6){

					stop();
					HAL_Delay(700);

					//UART CURRENT POSITION
					UARTposition(positionToDo);
////					UARTposition2(positionToDo);
//					UARTLeft(left);
//					UARTRight(right);
//					UARTFront(front);
//					UARTBottom(bottom);

					//SET AND CLEAR APPROPRIATE FLAGS
					topFlag = true;


					//Read Contamination Status
					readContamination();
					if(dayNight==0){
						if(  (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold)  ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHECK DIRECTION OF HORIZONTAL MOVEMENT

							//RE-ADJUST ALLIGNMENT
							readAccelerometer();
							if(ACCxValue > accThreshold_x){
								while(ACCxValue > accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnLeftTrackReverse();
								}
							}else if(ACCxValue < accThreshold_x){
								while(ACCxValue < accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnRightTrackReverse();
								}
							}
							stop();
							HAL_Delay(300);

							//CHECK POSITION STATUS TO DETERMINE TURN DIRECTION
							if ((LeftFlag == false) && (rightFlag == true)) {
								//REVERSE
								if((left <= Frame_Threshold) ){ //|| (right <= Frame_Threshold)
									moveReverse();
									HAL_Delay(3000);

									turnRightTrack();
									HAL_Delay(6000);

									moveReverse();
									HAL_Delay(500);

									//Turn LEFT TRACK REVERSE
									readAccelerometer();
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										turnLeftTrackReverse();
									}

								}else{
									moveReverse();
									HAL_Delay(4000);

									//TURN RIGHT TRACK REVERSE
									turnRightTrackReverse();
									HAL_Delay(8000);

									//STOP
									stop();
									HAL_Delay(500);

									moveReverse();
									HAL_Delay(2800);

									//Turn LEFT TRACK REVERSE
									readAccelerometer();
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										turnLeftTrackReverse();
									}
								}

							}else if ((LeftFlag == true) && (rightFlag == false)) {
								//REVERSE
									if( (right <= Frame_Threshold)){ //(left <= Frame_Threshold) ||
										moveReverse();
										HAL_Delay(3000);

										//TURN RIGHT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(6000);

										moveReverse();
										HAL_Delay(500);

										//Turn RIGHT TRACK REVERSE
										readAccelerometer();
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											turnRightTrackReverse();
										}
									}else{
										moveReverse();
										HAL_Delay(4000);

										//TURN RIGHT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(8000);

										moveReverse();
										HAL_Delay(2800);

										//Turn RIGHT TRACK REVERSE
										readAccelerometer();
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											turnRightTrackReverse();
										}
									}

							}else if ((LeftFlag == false) && (rightFlag == false)) {
								//REVERSE
								moveReverse();
								HAL_Delay(4000);
								//REVERSE
								if((left <= Frame_Threshold) ){ //|| (right <= Frame_Threshold)
									moveReverse();
									HAL_Delay(1500);

									//TURN RIGHT TRACK REVERSE
									turnRightTrackReverse();
									HAL_Delay(6000);

									moveReverse();
									HAL_Delay(500);

									//Turn LEFT TRACK REVERSE
									readAccelerometer();
									while(ACCxValue > accThreshold_x){ //ACCyValue > accThreshold
										readAccelerometer();
										turnLeftTrackReverse();
									}

								}else{
									//TURN RIGHT TRACK REVERSE
									turnRightTrackReverse();
									HAL_Delay(8000);

									moveReverse();
									HAL_Delay(7000);
								}

								//STOP
								stop();
								HAL_Delay(500);

								//Turn LEFT TRACK REVERSE
								readAccelerometer();
								while(ACCxValue > accThreshold_x){ //ACCyValue > accThreshold
									readAccelerometer();
									turnLeftTrackReverse();
								}

								//REVERSE
								moveReverse();
								HAL_Delay(2800);
							}
						}
					}else if (dayNight==1) {
						if(  (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold)  ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHECK DIRECTION OF HORIZONTAL MOVEMENT

							//RE-ADJUST ALLIGNMENT
							readAccelerometer();
							if(ACCxValue > accThreshold_x){
								while(ACCxValue > accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnLeftTrackReverse();
								}
							}else if(ACCxValue < accThreshold_x){
								while(ACCxValue < accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnRightTrackReverse();
								}
							}
							stop();
							HAL_Delay(300);

							//CHECK POSITION STATUS TO DETERMINE TURN DIRECTION
							if ((LeftFlag == false) && (rightFlag == true)) {
								//REVERSE
								if((left <= Frame_Threshold) ){ //|| (right <= Frame_Threshold)
									moveReverse();
									HAL_Delay(3000);

									turnRightTrack();
									HAL_Delay(6000);

									moveReverse();
									HAL_Delay(500);

									//Turn LEFT TRACK REVERSE
									readAccelerometer();
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										turnLeftTrackReverse();
									}

								}else{
									moveReverse();
									HAL_Delay(4000);

									//TURN RIGHT TRACK REVERSE
									turnRightTrackReverse();
									HAL_Delay(6000);

									//STOP
									stop();
									HAL_Delay(500);

									moveReverse();
									HAL_Delay(2800);

									//Turn LEFT TRACK REVERSE
									readAccelerometer();
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										turnLeftTrackReverse();
									}
								}

							}else if ((LeftFlag == true) && (rightFlag == false)) {
								//REVERSE
									if( (right <= Frame_Threshold)){ //(left <= Frame_Threshold) ||
										moveReverse();
										HAL_Delay(3000);

										//TURN RIGHT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(6000);

										moveReverse();
										HAL_Delay(500);

										//Turn RIGHT TRACK REVERSE
										readAccelerometer();
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											turnRightTrackReverse();
										}
									}else{
										moveReverse();
										HAL_Delay(4000);

										//TURN RIGHT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(8000);

										moveReverse();
										HAL_Delay(2800);

										//Turn RIGHT TRACK REVERSE
										readAccelerometer();
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											turnRightTrackReverse();
										}
									}

							}else if ((LeftFlag == false) && (rightFlag == false)) {
								//REVERSE
								moveReverse();
								HAL_Delay(4000);
								//REVERSE
								if((left <= Frame_Threshold) ){ //|| (right <= Frame_Threshold)
									moveReverse();
									HAL_Delay(1500);

									//TURN RIGHT TRACK REVERSE
									turnRightTrackReverse();
									HAL_Delay(6000);

									moveReverse();
									HAL_Delay(500);

									//Turn LEFT TRACK REVERSE
									readAccelerometer();
									while(ACCxValue > accThreshold_x){ //ACCyValue > accThreshold
										readAccelerometer();
										turnLeftTrackReverse();
									}

								}else{
									//TURN RIGHT TRACK REVERSE
									turnRightTrackReverse();
									HAL_Delay(8000);

									moveReverse();
									HAL_Delay(7000);
								}

								//STOP
								stop();
								HAL_Delay(500);

								//Turn LEFT TRACK REVERSE
								readAccelerometer();
								while(ACCxValue > accThreshold_x){ //ACCyValue > accThreshold
									readAccelerometer();
									turnLeftTrackReverse();
								}

								//REVERSE
								moveReverse();
								HAL_Delay(2800);
							}
						}
					}

				}
				break;

		   //BOTTOM LEFT
			case 7:

				//STOP FOR 2.5 SEC
				stop();
				HAL_Delay(2500);

				//RE-READ ULTRASONIC SENSORS AND RECHECK POSITION
				readUltrasonicDistance();
				positionToDo = positionScan();
				if(positionToDo == 7){

					//UART CURRENT POSITION
					UARTposition(positionToDo);
////					UARTposition2(positionToDo);
//					UARTLeft(left);
//					UARTRight(right);
//					UARTFront(front);
//					UARTBottom(bottom);

					//SET AND CLEAR APPROPRIATE FLAGS
					bottomLeftFlag = true;
					LeftFlag = true;
					rightFlag = false;
					topFlag = false;


					//Read Contamination Status
					readContamination();
					if(dayNight==0){
						if( (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold)  ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHERCK IF OTHER CORNERS HAVE BEEN LEFT OUT
							if ((topRightFlag == false) || (topLeftFlag == false) || (bottomRightFlag == false)) {
								moveForward();
								HAL_Delay(8000);
								stop();
							}else{
								stop();
							}
						}
					}else if (dayNight==1) {
						if( (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold)  ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHERCK IF OTHER CORNERS HAVE BEEN LEFT OUT
							if ((topRightFlag == false) || (topLeftFlag == false) || (bottomRightFlag == false)) {
								moveForward();
								HAL_Delay(8000);
								stop();
							}else{
								stop();
							}
						}
					}

				}
				break;

		   //LEFT
			case 8:
				//UART CURRENT POSITION
				UARTposition(positionToDo);
////				UARTposition2(positionToDo);
//				UARTLeft(left);
//				UARTRight(right);
//				UARTFront(front);
//				UARTBottom(bottom);

				//SET AND CLEAR APPROPRIATE FLAGS
				rightFlag = false;
				LeftFlag = true;

				//Read Contamination Status
				readContamination();
				if(dayNight==0){
					if(  (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold)  ){
						//UART CONTAMINATION STATUS
						//UARTContaminationFound();
						//CONTAMINATION ACTIONS
						cleaningAction();
					}else{
						//UART CONTAMINATION STATUS
						//UARTContaminationNOTFound();
						//CHECK DIRECTION OF MOVEMENT

						//INCREMENT COUNTER
						left_counter++;
						if(left_counter == 100){
							left_counter=0;
							stop();
							HAL_Delay(250);
							//RE-ADJUST ALLIGNMENT
								readAccelerometer();
								if(ACCxValue > accThreshold_x){
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnLeftTrackReverse();
									}
								}else if(ACCxValue < accThreshold_x){
									while(ACCxValue < accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnRightTrackReverse();
									}
								}
						}

						if (topFlag == false) {
							moveForward();
						}else{
							moveReverse();
						}
					}
				}else if (dayNight==1) {
					if(  (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold)  ){
						//UART CONTAMINATION STATUS
						//UARTContaminationFound();
						//CONTAMINATION ACTIONS
						cleaningAction();
					}else{
						//UART CONTAMINATION STATUS
						//UARTContaminationNOTFound();
						//CHECK DIRECTION OF MOVEMENT

						//INCREMENT COUNTER
						left_counter++;
						if(left_counter == 100){
							left_counter=0;
							stop();
							HAL_Delay(250);
							//RE-ADJUST ALLIGNMENT
								readAccelerometer();
								if(ACCxValue > accThreshold_x){
									while(ACCxValue > accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnLeftTrackReverse();
									}
								}else if(ACCxValue < accThreshold_x){
									while(ACCxValue < accThreshold_x){
										readAccelerometer();
										HAL_Delay(10);
										turnRightTrackReverse();
									}
								}
						}

						if (topFlag == false) {
							moveForward();
						}else{
							moveReverse();
						}
					}
				}

				break;

		   //TOP LEFT
			case 9:
				//STOP FOR 2.5 SEC
				stop();
				HAL_Delay(2500);

				//RE-READ ULTRASONIC SENSORS AND RECHECK POSITION
				readUltrasonicDistance();
				positionToDo = positionScan();
				if(positionToDo == 9){

					//UART CURRENT POSITION
					UARTposition(positionToDo);
////					UARTposition2(positionToDo);
//					UARTLeft(left);
//					UARTRight(right);
//					UARTFront(front);
//					UARTBottom(bottom);

					//SET AND CLEAR APPROPRIATE FLAGS
					topLeftFlag = true;
					topFlag = true;
					LeftFlag = true;
					rightFlag= false;

					//Read Contamination Status
					readContamination();
					if(dayNight==0){
						if(  (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold)  ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							UARTContaminationNOTFound();
							//CHECK CORNERS ALREADY DONE OR NOT DONE

							if ((bottomLeftFlag == true) && (bottomRightFlag == false) && (topRightFlag == false)) {
								//REVERSE
								moveReverse();
								HAL_Delay(4000);

								//Turn LEFT TRACK REVERSE
								turnLeftTrackReverse();
								HAL_Delay(8000);

								moveReverse();
								HAL_Delay(2000);

								//TURN RIGHT TRACK REVERSE
								readAccelerometer();
								while(ACCyValue > accThreshold){
									readAccelerometer();
									turnRightTrackReverse();
								}

								//REVERSE
								moveReverse();
								HAL_Delay(300);

							}else if (((bottomLeftFlag == true) && (bottomRightFlag == true) && (topLeftFlag == true))) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

							}else if ((bottomLeftFlag == false)) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);
							}
						}
					}else if (dayNight==1) {
						if(  (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold)  ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							UARTContaminationNOTFound();
							//CHECK CORNERS ALREADY DONE OR NOT DONE

							if ((bottomLeftFlag == true) && (bottomRightFlag == false) && (topRightFlag == false)) {
								//REVERSE
								moveReverse();
								HAL_Delay(4000);

								//Turn LEFT TRACK REVERSE
								turnLeftTrackReverse();
								HAL_Delay(8000);

								moveReverse();
								HAL_Delay(2000);

								//TURN RIGHT TRACK REVERSE
								readAccelerometer();
								while(ACCyValue > accThreshold){
									readAccelerometer();
									turnRightTrackReverse();
								}

								//REVERSE
								moveReverse();
								HAL_Delay(300);

							}else if (((bottomLeftFlag == true) && (bottomRightFlag == true) && (topLeftFlag == true))) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

							}else if ((bottomLeftFlag == false)) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);
							}
						}
					}

				}

				break;
		   //CLOSE TOP RIGHT
			case 10:
				//STOP FOR 2.5 SEC
				stop();
				HAL_Delay(2500);

				//RE-READ ULTRASONIC SENSORS AND RECHECK POSITION
				readUltrasonicDistance();
				positionToDo = positionScan();
				if(positionToDo == 10){

					//UART CURRENT POSITION
					UARTposition(positionToDo);
////					UARTposition2(positionToDo);
//					UARTLeft(left);
//					UARTRight(right);
//					UARTFront(front);
//					UARTBottom(bottom);


					//SET AND CLEAR APPROPRIATE FLAGS
					topRightFlag = true;
					rightFlag = true;
					topFlag = true;
					LeftFlag = false;


					//Read Contamination Status
					readContamination();
					if(dayNight==0){
						if(  (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold) ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHECK CORNERS ALREADY DONE OR NOT DONE

							//RE-ADJUST ALLIGNMENT
							readAccelerometer();
							if(ACCxValue > accThreshold_x){
								while(ACCxValue > accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnLeftTrackReverse();
								}
							}else if(ACCxValue < accThreshold_x){
								while(ACCxValue < accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnRightTrackReverse();
								}
							}
							stop();
							HAL_Delay(00);

							if ((bottomRightFlag == true) && (bottomLeftFlag == false) && (topLeftFlag == false)) {
								//REVERSE
								moveReverse();
								HAL_Delay(1000);

								//TURN RIGHT TRACK REVERSE
								turnRightTrackReverse();
								HAL_Delay(1000);

								//STOP
								stop();
								HAL_Delay(50);

								//REVERSE
								moveReverse();
								HAL_Delay(2000);

								//TURN RIGHT TRACK REVERSE
								turnRightTrackReverse();
								HAL_Delay(1000);

								//STOP
								stop();
								HAL_Delay(50);

								//REVERSE
								moveReverse();
								HAL_Delay(2000);

								//STOP
								stop();
								HAL_Delay(50);

								//TURN RIGHT TRACK REVERSE
								turnRightTrackReverse();
								HAL_Delay(7000);

								//Reverse
								moveReverse();
								HAL_Delay(2000);

								//Turn LEFT TRACK REVERSE
								readAccelerometer();
								while(ACCxValue > accThreshold_x){
									readAccelerometer();
									turnLeftTrackReverse();
								}

								//REVERSE
								moveReverse();
								HAL_Delay(300);

							}else if (((bottomRightFlag == true) && (bottomLeftFlag == true) && (topLeftFlag == true))) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

							}else if ((bottomRightFlag == false)) {
	//							//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

								//---TESTING PURPOSES---

	//							//REVERSE
	//							moveReverse();
	//							HAL_Delay(1000);
	//
	//							//STOP
	//							stop();
	//							HAL_Delay(1000);
	//
	//							//TURN RIGHT TRACK REVERSE
	//							turnRightTrackReverse();
	//							HAL_Delay(1000);
	//
	//							//STOP
	//							stop();
	//							HAL_Delay(1000);
	//
	//							//REVERSE
	//							moveReverse();
	//							HAL_Delay(2000);
	//
	//							//TURN RIGHT TRACK REVERSE
	//							turnRightTrackReverse();
	//							HAL_Delay(1000);
	//
	//							//STOP
	//							stop();
	//							HAL_Delay(1000);
	//
	//							//REVERSE
	//							moveReverse();
	//							HAL_Delay(2000);
	//
	//							//STOP
	//							stop();
	//							HAL_Delay(50);
	//
	//							//TURN RIGHT TRACK REVERSE
	//							turnRightTrackReverse();
	//							HAL_Delay(3000);
	//
	//							//Turn LEFT TRACK REVERSE
	//							readAccelerometer();
	//							while(ACCxValue > accThreshold_x){
	//								readAccelerometer();
	//								turnLeftTrackReverse();
	//							}
	//
	//							//REVERSE
	//							moveReverse();
	//							HAL_Delay(300);
							}
						}
					}else if (dayNight==1) {
						if(  (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold) ){
							//UART CONTAMINATION STATUS
							//UARTContaminationFound();
							//CONTAMINATION ACTIONS
							cleaningAction();
						}else{
							//UART CONTAMINATION STATUS
							//UARTContaminationNOTFound();
							//CHECK CORNERS ALREADY DONE OR NOT DONE

							//RE-ADJUST ALLIGNMENT
							readAccelerometer();
							if(ACCxValue > accThreshold_x){
								while(ACCxValue > accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnLeftTrackReverse();
								}
							}else if(ACCxValue < accThreshold_x){
								while(ACCxValue < accThreshold_x){
									readAccelerometer();
									HAL_Delay(10);
									turnRightTrackReverse();
								}
							}
							stop();
							HAL_Delay(00);

							if ((bottomRightFlag == true) && (bottomLeftFlag == false) && (topLeftFlag == false)) {
								//REVERSE
								moveReverse();
								HAL_Delay(1000);

								//TURN RIGHT TRACK REVERSE
								turnRightTrackReverse();
								HAL_Delay(1000);

								//STOP
								stop();
								HAL_Delay(50);

								//REVERSE
								moveReverse();
								HAL_Delay(2000);

								//TURN RIGHT TRACK REVERSE
								turnRightTrackReverse();
								HAL_Delay(1000);

								//STOP
								stop();
								HAL_Delay(50);

								//REVERSE
								moveReverse();
								HAL_Delay(2000);

								//STOP
								stop();
								HAL_Delay(50);

								//TURN RIGHT TRACK REVERSE
								turnRightTrackReverse();
								HAL_Delay(7000);

								//Reverse
								moveReverse();
								HAL_Delay(2000);

								//Turn LEFT TRACK REVERSE
								readAccelerometer();
								while(ACCxValue > accThreshold_x){
									readAccelerometer();
									turnLeftTrackReverse();
								}

								//REVERSE
								moveReverse();
								HAL_Delay(300);

							}else if (((bottomRightFlag == true) && (bottomLeftFlag == true) && (topLeftFlag == true))) {
								//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

							}else if ((bottomRightFlag == false)) {
	//							//REVERSE BACK TO BOTTOM CORNER
								moveReverse();
								HAL_Delay(4000);

								//---TESTING PURPOSES---

	//							//REVERSE
	//							moveReverse();
	//							HAL_Delay(1000);
	//
	//							//STOP
	//							stop();
	//							HAL_Delay(1000);
	//
	//							//TURN RIGHT TRACK REVERSE
	//							turnRightTrackReverse();
	//							HAL_Delay(1000);
	//
	//							//STOP
	//							stop();
	//							HAL_Delay(1000);
	//
	//							//REVERSE
	//							moveReverse();
	//							HAL_Delay(2000);
	//
	//							//TURN RIGHT TRACK REVERSE
	//							turnRightTrackReverse();
	//							HAL_Delay(1000);
	//
	//							//STOP
	//							stop();
	//							HAL_Delay(1000);
	//
	//							//REVERSE
	//							moveReverse();
	//							HAL_Delay(2000);
	//
	//							//STOP
	//							stop();
	//							HAL_Delay(50);
	//
	//							//TURN RIGHT TRACK REVERSE
	//							turnRightTrackReverse();
	//							HAL_Delay(3000);
	//
	//							//Turn LEFT TRACK REVERSE
	//							readAccelerometer();
	//							while(ACCxValue > accThreshold_x){
	//								readAccelerometer();
	//								turnLeftTrackReverse();
	//							}
	//
	//							//REVERSE
	//							moveReverse();
	//							HAL_Delay(300);
							}
						}
					}


				}
				break;
				   //CLOSE TOP LEFT
					case 11:
						//STOP FOR 2.5 SEC
						stop();
						HAL_Delay(2500);

						//RE-READ ULTRASONIC SENSORS AND RECHECK POSITION
						readUltrasonicDistance();
						positionToDo = positionScan();
						if(positionToDo == 11){

							//UART CURRENT POSITION
							UARTposition(positionToDo);
//		//					UARTposition2(positionToDo);
//							UARTLeft(left);
//							UARTRight(right);
//							UARTFront(front);
//							UARTBottom(bottom);


							//SET AND CLEAR APPROPRIATE FLAGS
							topLeftFlag = true;
							rightFlag = false;
							topFlag = true;
							LeftFlag = true;


							//Read Contamination Status
							readContamination();
							if(dayNight==0){
								if(  (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold) ){
									//UART CONTAMINATION STATUS
//									UARTContaminationFound();
									//CONTAMINATION ACTIONS
									cleaningAction();
								}else{
									//UART CONTAMINATION STATUS
									//UARTContaminationNOTFound();
									//CHECK CORNERS ALREADY DONE OR NOT DONE

									//RE-ADJUST ALLIGNMENT
									readAccelerometer();
									if(ACCxValue > accThreshold_x){
										while(ACCxValue > accThreshold_x){
											readAccelerometer();
											HAL_Delay(10);
											turnLeftTrackReverse();
										}
									}else if(ACCxValue < accThreshold_x){
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											HAL_Delay(10);
											turnRightTrackReverse();
										}
									}


									if ((bottomRightFlag == false) && (bottomLeftFlag == true) && (topRightFlag == false)) {
										//REVERSE
										moveReverse();
										HAL_Delay(1000);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(1000);

										//STOP
										stop();
										HAL_Delay(50);

										//REVERSE
										moveReverse();
										HAL_Delay(2000);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(1000);

										//STOP
										stop();
										HAL_Delay(50);

										//REVERSE
										moveReverse();
										HAL_Delay(2000);

										//STOP
										stop();
										HAL_Delay(50);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(7000);

										//REVERSE
										moveReverse();
										HAL_Delay(4500);

										//Turn RIGHT TRACK REVERSE
										readAccelerometer();
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											turnRightTrackReverse();
										}

										//REVERSE
										moveReverse();
										HAL_Delay(300);

									}else if (((bottomRightFlag == true) && (bottomLeftFlag == true) && (topLeftFlag == true))) {
										//REVERSE BACK TO BOTTOM CORNER
										moveReverse();
										HAL_Delay(4000);

									}else if ((bottomLeftFlag == false)) {
			//							//REVERSE BACK TO BOTTOM CORNER
										moveReverse();
										HAL_Delay(4000);

										//---TESTING PURPOSES---
	//
	//									//REVERSE
	//									moveReverse();
	//									HAL_Delay(1000);
	//
	//									//TURN LEFT TRACK REVERSE
	//									turnLeftTrackReverse();
	//									HAL_Delay(1000);
	//
	//									//STOP
	//									stop();
	//									HAL_Delay(50);
	//
	//									//REVERSE
	//									moveReverse();
	//									HAL_Delay(2000);
	//
	//									//TURN LEFT TRACK REVERSE
	//									turnLeftTrackReverse();
	//									HAL_Delay(1000);
	//
	//									//STOP
	//									stop();
	//									HAL_Delay(50);
	//
	//									//REVERSE
	//									moveReverse();
	//									HAL_Delay(2000);
	//
	//									//STOP
	//									stop();
	//									HAL_Delay(50);
	//
	//									//TURN LEFT TRACK REVERSE
	//									turnLeftTrackReverse();
	//									HAL_Delay(3000);
	//
	//									//Turn RIGHT TRACK REVERSE
	//									readAccelerometer();
	//									while(ACCxValue < accThreshold_x){
	//										readAccelerometer();
	//										turnRightTrackReverse();
	//									}
	//
	//									//REVERSE
	//									moveReverse();
	//									HAL_Delay(300);
									}
								}
							}else if (dayNight==1) {
								if(  (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold) ){
									//UART CONTAMINATION STATUS
									//UARTContaminationFound();
									//CONTAMINATION ACTIONS
									cleaningAction();
								}else{
									//UART CONTAMINATION STATUS
									//UARTContaminationNOTFound();
									//CHECK CORNERS ALREADY DONE OR NOT DONE

									//RE-ADJUST ALLIGNMENT
									readAccelerometer();
									if(ACCxValue > accThreshold_x){
										while(ACCxValue > accThreshold_x){
											readAccelerometer();
											HAL_Delay(10);
											turnLeftTrackReverse();
										}
									}else if(ACCxValue < accThreshold_x){
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											HAL_Delay(10);
											turnRightTrackReverse();
										}
									}


									if ((bottomRightFlag == false) && (bottomLeftFlag == true) && (topRightFlag == false)) {
										//REVERSE
										moveReverse();
										HAL_Delay(1000);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(1000);

										//STOP
										stop();
										HAL_Delay(50);

										//REVERSE
										moveReverse();
										HAL_Delay(2000);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(1000);

										//STOP
										stop();
										HAL_Delay(50);

//										//REVERSE
//										moveReverse();
//										HAL_Delay(2000);
//
//										//STOP
//										stop();
//										HAL_Delay(50);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(7000);

//										//REVERSE
//										moveReverse();
//										HAL_Delay(4500);

										//Turn RIGHT TRACK REVERSE
										readAccelerometer();
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											turnRightTrackReverse();
										}

										//REVERSE
										moveReverse();
										HAL_Delay(300);

									}else if (((bottomRightFlag == true) && (bottomLeftFlag == true) && (topLeftFlag == true))) {
										//REVERSE BACK TO BOTTOM CORNER
										moveReverse();
										HAL_Delay(4000);

									}else if ((bottomLeftFlag == false)) {
			//							//REVERSE BACK TO BOTTOM CORNER
										moveReverse();
										HAL_Delay(4000);

										//---TESTING PURPOSES---
	//
	//									//REVERSE
	//									moveReverse();
	//									HAL_Delay(1000);
	//
	//									//TURN LEFT TRACK REVERSE
	//									turnLeftTrackReverse();
	//									HAL_Delay(1000);
	//
	//									//STOP
	//									stop();
	//									HAL_Delay(50);
	//
	//									//REVERSE
	//									moveReverse();
	//									HAL_Delay(2000);
	//
	//									//TURN LEFT TRACK REVERSE
	//									turnLeftTrackReverse();
	//									HAL_Delay(1000);
	//
	//									//STOP
	//									stop();
	//									HAL_Delay(50);
	//
	//									//REVERSE
	//									moveReverse();
	//									HAL_Delay(2000);
	//
	//									//STOP
	//									stop();
	//									HAL_Delay(50);
	//
	//									//TURN LEFT TRACK REVERSE
	//									turnLeftTrackReverse();
	//									HAL_Delay(3000);
	//
	//									//Turn RIGHT TRACK REVERSE
	//									readAccelerometer();
	//									while(ACCxValue < accThreshold_x){
	//										readAccelerometer();
	//										turnRightTrackReverse();
	//									}
	//
	//									//REVERSE
	//									moveReverse();
	//									HAL_Delay(300);
									}
								}
							}


						}
				break;
				   //MEDIUM TOP LEFT
					case 12:
						//STOP FOR 2.5 SEC
						stop();
						HAL_Delay(2500);

						//RE-READ ULTRASONIC SENSORS AND RECHECK POSITION
						readUltrasonicDistance();
						positionToDo = positionScan();
						if(positionToDo == 12){

							//UART CURRENT POSITION
							UARTposition(positionToDo);
//		//					UARTposition2(positionToDo);
//							UARTLeft(left);
//							UARTRight(right);
//							UARTFront(front);
//							UARTBottom(bottom);


							//SET AND CLEAR APPROPRIATE FLAGS
							topLeftFlag = true;
							rightFlag = false;
							topFlag = true;
							LeftFlag = true;


							//Read Contamination Status
							readContamination();
							if(dayNight==0){
								if(  (sensor1 >= outsideContaminationThreshold) && (sensor2 >= outsideContaminationThreshold) && (sensor3 >= outsideContaminationThreshold) && (sensor4 >= outsideContaminationThreshold) && (sensor5 >= outsideContaminationThreshold) ){
									//UART CONTAMINATION STATUS
									//UARTContaminationFound();
									//CONTAMINATION ACTIONS
									cleaningAction();
								}else{
									//UART CONTAMINATION STATUS
									//UARTContaminationNOTFound();
									//CHECK CORNERS ALREADY DONE OR NOT DONE

									//RE-ADJUST ALLIGNMENT
									readAccelerometer();
									if(ACCxValue > accThreshold_x){
										while(ACCxValue > accThreshold_x){
											readAccelerometer();
											HAL_Delay(10);
											turnLeftTrackReverse();
										}
									}else if(ACCxValue < accThreshold_x){
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											HAL_Delay(10);
											turnRightTrackReverse();
										}
									}


									if ((bottomRightFlag == false) && (bottomLeftFlag == true) && (topRightFlag == false)) {
										//REVERSE
										moveReverse();
										HAL_Delay(1000);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(1000);

										//STOP
										stop();
										HAL_Delay(50);


										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(1000);

										//STOP
										stop();
										HAL_Delay(50);

										//REVERSE
										moveReverse();
										HAL_Delay(2000);

										//STOP
										stop();
										HAL_Delay(50);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(7000);

										//REVERSE
										moveReverse();
										HAL_Delay(4500);

										//Turn RIGHT TRACK REVERSE
										readAccelerometer();
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											turnRightTrackReverse();
										}

										//REVERSE
										moveReverse();
										HAL_Delay(300);

									}else if (((bottomRightFlag == true) && (bottomLeftFlag == true) && (topLeftFlag == true))) {
										//REVERSE BACK TO BOTTOM CORNER
										moveReverse();
										HAL_Delay(1000);

									}else if ((bottomLeftFlag == false)) {
			//							//REVERSE BACK TO BOTTOM CORNER
										moveReverse();
										HAL_Delay(1000);

										turnRightTrackReverse();
										HAL_Delay(5000);

										if(ACCxValue > accThreshold_x){
											while(ACCxValue > accThreshold_x){
												readAccelerometer();
												turnLeftTrackReverse();
											}
									}
								}
								}
							}else if (dayNight==1) {
								if(  (sensor1 <= outsideContaminationThreshold) && (sensor2 <= outsideContaminationThreshold) && (sensor3 <= outsideContaminationThreshold) && (sensor4 <= outsideContaminationThreshold) && (sensor5 <= outsideContaminationThreshold) ){
									//UART CONTAMINATION STATUS
									//UARTContaminationFound();
									//CONTAMINATION ACTIONS
									cleaningAction();
								}else{
									//UART CONTAMINATION STATUS
									//UARTContaminationNOTFound();
									//CHECK CORNERS ALREADY DONE OR NOT DONE

									//RE-ADJUST ALLIGNMENT
									readAccelerometer();
									if(ACCxValue > accThreshold_x){
										while(ACCxValue > accThreshold_x){
											readAccelerometer();
											HAL_Delay(10);
											turnLeftTrackReverse();
										}
									}else if(ACCxValue < accThreshold_x){
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											HAL_Delay(10);
											turnRightTrackReverse();
										}
									}


									if ((bottomRightFlag == false) && (bottomLeftFlag == true) && (topRightFlag == false)) {
										//REVERSE
										moveReverse();
										HAL_Delay(1000);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(1000);

										//STOP
										stop();
										HAL_Delay(50);

										//REVERSE
										moveReverse();
										HAL_Delay(2000);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(1000);

										//STOP
										stop();
										HAL_Delay(50);

										//REVERSE
										moveReverse();
										HAL_Delay(2000);

										//STOP
										stop();
										HAL_Delay(50);

										//TURN LEFT TRACK REVERSE
										turnLeftTrackReverse();
										HAL_Delay(7000);

										//REVERSE
										moveReverse();
										HAL_Delay(4500);

										//Turn RIGHT TRACK REVERSE
										readAccelerometer();
										while(ACCxValue < accThreshold_x){
											readAccelerometer();
											turnRightTrackReverse();
										}

										//REVERSE
										moveReverse();
										HAL_Delay(300);

									}else if (((bottomRightFlag == true) && (bottomLeftFlag == true) && (topLeftFlag == true))) {
										//REVERSE BACK TO BOTTOM CORNER
										moveReverse();
										HAL_Delay(1000);

									}else if ((bottomLeftFlag == false)) {
		//							//REVERSE BACK TO BOTTOM CORNER
									moveReverse();
									HAL_Delay(1000);

									turnRightTrackReverse();
									HAL_Delay(5000);

									if(ACCxValue > accThreshold_x){
										while(ACCxValue > accThreshold_x){
											readAccelerometer();
											turnLeftTrackReverse();
										}
									}


									}
								}
							}


						}
			default:
				break;
		}


	  }else{
		  //TurnLED off
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  //Turn Motors Off
		  stop();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  //ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 80;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 79;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 80-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 20000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 199;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 39999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 80-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 20000-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void readUltrasonicDistance(){
	  if(HAL_GetTick() - delay >= 100){
		  delay = HAL_GetTick();
		  echo1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
		  echo2 = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
		  echo3 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
		  echo4 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
//		  dis1 = echo1/58;
//		  dis2 = echo2/58;
//		  dis3 = echo3/58;
//		  dis4 = echo4/58;
		  left = echo1/58;
		  front = echo2/58;
		  right = echo3/58;
		  bottom = echo4/58;

		  //printf("Echo1: %u Dis1: %.2f Echo2: %u Dis2: %.2f Echo3: %u Dis3: %.2f Echo4: %u Dis4: %.2f\r\n", echo1, dis1, echo2, dis2, echo3, dis3, echo4, dis4);
		  printf("Echo1: %u left: %.2f Echo2: %u front: %.2f Echo3: %u right: %.2f Echo4: %u bottom: %.2f\r\n", echo1, left, echo2, front, echo3, right, echo4, bottom);
	  }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */

//   Encoder = __HAL_TIM_GET_COUNTER(&htim2);
//  __HAL_TIM_SET_COUNTER(&htim2, 0);
//

double timer2_cnt_freqA=0;
double timer2_cnt_resA=0;
double user_signal_time_periodA;
double user_signal_freqA=0;
char usr_msg[100];

uint32_t capture_differenceB =0;
double timer2_cnt_freqB=0;
double timer2_cnt_resB=0;
double user_signal_time_periodB =0;
double user_signal_freqB=0;
char usr_msgB[100];

if(htim->Instance == TIM2){
#if 1
 if(! is_capture_doneA)
 {
	 if(countA == 1)
	 {
		 input_capturesA[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
		 countA++;
	 }
	 else if (countA == 450)
	 {
		 input_capturesA[1] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
		 countA = 1;
		 is_capture_doneA = true;
	 }else{
	 countA++;
	 }



 }
#endif
}

if(htim->Instance == TIM5){
#if 1
 if(! is_capture_doneB)
 {
	 if(countB == 1)
	 {
		 input_capturesB[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
		 countB++;
	 }
	 else if (countB == 450)
	 {
		 input_capturesB[1] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
		 countB =1;
		 is_capture_doneB = true;
	 }else{
	 countB++;
	 }
 }
#endif
}

if(is_capture_doneA){
		  			if(input_capturesA[1] > input_capturesA[0]){
		  				capture_differenceA = input_capturesA[1] - input_capturesA[0];
		  			}else{
		  				capture_differenceA = (1000000 - input_capturesA[0]) + input_capturesA[1];

		  			}

		  		analyses = capture_differenceA;
		  		timer2_cnt_freqA = 1;
		  		timer2_cnt_resA = 1/timer2_cnt_freqA;
		  		user_signal_time_periodA = capture_differenceA * timer2_cnt_resA;


		  		//RPMA = 60000000/user_signal_time_periodA;
		  		RPMA = (4500000/user_signal_time_periodA); //RPM = pulseCount * 60000 /time for a single revolution /pulses per turn .... THIS GIVES: 450 * 60000/time for a single revolution/6

		  		RPMerrorA = (float)(RPMsetPointA - RPMA);
		  		IntegrationSumA += RPMerrorA;
		  		Ia = 0.5*KI*IntegrationSumA;


		  		DutyA = KpA*RPMerrorA + Ia;

		  		widthA = htim10.Instance->CCR1 + (int)DutyA;



		  		printf("RPM A = %.2f Error A = %.2f Duty A = %.2f width A = %u \r\n",RPMA, RPMerrorA, DutyA, widthA);

		  		is_capture_doneA = false;
}


//Motor B
if(is_capture_doneB)
		{
		if(input_capturesB[1] > input_capturesB[0])
			capture_differenceB = input_capturesB[1] - input_capturesB[0];
		else
			capture_differenceB = (1000000 - input_capturesB[0]) + input_capturesB[1];

	timer2_cnt_freqB = 1;
	timer2_cnt_resB = 1/timer2_cnt_freqB;
	user_signal_time_periodB = capture_differenceB * timer2_cnt_resB;


	//RPMB = 60000000/user_signal_time_periodB;
	RPMB = (4500000/user_signal_time_periodB); //RPM = pulseCount * 60000 /time for a single revolution /pulses per turn .... THIS GIVES: 450 * 60000/time for a single revolution/6
	RPMerrorB = (float)(RPMsetPointB - RPMB);

	IntegrationSumB += RPMerrorB;
	Ib = 0.5*KI*IntegrationSumB;

	DutyB = KpB*RPMerrorB +Ib;
	//midB = htim13.Instance->CCR1*(DutyB/100);
	widthB = htim13.Instance->CCR1 + (int)DutyB;



	printf("RPM B = %.2f Error B = %.2f Duty B = %.2f width B = %u \r\n",RPMB, RPMerrorB, DutyB, widthB);

	is_capture_doneB = false;

		}
	//Motor A
	htim10.Instance->CCR1 = widthA;
	//Motor B
	htim13.Instance->CCR1 = widthB;


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
	if(startFlag==true){
		startFlag = false;
	}else
		startFlag = true;
}


int positionScan(){

//	if((bottom <= 12.00) && (bottom > 1.01) && (right <= 12.00) && (right > 1.01) && (front > 12.00) && (left > 12.00)){ //BOTTOM RIGHT
//		return 1;
//	}else if ((front > 12.00) && (right <= 12.00) && (right > 1.01) && (bottom > 12.00) && (left > 12.00)) { //RIGHT
//		return 2;
//	}else if((front <= 12.00) && (right <= 12.00) && (right > 1.01) && (bottom > 12.00) && (left > 12.00)){	//TOP RIGHT
//		return 3;
//	}else if ((front > 12.00) && (right > 12.00) && (bottom > 12.00) && (left > 12.00)){ //MIDDLE
//		return 4;
//	}else if((front > 12.00) && (right > 12.00) && (bottom <= 12.00) && (bottom > 1.01) && (left > 12.00)){	//BOTTOM
//		return 5;
//	}else if((front <= 12.00) && (right > 6.00) && (bottom > 6.00) && (left > 6.00)){ //TOP
//		return 6;
//	}else if((front >= 12.00) && (right >= 12.00) && (bottom <= 12.00) && (left <= 12.00) && (left >= 1.01)){ //BOTTOM LEFT
//		return 7;
//	}else if((front >= 12.00) && (right >= 12.00) && (bottom > 12.00) && (left <= 12.00) && (left >= 1.01)){ //LEFT
//		return 8;
//	}else if((front <= 12.00) && (right > 12.00) && (bottom > 12.00) && (left <= 12.00) && (left >= 1.01)){	//TOP LEFT
//		return 9;
//	}else{
//		return 0;
//	}

	if((bottom <= 12.00) && (right <= 12.00)  && (front > 12.00) && (left > 12.00)){ //BOTTOM RIGHT
		return 1;
	}else if ((front > 12.00) && (right <= 12.00)  && (bottom > 12.00) && (left > 12.00)) { //RIGHT
		return 2;
	}else if((front <= 12.00) && (right <= 12.00) && (right > 6) && (bottom > 12.00) && (left > 12.00)){	//TOP RIGHT
		return 3;
	}else if ((front > 12.00) && (right > 12.00) && (bottom > 12.00) && (left > 12.00)){ //MIDDLE
		return 4;
	}else if((front > 12.00) && (right > 12.00) && (bottom <= 12.00) && (left > 12.00)){	//BOTTOM
		return 5;
	}else if((front <= 12.00) && (right > 6.00) && (bottom > 6.00) && (left > 6.00)){ //TOP
		return 6;
	}else if((front >= 12.00) && (right >= 12.00) && (bottom <= 12.00) && (left <= 12.00)){ //BOTTOM LEFT
		return 7;
	}else if((front >= 12.00) && (right >= 12.00) && (bottom > 12.00) && (left <= 12.00)){ //LEFT
		return 8;
	}else if((front <= 12.00) && (right > 12.00) && (bottom > 12.00) && (left <= 12.00) && (left > 6.00)){	//TOP LEFT
		return 9;
	}else if((front <= 12.00) && (right <= 6.00) && (bottom > 12.00) && (left > 12.00)){	//CLOSE TOP RIGHT
		return 10;
	}else if((front <= 12.00) && (right > 12.00) && (bottom > 12.00) && (left <= 6.00)){	//CLOSE TOP LEFT
		return 11;
//	}else if((front <= 12.00) && (right > 12.00) && (bottom > 12.00) && (left <= 20.00) && (left > 12.00 )){	//MEDIUM TOP LEFT
//		return 12;
	}else{
		return 0;
	}
}

void moveForward(){
	  //motor A
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	  //motor B
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}

void stop(){
	  //motor A
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	  //motor B
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

}

void moveReverse(){
	  //motor A
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	  //motor B
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
}

void turnRightTrack(){
	  //motor A (Left Track)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	  //motor B (Right Track)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}

void turnLeftTrack(){
	  //motor A (Left Track)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	  //motor B (Right Track)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}

void turnRightTrackReverse(){
	  //motor A (Left Track)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	  //motor B (Right Track)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}

void turnLeftTrackReverse(){
	  //motor A (Left Track)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	  //motor B (Right Track)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
}

void UARTposition(int currentPosition){
//Transmit Position over UART
	if(currentPosition == 1){
		//Bottom Right
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 2) {
		//Right
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 3) {
		//Top Right
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 4) {
		//Middle
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 5) {
		//Bottom
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 6) {
		//Top
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 7) {
		//Bottom Left
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 8) {
		//Left
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 9) {
		//Top Left
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 10) {
		//Top Right
		sprintf((char*)uartBuf, "P3\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 11) {
		//Top Left
		sprintf((char*)uartBuf, "P7\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else {
		//Position Estimation Error
		sprintf((char*)uartBuf, "PEE %u\r\n", currentPosition);
		HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}

}

void UARTposition2(int currentPosition){
//Transmit Position over UART
	if(currentPosition == 1){
		//Bottom Right
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 2) {
		//Right
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 3) {
		//Top Right
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 4) {
		//Middle
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 5) {
		//Bottom
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 6) {
		//Top
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 7) {
		//Bottom Left
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 8) {
		//Left
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 9) {
		//Top Left
		sprintf((char*)uartBuf, "P%u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 10) {
		//Top Right
		sprintf((char*)uartBuf, "P3\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else if (currentPosition == 11) {
		//Top Left
		sprintf((char*)uartBuf, "P7\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}else {
		//Position Estimation Error
		sprintf((char*)uartBuf, "PEE %u\r\n", currentPosition);
		HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);
	}

}

void UARTLeft(int LeftValue){
	sprintf((char*)uartBuf, "LV: %u\r\n", LeftValue);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
}
void UARTRight(int RightValue){
	sprintf((char*)uartBuf, "RV: %u\r\n", RightValue);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
}
void UARTFront(int FrontValue){
	sprintf((char*)uartBuf, "FV: %u\r\n", FrontValue);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
}
void UARTBottom(int BottomValue){
	sprintf((char*)uartBuf, "BV: %u\r\n", BottomValue);
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
}
void UARTContaminationFound(){
	sprintf((char*)uartBuf, "Y\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
}
void UARTContaminationNOTFound(){
	sprintf((char*)uartBuf, "N\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)uartBuf, strlen(uartBuf), 100);
}
void RPMCheck(){
	if(is_capture_doneA){
			  			if(input_capturesA[1] > input_capturesA[0]){
			  				capture_differenceA = input_capturesA[1] - input_capturesA[0];
			  			}else{
			  				capture_differenceA = (1000000 - input_capturesA[0]) + input_capturesA[1];

			  			}

			  		analyses = capture_differenceA;
			  		timer2_cnt_freqA = 1;
			  		timer2_cnt_resA = 1/timer2_cnt_freqA;
			  		user_signal_time_periodA = capture_differenceA * timer2_cnt_resA;


			  		RPMA = 60000000/user_signal_time_periodA;


			  		RPMerrorA = (float)(RPMsetPointA - RPMA);
			  		IntegrationSumA += RPMerrorA;
			  		Ia = 0.5*KI*IntegrationSumA;

			  		DutyA = KpA*RPMerrorA + Ia;
			  		widthA = htim10.Instance->CCR1 + (int)DutyA;



			  		printf("RPM A = %.2f Error A = %.2f Duty A = %.2f width A = %u \r\n",RPMA, RPMerrorA, DutyA, widthA);

			  		is_capture_doneA = false;

			  		}

			  		//Motor B
			  		if(is_capture_doneB)
			  		{
			  			if(input_capturesB[1] > input_capturesB[0])
			  				capture_differenceB = input_capturesB[1] - input_capturesB[0];
			  			else
			  				capture_differenceB = (1000000 - input_capturesB[0]) + input_capturesB[1];

			  		timer2_cnt_freqB = 1;
			  		timer2_cnt_resB = 1/timer2_cnt_freqB;
			  		user_signal_time_periodB = capture_differenceB * timer2_cnt_resB;


			  		RPMB = 60000000/user_signal_time_periodB;

			  		RPMerrorB = (float)(RPMsetPointB - RPMB);

			  		IntegrationSumB += RPMerrorB;
			  		Ib = 0.5*KI*IntegrationSumB;

			  		DutyB = KpB*RPMerrorB +Ib;
			  		midB = htim13.Instance->CCR1*(DutyB/100);
			  		widthB = htim13.Instance->CCR1 + (int)DutyB;



			  		printf("RPM B = %.2f Error B = %.2f Duty B = %.2f width B = %u \r\n",RPMB, RPMerrorB, DutyB, widthB);

			  		is_capture_doneB = false;

			  		}
			  		//Motor A
			  		htim10.Instance->CCR1 = widthA;
			  		//Motor B
			  	  	htim13.Instance->CCR1 = widthB;
}

void filterBottom(){

	//Filtering
	  switch (readingCountBottom) {
		case 1:

			pos1 = ultraReadingsBottom[1];
			pos0 = ultraReadingsBottom[0];


			ultraReadingsBottom[2] = pos1;
			ultraReadingsBottom[1] = pos0;
			ultraReadingsBottom[0] = bottom;

			pos1 = ultraReadingsBottom[1];
			pos0 = ultraReadingsBottom[0];
			readingCountBottom++;
			break;
		case 2:

			pos1 = ultraReadingsBottom[1];
			pos0 = ultraReadingsBottom[0];

			ultraReadingsBottom[2] = pos1;
			ultraReadingsBottom[1] = pos0;
			ultraReadingsBottom[0] = bottom;

			pos1 = ultraReadingsBottom[1];
			pos0 = ultraReadingsBottom[0];

			if( ( (ultraReadingsBottom[1] - ultraReadingsBottom[0]) >= filterThresholdBottom )  ||  ( (ultraReadingsBottom[1] - ultraReadingsBottom[0]) >= -filterThresholdBottom )  ){
				bottom = ultraReadingsBottom[1];
				ultraReadingsBottom[0] = ultraReadingsBottom[1];
				readingCountBottom++;
			}else {
				bottom = ultraReadingsBottom[0];
				readingCountBottom++;
			}
			break;
		case 3:


			pos1 = ultraReadingsBottom[1];
			pos0 = ultraReadingsBottom[0];

			ultraReadingsBottom[2] = ultraReadingsBottom[1];
			ultraReadingsBottom[1] = ultraReadingsBottom[0];
			ultraReadingsBottom[0] = bottom;

			pos1 = ultraReadingsBottom[1];
			pos0 = ultraReadingsBottom[0];

			if( ( (ultraReadingsBottom[1] - ultraReadingsBottom[0]) >= filterThresholdBottom )  ||  ( (ultraReadingsBottom[1] - ultraReadingsBottom[0]) >= -filterThresholdBottom )  ){
				bottom = ultraReadingsBottom[1];
				ultraReadingsBottom[0] = ultraReadingsBottom[1];

			}else {
				bottom = ultraReadingsBottom[0];
			}
			readingCountBottom = 2;
			break;
		default:
			readingCountBottom++;
			break;
	}

}
void filterRight(){
	//Filtering
	  switch (readingCountRight) {
		case 1:

			posRight1 = ultraReadingsRight[1];
			posRight0 = ultraReadingsRight[0];


			ultraReadingsRight[2] = posRight1;
			ultraReadingsRight[1] = posRight0;
			ultraReadingsRight[0] = right;

			posRight1 = ultraReadingsRight[1];
			posRight0 = ultraReadingsRight[0];
			readingCountRight++;
			break;
		case 2:

			posRight1 = ultraReadingsRight[1];
			posRight0 = ultraReadingsRight[0];

			ultraReadingsRight[2] = posRight1;
			ultraReadingsRight[1] = posRight0;
			ultraReadingsRight[0] = right;

			posRight1 = ultraReadingsRight[1];
			posRight0 = ultraReadingsRight[0];

			if( ( (ultraReadingsRight[1] - ultraReadingsRight[0]) >= filterThresholdRight )  ||  ( (ultraReadingsRight[1] - ultraReadingsRight[0]) >= -filterThresholdRight)  ){
				right = ultraReadingsRight[1];
				ultraReadingsRight[0] = ultraReadingsRight[1];
				readingCountRight++;
			}else {
				right = ultraReadingsRight[0];
				readingCountRight++;
			}
			break;
		case 3:


			posRight1 = ultraReadingsRight[1];
			posRight0 = ultraReadingsRight[0];

			ultraReadingsRight[2] = ultraReadingsRight[1];
			ultraReadingsRight[1] = ultraReadingsRight[0];
			ultraReadingsRight[0] = right;

			posRight1 = ultraReadingsRight[1];
			posRight0 = ultraReadingsRight[0];

			if( ( (ultraReadingsRight[1] - ultraReadingsRight[0]) >= filterThresholdRight )  ||  ( (ultraReadingsRight[1] - ultraReadingsRight[0]) >= -filterThresholdRight )  ){
				right = ultraReadingsRight[1];
				ultraReadingsRight[0] = ultraReadingsRight[1];

			}else {
				right = ultraReadingsRight[0];
			}
			readingCountRight = 2;
			break;
		default:
			readingCountRight++;
			break;
	}
}


void filterLeft(){
	//Filtering
	  switch (readingCountLeft) {
		case 1:

			posLeft1 = ultraReadingsLeft[1];
			posLeft0 = ultraReadingsLeft[0];


			ultraReadingsLeft[2] = posLeft1;
			ultraReadingsLeft[1] = posLeft0;
			ultraReadingsLeft[0] = left;

			posLeft1 = ultraReadingsLeft[1];
			posLeft0 = ultraReadingsLeft[0];
			readingCountLeft++;
			break;
		case 2:

			posLeft1 = ultraReadingsLeft[1];
			posLeft0 = ultraReadingsLeft[0];

			ultraReadingsLeft[2] = posLeft1;
			ultraReadingsLeft[1] = posLeft0;
			ultraReadingsLeft[0] = left;

			posLeft1 = ultraReadingsLeft[1];
			posLeft0 = ultraReadingsLeft[0];

			if( ( (ultraReadingsLeft[1] - ultraReadingsLeft[0]) >= filterThresholdLeft )  ||  ( (ultraReadingsLeft[1] - ultraReadingsLeft[0]) >= -filterThresholdLeft)  ){
				left = ultraReadingsLeft[1];
				ultraReadingsLeft[0] = ultraReadingsLeft[1];
				readingCountLeft++;
			}else {
				left = ultraReadingsLeft[0];
				readingCountLeft++;
			}
			break;
		case 3:


			posLeft1 = ultraReadingsLeft[1];
			posLeft0 = ultraReadingsLeft[0];

			ultraReadingsLeft[2] = ultraReadingsLeft[1];
			ultraReadingsLeft[1] = ultraReadingsLeft[0];
			ultraReadingsLeft[0] = left;

			posLeft1 = ultraReadingsLeft[1];
			posLeft0 = ultraReadingsLeft[0];

			if( ( (ultraReadingsLeft[1] - ultraReadingsLeft[0]) >= filterThresholdLeft )  ||  ( (ultraReadingsLeft[1] - ultraReadingsLeft[0]) >= -filterThresholdLeft )  ){
				left = ultraReadingsLeft[1];
				ultraReadingsLeft[0] = ultraReadingsLeft[1];

			}else {
				left = ultraReadingsLeft[0];
			}
			readingCountLeft = 2;
			break;
		default:
			readingCountLeft++;
			break;
	}

}


void readContamination(){
	  //ADC CHANNEL 1
		sConfig.Channel = ADC_CHANNEL_1;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	  HAL_ADC_Start(&hadc1);
	  if ((HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)) {
		  sensor1 = HAL_ADC_GetValue(&hadc1);
	}
//		sprintf((char*)uartBuf, "%u\r\n", adcVal0);
		//HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);


		//ADC CHANNEL 4
		sConfig.Channel = ADC_CHANNEL_4;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	  HAL_ADC_Start(&hadc1);
	  if ((HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)) {
		  sensor2 = HAL_ADC_GetValue(&hadc1);
		}


	  //ADC CHANNEL 8
		sConfig.Channel = ADC_CHANNEL_8;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	  HAL_ADC_Start(&hadc1);
	  if ((HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)) {
		  sensor3 = HAL_ADC_GetValue(&hadc1);
		}


	  //ADC CHANNEL 11
		sConfig.Channel = ADC_CHANNEL_11;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	  HAL_ADC_Start(&hadc1);
	  if ((HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)) {
		  sensor4 = HAL_ADC_GetValue(&hadc1);
		}


	  //ADC CHANNEL 10
		sConfig.Channel = ADC_CHANNEL_10;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	  HAL_ADC_Start(&hadc1);
	  if ((HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)) {
		  sensor5 = HAL_ADC_GetValue(&hadc1);
		}
	  printf("Sensor 1 = %u Sensor 2 = %u  Sensor 3 = %u Sensor 4 = %u  Sensor 5 = %u \r\n", sensor1, sensor2, sensor3, sensor4, sensor5);
}

void readAccelerometer(){

				  MPU6050_Get_Accel_Scale(&myAcc_ScaledData);
				  MPU6050_Get_Gyro_Scale(&myGyro_ScaledData);
				  HAL_Delay(5);
				  ACCxValue = myAcc_ScaledData.x;
				  ACCyValue = myAcc_ScaledData.y;
				  ACCzValue = myAcc_ScaledData.z;


//				  ACCxValue -= ACCxValue_Cal;
//				  ACCyValue -= ACCyValue_Cal;
//				  ACCzValue -= ACCzValue_Cal;

				  printf("X: %.2f  Y: %.2f  Z: %.2f \r\n", ACCxValue, ACCyValue, ACCzValue);
}
void cleaningAction(){
	stop();
//	HAL_Delay(1000);
//	moveForward();
//	HAL_Delay(22000);
//	moveReverse();
//	HAL_Delay(3000);
//	moveForward();
//	HAL_Delay(3000);
//	moveReverse();
//	HAL_Delay(3000);
//	moveForward();
//	HAL_Delay(3000);
//	moveReverse();
//	HAL_Delay(3000);
//
//	turnRightTrack();
//	HAL_Delay(4000);
//	turnLeftTrackReverse();
//
//
//	//RE-ADJUST ALLIGNMENT
//	readAccelerometer();
//	if(ACCxValue > accThreshold_x){
//		while(ACCxValue > accThreshold_x){
//			readAccelerometer();
//			HAL_Delay(10);
//			turnLeftTrackReverse();
//		}
//	}else if(ACCxValue < accThreshold_x){
//		while(ACCxValue < accThreshold_x){
//			readAccelerometer();
//			HAL_Delay(10);
//			turnRightTrackReverse();
//		}
//	}
//
//	moveForward();
//	HAL_Delay(3000);
//	moveReverse();
//	HAL_Delay(3000);
//
//	turnLeftTrack();
//	HAL_Delay(4000);
//
//	//RE-ADJUST ALLIGNMENT
//	readAccelerometer();
//	if(ACCxValue > accThreshold_x){
//		while(ACCxValue > accThreshold_x){
//			readAccelerometer();
//			HAL_Delay(10);
//			turnLeftTrackReverse();
//		}
//	}else if(ACCxValue < accThreshold_x){
//		while(ACCxValue < accThreshold_x){
//			readAccelerometer();
//			HAL_Delay(10);
//			turnRightTrackReverse();
//		}
//	}
//	moveReverse();
//	HAL_Delay(10000);
//	moveForward();
//	HAL_Delay(10000);
//	stop();
//	HAL_Delay(500);
//	//RE-ADJUST ALLIGNMENT
//	readAccelerometer();
//	if(ACCxValue > accThreshold_x){
//		while(ACCxValue > accThreshold_x){
//			readAccelerometer();
//			HAL_Delay(10);
//			turnLeftTrackReverse();
//		}
//	}else if(ACCxValue < accThreshold_x){
//		while(ACCxValue < accThreshold_x){
//			readAccelerometer();
//			HAL_Delay(10);
//			turnRightTrackReverse();
//		}
//	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
