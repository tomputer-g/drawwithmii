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
#include "hx8357d.h"
#include "n64.h"
#include "plotter.h"
#include "stdio.h"
#include "PixyI2C.h"
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
 I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
//---------------------------Driver----------------------------------
enum InputMode {IR, N64};
enum InputMode mode = N64; // TODO: write N64 Button Interrupt to swap modes
uint16_t drawColor;
//---------------------------Screen----------------------------------
//Chip Select pin
GPIO_TypeDef  *tftCS_GPIO = GPIOD;
uint16_t tftCS_PIN = GPIO_PIN_14;
//Data Command pin
GPIO_TypeDef  *tftDC_GPIO = GPIOD;
uint16_t tftDC_PIN = GPIO_PIN_15;
//Reset pin
GPIO_TypeDef  *tftRESET_GPIO = GPIOF;
uint16_t tftRESET_PIN = GPIO_PIN_12;
static int displayXStep = HX8357_TFTWIDTH/2; // 0,0 is starting corner
static int displayYStep = HX8357_TFTHEIGHT/2;
static int displayXGoalStep = 100;
static int displayYGoalStep = 100;

//----------------------------N64-------------------------------------
GPIO_TypeDef  *n64_GPIO = GPIOC;
uint16_t n64_PIN = GPIO_PIN_6;
//Debug pin
GPIO_TypeDef  *n64_DEBUG_GPIO = GPIOB;
uint16_t n64_DEBUG_PIN = GPIO_PIN_13;
//Interrupt pin (connected to data)
GPIO_TypeDef  *n64_INT_GPIO = GPIOB;
uint16_t n64_INT_PIN = GPIO_PIN_15;

//--------------------------XY Plotter--------------------------------
// Pin definitions
static GPIO_TypeDef *A0_GPIO = GPIOB;
static uint16_t A0_PIN = GPIO_PIN_10;
static GPIO_TypeDef *A1_GPIO = GPIOE;
static uint16_t A1_PIN = GPIO_PIN_15;
static GPIO_TypeDef *A2_GPIO = GPIOE;
static uint16_t A2_PIN = GPIO_PIN_14;
static GPIO_TypeDef *A3_GPIO = GPIOE;
static uint16_t A3_PIN = GPIO_PIN_12;

static GPIO_TypeDef *B0_GPIO = GPIOG;
static uint16_t B0_PIN = GPIO_PIN_1;
static GPIO_TypeDef *B1_GPIO = GPIOF;
static uint16_t B1_PIN = GPIO_PIN_9;
static GPIO_TypeDef *B2_GPIO = GPIOF;
static uint16_t B2_PIN = GPIO_PIN_7;
static GPIO_TypeDef *B3_GPIO = GPIOF;
static uint16_t B3_PIN = GPIO_PIN_8;

static int plotXStep = 0; // 0,0 is starting corner
static int plotYStep = 0;

static int plotXGoalStep = 0;
static int plotYGoalStep = 0;

static const uint8_t STEP_PER_CM = 50;
static const uint8_t TRAVEL_X_CM = 40;
static const uint8_t TRAVEL_Y_CM = 40;

//---------------------------Servo---------------------------------
#define TIM16_ADDR 0x40014400 //timer 4 base register
#define TIM_CCR1_OFFSET 0x34 //capture/compare register 2
#define SERVO_DOWN 150
#define SERVO_UP 125
static char servoState = 0;
static char ZPrevState = 0; //button push state for N64 Z
static uint32_t * tim16_ccr1 = (uint32_t *)(TIM16_ADDR + TIM_CCR1_OFFSET);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void movePlot(int dXStep, int dYStep){
	plotXStep += dXStep;
	plotYStep += dYStep;
	if(dXStep == 0){
		stepDiag(0, dYStep);
	}else if(dYStep == 0){
		stepDiag(dXStep, 0);
	}else{
		stepDiag(dXStep, dYStep);
	}
	printf("movePlot: now at (%d, %d)\n\r", plotXStep, plotYStep);
}

void moveTo(int xPos, int yPos)
{
  int dXStep = xPos - plotXStep;
  int dYStep = yPos - plotYStep;
  movePlot(dXStep, dYStep);
}

//this goes from -128 to +127 on each axis (N64 input)
void scaleN64Plot(signed char xval, signed char yval, int threshold){
	// plotXGoalStep = (xval + 128) * (STEP_PER_CM * TRAVEL_X_CM) / 255;
	// plotYGoalStep = (yval + 128) * (STEP_PER_CM * TRAVEL_Y_CM) / 255;
	// printf("N64Plot: going to (%d, %d)\n\r", plotXGoalStep, plotYGoalStep);
	int stepDistance = 50;
    plotXGoalStep = 0;
    plotYGoalStep = 0;
    if(xval > threshold && plotXStep < (STEP_PER_CM * TRAVEL_X_CM) - stepDistance)
      plotXGoalStep =  stepDistance;
    if(xval < -1 * threshold && plotXStep > stepDistance)
      plotXGoalStep =  -1 * stepDistance;
    if(yval > threshold && plotYStep < (STEP_PER_CM * TRAVEL_Y_CM) - stepDistance)
      plotYGoalStep =  stepDistance;
    if(yval < -1 * threshold && plotYStep > stepDistance)
      plotYGoalStep =  -1 * stepDistance;
    printf("N64Plot: going to (%d, %d)\n\r", plotXStep+ plotXGoalStep, plotYStep + plotYGoalStep);
    movePlot(plotXGoalStep, plotYGoalStep);
}
void scaleN64Display(signed char xval, signed char yval, int threshold)
{
  // Which direction is the stick in?
    displayXGoalStep = displayXStep;
    displayYGoalStep = displayYStep;
    int stepDistance = 6;
//    if(xval > threshold && displayXStep > 20)
//      displayXGoalStep =  displayXStep - 4;
//    if(xval < -1 * threshold && displayXStep < HX8357_TFTWIDTH - 20)
//      displayXGoalStep =  displayXStep + 4;
//    if(yval > threshold && displayYStep < HX8357_TFTHEIGHT - 20)
//      displayYGoalStep =  displayYStep + 4;
//    if(yval < -1 * threshold && displayYStep > 20)
//      displayYGoalStep =  displayYStep - 4;

    // Up on stick -> right on display
    if(yval > threshold && displayXStep < HX8357_TFTWIDTH - 20)
    {
        displayXGoalStep =  displayXStep + stepDistance;
    }
       // Down on Stick -> Left on Display
    if(yval < -1 * threshold && displayXStep > 20)
        displayXGoalStep =  displayXStep - stepDistance;
    // Left on stick -> down on display
    if(xval < -1 * threshold && displayYStep > 20)
        displayYGoalStep =  displayYStep - stepDistance;
    // Right on stick -> up on display
    if(xval > threshold &&displayYStep < HX8357_TFTHEIGHT - 20)
        displayYGoalStep =  displayYStep + stepDistance;

//    uint16_t XCenter = displayXGoalStep;
//	  uint16_t YCenter = displayYGoalStep;
    uint16_t XCenter = (plotYStep + plotYGoalStep) * (HX8357_TFTWIDTH) / (STEP_PER_CM * TRAVEL_Y_CM);
	uint16_t YCenter = (plotXStep + plotXGoalStep) * (HX8357_TFTHEIGHT) / (STEP_PER_CM * TRAVEL_X_CM);
	  uint16_t rectRadius = 12;
	  if(XCenter + rectRadius >= HX8357_TFTWIDTH) XCenter -= rectRadius;
	  if(YCenter + rectRadius >= HX8357_TFTHEIGHT) YCenter -= rectRadius;
	  LCD_rect(XCenter, YCenter, XCenter + rectRadius, YCenter + rectRadius, drawColor);
    displayXStep = displayXGoalStep;
    displayYStep = displayYGoalStep;
}
void scaleIRPlot(int *data, int threshold)
{
	//Tracking style
	if(data[0] != 0 && data[1] != 0)
	{
		int newX = data[0] * (STEP_PER_CM * TRAVEL_X_CM) / 320;
		newX = (STEP_PER_CM * TRAVEL_X_CM) - newX;
		int newY = data[1] * (STEP_PER_CM * TRAVEL_Y_CM) / 200;
		newY = (STEP_PER_CM * TRAVEL_Y_CM) - newY;
		int newXDiff = newX - plotXStep;
		int newYDiff = newY - plotYStep;
		int total = newXDiff + newYDiff;
		int stepDistance = 100;
		plotXGoalStep = newXDiff * stepDistance / total;
		plotYGoalStep = newYDiff * stepDistance / total;
		printf("IRPlot: going to (%d, %d)\n\r", plotXStep+ plotXGoalStep, plotYStep + plotYGoalStep);
		movePlot(plotXGoalStep, plotYGoalStep);
	}

    // N64 Style
//
//	int xval = -1* (data[0] * 256 / 320) - 128;
//	int yval = -1* ((data[0] * 256 / 200) - 128);
//	scaleN64Plot(xval, yval, threshold);
}
void scaleIRDisplay(int *data, int threshold)
{
//  uint16_t XCenter = data[0] * HX8357_TFTWIDTH / 320;
//	uint16_t YCenter = data[1] * HX8357_TFTHEIGHT / 200;
//	uint16_t rectRadius = 8;
//	LCD_rect(XCenter - rectRadius, YCenter - rectRadius, XCenter + rectRadius, YCenter + rectRadius, drawColor);

	//Tracking style
	if(data[0] != 0 && data[1] != 0)
	{
		int newX = data[0] * (STEP_PER_CM * TRAVEL_X_CM) / 320;
		newX = (STEP_PER_CM * TRAVEL_X_CM) - newX;
		int newY = data[1] * (STEP_PER_CM * TRAVEL_Y_CM) / 200;
		newY = (STEP_PER_CM * TRAVEL_Y_CM) - newY;
		int newXDiff = newX - plotXStep;
		int newYDiff = newY - plotYStep;
		int total = newXDiff + newYDiff;
		int stepDistance = 100;
		plotXGoalStep = newXDiff * stepDistance / total;
		plotYGoalStep = newYDiff * stepDistance / total;

		uint16_t XCenter = (plotXStep + plotXGoalStep) * (HX8357_TFTHEIGHT) / (STEP_PER_CM * TRAVEL_X_CM);
		uint16_t YCenter = (plotYStep + plotYGoalStep) * (HX8357_TFTWIDTH) / (STEP_PER_CM * TRAVEL_Y_CM);
	  uint16_t rectRadius = 12;
	  if(XCenter + rectRadius >= HX8357_TFTWIDTH) XCenter -= rectRadius;
	  if(YCenter + rectRadius >= HX8357_TFTHEIGHT) YCenter -= rectRadius;
	  LCD_rect(XCenter, YCenter, XCenter + rectRadius, YCenter + rectRadius, drawColor);
	}

    // N64 Style
//	int xval = -1* (data[0] * 256 / 320) - 128;
//	int yval = -1*((data[0] * 256 / 200) - 128);
//	scaleN64Display(xval, yval, threshold);
}

void drawPlotBounds(){
	//assumes we are at (0,0);
	movePlot((int)(STEP_PER_CM * TRAVEL_X_CM), 0);
	movePlot(0, (int)(STEP_PER_CM * TRAVEL_Y_CM));
	movePlot((int)(-STEP_PER_CM * TRAVEL_X_CM), 0);
	movePlot(0, (int)(-STEP_PER_CM * TRAVEL_Y_CM));
}

void reCalibrate(int vals)
{
	if((vals >> 28) & 0x1)
	{
		plotXStep = 0;
		plotYStep = 0;
	}
}

// Three button functions - Reset, Color Change, Input switch
void resetAll(){
	*tim16_ccr1 = SERVO_UP;
	servoState = 0;
	mode = N64;
	drawColor = 0x0000;
	LCD_fill(HX8357_WHITE);
	displayXStep = HX8357_TFTWIDTH/2;
	displayYStep = HX8357_TFTHEIGHT/2;
	moveTo(100, plotYStep);
	moveTo(100, 100);
}

void colorChange()
{
	switch(drawColor){
	case 0x0000:
		drawColor = 0x001F;
		break;
	case 0x001F:
		drawColor = 0xF800;
		break;
	case 0xF800:
		drawColor = 0x07E0;
		break;
	case 0x07E0:
		drawColor = 0x07FF;
		break;
	case 0x7FF:
		drawColor = 0xF81F;
		break;
	case 0xF81F:
		drawColor = 0xFFE0;
		break;
	case 0xFFE0:
		drawColor = 0x0000;
		break;
	}
//#define	HX8357_BLACK   0x0000 ///< BLACK color for drawing graphics
//#define	HX8357_BLUE    0x001F ///< BLUE color for drawing graphics
//#define	HX8357_RED     0xF800 ///< RED color for drawing graphics
//#define	HX8357_GREEN   0x07E0 ///< GREEN color for drawing graphics
//#define HX8357_CYAN    0x07FF ///< CYAN color for drawing graphics
//#define HX8357_MAGENTA 0xF81F ///< MAGENTA color for drawing graphics
//#define HX8357_YELLOW  0xFFE0 ///< YELLOW color for drawing graphics
//#define HX8357_WHITE   0xFFFF ///< WHITE color for drawing graphics
}
void modeSwap()
{
	mode = !mode;
}
void settingCheck(uint32_t vals)
{
    int aVal = vals >> 31;
    if(aVal) modeSwap();
//    int bVal = (vals >> 30) & 0x1;
//    if(bVal) resetAll();
    int lBumper = (vals >> 21) & 0x1;
    if(lBumper) colorChange();
}

//Servo
void toggleServo(){
	if(servoState){
		*tim16_ccr1 = SERVO_UP;
		servoState = 0;
	}else{
		*tim16_ccr1 = SERVO_DOWN;
		servoState = 1;
	}
}

void handleServo(char ZState){
	if(ZState && !ZPrevState){
		toggleServo();
	}
	ZPrevState = ZState;
}

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
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  printf("Initing...\n\r");

  //Servo
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  *tim16_ccr1 = SERVO_UP;

  //Stepper
  step_init(&htim2, 1, A0_GPIO, A0_PIN, A1_GPIO, A1_PIN, A2_GPIO, A2_PIN, A3_GPIO, A3_PIN, B0_GPIO, B0_PIN, B1_GPIO, B1_PIN, B2_GPIO, B2_PIN, B3_GPIO, B3_PIN);
  setSpeed(300); //yes 300, not above incl. 325

  //N64
  N64_init(&htim4, &htim5, n64_GPIO, n64_PIN, n64_DEBUG_GPIO, n64_DEBUG_PIN, n64_INT_GPIO, n64_INT_PIN);

  //LCD
  LCD_init(&hspi1, tftCS_GPIO, tftCS_PIN, tftDC_GPIO, tftDC_PIN, tftRESET_GPIO, tftRESET_PIN);
  HAL_Delay(200);
  LCD_fill(HX8357_WHITE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // start here

//  drawPlotBounds();
  movePlot((TRAVEL_X_CM * STEP_PER_CM / 2),(TRAVEL_Y_CM * STEP_PER_CM / 2));
  uint32_t vals = 0;
  printf("Starting...\n\r");
  while (1)
  {
    if(mode == N64)
    {
      // N64 Read
      vals = intRead(); // intRead();
      //reCalibrate(vals);
      settingCheck(vals);
      signed char xval = (vals >> 8) & 0xff; //both were signed
      signed char yval = vals & 0xff;
      printf("N64 read X: %d,Y: %d\n\r", xval, yval);
      // Input Tracking
      // To Display
      scaleN64Display(xval, yval, 45);
      // To Plotter Global
      scaleN64Plot(xval, yval, 45);
    }
    else if(mode == IR)
    {
    	// Check for mode switch
        vals = intRead(); // intRead();
        settingCheck(vals);
      // IR Read
      int ir_buf[2];
      getBlocks(&hi2c1, &ir_buf[0]);
      printf("IR read X: %d,Y: %d\n\r", ir_buf[0], ir_buf[1]);
      // Input Tracking
      // To Display
      scaleIRDisplay(&ir_buf[0], 10);
      // To Plotter Globals
      scaleIRPlot(&ir_buf[0], 10);
    }
    // Plot

	  handleServo((vals >> 29) & 1);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 59999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 11;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
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

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 119;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  htim16.Init.Prescaler = 1199;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE12 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
