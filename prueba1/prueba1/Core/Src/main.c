/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (NUCLEO-F446RE + L3G4200D)
  ******************************************************************************
  * @attention
  *  Gyroscope Payload for CURTIS
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Direcciones/Registros L3G4200D */
#define L3G4200D_ADDR      (0x69 << 1)  // 8-bit address (0xD2/0xD3)
#define L3G4200D_WHO_AM_I  0x0F
#define L3G4200D_CTRL_REG1 0x20
#define L3G4200D_CTRL_REG4  0x23
#define L3G4200D_OUT_X_L   0x28
#define L3G4200D_STATUS_REG  0x27
#define ZYXDA                0x08  // New data available on all axes
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int16_t gyro_x, gyro_y, gyro_z;
char msg[64];
bool gyro_ok = false;

/* Conversion and bias */
static int16_t bias_x=0, bias_y=0, bias_z=0;

/* ---- Estado para STS ---- */
volatile uint8_t flag_capture_active   = 0;  // B0: logging de giroscopio
volatile uint8_t flag_flash_saving     = 0;  // B1: guardando en MSFM
volatile uint8_t flag_data_transfer    = 0;  // B2: transferencia (no usada aquí)

#define huart_cam huart3

#define FLASH_CS_PORT     GPIOB
#define FLASH_CS_PIN      GPIO_PIN_12   // Adjust

#define FLASH_SECTOR_SIZE   (64UL*1024UL)
#define FLASH_PAGE_SIZE     (4UL*1024UL)
#define FLASH_SLOT_SIZE     (4UL*1024*1024UL)  // mem_block_size del PIC
#define GYRO_SLOT_INDEX     0
#define GYRO_BASE_ADDR      (GYRO_SLOT_INDEX * FLASH_SLOT_SIZE)

// Opcodes according to Main PIC's code
#define FLASH_CMD_RDID        0x9F
#define FLASH_CMD_RDSR        0x05
#define FLASH_CMD_WREN        0x06
#define FLASH_CMD_PP          0x12   // Page program
#define FLASH_CMD_READ        0x13   // Read 4-byte addressing
#define FLASH_CMD_ERASE_SECT  0xDC   // 64 KB erase

// Macros for CS
#define FLASH_CS_LOW()   HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET)
#define FLASH_CS_HIGH()  HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Prototipos de utilidades y sensor */
void UART_Print(char *msg);
uint8_t I2C_ReadByte(uint8_t reg);
void I2C_WriteByte(uint8_t reg, uint8_t value);
void L3G4200D_Init(void);
bool L3G4200D_ReadGyro(int16_t *x, int16_t *y, int16_t *z);
static void Gyro_Calibrate(uint16_t n);
void log_current_gyro(uint32_t t_ms, int16_t gx, int16_t gy, int16_t gz);
uint8_t cam_checksum(const char *s);
void cam_build_response(const char *inner, uint8_t *out);
void handle_cam_command(const char *inner_cmd);
void CheckOBC_Task(void);

/* Prototypes for FM*/
void FLASH_WriteEnable(void);
void FLASH_WaitBusy(void);
void FLASH_SectorErase(uint32_t addr);
void FLASH_PageProgram(uint32_t addr, const uint8_t *data, uint32_t len);
void FLASH_Read(uint32_t addr, uint8_t *data, uint32_t len);
void FLASH_WriteLogToMissionFlash(void);
static void build_STS_inner(char *inner, size_t inner_size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct __attribute__((packed)) {
    uint32_t t_ms;  // timestamp ms
    int16_t  gx;
    int16_t  gy;
    int16_t  gz;
} GyroSample;

#define MAX_SAMPLES 6000
GyroSample gyro_log[MAX_SAMPLES];
uint32_t   gyro_count = 0;

// MSN (UART)
#define CAM_MSG_LEN 64
#define OBC_HEADER  0x0B
#define CAM_HEADER  0xCA
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_Print("System Initialized\r\n");
  L3G4200D_Init();
  UART_Print("Gyro Initialized\r\n");
  UART_Print("Keep still for bias calib...\r\n");
  Gyro_Calibrate(200);
  UART_Print("Bias calibrated.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
  while (1)
  {
      static uint32_t t0_ms = 0;

      uint32_t now_ms = HAL_GetTick();
      if (t0_ms == 0) {
          t0_ms = now_ms;   // time reference for samples
      }

      /* 1️ logging to RAM */
      if (gyro_ok && gyro_count < MAX_SAMPLES) {
          flag_capture_active = 1;   // capturing data
          if (L3G4200D_ReadGyro(&gyro_x, &gyro_y, &gyro_z)) {
              int16_t gx_corr = gyro_x - bias_x;
              int16_t gy_corr = gyro_y - bias_y;
              int16_t gz_corr = gyro_z - bias_z;

              log_current_gyro(now_ms - t0_ms, gx_corr, gy_corr, gz_corr);
          }
      } else {
          flag_capture_active = 0;   // full buffer or sensor is not OK
      }

      /* 2️ check for OBC commands */
      CheckOBC_Task();

      HAL_Delay(100);
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_PIN_GPIO_Port, FLASH_CS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = FLASH_CS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ======== Utilidades UART/I2C y manejo de giroscopio ======== */
void UART_Print(char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

uint8_t I2C_ReadByte(uint8_t reg)
{
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  return data;
}

void I2C_WriteByte(uint8_t reg, uint8_t value)
{
  HAL_I2C_Mem_Write(&hi2c1, L3G4200D_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

void L3G4200D_Init(void)
{
    uint8_t who = I2C_ReadByte(L3G4200D_WHO_AM_I);
    if (who != 0xD3) {
        UART_Print("L3G4200D not found! WHO_AM_I failed.\r\n");
        gyro_ok = false;
        return;
    } else {
        UART_Print("L3G4200D detected successfully.\r\n");
        gyro_ok = true;
    }

    /* CTRL_REG1: 0x3F -> ODR=200Hz, BW=50Hz, PD=1, X/Y/Z enable */
    I2C_WriteByte(L3G4200D_CTRL_REG1, 0x3F);

    /* CTRL_REG4: BDU=1 (bit7), FS=±250 dps (00) -> 0x80 */
    I2C_WriteByte(L3G4200D_CTRL_REG4, 0x80);
}

static void I2C_RecoverIfNeeded(void)
{
  // Si el bus quedó en estado BUSY/AF/ARLO/BERR, reinicia el periférico I2C1
  if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {
    // Deshabilita/rehabilita periférico (soft reset de I2C)
    __HAL_I2C_DISABLE(&hi2c1);
    HAL_Delay(1);
    __HAL_I2C_ENABLE(&hi2c1);
  }
}

static void Gyro_Calibrate(uint16_t n)
{
  int32_t sx=0, sy=0, sz=0;
  int16_t x,y,z;
  for (uint16_t i=0; i<n; i++) {
    while (!L3G4200D_ReadGyro(&x,&y,&z)) { HAL_Delay(1); }
    sx += x; sy += y; sz += z;
    HAL_Delay(2);
  }
  bias_x = (int16_t)(sx / (int32_t)n);
  bias_y = (int16_t)(sy / (int32_t)n);
  bias_z = (int16_t)(sz / (int32_t)n);
}

bool L3G4200D_ReadGyro(int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t status = I2C_ReadByte(L3G4200D_STATUS_REG);
  if ((status & ZYXDA) == 0) {
    return false;
  }

  uint8_t buf[6];
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR,
                                          (L3G4200D_OUT_X_L | 0x80),
                                          I2C_MEMADD_SIZE_8BIT, buf, 6, 10);
  if (st != HAL_OK) {
    I2C_RecoverIfNeeded();
    st = HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR,
                          (L3G4200D_OUT_X_L | 0x80),
                          I2C_MEMADD_SIZE_8BIT, buf, 6, 10);
    if (st != HAL_OK) {
      HAL_I2C_DeInit(&hi2c1);
      HAL_I2C_Init(&hi2c1);
      return false;
    }
  }

  *x = (int16_t)((buf[1] << 8) | buf[0]);
  *y = (int16_t)((buf[3] << 8) | buf[2]);
  *z = (int16_t)((buf[5] << 8) | buf[4]);
  return true;
}

void log_current_gyro(uint32_t t_ms, int16_t gx, int16_t gy, int16_t gz)
{
    if (gyro_count >= MAX_SAMPLES) return;
    gyro_log[gyro_count].t_ms = t_ms;
    gyro_log[gyro_count].gx   = gx;
    gyro_log[gyro_count].gy   = gy;
    gyro_log[gyro_count].gz   = gz;
    gyro_count++;
}

uint8_t cam_checksum(const char *s)
{
    uint8_t c = 0;
    while (*s) { c ^= (uint8_t)(*s++); }
    return c;
}

void cam_build_response(const char *inner, uint8_t *out)
{
    memset(out, 0, CAM_MSG_LEN);
    size_t len = strlen(inner);
    uint8_t cs = cam_checksum(inner);

    out[0] = CAM_HEADER;             // 0xCA
    memcpy(&out[1], inner, len);     // "STS00000", etc.
    sprintf((char*)&out[1+len], "%02X", cs); // checksum ASCII HEX
    out[1 + len + 2] = CAM_HEADER + 1;       // 0xCB
}

void handle_cam_command(const char *inner_cmd)
{
    uint8_t tx_buf[CAM_MSG_LEN];

    snprintf(msg, sizeof(msg), "CMD from PIC: %s\r\n", inner_cmd);
    UART_Print(msg);

    if (strncmp(inner_cmd, "STS", 3) == 0) {

        char inner[16];
        build_STS_inner(inner, sizeof(inner));
        cam_build_response(inner, tx_buf);

    } else if (strncmp(inner_cmd, "TIM", 3) == 0) {

        cam_build_response("TIM00", tx_buf);

    } else if (strncmp(inner_cmd, "CAP", 3) == 0) {

        // Guardar log actual a Mission Flash
        FLASH_WriteLogToMissionFlash();
        cam_build_response("CAP00", tx_buf);

    } else if (strncmp(inner_cmd, "JPG", 3) == 0) {

        cam_build_response("JPG00", tx_buf);

    } else if (strncmp(inner_cmd, "CMC", 3) == 0) {

        cam_build_response("CMC00", tx_buf);

    } else if (strncmp(inner_cmd, "PDN", 3) == 0) {
        cam_build_response("PDN00", tx_buf);

    } else if (strncmp(inner_cmd, "RST", 3) == 0) {
        cam_build_response("RST00", tx_buf);

    } else if (strncmp(inner_cmd, "OWT", 3) == 0) {
        cam_build_response("OWT00", tx_buf);

    } else if (strncmp(inner_cmd, "CAN", 3) == 0) {
        cam_build_response("CAN00", tx_buf);

    } else if (strncmp(inner_cmd, "NUM", 3) == 0) {
        cam_build_response("NUM00000", tx_buf);

    } else if (strncmp(inner_cmd, "IMG", 3) == 0) {
        cam_build_response("IMG000000000000", tx_buf);

    } else if (strncmp(inner_cmd, "CMW", 3) == 0) {
        cam_build_response("CMW00", tx_buf);

    } else if (strncmp(inner_cmd, "CMR", 3) == 0) {
        cam_build_response("CMR0000", tx_buf);
    } else {

        cam_build_response("RST01", tx_buf);
    }

    HAL_UART_Transmit(&huart_cam, tx_buf, CAM_MSG_LEN, 100);
}


void CheckOBC_Task(void)
{
    uint8_t rx_buf[CAM_MSG_LEN];
    char    inner[CAM_MSG_LEN];

    if (HAL_UART_Receive(&huart_cam, rx_buf, CAM_MSG_LEN, 5) != HAL_OK)
        return;

    if (rx_buf[0] != OBC_HEADER) return;

    int footer = -1;
    for (int i = 1; i < CAM_MSG_LEN; i++) {
        if (rx_buf[i] == (OBC_HEADER + 1)) { // 0x0C
            footer = i;
            break;
        }
    }
    if (footer < 0) return;

    int inner_len = footer - 1 - 2; // -2 because of checksum
    if (inner_len <= 0 || inner_len >= CAM_MSG_LEN) return;

    memcpy(inner, &rx_buf[1], inner_len);
    inner[inner_len] = '\0';

    handle_cam_command(inner);
}

void FLASH_WriteEnable(void)
{
    uint8_t cmd = FLASH_CMD_WREN;
    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    FLASH_CS_HIGH();
}

void FLASH_WaitBusy(void)
{
    uint8_t cmd = FLASH_CMD_RDSR;
    uint8_t status = 0x01;

    do {
        FLASH_CS_LOW();
        HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
        HAL_SPI_Receive(&hspi2, &status, 1, 100);
        FLASH_CS_HIGH();
    } while (status & 0x01);
}

void FLASH_SectorErase(uint32_t addr)
{
    uint8_t cmd[5];
    cmd[0] = FLASH_CMD_ERASE_SECT;
    cmd[1] = (addr >> 24) & 0xFF;
    cmd[2] = (addr >> 16) & 0xFF;
    cmd[3] = (addr >> 8) & 0xFF;
    cmd[4] = (addr >> 0) & 0xFF;

    FLASH_WriteEnable();
    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 5, 200);
    FLASH_CS_HIGH();
    FLASH_WaitBusy();
}

void FLASH_PageProgram(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint8_t cmd[5];
    cmd[0] = FLASH_CMD_PP;
    cmd[1] = (addr >> 24) & 0xFF;
    cmd[2] = (addr >> 16) & 0xFF;
    cmd[3] = (addr >> 8) & 0xFF;
    cmd[4] = (addr >> 0) & 0xFF;

    FLASH_WriteEnable();
    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 5, 200);
    HAL_SPI_Transmit(&hspi2, (uint8_t*)data, len, 200);
    FLASH_CS_HIGH();
    FLASH_WaitBusy();
}

void FLASH_Read(uint32_t addr, uint8_t *data, uint32_t len)
{
    uint8_t cmd[5];
    cmd[0] = FLASH_CMD_READ;
    cmd[1] = (addr >> 24) & 0xFF;
    cmd[2] = (addr >> 16) & 0xFF;
    cmd[3] = (addr >> 8) & 0xFF;
    cmd[4] = (addr >> 0) & 0xFF;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 5, 200);
    HAL_SPI_Receive(&hspi2, data, len, 200);
    FLASH_CS_HIGH();
}

void FLASH_WriteLogToMissionFlash(void)
{
    if (gyro_count == 0) return;

    uint32_t file_size = gyro_count * sizeof(GyroSample);
    if (file_size > FLASH_SLOT_SIZE - 0x10) return; // leave space for 16B header

    uint32_t addr = GYRO_BASE_ADDR;

    flag_flash_saving = 1;  // for STS

    // 1) erase slot of 4 MiB
    for (uint32_t a = addr; a < (addr + FLASH_SLOT_SIZE); a += FLASH_SECTOR_SIZE)
        FLASH_SectorErase(a);

    // 2) header with 16 bytes (4 of size + 12 reserved)
    uint8_t header[16] = {0};

    header[0] = (file_size & 0xFF);
    header[1] = (file_size >> 8) & 0xFF;
    header[2] = (file_size >> 16) & 0xFF;
    header[3] = (file_size >> 24) & 0xFF;

    FLASH_PageProgram(addr, header, sizeof(header));
    addr += sizeof(header);  // offset for gyroscope's data

    // 3) write gyroscope's logs starting from 0x10
    const uint8_t *p = (uint8_t*)gyro_log;
    uint32_t remaining = file_size;

    while (remaining > 0)
    {
        uint32_t page_remain = FLASH_PAGE_SIZE - (addr % FLASH_PAGE_SIZE);
        uint32_t chunk = (remaining < page_remain) ? remaining : page_remain;

        FLASH_PageProgram(addr, p, chunk);

        addr      += chunk;
        p         += chunk;
        remaining -= chunk;
    }

    flag_flash_saving = 0;
    UART_Print("Gyro Log saved to Mission Flash (with 16-byte header)!\r\n");
}

/* Construye el inner "STSEEB0B1B2" dinámicamente */
static void build_STS_inner(char *inner, size_t inner_size)
{
    uint8_t EE = 0x00;

    /* Calculate error code EE */
    if (!gyro_ok) {
        // WHO_AM_I failed
        EE = 0x01;
    }

    char b0 = flag_capture_active ? '1' : '0';   // log (RAM)
    char b1 = flag_flash_saving   ? '1' : '0';   // storing to MSFM
    char b2 = flag_data_transfer  ? '1' : '0';   // transfer data (not used yet)

    // Thus: "STS" + EE(2 hex) + B0 + B1 + B2
    snprintf(inner, inner_size, "STS%02X%c%c%c", EE, b0, b1, b2);
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
#ifdef USE_FULL_ASSERT
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
