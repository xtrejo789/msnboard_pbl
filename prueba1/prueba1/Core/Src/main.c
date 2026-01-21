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
#define MAX_SAMPLES_TO_LOG  1
#define BYTES_PER_SAMPLE    10
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
volatile uint8_t tx_busy = 0;

/* Conversion and bias */
static int16_t bias_x = 0, bias_y = 0, bias_z = 0;

/* ---- Estado para STS ---- */
volatile uint8_t flag_capture_active = 0;  // B0: logging de giroscopio
volatile uint8_t flag_flash_saving = 0;  // B1: guardando en MSFM
volatile uint8_t flag_data_transfer = 0;  // B2: transferencia (no usada aquí)

// UART routing
#define UART_PC   huart2   // Laptop / ST-Link VCP (debug)
#define UART_OBC  huart3   // Link hacia OBC (commands/responses)

#define FLASH_SECTOR_SIZE   (64UL*1024UL)
#define FLASH_PAGE_SIZE     (4UL*1024UL)
#define FLASH_SLOT_SIZE     (4UL*1024*1024UL)  // mem_block_size del PIC
#define GYRO_SLOT_INDEX     1
#define GYRO_BASE_ADDR      (GYRO_SLOT_INDEX * FLASH_SLOT_SIZE)

// Opcodes according to Main PIC's code
#define FLASH_CMD_RDID        0x9F
#define FLASH_CMD_RDSR        0x05
#define FLASH_CMD_WREN        0x06
#define FLASH_CMD_PP          0x12   // Page program
#define FLASH_CMD_READ        0x13   // Read 4-byte addressing
#define FLASH_CMD_ERASE_SECT  0xDC

// Macros for CS
#define FLASH_CS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define FLASH_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

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
static void PC_Print(const char *s);
static void PC_PrintHex64(const char *tag, const uint8_t *buf);
uint8_t I2C_ReadByte(uint8_t reg);
void I2C_WriteByte(uint8_t reg, uint8_t value);
void L3G4200D_Init(void);
bool L3G4200D_ReadGyro(int16_t *x, int16_t *y, int16_t *z);
static void Gyro_Calibrate(uint16_t n);
void log_current_gyro(uint32_t t_ms, int16_t gx, int16_t gy, int16_t gz);
uint8_t cam_checksum(const char *s);
size_t cam_build_response(const char *inner, uint8_t *out);
void handle_cam_command(const char *inner_cmd);
void CheckOBC_Task(uint32_t timeout_ms);
static void PC_PrintHexN(const char *tag, const uint8_t *buf, size_t nbytes);
static inline void UART_OBC_ClearRxErrors(void);
static inline uint8_t cmd_q_pop(char *out);
static void FLASH_PrintSR(const char *tag);
static bool FLASH_WriteStatus(uint8_t sr);
static uint8_t FLASH_ReadSR(void);

/* Prototypes for FM*/
bool FLASH_WriteEnable(void);
bool FLASH_WaitBusy(uint32_t timeout_ms);
bool FLASH_SectorErase(uint32_t addr);
bool FLASH_PageProgram(uint32_t addr, const uint8_t *data, uint32_t len);
void FLASH_Read(uint32_t addr, uint8_t *data, uint32_t len);
bool FLASH_WriteLogToMissionFlash(void);
static void build_STS_inner(char *inner, size_t inner_size);
static int hex_nibble(char c);
static bool parse_hex2(const char *p, uint8_t *out);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct __attribute__((packed)) {
	uint32_t t_ms;  // timestamp ms
	int16_t gx;
	int16_t gy;
	int16_t gz;
} GyroSample;

#define MAX_SAMPLES 1500
GyroSample gyro_log[MAX_SAMPLES];
uint32_t gyro_count = 0;

volatile uint8_t pending_flash_write = 0;
volatile uint8_t last_cap_result_ok = 1; // opcional

// MSN (UART)
#define CAM_MSG_LEN 64
#define OBC_HEADER  0x0B
#define CAM_HEADER  0xCA
#define INNER_MAX   32
#define CMD_Q_SIZE  16

#define RXRB_SIZE 256
static volatile uint8_t rxrb[RXRB_SIZE];
static volatile uint16_t rxrb_w = 0, rxrb_r = 0;

static volatile uint8_t cmd_q_head = 0;
static volatile uint8_t cmd_q_tail = 0;
static char cmd_q[CMD_Q_SIZE][INNER_MAX];

static volatile uint8_t pending_rst01 = 0;
static uint8_t obc_rx_byte;

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
	//UART_Print("System Initialized\r\n");
  FLASH_CS_HIGH();
  FLASH_Enter4ByteAddr();
  PC_Print("[FLASH] Unlocking Status Register...\r\n");
  FLASH_PrintSR("[FLASH] SR before unlock:");
  FLASH_WriteStatus(0x00);   // BP=000, WEL auto-clear
  FLASH_PrintSR("[FLASH] SR after unlock:");
  HAL_UART_Receive_IT(&UART_OBC, &obc_rx_byte, 1);
  uint8_t sr = FLASH_ReadSR();
  char dbg[64];
  snprintf(dbg,sizeof(dbg),"SR raw = 0x%02X\r\n",sr);
  PC_Print(dbg);
	HAL_Delay(5000);
	L3G4200D_Init();
	//UART_Print("Gyro Initialized\r\n");
	//UART_Print("Keep still for bias calib...\r\n");
	Gyro_Calibrate(200);
	//UART_Print("Bias calibrated.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/* Infinite loop */
	while (1) {
		static uint32_t t0_ms = 0;
		uint32_t now_ms = HAL_GetTick();
		if (t0_ms == 0)
			t0_ms = now_ms;
		CheckOBC_Task(0);

		// 3.1) Si hubo checksum mismatch, responde UNA vez (desde main loop)
		if (!tx_busy && pending_rst01) {
		    pending_rst01 = 0;

		    uint8_t tx_buf[CAM_MSG_LEN];
		    size_t tx_len = cam_build_response("RST01", tx_buf);

		    tx_busy = 1;
		    HAL_UART_Transmit(&UART_OBC, tx_buf, CAM_MSG_LEN, 500);
		    UART_OBC_ClearRxErrors();
		    tx_busy = 0;

		    PC_PrintHexN("[TX->OBC frame] ", tx_buf, tx_len);
		    PC_Print("\r\n");
		    PC_PrintHexN("[TX->OBC 64B ] ", tx_buf, CAM_MSG_LEN);
		    PC_Print("\r\n");
		}

		// 1) Log a RAM (gyro)
		if (gyro_ok && gyro_count < MAX_SAMPLES) {
			flag_capture_active = 1;
			////CheckOBC_Task(5);
			if (L3G4200D_ReadGyro(&gyro_x, &gyro_y, &gyro_z)) {
				//CheckOBC_Task(5):
				log_current_gyro(now_ms - t0_ms, gyro_x - bias_x,
						gyro_y - bias_y, gyro_z - bias_z);
				//CheckOBC_Task(5);
			}
		} else {
			//CheckOBC_Task(5):
			flag_capture_active = 0;
		}

		// 2) Si hubo CAP, escribe flash y responde UNA vez
		if (pending_flash_write) {
			//CheckOBC_Task(5);
			pending_flash_write = 0;
			//CheckOBC_Task(5):
			bool ok = FLASH_WriteLogToMissionFlash();
			//CheckOBC_Task(5):
			last_cap_result_ok = ok ? 1 : 0;

			//CheckOBC_Task(5);

			flag_flash_saving = 0;
		}

		if (!tx_busy) {
			//CheckOBC_Task(5);
			char tmp[INNER_MAX];
			if (cmd_q_pop(tmp)) {
				handle_cam_command(tmp);
			}
		}

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void FLASH_PrintSR(const char *tag)
{
    uint8_t sr = FLASH_ReadSR();
    char s[80];
    snprintf(s, sizeof(s),
             "%s SR=0x%02X  WIP=%u  WEL=%u  BP=%u%u%u\r\n",
             tag, sr,
             (sr & 0x01) ? 1 : 0,
             (sr & 0x02) ? 1 : 0,
             (sr & 0x20) ? 1 : 0,  // BP2
             (sr & 0x10) ? 1 : 0,  // BP1
             (sr & 0x08) ? 1 : 0   // BP0  (ojo: en algunos chips BP bits pueden variar)
    );
    PC_Print(s);
}
static uint8_t FLASH_ReadSR(void)
{
    uint8_t cmd = FLASH_CMD_RDSR; // 0x05
    uint8_t sr  = 0;
    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi2, &sr,  1, 100);
    FLASH_CS_HIGH();
    return sr;
}

void FLASH_Enter4ByteAddr(void)
{
    uint8_t cmd = 0xB7; // Enter 4-byte address mode
    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    FLASH_CS_HIGH();
    PC_Print("[FLASH] Entered 4-byte address mode\r\n");
}

static void FLASH_DumpHex(const char *tag, const uint8_t *b, uint32_t n)
{
    char line[64];
    PC_Print(tag);
    for (uint32_t i = 0; i < n; i++) {
        snprintf(line, sizeof(line), "%02X ", b[i]);
        PC_Print(line);
    }
    PC_Print("\r\n");
}

/* Lee usando tu comando 0x13 (4-byte address) */
static void FLASH_Read_4B(uint32_t addr, uint8_t *data, uint32_t len)
{
    uint8_t cmd[5];
    cmd[0] = FLASH_CMD_READ; // 0x13
    cmd[1] = (addr >> 24) & 0xFF;
    cmd[2] = (addr >> 16) & 0xFF;
    cmd[3] = (addr >> 8)  & 0xFF;
    cmd[4] = (addr >> 0)  & 0xFF;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 5, 200);
    HAL_SPI_Receive(&hspi2, data, len, 200);
    FLASH_CS_HIGH();
}
static inline int rxrb_pop(uint8_t *out) {
    if (rxrb_r == rxrb_w) return 0;
    *out = rxrb[rxrb_r];
    rxrb_r = (rxrb_r + 1) % RXRB_SIZE;
    return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_OBC.Instance) {
        uint16_t next = (rxrb_w + 1) % RXRB_SIZE;
        if (next != rxrb_r) {          // no lleno
            rxrb[rxrb_w] = obc_rx_byte;
            rxrb_w = next;
        }
        HAL_UART_Receive_IT(&UART_OBC, &obc_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_OBC.Instance) {
        UART_OBC_ClearRxErrors();
        HAL_UART_AbortReceive_IT(&UART_OBC);
        HAL_UART_Receive_IT(&UART_OBC, &obc_rx_byte, 1);
    }
}
static inline void UART_OBC_ClearRxErrors(void) {
    __HAL_UART_CLEAR_OREFLAG(&UART_OBC);
    __HAL_UART_CLEAR_FEFLAG(&UART_OBC);
    __HAL_UART_CLEAR_NEFLAG(&UART_OBC);
    __HAL_UART_CLEAR_PEFLAG(&UART_OBC);
}

static inline uint8_t cmd_q_is_empty(void) {
	return (cmd_q_head == cmd_q_tail);
}
static inline uint8_t cmd_q_is_full(void) {
	return ((uint8_t) (cmd_q_head + 1) % CMD_Q_SIZE) == cmd_q_tail;
}
static inline void cmd_q_push(const char *s) {
  __disable_irq();
  if (cmd_q_is_full()) { __enable_irq(); return; }

  strncpy(cmd_q[cmd_q_head], s, INNER_MAX-1);
  cmd_q[cmd_q_head][INNER_MAX-1] = '\0';
  cmd_q_head = (uint8_t)(cmd_q_head + 1) % CMD_Q_SIZE;
  __enable_irq();
}

static inline uint8_t cmd_q_pop(char *out) {
  __disable_irq();
  if (cmd_q_is_empty()) { __enable_irq(); return 0; }

  strncpy(out, cmd_q[cmd_q_tail], INNER_MAX);
  cmd_q_tail = (uint8_t)(cmd_q_tail + 1) % CMD_Q_SIZE;
  __enable_irq();
  return 1;
}

static int hex_nibble(char c) {
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return 10 + (c - 'A');
	if (c >= 'a' && c <= 'f')
		return 10 + (c - 'a');
	return -1;
}

static bool parse_hex2(const char *p, uint8_t *out) {
	int hi = hex_nibble(p[0]);
	int lo = hex_nibble(p[1]);
	if (hi < 0 || lo < 0)
		return false;
	*out = (uint8_t) ((hi << 4) | lo);
	return true;
}

void FLASH_Probe(void) {
	uint8_t tx[4] = { FLASH_CMD_RDID, 0x00, 0x00, 0x00 };
	uint8_t rx[4] = { 0 };

	FLASH_CS_LOW();
	HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi2, tx, rx, sizeof(tx),
			200);
	FLASH_CS_HIGH();

	if (st != HAL_OK) {
		PC_Print("[FLASH][ERR] SPI TxRx failed\r\n");
	}

	// rx[0] es basura (respuesta al 0x9F), rx[1..3] deberían ser ID
	char id_dbg[64];
	snprintf(id_dbg, sizeof(id_dbg), "[FLASH] RDID = %02X %02X %02X\r\n", rx[1],
			rx[2], rx[3]);
	PC_Print(id_dbg);

	if ((rx[1] == 0xFF && rx[2] == 0xFF && rx[3] == 0xFF)
			|| (rx[1] == 0x00 && rx[2] == 0x00 && rx[3] == 0x00)) {
	}
}
/* ======== Utilidades UART/I2C y manejo de giroscopio ======== */
static void PC_Print(const char *s) {
	HAL_UART_Transmit(&UART_PC, (uint8_t*) s, strlen(s), 500);
}

static void PC_PrintHex64(const char *tag, const uint8_t *buf) {
	char line[256];
	int n = 0;

	n += snprintf(line + n, sizeof(line) - n, "%s", tag);
	for (int i = 0; i < CAM_MSG_LEN; i++) {
		n += snprintf(line + n, sizeof(line) - n, "%02X ", buf[i]);
		if (n >= (int) sizeof(line) - 4)
			break;
	}
	n += snprintf(line + n, sizeof(line) - n, "\r\n");
	PC_Print(line);
}

static void PC_PrintHexN(const char *tag, const uint8_t *buf, size_t nbytes) {
	char line[512];   // <-- antes 256
	int n = 0;
	n += snprintf(line + n, sizeof(line) - n, "%s", tag);
	for (size_t i = 0; i < nbytes; i++) {
		n += snprintf(line + n, sizeof(line) - n, "%02X ", buf[i]);
		if (n >= (int) sizeof(line) - 4)
			break;
	}
	n += snprintf(line + n, sizeof(line) - n, "\r\n");
	PC_Print(line);
}

uint8_t I2C_ReadByte(uint8_t reg) {
	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1,
			100);
	return data;
}

void I2C_WriteByte(uint8_t reg, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1, L3G4200D_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value,
			1, 100);
}

void L3G4200D_Init(void) {
	uint8_t who = I2C_ReadByte(L3G4200D_WHO_AM_I);
	if (who != 0xD3) {
		PC_Print("L3G4200D not found! WHO_AM_I failed.\r\n");
		gyro_ok = false;
		return;
	} else {
		PC_Print("L3G4200D detected successfully.\r\n");
		gyro_ok = true;
	}

	I2C_WriteByte(L3G4200D_CTRL_REG1, 0x0F);
	I2C_WriteByte(L3G4200D_CTRL_REG4, 0x00);
}

static void I2C_RecoverIfNeeded(void) {
	// Si el bus quedó en estado BUSY/AF/ARLO/BERR, reinicia el periférico I2C1
	if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {
		// Deshabilita/rehabilita periférico (soft reset de I2C)
		__HAL_I2C_DISABLE(&hi2c1);
		HAL_Delay(1);
		__HAL_I2C_ENABLE(&hi2c1);
	}
}

static void Gyro_Calibrate(uint16_t n) {
	int32_t sx = 0, sy = 0, sz = 0;
	int16_t x, y, z;
	for (uint16_t i = 0; i < n; i++) {
		while (!L3G4200D_ReadGyro(&x, &y, &z)) {
			HAL_Delay(1);
		}
		sx += x;
		sy += y;
		sz += z;
		HAL_Delay(2);
	}
	bias_x = (int16_t) (sx / (int32_t) n);
	bias_y = (int16_t) (sy / (int32_t) n);
	bias_z = (int16_t) (sz / (int32_t) n);
}

bool L3G4200D_ReadGyro(int16_t *x, int16_t *y, int16_t *z) {
	//CheckOBC_Task(5);
	uint8_t status = I2C_ReadByte(L3G4200D_STATUS_REG);
	if ((status & ZYXDA) == 0) {
		return false;
	}
	//CheckOBC_Task(5);
	uint8_t buf[6];
	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR,
			(L3G4200D_OUT_X_L | 0x80),
			I2C_MEMADD_SIZE_8BIT, buf, 6, 10);
	//CheckOBC_Task(5);
	if (st != HAL_OK) {
		//CheckOBC_Task(5);
		I2C_RecoverIfNeeded();
		st = HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR, (L3G4200D_OUT_X_L | 0x80),
		I2C_MEMADD_SIZE_8BIT, buf, 6, 10);
		if (st != HAL_OK) {
			//CheckOBC_Task(5);
			HAL_I2C_DeInit(&hi2c1);
			HAL_I2C_Init(&hi2c1);
			return false;
		}
	}
	//CheckOBC_Task(5);

	*x = (int16_t) ((buf[1] << 8) | buf[0]);
	*y = (int16_t) ((buf[3] << 8) | buf[2]);
	*z = (int16_t) ((buf[5] << 8) | buf[4]);
	return true;
}

void log_current_gyro(uint32_t t_ms, int16_t gx, int16_t gy, int16_t gz) {
	//CheckOBC_Task(5);
	if (gyro_count >= MAX_SAMPLES)
		return;
	//CheckOBC_Task(5);
	gyro_log[gyro_count].t_ms = t_ms;
	gyro_log[gyro_count].gx = gx;
	gyro_log[gyro_count].gy = gy;
	gyro_log[gyro_count].gz = gz;
	gyro_count++;
	//CheckOBC_Task(5);
}

uint8_t cam_checksum(const char *s) {
	uint8_t c = 0;
	while (*s) {
		c ^= (uint8_t) (*s++);
	}
	return c;
}

static uint8_t cam_checksum_xor64(const uint8_t *frame64) {
	// XOR bytes 1..61 (excluye header [0], checksum [62] y footer [63])
	uint8_t c = 0;
	for (int i = 1; i <= 61; i++)
		c ^= frame64[i];
	return c;
}

static char hex_digit(uint8_t v) {
	v &= 0x0F;
	return (v < 10) ? ('0' + v) : ('A' + (v - 10));
}

static uint8_t cam_checksum_xor_range(const uint8_t *buf, int start, int end) {
	uint8_t c = 0;
	for (int i = start; i <= end; i++)
		c ^= buf[i];
	return c;
}

size_t cam_build_response(const char *inner, uint8_t *out) {
	//CheckOBC_Task(5);
	memset(out, 0, CAM_MSG_LEN);     // 64 bytes

	out[0] = CAM_HEADER;             // 0xCA
	//CheckOBC_Task(5);
	// inner goes in [1..60] max (because [61..62] are checksum ASCII)
	size_t len = strlen(inner);
	if (len > 60)
		len = 60;
	memcpy(&out[1], inner, len);
	//CheckOBC_Task(5);
	// checksum XOR of the C-string (stops at '\0')
	uint8_t cs = 0;
	for (int i = 1; i <= 60; i++)
		cs ^= out[i];
	//CheckOBC_Task(5);
	// place checksum as 2 ASCII hex digits at [61] and [62]
	out[61] = (uint8_t) hex_digit(cs >> 4);
	out[62] = (uint8_t) hex_digit(cs);
	//CheckOBC_Task(5);
	// footer fixed at last byte
	out[63] = CAM_HEADER + 1;        // 0xCB
	//CheckOBC_Task(5);
	uint8_t cs_str = cam_checksum(inner);
	uint8_t cs_fix = cam_checksum_xor_range(out, 1, 60);
	//CheckOBC_Task(5);
	char dbg[96];
	snprintf(dbg, sizeof(dbg),
			"DBG: cs_str=%02X cs_fix(1..60)=%02X ascii=%c%c\r\n", cs_str,
			cs_fix, out[61], out[62]);
	PC_Print(dbg);
	//CheckOBC_Task(5);
	return CAM_MSG_LEN;
}

void handle_cam_command(const char *inner_cmd) {
	//CheckOBC_Task(0);
	uint8_t tx_buf[CAM_MSG_LEN];
	size_t tx_len = 0;
	//CheckOBC_Task(0);
	snprintf(msg, sizeof(msg), "CMD from OBC: %s\r\n", inner_cmd);
	PC_Print(msg);
	//CheckOBC_Task(0);
	if (strncmp(inner_cmd, "STS", 3) == 0) {
		char inner[16];
		//CheckOBC_Task(0);
		build_STS_inner(inner, sizeof(inner));
		//CheckOBC_Task(0);
		tx_len = cam_build_response(inner, tx_buf);

	} else if (strncmp(inner_cmd, "TIM", 3) == 0) {
		tx_len = cam_build_response("TIM00", tx_buf);

	} else if (strncmp(inner_cmd, "CAP", 3) == 0) {

	    flag_flash_saving = 1;
	    pending_flash_write = 1;

	    tx_len = cam_build_response("CAP00", tx_buf);
	    tx_busy = 1;
	    HAL_UART_Transmit(&UART_OBC, tx_buf, CAM_MSG_LEN, 500);
	    UART_OBC_ClearRxErrors();
	    tx_busy = 0;

	    PC_PrintHexN("[TX->OBC frame] ", tx_buf, tx_len);
	    PC_Print("\r\n");
	    PC_PrintHexN("[TX->OBC 64B ] ", tx_buf, CAM_MSG_LEN);
	    PC_Print("\r\n");

	    return;
	} else if (strncmp(inner_cmd, "JPG", 3) == 0) {
		tx_len = cam_build_response("JPG00", tx_buf);

	} else if (strncmp(inner_cmd, "CMC", 3) == 0) {
		tx_len = cam_build_response("CMC00", tx_buf);

	} else if (strncmp(inner_cmd, "PDN", 3) == 0) {
		tx_len = cam_build_response("PDN00", tx_buf);

	} else if (strncmp(inner_cmd, "RST", 3) == 0) {
		tx_len = cam_build_response("RST00", tx_buf);

	} else if (strncmp(inner_cmd, "OWT", 3) == 0) {
		tx_len = cam_build_response("OWT00", tx_buf);

	} else if (strncmp(inner_cmd, "CAN", 3) == 0) {
		tx_len = cam_build_response("CAN00", tx_buf);

	} else if (strncmp(inner_cmd, "NUM", 3) == 0) {
		tx_len = cam_build_response("NUM00000", tx_buf);

	} else if (strncmp(inner_cmd, "IMG", 3) == 0) {
		tx_len = cam_build_response("IMG000000000000", tx_buf);

	} else if (strncmp(inner_cmd, "CMW", 3) == 0) {
		tx_len = cam_build_response("CMW00", tx_buf);

	} else if (strncmp(inner_cmd, "CMR", 3) == 0) {
		tx_len = cam_build_response("CMR0000", tx_buf);

	} else {
		tx_len = cam_build_response("RST01", tx_buf);
	}
	//CheckOBC_Task(5);
	//HAL_Delay(20);
	if (tx_len > 0 && tx_buf[tx_len - 1] != 0xCB) {
		PC_Print("ERR: tx footer not CB!\r\n");
	}

	char dbg2[64];
	snprintf(dbg2, sizeof(dbg2), "DBG2: tx_len=%u last=%02X\r\n",
			(unsigned) tx_len, tx_buf[tx_len - 1]);
	PC_Print(dbg2);

	tx_busy = 1;
	HAL_UART_Transmit(&UART_OBC, tx_buf, CAM_MSG_LEN, 500);
	UART_OBC_ClearRxErrors();
	tx_busy = 0;
	//HAL_Delay(10);
	//CheckOBC_Task(5);
	// 1) Frame real (útil para ver header, inner, checksum y CB rápido)
	PC_PrintHexN("[TX->OBC frame] ", tx_buf, tx_len);
	PC_Print("\r\n");
	//CheckOBC_Task(5);
	// 2) Paquete completo (útil para confirmar padding y que CB sí está dentro de 64)
	PC_PrintHexN("[TX->OBC 64B ] ", tx_buf, CAM_MSG_LEN);
	PC_Print("\r\n");
}

// Makes sure Payload sends STS even while it is writing data to MSFM
// Footer-terminated parser (Main PIC style): header 0x0B ... checksum(2 ASCII) ... footer 0x0C
void CheckOBC_Task(uint32_t timeout_ms_unused)
{

	static uint8_t reentry = 0;
	if (reentry) return;
	reentry = 1;

    (void)timeout_ms_unused;

    // Buffer para un frame "variable", pero con límite
    static uint8_t rx_buf[CAM_MSG_LEN];   // puedes dejar 64 como máximo permitido
    static uint8_t rx_pos = 0;
    static uint8_t in_frame = 0;
    static uint32_t t0 = 0;

    const uint8_t HDR = OBC_HEADER;       // 0x0B
    const uint8_t FTR = (OBC_HEADER + 1); // 0x0C

    // Si se quedó a medias mucho tiempo, resetea
    if (in_frame && (HAL_GetTick() - t0) > 150) {
        in_frame = 0;
        rx_pos = 0;
    }

    // Si estás transmitiendo, no leas (tu lógica actual)
    /*
    if (tx_busy) {
        return;
    }
    */

    uint8_t b;

    // Lee todo lo disponible sin bloquear
    while (rxrb_pop(&b))
    {
        // Espera header
        if (!in_frame) {
            if (b == HDR) {
                in_frame = 1;
                rx_pos = 0;
                rx_buf[rx_pos++] = b;
                t0 = HAL_GetTick();
            }
            continue;
        }

        // Re-sync: si aparece un nuevo header dentro del frame, reinicia
        if (b == HDR) {
            rx_pos = 0;
            rx_buf[rx_pos++] = b;
            t0 = HAL_GetTick();
            continue;
        }

        // Guarda byte si hay espacio
        if (rx_pos < CAM_MSG_LEN) {
            rx_buf[rx_pos++] = b;
        } else {
            // overflow -> drop frame
            in_frame = 0;
            rx_pos = 0;
            continue;
        }

        // Si llegó footer, parsea YA (frame variable)
        if (b == FTR)
        {
        	PC_PrintHexN("[RX<-OBC frame] ", rx_buf, rx_pos);
        	PC_Print("\r\n");

            int footer_pos = (int)rx_pos - 1;   // índice donde está el footer

            // Necesitas mínimo: [HDR][3 cmd][2 cs][FTR] => 1 + 3 + 2 + 1 = 7
            // En general: inner_len >= 3
            if (footer_pos >= (1 + 3 + 2))
            {
                int inner_len = footer_pos - 1 - 2;  // bytes entre header y checksum
                if (inner_len > 0 && inner_len < INNER_MAX)
                {
                    char inner[INNER_MAX];
                    memcpy(inner, &rx_buf[1], inner_len);
                    inner[inner_len] = '\0';

                    char cs_ascii[3];
                    cs_ascii[0] = (char)rx_buf[footer_pos - 2];
                    cs_ascii[1] = (char)rx_buf[footer_pos - 1];
                    cs_ascii[2] = '\0';

                    uint8_t cs_rx = 0;
                    if (parse_hex2(cs_ascii, &cs_rx))
                    {
                        uint8_t cs_calc = cam_checksum(inner);

                        if (cs_rx == cs_calc) {
                            // OK -> encola comando
                            cmd_q_push(inner);
                        } else {
                        	pending_rst01 = 1;
                        }
                    }
                }
            }

            // Frame terminado: reset para el siguiente
            in_frame = 0;
            rx_pos = 0;

            // OJO: salimos para que el main loop procese cmd_q -> handle_cam_command()
            break;
        }
    }
    reentry = 0;

}

bool FLASH_WriteEnable(void)
{
    uint8_t cmd = FLASH_CMD_WREN; // 0x06

    FLASH_CS_LOW();
    if (HAL_SPI_Transmit(&hspi2, &cmd, 1, 100) != HAL_OK) {
        FLASH_CS_HIGH();
        return false;
    }
    FLASH_CS_HIGH();

    // confirma WEL
    uint8_t sr = FLASH_ReadSR();
    if ((sr & 0x02) == 0) {
        PC_Print("[FLASH][ERR] WREN sent but WEL=0\r\n");
        return false;
    }
    return true;
}

bool FLASH_WaitBusy(uint32_t timeout_ms) {
    uint8_t cmd = FLASH_CMD_RDSR; // 0x05
    uint8_t status = 0x01;
    uint32_t t0 = HAL_GetTick();

    while (1) {
        FLASH_CS_LOW();
        if (HAL_SPI_Transmit(&hspi2, &cmd, 1, 100) != HAL_OK) {
            FLASH_CS_HIGH();
            return false;
        }
        if (HAL_SPI_Receive(&hspi2, &status, 1, 100) != HAL_OK) {
            FLASH_CS_HIGH();
            return false;
        }
        FLASH_CS_HIGH();

        if ((status & 0x01) == 0) {
            return true; // ya no busy
        }

        if ((HAL_GetTick() - t0) > timeout_ms) {
            return false; // timeout
        }
    }
}

bool FLASH_SectorErase(uint32_t addr)
{
    uint8_t cmd[5] = {
        FLASH_CMD_ERASE_SECT,
        (addr >> 24) & 0xFF,
        (addr >> 16) & 0xFF,
        (addr >>  8) & 0xFF,
        (addr >>  0) & 0xFF
    };

    FLASH_PrintSR("[FLASH] Before WREN:");

    if (!FLASH_WriteEnable()) {
        FLASH_PrintSR("[FLASH] After WREN fail:");
        return false;
    }

    FLASH_PrintSR("[FLASH] After WREN:");

    FLASH_CS_LOW();
    if (HAL_SPI_Transmit(&hspi2, cmd, 5, 200) != HAL_OK) {
        FLASH_CS_HIGH();
        PC_Print("[FLASH][ERR] Erase TX failed\r\n");
        return false;
    }
    FLASH_CS_HIGH();

    // Checa si arrancó busy
    FLASH_PrintSR("[FLASH] After ERASE cmd:");

    // OJO: 64KB erase puede tardar >5s según chip/condiciones
    if (!FLASH_WaitBusy(30000)) {   // prueba con 30s
        FLASH_PrintSR("[FLASH] WaitBusy timeout/fail:");
        return false;
    }

    FLASH_PrintSR("[FLASH] After ERASE done:");
    return true;
}

bool FLASH_PageProgram(uint32_t addr, const uint8_t *data, uint32_t len) {
	uint8_t cmd[5] = {
	FLASH_CMD_PP, (addr >> 24) & 0xFF, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF,
			(addr >> 0) & 0xFF };

	if (!FLASH_WriteEnable()){
		PC_Print("[CAP]!FLASH_WriteEnable()\r\n");
		return false;
	}

	FLASH_CS_LOW();
	if (HAL_SPI_Transmit(&hspi2, cmd, 5, 200) != HAL_OK) {
		FLASH_CS_HIGH();
		PC_Print("[CAP] HAL_SPI_Transmit(&hspi2, cmd, 5, 200) != HAL_OK\r\n");
		return false;
	}
	if (HAL_SPI_Transmit(&hspi2, (uint8_t*) data, len, 200) != HAL_OK) {
		FLASH_CS_HIGH();
		PC_Print("[CAP] HAL_SPI_Transmit(&hspi2, cmd, 5, 200) != HAL_OK\r\n");
		return false;
	}
	FLASH_CS_HIGH();

	return FLASH_WaitBusy(2000);
}

static bool FLASH_WriteStatus(uint8_t sr)
{
    uint8_t cmd[2] = { 0x01, sr }; // WRSR

    if (!FLASH_WriteEnable()) {
        PC_Print("[FLASH][ERR] WREN failed before WRSR\r\n");
        return false;
    }

    FLASH_CS_LOW();
    if (HAL_SPI_Transmit(&hspi2, cmd, 2, 200) != HAL_OK) {
        FLASH_CS_HIGH();
        PC_Print("[FLASH][ERR] WRSR TX failed\r\n");
        return false;
    }
    FLASH_CS_HIGH();

    return FLASH_WaitBusy(5000);
}

void FLASH_Read(uint32_t addr, uint8_t *data, uint32_t len) {
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

bool FLASH_WriteLogToMissionFlash(void) {
	//CheckOBC_Task(5);
    PC_Print("[CAP] Enter FLASH_WriteLogToMissionFlash()\r\n");
    flag_flash_saving = 1;

    char dbg[96];
    uint32_t file_size = 0;
    //CheckOBC_Task(5);
    // 0) Decide cuántas muestras
    uint32_t samples_to_log = gyro_count;
    if (samples_to_log > MAX_SAMPLES_TO_LOG) samples_to_log = MAX_SAMPLES_TO_LOG;
    //CheckOBC_Task(5);
    snprintf(dbg, sizeof(dbg), "[CAP] gyro_count=%lu\r\n", (unsigned long)gyro_count);
    PC_Print(dbg);
    snprintf(dbg, sizeof(dbg), "[CAP] samples_to_log=%lu\r\n", (unsigned long)samples_to_log);
    PC_Print(dbg);

    //CheckOBC_Task(5);

    // 1) Construye payload de datos (t_ms,gx,gy,gz) = 10 bytes por muestra
    file_size = samples_to_log * BYTES_PER_SAMPLE;
    snprintf(dbg, sizeof(dbg), "[CAP] file_size=%lu bytes\r\n", (unsigned long)file_size);
    PC_Print(dbg);

    if (file_size > (FLASH_SLOT_SIZE - 0x10)) {
        flag_flash_saving = 0;
        return false;
    }
    //CheckOBC_Task(5);
    uint32_t base = GYRO_BASE_ADDR;
    snprintf(dbg, sizeof(dbg),
             "[CAP] SLOT=%lu  BASE=0x%08lX  (slot_size=%lu bytes)\r\n",
             (unsigned long)GYRO_SLOT_INDEX,
             (unsigned long)base,
             (unsigned long)FLASH_SLOT_SIZE);
    PC_Print(dbg);
    // 2) BORRA SOLO 1 SECTOR (64KB) -> suficiente para header+datos
    PC_Print("[CAP] Erasing only 1 sector (64KB)\r\n");
    if (!FLASH_SectorErase(base)) {
        PC_Print("[CAP][ERR] Sector erase failed\r\n");
        flag_flash_saving = 0;
        return false;
    }
    //CheckOBC_Task(5);

    // 3) Header 16B (igual que tu formato)
    uint8_t header[16] = {0};
    header[0] = (file_size & 0xFF);
    header[1] = (file_size >> 8) & 0xFF;
    header[2] = (file_size >> 16) & 0xFF;
    header[3] = (file_size >> 24) & 0xFF;
    //CheckOBC_Task(5);
    PC_Print("[CAP] Writing header (16B)\r\n");
    if (!FLASH_PageProgram(base, header, sizeof(header))) {
        PC_Print("[CAP][ERR] Header PageProgram failed\r\n");
        flag_flash_saving = 0;
        return false;
    }
    //CheckOBC_Task(5);
    // --- DEBUG: readback del header ---
    uint8_t rb_hdr[16] = {0};
    FLASH_Read_4B(base, rb_hdr, sizeof(rb_hdr));

    FLASH_DumpHex("[CAP][WR] HDR: ", header, sizeof(header));
    FLASH_DumpHex("[CAP][RB] HDR: ", rb_hdr, sizeof(rb_hdr));

    // 4) Escribir 1 muestra (t_ms + gx + gy + gz) = 10 bytes
    uint32_t addr = base + sizeof(header);

    if (gyro_count == 0) {
        PC_Print("[CAP][ERR] No samples in RAM\r\n");
        flag_flash_saving = 0;
        return false;
    }
    //CheckOBC_Task(5);
    uint8_t row[BYTES_PER_SAMPLE];
    uint32_t idx = gyro_count - 1;

    uint32_t ts = gyro_log[idx].t_ms;
    int16_t gx = gyro_log[idx].gx;
    int16_t gy = gyro_log[idx].gy;
    int16_t gz = gyro_log[idx].gz;

    //CheckOBC_Task(5);

    // t_ms (uint32_t) little-endian
    row[0] = (uint8_t)(ts & 0xFF);
    row[1] = (uint8_t)((ts >> 8) & 0xFF);
    row[2] = (uint8_t)((ts >> 16) & 0xFF);
    row[3] = (uint8_t)((ts >> 24) & 0xFF);
    //CheckOBC_Task(5);
    // gx, gy, gz (int16_t) little-endian
    row[4] = (uint8_t)(gx & 0xFF);
    row[5] = (uint8_t)((gx >> 8) & 0xFF);
    row[6] = (uint8_t)(gy & 0xFF);
    row[7] = (uint8_t)((gy >> 8) & 0xFF);
    row[8] = (uint8_t)(gz & 0xFF);
    row[9] = (uint8_t)((gz >> 8) & 0xFF);
    //CheckOBC_Task(5);
    if (!FLASH_PageProgram(addr, row, sizeof(row))) {
        snprintf(dbg, sizeof(dbg),
                 "[CAP][ERR] Data write failed @ 0x%08lX\r\n",
                 (unsigned long)addr);
        PC_Print(dbg);
        flag_flash_saving = 0;
        return false;
    }
    //CheckOBC_Task(5);

    // --- DEBUG: readback de la muestra ---
    uint8_t rb_row[BYTES_PER_SAMPLE] = {0};
    FLASH_Read_4B(addr, rb_row, sizeof(rb_row));

    snprintf(dbg, sizeof(dbg),
             "[CAP] DATA_ADDR=0x%08lX  bytes=%u\r\n",
             (unsigned long)addr, (unsigned)BYTES_PER_SAMPLE);
    PC_Print(dbg);

    FLASH_DumpHex("[CAP][WR] ROW: ", row, sizeof(row));
    FLASH_DumpHex("[CAP][RB] ROW: ", rb_row, sizeof(rb_row));

    flag_flash_saving = 0;
    PC_Print("[CAP] Flash write SUCCESS\r\n");
    return true;
}

/* Construye el inner "STSEEB0B1B2" dinámicamente */
static void build_STS_inner(char *inner, size_t inner_size) {
	uint8_t EE = 0x00;

	/* Calculate error code EE */
	if (!gyro_ok) {
		// WHO_AM_I failed
		EE = 0x01;
	}

	char b0 = flag_capture_active ? '1' : '0';   // log (RAM)
	char b1 = flag_flash_saving ? '1' : '0';   // storing to MSFM
	char b2 = flag_data_transfer ? '1' : '0';   // transfer data (not used yet)
	//CheckOBC_Task(0);
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
	while (1) {
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
