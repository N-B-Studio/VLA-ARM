/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "ws2812.h"
#include "bsp_fdcan.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    uint8_t node_id;
    float pos_turns;
    float vel_turns_s;
    float torque_target_nm;
    float torque_estimate_nm;
    uint32_t axis_error;
    uint8_t axis_state;
    uint32_t hb_count;
    uint32_t enc_count;
    uint32_t tq_count;
    uint32_t last_rx_ms;
} ODriveState;

#define NODE_LEADER    2   // motor on CAN1
#define NODE_FOLLOWER  1   // motor on CAN2

static ODriveState leader   = {NODE_LEADER,   0};
static ODriveState follower = {NODE_FOLLOWER, 0};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t loop_1khz_flag = 0;
static uint32_t loop_count = 0;
static uint32_t can_tx_drop = 0;
static uint32_t can_rx_other = 0;

#define CTRL_DIV 100   // 1=1kHz, 2=500Hz, 4=250Hz, 10=100Hz

// Start conservative. These are real Nm now if ODrive torque calibration is correct.
static float K_COUPLE = 0.01f;     // Nm/rad
static float D_COUPLE = 0.000f;    // Nm/(rad/s)
static float TAU_MAX  = 0.01f;     // Nm

// If direction is wrong, flip one or both signs.
//static float LEADER_TORQUE_SIGN   =  1.0f;
//static float FOLLOWER_TORQUE_SIGN = -1.0f;

static const float TWO_PI = 6.28318530718f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void led_yellow(void){ WS2812_Ctrl(60, 60, 0); }
static inline void led_red(void)   { WS2812_Ctrl(80, 0, 0);  }
static inline void led_green(void) { WS2812_Ctrl(0, 80, 0);  }

static char logbuf[224];

static void log_uart1(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), 100);
}

static void logf_uart1(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(logbuf, sizeof(logbuf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    size_t len = strnlen(logbuf, sizeof(logbuf));
    HAL_UART_Transmit(&huart1, (uint8_t*)logbuf, (uint16_t)len, 100);
}

#define MOTOR_EN_PORT GPIOC
#define MOTOR_EN_PIN_1  GPIO_PIN_14
#define MOTOR_EN_PIN_2  GPIO_PIN_13

static inline void arm_enable(uint8_t en)
{
    HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN_1, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN_2, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline float clampf(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

static uint32_t le_u32(const uint8_t *d)
{
    return ((uint32_t)d[0]) | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}

static float le_float(const uint8_t *d)
{
    float f;
    uint32_t u = le_u32(d);
    memcpy(&f, &u, sizeof(float));
    return f;
}

static void put_u32_le(uint8_t *d, uint32_t v)
{
    d[0] = (uint8_t)(v & 0xFF);
    d[1] = (uint8_t)((v >> 8) & 0xFF);
    d[2] = (uint8_t)((v >> 16) & 0xFF);
    d[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void put_float_le(uint8_t *d, float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof(float));
    put_u32_le(d, u);
}

static inline uint16_t odrive_id(uint8_t node_id, uint8_t cmd_id)
{
    return (uint16_t)((node_id << 5) | (cmd_id & 0x1F));
}

static int odrive_send(FDCAN_HandleTypeDef *hcan, uint8_t node_id, uint8_t cmd_id, const uint8_t *data, uint32_t len)
{
    uint8_t payload[8] = {0};
    if (data && len > 0) {
        if (len > 8) len = 8;
        memcpy(payload, data, len);
    }
    int r = fdcanx_send_data(hcan, odrive_id(node_id, cmd_id), payload, len);
    if (r != 0) can_tx_drop++;
    return r;
}

static int odrive_clear_errors(FDCAN_HandleTypeDef *hcan, uint8_t node_id)
{
    return odrive_send(hcan, node_id, 0x18, NULL, 0);
}

static int odrive_set_axis_state(FDCAN_HandleTypeDef *hcan, uint8_t node_id, uint32_t state)
{
    uint8_t d[4];
    put_u32_le(d, state);
    return odrive_send(hcan, node_id, 0x07, d, 4);
}

static int odrive_set_controller_mode(FDCAN_HandleTypeDef *hcan, uint8_t node_id, uint32_t control_mode, uint32_t input_mode)
{
    uint8_t d[8];
    put_u32_le(&d[0], control_mode);
    put_u32_le(&d[4], input_mode);
    return odrive_send(hcan, node_id, 0x0B, d, 8);
}

static int odrive_set_input_torque(FDCAN_HandleTypeDef *hcan, uint8_t node_id, float torque_nm)
{
    torque_nm = clampf(torque_nm, -TAU_MAX, TAU_MAX);
    uint8_t d[4];
    put_float_le(d, torque_nm);
    return odrive_send(hcan, node_id, 0x0E, d, 4);
}

void jc_parse_feedback(uint16_t rx_id, uint8_t *d, uint8_t len)  // called from bsp_fdcan.c
{
    if (len < 1) return;

    uint8_t node = (uint8_t)(rx_id >> 5);
    uint8_t cmd  = (uint8_t)(rx_id & 0x1F);

    ODriveState *m = NULL;
    if (node == NODE_LEADER) {
        m = &leader;
    } else if (node == NODE_FOLLOWER) {
        m = &follower;
    } else {
        can_rx_other++;
        return;
    }

    switch (cmd) {
        case 0x01: // Heartbeat: Axis_Error uint32, Axis_State uint8 at byte 4
            if (len >= 8) {
                m->axis_error = le_u32(&d[0]);
                m->axis_state = d[4];
                m->hb_count++;
                m->last_rx_ms = HAL_GetTick();
            }
            break;

        case 0x09: // Get_Encoder_Estimates: pos turns, vel turns/s
            if (len >= 8) {
                m->pos_turns = le_float(&d[0]);
                m->vel_turns_s = le_float(&d[4]);
                m->enc_count++;
                m->last_rx_ms = HAL_GetTick();
            }
            break;

        case 0x1C: // Get_Torques: target Nm, estimate Nm. Only if enabled/requested.
            if (len >= 8) {
                m->torque_target_nm = le_float(&d[0]);
                m->torque_estimate_nm = le_float(&d[4]);
                m->tq_count++;
                m->last_rx_ms = HAL_GetTick();
            }
            break;

        default:
            break;
    }
}

static void fatal(const char *msg)
{
    led_red();
    logf_uart1("[FATAL] %s\r\n", msg);
    odrive_set_input_torque(&hfdcan1, NODE_LEADER, 0.0f);
    odrive_set_input_torque(&hfdcan2, NODE_FOLLOWER, 0.0f);
    arm_enable(0);
    __disable_irq();
    while (1) { HAL_Delay(1000); }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART10_UART_Init();
  MX_SPI6_Init();

  /* USER CODE BEGIN 2 */
  led_yellow();
  log_uart1("\r\n[BOOT] ODrive 1kHz clean torque-coupling demo\r\n");

  arm_enable(0);
  HAL_Delay(50);

  log_uart1("[INIT] CAN...\r\n");
  bsp_can_init();
  HAL_Delay(50);

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_TIM_Base_Start_IT(&htim2);

  log_uart1("[INIT] motor power enable\r\n");
  arm_enable(1);
  HAL_Delay(500);

  log_uart1("[INIT] clear errors\r\n");
  odrive_clear_errors(&hfdcan1, NODE_LEADER);
  HAL_Delay(20);
  odrive_clear_errors(&hfdcan2, NODE_FOLLOWER);
  HAL_Delay(100);

  log_uart1("[INIT] torque control + passthrough\r\n");
  // Control_Mode=1 torque, Input_Mode=1 passthrough
  if (odrive_set_controller_mode(&hfdcan1, NODE_LEADER, 1, 1) != 0) fatal("mode leader");
  HAL_Delay(50);
  if (odrive_set_controller_mode(&hfdcan2, NODE_FOLLOWER, 1, 1) != 0) fatal("mode follower");
  HAL_Delay(50);

  log_uart1("[INIT] closed loop\r\n");
  // Axis_State=8 closed loop
  if (odrive_set_axis_state(&hfdcan1, NODE_LEADER, 8) != 0) fatal("closed leader");
  HAL_Delay(100);
  if (odrive_set_axis_state(&hfdcan2, NODE_FOLLOWER, 8) != 0) fatal("closed follower");
  HAL_Delay(100);

  odrive_set_input_torque(&hfdcan1, NODE_LEADER, 0.0f);
  odrive_set_input_torque(&hfdcan2, NODE_FOLLOWER, 0.0f);

  led_green();
  logf_uart1("[OK] running. leader=%d follower=%d\r\n", NODE_LEADER, NODE_FOLLOWER);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  uint32_t last_log_ms = HAL_GetTick();
  uint32_t last_enc_L = 0, last_enc_F = 0, last_hb_L = 0, last_hb_F = 0;
  uint8_t zero_init = 0;
  float zero_L = 0.0f, zero_F = 0.0f;

  while (1)
  {
    if (!loop_1khz_flag) continue;
    loop_1khz_flag = 0;
    loop_count++;
    if ((loop_count % CTRL_DIV) != 0) continue;

    if (!zero_init && leader.enc_count > 0 && follower.enc_count > 0) {
        zero_L = leader.pos_turns;
        zero_F = follower.pos_turns;
        zero_init = 1;
        logf_uart1("[ZERO] L %.4f F %.4f turns\r\n", zero_L, zero_F);
    }

    float tau = 0.0f;

	float qL  = (leader.pos_turns - zero_L) * TWO_PI;
	float qF  = (follower.pos_turns - zero_F) * TWO_PI;
	float dqL = leader.vel_turns_s * TWO_PI;
	float dqF = follower.vel_turns_s * TWO_PI;

	// SimpleFOC haptic virtual spring:
	// leader torque   = K * (follower - leader)
	// follower torque = K * (leader - follower)
	float tau_L = K_COUPLE * (qF - qL) + D_COUPLE * (dqF - dqL);
	float tau_F = K_COUPLE * (qL - qF) + D_COUPLE * (dqL - dqF);

	tau_L = clampf(tau_L, -TAU_MAX, TAU_MAX);
	tau_F = clampf(tau_F, -TAU_MAX, TAU_MAX);

	int r1 = odrive_set_input_torque(&hfdcan1, NODE_LEADER, tau_L);
	int r2 = odrive_set_input_torque(&hfdcan2, NODE_FOLLOWER, tau_F);


    uint32_t now = HAL_GetTick();
    if ((now - last_log_ms) >= 100)
    {
        uint32_t dencL = leader.enc_count - last_enc_L;
        uint32_t dencF = follower.enc_count - last_enc_F;
        uint32_t dhbL  = leader.hb_count - last_hb_L;
        uint32_t dhbF  = follower.hb_count - last_hb_F;
        last_enc_L = leader.enc_count;
        last_enc_F = follower.enc_count;
        last_hb_L = leader.hb_count;
        last_hb_F = follower.hb_count;
        last_log_ms = now;

        float qL_deg = zero_init ? (leader.pos_turns - zero_L) * 360.0f : 0.0f;
        float qF_deg = zero_init ? (follower.pos_turns - zero_F) * 360.0f : 0.0f;

        logf_uart1(
            "qL %.2f qF %.2f e %.2f tau %.3f tqL %.3f tqF %.3f st %u %u err %08lX %08lX | enc %lu %lu hb %lu %lu drop %lu r %d %d\r\n",
            qL_deg,
            qF_deg,
            qL_deg - qF_deg,
            tau,
            leader.torque_estimate_nm,
            follower.torque_estimate_nm,
            leader.axis_state,
            follower.axis_state,
            (unsigned long)leader.axis_error,
            (unsigned long)follower.axis_error,
            (unsigned long)dencL,
            (unsigned long)dencF,
            (unsigned long)dhbL,
            (unsigned long)dhbF,
            (unsigned long)can_tx_drop,
            r1,
            r2
        );
    }
  }
  /* USER CODE END WHILE */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        loop_1khz_flag = 1;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
