#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"

#define BUFFER_SIZE 64

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
uint8_t rx_data;
uint8_t rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_write_index = 0;
volatile uint8_t rx_read_index = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

void ProcessKeyboardInput(void);
void PlayNote1(uint16_t frequency);
void PlayNote2(uint16_t frequency);
void StopNote(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);

    while(1){
    	PlayNote2(5000);
    	HAL_Delay(1000);
    	PlayNote2(300);
    	HAL_Delay(1000);
    }

    HAL_UART_Receive_IT(&huart2, &rx_data, 1);

    while (1)
    {
        if (rx_read_index != rx_write_index) {
            uint8_t data = rx_buffer[rx_read_index];
            rx_read_index = (rx_read_index + 1) % BUFFER_SIZE;

            switch (data) {
                case 'A': PlayNote1(261); break; // do octave 1
                case 'Z': PlayNote1(277); break; // do#
                case 'E': PlayNote1(293); break; // ré
                case 'R': PlayNote1(311); break; // ré#
                case 'T': PlayNote1(329); break; // mi
                case 'Y': PlayNote1(349); break; // fa
                case 'U': PlayNote1(369); break; // fa#
                case 'I': PlayNote1(392); break; // sol
                case 'O': PlayNote1(415); break; // sol#
                case 'P': PlayNote1(440); break; // la
                case 'Q': PlayNote1(466); break; // la#
                case 'S': PlayNote1(493); break; // si
                case 'D': PlayNote1(523); break; // do octave 2
                case 'F': PlayNote1(261); break; // do octave 1
                case 'G': PlayNote2(277); break; // do#
                case 'H': PlayNote1(293); break; // ré
                case 'J': PlayNote2(311); break; // ré#
                case 'K': PlayNote1(329); break; // mi
                case 'L': PlayNote2(349); break; // fa
                case 'M': PlayNote1(369); break; // fa#
                case 'W': PlayNote2(392); break; // sol
                case 'X': PlayNote1(415); break; // sol#
                case 'C': PlayNote2(440); break; // la
                case 'V': PlayNote1(466); break; // la#
                case 'B': PlayNote2(493); break; // si
                case 'N': PlayNote1(523); break; // do octave 2
                default: StopNote(); break;
            }
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 691;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000000 / 440 - 1; // Default period for 440 Hz (A4)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART2_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        rx_buffer[rx_write_index] = rx_data;
        rx_write_index = (rx_write_index + 1) % BUFFER_SIZE;
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

void PlayNote1(uint16_t frequency) {
    uint16_t period = 1000000 / frequency - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim2, period);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period / 2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void PlayNote2(uint16_t frequency) {
    uint16_t period = 1000000 / frequency - 1;
//__HAL_TIM_SET_AUTORELOAD(&htim2, period);
    htim2.Instance->CCR1 = frequency;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void StopNote() {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
