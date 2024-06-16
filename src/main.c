/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Corps principal du programme
  ******************************************************************************
  * COPYRIGHT(c) 2024 STMicroelectronics
  ******************************************************************************
  */

/* Inclusions ----------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Variables privées ---------------------------------------------------------*/
TIM_HandleTypeDef htim2; // Gestionnaire pour le timer 2
UART_HandleTypeDef huart2; // Gestionnaire pour l'UART 2

/* USER CODE BEGIN PV */
/* Variables privées ---------------------------------------------------------*/

/* USER CODE END PV */

/* Prototypes de fonctions privées -------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void set_PWM_frequency(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency);

/* USER CODE BEGIN PFP */
/* Prototypes de fonctions privées -------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Tableau des fréquences des notes (en Hz)
uint32_t note_frequencies[] = {
  261,  // do (C4)
  277,  // do# (C#4)
  293,  // re (D4)
  311,  // re# (D#4)
  329,  // mi (E4)
  349,  // fa (F4)
  369,  // fa# (F#4)
  392,  // sol (G4)
  415,  // sol# (G#4)
  440,  // la (A4)
  466,  // la# (A#4)
  493,  // si (B4)
  523,  // do (C5)
  554,  // do# (C#5)
  587,  // re (D5)
  622,  // re# (D#5)
  659,  // mi (E5)
  698,  // fa (F5)
  739,  // fa# (F#5)
  783,  // sol (G5)
  830,  // sol# (G#5)
  880,  // la (A5)
  932,  // la# (A#5)
  987,  // si (B5)
};

#define NUM_NOTES (sizeof(note_frequencies)/sizeof(note_frequencies[0])) // Nombre total de notes

/* USER CODE END 0 */

/**
  * @brief  Point d'entrée de l'application.
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Configuration du MCU ----------------------------------------------------*/

  /* Réinitialisation de tous les périphériques, initialisation de l'interface Flash et du Systick */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configuration de l'horloge système */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialisation de tous les périphériques configurés */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();  // Initialisation de l'UART

  /* USER CODE BEGIN 2 */

  // Démarrer le PWM sur le canal 1 du timer 2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Boucle infinie */
  /* USER CODE BEGIN WHILE */
  uint8_t receivedChar; // Variable pour stocker le caractère reçu via l'UART
  while (1)
  {
    // Attendre de recevoir un caractère via l'UART
    if (HAL_UART_Receive(&huart2, &receivedChar, 1, HAL_MAX_DELAY) == HAL_OK)
    {
      // Convertir le caractère reçu en un index
      int note_index = receivedChar - 'a'; // en supposant que 'a' est la première note, 'b' la seconde, etc.

      // Vérifier si l'index est dans la plage valide
      if (note_index >= 0 && note_index < NUM_NOTES)
      {
        // Régler la fréquence du PWM sur la note correspondante
        set_PWM_frequency(&htim2, TIM_CHANNEL_1, note_frequencies[note_index]);
      }
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

}

/**
  * @brief Configuration de l'horloge système
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Configuration de l'oscillateur
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // Activation de la suralimentation
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // Configuration des horloges
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // Configuration du Systick
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* Initialisation de TIM2 */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  // Configuration du timer 2
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1; // Diviseur de fréquence
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1; // Période du timer
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // Initialisation du PWM
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // Appel de la fonction pour la configuration post-init du PWM
  HAL_TIM_MspPostInit(&htim2);
}

/* Initialisation de USART2 */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600; // Débit en bauds
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* Fonction pour régler la fréquence PWM */
void set_PWM_frequency(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t frequency)
{
  uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;  // Fréquence d'horloge APB1
  uint32_t prescaler = (timer_clock / (frequency * 65536)) + 1; // Calcul du prescaler
  uint32_t period = (timer_clock / (prescaler * frequency)) - 1; // Calcul de la période

  // Mise à jour des registres du timer
  htim->Instance->PSC = prescaler - 1;
  htim->Instance->ARR = period;
  htim->Instance->CCR1 = period / 2;  // Rapport cyclique de 50%

  // Redémarrage du PWM avec les nouvelles valeurs
  HAL_TIM_PWM_Start(htim, channel);
}

/** Configuration des broches comme
        * Analogique
        * Entrée
        * Sortie
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{
  // Activation des horloges pour les ports GPIO
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief  Cette fonction est exécutée en cas d'erreur.
  * @param  file: Le nom du fichier sous forme de chaîne.
  * @param  line: La ligne dans le fichier sous forme de nombre.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  // Boucle infinie en cas d'erreur
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Signale le nom du fichier source et le numéro de ligne où l'erreur assert_param a eu lieu.
  * @param  file: pointeur vers le nom du fichier source.
  * @param  line: numéro de ligne où l'erreur assert_param a eu lieu.
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  // Code pour gérer l'échec des assertions
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
