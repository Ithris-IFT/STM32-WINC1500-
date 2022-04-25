/* USER CODE BEGIN Header */
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "driver/include/m2m_wifi.h"
#include "config/conf_winc.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
//SPI_HandleTypeDef hspi3;

UART_HandleTypeDef UartHandle;

/* USER CODE BEGIN PV */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
static void SystemClock_Config(void);
static void Error_Handler(void);
extern void isr(void);

void EXTI15_10_IRQHandler(void)
{
    uint16_t GPIO_Pin;

    /* Get GPIO_Pin */
    if (__HAL_GPIO_EXTI_GET_IT(CONF_WINC_SPI_INT_PIN))
    {
        GPIO_Pin = CONF_WINC_SPI_INT_PIN;
    }

    HAL_GPIO_EXTI_IRQHandler(GPIO_Pin);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == CONF_WINC_SPI_INT_PIN)
    {
        isr();
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */


static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("Station disconnected\r\n");
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("Station connected\r\n");
		printf("Station IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		break;
	}

	default:
	{
		break;
	}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	tstrWifiInitParam param;
	tstrM2MAPConfig strM2MAPConfig;
	int8_t ret;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  UartHandle.Instance        = USARTx;

   UartHandle.Init.BaudRate   = 115200 ; //9600
   UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
   UartHandle.Init.StopBits   = UART_STOPBITS_1;
   UartHandle.Init.Parity     = UART_PARITY_NONE; //UART_PARITY_ODD
   UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
   UartHandle.Init.Mode       = UART_MODE_TX_RX;
   UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
   if (HAL_UART_Init(&UartHandle) != HAL_OK)
   {
     /* Initialization Error */
     Error_Handler();
   }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
   /* GPIO Ports Clock Enable */
   	__GPIOC_CLK_ENABLE();
   	__GPIOA_CLK_ENABLE();
   	__GPIOB_CLK_ENABLE();
     /* Initialize the BSP. */
     nm_bsp_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

  		/* Initialize Wi-Fi driver with data and status callbacks. */
  		param.pfAppWifiCb = wifi_cb;
  		ret = m2m_wifi_init(&param);
  		  if (M2M_SUCCESS != ret)
  		  {
  			  printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
  			  while (1)
  			  {

  			  }
  		  }

  			/* Initialize AP mode parameters structure with SSID, channel and OPEN security type. */
  		  memset(&strM2MAPConfig, 0x00, sizeof(tstrM2MAPConfig));
  		  strcpy((char *)&strM2MAPConfig.au8SSID, MAIN_WLAN_SSID);
  		  strM2MAPConfig.u8ListenChannel = MAIN_WLAN_CHANNEL;
  		  strM2MAPConfig.u8SecType = MAIN_WLAN_AUTH;

  		  strM2MAPConfig.au8DHCPServerIP[0] = 192;
  		  strM2MAPConfig.au8DHCPServerIP[1] = 168;
  		  strM2MAPConfig.au8DHCPServerIP[2] = 1;
  		  strM2MAPConfig.au8DHCPServerIP[3] = 1;

  	  	  	#if USE_WEP
  	  	  		strcpy((char *)&strM2MAPConfig.au8WepKey, MAIN_WLAN_WEP_KEY);
  	  	  		strM2MAPConfig.u8KeySz = strlen(MAIN_WLAN_WEP_KEY);
  	  	  		strM2MAPConfig.u8KeyIndx = MAIN_WLAN_WEP_KEY_INDEX;
  	  	  	#endif
  	  	  		/* Bring up AP mode with parameters structure. */
  	  	  		ret = m2m_wifi_enable_ap(&strM2MAPConfig);
  	  	  		if (M2M_SUCCESS != ret)
  	  	  		{
  	  	  			printf("main: m2m_wifi_enable_ap call error!\r\n");
  	  	  			while (1) {
  	  	  			}
  	  	  		}

  	  	  		printf("AP mode started. You can connect to %s.\r\n", (char *)MAIN_WLAN_SSID);
  	  	  	  while(1)
  	  	  	  {
  	  	  		  m2m_wifi_handle_events(NULL);

  	  	  	  }
  	  	  	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */


/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	while(1)
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
	while(1)
	{

	}
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

