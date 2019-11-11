/**
  ******************************************************************************
  * @file    Wifi/WiFi_Client_Server/src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license SLA0044,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        http://www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "math.h"
#include "stdio.h"


/* Private defines -----------------------------------------------------------*/

#define TERMINAL_USE

/* Update SSID and PASSWORD with own Access point settings */
#define SSID     "NETGEAR87"
#define PASSWORD "quickviolet413"

uint8_t RemoteIP[] = {192,168,1,4};
#define RemotePORT	8002

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000

#define CONNECTION_TRIAL_MAX          10

#if defined (TERMINAL_USE)
#define TERMOUT(...)  printf(__VA_ARGS__)
#else
#define TERMOUT(...)
#endif

/* Private variables ---------------------------------------------------------*/
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */
static uint8_t RxData [500];


/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small TERMOUT (option LD Linker->Libraries->Small TERMOUT
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);



extern  SPI_HandleTypeDef hspi;

/* Private functions ---------------------------------------------------------*/
void getAllValues(void)
{
		/*Initalize Temp Sensor */
	int ret = 0;
	
	if (HSENSOR_OK != BSP_HSENSOR_Init())
  {
    TERMOUT("BSP_HSENSOR_Init() returns %d\n\r", ret);
    ret = -1;
  }
  
  if (TSENSOR_OK != BSP_TSENSOR_Init())
  {
    TERMOUT("BSP_TSENSOR_Init() returns %d\n\r", ret);
    ret = -1;
  }
  
  if (PSENSOR_OK != BSP_PSENSOR_Init())
  {
    TERMOUT("BSP_PSENSOR_Init() returns %d\n\r", ret);
    ret = -1;
  }
	
	/* Get Values */ 
	float TEMPERATURE_Value, HUMIDITY_Value, PRESSURE_Value;
	
	TEMPERATURE_Value = BSP_TSENSOR_ReadTemp();
	HUMIDITY_Value = BSP_HSENSOR_ReadHumidity();
	PRESSURE_Value = BSP_PSENSOR_ReadPressure();
	

	TERMOUT("> Temperature %.2f\n\r", TEMPERATURE_Value);
	TERMOUT("> Humidity %.2f\n\r", HUMIDITY_Value);
	TERMOUT("> Pressure %.2f\n\r", PRESSURE_Value);
}

//uint8_t getTemp(uint8_t TxData)
//{
//		/*Initalize Temp Sensor */
//	int ret = 0;
//  
//  if (TSENSOR_OK != BSP_TSENSOR_Init())
//  {
//    TERMOUT("BSP_TSENSOR_Init() returns %d\n\r", ret);
//    ret = -1;
//  }
//	
//	/* Get Values */ 
//	uint8_t TEMPERATURE_Value;
//	
//	TEMPERATURE_Value = BSP_TSENSOR_ReadTemp();

//	TxData = TEMPERATURE_Value;
//	
//	return TxData;

//}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t  MAC_Addr[6];
  uint8_t  IP_Addr[4];
  uint8_t TxData[] = "STM32 : Hello!\n\r";
  int32_t Socket = -1;
  uint16_t Datalen;
  int32_t ret;
  int16_t Trials = CONNECTION_TRIAL_MAX;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* Configure LED2 */
  BSP_LED_Init(LED2);

#if defined (TERMINAL_USE)
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  BSP_COM_Init(COM1, &hDiscoUart);
#endif /* TERMINAL_USE */

  TERMOUT("****** WIFI Module in TCP Client mode demonstration ****** \n\n\r");
  TERMOUT("TCP Client Instructions :\n\r");
  TERMOUT("1- Make sure your Phone is connected to the same network that\n\r");
  TERMOUT("   you configured using the Configuration Access Point.\n\r");
  TERMOUT("2- Create a server by using the android application TCP Server\n\r");
  TERMOUT("   with port(8002).\n\r");
  TERMOUT("3- Get the Network Name or IP Address of your Android from the step 2.\n\n\r");
	
	//getValues();


  /*Initialize  WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    TERMOUT("> WIFI Module Initialized.\n\r");
    if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
    {
      TERMOUT("> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\n\r",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]);
    }
    else
    {
      TERMOUT("> ERROR : CANNOT get MAC address\n\r");
      BSP_LED_On(LED2);
    }

    if( WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
    {
      TERMOUT("> es-wifi module connected \n\r");
      if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
      {
        TERMOUT("> es-wifi module got IP Address : %d.%d.%d.%d\n\r",
               IP_Addr[0],
               IP_Addr[1],
               IP_Addr[2],
               IP_Addr[3]);

        TERMOUT("> Trying to connect to Server: %d.%d.%d.%d:%d ...\n\r",
               RemoteIP[0],
               RemoteIP[1],
               RemoteIP[2],
               RemoteIP[3],
							 RemotePORT);

        while (Trials--)
        {
          if( WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
          {
            TERMOUT("> TCP Connection opened successfully.\n\r");
            Socket = 0;
            break;
          }
        }
        if(Socket == -1)
        {
          TERMOUT("> ERROR : Cannot open Connection\n\r");
          BSP_LED_On(LED2);
        }
      }
      else
      {
        TERMOUT("> ERROR : es-wifi module CANNOT get IP address\n\r");
        BSP_LED_On(LED2);
      }
    }
    else
    {
      TERMOUT("> ERROR : es-wifi module NOT connected\n\r");
      BSP_LED_On(LED2);
    }
  }
  else
  {
    TERMOUT("> ERROR : WIFI Module cannot be initialized.\n\r");
    BSP_LED_On(LED2);
  }

  while(1)
  {
    if(Socket != -1)
    {
      ret = WIFI_ReceiveData(Socket, RxData, sizeof(RxData)-1, &Datalen, WIFI_READ_TIMEOUT);
      if(ret == WIFI_STATUS_OK)
      {
        if(Datalen > 0)
        {
          RxData[Datalen]=0;
          TERMOUT("Received: %s\n",RxData);
					
					// Temperature
					int ret = 0;
					
					if (HSENSOR_OK != BSP_HSENSOR_Init())
					{
						TERMOUT("BSP_HSENSOR_Init() returns %d\n\r", ret);
						ret = -1;
					}
					
					if (TSENSOR_OK != BSP_TSENSOR_Init())
					{
						TERMOUT("BSP_TSENSOR_Init() returns %d\n\r", ret);
						ret = -1;
					}
					
					if (PSENSOR_OK != BSP_PSENSOR_Init())
					{
						TERMOUT("BSP_PSENSOR_Init() returns %d\n\r", ret);
						ret = -1;
					}
					
					int resultT = 0, resultH = 0, resultP = 0;
					char TempValue[6], HumValue[6], PressValue[6];
					
					//Get temp value	
					float temp = BSP_TSENSOR_ReadTemp();
					resultT = snprintf(TempValue, 6, "%f", temp);
		
					//Get humidity value	
					float hum = BSP_HSENSOR_ReadHumidity();
					resultH = snprintf(HumValue, 6, "%f", hum);
					
					//Get humidity value	
					float press = BSP_PSENSOR_ReadPressure();
					resultP = snprintf(PressValue, 6, "%f", press);
		
					
					//Return Temp
					ret = WIFI_SendData(Socket, (uint8_t*)"Temperature: ", 14, &Datalen, WIFI_WRITE_TIMEOUT);
					ret = WIFI_SendData(Socket, (uint8_t*)TempValue, sizeof(TempValue), &Datalen, WIFI_WRITE_TIMEOUT);
          ret = WIFI_SendData(Socket, (uint8_t*)" degrees C \n", 12, &Datalen, WIFI_WRITE_TIMEOUT);
					
					//Return Humidity
					ret = WIFI_SendData(Socket, (uint8_t*)"Humidity: ", 10, &Datalen, WIFI_WRITE_TIMEOUT);
					ret = WIFI_SendData(Socket, (uint8_t*)HumValue, sizeof(HumValue), &Datalen, WIFI_WRITE_TIMEOUT);
          ret = WIFI_SendData(Socket, (uint8_t*)" % \n", 5, &Datalen, WIFI_WRITE_TIMEOUT);
					
					//Return Pressure
					ret = WIFI_SendData(Socket, (uint8_t*)"Pressure: ", 10, &Datalen, WIFI_WRITE_TIMEOUT);
					ret = WIFI_SendData(Socket, (uint8_t*)PressValue, sizeof(PressValue), &Datalen, WIFI_WRITE_TIMEOUT);
          ret = WIFI_SendData(Socket, (uint8_t*)" mBar \n", 7, &Datalen, WIFI_WRITE_TIMEOUT);
					
					//Extra Newline
					ret = WIFI_SendData(Socket, (uint8_t*)" \n", 3, &Datalen, WIFI_WRITE_TIMEOUT);
					
          if (ret != WIFI_STATUS_OK)
          {
            TERMOUT("> ERROR : Failed to Send Data, connection closed\n\r");
            break;
          }
        }
      }
      else
      {
        TERMOUT("> ERROR : Failed to Receive Data, connection closed\n\r");
        break;
      }
    }
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library TERMOUT function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif /* TERMINAL_USE */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: TERMOUT("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
    default:
    {
      break;
    }
  }
}

void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
