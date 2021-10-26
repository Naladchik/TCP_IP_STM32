/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdbool.h>
#include "wizchip_conf.h"
#include "socket.h"
#include "dhcp.h"
#include "dns.h"
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2

void SWO_Printf(const char* msg) {
	for(uint8_t i =0; i < 255; i++){
		ITM_SendChar(msg[i]);
		if(msg[i] == '\n') break;
	}
}

void SWO_Printn(const uint8_t num) {
	uint8_t buf, dig_n, nn[3];
	buf = num;
	dig_n = 0;
	
	if(num == 0){
		ITM_SendChar(num + 0x30);
	}else{
		while(buf > 0){
			buf /= 10;
			dig_n++;
		}
		buf = num;
		for(uint8_t i = 0; i < dig_n; i++){
			nn[i] = buf % 10;
			buf /= 10;
		}
		for(uint8_t i = dig_n; i > 0; i--){
			ITM_SendChar(nn[i - 1] + 0x30);
		}
	}
	SWO_Printf("  ");
}



void W5500_Select(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}

volatile bool ip_assigned = false;

void Callback_IPAssigned(void) {
    SWO_Printf("Callback: IP assigned! Leased time: %d sec\r\n");
    ip_assigned = true;
}
 
void Callback_IPConflict(void) {
    SWO_Printf("Callback: IP conflict!\r\n");
}

// 1K should be enough, see https://forum.wiznet.io/t/topic/1612/2
uint8_t dhcp_buffer[1024];
// 1K seems to be enough for this buffer as well
uint8_t dns_buffer[1024];

void init() {
    SWO_Printf("\r\ninit() called!\r\n");

    SWO_Printf("Registering W5500 callbacks...\r\n");
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

    SWO_Printf("Calling wizchip_init()...\r\n");
    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

    SWO_Printf("Calling DHCP_init()...\r\n");
    wiz_NetInfo net_info = {
        .mac  = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA },
        .dhcp = NETINFO_DHCP
    };
    // set MAC address before using DHCP
    setSHAR(net_info.mac);
    DHCP_init(DHCP_SOCKET, dhcp_buffer);

    SWO_Printf("Registering DHCP callbacks...\r\n");
    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );

    SWO_Printf("Calling DHCP_run()...\r\n");
    // actually should be called in a loop, e.g. by timer
    uint32_t ctr = 10000;
    while((!ip_assigned) && (ctr > 0)) {
        DHCP_run();
        ctr--;
    }
    if(!ip_assigned) {
        SWO_Printf("\r\nIP was not assigned :(\r\n");
        return;
    }

    getIPfromDHCP(net_info.ip);
    getGWfromDHCP(net_info.gw);
    getSNfromDHCP(net_info.sn);

    uint8_t dns[4];
    getDNSfromDHCP(dns);

    SWO_Printf("IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\nDNS: %d.%d.%d.%d\r\n");
		SWO_Printn(net_info.ip[0]); SWO_Printn(net_info.ip[1]); SWO_Printn(net_info.ip[2]); SWO_Printn(net_info.ip[3]); SWO_Printf("\n");
    SWO_Printn(net_info.gw[0]); SWO_Printn(net_info.gw[1]); SWO_Printn(net_info.gw[2]); SWO_Printn(net_info.gw[3]); SWO_Printf("\n");
    SWO_Printn(net_info.sn[0]); SWO_Printn(net_info.sn[1]); SWO_Printn(net_info.sn[2]); SWO_Printn(net_info.sn[3]); SWO_Printf("\n");
    SWO_Printn(dns[0]); 				SWO_Printn(dns[1]); 				SWO_Printn(dns[2]); 				SWO_Printn(dns[3]); 				SWO_Printf("\n");

    SWO_Printf("Calling wizchip_setnetinfo()...\r\n");
    wizchip_setnetinfo(&net_info);

    SWO_Printf("Calling DNS_init()...\r\n");
    DNS_init(DNS_SOCKET, dns_buffer);

    uint8_t addr[4];
    {
        char domain_name[] = "eax.me";
        SWO_Printf("Resolving domain name \"%s\"...\r\n");
        int8_t res = DNS_run(dns, (uint8_t*)&domain_name, addr);
        if(res != 1) {
            SWO_Printf("DNS_run() failed, res = %d");
            return;
        }
        SWO_Printf("Result: %d.%d.%d.%d\r\n");
    }

    SWO_Printf("Creating socket...\r\n");
    uint8_t http_socket = HTTP_SOCKET;
    uint8_t code = socket(http_socket, Sn_MR_TCP, 10888, 0);
    if(code != http_socket) {
        SWO_Printf("socket() failed, code = %d\r\n");
        return;
    }

    SWO_Printf("Socket created, connecting...\r\n");
    code = connect(http_socket, addr, 80);
    if(code != SOCK_OK) {
        SWO_Printf("connect() failed, code = %d\r\n");
        close(http_socket);
        return;
    }

    SWO_Printf("Connected, sending HTTP request...\r\n");
    {
        char req[] = "GET / HTTP/1.0\r\nHost: eax.me\r\n\r\n";
        uint16_t len = sizeof(req) - 1;
        uint8_t* buff = (uint8_t*)&req;
        while(len > 0) {
            SWO_Printf("Sending %d bytes...\r\n");
            int32_t nbytes = send(http_socket, buff, len);
            if(nbytes <= 0) {
                SWO_Printf("send() failed, %d returned\r\n");
                close(http_socket);
                return;
            }
            SWO_Printf("%d bytes sent!\r\n");
            len -= nbytes;
        }
    }

    SWO_Printf("Request sent. Reading response...\r\n");
    {
        char buff[32];
        for(;;) {
            int32_t nbytes = recv(http_socket, (uint8_t*)&buff, sizeof(buff)-1);
            if(nbytes == SOCKERR_SOCKSTATUS) {
                SWO_Printf("\r\nConnection closed.\r\n");
                break;
            }

            if(nbytes <= 0) {
                SWO_Printf("\r\nrecv() failed, %d returned\r\n");
                break;
            }

            buff[nbytes] = '\0';
            SWO_Printf("%s");
        }
    }

    SWO_Printf("Closing socket.\r\n");
    close(http_socket);
}

void loop() {
    HAL_Delay(1000);
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
  /* USER CODE BEGIN 2 */
	//init();
	HAL_GPIO_WritePin(PORT_LED0, LED0, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(PORT_LED0, LED0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_LED1, LED1, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(PORT_LED1, LED1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_LED2, LED2, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(PORT_LED2, LED2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_WIZ_RS, WIZ_RS, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(PORT_WIZ_RS, WIZ_RS, GPIO_PIN_RESET);
	HAL_Delay(500);
	
	uint8_t tx_buff[SPI_FRAME_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx_buff[SPI_FRAME_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t* p  = tx_buff;
	
	
	while(1){
	p  = tx_buff;
	*p++ = 0x00;	*p++ = 0x2e;	//address phase
	*p++ = 0x00 | 0x00 | 0x01; //control phase (1 bytes)
	*p++ = 0x00;//data phase
	HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff ,4, SPI_TIME_OUT);
		if((rx_buff[3] & 0x01) != 0) break;
		HAL_Delay(100);
	}
	HAL_GPIO_WritePin(PORT_LED0, LED0, GPIO_PIN_SET);

	
	//set Gateway address
	p  = tx_buff;
	*p++ = 0x00;	*p++ = 0x01;	//address phase
	*p++ = 0x00 | 0x04 | 0x03; //control phase (4 bytes)
	*p++ = 0x01; *p++ = 0x01; *p++ = 0x01;  *p++ = 0x01;  //data phase
	HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff ,SPI_FRAME_SIZE, SPI_TIME_OUT);
	
	//set Subnet mask
	p  = tx_buff;
	*p++ = 0x00;	*p++ = 0x05;	//address phase
	*p++ = 0x00 | 0x04 | 0x03; //control phase (4 bytes)
	*p++ = 0xff; *p++ = 0xff; *p++ = 0xff;  *p++ = 0x00;  //data phase
	HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff , SPI_FRAME_SIZE, SPI_TIME_OUT);
	
	//set MAC
	p  = tx_buff;
	*p++ = 0x00;	*p++ = 0x09;	//address phase
	*p++ = 0x00 | 0x04 | 0x03; //control phase (4 bytes)
	*p++ = 0xa4; *p++ = 0x03; *p++ = 0x26;  *p++ = 0xa7;  //data phase
	HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff , SPI_FRAME_SIZE, SPI_TIME_OUT);
	p  = tx_buff;
	*p++ = 0x00;	*p++ = 0x0d;	//address phase
	*p++ = 0x00 | 0x04 | 0x02; //control phase (2 bytes)
	*p++ = 0x44; *p++ = 0x01;  //data phase
	HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff , 5, SPI_TIME_OUT);
	
	//set IP
	p  = tx_buff;
	*p++ = 0x00;	*p++ = 0x0f;	//address phase
	*p++ = 0x00 | 0x04 | 0x03; //control phase (4 bytes)
	*p++ = 192; *p++ = 168; *p++ = 100;  *p++ = 51;  //data phase
	HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff ,SPI_FRAME_SIZE, SPI_TIME_OUT);
	
	//read IP
	p  = tx_buff;
	*p++ = 0x00;	*p++ = 0x0f;	//address phase
	*p++ = 0x00 | 0x00 | 0x03; //control phase (4 bytes)
	*p++ = 0x00; *p++ = 0x00; *p++ = 0x000;  *p++ = 0x00;  //data phase
	HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff ,SPI_FRAME_SIZE, SPI_TIME_OUT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		p  = tx_buff;
		*p++ = 0x00;	*p++ = 0x2e;	//address phase
		*p++ = 0x00 | 0x00 | 0x01; //control phase (1 bytes)
		*p++ = 0x00;//data phase
		HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff ,4, SPI_TIME_OUT);
		if((rx_buff[3] & 0x01) != 0) 
			HAL_GPIO_WritePin(PORT_LED0, LED0, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(PORT_LED0, LED0, GPIO_PIN_RESET);
		HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
