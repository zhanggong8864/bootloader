/**************************************************************************************************
 * ģ�����ƣ�bootģ��
 * �ļ����ƣ�boot.c
 * ��   ����V1.0
 * ˵   ����ʵ��bootloader��ؽӿں���         
 *************************************************************************************************/

/**************Own headfile*************/
#include "boot.h"
/*************Other headfile************/
#include "boot_cfg.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
/**************************************************************************************************
                                           Macro Definition       
**************************************************************************************************/

/**************************************************************************************************
                                            Gobal Variable       
**************************************************************************************************/
uint8_t BL_VERSION[2];
/**************************************************************************************************
                                            Local Variable       
**************************************************************************************************/

/**************************************************************************************************
                                            Static Variable       
**************************************************************************************************/
static uint16_t application_write_idx = 0;
static uint16_t application_size = 0;

/* 
  __svc(1)��һ�����ָ����ڴ�������жϣ�Software Interrupt����������Ӧ�ó���ͨ�������ʽ�������ϵͳ����Ȩģʽ�µķ���
  ������ԣ���ִ�е�__svc(1)ָ��ʱ�����������Զ�������Ȩģʽ������������ת�Ƶ�Ԥ������쳣�������Ȼ�󣬲���ϵͳ���Ը��ݴ���
  ��__svc(1)�Ĳ�����ʶ���ִ����Ӧ�ķ�����ܡ�
*/
void __svc(1)   EnablePrivilegedMode(void);
void __SVC_1(void)
{
  DISABLE_ALL_INTERRUPT();
  __set_CONTROL((__get_CONTROL())& 0xFFFFFFFE);  // ������Ȩģʽ
  ENABLE_ALL_INTERRUPT();
}
/**************************************************************************************************
                                        Static Function Declaration       
**************************************************************************************************/


/************************************************************************************
*@name   �� boot_UART_Write_Loop
*@brief  �� ������������APP����֪ͨ�ź�
*@par    �� none
*@return �� uint8
*************************************************************************************/
static uint8_t boot_UartWriteLoop(void)
{
  char tx_dat = 'g';
  char rx_dat = '0';
  uint8_t retval = 0;
  uint8_t count = 0;
  HAL_StatusTypeDef temp;

  while(1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);//����

    HAL_UART_Transmit(&huart3, (uint8_t *)&tx_dat, 1, HAL_MAX_DELAY);//2 �����������ַ�'g'

    temp = HAL_UART_Receive(&huart3, (uint8_t *)&rx_dat, 1, 10);//���������������͵��ַ�

    if( (temp == HAL_OK ) && ( rx_dat == 'r' ) )//3 �������͸��ӻ�r��ʼ��һ��
    {
      //���յ�����
      printf("Firmware Update Started\r\n");
      retval = 1;
      break;
    }

    if(count == 100)
    {
      //û�н��յ�����
			/*���⣺�˴���Ҫע�͵��������������������������HardFault_Handler����*/
      //printf("No Data Received for Firmware Update\r\n"); 
      break;
    }
		
    count ++;
    HAL_Delay(20);//��ʱ20ms
  }

  return retval;
}

/************************************************************************************
*@name   �� boot_WriteDatToFlash
*@brief  �� �ѽ��յ�����д�뵽flash
*@par    �� none
*@return �� none
*************************************************************************************/
static HAL_StatusTypeDef boot_WriteDatToFlash(uint8_t *data, uint16_t data_len, uint8_t is_first_block)
{
  HAL_StatusTypeDef ret;

  do
  {
    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )
    {
      break;
    }

    //No need to erase every time. Erase only the first time.
    if( is_first_block )
    {
      printf("Erasing the Flash memory...\r\n");
      //Erase the Flash
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError;

      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.PageAddress   = APP_START_ADDRESS;
      EraseInitStruct.NbPages       = 47;                     //47 Pages

      ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
      if( ret != HAL_OK )
      {
        break;
      }
      application_write_idx = 0;
    }

    for(int i = 0; i < data_len/2; i++)
    {
      uint16_t halfword_data = data[i * 2] | (data[i * 2 + 1] << 8);
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD,(APP_START_ADDRESS + application_write_idx ),halfword_data);
      if( ret == HAL_OK )
      {
        //update the data count
        application_write_idx += 2;
      }
      else
      {
        printf("Flash Write Error...HALT!!!\r\n");
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )
    {
      break;
    }
  }while(0);

  return ret;
}

/************************************************************************************
*@name   �� boot_FirmwareUpdate
*@brief  �� ��������������Ӧ�ó������ݲ�ʹ�øú�����������д�����棬huart3�������ݴ���
*@par    �� none
*@return �� none
*************************************************************************************/
void boot_FirmwareUpdate(void)
{
  uint8_t rx_buf1,rx_buf2;
  uint8_t tx_data1 = 'y';
  uint8_t tx_data2 = 'x';
  uint16_t current_app_size = 0;
  uint16_t i = 0;
	HAL_StatusTypeDef temp = HAL_OK;

  uint8_t block[BOOT_FLASH_PAGE_SIZE] = {0};

  do
  {
    if( boot_UartWriteLoop() != 0 )//����Ϊ�������������ʼ����APP����
    {
      //���ͷ�׼������APP����

      // Ask yy
      HAL_UART_Transmit(&huart3, &tx_data1, 1, HAL_MAX_DELAY);//5 �����������ַ�'y'
      temp = HAL_UART_Receive(&huart3, &rx_buf1, 1, 5000);//6 �����������͵��ַ�
      if( temp != HAL_OK )
      {
        printf("Get application Size error (rx_huf1)...HALT!!!\r\n");
        break;
      }

      // Ask xx
      HAL_UART_Transmit(&huart3, &tx_data2, 1, HAL_MAX_DELAY);//7 �����������ַ�'x'
      temp = HAL_UART_Receive(&huart3, &rx_buf2, 1, 5000);//8 �����������͵��ַ�
      if( temp != HAL_OK )
      {
        printf("Get application Size error(rx_buf2)...HALT!!!\r\n");
        break;
      }

      application_size = rx_buf1 | (rx_buf2 << 8);
      printf("Application Size = %d bytes\r\n", application_size);

      while(1)
      {
        if( ( i == BOOT_FLASH_PAGE_SIZE ) || ( current_app_size >= application_size) )
        {
          printf("Received Block[%d]\r\n", current_app_size/BOOT_FLASH_PAGE_SIZE);

          //write to flash
          temp = boot_WriteDatToFlash(block, BOOT_FLASH_PAGE_SIZE, (current_app_size <= BOOT_FLASH_PAGE_SIZE) );//14

          if(temp != HAL_OK)
          {
            break;
          }

          //clear the memory
          memset(block, 0, BOOT_FLASH_PAGE_SIZE);
          i = 0;
        }

        if( current_app_size >= application_size)//15
        {
          //received all data. exit
          temp = HAL_OK;
          break;
        }

        // Ask yy
        HAL_UART_Transmit(&huart3, &tx_data1, 1, HAL_MAX_DELAY);//9 �����������ַ�'y'
        temp = HAL_UART_Receive(&huart3, &rx_buf1, 1, 5000);//10 �����������͵��ַ�
        if( temp != HAL_OK )
        {
          printf("Get application data[index:%d] error (rx_buf1)...HALT!!!\r\n", i);
          break;
        }

        // Ask xx
        HAL_UART_Transmit(&huart3, &tx_data2, 1, HAL_MAX_DELAY);//11 �����������ַ�'x'
        temp = HAL_UART_Receive(&huart3, &rx_buf2, 1, 5000);//12 �����������͵��ַ�
        if( temp != HAL_OK )
        {
          printf("Get application data[index:%d] error(rx_buf2)...HALT!!!\r\n", i);
          break;
        }

        //--- Save xxyy in block[i]
        block[i++] = rx_buf1;
        block[i++] = rx_buf2;
        current_app_size += 2;
      }
    }
  }
  while(0);

  if( temp != HAL_OK )
  {
    printf("Receive App fail !!!\r\n");
    while(1);//����ʧ��
  }
}

/************************************************************************************
*@name   �� boot_JumpToAppFun
*@brief  �� boot��ת��app����
*@par    �� none
*@return �� none
*************************************************************************************/
void boot_JumpToAppFun(void)
{
  printf("Gonna jump to Application!\n");
	
	/*1.Ҫȷ��CPU����Ȩģʽ*/
	if (CONTROL_nPRIV_Msk & __get_CONTROL())//ֻҪ__get_CONTROL���һλΪ0��������Ȩģʽ
  {  /*��������������������CPU������Ȩģʽ*/
    EnablePrivilegedMode();
  }
	
	/*2.�ر������ж�*/
	DISABLE_ALL_INTERRUPT();
	
	/*3. 32*8=256���ж�ʹ��λ����ARM Cortex-M�е�256���ж�ȫ��ʧ�� */
	NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0xFFFFFFFF;
  NVIC->ICER[2] = 0xFFFFFFFF;
  NVIC->ICER[3] = 0xFFFFFFFF;
  NVIC->ICER[4] = 0xFFFFFFFF;
  NVIC->ICER[5] = 0xFFFFFFFF;
  NVIC->ICER[6] = 0xFFFFFFFF;
  NVIC->ICER[7] = 0xFFFFFFFF;
	
	/*4.�����������õĿ��ܲ����ж���������裬�������Ӧ�������жϱ�־λ����*/
	NVIC->ICPR[0] = 0xFFFFFFFF;
  NVIC->ICPR[1] = 0xFFFFFFFF;
  NVIC->ICPR[2] = 0xFFFFFFFF;
  NVIC->ICPR[3] = 0xFFFFFFFF;
  NVIC->ICPR[4] = 0xFFFFFFFF;
  NVIC->ICPR[5] = 0xFFFFFFFF;
  NVIC->ICPR[6] = 0xFFFFFFFF;
  NVIC->ICPR[7] = 0xFFFFFFFF;

	/* 5. ���� SysTick ��������쳣����λ����������������س�����ʹ�ã������� RTX ʹ��*/
  SysTick->CTRL = 0;
  SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;
	
	 /* 6. ����������س���ʹ���˸������ϴ��������������ǡ�*/
  SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);
	
	 /* 7. ������ֺ��ĵ�ǰ��PSPһ�����У��򼤻� MSP��*/
  if( CONTROL_SPSEL_Msk & __get_CONTROL())
  { /* �����������ʾMSPδ�������м������ */
    __set_CONTROL(__get_CONTROL() & ~CONTROL_SPSEL_Msk);
  }
	
	/*
    8. ���û�Ӧ�ó�����������ַ���ص�SCB->VTOR�Ĵ�����ȷ����ַ���ϼĴ����Ķ���Ҫ��
    ָ���ж�������λ����ȷ������������ȷ���ҵ��жϴ�������
  */
  SCB->VTOR = (uint32_t)0x8004400;
	/*����һ��ָ���û�Ӧ�ó��������������ָ�롣��ARM Cortex-M΢�������У�
	���������Ǵ洢��������ĵڶ���λ�ã������������4���ģ����������û�Ӧ�ó������ڵ㣨������ʼִ�еĵط���*/
	void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*)(APP_START_ADDRESS + 4u)));
  if(app_reset_handler == (void*)0xFFFFFFFF)
	{
	  printf("Invalid Application... HALT!\r\n");
	  while(1);//��ת��ַ�������ͣ������
	}
	
	//9.��������ջָ���ַ������ջָ������ָʾ��ǰ����ִ�еĴ���λ��,��MSP����Ϊ���û�Ӧ�ó������������ҵ���ֵ
  __set_MSP(*(volatile uint32_t*)APP_START_ADDRESS);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);//�ص�
	
	/* 10. ͨ���������ý�PC����Ϊ�û�Ӧ�ó���ĸ�λ����ֵ��*/
  app_reset_handler();//����APP��λ���
}
