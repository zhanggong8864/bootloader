/**************************************************************************************************
 * 模块名称：boot模块
 * 文件名称：boot.c
 * 版   本：V1.0
 * 说   明：实现bootloader相关接口函数         
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
  __svc(1)是一个汇编指令，用于触发软件中断（Software Interrupt）。它允许应用程序通过软件方式请求操作系统或特权模式下的服务。
  具体而言，当执行到__svc(1)指令时，处理器会自动进入特权模式，并将控制流转移到预定义的异常处理程序。然后，操作系统可以根据传递
  给__svc(1)的参数来识别和执行相应的服务或功能。
*/
void __svc(1)   EnablePrivilegedMode(void);
void __SVC_1(void)
{
  DISABLE_ALL_INTERRUPT();
  __set_CONTROL((__get_CONTROL())& 0xFFFFFFFE);  // 进入特权模式
  ENABLE_ALL_INTERRUPT();
}
/**************************************************************************************************
                                        Static Function Declaration       
**************************************************************************************************/


/************************************************************************************
*@name   ： boot_UART_Write_Loop
*@brief  ： 接收来自主机APP更新通知信号
*@par    ： none
*@return ： uint8
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
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);//闪灯

    HAL_UART_Transmit(&huart3, (uint8_t *)&tx_dat, 1, HAL_MAX_DELAY);//2 给主机发送字符'g'

    temp = HAL_UART_Receive(&huart3, (uint8_t *)&rx_dat, 1, 10);//接收来自主机发送的字符

    if( (temp == HAL_OK ) && ( rx_dat == 'r' ) )//3 主机发送给从机r开始下一步
    {
      //接收到数据
      printf("Firmware Update Started\r\n");
      retval = 1;
      break;
    }

    if(count == 100)
    {
      //没有接收到数据
			/*问题：此代码要注释掉程序才能运行正常否则程序进入HardFault_Handler函数*/
      //printf("No Data Received for Firmware Update\r\n"); 
      break;
    }
		
    count ++;
    HAL_Delay(20);//延时20ms
  }

  return retval;
}

/************************************************************************************
*@name   ： boot_WriteDatToFlash
*@brief  ： 把接收的数据写入到flash
*@par    ： none
*@return ： none
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
*@name   ： boot_FirmwareUpdate
*@brief  ： 接收来自主机的应用程序数据并使用该函数将该数据写入闪存，huart3用于数据传输
*@par    ： none
*@return ： none
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
    if( boot_UartWriteLoop() != 0 )//条件为真则表明主机开始发送APP程序
    {
      //发送方准备发送APP程序

      // Ask yy
      HAL_UART_Transmit(&huart3, &tx_data1, 1, HAL_MAX_DELAY);//5 向主机发送字符'y'
      temp = HAL_UART_Receive(&huart3, &rx_buf1, 1, 5000);//6 接收主机发送的字符
      if( temp != HAL_OK )
      {
        printf("Get application Size error (rx_huf1)...HALT!!!\r\n");
        break;
      }

      // Ask xx
      HAL_UART_Transmit(&huart3, &tx_data2, 1, HAL_MAX_DELAY);//7 向主机发送字符'x'
      temp = HAL_UART_Receive(&huart3, &rx_buf2, 1, 5000);//8 接收主机发送的字符
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
        HAL_UART_Transmit(&huart3, &tx_data1, 1, HAL_MAX_DELAY);//9 向主机发送字符'y'
        temp = HAL_UART_Receive(&huart3, &rx_buf1, 1, 5000);//10 接收主机发送的字符
        if( temp != HAL_OK )
        {
          printf("Get application data[index:%d] error (rx_buf1)...HALT!!!\r\n", i);
          break;
        }

        // Ask xx
        HAL_UART_Transmit(&huart3, &tx_data2, 1, HAL_MAX_DELAY);//11 向主机发送字符'x'
        temp = HAL_UART_Receive(&huart3, &rx_buf2, 1, 5000);//12 接收主机发送的字符
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
    while(1);//接收失败
  }
}

/************************************************************************************
*@name   ： boot_JumpToAppFun
*@brief  ： boot跳转到app函数
*@par    ： none
*@return ： none
*************************************************************************************/
void boot_JumpToAppFun(void)
{
  printf("Gonna jump to Application!\n");
	
	/*1.要确保CPU在特权模式*/
	if (CONTROL_nPRIV_Msk & __get_CONTROL())//只要__get_CONTROL最后一位为0，则不是特权模式
  {  /*上面条件不成立，则让CPU进入特权模式*/
    EnablePrivilegedMode();
  }
	
	/*2.关闭所有中断*/
	DISABLE_ALL_INTERRUPT();
	
	/*3. 32*8=256个中断使能位，把ARM Cortex-M中的256个中断全部失能 */
	NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0xFFFFFFFF;
  NVIC->ICER[2] = 0xFFFFFFFF;
  NVIC->ICER[3] = 0xFFFFFFFF;
  NVIC->ICER[4] = 0xFFFFFFFF;
  NVIC->ICER[5] = 0xFFFFFFFF;
  NVIC->ICER[6] = 0xFFFFFFFF;
  NVIC->ICER[7] = 0xFFFFFFFF;
	
	/*4.禁用所有启用的可能产生中断请求的外设，把上面对应的所有中断标志位清零*/
	NVIC->ICPR[0] = 0xFFFFFFFF;
  NVIC->ICPR[1] = 0xFFFFFFFF;
  NVIC->ICPR[2] = 0xFFFFFFFF;
  NVIC->ICPR[3] = 0xFFFFFFFF;
  NVIC->ICPR[4] = 0xFFFFFFFF;
  NVIC->ICPR[5] = 0xFFFFFFFF;
  NVIC->ICPR[6] = 0xFFFFFFFF;
  NVIC->ICPR[7] = 0xFFFFFFFF;

	/* 5. 禁用 SysTick 并清除其异常挂起位（如果它在引导加载程序中使用，例如由 RTX 使用*/
  SysTick->CTRL = 0;
  SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;
	
	 /* 6. 如果引导加载程序使用了各个故障处理程序，则禁用它们。*/
  SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);
	
	 /* 7. 如果发现核心当前与PSP一起运行，则激活 MSP。*/
  if( CONTROL_SPSEL_Msk & __get_CONTROL())
  { /* 进入这里则表示MSP未激活，则进行激活操作 */
    __set_CONTROL(__get_CONTROL() & ~CONTROL_SPSEL_Msk);
  }
	
	/*
    8. 将用户应用程序的向量表地址加载到SCB->VTOR寄存器中确保地址符合寄存器的对齐要求。
    指定中断向量表位置以确保处理器能正确地找到中断处理函数。
  */
  SCB->VTOR = (uint32_t)0x8004400;
	/*这是一个指向用户应用程序的重置向量的指针。在ARM Cortex-M微控制器中，
	重置向量是存储在向量表的第二个位置（所以这里加了4）的，它包含了用户应用程序的入口点（即程序开始执行的地方）*/
	void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*)(APP_START_ADDRESS + 4u)));
  if(app_reset_handler == (void*)0xFFFFFFFF)
	{
	  printf("Invalid Application... HALT!\r\n");
	  while(1);//跳转地址错误程序停在这里
	}
	
	//9.设置主堆栈指针地址，主堆栈指针用于指示当前正在执行的代码位置,将MSP设置为在用户应用程序向量表中找到的值
  __set_MSP(*(volatile uint32_t*)APP_START_ADDRESS);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);//关灯
	
	/* 10. 通过函数调用将PC设置为用户应用程序的复位向量值。*/
  app_reset_handler();//调用APP复位句柄
}
