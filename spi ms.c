#include<stdint.h>
#include "stm32f4xx.h"
#include "hal_spi_driver.h"
#include "hal_gpio_driver.h"
#include "spi_main.h"
#include "led.h"
#include "uart_debug.h"

spi_handle_t SpiHandle;

int TestReady = 0;


uint8_t slave_reply_data[4]={ 0x55, 0xaa, 0x55, 0xaa};

uint8_t master_write_data[]={ 0xa, 0xb, 0xc, 0xd };

uint8_t master_read_buffer[4];

void spi_gpio_init(void)
{
	gpio_pin_conf_t spi_conf;
	
	
	_HAL_RCC_GPIOB_CLK_ENABLE();
	
	spi_conf.pin = SPI_CLK_PIN;
	spi_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	spi_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	spi_conf.pull = GPIO_PIN_PULL_DOWN;
	spi_conf.speed = GPIO_PIN_SPEED_MEDIUM;
	
	hal_gpio_set_alt_function(GPIOB,SPI_CLK_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	spi_conf.pin = SPI_MISO_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB,SPI_MISO_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	spi_conf.pin = SPI_MOSI_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB,SPI_MOSI_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
}

void delay_gen( )
{
	uint32_t cnt = 800000;
	while(cnt--);
}

	
void assert_error(void)
{
	while(1)
	{
	  led_toggle(GPIOD,LED_RED);
		delay_gen();
	}
}

static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }
	return 0;
}
	
int main(void)
{
	uint32_t i=0;
	uint8_t addrcmd[CMD_LENGTH];
	uint8_t ack_buf[2];
	
	spi_gpio_init();
	
	led_init();
	
	_HAL_RCC_GPIOA_CLK_ENABLE();
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE);
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);
	
	_HAL_RCC_SPI2_CLK_ENABLE() ;
	
	SpiHandle.Instance               = SPI_2;
	SpiHandle.Init.BaudRatePrescaler = SPI_REG_CR1_BR_PCLK_DIV_32;
	SpiHandle.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.CLKPhase          = SPI_SECOND_CLOCK_TRANS;
	SpiHandle.Init.CLKPolarity       = SPI_CPOL_LOW;
	SpiHandle.Init.DataSize          = SPI_8BIT_DF_ENABLE;
	SpiHandle.Init.FirstBit          = SPI_TX_MSB_FIRST;
	SpiHandle.Init.NSS               = SPI_SSM_ENABLE;
	SpiHandle.Init.Mode              = SPI_MASTER_MODE_SEL;
	
	SpiHandle.State = HAL_SPI_STATE_READY;
	
	
	hal_spi_init(&SpiHandle);
	
  NVIC_EnableIRQ(SPI2_IRQn);

	hal_debug_uart_init( DEBUG_USART_BAUD_9600);
	
	uart_printf("SPI master Application Running ... \n");
	
	
  while(TestReady != SET)
  {
    led_toggle(GPIOD,LED_ORANGE);
		//LED3 (orange)
    delay_gen();
  }
	
	 led_turn_off(GPIOD,LED_ORANGE);
	

while(1)
{
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	addrcmd[0] = (uint8_t) CMD_MASTER_WRITE;
	addrcmd[1] = (uint8_t) ( CMD_MASTER_WRITE >> 8 );
	
	hal_spi_master_tx(&SpiHandle, addrcmd,CMD_LENGTH );
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	delay_gen();
	
	hal_spi_master_rx(&SpiHandle,ack_buf, ACK_LEN);
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	if(ack_buf[1] == 0XE5 && ack_buf[0] == 0xD5 )
	{
		led_toggle(GPIOD,LED_GREEN);
		memset(ack_buf,0,2);
	}
	else
	{
		assert_error();
		memset(ack_buf,0,2);
	}

	hal_spi_master_tx(&SpiHandle, master_write_data,DATA_LENGTH );
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	delay_gen();


	addrcmd[0] = (uint8_t) CMD_MASTER_READ;
	addrcmd[1] = (uint8_t) ( CMD_MASTER_READ >> 8 );
	
	hal_spi_master_tx(&SpiHandle, addrcmd,CMD_LENGTH );
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	delay_gen();
	
	hal_spi_master_rx(&SpiHandle,ack_buf, ACK_LEN);
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	if(ack_buf[1] == 0XE5 && ack_buf[0] == 0xD5 )
	{
		led_toggle(GPIOD,LED_GREEN);
		memset(ack_buf,0,2);
	}
	else
	{
		assert_error();
		memset(ack_buf,0,2);
	}
	
	hal_spi_master_rx(&SpiHandle,master_read_buffer, DATA_LENGTH);
	
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	if(Buffercmp(master_read_buffer,slave_reply_data,DATA_LENGTH))
	{
		// we didnt rcv what needs to be rcvd !!! Error !
		led_toggle(GPIOD,LED_RED);
	}else
	{
		//Rcvd correct data 
		led_toggle(GPIOD,LED_BLUE);
		for(i=0;i<DATA_LENGTH;i++)
		uart_printf("Data Received from slave: %x\n",master_read_buffer[i]);
	}
			
	delay_gen();
}

	return 0;
}



void SPI2_IRQHandler(void)
{
  hal_spi_irq_handler(&SpiHandle);
}


void EXTI0_IRQHandler(void)
{
  hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	TestReady = SET;
}


