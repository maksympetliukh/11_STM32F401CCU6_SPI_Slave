/*
 * 02_SPI_Slave_with_IRQ.c
 *
 *  Created on: Feb 25, 2026
 *      Author: maksym
 */

#include "stm32f401xx.h"
#include "gpio.h"
#include "spi.h"

void Delay(void){for(volatile uint32_t i = 0; i < 200000; i++);}

uint8_t tx = 0x10;
uint8_t rx = 0;

void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle, uint8_t event_flag){
	if(event_flag == SPI_EVENT_RX_CMPLT){
			if(rx == 0x01){
				SPI_Transmit_IT(pHandle, &tx, 1);
				SPI_Receive_IT(pHandle, &rx, 1);
			}else if(rx == 0xFF){
				GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
				Delay();
				GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

				SPI_Receive_IT(pHandle, &rx, 1);
			}
		}else if(event_flag == SPI_EVENT_TX_CMPLT){

		}
}

void SPI2_GPIO_Init(void){
	GPIO_Handle_t spi2_gpio;

	spi2_gpio.pGPIOx = GPIOB;

	GPIO_ClockControl(GPIOB, ENABLE);

	spi2_gpio.GPIOx_CFG.pin_mode = GPIO_MODE_ALT_FN;
	spi2_gpio.GPIOx_CFG.pin_alt_func_mode = 5;
	spi2_gpio.GPIOx_CFG.pin_op_type = GPIO_OUT_PP;
	spi2_gpio.GPIOx_CFG.pin_pu_pd_ctrl = GPIO_NO_PUPD;
	spi2_gpio.GPIOx_CFG.pin_speed = GPIO_OSPEED_HIGH;

	//SCLK
	spi2_gpio.GPIOx_CFG.pin_number = GPIO_PIN_13;
	GPIO_Init(&spi2_gpio);

	//MOSI
	spi2_gpio.GPIOx_CFG.pin_number = GPIO_PIN_15;
	GPIO_Init(&spi2_gpio);

	//MISO
	spi2_gpio.GPIOx_CFG.pin_number = GPIO_PIN_14;
	GPIO_Init(&spi2_gpio);

	//NSS
	spi2_gpio.GPIOx_CFG.pin_number = GPIO_PIN_12;
	GPIO_Init(&spi2_gpio);
}
SPI_Handle_t spi2;
void SPI2_IRQHandler(void){SPI_IRQ_Handler(&spi2);}

void SPI2_Init(void){

	spi2.pSPIx = SPI2;
	SPI_ClockControl(SPI2, ENABLE);

	spi2.SPI_Configs.spi_bus_config = SPI_BUS_CFG_FD;
	spi2.SPI_Configs.spi_device_mode = SPI_DEVICE_MODE_SLAVE;
	spi2.SPI_Configs.spi_dff = SPI_DFF_8B;
	spi2.SPI_Configs.spi_cpol = SPI_CPOL_LOW;
	spi2.SPI_Configs.spi_cpha = SPI_CPHA_LOW;
	spi2.SPI_Configs.spi_ssm = SPI_SSM_DI;

	SPI_Init(&spi2);
}

void LED_Init(void){
	GPIO_Handle_t led;

	led.pGPIOx = GPIOC;
	GPIO_ClockControl(GPIOC, ENABLE);

	led.GPIOx_CFG.pin_number = GPIO_PIN_13;
	led.GPIOx_CFG.pin_mode = GPIO_MODE_OUT;
	led.GPIOx_CFG.pin_op_type = GPIO_OUT_PP;
	led.GPIOx_CFG.pin_pu_pd_ctrl = GPIO_NO_PUPD;
	led.GPIOx_CFG.pin_speed = GPIO_OSPEED_HIGH;

	GPIO_Init(&led);
	GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

}

int main(void){
	SPI2_GPIO_Init();
	SPI2_Init();
	LED_Init();

	SPI_PeripheralControl(SPI2, ENABLE);
	SPI_IRQ_Interrupt_CFG(IRQ_NO_SPI2, ENABLE);

	SPI_Receive_IT(&spi2, &rx, 1);

	while(1);
}
