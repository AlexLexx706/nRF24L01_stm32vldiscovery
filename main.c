#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "nrf24l01.h"
#include "nrf24l01_register_map.h"
#include "rf_spi2.h"
#include "servo_controll/servo_controll.h"
#include <stm32f10x_tim.h>
#include <math.h>
#include "message_processor/message_processor.h"

#define DEVICE_1_ADDRESS	0X0101010101
#define DEVICE_2_ADDRESS	0X0202020202


//передтчик
NRF24L01_Device NRF24L01_1;

//сервы.
static struct GroupsData servos_data;

//прерывание от NRF24L01
void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		NRF24L01_Interrupt(&NRF24L01_1);

		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

//таймер 1.
void TIM1_UP_TIM16_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        TIM_SetCompare1(TIM1, servos_data.group[0].servos[0].timer_value);
        TIM_SetCompare2(TIM1, servos_data.group[0].servos[1].timer_value);
        TIM_SetCompare3(TIM1, servos_data.group[0].servos[2].timer_value);
        TIM_SetCompare4(TIM1, servos_data.group[0].servos[3].timer_value);

        //отключим прерывание.
        TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    }
}

//таймер 2.
void TIM2_IRQHandler(void)
{
    //Если счётчик переполнился, можно смело закидывать в регистр сравнения новое значение.
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TIM_SetCompare1(TIM2, servos_data.group[1].servos[0].timer_value);
        TIM_SetCompare2(TIM2, servos_data.group[1].servos[1].timer_value);
        TIM_SetCompare3(TIM2, servos_data.group[1].servos[3].timer_value);
        TIM_SetCompare4(TIM2, servos_data.group[1].servos[2].timer_value);
        TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    }
}

//таймер 3.
void TIM3_IRQHandler(void)
{
    //Если счётчик переполнился, можно смело закидывать в регистр сравнения новое значение.
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        TIM_SetCompare1(TIM3, servos_data.group[2].servos[0].timer_value);
        TIM_SetCompare2(TIM3, servos_data.group[2].servos[1].timer_value);
        TIM_SetCompare3(TIM3, servos_data.group[2].servos[2].timer_value);
        TIM_SetCompare4(TIM3, servos_data.group[2].servos[3].timer_value);
        TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    }
}

//таймер 4.
void TIM4_IRQHandler(void)
{
    //Если счётчик переполнился, можно смело закидывать в регистр сравнения новое значение.
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        TIM_SetCompare1(TIM4, servos_data.group[3].servos[0].timer_value);
        TIM_SetCompare2(TIM4, servos_data.group[3].servos[1].timer_value);
        TIM_SetCompare3(TIM4, servos_data.group[3].servos[2].timer_value);
        TIM_SetCompare4(TIM4, servos_data.group[3].servos[3].timer_value);
        TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    }
}

void BOARD_Init()
{
	/* nRF24L01_1 */
	NRF24L01_1.NRF24L01_DeviceName	= "1";
	NRF24L01_1.CSN_Pin 				= GPIO_Pin_12;
	NRF24L01_1.CSN_GPIO				= GPIOB;

	NRF24L01_1.CE_Pin 				= GPIO_Pin_6;
	NRF24L01_1.CE_GPIO				= GPIOC;

	NRF24L01_1.IRQ_Pin 				= GPIO_Pin_1;
	NRF24L01_1.IRQ_GPIO				= GPIOC;
	NRF24L01_1.IRQ_GPIO_PortSource 	= GPIO_PortSourceGPIOC;
	NRF24L01_1.IRQ_GPIO_PinSource 	= GPIO_PinSource1;
	NRF24L01_1.IRQ_EXTI_Line 		= EXTI_Line1;
	NRF24L01_1.IRQ_NVIC_IRQChannel 	= EXTI1_IRQn;

	NRF24L01_1.SPIx 				= SPI2;
	NRF24L01_1.SPIx_Init 			= RF_SPI2_Init;
	NRF24L01_1.SPIx_WriteRead 		= RF_SPI2_WriteRead;
	NRF24L01_1.SPIx_Write 			= RF_SPI2_Write;
	NRF24L01_Init(&NRF24L01_1);

	NRF24L01_SetRFChannel(&NRF24L01_1, 66);
	NRF24L01_SetTxAddress(&NRF24L01_1, DEVICE_2_ADDRESS);
	NRF24L01_SetRxPipeAddress(&NRF24L01_1, 0, DEVICE_2_ADDRESS);
	NRF24L01_SetRxPipeAddress(&NRF24L01_1, 1, DEVICE_1_ADDRESS);

	//Затактируем порт C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);

	//Прерывания - это альтернативная функция порта
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE);

	// по умолчанию там ноли, поэтому установим только 1 бит
	AFIO->EXTICR[0]|=AFIO_EXTICR1_EXTI1_PC;

	//Прерывания от первой ноги
	EXTI->IMR|=(EXTI_IMR_MR1);

	//Прерывания по спадаюшему фронту
	EXTI->FTSR|=(EXTI_RTSR_TR1);

	//Разрешаем оба прерывания
	NVIC_EnableIRQ (EXTI1_IRQn);
}



void init_lamps()
{
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpio_init_struct;
	GPIO_StructInit(&gpio_init_struct);
	gpio_init_struct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio_init_struct);

    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
}


int main(void)
{
    //3. инициализация лампочек.
    init_lamps();

	//1. включаем nrf24l01
	BOARD_Init();

	//2. инициализация серв.
    init_servo_data(&servos_data);

    struct GroupSettings g0 = {0, 0.02, 240,
            {{0.00039,0.00242,},
            {0.00048, 0.0025},
            {0.00048, 0.00249},
            {0.0004, 0.00242}}};

    set_servo_range(&servos_data, &g0);

    struct GroupSettings g1 = {1, 0.02, 240,
            {{0.00053,0.00255},
            {0.000449,0.00244},
            {0.00048,0.0025},
            {0.000459, 0.00248}}};
    set_servo_range(&servos_data, &g1);

    struct GroupSettings g2 = {2, 0.02, 240,
            {{0.000440, 0.002480},
            {0.000440, 0.002440},
            {0.000510, 0.002460},
            {0.000480,  0.002500}}};
    set_servo_range(&servos_data, &g2);

    struct GroupSettings g3 = {3, 0.02, 240,
            {{0.00048, 0.0025},
            {0.00048, 0.00249},
            {0.000449, 0.00243},
            {0.00047, 0.0025}}};
    set_servo_range(&servos_data, &g3);

	//SysTick_Config(SystemCoreClock/50);
	__enable_irq();


    uint8_t ch;

	while(1)
    {
		if (circularBuffer_GetCount(&NRF24L01_1.RxPipeBuffer[1]) > 0 )
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_8);
			while (!circularBuffer_IsEmpty(&NRF24L01_1.RxPipeBuffer[1]) )
			{
				ch = circularBuffer_Remove(&NRF24L01_1.RxPipeBuffer[1]);
				build_cmd(&servos_data, ch);
			}
			GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		}
    }
}

void SysTick_Handler(void)
{
    struct ServoPosData spd;
    spd.group_id = 0;
    spd.number = 0;
    static float angle = 0.0;
    float len =  0.3f;
    float d_len = (1.f - len) / 2.f;

    spd.value = (cos(angle) + 1.0) / 2.0 * len + d_len;
    angle = angle + 0.05;

    for (; spd.group_id < GROUPS_COUNT; spd.group_id++ )
    {
         spd.number = 0;
        for (; spd.number < SERVOS_COUNT_IN_GROUP; spd.number++ )
        {
            set_servo_angle(&servos_data, &spd);
        }
    }
}


