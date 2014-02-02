#include <stm32f10x_conf.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
#include "servo_controll.h"
#include "../common.h"

//инициализация данных сервы и таймера
void init_servo_data(struct GroupsData * groups_data)
{
	assert_param(groups_data);

    //1. инициализация портов
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_0 |
    				GPIO_Pin_1 |
    				GPIO_Pin_6 |
    				GPIO_Pin_7 |
    				GPIO_Pin_8 |
    				GPIO_Pin_9 |
    				GPIO_Pin_10 |
    				GPIO_Pin_11;

    GPIO_Init(GPIOA, &gpio);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_0 |
    				GPIO_Pin_1 |
    				GPIO_Pin_6 |
    				GPIO_Pin_7 |
    				GPIO_Pin_8 |
    				GPIO_Pin_9 |
    				GPIO_Pin_10 |
    				GPIO_Pin_11;

    GPIO_Init(GPIOB, &gpio);

    //2. инициализация таймеров.
    float min_time = 0.00045;
    float max_time = 0.0030;
    float duration = max_time - min_time;
    float middle_time = min_time + duration / 2.f;
    uint16_t resolution = 240;
    float period = 0.02;
    float angle_step = duration / resolution;

    struct GroupData * begin_group = groups_data->group;
    struct GroupData * end_group = &groups_data->group[GROUPS_COUNT];

    struct ServoData * servo_begin;
    struct ServoData * servo_end;

    //3. инициализация данных
    for (; begin_group < end_group; begin_group++ )
    {
    	begin_group->period = period;
    	begin_group->resolution = resolution;
    	begin_group->angle_step = angle_step;

    	servo_begin = begin_group->servos;
    	servo_end = &begin_group->servos[SERVOS_COUNT_IN_GROUP];

    	for (; servo_begin < servo_end; servo_begin++ )
    	{
    		servo_begin->min_time = min_time;
    		servo_begin->max_time = max_time;
    		servo_begin->cur_time = middle_time;
    		servo_begin->timer_value = servo_begin->cur_time / begin_group->angle_step;
    	}
    }

    //таймер TIM1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    //таймер TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    //таймер TIM3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    //таймер TIM4
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef base_timer;
    TIM_TimeBaseStructInit(&base_timer);

    base_timer.TIM_Prescaler = (uint16_t)(angle_step / (1. / SystemCoreClock));
    base_timer.TIM_Period = (uint16_t)(period / angle_step);

    TIM_TimeBaseInit(TIM1, &base_timer);
    TIM_TimeBaseInit(TIM2, &base_timer);
    TIM_TimeBaseInit(TIM3, &base_timer);
    TIM_TimeBaseInit(TIM4, &base_timer);


    /* Конфигурируем канал:
    - начальное заполнение: SERVO_SHORTEST_PULSE
    - режим: edge-aligned PWM */
    TIM_OCInitTypeDef timer_oc;
    TIM_OCStructInit(&timer_oc);

    timer_oc.TIM_Pulse = middle_time / angle_step;
    timer_oc.TIM_OCMode = TIM_OCMode_PWM1;
    timer_oc.TIM_OutputState = TIM_OutputState_Enable;

    TIM_OC1Init(TIM1, &timer_oc);
    TIM_OC2Init(TIM1, &timer_oc);
    TIM_OC3Init(TIM1, &timer_oc);
    TIM_OC4Init(TIM1, &timer_oc);

    TIM_OC1Init(TIM2, &timer_oc);
    TIM_OC2Init(TIM2, &timer_oc);
    TIM_OC3Init(TIM2, &timer_oc);
    TIM_OC4Init(TIM2, &timer_oc);

    TIM_OC1Init(TIM3, &timer_oc);
    TIM_OC2Init(TIM3, &timer_oc);
    TIM_OC3Init(TIM3, &timer_oc);
    TIM_OC4Init(TIM3, &timer_oc);

    TIM_OC1Init(TIM4, &timer_oc);
    TIM_OC2Init(TIM4, &timer_oc);
    TIM_OC3Init(TIM4, &timer_oc);
    TIM_OC4Init(TIM4, &timer_oc);


    /* Включаем прерывание переполнения счётчика */
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);



    /* Включаем счётчик */
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM4_IRQn);
}


//Установить переделы для серв
int set_servo_range(struct GroupsData * groups_data, const struct GroupSettings * group_settings)
{
	assert_param(groups_data);
    assert_param(group_settings);

    if ( group_settings->group_id < 0 || group_settings->group_id >= GROUPS_COUNT )
    	return WRONG_GROUP_NUMBER;

    if ( group_settings->ranges->min_time > group_settings->ranges->max_time )
        return WRONG_PARAMS;

    if ( group_settings->resolution < 10 )
        return WRONG_PARAMS;

    if ( group_settings->period < 0.005 )
        return WRONG_PARAMS;


    int i;
    float min_range = group_settings->ranges->max_time - group_settings->ranges->min_time;
    float cur_range;

    //1. подсчёт минимума и максимума.
    for (i = 1; i < SERVOS_COUNT_IN_GROUP; i++)
    {
        if (group_settings->ranges[i].min_time > group_settings->ranges[i].max_time)
            return WRONG_PARAMS;

        cur_range = group_settings->ranges[i].max_time - group_settings->ranges[i].min_time;

        if ( cur_range < min_range )
            min_range = cur_range;
    }

    struct GroupData * group_data = &groups_data->group[group_settings->group_id];
    group_data->period = group_settings->period;
    group_data->resolution = group_settings->resolution;
    group_data->angle_step = min_range / group_data->resolution;

    float old_value;

    struct ServoData * begin_servo = group_data->servos;
    struct ServoData * end_servo = &group_data->servos[SERVOS_COUNT_IN_GROUP];

    const struct ServoRange * range = group_settings->ranges;


    //2. установка параметров.
    for (; begin_servo < end_servo; begin_servo++, range++)
    {
    	old_value = (begin_servo->cur_time - begin_servo->min_time) /
    				(begin_servo->max_time - begin_servo->min_time);

        cur_range = (range->max_time - range->min_time);

        begin_servo->min_time = range->min_time;
        begin_servo->max_time = range->max_time;
        begin_servo->cur_time = begin_servo->min_time + cur_range * old_value;
        begin_servo->timer_value = begin_servo->cur_time / group_data->angle_step;
    }

    //3. Установка параметров таймера.
    TIM_TimeBaseInitTypeDef base_timer;
    TIM_TimeBaseStructInit(&base_timer);
    base_timer.TIM_Prescaler = (uint16_t)(group_data->angle_step / (1. / SystemCoreClock));
    base_timer.TIM_Period = (uint16_t)(group_data->period / group_data->angle_step);

    if (group_settings->group_id == 0)
    {
    	TIM_TimeBaseInit(TIM1, &base_timer);
    	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    }
    else if (group_settings->group_id == 1)
    {
    	TIM_TimeBaseInit(TIM2, &base_timer);
    	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    }
    else if (group_settings->group_id == 2)
    {
    	TIM_TimeBaseInit(TIM3, &base_timer);
    	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    }
    else if (group_settings->group_id == 3)
    {
    	TIM_TimeBaseInit(TIM4, &base_timer);
    	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    }
    return NO_ERROR;
}

int get_servo_range(const struct GroupsData * groups_data, int group_id, struct GroupSettings * group_settings)
{
	assert_param(group_settings);

	if (group_id < 0 || group_id >= GROUPS_COUNT )
		return WRONG_GROUP_NUMBER;

	const struct  GroupData * group_data = &groups_data->group[group_id];
	group_settings->group_id = (uint8_t)group_id;
	group_settings->period = group_data->period;
	group_settings->resolution = group_data->resolution;

	TIM_TypeDef * cur_timer;
    if (group_settings->group_id == 0)
    	cur_timer = TIM1;
    else if (group_settings->group_id == 1)
    	cur_timer = TIM2;
    else if (group_settings->group_id == 2)
    	cur_timer = TIM3;
    else if (group_settings->group_id == 3)
    	cur_timer = TIM4;

	int i = 0;

	for (; i < SERVOS_COUNT_IN_GROUP; i++ )
	{
		group_settings->ranges[i].min_time =  group_data->servos[i].min_time;
		group_settings->ranges[i].max_time =  group_data->servos[i].max_time;
	}
	return 0;
}


//установить угол сервы.
int set_servo_angle(struct GroupsData * groups_data, const struct ServoPosData * servo_pos_data)
{
	assert_param(groups_data);
	assert_param(servo_pos_data);

	if ( servo_pos_data->group_id < 0 || servo_pos_data->group_id >= GROUPS_COUNT )
		return WRONG_GROUP_NUMBER;

	if ( servo_pos_data->number < 0 || servo_pos_data->number >= SERVOS_COUNT_IN_GROUP )
		return WRONG_SERVO_NUMBER;

	if ( servo_pos_data->value < 0.f || servo_pos_data->value > 1.f )
		return WRONG_SERVO_ANGLE;

	struct GroupData * group_data = &groups_data->group[servo_pos_data->group_id];


	struct ServoData * servo_data = &group_data->servos[servo_pos_data->number];
	servo_data->cur_time = servo_data->min_time + (servo_data->max_time - servo_data->min_time) * servo_pos_data->value;
	servo_data->timer_value = servo_data->cur_time / group_data->angle_step;

	if (servo_pos_data->group_id == 0)
    	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    else if (servo_pos_data->group_id == 1)
    	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    else if (servo_pos_data->group_id == 2)
    	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    else if (servo_pos_data->group_id == 3)
    	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	return NO_ERROR;
}
