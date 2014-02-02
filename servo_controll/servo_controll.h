#ifndef __SERVO_CONTROL_H_
#define __SERVO_CONTROL_H_
#include "stm32f10x.h"

#pragma pack (push, 1)
#define SERVOS_COUNT_IN_GROUP 4
#define GROUPS_COUNT 4

struct ServoData
{
    //минимальное время, значение в отсчётах с учётом прескеллера
    float min_time;

    //минимальное время, значение в отсчётах с учётом прескеллера
    float max_time;

    //текущее значение сервы
    float cur_time;

    //значение для таймера
    uint16_t timer_value;
};

struct GroupData
{
    //индекс таймера.
    uint8_t timer_id;

    //переод шима
    float period;

    float angle_step;

    //разрешение серв.
    uint16_t resolution;


    //настройки сервы.
    struct ServoData servos[SERVOS_COUNT_IN_GROUP];
};


//общая настройка групп
struct GroupsData
{
    struct GroupData group[GROUPS_COUNT];
};

struct ServoRange
{
    //минимальное время в cекундах
    float min_time;

    //минимальное время в cекундах
    float max_time;
};



//настройки переделов
struct GroupSettings
{
    //индекс таймера.
    uint8_t group_id;

    //переод шима
    float period;

    //разрешение серв.
    uint8_t resolution;

    //используется для передачи пределов
    struct ServoRange ranges[SERVOS_COUNT_IN_GROUP];
};

//установка позиции
struct ServoPosData
{
    //индекс таймера.
    uint8_t group_id;

    //номер сервы.
    uint8_t number;

    //значение угла, продел от 0.0 - 1.0
    float value;
};
#pragma pack (pop)

//инициализация.
void init_servo_data(struct GroupsData * servos_data);

//установить пределы
int set_servo_range(struct GroupsData * servos_data, const struct GroupSettings * ranges);

int get_servo_range(const struct GroupsData * groups_data, int group_id, struct GroupSettings * group_settings);

//установить угол сервы.
int set_servo_angle(struct GroupsData * servos_data, const struct ServoPosData * servo_pos_data);


#endif /* __SERVO_CONTROL_H_ */
