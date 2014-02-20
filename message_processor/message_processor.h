#ifndef __MESSAGE_PROCESSOR_H
#define __MESSAGE_PROCESSOR_H

#include "servo_controll.h"


/**
 * Накапливает и далее исполняет комманды протокола управление сервами.
 * groups_data - in, данные группы серв
 * uart_data - in, очередной дайт данных.
 * */
void build_cmd(struct GroupsData * groups_data, uint8_t uart_data);

#endif __MESSAGE_PROCESSOR_H
