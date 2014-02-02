#include "message_processor.h"
#include "common.h"

//Буффер для сборки сообщений.
uint8_t cmd_buffer[40];
uint8_t cmd_buffer_index = 0;

void build_cmd(struct GroupsData * servos_data, uint8_t uart_data)
{
    //Приём байта
    cmd_buffer[cmd_buffer_index] = uart_data;
    cmd_buffer_index++;

    //завершение приёма пакета
    if ( (cmd_buffer[0] + 2) == cmd_buffer_index )
    {
        cmd_buffer_index = 0;

        switch (cmd_buffer[1])
        {
            //эхо
            case CMD_ECHO:
            {
                break;
            }
            //установка позиции.
            case CMD_SET_SEVO_POS:
            {
                if ( cmd_buffer[0]  == sizeof(struct ServoPosData))
                    cmd_buffer[2] = set_servo_angle(servos_data, (struct ServoPosData *)&cmd_buffer[2]);
                else
                    cmd_buffer[2] = WRONG_CMD_PACKET_SIZE;

                cmd_buffer[0] = 1;
                break;
            }
            case CMD_SET_SEVOS_RANGES:
            {
                if ( cmd_buffer[0] == sizeof(struct GroupSettings))
                    cmd_buffer[2] = set_servo_range(servos_data, (struct GroupSettings *)&cmd_buffer[2]);
                else
                    cmd_buffer[2] = WRONG_CMD_PACKET_SIZE;

                cmd_buffer[0] = 1;
                break;
            }
            case CMD_ADD_ANIMATION:
            {
                cmd_buffer[0] = 1;
                cmd_buffer[2] = NOT_IMPLEMENTED;
                break;
            }
            case CMD_CLEAR_ANIMATION:
            {
                cmd_buffer[0] = 1;
                cmd_buffer[2] = NOT_IMPLEMENTED;
                break;
            }
            case CMD_START_ANIMATIONS:
            {
                cmd_buffer[0] = 1;
                cmd_buffer[2] = NOT_IMPLEMENTED;
                break;
            }
            case CMD_STOP_ANIMATIONS:
            {
                cmd_buffer[0] = 1;
                cmd_buffer[2] = NOT_IMPLEMENTED;
                break;
            }
            case CMD_GET_SEVOS_RANGES:
            {
                cmd_buffer[0] = 1;

                if ( cmd_buffer[0] == sizeof(uint8_t) )
                {
                    cmd_buffer[2] = get_servo_range(servos_data,
                                                        *((uint8_t *)&cmd_buffer[2]),
                                                        (struct GroupSettings *)&cmd_buffer[3]);
                    if ( cmd_buffer[2] == 0 )
                        cmd_buffer[0] = 1 + sizeof(struct GroupSettings);
                }
                else
                    cmd_buffer[2] = WRONG_CMD_PACKET_SIZE;
                break;
            }
        }
    }
}
