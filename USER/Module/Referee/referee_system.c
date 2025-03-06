//
// Created by 刘嘉俊 on 25-2-25.
//

#include <stdio.h>
#include "referee_system.h"
#include "crc8_crc16.h"
#include "FreeRTOS.h"
#include "string.h"

/* --------------------------------裁判系统串口句柄 ------------------------------- */
static referee_data_header_t referee_data_header;   //接收数据帧头结构体
static referee_data_t referee_data;   //接收数据帧头结构体
static unpack_data_t referee_unpack_obj;

struct referee_fdb_msg referee_fdb;

/*!结构体实例化*/
static game_status_t                           game_status;
static game_result_t                           game_result;
static game_robot_HP_t                         game_robot_HP;
static event_data_t                            event_data;
static referee_warning_t                       referee_warning;
static dart_info_t                             dart_info;
static robot_status_t                          robot_status;
static power_heat_data_t                       power_heat_data;
static robot_pos_t                             robot_pos;
static buff_t                                  buff;
static hurt_data_t                             hurt_data;
static shoot_data_t                            shoot_data;
static projectile_allowance_t                  projectile_allowance;
static rfid_status_t                           rfid_status;
static dart_client_cmd_t                       dart_client_cmd;
static ground_robot_position_t                 ground_robot_position;
static radar_mark_data_t                       radar_mark_data;
static sentry_info_t                           sentry_info;
static radar_info_t                            radar_info;
static robot_interaction_data_t                robot_interaction_data;
static map_command_t                           map_command;
static map_robot_data_t                        map_robot_data;
static map_data_t                              map_data;
static custom_info_t                           custom_info;
static custom_robot_data_t                     custom_robot_data;
static robot_custom_data_t                     robot_custom_data;
static remote_control_t                        remote_control;
static custom_client_data_t                    custom_client_data;

void referee_system_init()
{
    memset(&referee_data_header, 0, sizeof(referee_data_header_t));
    memset(&referee_data, 0, sizeof(referee_data_t));
    memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));
    memset(&event_data, 0, sizeof(event_data_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&dart_info, 0, sizeof(dart_info_t));
    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&buff, 0, sizeof(buff_t));
    memset(&hurt_data, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
    memset(&rfid_status, 0, sizeof(rfid_status_t));
    memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));
    memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
    memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
    memset(&sentry_info, 0, sizeof(sentry_info_t));
    memset(&radar_info, 0, sizeof(radar_info_t));
    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
    memset(&map_command, 0, sizeof(map_command_t));
    memset(&map_robot_data, 0, sizeof(map_robot_data_t ));
    memset(&map_data, 0, sizeof(map_data_t));
    memset(&custom_info, 0, sizeof(custom_info_t));
    memset(&custom_robot_data, 0, sizeof(custom_robot_data_t));
    memset(&robot_custom_data, 0, sizeof(robot_custom_data_t));
    memset(&remote_control, 0, sizeof(remote_control_t));
    memset(&custom_client_data, 0, sizeof(custom_client_data_t));

    memset(&referee_fdb, 0, sizeof(struct referee_fdb_msg));
}


/**
 * @brief 裁判系统数据解包函数
 */
void referee_data_unpack(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        switch (referee_unpack_obj.unpack_step)  //状态转换机
        {
            case STEP_HEADER_SOF:      //如果是读取帧头SOF的状态
            {
                if (byte == HEADER_SOF)       //判断是否为SOF
                {
                    referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;  //将数据码好，并将索引长度加1
                    referee_unpack_obj.unpack_step = STEP_DATA_SIZE_LOW;       //改变状态，下次拿出来的byte，去试图照应数据长度的低八位
                } else {
                    referee_unpack_obj.index = 0;   //如果不是，就再从fifo中拿出来一个byte，继续读，直到读出来一个sof
                }
            }
                break;

            case STEP_DATA_SIZE_LOW:       //如果目前的状态是读的数据长度的低八位
            {
                referee_unpack_obj.p_header->data_length = byte;           //低八位直接放入
                referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;   //码好数据
                referee_unpack_obj.unpack_step = STEP_DATA_SIZE_HIGH;          //转变状态
            }
                break;

            case STEP_DATA_SIZE_HIGH:  //如果目前的状态时读数据长度的高八位
            {
                referee_unpack_obj.p_header->data_length |= (byte << 8);     //放入data_len的高八位
                referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;  //码好数据
                if (referee_unpack_obj.p_header->data_length < REF_PROTOCOL_DATA_MAX_SIZE) {
                    referee_unpack_obj.unpack_step = STEP_FRAME_SEQ;        //转变状态，下一个该读包序号
                } else {
                    //如果数据长度不合法，就重头开始读取，并且之前码好的数据作废
                    memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
                    referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
                    referee_unpack_obj.index = 0;
                }

            }
                break;

            case STEP_FRAME_SEQ: {
                referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;  //码好数据
                referee_unpack_obj.unpack_step = STEP_HEADER_CRC8;          //转换状态，下一个byte读的是CRC8
            }
                break;

            case STEP_HEADER_CRC8: {
                //先将这一byte数据放入，使帧头结构完整，以便后面可以进行CRC校验
                referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
                //如果这一byte放入之后，数据长度是一个帧头的长度，那么就进行CRC校验
                if (referee_unpack_obj.index == REF_PROTOCOL_HEADER_SIZE) {
                    if (verify_CRC8_check_sum(referee_unpack_obj.protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
                        referee_unpack_obj.unpack_step = STEP_DATA_CRC16;   //如果校验通过，则状态转换成去读取帧尾
                    } else {
                        //如果校验不通过，则从头开始，之前码好的数据作废
                        memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
                        referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
                        referee_unpack_obj.index = 0;
                    }
                }
            }
                break;

            case STEP_DATA_CRC16: {
                //从帧头到帧尾的过程中的数据一律码好
                if (referee_unpack_obj.index < (REF_HEADER_CRC_CMD_SIZE + referee_unpack_obj.p_header->data_length)) {
                    referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
                }
                //如果数据读取到data末尾，则转换状态，准备开始新一帧的读取
                if (referee_unpack_obj.index >= (REF_HEADER_CRC_CMD_SIZE + referee_unpack_obj.p_header->data_length)) {
                    //整包数据校验
                    if (verify_CRC16_check_sum(referee_unpack_obj.protocol_packet,
                                               REF_HEADER_CRC_CMD_SIZE + referee_unpack_obj.p_header->data_length)) {
                        referee_data_save(referee_unpack_obj.protocol_packet);
                    }
                    memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
                    referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
                    referee_unpack_obj.index = 0;
                }
            }
                break;

            default:
                memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
                referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
                referee_unpack_obj.index = 0;
                break;
        }
        // 防止缓冲区溢出
//        if(referee_unpack_obj.index >= REF_PROTOCOL_FRAME_MAX_SIZE) {
//            memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
//            referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
//            referee_unpack_obj.index = 0;
//        }
    }
}

float float_values[7] = {0}; // 存储转换后的6个float


/**
 * @brief 裁判系统命令数据解包函数
 * @param referee_data_frame: 接收到的整帧数据
 */
void referee_data_save(uint8_t* frame)
{
    uint16_t cmd_id = 0;
    uint8_t index = 0;

    memcpy(&referee_data_header, frame, sizeof(referee_data_header_t));
    index += sizeof(referee_data_header_t);
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case GAME_STATUS_CMD_ID:
            memcpy(&game_status, frame + index, sizeof(game_status_t));
            break;
        case GAME_RESULT_CMD_ID:
            memcpy(&game_result, frame + index, sizeof(game_result_t));
            break;
        case GAME_ROBOT_HP_CMD_ID:
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
            break;
        case FIELD_EVENTS_CMD_ID:
            memcpy(&event_data, frame + index, sizeof(event_data_t));
            break;
        case REFEREE_WARNING_CMD_ID:
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
            break;
        case DART_FIRE_CMD_ID :
            memcpy(&dart_info, frame + index, sizeof(dart_info_t));
            break;
        case ROBOT_STATUS_CMD_ID:
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
            memcpy(&(referee_fdb.robot_status),&robot_status, sizeof(robot_status_t));
            break;
        case POWER_HEAT_DATA_CMD_ID:
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
            break;
        case ROBOT_POS_CMD_ID:
            memcpy(&robot_pos, frame + index, sizeof(robot_pos_t));
            break;
        case BUFF_MUSK_CMD_ID:
            memcpy(&buff, frame + index, sizeof(buff_t));
            break;
        case ROBOT_HURT_CMD_ID:
            memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
            break;
        case SHOOT_DATA_CMD_ID:
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
            break;
        case BULLET_REMAINING_CMD_ID:
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
            break;
        case ROBOT_RFID_CMD_ID :
            memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
            break;
        case DART_DIRECTIONS_CMD_ID  :
            memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
            break;
        case ROBOT_LOCATION_CMD_ID :
            memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
            break;
        case RADAR_PROGRESS_CMD_ID :
            memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data_t));
            break;
        case SENTRY_AUTONOMY__CMD_ID :
            memcpy(&sentry_info, frame + index, sizeof(sentry_info_t));
            break;
        case RADAR_AUTONOMY_CMD_ID :
            memcpy(&radar_info, frame + index, sizeof(radar_info_t));
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
            memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
            break;
        case ARM_DATA_FROM_CONTROLLER_CMD_ID_2 :
            memcpy(&custom_robot_data, frame + index, sizeof(custom_robot_data_t));
            memcpy(&(referee_fdb.custom_robot_data),&custom_robot_data, sizeof(custom_robot_data_t));
            for (int i = 0; i < 7; i++) {
                uint8_t* byte_ptr = &custom_robot_data.data[i * 4];
                memcpy(&float_values[i], byte_ptr, sizeof(float));
            }
            break;
        case PLAYER_MINIMAP_CMD_ID :
            memcpy(&map_command, frame + index, sizeof(map_command_t));
            break;
        case KEYBOARD_MOUSE_CMD_ID :
            memcpy(&remote_control, frame + index, sizeof(remote_control_t));
            break;
        case RADAR_MINIMAP_CMD_ID :
            memcpy(&map_robot_data, frame + index, sizeof(map_robot_data_t));
            break;
        case CUSTOMER_CONTROLLER_PLAYER_CMD_ID :
            memcpy(&custom_client_data, frame + index, sizeof(custom_client_data_t));
            break;
        case PLAYER_MINIMAP_SENTRY_CMD_ID :
            memcpy(&map_data, frame + index, sizeof(map_data_t));
            break;
        case PLAYER_MINIMAP_ROBOT_CMD_ID :
            memcpy(&custom_info, frame + index, sizeof(custom_info_t));
            break;
        case ARM_DATA_FROM_CONTROLLER_CMD_ID_9 :
            memcpy(&robot_custom_data, frame + index, sizeof(robot_custom_data_t));
            break;
        default:
            break;
    }
}
