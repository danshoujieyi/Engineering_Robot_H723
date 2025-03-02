//
// Created by ���ο� on 25-2-25.
//

#include "referee_system.h"
#include "string.h"
#include "fifo.h"
#include "crc8_crc16.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "usart.h"

// ��ͷ�ļ��������о��
QueueHandle_t xUARTQueue;

/* --------------------------------����ϵͳ���ھ�� ------------------------------- */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

static referee_data_header_t referee_data_header;   //��������֡ͷ�ṹ��
static referee_data_t referee_data;   //��������֡ͷ�ṹ��

static unpack_data_t referee_unpack_obj;

static uint8_t referee_rx_buffer_index = 0;  // ��ǰʹ�õĽ��ջ�����
static uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];

volatile uint8_t referee_data_ready = 0;      // ��־λ����ʾ���ݽ������


/*!�ṹ��ʵ����*/
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





/**
* @brief ��������
* @param tx_buffer ���ͻ�����
* @param Size ���͵����ݳ���
*/
void referee_send_data(uint8_t *tx_buffer, uint16_t Size)
{
    HAL_UART_Transmit_DMA(&huart1, tx_buffer, Size);
}


/*����ϵͳ�߳����*/
void referee_thread_entry(void *argument)
{
    /*����ϵͳ��ʼ��*/
    referee_system_init();

    /*����ϵͳ���ݽ��*/
    while(1)
    {
        if (referee_data_ready) {
            referee_data_ready = 0; // �����־λ
            // �������յ�������
            referee_data_unpack();
            // �ɹ����͵����к������ǰ�������������Ҫ��
            memset(referee_rx_buffer[(referee_rx_buffer_index == 0) ? 1 : 0], 0, REFEREE_RX_BUF_SIZE);
        }

        vTaskDelay(1);
    }
}

void USART1_DMA_Init(void) {
    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
    //ʹ��DMA���ڽ���
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // �ر�DMA�Ĵ�������жϣ�����������ж�
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/**
 *@brief ����ϵͳ��ʼ������ͨ���Լ�ʵ�����ṹ��
 * @param rx1_buf       �趨�Ľ��ղ���ϵͳ�������ݵĽ��ջ�����1
 * @param rx2_buf       �趨�Ľ��ղ���ϵͳ�������ݵĽ��ջ�����2
 * @param dma_buf_num   DMAת�����ݵĿռ��С
 *
 */
void referee_system_init()
{
    xUARTQueue = xQueueCreate(1024, sizeof(uint8_t));
    if (xUARTQueue == NULL) {
        Error_Handler(); // ���д���ʧ��
    }

    memset(&referee_data_header, 0, sizeof(referee_data_header_t));
    memset(&referee_data, 0, sizeof(referee_data_t));

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

    USART1_DMA_Init();
}

/**
 * @brief ����ϵͳ���ݽ������
 */

void referee_data_unpack()
{
    unpack_data_t *p_obj = &referee_unpack_obj;
    uint8_t byte = 0;
    for(;;) {
        // ʹ�ö��н�������
        if(xQueueReceive(xUARTQueue, &byte, pdMS_TO_TICKS(10)) == pdPASS)
        {
            switch (p_obj->unpack_step)  //״̬ת����
            {
                case STEP_HEADER_SOF:      //����Ƕ�ȡ֡ͷSOF��״̬
                {
                    if (byte == HEADER_SOF)       //�ж��Ƿ�ΪSOF
                    {
                        p_obj->unpack_step = STEP_LENGTH_LOW;       //�ı�״̬���´��ó�����byte��ȥ��ͼ��Ӧ���ݳ��ȵĵͰ�λ
                        p_obj->protocol_packet[p_obj->index++] = byte;  //��������ã������������ȼ�1
                    } else {
                        p_obj->index = 0;   //������ǣ����ٴ�fifo���ó���һ��byte����������ֱ��������һ��sof
                    }
                }
                    break;

                case STEP_LENGTH_LOW:       //���Ŀǰ��״̬�Ƕ������ݳ��ȵĵͰ�λ
                {
                    p_obj->data_len = byte;           //�Ͱ�λֱ�ӷ���
                    p_obj->protocol_packet[p_obj->index++] = byte;   //�������
                    p_obj->unpack_step = STEP_LENGTH_HIGH;          //ת��״̬
                }
                    break;

                case STEP_LENGTH_HIGH:  //���Ŀǰ��״̬ʱ�����ݳ��ȵĸ߰�λ
                {
                    p_obj->data_len |= (byte << 8);     //����data_len�ĸ߰�λ
                    p_obj->protocol_packet[p_obj->index++] = byte;  //�������
                    //�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ���ȥ frame_header,cmd_id �� frame_tail �� 9 ���ֽ��Լ����ݶ�ͷ�ṹ�� 6 ���ֽ�
                    if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
                        p_obj->unpack_step = STEP_FRAME_SEQ;        //ת��״̬����һ���ö������
                    } else {
                        //������ݳ��Ȳ��Ϸ�������ͷ��ʼ��ȡ������֮ǰ��õ���������
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
                    break;

                case STEP_FRAME_SEQ: {
                    p_obj->protocol_packet[p_obj->index++] = byte;  //�������
                    p_obj->unpack_step = STEP_HEADER_CRC8;          //ת��״̬����һ��byte������CRC8
                }
                    break;

                case STEP_HEADER_CRC8: {
                    //�Ƚ���һbyte���ݷ��룬ʹ֡ͷ�ṹ�������Ա������Խ���CRCУ��
                    p_obj->protocol_packet[p_obj->index++] = byte;
                    //�����һbyte����֮�����ݳ�����һ��֡ͷ�ĳ��ȣ���ô�ͽ���CRCУ��
                    if (p_obj->index == REF_PROTOCOL_HEADER_SIZE) {
                        if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
                            p_obj->unpack_step = STEP_DATA_CRC16;   //���У��ͨ������״̬ת����ȥ��ȡ֡β
                        } else {
                            //���У�鲻ͨ�������ͷ��ʼ��֮ǰ��õ���������
                            p_obj->unpack_step = STEP_HEADER_SOF;
                            p_obj->index = 0;
                        }
                    }
                }
                    break;

                case STEP_DATA_CRC16: {
                    //��֡ͷ��֡β�Ĺ����е�����һ�����
                    if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                        p_obj->protocol_packet[p_obj->index++] = byte;
                    }
                    //������ݶ�ȡ��dataĩβ����ת��״̬��׼����ʼ��һ֡�Ķ�ȡ
                    if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                        //��������У��
                        if (verify_CRC16_check_sum(p_obj->protocol_packet,
                                                   REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                            //У��ͨ��������õ�����memcp��ָ���ṹ����
                            referee_data_save(p_obj->protocol_packet);
                        }
                    }
                }
                    break;
            }
        }
    }
}


/**
 * @brief ����ϵͳ�������ݽ������
 * @param referee_data_frame: ���յ�����֡����
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
    }
}
