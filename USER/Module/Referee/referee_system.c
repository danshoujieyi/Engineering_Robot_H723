//
// Created by ���ο� on 25-2-25.
//

#include "referee_system.h"
#include "string.h"
#include "fifo.h"
#include "crc8_crc16.h"

static fifo_s_t RX_AgreementData_FIFO;           //ʵ�����Ĳ���ϵͳ��������FIFO����
static uint8_t RX_FIFO_Space[FIFO_BUF_LENGTH];              //FIFO��ʵ�ʴ洢��??
/* --------------------------------����ϵͳ���ھ�� ------------------------------- */
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

static Frame_header_Typedef Referee_Data_Header;   //��������֡ͷ�ṹ��
static uint8_t RX_AgreementData_Buffer0[Agreement_RX_BUF_NUM];   //���ղ���ϵͳ�������ݵĽ��ջ�����0,�û��������õ��൱��ԣ
static uint8_t RX_AgreementData_Buffer1[Agreement_RX_BUF_NUM];   //���ղ���ϵͳ�������ݵĽ��ջ�����1���û��������õ��൱��ԣ
static uint8_t RX_Agreement_Data[Agreement_RX_BUF_NUM];          //����������Ž������ݵ�Data��
static unpack_data_t referee_unpack_obj;

/*!�ṹ��ʵ����*/
static game_status_t                           game_status;
static game_result_t                           game_result;
static game_robot_HP_t                         game_robot_HP;
static event_data_t                            field_event;
static referee_warning_t                       referee_warning;
static dart_info_t                             dart_info;
static robot_status_t                          robot_status;
static power_heat_data_t                       power_heat_data;
static robot_pos_t                             robot_pos;
static buff_t                                  buff_musk_t;
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
static custom_robot_data_t                     custom_robot_data;
static map_command_t                           map_command;
static remote_control_t                        remote_control;
static map_robot_data_t                        map_robot_data;
static custom_client_data_t                    custom_client_data;
static map_data_t                              map_data;
static custom_info_t                           custom_info;
static Frame_header_Typedef Referee_Data_header;         //ʵ����һ��֡ͷ�ṹ��
static RX_AgreementData     Referee_Data;                //ʵ����һ������֡�ṹ��

/*����ϵͳ�߳����*/
void referee_thread_entry(void *argument)
{
    /*�û�3pin���ڳ�ʼ��*/
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();
    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    /* DMA2_Stream6_IRQn interrupt configuration */
    /* HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
     HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);*/
    huart6.Instance=USART6;
    huart6.Init.BaudRate=115200;
    huart6.Init.WordLength=UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity=UART_PARITY_NONE;
    huart6.Init.Mode=UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart6.Init.OverSampling=UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);
    /*����ϵͳ��ʼ��*/
    Referee_system_Init(RX_AgreementData_Buffer0,RX_AgreementData_Buffer1,Agreement_RX_BUF_NUM);

    /*����ϵͳ���ݽ��*/
    while(1)
    {
        /*�������ݽ��*/
        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == 0U) //�����ǰ��������0�����0��������������1������
        {
            Referee_Data_Unpack(RX_AgreementData_Buffer0, &Referee_Data_header, &Referee_Data);
        }
        else
        {
            Referee_Data_Unpack(RX_AgreementData_Buffer1, &Referee_Data_header, &Referee_Data);
        }
        /*������ϵͳ����ת��Ҫ���͵�buffer����*/
        memcpy(&(referee_fdb.robot_status),&robot_status, sizeof(robot_status_t));
        memcpy(&(referee_fdb.power_heat_data),&power_heat_data, sizeof(power_heat_data_t));

        rt_thread_mdelay(1);
    }
}

/**
 *@brief ����ϵͳ�߳���ں���
 *
 */
void USART6_IRQHandler(void)
{
    if (huart6.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if (USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_IDLEFLAG(&huart6);      //��������ж�

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //�����ǰ�Ļ������ǻ�����0
        {
            //������һ֡���յ����ݵĳ���
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = Agreement_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
            //�����趨���ݳ���
            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
            //�ѻ��������óɻ�����1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            //����1֡���ݷ���fifo0
            fifo_s_puts(&RX_AgreementData_FIFO, (char *) RX_AgreementData_Buffer0, this_time_rx_len);
        }
        else //�����ǰ�Ļ������ǻ�����1
        {
            //������һ֡���յ����ݵĳ���
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = Agreement_RX_BUF_NUM -hdma_usart6_rx.Instance->NDTR;
            //osSemaphoreRelease(RefereeRxOKHandle);  //�ͷ��ź���
            //�����趨���ݳ���
            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
            //�ѻ��������óɻ�����0
            hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            fifo_s_puts(&RX_AgreementData_FIFO, (char *) RX_AgreementData_Buffer1, this_time_rx_len);

        }
    }
    HAL_UART_IRQHandler(&huart6);
}


/**
 *@brief ����ϵͳ��ʼ������ͨ���Լ�ʵ�����ṹ��
 * @param rx1_buf       �趨�Ľ��ղ���ϵͳ�������ݵĽ��ջ�����1
 * @param rx2_buf       �趨�Ľ��ղ���ϵͳ�������ݵĽ��ջ�����2
 * @param dma_buf_num   DMAת�����ݵĿռ��С
 *
 */
void Referee_system_Init(uint8_t *  rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    memset(&Referee_Data_header, 0, sizeof(Frame_header_Typedef));
    memset(&Referee_Data, 0, sizeof(RX_AgreementData));

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));
    memset(&field_event, 0, sizeof(event_data_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&dart_info, 0, sizeof(dart_info_t));
    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(buff_t));
    memset(&hurt_data, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
    memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
    memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
    memset(&sentry_info, 0, sizeof(sentry_info_t));
    memset(&radar_info, 0, sizeof(radar_info_t));
    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
    memset(&custom_robot_data, 0, sizeof(custom_robot_data_t));
    memset(&map_command, 0, sizeof(map_command_t));
    memset(&remote_control, 0, sizeof(remote_control_t));
    memset(&map_robot_data, 0, sizeof(map_robot_data_t ));
    memset(&custom_client_data, 0, sizeof(custom_client_data_t));
    memset(&map_data, 0, sizeof(map_data_t));
    memset(&custom_info, 0, sizeof(custom_info_t));
    //ʹ��DMA���ڽ���
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //ʧЧDMA�����ȴ�ֱ��SxCR_EN�Ĵ�����0���Ա�֤�������������ݿ���д��
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //�ڴ滺����1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    //fifo��ʼ��
    fifo_s_init(&RX_AgreementData_FIFO,RX_FIFO_Space,FIFO_BUF_LENGTH);    //����FIFO�洢����

}

/**
 * @brief ����ϵͳ���ݽ������
 */

void Referee_Data_Unpack()
{
    unpack_data_t *p_obj = &referee_unpack_obj;
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    while(fifo_s_used(&RX_AgreementData_FIFO))
    {
        byte = fifo_s_get(&RX_AgreementData_FIFO);
        switch (p_obj->unpack_step)  //״̬ת����
        {
            case STEP_HEADER_SOF:      //����Ƕ�ȡ֡ͷSOF��״̬
            {
                if(byte == sof)       //�ж��Ƿ�ΪSOF
                {
                    p_obj->unpack_step = STEP_LENGTH_LOW;       //�ı�״̬���´��ó�����byte��ȥ��ͼ��Ӧ���ݳ��ȵĵͰ�λ
                    p_obj->protocol_packet[p_obj->index++] = byte;  //��������ã������������ȼ�1
                }
                else
                {
                    p_obj->index = 0;   //������ǣ����ٴ�fifo���ó���һ��byte����������ֱ��������һ��sof
                }
            }break;
            case STEP_LENGTH_LOW:       //���Ŀǰ��״̬�Ƕ������ݳ��ȵĵͰ�λ
            {
                p_obj->data_len = byte;           //�Ͱ�λֱ�ӷ���
                p_obj->protocol_packet[p_obj->index++] = byte;   //�������
                p_obj->unpack_step = STEP_LENGTH_HIGH;          //ת��״̬
            }break;

            case STEP_LENGTH_HIGH:  //���Ŀǰ��״̬ʱ�����ݳ��ȵĸ߰�λ
            {
                p_obj->data_len |= (byte << 8);     //����data_len�ĸ߰�λ
                p_obj->protocol_packet[p_obj->index++] = byte;  //�������
                //�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ���ȥ frame_header,cmd_id �� frame_tail �� 9 ���ֽ��Լ����ݶ�ͷ�ṹ�� 6 ���ֽ�
                if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
                {
                    p_obj->unpack_step = STEP_FRAME_SEQ;        //ת��״̬����һ���ö������
                }
                else
                {
                    //������ݳ��Ȳ��Ϸ�������ͷ��ʼ��ȡ������֮ǰ��õ���������
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }break;
            case STEP_FRAME_SEQ:
            {
                p_obj->protocol_packet[p_obj->index++] = byte;  //�������
                p_obj->unpack_step = STEP_HEADER_CRC8;          //ת��״̬����һ��byte������CRC8
            }break;

            case STEP_HEADER_CRC8:
            {
                //�Ƚ���һbyte���ݷ��룬ʹ֡ͷ�ṹ�������Ա������Խ���CRCУ��
                p_obj->protocol_packet[p_obj->index++] = byte;
                //�����һbyte����֮�����ݳ�����һ��֡ͷ�ĳ��ȣ���ô�ͽ���CRCУ��
                if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
                {
                    if ( Verify_CRC8_Check_Sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
                    {
                        p_obj->unpack_step = STEP_DATA_CRC16;   //���У��ͨ������״̬ת����ȥ��ȡ֡β
                    }
                    else
                    {
                        //���У�鲻ͨ�������ͷ��ʼ��֮ǰ��õ���������
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
            }break;

            case STEP_DATA_CRC16:
            {
                //��֡ͷ��֡β�Ĺ����е�����һ�����
                if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                //������ݶ�ȡ��dataĩβ����ת��״̬��׼����ʼ��һ֡�Ķ�ȡ
                if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                    //��������У��
                    if ( Verify_CRC16_Check_Sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
                    {
                        //У��ͨ��������õ�����memcp��ָ���ṹ����
                        Referee_Data_Solve(p_obj->protocol_packet);
                    }
                }
            }break;

        }
    }
}


/**
 * @brief ����ϵͳ�������ݽ������
 * @param referee_data_frame: ���յ�����֡����
 */
void Referee_Data_Solve(uint8_t* frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;
    memcpy(&Referee_Data_header, frame, sizeof(Frame_header_Typedef));
    index += sizeof(Frame_header_Typedef);
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
            memcpy(&field_event, frame + index, sizeof(event_data_t));
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
            memcpy(&buff_musk_t, frame + index, sizeof(buff_t));
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
        case CUSTOMER_CONTROLLER_ROBOT_CMD_ID :
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
    }
}
