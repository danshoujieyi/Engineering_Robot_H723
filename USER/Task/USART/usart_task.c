//
// Created by ���ο� on 25-4-9.
//

#include "usart_task.h"
#include "FreeRTOS.h"
#include "rc_sbus.h"
#include "referee_system.h"
#include "usart.h"
#include <string.h>
#include "drv_dwt.h"

#include <stdio.h>
#include <stdarg.h>

// �Զ������������
static volatile uint8_t usart1_rx_buffer_index = 0;  // ��ǰʹ�õĽ��ջ�����
static volatile uint16_t usart1_rx_size = 0;
static uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];
static SemaphoreHandle_t xSemaphoreUART1_RX = NULL;           // ֪ͨ�������ź���

// ��˹ң����
static volatile uint8_t usart5_rx_buffer_index = 0;  // ��ǰʹ�õĽ��ջ�����
static volatile uint16_t usart5_rx_size = 0;
static uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];
static SemaphoreHandle_t xSemaphoreUART5_RX = NULL;

// ����ϵͳ���� 10
static volatile uint8_t referee_rx_buffer_index = 0;  // ��ǰʹ�õĽ��ջ�����
static volatile uint16_t referee_rx_size = 0;
static uint8_t referee_rx_buffer[2][REFEREE_RX_BUF_SIZE];
static SemaphoreHandle_t xSemaphoreUART10_RX = NULL;

static QueueSetHandle_t xUartQueueSet = NULL; // ������ն��м����,ͳһ�������ж��ź���

// usart7˫���巢��
#define USART7_TX_DEBUG_BUFFER_SIZE 256 // printf���ͻ�������С
static char usart7_tx_debug_buffer[2][USART7_TX_DEBUG_BUFFER_SIZE];
static volatile uint8_t usart7_tx_buffer_index = 0;  // ��ǰʹ�õĻ�����
static SemaphoreHandle_t xSemaphoreUART7_TX = NULL; // ����1�����ź���

extern struct referee_fdb_msg referee_fdb;

void usart_rx_semaphore_init(void)
{
    // FreeRTOS ��ʼ��
    xSemaphoreUART10_RX = xSemaphoreCreateBinary();  // <-- �ڴ˴������ź���
    xSemaphoreUART1_RX = xSemaphoreCreateBinary();  // <-- �ڴ˴������ź���
    xSemaphoreUART5_RX = xSemaphoreCreateBinary();  // <-- �ڴ˴������ź���
    // ������м��������� 3 ���ź�����
    xUartQueueSet = xQueueCreateSet(3);
    // ���������ź���������м�
    xQueueAddToSet(xSemaphoreUART1_RX, xUartQueueSet);
    xQueueAddToSet(xSemaphoreUART5_RX, xUartQueueSet);
    xQueueAddToSet(xSemaphoreUART10_RX, xUartQueueSet);
}

void usart_tx_semaphore_init(void)
{
    // ������ֵ�ź���������DMA����ͬ����
    xSemaphoreUART7_TX = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphoreUART7_TX);
}

void USART1_RX_DMA_Init(void) {
    memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));
    //ʹ��DMA���ڽ���
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE);
    // �ر�DMA�Ĵ�������жϣ�����������ж�
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void USART5_RX_DMA_Init(void) {
    memset(usart5_rx_buffer, 0, SBUS_RX_BUF_SIZE);
    // �ر�DMA�Ĵ�������жϣ�����������ж�
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // ������Ϻ�����
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void USART10_RX_DMA_Init(void) {
    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
    //ʹ��DMA���ڽ���
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
    // �ر�DMA�Ĵ�������жϣ�����������ж�
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}

void process_usart1_rx_data(void) {
    uint8_t finishedBuffer;

    // ��������ȡ�ź�������Ϊ���м���ȷ���ź�����Ч��
    if (xSemaphoreTake(xSemaphoreUART1_RX, 0) == pdTRUE) {
        finishedBuffer = usart1_rx_buffer_index ^ 1;
        /* ���ݽ��� */
        referee_data_unpack(usart1_rx_buffer[finishedBuffer], usart1_rx_size);

        memset(usart1_rx_buffer[finishedBuffer], 0, CUSTOMER_CONTROLLER_BUF_SIZE);

    }
}

void process_uart5_rx_data(void) {
    uint8_t finishedBuffer;

    if (xSemaphoreTake(xSemaphoreUART5_RX, 0) == pdTRUE) {
        finishedBuffer = usart5_rx_buffer_index ^ 1;
        /* SBUSЭ����� */
        sbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);

        memset(usart5_rx_buffer[finishedBuffer], 0, SBUS_RX_BUF_SIZE);
    }
}

void process_uart10_rx_data(void) {
    uint8_t finishedBuffer;

    if (xSemaphoreTake(xSemaphoreUART10_RX, 0) == pdTRUE) {
        finishedBuffer = referee_rx_buffer_index ^ 1;
        /* ����ϵͳ���ݽ��� */
        referee_data_unpack(referee_rx_buffer[finishedBuffer], referee_rx_size);

        memset(referee_rx_buffer[finishedBuffer], 0, REFEREE_RX_BUF_SIZE);
    }
}

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
static uint32_t usart_task_dwt = 0;   // ������
static float usart_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float usart_task_delta = 0;    // ����߳�����ʱ��
static float usart_task_start_dt = 0; // ����߳̿�ʼʱ��
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

void UsartTask_Entry(void const * argument)
{
    QueueSetMemberHandle_t xActivatedMember;

    USART1_RX_DMA_Init();
    USART5_RX_DMA_Init();
    USART10_RX_DMA_Init();
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    usart_task_dt = dwt_get_delta(&usart_task_dwt);
    usart_task_start_dt = dwt_get_time_ms();
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    for(;;)
    {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        usart_task_delta = dwt_get_time_ms() - usart_task_start_dt;
        usart_task_start_dt = dwt_get_time_ms();

        usart_task_dt = dwt_get_delta(&usart_task_dwt);
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        // �����ȴ���һ�ź�������
        xActivatedMember = xQueueSelectFromSet(xUartQueueSet, portMAX_DELAY);

        // �жϴ���Դ���������ݣ����ݴ������ݵ���Ҫ�̶ȵ�������˳��
        if (xActivatedMember == xSemaphoreUART1_RX) {
            process_usart1_rx_data();  // �Զ��������
        } else if (xActivatedMember == xSemaphoreUART5_RX) {
            process_uart5_rx_data();  // ��˹ң����
        } else if (xActivatedMember == xSemaphoreUART10_RX) {
            process_uart10_rx_data(); // ����ϵͳ����ܣ�
        }

        vTaskDelay(1);
    }
    /* USER CODE END USARTRecTask_Entry */
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == UART5)
    {
        if (Size > SBUS_RX_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        usart5_rx_size = Size;
        usart5_rx_buffer_index = usart5_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE);

        xSemaphoreGiveFromISR(xSemaphoreUART5_RX, NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == USART10)
    {
        // �жϽ��յ����ݴ�С�Ƿ����ƣ�����������򲻴���
        if (Size > REFEREE_RX_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        referee_rx_size = Size;
        referee_rx_buffer_index = referee_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);

        xSemaphoreGiveFromISR(xSemaphoreUART10_RX,NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (huart->Instance == USART1)  // �޸��ж�����
    {
        // �жϽ��յ����ݴ�С�Ƿ����ƣ�����������򲻴���
        if (Size > CUSTOMER_CONTROLLER_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        usart1_rx_size = Size;
        usart1_rx_buffer_index = usart1_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1_rx_buffer[usart1_rx_buffer_index],CUSTOMER_CONTROLLER_BUF_SIZE);

        xSemaphoreGiveFromISR(xSemaphoreUART1_RX, NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// DMA������ɻص�����
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // ����Ƿ���USART1��DMA�������
    if (huart == &huart1) {
        // �ͷ��ź�������ʾDMA���Դ�����һ��������
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//        // ���ж����л�������
        usart7_tx_buffer_index = usart7_tx_buffer_index ^ 1;

        xSemaphoreGiveFromISR(xSemaphoreUART7_TX, &xHigherPriorityTaskWoken);
        // �����Ҫ���Ѹ������ȼ������򴥷��������л�
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // ���շ������������
        memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));							   // ������ջ���
    }

    if(huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE); // ���շ������������
        memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));// ���˫����
    }

    if(huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE); // ���շ������������
        memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));// ���˫����
    }
}

// �Զ���DebugPrintf����������printf��
void USART7_DebugPrintf(const char *format, ...) {
    va_list args;
    uint16_t length;
    uint8_t finishedBuffer;

    // �ȴ�DMA������ɣ���ȡ�ź�����
    if (xSemaphoreTake(xSemaphoreUART7_TX, portMAX_DELAY) == pdTRUE) {
        finishedBuffer = usart7_tx_buffer_index ^ 1; // �л�����һ��������

        // ʹ�ÿɱ������ʽ���ַ�������ǰ������
        va_start(args, format);
        length = vsnprintf(usart7_tx_debug_buffer[finishedBuffer], USART7_TX_DEBUG_BUFFER_SIZE, format, args);
        va_end(args);

        // ȷ���ַ���������Ч
        if (length > 0) {
            // ʹ��DMA�첽���͵�ǰ����������
            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)usart7_tx_debug_buffer[usart7_tx_buffer_index], length);
        } else {
            // ������Чʱֱ���ͷ��ź���
            xSemaphoreGive(xSemaphoreUART7_TX);
        }
    }
}