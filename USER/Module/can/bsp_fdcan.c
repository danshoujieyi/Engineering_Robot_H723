#include "bsp_fdcan.h"
#include "dj_motor.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

static FDCAN_TxHeaderTypeDef  tx_message;

uint8_t CAN_send(FDCAN_HandleTypeDef *can, uint32_t send_id, uint8_t data[])
{
    // fdcanx_send_data(can, send_id, data, 8);
    // uint32_t send_mail_box;
    tx_message.Identifier=send_id;
    tx_message.IdType=FDCAN_STANDARD_ID;
    tx_message.TxFrameType=FDCAN_DATA_FRAME;
    tx_message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch=FDCAN_BRS_ON;//FDCAN_BRS_OFF;
    tx_message.FDFormat=FDCAN_FD_CAN;//FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    tx_message.MessageMarker=0;
//    if(len<=8)
    tx_message.DataLength = FDCAN_DLC_BYTES_8;
//    if(len==12)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
//    if(len==16)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
//    if(len==20)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
//    if(len==24)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
//    if(len==32)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
//    if(len==48)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
//    if(len==64)
//        pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
    /* HAL API*/
    if(HAL_FDCAN_AddMessageToTxFifoQ(can, &tx_message, data)!=HAL_OK)
        return 1;
    return 0;
}


/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	open can dev and enable can interrupt 
************************************************************************
**/
void bsp_can_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //open FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);

    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	can filter init
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //STD ID
	fdcan_filter.FilterIndex = 0;                                  //filter index                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           // filter0 to FIFO0  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;

	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
    HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);


    HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：fdcan struct
* @param:       id：CAN ID
* @param:       data
* @param:       len：data length
* @retval:     	0: send successful	1: send fail
* @details:    	send data
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	/* HAL API*/
	if(len<=8)
		pTxHeader.DataLength = len;
	if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
	/* HAL API*/
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
		return 1;
	return 0;	
}
/**
************************************************************************
* @brief:      	fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
* @param:       hfdcan：fdcan struct
* @param:       rec_id: receive id
* @param:       buf：receive data buffef
* @retval:     	receive data length
* @details:    	receive can data
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
			len = pRxHeader.DataLength;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_12)
			len = 12;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_16)
			len = 16;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_20)
			len = 20;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_24)
			len = 24;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_32)
			len = 32;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_48)
			len = 48;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_64)
			len = 64;
		
		return len;
	}
	return 0;	
}

//// can interrupt callback
//__weak void fdcan1_rx_callback(void)
//{
//
//}
//// can interrupt callback
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//    if (hfdcan == &hfdcan1)
//    {
//		fdcan1_rx_callback();
//    }
//}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        if (hfdcan == &hfdcan1)
        {
             HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
            dji_motot_rx_callback(rx_header.Identifier, rx_data);
             //             if(dji_motot_rx_callback(rx_header.Identifier, rx_data) == 0)
//                 return;
        }
    }
}

//void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo1ITs)
//{
//    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
//    {
//        FDCAN_RxHeaderTypeDef rx_header;
//        uint8_t rx_data[8];
//        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1)) // FIFO不为空,有可能在其他中断时有多帧数据进入
//        {
//            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);
//            // TODO: 2024 RMUL 期间部署上下板间通讯，之后需要优化调整
//            if (hfdcan == &hfdcan2)
//            {
//                 if(dji_motot_rx_callback(rx_header.Identifier, rx_data) == 0)
//                     return;
////                if(chassis_board_rx_callback(rx_header.StdId, rx_data) == 0)
////                    return;
//            }
//        }
//    }
//}












