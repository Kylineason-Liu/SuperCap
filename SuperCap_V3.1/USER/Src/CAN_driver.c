#include "math.h"
#include "fdcan.h"
#include "CAN_driver.h"
#include "power_calculation.h"

uint8_t CAN_RXbuff[8];
uint8_t CAN_TXbuff[8];

/**
 * @description: CAN�����ն˻ص��������������ƹ���
 * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
 *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
 * @return: void
 * @note: 
 */ 
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	static FDCAN_RxHeaderTypeDef RxHeader;
	
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN_RXbuff);
    if (RxHeader.Identifier == RECEIVE_ID)
    {
		buckboost.max_power_lim = (uint16_t)( CAN_RXbuff[RECEIVE_NUM * 2] << 8 | CAN_RXbuff[RECEIVE_NUM * 2 + 1]);
		if (buckboost.max_power_lim < 40)
			buckboost.max_power_lim = 40;
		else
			buckboost.max_power_lim = buckboost.max_power_lim - 1;
	}
}

/**
 * @description: CAN���ͺ���
 * @param  FDCAN_HandleTypeDefָ��
 * @param  ����CANID
 * @param  �������ݳ���
 * @param  ������������
 * @return: void
 * @note: 
 */ 
void FCAN_transmit(FDCAN_HandleTypeDef *hfdcan,uint32_t can_id, uint32_t DataLength, uint8_t tx_data[])
{
	static FDCAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.Identifier = can_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = DataLength;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;	//marker++;
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_data);
}

/**
 * @description: ����λ�����͵��ݵ�ѹ����ǰ�������ơ���ǰ������ʵȲ���
 * @param {unused} 
 * @return: void
 * @note: 
 */ 
void power_send()
{
	CAN_TXbuff[0] = (uint8_t)((uint16_t)(cap.U_now*100));
	CAN_TXbuff[1] = (uint8_t)((uint16_t)(cap.U_now*100)>> 8);
	CAN_TXbuff[2] = (uint8_t) buckboost.max_power_lim;
	CAN_TXbuff[3] = (uint8_t)(buckboost.max_power_lim >> 8);
	CAN_TXbuff[4] = (uint8_t)((uint16_t)(out.P_now*100));
	CAN_TXbuff[5] = (uint8_t)((uint16_t)(out.P_now*100) >> 8);
	CAN_TXbuff[6] = (uint8_t)(((uint16_t)(fabs(dcdc_in.P_now*100))));
	CAN_TXbuff[7] = (uint8_t)((uint16_t)(fabs(dcdc_in.P_now*100)) >> 8);

	FCAN_transmit(&hfdcan2, SEND_ID, FDCAN_DLC_BYTES_8, CAN_TXbuff);
}

void FDCAN_FilterInit()
{
	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x7FF;
	
	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

void FDCAN_start()
{
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
