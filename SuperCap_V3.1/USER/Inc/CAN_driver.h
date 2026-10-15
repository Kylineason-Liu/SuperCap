#ifndef _CAN_DRIVER_H
#define _CAN_DRIVER_H

#define SEND_ID			0x211		//回传ID
#define RECEIVE_ID		0x1fe		//接收ID
#define RECEIVE_NUM		1			//接收数据位数(0~3)

//英雄 0x2ff 1(第二位)
//步兵 0x1fe 1(第二位)
//腿车 0x200 2(第三位)
//哨兵 0x2fe 1(第二位)

void FDCAN_FilterInit(void);
void FDCAN_start(void);
void FCAN_transmit(FDCAN_HandleTypeDef *hfdcan,uint32_t can_id, uint32_t DataLength, uint8_t tx_data[]);
void power_send(void);

#endif
