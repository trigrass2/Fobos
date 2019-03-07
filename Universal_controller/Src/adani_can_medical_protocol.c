/*
 * adani_can_medical_protocol.c
 *
 *  Created on: 11 дек. 2018 г.
 *      Author: zilkov
 */

#include "adani_can_medical_protocol.h"
#include "own_defines.h"

uint8_t can_protocol_data_analyzing(FDCAN_HandleTypeDef *hfdcan,
								FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData)
{
	//uint32_t level = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
	if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, pRxHeader, pRxData) == HAL_OK)
	//if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_BUFFER0, pRxHeader, pRxData) == HAL_OK)
	{
		uint32_t address;
		address = pRxHeader->Identifier;

		static char a=0;
		  LED_VD6(a^=1);
		return 0xFF;
	}
	else
		return 0;
}
