/*
 * adani_can_medical_protocol.h
 *
 *  Created on: 10 дек. 2018 г.
 *      Author: zilkov
 */

#ifndef ADANI_CAN_MEDICAL_PROTOCOL_H_
#define ADANI_CAN_MEDICAL_PROTOCOL_H_
#include "stm32h7xx_hal.h"
//GPIO Output pins switch on/off
uint8_t can_protocol_data_analyzing(FDCAN_HandleTypeDef *hfdcan,
								FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData);

////++++++++++++++++++++++++++++++++++++++++++
/* CAN:
 * ID = Action (0x7E0) + address (0x1F) --- masks
 * Action:
 * 0x620 - DigOutputs | DigInputs
 * 0x640 - ADCs
 * 0x660 - SSI
 * 0x680 - SPI
 * 0x6A0 - rs485
 * 0x6C0 -
 * 0x6E0 -
 */
////++++++++++++++++++++++++++++++++++++++++++

#define MASTER_ADR			0x16


#endif /* ADANI_CAN_MEDICAL_PROTOCOL_H_ */
