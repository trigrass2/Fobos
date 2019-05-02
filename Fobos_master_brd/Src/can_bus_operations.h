/*
 * can_bus_operations.h
 *
 *  Created on: 2 мая 2019 г.
 *      Author: zilkov
 */

#ifndef CAN_BUS_OPERATIONS_H_
#define CAN_BUS_OPERATIONS_H_

uint8_t can_bus_tx(uint16_t ID, uint8_t length, uint8_t *data);

#endif /* CAN_BUS_OPERATIONS_H_ */
