/*
 * lis2dh12_read_data_polling.h
 *
 *  Created on: Apr 28, 2021
 *      Author: Josef
 */

#ifndef INC_LIS2DH12_READ_DATA_POLLING_H_
#define INC_LIS2DH12_READ_DATA_POLLING_H_

/* Extern variables ----------------------------------------------------------*/

extern uint8_t TxData;
extern uint8_t RxData;

void lis2dh12_read_data_polling(void);


#endif /* INC_LIS2DH12_READ_DATA_POLLING_H_ */
