/*
 * my_lis.h
 *
 *  Created on: 27 Apr 2021
 *      Author: Josef
 */

#ifndef INC_MY_LIS_H_
#define INC_MY_LIS_H_

#include "main.h"
#include "lis2dh12_reg.h"
#include "stm32l4xx_hal.h"

#define    BOOT_TIME            5 //ms

void LIS_init(void);
void lis2dh12_read_data_polling(void);
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

#endif /* INC_MY_LIS_H_ */
