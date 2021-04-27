/*
 * my_lis.c
 *
 *  Created on: 27 Apr 2021
 *      Author: Josef
 */


#include "my_lis.h"


#define NUCLEO_F411RE
#define SENSOR_BUS hi2c1

static int16_t data_raw_acceleration[3];
//static int16_t data_raw_temperature;
//static float temperature_degC;
static float acceleration_mg[3];
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

void lis2dh12_read_data_polling(void)
{
	stmdev_ctx_t dev_ctx;

	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &SENSOR_BUS;
  /* Read samples in polling mode (no int) */

    lis2dh12_reg_t reg;

    /* Read output only if new value available */
    lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);
    if (reg.byte) {
      /* Read accelerometer data */
      memset(data_raw_acceleration, 0x00, 3*sizeof(int16_t));
      lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
    		  lis2dh12_from_fs16_lp_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
    		  lis2dh12_from_fs16_lp_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
    		  lis2dh12_from_fs16_lp_to_mg(data_raw_acceleration[2]);

      sprintf((char*)tx_buffer, "Acceleration [mg]: %4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
	  //HAL_UART_Transmit(&huart2, tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
    }
// comment out Temperature read
//    lis2dh12_temp_data_ready_get(&dev_ctx, &reg.byte);
//    if (reg.byte) {
//      /* Read temperature data */
//      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//      lis2dh12_temperature_raw_get(&dev_ctx, &data_raw_temperature);
//      temperature_degC =
//        lis2dh12_from_lsb_hr_to_celsius(data_raw_temperature);
//
//      sprintf((char*)tx_buffer,
//              "Temperature [degC]:%6.2f\r\n",
//              temperature_degC);
//      tx_com(tx_buffer, strlen((char const*)tx_buffer));
//    }

}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
#if defined(NUCLEO_F411RE)
  /* Write multiple command */
  reg |= 0x80;
//  HAL_I2C_Mem_Write(handle, LIS2DH12_I2C_ADD_L, reg,
//                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	HAL_I2C_Mem_Write(handle, LIS2DH12_I2C_ADD_H, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Write multiple command */
  reg |= 0x40;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Write multiple command */
  reg |= 0x80;
  i2c_lld_write(handle,  LIS2DH12_I2C_ADD_L & 0xFE, reg, bufp, len);
#endif
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
#if defined(NUCLEO_F411RE)
  /* Read multiple command */
  reg |= 0x80;
//  HAL_I2C_Mem_Read(handle, LIS2DH12_I2C_ADD_L, reg,
//                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  HAL_I2C_Mem_Read(handle, LIS2DH12_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Read multiple command */
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Read multiple command */
  reg |= 0x80;
  i2c_lld_read(handle, LIS2DH12_I2C_ADD_L & 0xFE, reg, bufp, len);
#endif
  return 0;
}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(STEVAL_MKI109V3)
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#endif
}

static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F411RE) | defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}

void LIS_init(void)
  {
  /* Initialize mems driver interface */
    stmdev_ctx_t dev_ctx;

    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &SENSOR_BUS;

    /* Wait boot time and initialize platform specific hardware */
    platform_init();

    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);

    /* Check device ID */
    lis2dh12_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LIS2DH12_ID){
      while(1) {
        /* manage here device not found */
      }
    }

    /* Enable Block Data Update. */
    lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set Output Data Rate to 1Hz. */
    lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP);

    /* Set full scale to 2g. */
    lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_16g);

    /* Enable temperature sensor. */
    lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_DISABLE);

    /* Set device in continuous mode with 12 bit resol. */
    lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_LP_8bit);
  }
