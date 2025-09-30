/**
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bmi3.h"
#include "stm32f4xx_hal.h"
#include "main.h"
/******************************************************************************/
/*!                Macro definitbion                                           */

#define READ_WRITE_LEN  UINT8_C(8)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
/* ===== Configure your I2C address here (7-bit) ===== */
#define BMI3_ADDR_7B   (0x68)    /* 0x68 if SDO=GND, 0x69 if SDO=VDDIO */

/* ===== Optional: change I2C instance if not hi2c1 ===== */
extern I2C_HandleTypeDef hi2c1;
#define BMI3_I2C_HANDLE  hi2c1

/* Bosch driver expects this field; keep a sensible length */
#define READ_WRITE_LEN  UINT8_C(32)

/* ---------- Forward declarations (HAL-backed) ---------- */
static BMI3_INTF_RET_TYPE bmi3_i2c_read (uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static BMI3_INTF_RET_TYPE bmi3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static BMI3_INTF_RET_TYPE bmi3_spi_read (uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static BMI3_INTF_RET_TYPE bmi3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void               bmi3_delay_us(uint32_t period, void *intf_ptr);

/* ---------- Helper: printf error decoder (unchanged) ---------- */
void bmi3_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI3_OK: break;
        case BMI3_E_NULL_PTR:
            printf("%s\tError [%d] : Null pointer\r\n", api_name, rslt); break;
        case BMI3_E_COM_FAIL:
            printf("%s\tError [%d] : Communication failure\r\n", api_name, rslt); break;
        case BMI3_E_DEV_NOT_FOUND:
            printf("%s\tError [%d] : Device not found (bad chip-id?)\r\n", api_name, rslt); break;
        case BMI3_E_INVALID_SENSOR:
            printf("%s\tError [%d] : Invalid sensor selection\r\n", api_name, rslt); break;
        case BMI3_E_INVALID_INT_PIN:
            printf("%s\tError [%d] : Invalid INT pin (only INT1/INT2)\r\n", api_name, rslt); break;
        case BMI3_E_ACC_INVALID_CFG:
            printf("%s\tError [%d] : Accel config invalid\r\n", api_name, rslt); break;
        case BMI3_E_GYRO_INVALID_CFG:
            printf("%s\tError [%d] : Gyro config invalid\r\n", api_name, rslt); break;
        case BMI3_E_INVALID_INPUT:
            printf("%s\tError [%d] : Invalid input\r\n", api_name, rslt); break;
        case BMI3_E_INVALID_STATUS:
            printf("%s\tError [%d] : Invalid status\r\n", api_name, rslt); break;
        case BMI3_E_DATA_RDY_INT_FAILED:
            printf("%s\tError [%d] : DRDY error\r\n", api_name, rslt); break;
        case BMI3_E_INVALID_FOC_POSITION:
            printf("%s\tError [%d] : FOC position invalid\r\n", api_name, rslt); break;
        case BMI3_E_INVALID_ST_SELECTION:
            printf("%s\tError [%d] : Self-test selection invalid\r\n", api_name, rslt); break;
        case BMI3_E_OUT_OF_RANGE:
            printf("%s\tError [%d] : Out of range\r\n", api_name, rslt); break;
        case BMI3_E_FEATURE_ENGINE_STATUS:
            printf("%s\tError [%d] : Feature engine enable mask missing\r\n", api_name, rslt); break;
        default:
            printf("%s\tError [%d] : Unknown\r\n", api_name, rslt); break;
    }
}

/* Bind Bosch */
int8_t bmi3_interface_init(struct bmi3_dev *dev, int8_t intf)
{
    if (dev == NULL) return BMI3_E_NULL_PTR;

    /* Configure delay callback (Bosch often requests us-level delays; coarse ms is OK) */
    dev->delay_us = bmi3_delay_us;

    /* Configure max read/write length (not strictly required on MCU, but set anyway) */
    dev->read_write_len = READ_WRITE_LEN;

    /* Choose interface: prefer I2C for bring-up */
    if (intf == BMI3_I2C_INTF)
    {
        dev->intf     = BMI3_I2C_INTF;
        dev->read     = bmi3_i2c_read;
        dev->write    = bmi3_i2c_write;

        /* Pass the HAL handle as intf_ptr (driver passes it back to our hooks) */
        dev->intf_ptr = (void*)&BMI3_I2C_HANDLE;
    }
    else if (intf == BMI3_SPI_INTF)
    {
        dev->intf     = BMI3_SPI_INTF;
        dev->read     = bmi3_spi_read;   /* currently stubbed to return error */
        dev->write    = bmi3_spi_write;  /* currently stubbed to return error */
        dev->intf_ptr = NULL;            /* set later if you implement SPI */
    }
    else
    {
        return BMI3_E_INVALID_INPUT;
    }

    return BMI3_OK;
}

/* ========== HAL-backed implementations ========== */
/* I2C: use HAL_I2C_Mem_Read/Write, note HAL uses 8-bit address */
static BMI3_INTF_RET_TYPE bmi3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef*)intf_ptr;
    if (hi2c == NULL) hi2c = &BMI3_I2C_HANDLE;

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c,
                                            (BMI3_ADDR_7B << 1), /* 8-bit addr */
                                            reg_addr,
                                            I2C_MEMADD_SIZE_8BIT,
                                            reg_data,
                                            (uint16_t)len,
                                            1000);
    return (st == HAL_OK) ? BMI3_INTF_RET_SUCCESS : BMI3_E_COM_FAIL;
}

static BMI3_INTF_RET_TYPE bmi3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef*)intf_ptr;
    if (hi2c == NULL) hi2c = &BMI3_I2C_HANDLE;

    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(hi2c,
                                             (BMI3_ADDR_7B << 1), /* 8-bit addr */
                                             reg_addr,
                                             I2C_MEMADD_SIZE_8BIT,
                                             (uint8_t*)reg_data,
                                             (uint16_t)len,
                                             1000);
    return (st == HAL_OK) ? BMI3_INTF_RET_SUCCESS : BMI3_E_COM_FAIL;
}

/* SPI: leave as stubs until you wire SPI + CS pin; prevents link errors */
static BMI3_INTF_RET_TYPE bmi3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)reg_addr; (void)reg_data; (void)len; (void)intf_ptr;
    return BMI3_E_COM_FAIL;
}

static BMI3_INTF_RET_TYPE bmi3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)reg_addr; (void)reg_data; (void)len; (void)intf_ptr;
    return BMI3_E_COM_FAIL;
}

/* Delay: Bosch passes microseconds; we approximate using HAL_Delay(ms) */
static void bmi3_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    HAL_Delay((period + 999) / 1000);
}
