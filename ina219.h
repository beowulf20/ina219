#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

typedef enum
{
    INA219_OK,
    INA219_ERR_INVALID_ADDRESS,
    INA219_ERR_INVALID_ARGS,
    INA219_ERR_INVALID_CONFIG,
} ina219_err_t;

typedef enum
{
    INA219_REG_CONFIGURATION = 0,
    INA219_REG_SHUNT_VOLTAGE,
    INA219_REG_BUS_VOLTAGE,
    INA219_REG_POWER,
    INA219_REG_CURRENT,
    INA219_REG_CALIBRATION
} ina219_reg_t;

typedef enum
{
    INA219_BRNG_16V,
    INA219_BRNG_32V
} ina219_brng_t;

typedef enum
{
    INA219_PG_40mV = 0,
    INA219_PG_80mV,
    INA219_PG_160mV,
    INA219_PG_320mV
} ina219_pg_t;

typedef enum
{
    INA219_ADC_84us = 0,
    INA219_ADC_148us,
    INA219_ADC_276us,
    INA219_ADC_532us,
    INA219_ADC_1060us = 0b1001,
    INA219_ADC_2130us,
    INA219_ADC_4260us,
    INA219_ADC_8510us,
    INA219_ADC_17020us,
    INA219_ADC_34050us,
    INA219_ADC_68100us,
} ina219_adc_t;

typedef enum
{
    INA219_MODE_POWER_DOWN,
    INA219_MODE_SHUNT_TRIGGER,
    INA219_MODE_BUS_TRIGGER,
    INA219_MODE_SHUNT_BUS_TRIGGER,
    INA219_MODE_ADC_OFF,
    INA219_MODE_SHUNT_CONTINUOUS,
    INA219_MODE_BUS_CONTINUOUS,
    INA219_MODE_SHUNT_BUS_CONTINUOUS
} ina219_mode_t;

typedef struct ina219_reg_config_t
{
    bool RESET;
    uint8_t RESERVED;
    ina219_brng_t BUS_VOLTAGE_RANGE;
    ina219_pg_t PG;
    ina219_adc_t BADC;
    ina219_adc_t SADC;
    ina219_mode_t MODE;
} ina219_reg_config_t;

typedef struct ina219_i2c_calibration_t
{
    float _cur;
} ina219_i2c_calibration_t;

typedef struct ina219_handle_t
{
    ina219_i2c_calibration_t cal;
    i2c_port_t port;
    uint8_t addr;
    ina219_reg_config_t cfg;
} ina219_handle_t;

ina219_err_t ina219_init(i2c_port_t i2c_port, uint8_t addr, ina219_handle_t *handle);
ina219_err_t ina219_set_config(ina219_handle_t *ina219, ina219_reg_config_t cfg, uint16_t timeout_ms);
ina219_err_t ina219_reset_config(ina219_handle_t *ina219, uint16_t timeout_ms);
ina219_err_t ina219_read_config(ina219_handle_t *ina219, ina219_reg_config_t *cfg, uint16_t timeout_ms);
ina219_err_t ina219_read_shunt_voltage(ina219_handle_t *ina219, float *voltage, uint16_t timeout_ms);
ina219_err_t ina219_read_bus_voltage(ina219_handle_t *ina219, float *voltage, uint16_t timeout_ms);
ina219_err_t ina219_read_power(ina219_handle_t *ina219, float *power, uint16_t timeout_ms);
ina219_err_t ina219_read_current(ina219_handle_t *ina219, float *current, uint16_t timeout_ms);
ina219_err_t ina219_calibrate(ina219_handle_t *ina219, float shuntR, float maxCurrent, uint16_t timeout_ms);