#include "ina219.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>


bool ina219_check_address(uint8_t addr)
{
    return addr >= 64 && addr <= 80;
}

ina219_err_t
ina219_read_reg(ina219_handle_t *ina219, ina219_reg_t reg, uint16_t *reg_value, uint16_t timeout_ms)
{
    if (ina219 == NULL)
    {
        return INA219_ERR_INVALID_ARGS;
    }
    if (!ina219_check_address(ina219->addr))
    {
        return INA219_ERR_INVALID_ADDRESS;
    }
    if (reg > INA219_REG_CALIBRATION || reg_value == NULL)
    {
        return INA219_ERR_INVALID_ARGS;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ina219->addr) << 1 | I2C_MASTER_WRITE, 0);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ina219->port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        // ESP_LOGE(__func__, "ret0: %d", ret);
        return ret;
    }

    uint8_t data[2];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ina219->addr) << 1 | I2C_MASTER_READ, 0);
    i2c_master_read(cmd, data, 2, 0);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(ina219->port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        // ESP_LOGE(__func__, "ret1: %d", ret);
        return ret;
    }

    *reg_value = (data[0] << 8) | data[1];

    return INA219_OK;
}

ina219_err_t ina219_write_reg(ina219_handle_t *ina219, ina219_reg_t reg, uint16_t reg_value, uint16_t timeout_ms)
{
    if (ina219 == NULL)
    {
        return INA219_ERR_INVALID_ARGS;
    }

    if (reg > INA219_REG_CALIBRATION)
    {
        return INA219_ERR_INVALID_ARGS;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ina219->addr) << 1 | I2C_MASTER_WRITE, 0);
    i2c_master_write_byte(cmd, reg, 0);
    i2c_master_write_byte(cmd, reg_value >> 8, 0);
    i2c_master_write_byte(cmd, reg_value & 0x00FF, 1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ina219->port, cmd, pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return INA219_OK;
}

ina219_err_t ina219_read_config(ina219_handle_t *ina219, ina219_reg_config_t *cfg, uint16_t timeout_ms)
{
    uint16_t reg_value = 0;
    ina219_err_t ret = ina219_read_reg(ina219, INA219_REG_CONFIGURATION, &reg_value, timeout_ms);
    if (ret != INA219_OK)
    {
        return ret;
    }

    cfg->RESET = BIT_SELECT(reg_value, 15);
    cfg->RESERVED = BIT_SELECT(reg_value, 14);
    cfg->BUS_VOLTAGE_RANGE = BIT_SELECT(reg_value, 13);
    cfg->PG = (BIT_SELECT(reg_value, 12) << 1) | (BIT_SELECT(reg_value, 11) << 0);
    cfg->BADC = (BIT_SELECT(reg_value, 10) << 3) | (BIT_SELECT(reg_value, 9) << 2) | (BIT_SELECT(reg_value, 8) << 1) | (BIT_SELECT(reg_value, 7) << 0);
    cfg->SADC = (BIT_SELECT(reg_value, 6) << 3) | (BIT_SELECT(reg_value, 5) << 2) | (BIT_SELECT(reg_value, 4) << 1) | (BIT_SELECT(reg_value, 3) << 0);
    cfg->MODE = (BIT_SELECT(reg_value, 2) << 2) | (BIT_SELECT(reg_value, 1) << 1) | (BIT_SELECT(reg_value, 0) << 0);

    return INA219_OK;
}

ina219_err_t ina219_read_shunt_voltage(ina219_handle_t *ina219, float *voltage, uint16_t timeout_ms)
{
    uint16_t reg_value = 0;

    ina219_err_t ret = ina219_read_reg(ina219, INA219_REG_SHUNT_VOLTAGE, &reg_value, timeout_ms);
    if (ret != INA219_OK)
    {
        return ret;
    }

    *voltage = ((int16_t)reg_value) * 0.00001;

    return INA219_OK;
}

ina219_err_t ina219_read_bus_voltage(ina219_handle_t *ina219, float *voltage, uint16_t timeout_ms)
{
    uint16_t reg_value = 0;

    ina219_err_t ret = ina219_read_reg(ina219, INA219_REG_BUS_VOLTAGE, &reg_value, timeout_ms);
    if (ret != INA219_OK)
    {
        return ret;
    }
    int16_t vv = (int16_t)((reg_value >> 3) * 4);
    *voltage = vv * 0.001;

    return INA219_OK;
}

ina219_err_t ina219_read_power(ina219_handle_t *ina219, float *power, uint16_t timeout_ms)
{
    uint16_t reg_value = 0;
    ina219_err_t ret = ina219_read_reg(ina219, INA219_REG_POWER, &reg_value, timeout_ms);
    if (ret != INA219_OK)
    {
        return ret;
    }
    *power = 20.0 * reg_value * ina219->cal._cur;

    return INA219_OK;
}

ina219_err_t ina219_read_current(ina219_handle_t *ina219, float *current, uint16_t timeout_ms)
{
    uint16_t reg_value = 0;
    ina219_err_t ret = ina219_read_reg(ina219, INA219_REG_CURRENT, &reg_value, timeout_ms);
    if (ret != INA219_OK)
    {
        return ret;
    }

    *current = reg_value * ina219->cal._cur;

    return INA219_OK;
}

ina219_err_t ina219_calibrate(ina219_handle_t *ina219, float shuntR, float maxCurrent, uint16_t timeout_ms)
{
    if (shuntR == 0 || maxCurrent == 0 || ina219 == NULL)
    {
        return INA219_ERR_INVALID_ARGS;
    }
    float currentLSB = maxCurrent / pow(2, 15);
    uint16_t cal = (uint16_t)trunc(.04096 / (shuntR * currentLSB));
    if (cal > pow(2, 15))
    {
        ESP_LOGW(__func__, "not possible to calibrate for %.3fA, using %.3fA instead", maxCurrent, .04096 / shuntR);
        cal = 32768;
    }

    ina219->cal._cur = .04096 / (shuntR * cal);

    ina219_err_t ret = ina219_write_reg(ina219, INA219_REG_CALIBRATION, cal << 1, timeout_ms);
    if (ret != INA219_OK)
    {
        return ret;
    }

    // uint16_t reg_value = 0;
    // ret = ina219_read_reg(ina219, INA219_REG_CALIBRATION, &reg_value, timeout_ms);
    // if (ret != INA219_OK)
    // {
    //     return ret;
    // }
    // ESP_LOGD(__func__,"%04X",reg_value);

    return INA219_OK;
}

ina219_err_t ina219_init(i2c_port_t i2c_port, uint8_t addr, ina219_handle_t *handle)
{
    if (!ina219_check_address(addr))
    {
        return INA219_ERR_INVALID_ADDRESS;
    }
    if (handle == NULL)
    {
        return INA219_ERR_INVALID_ARGS;
    }

    if (i2c_port >= I2C_NUM_MAX)
    {
        return INA219_ERR_INVALID_ARGS;
    }

    handle->addr = 0x45;
    handle->port = i2c_port;

    return INA219_OK;
}

ina219_err_t ina219_set_config(ina219_handle_t *ina219, ina219_reg_config_t cfg, uint16_t timeout_ms)
{

    if (cfg.BUS_VOLTAGE_RANGE > INA219_BRNG_32V)
    {
        return INA219_ERR_INVALID_CONFIG;
    }

    if (cfg.PG > INA219_PG_320mV)
    {
        return INA219_ERR_INVALID_CONFIG;
    }
    if (cfg.BADC > INA219_ADC_68100us || cfg.SADC > INA219_ADC_68100us)
    {
        return INA219_ERR_INVALID_CONFIG;
    }

    if (cfg.MODE > INA219_MODE_SHUNT_BUS_CONTINUOUS)
    {
        return INA219_ERR_INVALID_CONFIG;
    }
    // cfg.RESET = true;
    uint16_t cfg_data = (cfg.RESET << 15) | (cfg.RESERVED << 14) | (cfg.BUS_VOLTAGE_RANGE << 13) | (cfg.PG << 11) | (cfg.BADC << 7) | (cfg.SADC << 3) | (cfg.MODE << 0);
    // uint16_t cfg_data = 0;
    ina219_err_t ret = ina219_write_reg(ina219, INA219_REG_CONFIGURATION, cfg_data, timeout_ms);
    if (ret != INA219_OK)
    {
        return ret;
    }

    ina219->cfg = cfg;

    return INA219_OK;
}

ina219_err_t ina219_reset_config(ina219_handle_t *ina219, uint16_t timeout_ms)
{
    return ina219_write_reg(ina219, INA219_REG_CONFIGURATION, 0x8000, timeout_ms);
}
