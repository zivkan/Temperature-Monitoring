﻿// Copied from:
// https://github.com/BoschSensortec/BME280_driver/blob/bdf4e573cbc77dc1d30a24845d0c1e35eebc84fa/bme280.c
// https://github.com/BoschSensortec/BME280_driver/blob/bdf4e573cbc77dc1d30a24845d0c1e35eebc84fa/bme280.h
// https://github.com/BoschSensortec/BME280_driver/blob/bdf4e573cbc77dc1d30a24845d0c1e35eebc84fa/bme280_defs.h

/**\mainpage
 * Copyright (C) 2018 - 2019 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * File     bme280.c
 * Date     26 Aug 2019
 * Version  3.3.7
 *
 */

/*! @file bme280.c
 * @brief Sensor driver for BME280 sensor
 */
#include "bme280.h"

/**\name Internal macros */
/* To identify osr settings selected by user */
#define OVERSAMPLING_SETTINGS   UINT8_C(0x07)

/* To identify filter and standby settings selected by user */
#define FILTER_STANDBY_SETTINGS UINT8_C(0x18)

/*!
 * @brief This internal API puts the device to sleep mode.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t put_device_to_sleep(const struct bme280_dev * dev);

/*!
 * @brief This internal API writes the power mode in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] sensor_mode : Variable which contains the power mode to be set.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t write_power_mode(uint8_t sensor_mode, const struct bme280_dev * dev);

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t null_ptr_check(const struct bme280_dev * dev);

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 *
 * @param[in] reg_addr : Contains the register address array.
 * @param[out] temp_buff : Contains the temporary buffer to store the
 * register data and register address.
 * @param[in] reg_data : Contains the register data to be written in the
 * temporary buffer.
 * @param[in] len : No of bytes of data to be written for burst write.
 */
static void interleave_reg_addr(const uint8_t* reg_addr, uint8_t* temp_buff, const uint8_t* reg_data, uint8_t len);

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_calib_data(struct bme280_dev * dev);

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in the device structure.
 *
 *  @param[out] dev : Structure instance of bme280_dev to store the calib data.
 *  @param[in] reg_data : Contains the calibration data to be parsed.
 */
static void parse_temp_press_calib_data(const uint8_t* reg_data, struct bme280_dev * dev);

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 *
 *  @param[out] dev : Structure instance of bme280_dev to store the calib data.
 *  @param[in] reg_data : Contains calibration data to be parsed.
 */
static void parse_humidity_calib_data(const uint8_t* reg_data, struct bme280_dev * dev);

#ifdef BME280_FLOAT_ENABLE

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated pressure data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Compensated pressure data.
 * @retval Compensated pressure data in double.
 */
static double compensate_pressure(const struct bme280_uncomp_data * uncomp_data,
                                  const struct bme280_calib_data * calib_data);

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated humidity data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Compensated humidity data.
 * @retval Compensated humidity data in double.
 */
static double compensate_humidity(const struct bme280_uncomp_data * uncomp_data,
                                  const struct bme280_calib_data * calib_data);

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated temperature data.
 * @param[in] calib_data : Pointer to calibration data structure.
 *
 * @return Compensated temperature data.
 * @retval Compensated temperature data in double.
 */
static double compensate_temperature(const struct bme280_uncomp_data * uncomp_data,
                                     struct bme280_calib_data * calib_data);

#else

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated temperature data.
 * @param[in] calib_data : Pointer to calibration data structure.
 *
 * @return Compensated temperature data.
 * @retval Compensated temperature data in integer.
 */
static int32_t compensate_temperature(const struct bme280_uncomp_data * uncomp_data,
                                      struct bme280_calib_data * calib_data);

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated pressure data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Compensated pressure data.
 * @retval Compensated pressure data in integer.
 */
static uint32_t compensate_pressure(const struct bme280_uncomp_data * uncomp_data,
                                    const struct bme280_calib_data * calib_data);

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 *
 * @param[in] uncomp_data : Contains the uncompensated humidity data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Compensated humidity data.
 * @retval Compensated humidity data in integer.
 */
static uint32_t compensate_humidity(const struct bme280_uncomp_data * uncomp_data,
                                    const struct bme280_calib_data * calib_data);

#endif

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 *
 * @param[in] sub_settings : Contains the settings subset to identify particular
 * group of settings which the user is interested to change.
 * @param[in] desired_settings : Contains the user specified settings.
 *
 * @return Indicates whether user is interested to modify the settings which
 * are related to sub_settings.
 * @retval True -> User wants to modify this group of settings
 * @retval False -> User does not want to modify this group of settings
 */
static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings);

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_osr_humidity_settings(const struct bme280_settings * settings, const struct bme280_dev * dev);

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 *
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set.
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_osr_settings(uint8_t desired_settings,
                               const struct bme280_settings * settings,
                               const struct bme280_dev * dev);

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] desired_settings: variable to select the pressure and/or
 * temperature oversampling settings.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_osr_press_temp_settings(uint8_t desired_settings,
                                          const struct bme280_settings * settings,
                                          const struct bme280_dev * dev);

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] reg_data : Variable which is filled according to the pressure
 * oversampling data provided by the user.
 */
static void fill_osr_press_settings(uint8_t* reg_data, const struct bme280_settings * settings);

/*!
 * @brief This internal API fills the temperature oversampling settings provided
 * by the user in the data buffer so as to write in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] reg_data : Variable which is filled according to the temperature
 * oversampling data provided by the user.
 */
static void fill_osr_temp_settings(uint8_t* reg_data, const struct bme280_settings * settings);

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] desired_settings : variable to select the filter and/or
 * standby duration settings.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_filter_standby_settings(uint8_t desired_settings,
                                          const struct bme280_settings * settings,
                                          const struct bme280_dev * dev);

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] reg_data : Variable which is filled according to the filter
 * settings data provided by the user.
 */
static void fill_filter_settings(uint8_t* reg_data, const struct bme280_settings * settings);

/*!
 * @brief This internal API fills the standby duration settings provided by the
 * user in the data buffer so as to write in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] reg_data : Variable which is filled according to the standby
 * settings data provided by the user.
 */
static void fill_standby_settings(uint8_t* reg_data, const struct bme280_settings * settings);

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * device structure.
 *
 * @param[out] dev : Structure instance of bme280_dev.
 * @param[in] reg_data : Register data to be parsed.
 */
static void parse_device_settings(const uint8_t* reg_data, struct bme280_settings * settings);

/*!
 * @brief This internal API reloads the already existing device settings in the
 * sensor after soft reset.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] settings : Pointer variable which contains the settings to
 * be set in the sensor.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t reload_device_settings(const struct bme280_settings * settings, const struct bme280_dev * dev);

/****************** Global Function Definitions *******************************/

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 */
int8_t bme280_init(struct bme280_dev * dev)
{
    int8_t rslt;

/* chip id read try count */
uint8_t try_count = 5;
uint8_t chip_id = 0;

/* Check for null pointer in the device structure*/
rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        while (try_count)
        {
            /* Read the chip-id of bme280 sensor */
            rslt = bme280_get_regs(BME280_CHIP_ID_ADDR, &chip_id, 1, dev);

            /* Check for chip id validity */
            if ((rslt == BME280_OK) && (chip_id == BME280_CHIP_ID))
            {
                dev->chip_id = chip_id;

                /* Reset the sensor */
                rslt = bme280_soft_reset(dev);
                if (rslt == BME280_OK)
                {
                    /* Read the calibration data */
                    rslt = get_calib_data(dev);
                }
                break;
            }

            /* Wait for 1 ms */
            dev->delay_ms(1);
            --try_count;
        }

        /* Chip id check failed */
        if (!try_count)
        {
            rslt = BME280_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t* reg_data, uint16_t len, const struct bme280_dev * dev)
{
    int8_t rslt;

/* Check for null pointer in the device structure*/
rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        /* If interface selected is SPI */
        if (dev->intf != BME280_I2C_INTF)
        {
            reg_addr = reg_addr | 0x80;
        }

        /* Read the data  */
        rslt = dev->read(dev->dev_id, reg_addr, reg_data, len);

        /* Check for communication error */
        if (rslt != BME280_OK)
        {
            rslt = BME280_E_COMM_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t bme280_set_regs(uint8_t* reg_addr, const uint8_t* reg_data, uint8_t len, const struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t temp_buff[20]; /* Typically not to write more than 10 registers */

    if (len > 10)
    {
        len = 10;
    }
    uint16_t temp_len;
uint8_t reg_addr_cnt;

/* Check for null pointer in the device structure*/
rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if ((rslt == BME280_OK) && (reg_addr != NULL) && (reg_data != NULL))
    {
        if (len != 0)
        {
            temp_buff[0] = reg_data[0];

            /* If interface selected is SPI */
            if (dev->intf != BME280_I2C_INTF)
            {
                for (reg_addr_cnt = 0; reg_addr_cnt<len; reg_addr_cnt++)
                {
                    reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
                }
            }

            /* Burst write mode */
            if (len > 1)
            {
                /* Interleave register address w.r.t data for
                 * burst write
                 */
                interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
temp_len = ((len* 2) - 1);
            }
            else
            {
                temp_len = len;
            }
            rslt = dev->write(dev->dev_id, reg_addr[0], temp_buff, temp_len);

            /* Check for communication error */
            if (rslt != BME280_OK)
            {
                rslt = BME280_E_COMM_FAIL;
            }
        }
        else
        {
            rslt = BME280_E_INVALID_LEN;
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 */
int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t sensor_mode;

/* Check for null pointer in the device structure*/
rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        rslt = bme280_get_sensor_mode(&sensor_mode, dev);
        if ((rslt == BME280_OK) && (sensor_mode != BME280_SLEEP_MODE))
        {
            rslt = put_device_to_sleep(dev);
        }
        if (rslt == BME280_OK)
        {
            /* Check if user wants to change oversampling
             * settings
             */
            if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings))
            {
                rslt = set_osr_settings(desired_settings, &dev->settings, dev);
            }

            /* Check if user wants to change filter and/or
             * standby settings
             */
            if ((rslt == BME280_OK) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings))
            {
                rslt = set_filter_standby_settings(desired_settings, &dev->settings, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 */
int8_t bme280_get_sensor_settings(struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t reg_data[4];

/* Check for null pointer in the device structure*/
rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4, dev);
        if (rslt == BME280_OK)
        {
            parse_device_settings(reg_data, &dev->settings);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t last_set_mode;

/* Check for null pointer in the device structure*/
rslt = null_ptr_check(dev);
    if (rslt == BME280_OK)
    {
        rslt = bme280_get_sensor_mode(&last_set_mode, dev);

        /* If the sensor is not in sleep mode put the device to sleep
         * mode
         */
        if ((rslt == BME280_OK) && (last_set_mode != BME280_SLEEP_MODE))
        {
            rslt = put_device_to_sleep(dev);
        }

        /* Set the power mode */
        if (rslt == BME280_OK)
        {
            rslt = write_power_mode(sensor_mode, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
int8_t bme280_get_sensor_mode(uint8_t* sensor_mode, const struct bme280_dev * dev)
{
    int8_t rslt;

/* Check for null pointer in the device structure*/
rslt = null_ptr_check(dev);
    if (rslt == BME280_OK)
    {
        /* Read the power mode register */
        rslt = bme280_get_regs(BME280_PWR_CTRL_ADDR, sensor_mode, 1, dev);

        /* Assign the power mode in the device structure */
        * sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);
    }

    return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t bme280_soft_reset(const struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t reg_addr = BME280_RESET_ADDR;
uint8_t status_reg = 0;
uint8_t try_run = 5;

/* 0xB6 is the soft reset command */
uint8_t soft_rst_cmd = BME280_SOFT_RESET_COMMAND;

/* Check for null pointer in the device structure*/
rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BME280_OK)
    {
        /* Write the soft reset command in the sensor */
        rslt = bme280_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);

        if (rslt == BME280_OK)
        {
            /* If NVM not copied yet, Wait for NVM to copy */
            do
            {
                /* As per data sheet - Table 1, startup time is 2 ms. */
                dev->delay_ms(2);
rslt = bme280_get_regs(BME280_STATUS_REG_ADDR, &status_reg, 1, dev);
            } while ((rslt == BME280_OK) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));

            if (status_reg & BME280_STATUS_IM_UPDATE)
            {
                rslt = BME280_E_NVM_COPY_FAILED;
            }

        }
    }

    return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data * comp_data, struct bme280_dev * dev)
{
    int8_t rslt;

/* Array to store the pressure, temperature and humidity data read from
 * the sensor
 */
uint8_t reg_data[BME280_P_T_H_DATA_LEN] = { 0 };
struct bme280_uncomp_data uncomp_data = { 0 };

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if ((rslt == BME280_OK) && (comp_data != NULL))
    {
        /* Read the pressure and temperature data from the sensor */
        rslt = bme280_get_regs(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN, dev);
        if (rslt == BME280_OK)
        {
            /* Parse the read data from the sensor */
            bme280_parse_sensor_data(reg_data, &uncomp_data);

/* Compensate the pressure and/or temperature and/or
 * humidity data from the sensor
 */
rslt = bme280_compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void bme280_parse_sensor_data(const uint8_t* reg_data, struct bme280_uncomp_data * uncomp_data)
{
    /* Variables to store the sensor data */
    uint32_t data_xlsb;
uint32_t data_lsb;
uint32_t data_msb;

/* Store the parsed register values for pressure data */
data_msb = (uint32_t) reg_data[0] << 12;
    data_lsb = (uint32_t) reg_data[1] << 4;
    data_xlsb = (uint32_t) reg_data[2] >> 4;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_msb = (uint32_t) reg_data[3] << 12;
    data_lsb = (uint32_t) reg_data[4] << 4;
    data_xlsb = (uint32_t) reg_data[5] >> 4;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_lsb = (uint32_t) reg_data[6] << 8;
    data_msb = (uint32_t) reg_data[7];
uncomp_data->humidity = data_msb | data_lsb;
}

/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
int8_t bme280_compensate_data(uint8_t sensor_comp,
                              const struct bme280_uncomp_data * uncomp_data,
                              struct bme280_data * comp_data,
                              struct bme280_calib_data * calib_data)
{
    int8_t rslt = BME280_OK;

    if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL))
    {
        /* Initialize to zero */
        comp_data->temperature = 0;
        comp_data->pressure = 0;
        comp_data->humidity = 0;

        /* If pressure or temperature component is selected */
        if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM))
        {
            /* Compensate the temperature data */
            comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
        }
        if (sensor_comp & BME280_PRESS)
        {
            /* Compensate the pressure data */
            comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
        }
        if (sensor_comp & BME280_HUM)
        {
            /* Compensate the humidity data */
            comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 */
static int8_t set_osr_settings(uint8_t desired_settings,
                               const struct bme280_settings * settings,
                               const struct bme280_dev * dev)
{
    int8_t rslt = BME280_W_INVALID_OSR_MACRO;

    if (desired_settings & BME280_OSR_HUM_SEL)
    {
        rslt = set_osr_humidity_settings(settings, dev);
    }
    if (desired_settings & (BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL))
    {
        rslt = set_osr_press_temp_settings(desired_settings, settings, dev);
    }

    return rslt;
}

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 */
static int8_t set_osr_humidity_settings(const struct bme280_settings * settings, const struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t ctrl_hum;
uint8_t ctrl_meas;
uint8_t reg_addr = BME280_CTRL_HUM_ADDR;

ctrl_hum = settings->osr_h & BME280_CTRL_HUM_MSK;

    /* Write the humidity control value in the register */
    rslt = bme280_set_regs(&reg_addr, &ctrl_hum, 1, dev);

    /* Humidity related changes will be only effective after a
     * write operation to ctrl_meas register
     */
    if (rslt == BME280_OK)
    {
        reg_addr = BME280_CTRL_MEAS_ADDR;
        rslt = bme280_get_regs(reg_addr, &ctrl_meas, 1, dev);
        if (rslt == BME280_OK)
        {
            rslt = bme280_set_regs(&reg_addr, &ctrl_meas, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 */
static int8_t set_osr_press_temp_settings(uint8_t desired_settings,
                                          const struct bme280_settings * settings,
                                          const struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t reg_addr = BME280_CTRL_MEAS_ADDR;
uint8_t reg_data;

rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);
    if (rslt == BME280_OK)
    {
        if (desired_settings & BME280_OSR_PRESS_SEL)
        {
            fill_osr_press_settings(&reg_data, settings);
        }
        if (desired_settings & BME280_OSR_TEMP_SEL)
        {
            fill_osr_temp_settings(&reg_data, settings);
        }

        /* Write the oversampling settings in the register */
        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 */
static int8_t set_filter_standby_settings(uint8_t desired_settings,
                                          const struct bme280_settings * settings,
                                          const struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t reg_addr = BME280_CONFIG_ADDR;
uint8_t reg_data;

rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);
    if (rslt == BME280_OK)
    {
        if (desired_settings & BME280_FILTER_SEL)
        {
            fill_filter_settings(&reg_data, settings);
        }
        if (desired_settings & BME280_STANDBY_SEL)
        {
            fill_standby_settings(&reg_data, settings);
        }

        /* Write the oversampling settings in the register */
        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 */
static void fill_filter_settings(uint8_t* reg_data, const struct bme280_settings * settings)
{
    * reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, settings->filter);
}

/*!
 * @brief This internal API fills the standby duration settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
static void fill_standby_settings(uint8_t* reg_data, const struct bme280_settings * settings)
{
    * reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, settings->standby_time);
}

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
static void fill_osr_press_settings(uint8_t* reg_data, const struct bme280_settings * settings)
{
    * reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, settings->osr_p);
}

/*!
 * @brief This internal API fills the temperature oversampling settings
 * provided by the user in the data buffer so as to write in the sensor.
 */
static void fill_osr_temp_settings(uint8_t* reg_data, const struct bme280_settings * settings)
{
    * reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, settings->osr_t);
}

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * device structure.
 */
static void parse_device_settings(const uint8_t* reg_data, struct bme280_settings * settings)
{
    settings->osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
settings->osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
settings->osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
settings->filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
settings->standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);
}

/*!
 * @brief This internal API writes the power mode in the sensor.
 */
static int8_t write_power_mode(uint8_t sensor_mode, const struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t reg_addr = BME280_PWR_CTRL_ADDR;

/* Variable to store the value read from power mode register */
uint8_t sensor_mode_reg_val;

/* Read the power mode register */
rslt = bme280_get_regs(reg_addr, &sensor_mode_reg_val, 1, dev);

    /* Set the power mode */
    if (rslt == BME280_OK)
    {
        sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val, BME280_SENSOR_MODE, sensor_mode);

/* Write the power mode in the register */
rslt = bme280_set_regs(&reg_addr, &sensor_mode_reg_val, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API puts the device to sleep mode.
 */
static int8_t put_device_to_sleep(const struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t reg_data[4];
struct bme280_settings settings;

    rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4, dev);
    if (rslt == BME280_OK)
    {
        parse_device_settings(reg_data, &settings);
rslt = bme280_soft_reset(dev);
        if (rslt == BME280_OK)
        {
            rslt = reload_device_settings(&settings, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API reloads the already existing device settings in
 * the sensor after soft reset.
 */
static int8_t reload_device_settings(const struct bme280_settings * settings, const struct bme280_dev * dev)
{
    int8_t rslt;

rslt = set_osr_settings(BME280_ALL_SETTINGS_SEL, settings, dev);
    if (rslt == BME280_OK)
    {
        rslt = set_filter_standby_settings(BME280_ALL_SETTINGS_SEL, settings, dev);
    }

    return rslt;
}

#ifdef BME280_FLOAT_ENABLE

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
static double compensate_temperature(const struct bme280_uncomp_data * uncomp_data, struct bme280_calib_data * calib_data)
{
    double var1;
double var2;
double temperature;
double temperature_min = -40;
double temperature_max = 85;

var1 = ((double) uncomp_data->temperature) / 16384.0 - ((double) calib_data->dig_T1) / 1024.0;
    var1 = var1* ((double) calib_data->dig_T2);
    var2 = (((double) uncomp_data->temperature) / 131072.0 - ((double) calib_data->dig_T1) / 8192.0);
    var2 = (var2* var2) * ((double) calib_data->dig_T3);
    calib_data->t_fine = (int32_t) (var1 + var2);
    temperature = (var1 + var2) / 5120.0;
    if (temperature<temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
static double compensate_pressure(const struct bme280_uncomp_data * uncomp_data,
                                  const struct bme280_calib_data * calib_data)
{
    double var1;
double var2;
double var3;
double pressure;
double pressure_min = 30000.0;
double pressure_max = 110000.0;

var1 = ((double) calib_data->t_fine / 2.0) - 64000.0;
    var2 = var1* var1 * ((double) calib_data->dig_P6) / 32768.0;
    var2 = var2 + var1* ((double) calib_data->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double) calib_data->dig_P4) * 65536.0);
    var3 = ((double) calib_data->dig_P3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double) calib_data->dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double) calib_data->dig_P1);

    /* avoid exception caused by division by zero */
    if (var1)
    {
        pressure = 1048576.0 - (double) uncomp_data->pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double) calib_data->dig_P9) * pressure * pressure / 2147483648.0;
        var2 = pressure* ((double) calib_data->dig_P8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double) calib_data->dig_P7)) / 16.0;
        if (pressure<pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
static double compensate_humidity(const struct bme280_uncomp_data * uncomp_data,
                                  const struct bme280_calib_data * calib_data)
{
    double humidity;
double humidity_min = 0.0;
double humidity_max = 100.0;
double var1;
double var2;
double var3;
double var4;
double var5;
double var6;

var1 = ((double) calib_data->t_fine) - 76800.0;
    var2 = (((double) calib_data->dig_H4) * 64.0 + (((double) calib_data->dig_H5) / 16384.0) * var1);
    var3 = uncomp_data->humidity - var2;
    var4 = ((double) calib_data->dig_H2) / 65536.0;
    var5 = (1.0 + (((double) calib_data->dig_H3) / 67108864.0) * var1);
    var6 = 1.0 + (((double) calib_data->dig_H6) / 67108864.0) * var1 * var5;
var6 = var3* var4 * (var5* var6);
    humidity = var6* (1.0 - ((double) calib_data->dig_H1) * var6 / 524288.0);
    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity<humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}

#else

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
static int32_t compensate_temperature(const struct bme280_uncomp_data * uncomp_data,
                                      struct bme280_calib_data * calib_data)
{
    int32_t var1;
int32_t var2;
int32_t temperature;
int32_t temperature_min = -4000;
int32_t temperature_max = 8500;

var1 = (int32_t) ((uncomp_data->temperature / 8) - ((int32_t) calib_data->dig_T1* 2));
    var1 = (var1* ((int32_t) calib_data->dig_T2)) / 2048;
    var2 = (int32_t) ((uncomp_data->temperature / 16) - ((int32_t) calib_data->dig_T1));
    var2 = (((var2* var2) / 4096) * ((int32_t) calib_data->dig_T3)) / 16384;
    calib_data->t_fine = var1 + var2;
    temperature = (calib_data->t_fine* 5 + 128) / 256;
    if (temperature<temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}
#ifdef BME280_64BIT_ENABLE

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type with higher
 * accuracy.
 */
static uint32_t compensate_pressure(const struct bme280_uncomp_data * uncomp_data,
                                    const struct bme280_calib_data * calib_data)
{
    int64_t var1;
int64_t var2;
int64_t var3;
int64_t var4;
uint32_t pressure;
uint32_t pressure_min = 3000000;
uint32_t pressure_max = 11000000;

var1 = ((int64_t) calib_data->t_fine) - 128000;
    var2 = var1* var1 * (int64_t) calib_data->dig_P6;
    var2 = var2 + ((var1* (int64_t) calib_data->dig_P5) * 131072);
    var2 = var2 + (((int64_t) calib_data->dig_P4) * 34359738368);
    var1 = ((var1* var1 * (int64_t) calib_data->dig_P3) / 256) + ((var1* ((int64_t) calib_data->dig_P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t) calib_data->dig_P1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0)
    {
        var4 = 1048576 - uncomp_data->pressure;
        var4 = (((var4* INT64_C(2147483648)) - var2) * 3125) / var1;
        var1 = (((int64_t) calib_data->dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t) calib_data->dig_P8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t) calib_data->dig_P7) * 16);
        pressure = (uint32_t) (((var4 / 2) * 100) / 128);
        if (pressure<pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}
#else

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
static uint32_t compensate_pressure(const struct bme280_uncomp_data * uncomp_data,
                                    const struct bme280_calib_data * calib_data)
{
    int32_t var1;
int32_t var2;
int32_t var3;
int32_t var4;
uint32_t var5;
uint32_t pressure;
uint32_t pressure_min = 30000;
uint32_t pressure_max = 110000;

var1 = (((int32_t) calib_data->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t) calib_data->dig_P6);
    var2 = var2 + ((var1* ((int32_t) calib_data->dig_P5)) * 2);
    var2 = (var2 / 4) + (((int32_t) calib_data->dig_P4) * 65536);
    var3 = (calib_data->dig_P3* (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t) calib_data->dig_P2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t) calib_data->dig_P1)) / 32768;

    /* avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (uint32_t) ((uint32_t)1048576) - uncomp_data->pressure;
        pressure = ((uint32_t)(var5 - (uint32_t) (var2 / 4096))) * 3125;
        if (pressure< 0x80000000)
        {
            pressure = (pressure << 1) / ((uint32_t) var1);
        }
        else
        {
            pressure = (pressure / (uint32_t) var1) * 2;
        }
        var1 = (((int32_t) calib_data->dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t) calib_data->dig_P8)) / 8192;
        pressure = (uint32_t) ((int32_t) pressure + ((var1 + var2 + calib_data->dig_P7) / 16));
        if (pressure<pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}
#endif

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 */
static uint32_t compensate_humidity(const struct bme280_uncomp_data * uncomp_data,
                                    const struct bme280_calib_data * calib_data)
{
    int32_t var1;
int32_t var2;
int32_t var3;
int32_t var4;
int32_t var5;
uint32_t humidity;
uint32_t humidity_max = 102400;

var1 = calib_data->t_fine - ((int32_t)76800);
    var2 = (int32_t) (uncomp_data->humidity* 16384);
    var3 = (int32_t) (((int32_t) calib_data->dig_H4) * 1048576);
    var4 = ((int32_t) calib_data->dig_H5) * var1;
var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1* ((int32_t) calib_data->dig_H6)) / 1024;
    var3 = (var1* ((int32_t) calib_data->dig_H3)) / 2048;
    var4 = ((var2* (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4* ((int32_t) calib_data->dig_H2)) + 8192) / 16384;
    var3 = var5* var2;
var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4* ((int32_t) calib_data->dig_H1)) / 16);
    var5 = (var5< 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t) (var5 / 4096);
    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    return humidity;
}
#endif

/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
static int8_t get_calib_data(struct bme280_dev * dev)
{
    int8_t rslt;
uint8_t reg_addr = BME280_TEMP_PRESS_CALIB_DATA_ADDR;

/* Array to store calibration data */
uint8_t calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = { 0 };

/* Read the calibration data from the sensor */
rslt = bme280_get_regs(reg_addr, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN, dev);
    if (rslt == BME280_OK)
    {
        /* Parse temperature and pressure calibration data and store
         * it in device structure
         */
        parse_temp_press_calib_data(calib_data, dev);
reg_addr = BME280_HUMIDITY_CALIB_DATA_ADDR;

        /* Read the humidity calibration data from the sensor */
        rslt = bme280_get_regs(reg_addr, calib_data, BME280_HUMIDITY_CALIB_DATA_LEN, dev);
        if (rslt == BME280_OK)
        {
            /* Parse humidity calibration data and store it in
             * device structure
             */
            parse_humidity_calib_data(calib_data, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 */
static void interleave_reg_addr(const uint8_t* reg_addr, uint8_t* temp_buff, const uint8_t* reg_data, uint8_t len)
{
    uint8_t index;

    for (index = 1; index < len; index++)
    {
        temp_buff[(index * 2) - 1] = reg_addr[index];
        temp_buff[index * 2] = reg_data[index];
    }
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
 */
static void parse_temp_press_calib_data(const uint8_t* reg_data, struct bme280_dev * dev)
{
    struct bme280_calib_data * calib_data = &dev->calib_data;

calib_data->dig_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
calib_data->dig_T2 = (int16_t) BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
calib_data->dig_T3 = (int16_t) BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
calib_data->dig_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
calib_data->dig_P2 = (int16_t) BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
calib_data->dig_P3 = (int16_t) BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
calib_data->dig_P4 = (int16_t) BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
calib_data->dig_P5 = (int16_t) BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
calib_data->dig_P6 = (int16_t) BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
calib_data->dig_P7 = (int16_t) BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
calib_data->dig_P8 = (int16_t) BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
calib_data->dig_P9 = (int16_t) BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
calib_data->dig_H1 = reg_data[25];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
static void parse_humidity_calib_data(const uint8_t* reg_data, struct bme280_dev * dev)
{
    struct bme280_calib_data * calib_data = &dev->calib_data;
int16_t dig_H4_lsb;
int16_t dig_H4_msb;
int16_t dig_H5_lsb;
int16_t dig_H5_msb;

calib_data->dig_H2 = (int16_t) BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
calib_data->dig_H3 = reg_data[2];
    dig_H4_msb = (int16_t) (int8_t) reg_data[3] * 16;
    dig_H4_lsb = (int16_t) (reg_data[4] & 0x0F);
    calib_data->dig_H4 = dig_H4_msb | dig_H4_lsb;
    dig_H5_msb = (int16_t) (int8_t) reg_data[5] * 16;
    dig_H5_lsb = (int16_t) (reg_data[4] >> 4);
    calib_data->dig_H5 = dig_H5_msb | dig_H5_lsb;
    calib_data->dig_H6 = (int8_t) reg_data[6];
}

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 */
static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings)
{
    uint8_t settings_changed = FALSE;

    if (sub_settings & desired_settings)
    {
        /* User wants to modify this particular settings */
        settings_changed = TRUE;
    }
    else
    {
        /* User don't want to modify this particular settings */
        settings_changed = FALSE;
    }

    return settings_changed;
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bme280_dev * dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BME280_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BME280_OK;
    }

    return rslt;
}

/**
 * Copyright (C) 2018 - 2019 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file    bme280.h
 * @date    26 Aug 2019
 * @version 3.3.7
 * @brief
 *
 */

/*! @file bme280.h
 * @brief Sensor driver for BME280 sensor
 */

/*!
 * @defgroup BME280 SENSOR API
 */
#ifndef BME280_H_
#define BME280_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/* Header includes */
#include "bme280_defs.h"

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 *
 *  @param[in,out] dev : Structure instance of bme280_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_init(struct bme280_dev * dev);

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be written.
 * @param[in] reg_data : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len : No of bytes of data to write..
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_set_regs(uint8_t* reg_addr, const uint8_t* reg_data, uint8_t len, const struct bme280_dev * dev);

/*!
 * @brief This API reads the data from the given register address of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] len : No of bytes of data to be read.
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t* reg_data, uint16_t len, const struct bme280_dev * dev);

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros         |   Functionality
 * -----------------------|----------------------------------------------
 * BME280_OSR_PRESS_SEL    |   To set pressure oversampling.
 * BME280_OSR_TEMP_SEL     |   To set temperature oversampling.
 * BME280_OSR_HUM_SEL    |   To set humidity oversampling.
 * BME280_FILTER_SEL     |   To set filter setting.
 * BME280_STANDBY_SEL  |   To set standby duration setting.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_dev * dev);

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme280_get_sensor_settings(struct bme280_dev * dev);

/*!
 * @brief This API sets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] sensor_mode : Variable which contains the power mode to be set.
 *
 *    sensor_mode           |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev * dev);

/*!
 * @brief This API gets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] sensor_mode : Pointer variable to store the power mode.
 *
 *   sensor_mode            |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_get_sensor_mode(uint8_t* sensor_mode, const struct bme280_dev * dev);

/*!
 * @brief This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme280_soft_reset(const struct bme280_dev * dev);

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BME280_PRESS
 *     2       | BME280_TEMP
 *     4       | BME280_HUM
 *     7       | BME280_ALL
 *
 * @param[out] comp_data : Structure instance of bme280_data.
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data * comp_data, struct bme280_dev * dev);

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 *
 *  @param[in] reg_data     : Contains register data which needs to be parsed
 *  @param[out] uncomp_data : Contains the uncompensated pressure, temperature
 *  and humidity data.
 */
void bme280_parse_sensor_data(const uint8_t* reg_data, struct bme280_uncomp_data * uncomp_data);

/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected by the
 * user.
 *
 * @param[in] sensor_comp : Used to select pressure and/or temperature and/or
 * humidity.
 * @param[in] uncomp_data : Contains the uncompensated pressure, temperature and
 * humidity data.
 * @param[out] comp_data : Contains the compensated pressure and/or temperature
 * and/or humidity data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bme280_compensate_data(uint8_t sensor_comp,
                              const struct bme280_uncomp_data * uncomp_data,
                              struct bme280_data * comp_data,
                              struct bme280_calib_data * calib_data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* BME280_H_ */
/** @}*/

/**
 * Copyright (C) 2018 - 2019 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file    bme280_defs.h
 * @date    26 Aug 2019
 * @version 3.3.7
 * @brief
 *
 */

/*! @file bme280_defs.h
 * @brief Sensor driver for BME280 sensor
 */

/*!
 * @defgroup BME280 SENSOR API
 * @brief
 */
#ifndef BME280_DEFS_H_
#define BME280_DEFS_H_

/********************************************************/
/* header includes */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/********************************************************/
/*! @name       Common macros               */
/********************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)   S8_C(x)
#define UINT8_C(x)  U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)  S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)  S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)  S64_C(x)
#define UINT64_C(x) U64_C(x)
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif

/********************************************************/

#ifndef BME280_FLOAT_ENABLE

/* #define BME280_FLOAT_ENABLE */
#endif

#ifndef BME280_FLOAT_ENABLE
#ifndef BME280_64BIT_ENABLE
#define BME280_64BIT_ENABLE
#endif
#endif

#ifndef TRUE
#define TRUE                              UINT8_C(1)
#endif
#ifndef FALSE
#define FALSE                             UINT8_C(0)
#endif

/**\name I2C addresses */
#define BME280_I2C_ADDR_PRIM              UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC               UINT8_C(0x77)

/**\name BME280 chip identifier */
#define BME280_CHIP_ID                    UINT8_C(0x60)

/**\name Register Address */
#define BME280_CHIP_ID_ADDR               UINT8_C(0xD0)
#define BME280_RESET_ADDR                 UINT8_C(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR UINT8_C(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR   UINT8_C(0xE1)
#define BME280_PWR_CTRL_ADDR              UINT8_C(0xF4)
#define BME280_CTRL_HUM_ADDR              UINT8_C(0xF2)
#define BME280_CTRL_MEAS_ADDR             UINT8_C(0xF4)
#define BME280_CONFIG_ADDR                UINT8_C(0xF5)
#define BME280_DATA_ADDR                  UINT8_C(0xF7)

/**\name API success code */
#define BME280_OK                         INT8_C(0)

/**\name API error codes */
#define BME280_E_NULL_PTR                 INT8_C(-1)
#define BME280_E_DEV_NOT_FOUND            INT8_C(-2)
#define BME280_E_INVALID_LEN              INT8_C(-3)
#define BME280_E_COMM_FAIL                INT8_C(-4)
#define BME280_E_SLEEP_MODE_FAIL          INT8_C(-5)
#define BME280_E_NVM_COPY_FAILED          INT8_C(-6)

/**\name API warning codes */
#define BME280_W_INVALID_OSR_MACRO        INT8_C(1)

/**\name Macros related to size */
#define BME280_TEMP_PRESS_CALIB_DATA_LEN  UINT8_C(26)
#define BME280_HUMIDITY_CALIB_DATA_LEN    UINT8_C(7)
#define BME280_P_T_H_DATA_LEN             UINT8_C(8)

/**\name Sensor power modes */
#define BME280_SLEEP_MODE                 UINT8_C(0x00)
#define BME280_FORCED_MODE                UINT8_C(0x01)
#define BME280_NORMAL_MODE                UINT8_C(0x03)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)            (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)       ((reg_data & (bitname##_MSK)) >> \
                                                  (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

/**\name Macros for bit masking */
#define BME280_SENSOR_MODE_MSK      UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS      UINT8_C(0x00)

#define BME280_CTRL_HUM_MSK         UINT8_C(0x07)
#define BME280_CTRL_HUM_POS         UINT8_C(0x00)

#define BME280_CTRL_PRESS_MSK       UINT8_C(0x1C)
#define BME280_CTRL_PRESS_POS       UINT8_C(0x02)

#define BME280_CTRL_TEMP_MSK        UINT8_C(0xE0)
#define BME280_CTRL_TEMP_POS        UINT8_C(0x05)

#define BME280_FILTER_MSK           UINT8_C(0x1C)
#define BME280_FILTER_POS           UINT8_C(0x02)

#define BME280_STANDBY_MSK          UINT8_C(0xE0)
#define BME280_STANDBY_POS          UINT8_C(0x05)

/**\name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.
 */
#define BME280_PRESS                UINT8_C(1)
#define BME280_TEMP                 UINT8_C(1 << 1)
#define BME280_HUM                  UINT8_C(1 << 2)
#define BME280_ALL                  UINT8_C(0x07)

/**\name Settings selection macros */
#define BME280_OSR_PRESS_SEL        UINT8_C(1)
#define BME280_OSR_TEMP_SEL         UINT8_C(1 << 1)
#define BME280_OSR_HUM_SEL          UINT8_C(1 << 2)
#define BME280_FILTER_SEL           UINT8_C(1 << 3)
#define BME280_STANDBY_SEL          UINT8_C(1 << 4)
#define BME280_ALL_SETTINGS_SEL     UINT8_C(0x1F)

/**\name Oversampling macros */
#define BME280_NO_OVERSAMPLING      UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X      UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X      UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X      UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X      UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X     UINT8_C(0x05)

/**\name Standby duration selection macros */
#define BME280_STANDBY_TIME_0_5_MS  (0x00)
#define BME280_STANDBY_TIME_62_5_MS (0x01)
#define BME280_STANDBY_TIME_125_MS  (0x02)
#define BME280_STANDBY_TIME_250_MS  (0x03)
#define BME280_STANDBY_TIME_500_MS  (0x04)
#define BME280_STANDBY_TIME_1000_MS (0x05)
#define BME280_STANDBY_TIME_10_MS   (0x06)
#define BME280_STANDBY_TIME_20_MS   (0x07)

/**\name Filter coefficient selection macros */
#define BME280_FILTER_COEFF_OFF     (0x00)
#define BME280_FILTER_COEFF_2       (0x01)
#define BME280_FILTER_COEFF_4       (0x02)
#define BME280_FILTER_COEFF_8       (0x03)
#define BME280_FILTER_COEFF_16      (0x04)

#define BME280_STATUS_REG_ADDR      (0xF3)
#define BME280_SOFT_RESET_COMMAND   (0xB6)
#define BME280_STATUS_IM_UPDATE     (0x01)

/*!
 * @brief Interface selection Enums
 */
enum bme280_intf {
    /*! SPI interface */
    BME280_SPI_INTF,

    /*! I2C interface */
    BME280_I2C_INTF
};

/*!
 * @brief Type definitions
 */
typedef int8_t (*bme280_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bme280_delay_fptr_t)(uint32_t period);

/*!
 * @brief Calibration data
 */
struct bme280_calib_data
{
    /**
     * @ Trim Variables
     */

    /**@{*/
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
    int32_t t_fine;

    /**@}*/
};

/*!
 * @brief bme280 sensor structure which comprises of temperature, pressure and
 * humidity data
 */
#ifdef BME280_FLOAT_ENABLE
struct bme280_data
{
    /*! Compensated pressure */
    double pressure;

    /*! Compensated temperature */
    double temperature;

    /*! Compensated humidity */
    double humidity;
};
#else
struct bme280_data
{
    /*! Compensated pressure */
    uint32_t pressure;

    /*! Compensated temperature */
    int32_t temperature;

    /*! Compensated humidity */
    uint32_t humidity;
};
#endif /* BME280_USE_FLOATING_POINT */

/*!
 * @brief bme280 sensor structure which comprises of uncompensated temperature,
 * pressure and humidity data
 */
struct bme280_uncomp_data
{
    /*! un-compensated pressure */
    uint32_t pressure;

    /*! un-compensated temperature */
    uint32_t temperature;

    /*! un-compensated humidity */
    uint32_t humidity;
};

/*!
 * @brief bme280 sensor settings structure which comprises of mode,
 * oversampling and filter settings.
 */
struct bme280_settings
{
    /*! pressure oversampling */
    uint8_t osr_p;

    /*! temperature oversampling */
    uint8_t osr_t;

    /*! humidity oversampling */
    uint8_t osr_h;

    /*! filter coefficient */
    uint8_t filter;

    /*! standby time */
    uint8_t standby_time;
};

/*!
 * @brief bme280 device structure
 */
struct bme280_dev
{
    /*! Chip Id */
    uint8_t chip_id;

    /*! Device Id */
    uint8_t dev_id;

    /*! SPI/I2C interface */
    enum bme280_intf intf;

    /*! Read function pointer */
    bme280_com_fptr_t read;

    /*! Write function pointer */
    bme280_com_fptr_t write;

    /*! Delay function pointer */
    bme280_delay_fptr_t delay_ms;

    /*! Trim data */
    struct bme280_calib_data calib_data;

    /*! Sensor settings */
    struct bme280_settings settings;
};

#endif /* BME280_DEFS_H_ */
/** @}*/
/** @}*/
