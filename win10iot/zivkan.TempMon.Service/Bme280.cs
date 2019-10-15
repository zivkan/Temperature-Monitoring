// Copied from:
// https://github.com/BoschSensortec/BME280_driver/blob/bdf4e573cbc77dc1d30a24845d0c1e35eebc84fa/bme280.c
// https://github.com/BoschSensortec/BME280_driver/blob/bdf4e573cbc77dc1d30a24845d0c1e35eebc84fa/bme280.h
// https://github.com/BoschSensortec/BME280_driver/blob/bdf4e573cbc77dc1d30a24845d0c1e35eebc84fa/bme280_defs.h
// See git history for changes

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

using System;
using System.Diagnostics;
using System.Threading.Tasks;
using Windows.Devices.I2c;

namespace zivkan.TempMon.Service
{
    using int8_t = System.SByte;
    using INT8_C = System.SByte;
    using uint8_t = System.Byte;
    using UINT8_C = System.Byte;
    using uint16_t = System.UInt16;
    using int16_t = System.Int16;
    using uint32_t = System.UInt32;
    using int32_t = System.Int32;

    internal sealed class Bme280
    {
        private I2cDevice _device;
        private bme280_dev _dev;

        private Bme280(I2cDevice device, bme280_dev dev)
        {
            _device = device;
            _dev = dev;
        }

        public static Task<Bme280> CreateAsync(I2cDevice device)
        {
            var dev = new bme280_dev();
            dev.dev_id = BME280_I2C_ADDR_PRIM;
            dev.intf = bme280_intf.BME280_I2C_INTF;
            dev.delay_ms = (ms) => System.Threading.Thread.Sleep((int)ms);
            dev.read = (uint8_t dev_id, uint8_t reg_addr, uint8_t[] data, uint16_t len) =>
            {
                if (data.Length != len)
                {
                    throw new ArgumentException();
                }

                var writeBuffer = new byte[] { reg_addr };
                device.WriteRead(writeBuffer, data);

                return BME280_OK;
            };
            dev.write = (uint8_t dev_id, uint8_t reg_addr, uint8_t[] data, uint16_t len) =>
            {
                if (data.Length != len)
                {
                    throw new ArgumentException();
                }

                var writeBuffer = new byte[len + 1];
                writeBuffer[0] = reg_addr;
                Array.Copy(data, 0, writeBuffer, 1, len);
                device.Write(writeBuffer);

                return BME280_OK;
            };

            var result = bme280_init(dev);
            if (result != BME280_OK)
            {
                throw new Exception();
            }

            // weather monitoring recommended values from datasheet
            dev.settings.osr_h = BME280_OVERSAMPLING_1X;
            dev.settings.osr_p = BME280_OVERSAMPLING_1X;
            dev.settings.osr_t = BME280_OVERSAMPLING_1X;
            dev.settings.filter = BME280_FILTER_COEFF_OFF;
            uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

            result = bme280_set_sensor_settings(settings_sel, dev);
            if (result != BME280_OK)
            {
                throw new Exception();
            }

            return Task.FromResult(new Bme280(device, dev));
        }

        public async Task<Measurement> GetMeasurementAsync()
        {
            var result = bme280_set_sensor_mode(BME280_FORCED_MODE, _dev);
            if (result != BME280_OK)
            {
                throw new Exception();
            }

            var delay_len = GetMeasurementDelay();

            var delay = Task.Delay(delay_len);
            var when = DateTime.UtcNow;
            Debug.WriteLine("delay = " + delay_len);
            await delay.ConfigureAwait(false);

            var comp_data = new bme280_data();
            result = bme280_get_sensor_data(BME280_ALL, comp_data, _dev);
            if (result != BME280_OK)
            {
                throw new Exception();
            }

            return new Measurement(comp_data.temperature, comp_data.humidity, comp_data.pressure, when);
        }

        private TimeSpan GetMeasurementDelay()
        {
            var delay = TimeSpan.FromMilliseconds(1.25);

            int osr = GetOversamplingRate(_dev.settings.osr_t);
            if (osr != 0)
            {
                delay += TimeSpan.FromMilliseconds(2.3 * osr);
            }

            osr = GetOversamplingRate(_dev.settings.osr_p);
            if (osr != 0)
            {
                delay += TimeSpan.FromMilliseconds(2.3 * osr + 0.575);
            }

            osr = GetOversamplingRate(_dev.settings.osr_h);
            if (osr != 0)
            {
                delay += TimeSpan.FromMilliseconds(2.3 * osr + 0.575);
            }

            return delay;
        }

        private int GetOversamplingRate(byte osr)
        {
            switch(osr)
            {
                case 0:
                    return 0;

                case BME280_OVERSAMPLING_1X:
                    return 1;

                case BME280_OVERSAMPLING_2X:
                    return 2;

                case BME280_OVERSAMPLING_4X:
                    return 4;

                case BME280_OVERSAMPLING_8X:
                    return 8;

                case BME280_OVERSAMPLING_16X:
                    return 16;

                default:
                    throw new ArgumentException(nameof(osr));
            }
        }

        private const UINT8_C OVERSAMPLING_SETTINGS = (UINT8_C)(0x07);
        private const UINT8_C FILTER_STANDBY_SETTINGS = (UINT8_C)(0x18);

        /// <summary>
        /// This API is the entry point. It reads the chip-id and calibration data from the sensor.
        /// </summary>
        /// <param name="dev">Structure instance of bme280_dev</param>
        /// <returns>Result of API execution status
        /// <para>zero -> Success / +ve value -> Warning / -ve value -> Error</para></returns>
        static int8_t bme280_init(bme280_dev dev)
        {
            int8_t rslt;

            /* chip id read try count */
            uint8_t try_count = 5;
            uint8_t[] chip_id = new uint8_t[1] { 0 };

            /* Check for null pointer in the device structure*/
            rslt = null_ptr_check(dev);

            /* Proceed if null check is fine */
            if (rslt == BME280_OK)
            {
                while (try_count > 0)
                {
                    /* Read the chip-id of bme280 sensor */
                    rslt = bme280_get_regs(BME280_CHIP_ID_ADDR, chip_id, 1, dev);

                    /* Check for chip id validity */
                    if ((rslt == BME280_OK) && (chip_id[0] == BME280_CHIP_ID))
                    {
                        dev.chip_id = chip_id[0];

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
                    dev.delay_ms(1);
                    --try_count;
                }

                /* Chip id check failed */
                if (try_count == 0)
                {
                    rslt = BME280_E_DEV_NOT_FOUND;
                }
            }

            return rslt;
        }

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
        private static int8_t bme280_get_regs(uint8_t reg_addr, uint8_t[] reg_data, uint16_t len, bme280_dev dev)
        {
            int8_t rslt;

            /* Check for null pointer in the device structure*/
            rslt = null_ptr_check(dev);

            /* Proceed if null check is fine */
            if (rslt == BME280_OK)
            {
                /* If interface selected is SPI */
                if (dev.intf != bme280_intf.BME280_I2C_INTF)
                {
                    reg_addr = (byte)(reg_addr | 0x80);
                }

                /* Read the data  */
                rslt = dev.read(dev.dev_id, reg_addr, reg_data, len);

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
        private static int8_t bme280_set_regs(uint8_t[] reg_addr, uint8_t[] reg_data, uint8_t len, bme280_dev dev)
        {
            int8_t rslt;
            uint8_t[] temp_buff;

            uint16_t temp_len;
            uint8_t reg_addr_cnt;

            /* Check for null pointer in the device structure*/
            rslt = null_ptr_check(dev);

            /* Check for arguments validity */
            if ((rslt == BME280_OK) && (reg_addr != null) && (reg_data != null))
            {
                if (len != 0)
                {
                    /* If interface selected is SPI */
                    if (dev.intf != bme280_intf.BME280_I2C_INTF)
                    {
                        for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++)
                        {
                            reg_addr[reg_addr_cnt] = (byte)(reg_addr[reg_addr_cnt] & 0x7F);
                        }
                    }

                    /* Burst write mode */
                    if (len > 1)
                    {
                        /* Interleave register address w.r.t data for
                         * burst write
                         */
                        temp_buff = new uint8_t[(len * 2) - 1];
                        interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
                        temp_len = (ushort)((len * 2) - 1);
                    }
                    else
                    {
                        temp_buff = reg_data;
                        temp_len = len;
                    }
                    rslt = dev.write(dev.dev_id, reg_addr[0], temp_buff, temp_len);

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
        private static int8_t bme280_set_sensor_settings(uint8_t desired_settings, bme280_dev dev)
        {
            int8_t rslt;
            uint8_t sensor_mode;

            /* Check for null pointer in the device structure*/
            rslt = null_ptr_check(dev);

            /* Proceed if null check is fine */
            if (rslt == BME280_OK)
            {
                rslt = bme280_get_sensor_mode(out sensor_mode, dev);
                if ((rslt == BME280_OK) && (sensor_mode != BME280_SLEEP_MODE))
                {
                    rslt = put_device_to_sleep(dev);
                }
                if (rslt == BME280_OK)
                {
                    /* Check if user wants to change oversampling
                     * settings
                     */
                    if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings) != FALSE)
                    {
                        rslt = set_osr_settings(desired_settings, dev.settings, dev);
                    }

                    /* Check if user wants to change filter and/or
                     * standby settings
                     */
                    if ((rslt == BME280_OK) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings) != FALSE)
                    {
                        rslt = set_filter_standby_settings(desired_settings, dev.settings, dev);
                    }
                }
            }

            return rslt;
        }

        /*!
         * @brief This API gets the oversampling, filter and standby duration
         * (normal mode) settings from the sensor.
         *
         * @param[in,out] dev : Structure instance of bme280_dev.
         *
         * @return Result of API execution status
         * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
         */
        private int8_t bme280_get_sensor_settings(bme280_dev dev)
        {
            int8_t rslt;
            uint8_t[] reg_data = new uint8_t[4];

            /* Check for null pointer in the device structure*/
            rslt = null_ptr_check(dev);

            /* Proceed if null check is fine */
            if (rslt == BME280_OK)
            {
                rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4, dev);
                if (rslt == BME280_OK)
                {
                    parse_device_settings(reg_data, dev.settings);
                }
            }

            return rslt;
        }

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
        int8_t bme280_set_sensor_mode(uint8_t sensor_mode, bme280_dev dev)
        {
            int8_t rslt;
            uint8_t last_set_mode;

            /* Check for null pointer in the device structure*/
            rslt = null_ptr_check(dev);
            if (rslt == BME280_OK)
            {
                rslt = bme280_get_sensor_mode(out last_set_mode, dev);

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
        private static int8_t bme280_get_sensor_mode(out uint8_t sensor_mode, bme280_dev dev)
        {
            int8_t rslt;
            sensor_mode = 0;

            /* Check for null pointer in the device structure*/
            rslt = null_ptr_check(dev);
            if (rslt == BME280_OK)
            {
                var data = new byte[1];
                /* Read the power mode register */
                rslt = bme280_get_regs(BME280_PWR_CTRL_ADDR, data, 1, dev);
                sensor_mode = data[0];

                /* Assign the power mode in the device structure */
                sensor_mode = (byte)(sensor_mode & (BME280_SENSOR_MODE_MSK));
            }

            return rslt;
        }

        /*!
         * @brief This API performs the soft reset of the sensor.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         *
         * @return Result of API execution status
         * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
         */
        static int8_t bme280_soft_reset(bme280_dev dev)
        {
            int8_t rslt;
            uint8_t[] reg_addr = new uint8_t[1] { BME280_RESET_ADDR };
            uint8_t[] status_reg = new uint8_t[1] { 0 };
            uint8_t try_run = 5;

            /* 0xB6 is the soft reset command */
            uint8_t[] soft_rst_cmd = new uint8_t[1] { BME280_SOFT_RESET_COMMAND };

            /* Check for null pointer in the device structure*/
            rslt = null_ptr_check(dev);

            /* Proceed if null check is fine */
            if (rslt == BME280_OK)
            {
                /* Write the soft reset command in the sensor */
                rslt = bme280_set_regs(reg_addr, soft_rst_cmd, 1, dev);

                if (rslt == BME280_OK)
                {
                    /* If NVM not copied yet, Wait for NVM to copy */
                    do
                    {
                        /* As per data sheet - Table 1, startup time is 2 ms. */
                        dev.delay_ms(2);
                        rslt = bme280_get_regs(BME280_STATUS_REG_ADDR, status_reg, 1, dev);
                    } while ((rslt == BME280_OK) && (try_run-- > 0) && ((status_reg[0] & BME280_STATUS_IM_UPDATE) != 0));

                    if ((status_reg[0] & BME280_STATUS_IM_UPDATE) != 0)
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
        int8_t bme280_get_sensor_data(uint8_t sensor_comp, bme280_data comp_data, bme280_dev dev)
        {
            int8_t rslt;

            /* Array to store the pressure, temperature and humidity data read from
             * the sensor
             */
            uint8_t[] reg_data = new uint8_t[BME280_P_T_H_DATA_LEN] { 0, 0, 0, 0, 0, 0, 0, 0 };
            bme280_uncomp_data uncomp_data =  new bme280_uncomp_data();

            /* Check for null pointer in the device structure*/
            rslt = null_ptr_check(dev);
            if ((rslt == BME280_OK) && (comp_data != null))
            {
                /* Read the pressure and temperature data from the sensor */
                rslt = bme280_get_regs(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN, dev);
                if (rslt == BME280_OK)
                {
                    /* Parse the read data from the sensor */
                    bme280_parse_sensor_data(reg_data, uncomp_data);

                    /* Compensate the pressure and/or temperature and/or
                     * humidity data from the sensor
                     */
                    rslt = bme280_compensate_data(sensor_comp, uncomp_data, comp_data, dev.calib_data);
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
         *
         *  @param[in] reg_data     : Contains register data which needs to be parsed
         *  @param[out] uncomp_data : Contains the uncompensated pressure, temperature
         *  and humidity data.
         */
        static void bme280_parse_sensor_data(uint8_t[] reg_data, bme280_uncomp_data uncomp_data)
        {
            /* Variables to store the sensor data */
            uint32_t data_xlsb;
            uint32_t data_lsb;
            uint32_t data_msb;

            /* Store the parsed register values for pressure data */
            data_msb = (uint32_t)reg_data[0] << 12;
            data_lsb = (uint32_t)reg_data[1] << 4;
            data_xlsb = (uint32_t)reg_data[2] >> 4;
            uncomp_data.pressure = data_msb | data_lsb | data_xlsb;

            /* Store the parsed register values for temperature data */
            data_msb = (uint32_t)reg_data[3] << 12;
            data_lsb = (uint32_t)reg_data[4] << 4;
            data_xlsb = (uint32_t)reg_data[5] >> 4;
            uncomp_data.temperature = data_msb | data_lsb | data_xlsb;

            /* Store the parsed register values for temperature data */
            data_lsb = (uint32_t)reg_data[6] << 8;
            data_msb = (uint32_t)reg_data[7];
            uncomp_data.humidity = data_msb | data_lsb;
        }

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
        static int8_t bme280_compensate_data(uint8_t sensor_comp,
                              bme280_uncomp_data uncomp_data,
                              bme280_data comp_data,
                              bme280_calib_data calib_data)
        {
            int8_t rslt = BME280_OK;

            if ((uncomp_data != null) && (comp_data != null) && (calib_data != null))
            {
                /* Initialize to zero */
                comp_data.temperature = 0;
                comp_data.pressure = 0;
                comp_data.humidity = 0;

                /* If pressure or temperature component is selected */
                if ((sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM)) != 0)
                {
                    /* Compensate the temperature data */
                    comp_data.temperature = compensate_temperature(uncomp_data, calib_data);
                }
                if ((sensor_comp & BME280_PRESS) != 0)
                {
                    /* Compensate the pressure data */
                    comp_data.pressure = compensate_pressure(uncomp_data, calib_data);
                }
                if ((sensor_comp & BME280_HUM) != 0)
                {
                    /* Compensate the humidity data */
                    comp_data.humidity = compensate_humidity(uncomp_data, calib_data);
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
         *
         * @param[in] desired_settings : Variable used to select the settings which
         * are to be set.
         * @param[in] dev : Structure instance of bme280_dev.
         *
         * @return Result of API execution status
         * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
         */
        private static int8_t set_osr_settings(uint8_t desired_settings,
                                       bme280_settings settings,
                                       bme280_dev dev)
        {
            int8_t rslt = BME280_W_INVALID_OSR_MACRO;

            if ((desired_settings & BME280_OSR_HUM_SEL) != 0)
            {
                rslt = set_osr_humidity_settings(settings, dev);
            }
            if ((desired_settings & (BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL)) != 0)
            {
                rslt = set_osr_press_temp_settings(desired_settings, settings, dev);
            }

            return rslt;
        }

        /*!
         * @brief This API sets the humidity oversampling settings of the sensor.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         *
         * @return Result of API execution status
         * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
         */
        static int8_t set_osr_humidity_settings(bme280_settings settings, bme280_dev dev)
        {
            int8_t rslt;
            uint8_t[] ctrl_hum =  new uint8_t[1];
            uint8_t[] ctrl_meas = new uint8_t[1];
            uint8_t[] reg_addr = new uint8_t[1] { BME280_CTRL_HUM_ADDR };

            ctrl_hum[0] = (byte)(settings.osr_h & BME280_CTRL_HUM_MSK);

            /* Write the humidity control value in the register */
            rslt = bme280_set_regs(reg_addr, ctrl_hum, 1, dev);

            /* Humidity related changes will be only effective after a
             * write operation to ctrl_meas register
             */
            if (rslt == BME280_OK)
            {
                reg_addr[0] = BME280_CTRL_MEAS_ADDR;
                rslt = bme280_get_regs(reg_addr[0], ctrl_meas, 1, dev);
                if (rslt == BME280_OK)
                {
                    rslt = bme280_set_regs(reg_addr, ctrl_meas, 1, dev);
                }
            }

            return rslt;
        }

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
                                                  bme280_settings settings,
                                                  bme280_dev dev)
        {
            int8_t rslt;
            uint8_t[] reg_addr = new uint8_t[1] { BME280_CTRL_MEAS_ADDR };
            uint8_t[] reg_data = new uint8_t[1];

            rslt = bme280_get_regs(reg_addr[0], reg_data, 1, dev);
            if (rslt == BME280_OK)
            {
                if ((desired_settings & BME280_OSR_PRESS_SEL) != 0)
                {
                    fill_osr_press_settings(ref reg_data[0], settings);
                }
                if ((desired_settings & BME280_OSR_TEMP_SEL) != 0)
                {
                    fill_osr_temp_settings(ref reg_data[0], settings);
                }

                /* Write the oversampling settings in the register */
                rslt = bme280_set_regs(reg_addr, reg_data, 1, dev);
            }

            return rslt;
        }

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
        private static int8_t set_filter_standby_settings(uint8_t desired_settings,
                                                  bme280_settings settings,
                                                  bme280_dev dev)
        {
            int8_t rslt;
            uint8_t[] reg_addr = new uint8_t[1] { BME280_CONFIG_ADDR };
            uint8_t[] reg_data =  new uint8_t[1];

            rslt = bme280_get_regs(reg_addr[0], reg_data, 1, dev);
            if (rslt == BME280_OK)
            {
                if ((desired_settings & BME280_FILTER_SEL) != 0)
                {
                    fill_filter_settings(ref reg_data[0], settings);
                }
                if ((desired_settings & BME280_STANDBY_SEL) != 0)
                {
                    fill_standby_settings(ref reg_data[0], settings);
                }

                /* Write the oversampling settings in the register */
                rslt = bme280_set_regs(reg_addr, reg_data, 1, dev);
            }

            return rslt;
        }

        /*!
         * @brief This internal API fills the filter settings provided by the user
         * in the data buffer so as to write in the sensor.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         * @param[out] reg_data : Variable which is filled according to the filter
         * settings data provided by the user.
         */
        static void fill_filter_settings(ref uint8_t reg_data, bme280_settings settings)
        {
            reg_data = (byte)((reg_data & ~(BME280_FILTER_MSK)) | ((settings.filter << BME280_FILTER_POS) & BME280_FILTER_MSK));
        }

        /*!
         * @brief This internal API fills the standby duration settings provided by the
         * user in the data buffer so as to write in the sensor.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         * @param[out] reg_data : Variable which is filled according to the standby
         * settings data provided by the user.
         */
        static void fill_standby_settings(ref uint8_t reg_data, bme280_settings settings)
        {
            reg_data = (byte)((reg_data & ~(BME280_STANDBY_MSK)) | ((settings.standby_time << BME280_STANDBY_POS) & BME280_STANDBY_MSK));
        }

        /*!
         * @brief This internal API fills the pressure oversampling settings provided by
         * the user in the data buffer so as to write in the sensor.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         * @param[out] reg_data : Variable which is filled according to the pressure
         * oversampling data provided by the user.
         */
        static void fill_osr_press_settings(ref uint8_t reg_data, bme280_settings settings)
        {
            reg_data = (byte)((reg_data & ~(BME280_CTRL_PRESS_MSK)) | ((settings.osr_p << BME280_CTRL_PRESS_POS) & BME280_CTRL_PRESS_MSK));
        }

        /*!
         * @brief This internal API fills the temperature oversampling settings provided
         * by the user in the data buffer so as to write in the sensor.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         * @param[out] reg_data : Variable which is filled according to the temperature
         * oversampling data provided by the user.
         */
        static void fill_osr_temp_settings(ref uint8_t reg_data, bme280_settings settings)
        {
            reg_data = (byte)((reg_data & ~(BME280_CTRL_TEMP_MSK)) | ((settings.osr_t << BME280_CTRL_TEMP_POS) & BME280_CTRL_TEMP_MSK));
        }

        /*!
         * @brief This internal API parse the oversampling(pressure, temperature
         * and humidity), filter and standby duration settings and store in the
         * device structure.
         *
         * @param[out] dev : Structure instance of bme280_dev.
         * @param[in] reg_data : Register data to be parsed.
         */
        static void parse_device_settings(uint8_t[] reg_data, bme280_settings settings)
        {
            settings.osr_h = (byte)(reg_data[0] & BME280_CTRL_HUM_MSK);
            settings.osr_p = (byte)((reg_data[2] & (BME280_CTRL_PRESS_MSK)) >> (BME280_CTRL_PRESS_POS));
            settings.osr_t = (byte)((reg_data[2] & (BME280_CTRL_TEMP_MSK)) >> (BME280_CTRL_TEMP_POS));
            settings.filter = (byte)((reg_data[3] & (BME280_FILTER_MSK)) >> (BME280_FILTER_POS));
            settings.standby_time = (byte)((reg_data[3] & (BME280_STANDBY_MSK)) >> (BME280_STANDBY_POS));
        }

        /*!
         * @brief This internal API writes the power mode in the sensor.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         * @param[in] sensor_mode : Variable which contains the power mode to be set.
         *
         * @return Result of API execution status.
         * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
         */
        static int8_t write_power_mode(uint8_t sensor_mode, bme280_dev dev)
        {
            int8_t rslt;
            uint8_t[] reg_addr = new uint8_t[1] { BME280_PWR_CTRL_ADDR };

            /* Variable to store the value read from power mode register */
            uint8_t[] sensor_mode_reg_val = new uint8_t[1];

            /* Read the power mode register */
            rslt = bme280_get_regs(reg_addr[0], sensor_mode_reg_val, 1, dev);

            /* Set the power mode */
            if (rslt == BME280_OK)
            {
                sensor_mode_reg_val[0] = (byte)((sensor_mode_reg_val[0] & ~(BME280_SENSOR_MODE_MSK)) | (sensor_mode & BME280_SENSOR_MODE_MSK));

                /* Write the power mode in the register */
                rslt = bme280_set_regs(reg_addr, sensor_mode_reg_val, 1, dev);
            }

            return rslt;
        }

        /*!
         * @brief This internal API puts the device to sleep mode.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         *
         * @return Result of API execution status.
         * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
         */
        static int8_t put_device_to_sleep(bme280_dev dev)
        {
            int8_t rslt;
            uint8_t[] reg_data = new uint8_t[4];
            bme280_settings settings = new bme280_settings();

            rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4, dev);
            if (rslt == BME280_OK)
            {
                parse_device_settings(reg_data, settings);
                rslt = bme280_soft_reset(dev);
                if (rslt == BME280_OK)
                {
                    rslt = reload_device_settings(settings, dev);
                }
            }

            return rslt;
        }

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
        private static int8_t reload_device_settings(bme280_settings settings, bme280_dev dev)
        {
            int8_t rslt;

            rslt = set_osr_settings(BME280_ALL_SETTINGS_SEL, settings, dev);
            if (rslt == BME280_OK)
            {
                rslt = set_filter_standby_settings(BME280_ALL_SETTINGS_SEL, settings, dev);
            }

            return rslt;
        }

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
        private static double compensate_temperature(bme280_uncomp_data uncomp_data, bme280_calib_data calib_data)
        {
            double var1;
            double var2;
            double temperature;
            double temperature_min = -40;
            double temperature_max = 85;

            var1 = ((double)uncomp_data.temperature) / 16384.0 - ((double)calib_data.dig_T1) / 1024.0;
            var1 = var1 * ((double)calib_data.dig_T2);
            var2 = (((double)uncomp_data.temperature) / 131072.0 - ((double)calib_data.dig_T1) / 8192.0);
            var2 = (var2 * var2) * ((double)calib_data.dig_T3);
            calib_data.t_fine = (int32_t)(var1 + var2);
            temperature = (var1 + var2) / 5120.0;
            if (temperature < temperature_min)
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
         *
         * @param[in] uncomp_data : Contains the uncompensated pressure data.
         * @param[in] calib_data : Pointer to the calibration data structure.
         *
         * @return Compensated pressure data.
         * @retval Compensated pressure data in double.
         */
        private static double compensate_pressure(bme280_uncomp_data uncomp_data,
                                          bme280_calib_data calib_data)
        {
            double var1;
            double var2;
            double var3;
            double pressure;
            double pressure_min = 30000.0;
            double pressure_max = 110000.0;

            var1 = ((double)calib_data.t_fine / 2.0) - 64000.0;
            var2 = var1 * var1 * ((double)calib_data.dig_P6) / 32768.0;
            var2 = var2 + var1 * ((double)calib_data.dig_P5) * 2.0;
            var2 = (var2 / 4.0) + (((double)calib_data.dig_P4) * 65536.0);
            var3 = ((double)calib_data.dig_P3) * var1 * var1 / 524288.0;
            var1 = (var3 + ((double)calib_data.dig_P2) * var1) / 524288.0;
            var1 = (1.0 + var1 / 32768.0) * ((double)calib_data.dig_P1);

            /* avoid exception caused by division by zero */
            if (var1 != 0.0)
            {
                pressure = 1048576.0 - (double)uncomp_data.pressure;
                pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
                var1 = ((double)calib_data.dig_P9) * pressure * pressure / 2147483648.0;
                var2 = pressure * ((double)calib_data.dig_P8) / 32768.0;
                pressure = pressure + (var1 + var2 + ((double)calib_data.dig_P7)) / 16.0;
                if (pressure < pressure_min)
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
         *
         * @param[in] uncomp_data : Contains the uncompensated humidity data.
         * @param[in] calib_data : Pointer to the calibration data structure.
         *
         * @return Compensated humidity data.
         * @retval Compensated humidity data in double.
         */
        private static double compensate_humidity(bme280_uncomp_data uncomp_data,
                                          bme280_calib_data calib_data)
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

            var1 = ((double)calib_data.t_fine) - 76800.0;
            var2 = (((double)calib_data.dig_H4) * 64.0 + (((double)calib_data.dig_H5) / 16384.0) * var1);
            var3 = uncomp_data.humidity - var2;
            var4 = ((double)calib_data.dig_H2) / 65536.0;
            var5 = (1.0 + (((double)calib_data.dig_H3) / 67108864.0) * var1);
            var6 = 1.0 + (((double)calib_data.dig_H6) / 67108864.0) * var1 * var5;
            var6 = var3 * var4 * (var5 * var6);
            humidity = var6 * (1.0 - ((double)calib_data.dig_H1) * var6 / 524288.0);
            if (humidity > humidity_max)
            {
                humidity = humidity_max;
            }
            else if (humidity < humidity_min)
            {
                humidity = humidity_min;
            }

            return humidity;
        }

        /*!
         * @brief This internal API reads the calibration data from the sensor, parse
         * it and store in the device structure.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         *
         * @return Result of API execution status
         * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
         */
        private static int8_t get_calib_data(bme280_dev dev)
        {
            int8_t rslt;
            uint8_t reg_addr = BME280_TEMP_PRESS_CALIB_DATA_ADDR;

            /* Array to store calibration data */
            uint8_t[] calib_data = new uint8_t[BME280_TEMP_PRESS_CALIB_DATA_LEN] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            /* Read the calibration data from the sensor */
            rslt = bme280_get_regs(reg_addr, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN, dev);
            if (rslt == BME280_OK)
            {
                /* Parse temperature and pressure calibration data and store
                 * it in device structure
                 */
                parse_temp_press_calib_data(calib_data, dev);
                reg_addr = BME280_HUMIDITY_CALIB_DATA_ADDR;

                calib_data = new uint8_t[BME280_HUMIDITY_CALIB_DATA_LEN];
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
         *
         * @param[in] reg_addr : Contains the register address array.
         * @param[out] temp_buff : Contains the temporary buffer to store the
         * register data and register address.
         * @param[in] reg_data : Contains the register data to be written in the
         * temporary buffer.
         * @param[in] len : No of bytes of data to be written for burst write.
         */
        private static void interleave_reg_addr(uint8_t[] reg_addr, uint8_t[] temp_buff, uint8_t[] reg_data, uint8_t len)
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
         *  pressure calibration data and store it in the device structure.
         *
         *  @param[out] dev : Structure instance of bme280_dev to store the calib data.
         *  @param[in] reg_data : Contains the calibration data to be parsed.
         */
        private static void parse_temp_press_calib_data(uint8_t[] reg_data, bme280_dev dev)
        {
            bme280_calib_data calib_data = dev.calib_data;

            calib_data.dig_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
            calib_data.dig_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
            calib_data.dig_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
            calib_data.dig_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
            calib_data.dig_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
            calib_data.dig_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
            calib_data.dig_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
            calib_data.dig_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
            calib_data.dig_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
            calib_data.dig_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
            calib_data.dig_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
            calib_data.dig_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
            calib_data.dig_H1 = reg_data[25];
        }

        private static ushort BME280_CONCAT_BYTES(byte v1, byte v2)
        {
            return (ushort)((v1 << 8) | v2);
        }

        /*!
         *  @brief This internal API is used to parse the humidity calibration data
         *  and store it in device structure.
         *
         *  @param[out] dev : Structure instance of bme280_dev to store the calib data.
         *  @param[in] reg_data : Contains calibration data to be parsed.
         */
        private static void parse_humidity_calib_data(uint8_t[] reg_data, bme280_dev dev)
        {
            bme280_calib_data calib_data = dev.calib_data;
            int16_t dig_H4_lsb;
            int16_t dig_H4_msb;
            int16_t dig_H5_lsb;
            int16_t dig_H5_msb;

            calib_data.dig_H2 = (int16_t)BitConverter.ToInt16(reg_data, 0);
            calib_data.dig_H3 = reg_data[2];
            dig_H4_msb = (int16_t)(reg_data[3] * 16);
            dig_H4_lsb = (int16_t)(reg_data[4] & 0x0F);
            calib_data.dig_H4 = (short)(dig_H4_msb | dig_H4_lsb);
            dig_H5_msb = (int16_t)(reg_data[5] * 16);
            dig_H5_lsb = (int16_t)(reg_data[4] >> 4);
            calib_data.dig_H5 = (short)(dig_H5_msb | dig_H5_lsb);
            calib_data.dig_H6 = (int8_t)reg_data[6];
        }

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
        private static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings)
        {
            uint8_t settings_changed = FALSE;

            if ((sub_settings & desired_settings) != 0)
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
         * @brief This internal API is used to validate the device pointer for
         * null conditions.
         *
         * @param[in] dev : Structure instance of bme280_dev.
         *
         * @return Result of API execution status
         * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
         */
        private static int8_t null_ptr_check(bme280_dev dev)
        {
            int8_t rslt;

            if ((dev == null) || (dev.read == null) || (dev.write == null) || (dev.delay_ms == null))
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
         * @file    bme280_defs.h
         * @date    26 Aug 2019
         * @version 3.3.7
         * @brief
         *
         */

        private const UINT8_C TRUE = (UINT8_C)(1);
        private const UINT8_C FALSE = (UINT8_C)(0);

        /**\name I2C addresses */
        private const UINT8_C BME280_I2C_ADDR_PRIM = (UINT8_C)(0x76);
        private const UINT8_C BME280_I2C_ADDR_SEC = (UINT8_C)(0x77);

        /**\name BME280 chip identifier */
        private const UINT8_C BME280_CHIP_ID = (UINT8_C)(0x60);

        /**\name Register Address */
        private const UINT8_C BME280_CHIP_ID_ADDR = (UINT8_C)(0xD0);
        private const UINT8_C BME280_RESET_ADDR = (UINT8_C)(0xE0);
        private const UINT8_C BME280_TEMP_PRESS_CALIB_DATA_ADDR = (UINT8_C)(0x88);
        private const UINT8_C BME280_HUMIDITY_CALIB_DATA_ADDR = (UINT8_C)(0xE1);
        private const UINT8_C BME280_PWR_CTRL_ADDR = (UINT8_C)(0xF4);
        private const UINT8_C BME280_CTRL_HUM_ADDR = (UINT8_C)(0xF2);
        private const UINT8_C BME280_CTRL_MEAS_ADDR = (UINT8_C)(0xF4);
        private const UINT8_C BME280_CONFIG_ADDR = (UINT8_C)(0xF5);
        private const UINT8_C BME280_DATA_ADDR = (UINT8_C)(0xF7);

        /**\name API success code */
        private const INT8_C BME280_OK = (INT8_C)(0);

        /**\name API error codes */
        private const INT8_C BME280_E_NULL_PTR = (INT8_C)(-1);
        private const INT8_C BME280_E_DEV_NOT_FOUND = (INT8_C)(-2);
        private const INT8_C BME280_E_INVALID_LEN = (INT8_C)(-3);
        private const INT8_C BME280_E_COMM_FAIL = (INT8_C)(-4);
        private const INT8_C BME280_E_SLEEP_MODE_FAIL = (INT8_C)(-5);
        private const INT8_C BME280_E_NVM_COPY_FAILED = (INT8_C)(-6);

        /**\name API warning codes */
        private const INT8_C BME280_W_INVALID_OSR_MACRO = (INT8_C)(1);

        /**\name Macros related to size */
        private const UINT8_C BME280_TEMP_PRESS_CALIB_DATA_LEN = (UINT8_C)(26);
        private const UINT8_C BME280_HUMIDITY_CALIB_DATA_LEN = (UINT8_C)(7);
        private const UINT8_C BME280_P_T_H_DATA_LEN = (UINT8_C)(8);

        /**\name Sensor power modes */
        private const UINT8_C BME280_SLEEP_MODE = (UINT8_C)(0x00);
        private const UINT8_C BME280_FORCED_MODE = (UINT8_C)(0x01);
        private const UINT8_C BME280_NORMAL_MODE = (UINT8_C)(0x03);



        /**\name Macros for bit masking */
        private const UINT8_C BME280_SENSOR_MODE_MSK = (UINT8_C)(0x03);
        private const UINT8_C BME280_SENSOR_MODE_POS = (UINT8_C)(0x00);

        private const UINT8_C BME280_CTRL_HUM_MSK = (UINT8_C)(0x07);
        private const UINT8_C BME280_CTRL_HUM_POS = (UINT8_C)(0x00);

        private const UINT8_C BME280_CTRL_PRESS_MSK = (UINT8_C)(0x1C);
        private const UINT8_C BME280_CTRL_PRESS_POS = (UINT8_C)(0x02);

        private const UINT8_C BME280_CTRL_TEMP_MSK = (UINT8_C)(0xE0);
        private const UINT8_C BME280_CTRL_TEMP_POS = (UINT8_C)(0x05);

        private const UINT8_C BME280_FILTER_MSK = (UINT8_C)(0x1C);
        private const UINT8_C BME280_FILTER_POS = (UINT8_C)(0x02);

        private const UINT8_C BME280_STANDBY_MSK = (UINT8_C)(0xE0);
        private const UINT8_C BME280_STANDBY_POS = (UINT8_C)(0x05);

        /**\name Sensor component selection macros
         * These values are internal for API implementation. Don't relate this to
         * data sheet.
         */
        private const UINT8_C BME280_PRESS = (UINT8_C)(1);
        private const UINT8_C BME280_TEMP = (UINT8_C)(1 << 1);
        private const UINT8_C BME280_HUM = (UINT8_C)(1 << 2);
        private const UINT8_C BME280_ALL = (UINT8_C)(0x07);

        /**\name Settings selection macros */
        private const UINT8_C BME280_OSR_PRESS_SEL = (UINT8_C)(1);
        private const UINT8_C BME280_OSR_TEMP_SEL = (UINT8_C)(1 << 1);
        private const UINT8_C BME280_OSR_HUM_SEL = (UINT8_C)(1 << 2);
        private const UINT8_C BME280_FILTER_SEL = (UINT8_C)(1 << 3);
        private const UINT8_C BME280_STANDBY_SEL = (UINT8_C)(1 << 4);
        private const UINT8_C BME280_ALL_SETTINGS_SEL = (UINT8_C)(0x1F);

        /**\name Oversampling macros */
        private const UINT8_C BME280_NO_OVERSAMPLING = (UINT8_C)(0x00);
        private const UINT8_C BME280_OVERSAMPLING_1X = (UINT8_C)(0x01);
        private const UINT8_C BME280_OVERSAMPLING_2X = (UINT8_C)(0x02);
        private const UINT8_C BME280_OVERSAMPLING_4X = (UINT8_C)(0x03);
        private const UINT8_C BME280_OVERSAMPLING_8X = (UINT8_C)(0x04);
        private const UINT8_C BME280_OVERSAMPLING_16X = (UINT8_C)(0x05);

        /**\name Standby duration selection macros */
        private const UINT8_C BME280_STANDBY_TIME_0_5_MS = (0x00);
        private const UINT8_C BME280_STANDBY_TIME_62_5_MS = (0x01);
        private const UINT8_C BME280_STANDBY_TIME_125_MS = (0x02);
        private const UINT8_C BME280_STANDBY_TIME_250_MS = (0x03);
        private const UINT8_C BME280_STANDBY_TIME_500_MS = (0x04);
        private const UINT8_C BME280_STANDBY_TIME_1000_MS = (0x05);
        private const UINT8_C BME280_STANDBY_TIME_10_MS = (0x06);
        private const UINT8_C BME280_STANDBY_TIME_20_MS = (0x07);

        /**\name Filter coefficient selection macros */
        private const UINT8_C BME280_FILTER_COEFF_OFF = (0x00);
        private const UINT8_C BME280_FILTER_COEFF_2 = (0x01);
        private const UINT8_C BME280_FILTER_COEFF_4 = (0x02);
        private const UINT8_C BME280_FILTER_COEFF_8 = (0x03);
        private const UINT8_C BME280_FILTER_COEFF_16 = (0x04);

        private const UINT8_C BME280_STATUS_REG_ADDR = (0xF3);
        private const UINT8_C BME280_SOFT_RESET_COMMAND = (0xB6);
        private const UINT8_C BME280_STATUS_IM_UPDATE = (0x01);

        /*!
         * @brief Interface selection Enums
         */
        enum bme280_intf
        {
            /*! SPI interface */
            BME280_SPI_INTF,

            /*! I2C interface */
            BME280_I2C_INTF
        };

        /*!
         * @brief Type definitions
         */
        private delegate int8_t bme280_com_fptr_t(uint8_t dev_id, uint8_t reg_addr, uint8_t[] data, uint16_t len);
        private delegate void bme280_delay_fptr_t(uint32_t period);

        /*!
         * @brief Calibration data
         */
        private class bme280_calib_data
        {
            /**
             * @ Trim Variables
             */

            /**@{*/
            public uint16_t dig_T1;
            public int16_t dig_T2;
            public int16_t dig_T3;
            public uint16_t dig_P1;
            public int16_t dig_P2;
            public int16_t dig_P3;
            public int16_t dig_P4;
            public int16_t dig_P5;
            public int16_t dig_P6;
            public int16_t dig_P7;
            public int16_t dig_P8;
            public int16_t dig_P9;
            public uint8_t dig_H1;
            public int16_t dig_H2;
            public uint8_t dig_H3;
            public int16_t dig_H4;
            public int16_t dig_H5;
            public int8_t dig_H6;
            public int32_t t_fine;

            /**@}*/
        };

        /*!
         * @brief bme280 sensor structure which comprises of temperature, pressure and
         * humidity data
         */
        private class bme280_data
        {
            /*! Compensated pressure */
            public double pressure;

            /*! Compensated temperature */
            public double temperature;

            /*! Compensated humidity */
            public double humidity;
        };

        /*!
         * @brief bme280 sensor structure which comprises of uncompensated temperature,
         * pressure and humidity data
         */
        private class bme280_uncomp_data
        {
            /*! un-compensated pressure */
            public uint32_t pressure;

            /*! un-compensated temperature */
            public uint32_t temperature;

            /*! un-compensated humidity */
            public uint32_t humidity;
        };

        /*!
         * @brief bme280 sensor settings structure which comprises of mode,
         * oversampling and filter settings.
         */
        private class bme280_settings
        {
            /*! pressure oversampling */
            public uint8_t osr_p;

            /*! temperature oversampling */
            public uint8_t osr_t;

            /*! humidity oversampling */
            public uint8_t osr_h;

            /*! filter coefficient */
            public uint8_t filter;

            /*! standby time */
            public uint8_t standby_time;
        };

        /*!
         * @brief bme280 device structure
         */
        private class bme280_dev
        {
            /*! Chip Id */
            public uint8_t chip_id;

            /*! Device Id */
            public uint8_t dev_id;

            /*! SPI/I2C interface */
            public bme280_intf intf;

            /*! Read function pointer */
            public bme280_com_fptr_t read;

            /*! Write function pointer */
            public bme280_com_fptr_t write;

            /*! Delay function pointer */
            public bme280_delay_fptr_t delay_ms;

            /*! Trim data */
            public bme280_calib_data calib_data;

            /*! Sensor settings */
            public bme280_settings settings;

            public bme280_dev()
            {
                calib_data = new bme280_calib_data();
                settings = new bme280_settings();
            }
        }

        public class Measurement
        {
            public double Temperature { get; }
            public double Humidity { get; }
            public double Pressure { get; }
            public DateTime When { get; }

            public Measurement(double temperature, double humidity, double pressure, DateTime when)
            {
                Temperature = temperature;
                Humidity = humidity;
                Pressure = pressure;
                When = when;
            }
        }
    }
}
