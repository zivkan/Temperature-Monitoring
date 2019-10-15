using System;
using System.Globalization;
using System.Linq;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;

namespace zivkan.TempMon.Service
{
    internal sealed class DeviceManager
    {
        private I2cDevice _device;
        private Bme280 _bme280;
        private Task _initializeTask;

        public DeviceManager()
        {
            InitializeAsync();
        }

        public async Task<Bme280.Measurement> GetMeasurementAsync()
        {
            try
            {
                await InitializeAsync().ConfigureAwait(false);

                var measurement = await _bme280.GetMeasurementAsync().ConfigureAwait(false);

                return measurement;
            }
            catch
            {
                _device = null;
                _bme280 = null;
                _initializeTask = null;
                throw;
            }
        }

        private Task InitializeAsync()
        {
            if (_initializeTask == null || !_initializeTask.IsCompletedSuccessfully)
            {
                _initializeTask = Task.Run(async ()=>
                {
                    try
                    {
                        var devices = await DeviceInformation.FindAllAsync(I2cDevice.GetDeviceSelector());

                        var settings = new I2cConnectionSettings(118)
                        {
                            BusSpeed = I2cBusSpeed.FastMode
                        };

                        _device = await I2cDevice.FromIdAsync(devices[0].Id, settings);

                        _bme280 = await Bme280.CreateAsync(_device).ConfigureAwait(false);
                    }
                    catch
                    {
                        _device = null;
                        _bme280 = null;
                        _initializeTask = null;
                        throw;
                    }
                });
            }

            return _initializeTask;
        }
    }
}
