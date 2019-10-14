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
        private Task _initializeTask;

        public DeviceManager()
        {
            Initialize();
        }

        public async Task<string> GetMeasurement()
        {
            try
            {
                await Initialize().ConfigureAwait(false);

                var writeData = new byte[] { 0x00 };
                var readData = new byte[10];
                _device.WriteRead(writeData, readData);

                return string.Join(' ', readData.Select(b => b.ToString("x02", CultureInfo.InvariantCulture)));
            }
            catch
            {
                _device = null;
                _initializeTask = null;
                throw;
            }
        }

        private Task Initialize()
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
                    }
                    catch
                    {
                        _device = null;
                        _initializeTask = null;
                        throw;
                    }
                });
            }

            return _initializeTask;
        }
    }
}
