using System;
using System.Diagnostics;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Windows.ApplicationModel;
using Windows.ApplicationModel.AppService;
using Windows.ApplicationModel.Background;
using Windows.Foundation.Collections;

namespace zivkan.TempMon.Service
{
    public sealed class StartupTask : IBackgroundTask
    {
        private AppServiceConnection _connection;

        public void Run(IBackgroundTaskInstance taskInstance)
        {
            if (taskInstance == null)
            {
                return;
            }

            Debug.WriteLine("Starting " + taskInstance.InstanceId);

            switch (taskInstance.TriggerDetails)
            {
                case null:
                    App.Create(taskInstance);
                    CreateConnectionAsync();
                    break;

                case AppServiceTriggerDetails appServiceTriggerDetails:
                    App.Instance.AddConnection(taskInstance, appServiceTriggerDetails);
                    break;
            }
        }

        private async Task CreateConnectionAsync()
        {
            var connection = new AppServiceConnection();
            connection.PackageFamilyName = Package.Current.Id.FamilyName;
            connection.AppServiceName = "Sensor";
            connection.RequestReceived += Connection_RequestReceived;
            connection.ServiceClosed += Connection_ServiceClosed;
            var result = await connection.OpenAsync();
            if (result == AppServiceConnectionStatus.Success)
            {
                _connection = connection;
            }
            else
            {
                Debug.WriteLine("Error connecting to app service");
            }
        }

        private void Connection_ServiceClosed(AppServiceConnection sender, AppServiceClosedEventArgs args)
        {
            Debug.WriteLine("Connection service closed");
            _connection.RequestReceived -= Connection_RequestReceived;
            _connection.ServiceClosed -= Connection_ServiceClosed;
            _connection = null;
        }

        private void Connection_RequestReceived(AppServiceConnection sender, AppServiceRequestReceivedEventArgs args)
        {
            var message = args.Request.Message;
            var output = MessageAsJson(message);
            Debug.WriteLine("Got message: " + output);
        }

        private string MessageAsJson(ValueSet message)
        {
            var obj = new JObject();

            foreach (var kvp in message)
            {
                obj[kvp.Key] = kvp.Value.ToString();
            }

            return obj.ToString(Formatting.None);
        }
    }
}
