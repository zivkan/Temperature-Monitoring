using System;
using System.Diagnostics;
using System.Threading;
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
        private BackgroundTaskDeferral _deferral;
        private AppServiceConnection _connection;
        private CancellationTokenSource _cancellationTokenSource;

        public void Run(IBackgroundTaskInstance taskInstance)
        {
            if (taskInstance == null)
            {
                return;
            }

            _deferral = taskInstance.GetDeferral();
            taskInstance.Canceled += TaskInstance_Canceled;

            Debug.WriteLine("Starting " + taskInstance.InstanceId);

            if (taskInstance.TriggerDetails == null)
            {
                _cancellationTokenSource = new CancellationTokenSource();

                Task.Delay(10000).ContinueWith(TimerTick);
            }
            else if (taskInstance.TriggerDetails is AppServiceTriggerDetails appServiceTriggerDetails)
            {
                _connection = appServiceTriggerDetails.AppServiceConnection;
                _connection.RequestReceived += Connection_RequestReceived;
            }
            else
            {
                _deferral.Complete();
            }
        }

        private async void TimerTick(Task task)
        {
            if (_cancellationTokenSource?.IsCancellationRequested != false)
            {
                return;
            }

            Debug.WriteLine("Timer tick at {0:o}", DateTime.UtcNow);

            if (_connection == null)
            {
                try
                {
                    var connection = new AppServiceConnection();
                    connection.PackageFamilyName = Package.Current.Id.FamilyName;
                    connection.AppServiceName = "Sensor";
                    var result = await connection.OpenAsync();
                    if (result == AppServiceConnectionStatus.Success)
                    {
                        _connection = connection;
                    }
                }
                catch (Exception e)
                {
                    Debug.WriteLine("Excepion trying to open connection: " + e.Message);
                }
            }

            if (_connection != null)
            {
                var message = new ValueSet();
                message["ping"] = "ping";
                try
                {
                    var response = await _connection.SendMessageAsync(message);
                    Debug.WriteLine("Message send status: " + response.Status);
                }
                catch (Exception e)
                {
                    Debug.WriteLine("Exception sending message: " + e.Message);
                }
            }

            Task.Delay(10000).ContinueWith(TimerTick);
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

        private void TaskInstance_Canceled(IBackgroundTaskInstance sender, BackgroundTaskCancellationReason reason)
        {
            Debug.WriteLine("Canceled " + sender.InstanceId);

            _cancellationTokenSource?.Cancel();

            if (_connection != null)
            {
                _connection.Dispose();
                _connection = null;
            }

            _deferral.Complete();
        }
    }
}
