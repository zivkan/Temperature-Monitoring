using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Windows.ApplicationModel.AppService;
using Windows.ApplicationModel.Background;
using Windows.Foundation.Collections;

namespace zivkan.TempMon.Service
{
    internal sealed class ConnectionManager
    {
        private Dictionary<AppServiceConnection, ConnectionInfo> _connections;

        public ConnectionManager()
        {
            _connections = new Dictionary<AppServiceConnection, ConnectionInfo>();
        }

        public void AddConnection(IBackgroundTaskInstance taskInstance, AppServiceTriggerDetails triggerDetails)
        {
            var deferral = taskInstance.GetDeferral();
            var connection = triggerDetails.AppServiceConnection;

            taskInstance.Canceled += Connection_Disconnected;
            connection.RequestReceived += Connection_RequestReceived;
            connection.ServiceClosed += Connection_ServiceClosed;

            var connectionInfo = new ConnectionInfo(deferral, taskInstance);
            _connections.Add(connection, connectionInfo);

            Debug.WriteLine("New connection " + taskInstance.InstanceId);
        }

        public async Task BroadcastMessageAsync(ValueSet message)
        {
            List<AppServiceConnection> toRemove = null;

            foreach (var kvp in _connections)
            {
                try
                {
                    var response = await kvp.Key.SendMessageAsync(message);
                    if (response.Status != AppServiceResponseStatus.Success)
                    {
                        if (toRemove == null)
                        {
                            toRemove = new List<AppServiceConnection>();
                        }
                        toRemove.Add(kvp.Key);
                    }
                }
                catch
                {
                    if (toRemove == null)
                    {
                        toRemove = new List<AppServiceConnection>();
                    }

                    toRemove.Add(kvp.Key);
                }
            }

            if (toRemove != null)
            {
                foreach (var connection in toRemove)
                {
                    Remove(connection);
                }
            }
        }

        private void Connection_ServiceClosed(AppServiceConnection sender, AppServiceClosedEventArgs args)
        {
            Debug.WriteLine("Service closed");

            Remove(sender);
        }

        private void Connection_RequestReceived(AppServiceConnection sender, AppServiceRequestReceivedEventArgs args)
        {
            Debug.WriteLine("Connection request reveived: " + MessageAsJson(args.Request.Message));
        }

        private void Connection_Disconnected(IBackgroundTaskInstance sender, BackgroundTaskCancellationReason reason)
        {
            Debug.WriteLine("Disconnection " + sender.InstanceId);

            if (sender.TriggerDetails is AppServiceTriggerDetails triggerDetails)
            {
                var connection = triggerDetails.AppServiceConnection;
                Remove(connection);
            }
        }

        internal void CloseAll()
        {
            while (_connections.Count > 0)
            {
                var connection = _connections.First();
                Remove(connection.Key);
            }
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

        private void Remove(AppServiceConnection connection)
        {
            if (_connections.TryGetValue(connection, out var connectionInfo))
            {
                connection.RequestReceived -= Connection_RequestReceived;
                connectionInfo.TaskInstance.Canceled -= Connection_Disconnected;

                _connections.Remove(connection);

                connection.Dispose();
                connectionInfo.Deferral.Complete();
            }
        }

        private class ConnectionInfo
        {
            public BackgroundTaskDeferral Deferral { get; }
            public IBackgroundTaskInstance TaskInstance { get; }

            public ConnectionInfo(BackgroundTaskDeferral deferral,
                IBackgroundTaskInstance taskInstance)
            {
                Deferral = deferral;
                TaskInstance = taskInstance;
            }
        }
    }
}
