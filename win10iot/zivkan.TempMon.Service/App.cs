﻿using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using Windows.ApplicationModel.AppService;
using Windows.ApplicationModel.Background;
using Windows.Foundation.Collections;

namespace zivkan.TempMon.Service
{
    internal sealed class App
    {
        public static App Instance
        {
            get
            {
                if (_instance == null)
                {
                    throw new InvalidOperationException("App must be created first");
                }

                return _instance;
            }
        }

        public static void Create(IBackgroundTaskInstance taskInstance)
        {
            if (_instance != null)
            {
                throw new Exception("App has already been created");
            }

            var app = new App(taskInstance);
            _instance = app;
        }

        private static App _instance;

        private IBackgroundTaskInstance _backgroundTaskInstance;
        private BackgroundTaskDeferral _deferral;

        internal void AddConnection(IBackgroundTaskInstance taskInstance, AppServiceTriggerDetails appServiceTriggerDetails)
        {
            _connectionManager.AddConnection(taskInstance, appServiceTriggerDetails);
        }

        private CancellationTokenSource _cancellationTokenSource;
        private ConnectionManager _connectionManager;

        private App(IBackgroundTaskInstance taskInstance)
        {
            _backgroundTaskInstance = taskInstance;
            _deferral = taskInstance.GetDeferral();
            _cancellationTokenSource = new CancellationTokenSource();
            _connectionManager = new ConnectionManager();

            taskInstance.Canceled += App_Closing;

            SetNextTimer();
        }

        private void App_Closing(IBackgroundTaskInstance sender, BackgroundTaskCancellationReason reason)
        {
            sender.Canceled -= App_Closing;

            _cancellationTokenSource.Cancel();

            _connectionManager.CloseAll();

            _deferral.Complete();
        }

        private void SetNextTimer()
        {
            var now = DateTime.UtcNow;
            var next = new DateTime(now.Year, now.Month, now.Day, now.Hour, now.Minute, 0, now.Kind)
                .AddMinutes(1);
            var delay = next - now;
            Task.Delay(delay).ContinueWith(TimerTickAsync);
        }

        private async Task TimerTickAsync(Task task)
        {
            var deferral = _backgroundTaskInstance.GetDeferral();
            try
            {
                if (_cancellationTokenSource.IsCancellationRequested)
                {
                    return;
                }

                SetNextTimer();

                var now = DateTime.UtcNow.ToString("o");

                Debug.WriteLine("Timer tick at " + now);

                var message = new ValueSet();
                message["time"] = now;
                await _connectionManager.BroadcastMessageAsync(message);
            }
            finally
            {
                deferral.Complete();
            }
        }
    }
}