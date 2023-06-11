using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using RosSharp.RosBridgeClient.MessageTypes.JetsonOnWheels;

namespace RosSharp.RosBridgeClient
{
    public class ClockSync : UnityServiceConsumers<MessageTypes.JetsonOnWheels.ClockSyncRequest, MessageTypes.JetsonOnWheels.ClockSyncResponse>
    {
        public static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
        public double avgDelay = 0;
        public int sampleSize = 20;
        public int sampleCounter = 1;
        private double clockDiff = 0;
        public double avgcClockDiff = 0;
        public double avgSyncError = 0;
        public double stdSyncError = 0;
        public double syncError = 0;
        public List<double> syncErrorHistory;
        public bool isSync = false;
        private MessageTypes.JetsonOnWheels.ClockSyncRequest clockSyncRequest;

        private bool waitForResponse = false;

        void start()
        {
            clockSyncRequest = new MessageTypes.JetsonOnWheels.ClockSyncRequest();
        }
        void Update()
        {
            if (sampleCounter > sampleSize && avgSyncError + stdSyncError < 0.1)
            {
                isSync = true;
                return;
            }

            if (!waitForResponse){
                waitForResponse = true;
                CallService(clockSyncRequest);
            }
        }

        protected override void ServiceResponseHandler(MessageTypes.JetsonOnWheels.ClockSyncResponse message)
        {
            Debug.Log("now: " + message.now + ", delay: " + message.delay + ", sync_error: " + message.sync_error);
            sampleCounter++;
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            clockDiff = message.now - timeSpan.TotalSeconds + message.delay * 0.5;
            avgcClockDiff = ((sampleCounter - 1) * avgcClockDiff + clockDiff) / (sampleCounter);
            avgDelay = ((sampleCounter - 1) * avgDelay + message.delay) / (sampleCounter);
            syncErrorHistory.Add(message.sync_error);
            calc_mean_std(syncErrorHistory, out avgSyncError, out stdSyncError);
            Debug.Log("avgDelay: " + avgDelay + ", avgcClockDiff: " + avgcClockDiff);

            // Update the request with the previous clock and the predicted clock on the jetson
            clockSyncRequest = new MessageTypes.JetsonOnWheels.ClockSyncRequest(message.now, timeSpan.TotalSeconds + avgcClockDiff);
            waitForResponse = false;
        }

        public double getRosClockTime()
        {
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            return timeSpan.TotalSeconds + avgcClockDiff;
        }

        public void Now(out uint secs, out uint nsecs)
        {
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            double msecs = timeSpan.TotalMilliseconds + avgcClockDiff * 1000;
            secs = (uint)(msecs / 1000);
            nsecs = (uint)((msecs / 1000 - secs) * 1e+9);
        }

        public double diff(RosSharp.RosBridgeClient.MessageTypes.Std.Time rosTime)
        {
            double time = rosTime.secs + rosTime.nsecs * 1e-10;
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            return timeSpan.TotalSeconds + avgcClockDiff - time;
        }

        public void calc_mean_std(List<double> arr, out double average, out double std)
        {
            double avg = arr.Sum() / arr.Count;
            double sumOfSquaresOfDifferences = arr.Select(val => (val - avg) * (val - avg)).Sum();
            average = avg;
            std = Math.Sqrt(sumOfSquaresOfDifferences / arr.Count);
        }
    }
}