using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient
{
    public class CarCommandsPublisher : UnityPublisher<MessageTypes.Nav.Odometry>
    {
        private MessageTypes.Nav.Odometry message;
        private GameObject RosConnector;
        private ClockSync clockSync;
        public int throttleForwardValue = 1000;
        public int throttleStoppedValue = 0;
        public int throttleReverseValue = -1000;
        public int maxSteeringTickLeft = -7500;
        public int maxSteeringTickRight = 7500;
        public int maxSteeringLeft = 40;
        public int maxSteeringRight = -40;

        // Start is called before the first frame update
        protected override void Start()
        {
            base.Start();
            InitializeMessage();
            RosConnector = GameObject.Find("RosConnector");
            clockSync = RosConnector.GetComponent<ClockSync>();

            if (!LogitechGSDK.LogiSteeringInitialize(false))
            {
                Debug.LogError("Unable to initialize Logitech wheel! Make sure the wheel is connected");
            }
            InitLogiSteeringSetting();
        }

        void OnApplicationQuit()
        {
            Debug.Log("SteeringShutdown:" + LogitechGSDK.LogiSteeringShutdown());
        }

        private void OnApplicationFocus(bool focus)
        {
            if (focus)
            {
                InitLogiSteeringSetting();
            }
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            if (!clockSync.isSync)
            {
                Debug.Log("Wait for clock sync!");
                return; // Wait for clock sync with ROS.
            }

            if (!LogitechGSDK.LogiUpdate() || !LogitechGSDK.LogiIsConnected(0))
            {
                Debug.Log("PLEASE PLUG IN A STEERING WHEEL");
            }

            LogitechGSDK.DIJOYSTATE2ENGINES rec;
            rec = LogitechGSDK.LogiGetStateUnity(0);
            message.pose.pose.position.x = GetThrottleState(rec);
            message.pose.pose.position.y = GetSteeringState(rec);
            Debug.Log("throttle: " + message.pose.pose.position.x + "; steering: " + message.pose.pose.position.y);

            // Header
            uint secs;
            uint nsecs;
            clockSync.Now(out secs, out nsecs);
            RosSharp.RosBridgeClient.MessageTypes.Std.Time time = new RosSharp.RosBridgeClient.MessageTypes.Std.Time(secs, nsecs);
            message.header = new Header(0, time, "unity");

            Publish(message);
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Nav.Odometry();
            message.pose.pose.position.x = 0;
            message.pose.pose.position.y = 0;
        }

        public double GetSteeringState(LogitechGSDK.DIJOYSTATE2ENGINES rec)
        {
            return map(rec.lX, maxSteeringTickLeft, maxSteeringTickRight, maxSteeringLeft, maxSteeringRight);
        }

        public double GetThrottleState(LogitechGSDK.DIJOYSTATE2ENGINES rec)
        {
            int throttleTick = rec.lY;
            int breakTick = rec.lRz;

            // Initialize the throttle to the right value on the program start.
            if (rec.lY == 0 && rec.lX == 0 && rec.lRz == 0)
            {
                return 0;
            }

            // Test if the breaking pedal is pressed and update the throttle value accordingly.
            if (breakTick < 32767 - 10)
            {
                return map(breakTick, 32767, -32767, 0, throttleReverseValue);
            }

            return map(throttleTick, 32767, -32767, 0, throttleForwardValue);
        }

        public double map(double value, int x1, int x2, int y1, int y2)
        {
            double tmp = (value - x1) * (y2 - y1) / (x2 - x1) + y1;
            double max = Mathf.Max(y1, y2);
            double min = Mathf.Min(y1, y2);
            if (tmp > max)
            {
                return max;
            }
            if (tmp < min)
            {
                return min;
            }
            return tmp;
        }
        
        void InitLogiSteeringSetting()
        {
            LogitechGSDK.LogiUpdate();
            LogitechGSDK.LogiPlaySpringForce(0, 0, 25, 50);
            LogitechGSDK.LogiPlaySoftstopForce(0, 22);
        }
    }
}