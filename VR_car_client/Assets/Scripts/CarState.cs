using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient
{
    public class CarState
    {
        public uint seq;
        public float x;
        public float y;
        public float yaw;
        public float longitudinalVelocity;

        public CarState()
        { }

        public CarState(MessageTypes.Nav.Odometry message)
        {
            seq = message.header.seq;
            x = (float)message.pose.pose.position.x;
            y = (float)message.pose.pose.position.y;
            yaw = (float)message.pose.pose.orientation.z * Mathf.Rad2Deg;
            longitudinalVelocity = (float)message.twist.twist.linear.x;
        }

        public static CarState operator -(CarState x1, CarState x2)
        {
            CarState translation = new CarState();
            translation.x = x2.x - x1.x;
            translation.y = x2.y - x1.y;
            translation.yaw = x2.yaw - x1.yaw;
            return translation;
        }
    }
}