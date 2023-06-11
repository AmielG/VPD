using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient
{
    public class VCarHandler : UnitySubscriber<MessageTypes.Nav.Odometry>
    {
        public GameObject carPrefab;
        private ClockSync clockSync;
        public Transform startTm;
        private GameObject car;
        private Vector3 carPosition;
        private Quaternion carRotaion;
        private Vector3 carRotaionEulerAngles;
        private float distanceScale = 8.6f;
        protected override void Start()
        {
            base.Start();
            InitializeCar();
            GameObject RosConnector = GameObject.Find("RosConnector");
            clockSync = RosConnector.GetComponent<ClockSync>();
        }

        void InitializeCar()
        {
            car = GameObject.Find("Donkey");
            carPosition = car.transform.position;
            carRotaion = car.transform.rotation;

            Camera cam = Camera.main;
            CameraFollow cameraFollow = null;
            if (cam != null)
                cameraFollow = cam.transform.GetComponent<CameraFollow>();

            //set camera target follow tm
            if (cameraFollow != null)
                cameraFollow.target = getChildGameObject(car, "CameraFollowTm").transform;
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            car.transform.position = carPosition;
            car.transform.rotation = carRotaion;
        }

        protected override void ReceiveMessage(MessageTypes.Nav.Odometry message)
        {
            float diff = Mathf.Abs((float)clockSync.diff(message.header.stamp));
            Debug.Log("diff state: " + diff);
            
            float yaw = -(float)message.pose.pose.orientation.z;
            // carPosition.x = (float)message.pose.pose.position.y * distanceScale;
            // carPosition.z = (float)message.pose.pose.position.x * distanceScale;
            reestimate_state(out carPosition.z, out carPosition.x, yaw, carPosition, message, diff);

            // Update only the rotation along y axis.
            // Quaternion orientation = new Quaternion((float)message.pose.pose.orientation.x, (float)message.pose.pose.orientation.y,
            //  (float)message.pose.pose.orientation.z, (float)message.pose.pose.orientation.w);
            carRotaionEulerAngles = carRotaion.eulerAngles;
            carRotaionEulerAngles.y = (float)yaw * Mathf.Rad2Deg; // The orientaion is Eulat angles.
            carRotaion.eulerAngles = carRotaionEulerAngles;
            Debug.Log("car state: " + carRotaionEulerAngles);
        }

        static public GameObject getChildGameObject(GameObject fromGameObject, string withName)
        {
            //Author: Isaac Dart, June-13.
            Transform[] ts = fromGameObject.transform.GetComponentsInChildren<Transform>(true);
            foreach (Transform t in ts) if (t.gameObject.name == withName) return t.gameObject;

            Debug.LogError("couldn't find: " + withName);
            return null;
        }


        public void reestimate_state(out float x, out float y, float yaw,  Vector3 carPosition, MessageTypes.Nav.Odometry message, double delta_t)
        {
            
            x = (float)(message.pose.pose.position.x + message.twist.twist.linear.x * Mathf.Cos(yaw) * delta_t) * distanceScale;
            y = (float)(message.pose.pose.position.y + message.twist.twist.linear.x * Mathf.Sin(yaw) * delta_t) * distanceScale;
        }
    }
}