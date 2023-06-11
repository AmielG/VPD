using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using  bosqmode.libvlc;

namespace RosSharp.RosBridgeClient
{
    public class CarPredictor : UnitySubscriber<MessageTypes.Nav.Odometry>
    {
        public GameObject carPrefab;
        private ClockSync clockSync;
        private VLCPlayerMono vLCPlayerMono;
        public Transform startTm;
        private GameObject car;
        private Vector3 carPosition;
        private Quaternion carRotaion;
        private Vector3 carRotaionEulerAngles;
        public float distanceScale = 8.6f;
        private CustomDictionary<uint, CarState> stateHistory = new CustomDictionary<uint, CarState>();
        public int historySize = 500;
        private float measurementRate = 0.02f; // [s] (50 [Hz])
        protected override void Start()
        {
            base.Start();
            InitializeCar();
            
            vLCPlayerMono = GameObject.Find("360Camera").GetComponent<VLCPlayerMono>();
            GameObject RosConnector = GameObject.Find("RosConnector");
            clockSync = RosConnector.GetComponent<ClockSync>();        
            stateHistory.MaxItemsToHold = historySize;

            
        }

        void InitializeCar()
        {
            car = GameObject.Find("Donkey");
            carPosition = car.transform.position;
            carRotaion = car.transform.rotation;
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            car.transform.position = carPosition;
            car.transform.rotation = carRotaion;
        }

        protected override void ReceiveMessage(MessageTypes.Nav.Odometry message)
        {
            // Save the message in fixed size dictionary.
            CarState state = new CarState(message);
            stateHistory.Add(message.header.seq, state);

            Debug.Log("car_position.x: " + state.x + ", car_position.y: " + state.y);
            Debug.Log("car_velocity.x: " + state.longitudinalVelocity + ", yaw: " + state.yaw);

            float measurement_delay = Mathf.Abs((float)clockSync.diff(message.header.stamp));

            if (stateHistory.Count == historySize)
            {
                // Update only the rotation along y axis.
                carRotaionEulerAngles = carRotaion.eulerAngles;
                translate_dummy_car(out carPosition.x, out carPosition.z, out carRotaionEulerAngles.y, state, measurement_delay);
                carRotaion.eulerAngles = carRotaionEulerAngles;
            }
        }

        static public GameObject getChildGameObject(GameObject fromGameObject, string withName)
        {
            //Author: Isaac Dart, June-13.
            Transform[] ts = fromGameObject.transform.GetComponentsInChildren<Transform>(true);
            foreach (Transform t in ts) if (t.gameObject.name == withName) return t.gameObject;

            Debug.LogError("couldn't find: " + withName);
            return null;
        }


        public CarState reestimate_state(CarState latest_state, double delta_t)
        {
            CarState state = new CarState();
            state.x = (float)(latest_state.x + latest_state.longitudinalVelocity * Mathf.Cos(latest_state.yaw) * delta_t);
            state.y = (float)(latest_state.y + latest_state.longitudinalVelocity * Mathf.Sin(latest_state.yaw) * delta_t);
            state.yaw = latest_state.yaw;
            // return state;
            return latest_state;
        }

        public CarState get_previous_state(float measurement_delay, uint current_state_index)
        {
            uint state_index = (uint)((vLCPlayerMono.getFrameDelay() * 0.001 - measurement_delay) / measurementRate);
            Debug.Log("state_index: " + state_index);
            return stateHistory[current_state_index - state_index];
        }

        public void translate_dummy_car(out float x, out float y, out float yaw, CarState latest_state, float measurement_delay)
        {
            CarState previous_state = get_previous_state(measurement_delay, latest_state.seq);
            CarState reestimated_state = reestimate_state(latest_state, measurement_delay);

            CarState translation = reestimated_state - previous_state;
            x = translation.x * distanceScale;
            y = translation.y * distanceScale;
            yaw = translation.yaw - 90;
            Debug.Log("translation.x: " + x + ", translation.y: " + y + ", translation.yaw: " + yaw);
        }
    }
}   