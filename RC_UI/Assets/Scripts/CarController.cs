using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.UI;

public class CarController : MonoBehaviour
{
    public string ip = "127.0.0.1";

    public int socket_port = 60000;

    private Socket client;

    private float[] state;

    private Vector3 car_position;

    private Quaternion car_rotation;

    private Vector3 past_car_position;

    private Quaternion past_car_rotation;

    private Vector3 camera_position;

    private Quaternion camera_rotation;

    private int throttleForwardValue = 1;

    private int throttleReverseValue = -1;

    private int maxSteeringTickLeft = -7500;

    private int maxSteeringTickRight = 7500;

    private int maxSteeringLeft = -1;

    private int maxSteeringRight = 1;

    private Transform camera_view;

    private Dropdown mode_dropdown;

    private string rci_mode = null;

    // Start is called before the first frame update
    void Start()
    {
        if (!LogitechGSDK.LogiSteeringInitialize(false))
        {
            Debug
                .LogError("Unable to initialize Logitech wheel! Make sure the wheel is connected");
        }
        InitLogiSteeringSetting();

        GameObject tmp = GameObject.Find("OVRCameraRig");
        camera_view = tmp.transform;

        mode_dropdown =
            GameObject.Find("ModeDropdown").GetComponent<Dropdown>();
        rci_mode = mode_dropdown.options[mode_dropdown.value].text;
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

    void Update()
    {
        if (!LogitechGSDK.LogiUpdate() || !LogitechGSDK.LogiIsConnected(0))
        {
            Debug.Log("PLEASE PLUG IN A STEERING WHEEL");
        }

        float[] command = GetCommand();
        float[] state = SendAndReceive(command);
        UpdateCarPose (state);
        rci_mode = mode_dropdown.options[mode_dropdown.value].text;

        if (rci_mode == "PD")
        {
            // RCI mode PD = predictive display
            transform.position = car_position;
            transform.rotation = car_rotation;

            camera_position.y += (float) 1.57;
            camera_view.position = camera_position;
            camera_view.rotation = camera_rotation;

            // Show the car
            GameObject.Find("SkyCar").transform.localScale =
                new Vector3(1, 1, 1);
        }
        else if (rci_mode == "PVD")
        {
            // RCI mode PD = predictive virtual display
            car_position.y += (float) 1.57;
            camera_view.position = car_position;
            camera_view.rotation = car_rotation;

            // Hide the car
            GameObject.Find("SkyCar").transform.localScale =
                new Vector3(0, 0, 0);
        }
        else
        {
            // RCI mode WO = without mitigation method
            camera_position.y += (float) 1.57;
            camera_view.position = camera_position;
            camera_view.rotation = camera_rotation;

            // Hide the car
            GameObject.Find("SkyCar").transform.localScale =
                new Vector3(0, 0, 0);
        }
    }

    private float[] GetCommand()
    {
        LogitechGSDK.DIJOYSTATE2ENGINES rec;
        rec = LogitechGSDK.LogiGetStateUnity(0);

        float throttle = GetThrottleState(rec);
        float steering = GetSteeringState(rec);
        float[] command = { throttle, steering };
        return command;
    }

    private void UpdateCarPose(float[] state)
    {
        // Format the receive data to car pose
        car_rotation = new Quaternion(state[4], state[5], state[6], state[7]);
        car_rotation = ConvertRightHandedToLeftHandedQuaternion(car_rotation);
        car_position = new Vector3(state[1], state[2], state[3]);
        car_position = ConvertRightHandedToLeftHandedVector(car_position);

        // Format the receive data to camera pose
        camera_rotation =
            new Quaternion(state[12], state[13], state[14], state[15]);
        camera_rotation =
            ConvertRightHandedToLeftHandedQuaternion(camera_rotation);
        camera_position = new Vector3(state[9], state[10], state[11]);
        camera_position = ConvertRightHandedToLeftHandedVector(camera_position);

        // Format the receive data to past car pose
        past_car_rotation =
            new Quaternion(state[20], state[21], state[22], state[23]);
        past_car_rotation =
            ConvertRightHandedToLeftHandedQuaternion(past_car_rotation);
        past_car_position = new Vector3(state[17], state[18], state[19]);
        past_car_position =
            ConvertRightHandedToLeftHandedVector(past_car_position);
    }

    private float[] SendAndReceive(float[] dataOut)
    {
        //initialize socket
        float[] floatsReceived;
        client =
            new Socket(AddressFamily.InterNetwork,
                SocketType.Stream,
                ProtocolType.Tcp);
        client.Connect (ip, socket_port);
        if (!client.Connected)
        {
            Debug.LogError("Connection Failed");
            return null;
        }

        //convert floats to bytes, send to port
        var byteArray = new byte[dataOut.Length * 4];
        Buffer.BlockCopy(dataOut, 0, byteArray, 0, byteArray.Length);
        client.Send (byteArray);

        //allocate and receive bytes
        byte[] bytes = new byte[4000];
        int idxUsedBytes = client.Receive(bytes);

        //print(idxUsedBytes + " new bytes received.");
        //convert bytes to floats
        floatsReceived = new float[idxUsedBytes / 4];
        Buffer.BlockCopy(bytes, 0, floatsReceived, 0, idxUsedBytes);

        client.Close();
        return floatsReceived;
    }

    public float GetSteeringState(LogitechGSDK.DIJOYSTATE2ENGINES rec)
    {
        return map(rec.lX,
        maxSteeringTickLeft,
        maxSteeringTickRight,
        maxSteeringLeft,
        maxSteeringRight);
    }

    public float GetThrottleState(LogitechGSDK.DIJOYSTATE2ENGINES rec)
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
            return Mathf
                .Max(map(breakTick, 32767, -32767, 0, throttleReverseValue),
                -1);
        }

        return map(throttleTick, 32767, -32767, 0, throttleForwardValue);
    }

    public float map(float value, int x1, int x2, int y1, int y2)
    {
        float tmp = (value - x1) * (y2 - y1) / (x2 - x1) + y1;
        float max = Mathf.Max(y1, y2);
        float min = Mathf.Min(y1, y2);
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

    private Vector3
    ConvertRightHandedToLeftHandedVector(Vector3 rightHandedVector)
    {
        return new Vector3(rightHandedVector.y,
            -rightHandedVector.z,
            rightHandedVector.x);
    }

    private Quaternion
    ConvertRightHandedToLeftHandedQuaternion(Quaternion rightHandedQuaternion)
    {
        Vector3 leftHandedEulerAngles =
            new Vector3(rightHandedQuaternion.eulerAngles.y,
                -rightHandedQuaternion.eulerAngles.z,
                rightHandedQuaternion.eulerAngles.x);
        return Quaternion.Euler(leftHandedEulerAngles);
    }

    void InitLogiSteeringSetting()
    {
        LogitechGSDK.LogiUpdate();
        LogitechGSDK.LogiPlaySpringForce(0, 0, 25, 50);
        LogitechGSDK.LogiPlaySoftstopForce(0, 22);
    }
}
