using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using SocketIO;

public class Socket : MonoBehaviour
{
    /*
    This script establishes a bi-directional connection between the simulator
    application and an external interface.
    */

    private SocketIOComponent socket; // Socket.IO instance
    public Button ConnectionButton; // GUI button
    public Text ConnectionLabel; // GUI button label

    public VehicleController VehicleController; // `VehicleController` reference
    public VehicleLighting VehicleLighting; // `VehicleLighting` reference
    public WheelEncoder LeftWheelEncoder; // `WheelEncoder` reference for left wheel
    public WheelEncoder RightWheelEncoder; // `WheelEncoder` reference for right wheel
    public IPS IPS; // `IPS` reference
    public IMU IMU; // `IMU` reference
    public LIDAR LIDAR; // `LIDAR` reference
    public Camera FrontCamera; // Vehicle front camera
    private string LIDARRangeArray;
    private string LIDARIntensityArray;
    public Camera RearCamera; // Vehicle rear camera

    public TLController TL1; // Traffic light controller
    public TLController TL2; // Traffic light controller
    public TLController TL3; // Traffic light controller
    public TLController TL4; // Traffic light controller

    // Use this for initialization
    void Start()
    {
        socket = GameObject.Find("SocketIO").GetComponent<SocketIOComponent>(); // Cache `SocketIOComponent`
        socket.On("connect", OnConnect); // Declare connection event `connect` and corresponding event handler `OnConnect`
        socket.On("Bridge", OnBridge); // Declare event `AutonomousMode` and corresponding event handler `OnAutonomousMode`
        socket.On("disconnect", OnDisconnect); // Declare disconnection event `disconnect` and corresponding event handler `OnDisconnect`
    }

    void OnConnect(SocketIOEvent obj)
    {
        //Debug.Log("Connected!");
        ConnectionButton.interactable = false;
        ConnectionLabel.text = "Connected";
        EmitTelemetry(obj);
    }

    void OnDisconnect(SocketIOEvent obj)
    {
        //Debug.Log("Disconnected!");
        ConnectionButton.interactable = true;
        ConnectionLabel.text = "Disconnected";
        EmitTelemetry(obj);
    }

    void OnBridge(SocketIOEvent obj)
    {
        //Debug.Log("Bridge");
        JSONObject jsonObject = obj.data; // Read incoming data and store it in a `JSONObject`
        //Debug.Log(obj.data);

        TL1.CurrentState = int.Parse(jsonObject.GetField("Traffic Light 1").str); // Set traffic light
        TL2.CurrentState = int.Parse(jsonObject.GetField("Traffic Light 2").str); // Set traffic light
        TL3.CurrentState = int.Parse(jsonObject.GetField("Traffic Light 3").str); // Set traffic light
        TL4.CurrentState = int.Parse(jsonObject.GetField("Traffic Light 4").str); // Set traffic light

        if(VehicleController.CurrentDrivingMode == 1)
        {
            VehicleController.CurrentThrottle = float.Parse(jsonObject.GetField("Throttle").str); // Set throttle
            VehicleController.CurrentSteeringAngle = float.Parse(jsonObject.GetField("Steering").str); // Set steering angle
            VehicleLighting.Headlights = int.Parse(jsonObject.GetField("Headlights").str); // Set headlights
            VehicleLighting.Indicators = int.Parse(jsonObject.GetField("Indicators").str); // Set indicators
        }
        EmitTelemetry(obj); // Emit telemetry data
    }

    void EmitTelemetry(SocketIOEvent obj)
    {
        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            //Debug.Log("Attempting to write data...");
            Dictionary<string, string> data = new Dictionary<string, string>(); // Create new `data` dictionary
            data["Throttle"] = VehicleController.CurrentThrottle.ToString("F3"); // Get throttle
            data["Steering"] = VehicleController.CurrentSteeringAngle.ToString("F3"); // Get steering angle
            data["Encoder Ticks"] = LeftWheelEncoder.Ticks.ToString() + " " + RightWheelEncoder.Ticks.ToString(); // Get encoder ticks
            data["Encoder Angles"] = LeftWheelEncoder.Angle.ToString("F3") + " " + RightWheelEncoder.Angle.ToString("F3"); // Get encoder angles
            data["Position"] = IPS.CurrentPosition[0].ToString("F3") + " " + IPS.CurrentPosition[1].ToString("F3") + " " + IPS.CurrentPosition[2].ToString("F3"); // Get vehicle position
            data["Orientation Quaternion"] = IMU.CurrentOrientationQuaternion[0].ToString("F3") + " " + IMU.CurrentOrientationQuaternion[1].ToString("F3") + " " + IMU.CurrentOrientationQuaternion[2].ToString("F3") + " " + IMU.CurrentOrientationQuaternion[3].ToString("F3"); // Get vehicle orientation (Quaternion)
            data["Orientation Euler Angles"] = IMU.CurrentOrientationEulerAngles[0].ToString("F3") + " " + IMU.CurrentOrientationEulerAngles[1].ToString("F3") + " " + IMU.CurrentOrientationEulerAngles[2].ToString("F3"); // Get vehicle orientation (Euler Angles)
            data["Angular Velocity"] = IMU.CurrentAngularVelocity[0].ToString("F3") + " " + IMU.CurrentAngularVelocity[1].ToString("F3") + " " + IMU.CurrentAngularVelocity[2].ToString("F3"); // Get angular velocity of the vehicle
            data["Linear Acceleration"] = IMU.CurrentLinearAcceleration[0].ToString("F3") + " " + IMU.CurrentLinearAcceleration[1].ToString("F3") + " " + IMU.CurrentLinearAcceleration[2].ToString("F3"); // Get linear acceleration of the vehicle
            data["LIDAR Scan Rate"] = LIDAR.CurrentScanRate.ToString("F3"); // Get LIDAR scan rate
            if(LIDAR.CurrentRangeArray[359] != null)
            {
                for(int i=0;i<359;i++)
                {
                      LIDARRangeArray += LIDAR.CurrentRangeArray[i] + " ";
                      LIDARIntensityArray += LIDAR.CurrentIntensityArray[i] + " ";
                }
                LIDARRangeArray += LIDAR.CurrentRangeArray[359];
                LIDARIntensityArray += LIDAR.CurrentIntensityArray[359];
            }
            data["LIDAR Range Array"] = LIDARRangeArray; // Get LIDAR range array
            data["LIDAR Intensity Array"] = LIDARIntensityArray; // Get LIDAR intensity array
            LIDARRangeArray = ""; // Reset LIDAR range array for next measurement
            LIDARIntensityArray = ""; // Reset LIDAR intensity array for next measurement
            data["Front Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(FrontCamera)); // Get front camera image
            data["Rear Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(RearCamera)); // Get rear camera image
            socket.Emit("Bridge", new JSONObject(data)); // Write data to server
        });
    }
}
