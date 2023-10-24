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

    public VehicleController[] VehicleControllers; // `VehicleController` references
    public VehicleLighting[] VehicleLightings; // `VehicleLighting` references
    public WheelEncoder[] LeftWheelEncoders; // `WheelEncoder` references for left wheel
    public WheelEncoder[] RightWheelEncoders; // `WheelEncoder` references for right wheel
    public IPS[] IndoorPositioningSystems; // `IPS` references
    public IMU[] InertialMeasurementUnits; // `IMU` references
    public LIDAR[] LIDARUnits; // `LIDAR` references
    private string LIDARRangeArray;
    private string LIDARIntensityArray;
    public Camera[] FrontCameras; // Vehicle front camera references
    public Camera[] RearCameras; // Vehicle rear camera references

    public TLController[] TrafficLightControllers; // Traffic light controller references

    // Use this for initialization
    void Start()
    {
        socket = GameObject.Find("SocketIO").GetComponent<SocketIOComponent>(); // Cache `SocketIOComponent`
        socket.On("connect", OnConnect); // Declare connection event `connect` and corresponding event handler `OnConnect`
        socket.On("Bridge", OnBridge); // Declare event `AutonomousMode` and corresponding event handler `OnAutonomousMode`
        socket.On("disconnect", OnDisconnect); // Declare disconnection event `disconnect` and corresponding event handler `OnDisconnect`
        // Temporary render textures for all vehicles except the first one (first vehicle will render to GUI)
        if(FrontCameras.Length != 0)
        {
            for(int i=1;i<FrontCameras.Length;i++)
            {
                FrontCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
        if(RearCameras.Length != 0)
        {
            for(int i=1;i<RearCameras.Length;i++)
            {
                RearCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
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
        // Write data to traffic lights
        if(TrafficLightControllers.Length != 0)
        {
            for(int i=0;i<TrafficLightControllers.Length;i++)
            {
                TrafficLightControllers[i].CurrentState = int.Parse(jsonObject.GetField("TL"+(i+1).ToString()+" State").str); // Set traffic light
            }
        }
        // Write data to vehicles
        if(VehicleControllers.Length != 0)
        {
            for(int i=0;i<VehicleControllers.Length;i++)
            {
              if(VehicleControllers[i].CurrentDrivingMode == 1)
              {
                    VehicleControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                    VehicleControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                    if(VehicleLightings.Length != 0)
                    {
                        VehicleLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                        VehicleLightings[i].Indicators = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Indicators").str); // Set indicators
                    }
              }
            }
        }
        EmitTelemetry(obj); // Emit telemetry data
    }

    void EmitTelemetry(SocketIOEvent obj)
    {
        UnityMainThreadDispatcher.Instance().Enqueue(() =>
        {
            //Debug.Log("Attempting to write data...");
            Dictionary<string, string> data = new Dictionary<string, string>(); // Create new `data` dictionary
            // Read data from traffic lights
            if(TrafficLightControllers.Length != 0)
            {
                for(int i=0;i<TrafficLightControllers.Length;i++)
                {
                    data["TL"+(i+1).ToString()+" State"] = TrafficLightControllers[i].CurrentState.ToString(); // Get status
                }
            }
            // Read data from vehicles
            if(VehicleControllers.Length != 0)
            {
                for(int i=0;i<VehicleControllers.Length;i++) // Assumed that VehicleControllers.Length >= others
                {
                    data["V"+(i+1).ToString()+" Throttle"] = VehicleControllers[i].CurrentThrottle.ToString("F3"); // Get throttle
                    data["V"+(i+1).ToString()+" Steering"] = VehicleControllers[i].CurrentSteeringAngle.ToString("F3"); // Get steering angle
                    data["V"+(i+1).ToString()+" Encoder Ticks"] = LeftWheelEncoders[i].Ticks.ToString() + " " + RightWheelEncoders[i].Ticks.ToString(); // Get encoder ticks
                    data["V"+(i+1).ToString()+" Encoder Angles"] = LeftWheelEncoders[i].Angle.ToString("F3") + " " + RightWheelEncoders[i].Angle.ToString("F3"); // Get encoder angles
                    data["V"+(i+1).ToString()+" Position"] = IndoorPositioningSystems[i].CurrentPosition[0].ToString("F3") + " " + IndoorPositioningSystems[i].CurrentPosition[1].ToString("F3") + " " + IndoorPositioningSystems[i].CurrentPosition[2].ToString("F3"); // Get vehicle position
                    data["V"+(i+1).ToString()+" Orientation Quaternion"] = InertialMeasurementUnits[i].CurrentOrientationQuaternion[0].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[1].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[2].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[3].ToString("F3"); // Get vehicle orientation (Quaternion)
                    data["V"+(i+1).ToString()+" Orientation Euler Angles"] = InertialMeasurementUnits[i].CurrentOrientationEulerAngles[0].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[1].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[2].ToString("F3"); // Get vehicle orientation (Euler Angles)
                    data["V"+(i+1).ToString()+" Angular Velocity"] = InertialMeasurementUnits[i].CurrentAngularVelocity[0].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[1].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[2].ToString("F3"); // Get angular velocity of the vehicle
                    data["V"+(i+1).ToString()+" Linear Acceleration"] = InertialMeasurementUnits[i].CurrentLinearAcceleration[0].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[1].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[2].ToString("F3"); // Get linear acceleration of the vehicle
                    data["V"+(i+1).ToString()+" LIDAR Scan Rate"] = LIDARUnits[i].CurrentScanRate.ToString("F3"); // Get LIDAR scan rate
                    if(LIDARUnits[i].CurrentRangeArray[LIDARUnits[i].CurrentRangeArray.Length-1] != null)
                    {
                        for(int j=0;j<LIDARUnits[i].CurrentRangeArray.Length-1;j++)
                        {
                            LIDARRangeArray += LIDARUnits[i].CurrentRangeArray[j] + " ";
                            LIDARIntensityArray += LIDARUnits[i].CurrentIntensityArray[j] + " ";
                        }
                        LIDARRangeArray += LIDARUnits[i].CurrentRangeArray[LIDARUnits[i].CurrentRangeArray.Length-1];
                        LIDARIntensityArray += LIDARUnits[i].CurrentIntensityArray[LIDARUnits[i].CurrentRangeArray.Length-1];
                    }
                    data["V"+(i+1).ToString()+" LIDAR Range Array"] = LIDARRangeArray; // Get LIDAR range array
                    data["V"+(i+1).ToString()+" LIDAR Intensity Array"] = LIDARIntensityArray; // Get LIDAR intensity array
                    LIDARRangeArray = ""; // Reset LIDAR range array for next measurement
                    LIDARIntensityArray = ""; // Reset LIDAR intensity array for next measurement
                    if(FrontCameras.Length != 0) data["V"+(i+1).ToString()+" Front Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(FrontCameras[i])); // Get front camera image

                    if(RearCameras.Length != 0) data["V"+(i+1).ToString()+" Rear Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(RearCameras[i])); // Get rear camera image
                }
            }
            socket.Emit("Bridge", new JSONObject(data)); // Write data to server
        });
    }
}
