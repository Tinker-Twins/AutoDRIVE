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

    public bool WeatherAPI = false;
    public WeatherManager[] Weather; // `WeatherManager` reference
    private int weather = 1;
    public bool TimeOfDayAPI = false;
    public TimeOfDay[] TimeOfDay; // `TimeOfDay` reference

    public CoSimManager[] CoSimManagers; // Vehicle gameobjects
    public Rigidbody[] VehicleRigidBodies; // Vehicle rigid bodies
    public VehicleController[] VehicleControllers; // `VehicleController` references
    public AutomobileController[] AutomobileControllers; // `AutomobileController` references
    public VehicleLighting[] VehicleLightings; // `VehicleLighting` references
    public CarLighting[] CarLightings; // `CarLighting` references
    public ROVLighting[] ROVLightings; // `ROVLighting` references
    public WheelEncoder[] LeftWheelEncoders; // `WheelEncoder` references for left wheel
    public WheelEncoder[] RightWheelEncoders; // `WheelEncoder` references for right wheel
    public GPS[] PositioningSystems; // `IPS` references
    public IMU[] InertialMeasurementUnits; // `IMU` references
    public LIDAR[] LIDARUnits; // `LIDAR` references
    public LIDAR3D[] LIDAR3DUnits; // `LIDAR3D` references
    private string LIDARRangeArray;
    private string LIDARIntensityArray;
    public Camera[] FrontCameras; // Vehicle front camera references
    public Camera[] RearCameras; // Vehicle rear camera references
    public bool SideCameras = false; // Rename front/rear camera frames as left/right

    public bool CollisionDetection = false;

    public TLController[] TrafficLightControllers; // Traffic light controller references

    private Vector3 CoSimPosition;
    private Quaternion CoSimRotation;

    // Use this for initialization
    void Start()
    {
        socket = GameObject.Find("SocketIO").GetComponent<SocketIOComponent>(); // Cache `SocketIOComponent`
        socket.On("connect", OnConnect); // Declare connection event `connect` and corresponding event handler `OnConnect`
        socket.On("Bridge", OnBridge); // Declare event `Bridge` and corresponding event handler `OnBridge`
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

        // Set time of day
        if(TimeOfDayAPI && (TimeOfDay.Length !=0))
        {
            TimeOfDay[0].automaticUpdate = (jsonObject.GetField("Auto Time").str == "True"); // Set automatic update
            TimeOfDay[0].timeScale = float.Parse(jsonObject.GetField("Time Scale").str); // Set time scale
            TimeOfDay[0].timeOfDay = float.Parse(jsonObject.GetField("Time").str); // Set time of day
        }

        // Set weather
        if(WeatherAPI && (Weather.Length !=0))
        {
            weather = int.Parse(jsonObject.GetField("Weather").str); // Set weather
            if(weather == 0) Weather[0].weatherPreset = WeatherManager.WeatherPreset.Custom;
            else if(weather == 1) Weather[0].weatherPreset = WeatherManager.WeatherPreset.Sunny;
            else if(weather == 2) Weather[0].weatherPreset = WeatherManager.WeatherPreset.Cloudy;
            else if(weather == 3) Weather[0].weatherPreset = WeatherManager.WeatherPreset.LightFog;
            else if(weather == 4) Weather[0].weatherPreset = WeatherManager.WeatherPreset.HeavyFog;
            else if(weather == 5) Weather[0].weatherPreset = WeatherManager.WeatherPreset.LightRain;
            else if(weather == 6) Weather[0].weatherPreset = WeatherManager.WeatherPreset.HeavyRain;
            else if(weather == 7) Weather[0].weatherPreset = WeatherManager.WeatherPreset.LightSnow;
            else if(weather == 8) Weather[0].weatherPreset = WeatherManager.WeatherPreset.HeavySnow;
            Weather[0].CloudIntensity = float.Parse(jsonObject.GetField("Clouds").str); // Set cloud intensity
            Weather[0].FogIntensity = float.Parse(jsonObject.GetField("Fog").str); // Set fog intensity
            Weather[0].RainIntensity = float.Parse(jsonObject.GetField("Rain").str); // Set rain intensity
            Weather[0].SnowIntensity = float.Parse(jsonObject.GetField("Snow").str); // Set snow intensity
        }

        // Write data to vehicles
        if(VehicleControllers.Length != 0)
        {
            for(int i=0;i<VehicleControllers.Length;i++)
            {
                if(VehicleControllers[i].CurrentDrivingMode == 1)
                {
                    if(CoSimManagers.Length != 0)
                    {
                        if(int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" CoSim").str) == 1)
                        {
                            VehicleRigidBodies[i].isKinematic = true;
                            CoSimPosition.x = - float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosY").str); // Set position X-component
                            CoSimPosition.y = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosZ").str); // Set position Y-component
                            CoSimPosition.z = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosX").str); // Set position Z-component
                            CoSimRotation.x = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotY").str); // Set rotation X-component
                            CoSimRotation.y = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotZ").str); // Set rotation Y-component
                            CoSimRotation.z = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotX").str); // Set rotation Z-component
                            CoSimRotation.w = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotW").str); // Set rotation W-component
                            CoSimManagers[i].CoSimTimer = 0.0f;
                            CoSimManagers[i].CoSimPosition = CoSimPosition;
                            CoSimManagers[i].CoSimRotation = CoSimRotation;
                            CoSimManagers[i].enabled = true;
                        }
                        else
                        {
                            CoSimManagers[i].enabled = false;
                            VehicleRigidBodies[i].isKinematic = false;
                            VehicleControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                            // Debug.Log("Throttle: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str));
                            VehicleControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                            // Debug.Log("Steering: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str));
                        }
                    }
                    else
                    {
                        VehicleRigidBodies[i].isKinematic = false;
                        VehicleControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                        // Debug.Log("Throttle: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str));
                        VehicleControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                        // Debug.Log("Steering: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str));
                    }
                    if(VehicleLightings.Length != 0)
                    {
                        VehicleLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                        VehicleLightings[i].Indicators = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Indicators").str); // Set indicators
                    }
                }
            }
        }
        if(AutomobileControllers.Length != 0)
        {
            for(int i=0;i<AutomobileControllers.Length;i++)
            {
                if(AutomobileControllers[i].CurrentDrivingMode == 1)
                {
                    if(CoSimManagers.Length != 0)
                    {
                        if(int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" CoSim").str) == 1)
                        {
                            VehicleRigidBodies[i].isKinematic = true;
                            CoSimPosition.x = - float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosY").str); // Set position X-component
                            CoSimPosition.y = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosZ").str); // Set position Y-component
                            CoSimPosition.z = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" PosX").str); // Set position Z-component
                            CoSimRotation.x = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotY").str); // Set rotation X-component
                            CoSimRotation.y = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotZ").str); // Set rotation Y-component
                            CoSimRotation.z = -float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotX").str); // Set rotation Z-component
                            CoSimRotation.w = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" RotW").str); // Set rotation W-component
                            CoSimManagers[i].CoSimTimer = 0.0f;
                            CoSimManagers[i].CoSimPosition = CoSimPosition;
                            CoSimManagers[i].CoSimRotation = CoSimRotation;
                            CoSimManagers[i].enabled = true;
                        }
                        else
                        {
                            CoSimManagers[i].enabled = false;
                            VehicleRigidBodies[i].isKinematic = false;
                            AutomobileControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                            // Debug.Log("Throttle: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str));
                            AutomobileControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                            // Debug.Log("Steering: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str));
                            AutomobileControllers[i].CurrentBrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Brake").str); // Set brake
                            // Debug.Log("Brake: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Brake").str));
                            AutomobileControllers[i].CurrentHandbrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Handbrake").str); // Set handbrake
                            // Debug.Log("Handbrake: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Handbrake").str));
                        }
                    }
                    else
                    {
                        VehicleRigidBodies[i].isKinematic = false;
                        AutomobileControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                        // Debug.Log("Throttle: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str));
                        AutomobileControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                        // Debug.Log("Steering: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str));
                        AutomobileControllers[i].CurrentBrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Brake").str); // Set brake
                        // Debug.Log("Brake: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Brake").str));
                        AutomobileControllers[i].CurrentHandbrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Handbrake").str); // Set handbrake
                        // Debug.Log("Handbrake: " + float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Handbrake").str));
                    }
                    if(CarLightings.Length != 0)
                    {
                        CarLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                        CarLightings[i].Indicators = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Indicators").str); // Set indicators
                    }
                    if(ROVLightings.Length != 0)
                    {
                        ROVLightings[i].Headlights = int.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Headlights").str); // Set headlights
                    }
                }
            }
        }

        // Write data to traffic lights
        if(TrafficLightControllers.Length != 0)
        {
            for(int i=0;i<TrafficLightControllers.Length;i++)
            {
                TrafficLightControllers[i].CurrentState = int.Parse(jsonObject.GetField("TL"+(i+1).ToString()+" State").str); // Set traffic light
            }
        }

        // Emit telemetry data
        EmitTelemetry(obj);
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
            if(VehicleControllers.Length != 0 || AutomobileControllers.Length != 0)
            {
                for(int i=0;i<VehicleControllers.Length;i++)
                {
                    data["V"+(i+1).ToString()+" Throttle"] = VehicleControllers[i].CurrentThrottle.ToString("F3"); // Get throttle
                    data["V"+(i+1).ToString()+" Steering"] = VehicleControllers[i].CurrentSteeringAngle.ToString("F3"); // Get steering angle
                }
                for(int i=0;i<AutomobileControllers.Length;i++)
                {
                    data["V"+(i+1).ToString()+" Throttle"] = AutomobileControllers[i].CurrentThrottle.ToString("F3"); // Get throttle
                    data["V"+(i+1).ToString()+" Steering"] = AutomobileControllers[i].CurrentSteeringAngle.ToString("F3"); // Get steering angle
                    data["V"+(i+1).ToString()+" Brake"] = AutomobileControllers[i].CurrentBrake.ToString("F3"); // Get brake
                    data["V"+(i+1).ToString()+" Handbrake"] = AutomobileControllers[i].CurrentHandbrake.ToString("F3"); // Get handbrake
                    data["V"+(i+1).ToString()+" Collisions"] = AutomobileControllers[i].collisionCount.ToString(); // Get collision count
                }
                for(int i=0;i<VehicleControllers.Length+AutomobileControllers.Length;i++) // Assumed that VehicleControllers.Length+AutomobileControllers.Length >= others
                {    
                    data["V"+(i+1).ToString()+" Encoder Ticks"] = LeftWheelEncoders[i].Ticks.ToString() + " " + RightWheelEncoders[i].Ticks.ToString(); // Get encoder ticks
                    data["V"+(i+1).ToString()+" Encoder Angles"] = LeftWheelEncoders[i].Angle.ToString("F3") + " " + RightWheelEncoders[i].Angle.ToString("F3"); // Get encoder angles
                    data["V"+(i+1).ToString()+" Position"] = PositioningSystems[i].CurrentPosition[0].ToString("F3") + " " + PositioningSystems[i].CurrentPosition[1].ToString("F3") + " " + PositioningSystems[i].CurrentPosition[2].ToString("F3"); // Get vehicle position
                    data["V"+(i+1).ToString()+" Orientation Quaternion"] = InertialMeasurementUnits[i].CurrentOrientationQuaternion[0].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[1].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[2].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[3].ToString("F3"); // Get vehicle orientation (Quaternion)
                    data["V"+(i+1).ToString()+" Orientation Euler Angles"] = InertialMeasurementUnits[i].CurrentOrientationEulerAngles[0].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[1].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[2].ToString("F3"); // Get vehicle orientation (Euler Angles)
                    data["V"+(i+1).ToString()+" Angular Velocity"] = InertialMeasurementUnits[i].CurrentAngularVelocity[0].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[1].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[2].ToString("F3"); // Get angular velocity of the vehicle
                    data["V"+(i+1).ToString()+" Linear Acceleration"] = InertialMeasurementUnits[i].CurrentLinearAcceleration[0].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[1].ToString("F3") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[2].ToString("F3"); // Get linear acceleration of the vehicle
                    if(LIDARUnits.Length != 0)
                    {
                        data["V"+(i+1).ToString()+" LIDAR Scan Rate"] = LIDARUnits[i].CurrentScanRate.ToString("F3"); // Get LIDAR scan rate
                        if(LIDARUnits[i].CurrentRangeArray[LIDARUnits[i].CurrentRangeArray.Length-1] != null)
                        {
                            // for(int j=0;j<LIDARUnits[i].CurrentRangeArray.Length-1;j++)
                            // {
                            //     LIDARRangeArray += LIDARUnits[i].CurrentRangeArray[j] + " ";
                            //     LIDARIntensityArray += LIDARUnits[i].CurrentIntensityArray[j] + " ";
                            // }
                            // LIDARRangeArray += LIDARUnits[i].CurrentRangeArray[LIDARUnits[i].CurrentRangeArray.Length-1];
                            // LIDARIntensityArray += LIDARUnits[i].CurrentIntensityArray[LIDARUnits[i].CurrentRangeArray.Length-1];

                            LIDARRangeArray = string.Join(" ", LIDARUnits[i].CurrentRangeArray);
                            LIDARIntensityArray = string.Join(" ", LIDARUnits[i].CurrentIntensityArray);
                        }
                        data["V"+(i+1).ToString()+" LIDAR Range Array"] = LIDARRangeArray; // Get LIDAR range array
                        data["V"+(i+1).ToString()+" LIDAR Intensity Array"] = LIDARIntensityArray; // Get LIDAR intensity array

                        LIDARRangeArray = ""; // Reset LIDAR range array for next measurement
                        LIDARIntensityArray = ""; // Reset LIDAR intensity array for next measurement
                    }
                    if(LIDAR3DUnits.Length != 0) data["V"+(i+1).ToString()+" LIDAR Pointcloud"] = Convert.ToBase64String(LIDAR3DUnits[i].CurrentPointcloud); // Get LIDAR pointcloud
                    if(FrontCameras.Length != 0)
                    {
                        if(SideCameras) data["V"+(i+1).ToString()+" Left Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(FrontCameras[i])); // Get left camera image
                        else data["V"+(i+1).ToString()+" Front Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(FrontCameras[i])); // Get front camera image
                    }
                    if(RearCameras.Length != 0)
                    {
                        if(SideCameras) data["V"+(i+1).ToString()+" Right Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(RearCameras[i])); // Get right camera image
                        else data["V"+(i+1).ToString()+" Rear Camera Image"] = Convert.ToBase64String(FrameGrabber.CaptureFrame(RearCameras[i])); // Get rear camera image
                    }
                }
            }
            socket.Emit("Bridge", new JSONObject(data)); // Write data to server
        });
    }
}
