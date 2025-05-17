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

    public CoSimManager[] CoSimManagers; // `CoSimManager` references
    public ResetManager[] ResetManagers; // `ResetManager` references
    public Rigidbody[] VehicleRigidBodies; // Vehicle rigid bodies
    public VehicleController[] VehicleControllers; // `VehicleController` references
    public AutomobileController[] AutomobileControllers; // `AutomobileController` references
    public TwistController[] TwistControllers; // `TwistController` references
    public VehicleLighting[] VehicleLightings; // `VehicleLighting` references
    public CarLighting[] CarLightings; // `CarLighting` references
    public ROVLighting[] ROVLightings; // `ROVLighting` references
    public WheelEncoder[] LeftWheelEncoders; // `WheelEncoder` references for left wheel
    public WheelEncoder[] RightWheelEncoders; // `WheelEncoder` references for right wheel
    public GPS[] PositioningSystems; // `IPS` references
    public IMU[] InertialMeasurementUnits; // `IMU` references
    public LIDAR[] LIDARUnits; // `LIDAR` references
    public bool IntensityArray = true; // Choose to publish 2D LIDAR Intensity Array
    public LIDAR3D[] LIDAR3DUnits; // `LIDAR3D` references
    public Camera[] FrontCameras; // Vehicle front camera references
    public Camera[] RearCameras; // Vehicle rear camera references
    public bool SideCameras = false; // Rename front/rear camera frames as left/right
    public LapTimer[] LapTimers; // Lap timer references

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
                    if (ResetManagers.Length != 0)
                    {
                        ResetManagers[i].ResetFlag = bool.Parse(jsonObject.GetField("Reset").str); // Set reset flag
                    }
                    if (CoSimManagers.Length != 0)
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
                            if (TwistControllers.Length != 0)
                            {
                                TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Linear Velocity").str); // Set linear velocity
                                TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Angular Velocity").str); // Set angular velocity
                            }
                            else
                            {
                                VehicleControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                                VehicleControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                            }
                        }
                    }
                    else
                    {
                        VehicleRigidBodies[i].isKinematic = false;
                        if (TwistControllers.Length != 0)
                        {
                            TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Linear Velocity").str); // Set linear velocity
                            TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Angular Velocity").str); // Set angular velocity
                        }
                        else
                        {
                            VehicleControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                            VehicleControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                        }
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
                    if (ResetManagers.Length != 0)
                    {
                        ResetManagers[i].ResetFlag = bool.Parse(jsonObject.GetField("Reset").str); // Set reset flag
                    }
                    if (CoSimManagers.Length != 0)
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
                            if (TwistControllers.Length != 0)
                            {
                                TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Linear Velocity").str); // Set linear velocity
                                TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Angular Velocity").str); // Set angular velocity
                            }
                            else
                            {
                                AutomobileControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                                AutomobileControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                                AutomobileControllers[i].CurrentBrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Brake").str); // Set brake
                                AutomobileControllers[i].CurrentHandbrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Handbrake").str); // Set handbrake
                            }
                        }
                    }
                    else
                    {
                        VehicleRigidBodies[i].isKinematic = false;
                        if (TwistControllers.Length != 0)
                        {
                            TwistControllers[i].vSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Linear Velocity").str); // Set linear velocity
                            TwistControllers[i].wSetpoint = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Angular Velocity").str); // Set angular velocity
                        }
                        else
                        {
                            AutomobileControllers[i].CurrentThrottle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Throttle").str); // Set throttle
                            AutomobileControllers[i].CurrentSteeringAngle = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Steering").str); // Set steering angle
                            AutomobileControllers[i].CurrentBrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Brake").str); // Set brake
                            AutomobileControllers[i].CurrentHandbrake = float.Parse(jsonObject.GetField("V"+(i+1).ToString()+" Handbrake").str); // Set handbrake
                        }
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
                for (int i=0;i<VehicleControllers.Length;i++)
                {
                    data["V"+(i+1).ToString()+" Throttle"] = VehicleControllers[i].CurrentThrottle.ToString("F4"); // Get throttle
                    data["V"+(i+1).ToString()+" Steering"] = VehicleControllers[i].CurrentSteeringAngle.ToString("F4"); // Get steering angle
                    data["V"+(i+1).ToString()+" Speed"] = System.Math.Abs(System.Math.Round(VehicleControllers[i].Vehicle.transform.InverseTransformDirection(VehicleControllers[i].Vehicle.GetComponent<Rigidbody>().velocity).z, 4)).ToString("F4"); // Get speed
                }
                for(int i=0;i<AutomobileControllers.Length;i++)
                {
                    data["V"+(i+1).ToString()+" Throttle"] = AutomobileControllers[i].CurrentThrottle.ToString("F4"); // Get throttle
                    data["V"+(i+1).ToString()+" Steering"] = AutomobileControllers[i].CurrentSteeringAngle.ToString("F4"); // Get steering angle
                    data["V"+(i+1).ToString()+" Brake"] = AutomobileControllers[i].CurrentBrake.ToString("F4"); // Get brake
                    data["V"+(i+1).ToString()+" Handbrake"] = AutomobileControllers[i].CurrentHandbrake.ToString("F4"); // Get handbrake
                    data["V"+(i+1).ToString()+" Collisions"] = AutomobileControllers[i].collisionCount.ToString(); // Get collision count
                    data["V"+(i+1).ToString()+" Speed"] = System.Math.Abs(System.Math.Round(AutomobileControllers[i].currSpeed / AutomobileControllers[i].speedMultiplier, 4)).ToString("F4"); // Get speed
                }
                for(int i=0;i<VehicleControllers.Length+AutomobileControllers.Length;i++) // Assumed that VehicleControllers.Length+AutomobileControllers.Length >= others
                {    
                    data["V"+(i+1).ToString()+" Encoder Ticks"] = LeftWheelEncoders[i].Ticks.ToString() + " " + RightWheelEncoders[i].Ticks.ToString(); // Get encoder ticks
                    data["V"+(i+1).ToString()+" Encoder Angles"] = LeftWheelEncoders[i].Angle.ToString("F4") + " " + RightWheelEncoders[i].Angle.ToString("F4"); // Get encoder angles
                    data["V"+(i+1).ToString()+" Position"] = PositioningSystems[i].CurrentPosition[0].ToString("F4") + " " + PositioningSystems[i].CurrentPosition[1].ToString("F4") + " " + PositioningSystems[i].CurrentPosition[2].ToString("F4"); // Get vehicle position
                    data["V"+(i+1).ToString()+" Orientation Quaternion"] = InertialMeasurementUnits[i].CurrentOrientationQuaternion[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[2].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationQuaternion[3].ToString("F4"); // Get vehicle orientation (Quaternion)
                    data["V"+(i+1).ToString()+" Orientation Euler Angles"] = InertialMeasurementUnits[i].CurrentOrientationEulerAngles[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentOrientationEulerAngles[2].ToString("F4"); // Get vehicle orientation (Euler Angles)
                    data["V"+(i+1).ToString()+" Angular Velocity"] = InertialMeasurementUnits[i].CurrentAngularVelocity[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentAngularVelocity[2].ToString("F4"); // Get angular velocity of the vehicle
                    data["V"+(i+1).ToString()+" Linear Acceleration"] = InertialMeasurementUnits[i].CurrentLinearAcceleration[0].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[1].ToString("F4") + " " + InertialMeasurementUnits[i].CurrentLinearAcceleration[2].ToString("F4"); // Get linear acceleration of the vehicle
                    if(LIDARUnits.Length != 0)
                    {
                        data["V"+(i+1).ToString()+" LIDAR Scan Rate"] = LIDARUnits[i].CurrentScanRate.ToString("F4"); // Get LIDAR scan rate
                        if(LIDARUnits[i].CurrentRangeArray[LIDARUnits[i].CurrentRangeArray.Length-1] != null)
                        {
                            data["V" + (i + 1).ToString() + " LIDAR Range Array"] = DataCompressor.CompressArray(LIDARUnits[i].CurrentRangeArray); // Get LIDAR range array
                            if (IntensityArray) data["V" + (i + 1).ToString() + " LIDAR Intensity Array"] = DataCompressor.CompressArray(LIDARUnits[i].CurrentIntensityArray); // Get LIDAR intensity array
                        }
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
            if (LapTimers.Length != 0)
            {
                for (int i = 0; i < LapTimers.Length; i++)
                {
                    data["V"+(i+1).ToString()+" Lap Count"] = LapTimers[i].LapCount.ToString(); // Get lap count
                    data["V"+(i+1).ToString()+" Lap Time"] = LapTimers[i].LapTime.ToString("F4"); // Get lap time
                    data["V"+(i+1).ToString()+" Last Lap Time"] = LapTimers[i].LastLapTime.ToString("F4"); // Get last lap count
                    data["V"+(i+1).ToString()+" Best Lap Time"] = LapTimers[i].BestLapTime.ToString("F4"); // Get best lap time
                    data["V"+(i+1).ToString()+" Collisions"] = LapTimers[i].CollisionCount.ToString(); // Get collision count
                }
            }
            socket.Emit("Bridge", new JSONObject(data)); // Write data to server
        });
    }
}
