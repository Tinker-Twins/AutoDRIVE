using UnityEngine;
using System;

[ExecuteInEditMode]
public class Telemetry : MonoBehaviour
{

    string apiMode = "api";  // Constant to identify the package
    public string game = "AutoDRIVE Simulator";  // Constant to identify the game
    public string vehicle = "Hunter SE";  // Constant to identify the vehicle
    public string location = "Greensward";  // Constant to identify the location
    public float angularGain = 1f; // Gain for roll, pitch, yaw motion
    public IMU IMU; // `IMU` instance for inertial measurements
    uint apiVersion = 102;  // Constant of the current version of the api

    private float[] OrientationArray = new float[3];
    private float[] LinearVelocityArray = new float[3];
    private float[] AngularVelocityArray = new float[3];
    private float[] LinearAccelerationArray = new float[3];

    Rigidbody vehicleBody; // Vehicle rigidbody

    void Start()
    {
        vehicleBody = GetComponent<Rigidbody>(); // Cache rigidbody
    }

    void Update()
    {
        OrientationArray = IMU.CurrentOrientationEulerAngles;
        LinearVelocityArray = IMU.CurrentLinearVelocity;
        LinearAccelerationArray = IMU.CurrentLinearAcceleration;

        SimRacingStudio.SimRacingStudio_SendTelemetry(apiMode.PadRight(50).ToCharArray()
                                                     , apiVersion // API version
                                                     , game.PadRight(50).ToCharArray() // Game name
                                                     , vehicle.PadRight(50).ToCharArray() // Vehicle name
                                                     , location.PadRight(50).ToCharArray() // Environment name
                                                     , Convert.ToSingle(vehicleBody.velocity.magnitude * 3.6) // Speed
                                                     , 1000 // Engine RPM
                                                     , 30000 // Max. engine RPM
                                                     , 1 // Gear (-1=reverse, 0=neutral, 1-9=gear)
                                                     , -(((OrientationArray[1] * 57.29578f) + 180f) % 360f - 180f) *  angularGain // Pitch (+up, [-180,180] deg)
                                                     , (((OrientationArray[0] * 57.29578f) + 180f) % 360f - 180f) * angularGain // Roll (+right, [-180,180] deg)
                                                     , -(((OrientationArray[2] * 57.29578f) + 180f) % 360f - 180f) * angularGain // Yaw (+cwft, [-180,180] deg)
                                                     , LinearVelocityArray[1] // Lateral velocity
                                                     , LinearAccelerationArray[1] // Lateral acceleration
                                                     , LinearAccelerationArray[2] // Vertical acceleration
                                                     , LinearAccelerationArray[0] //Longitudinal acceleration
                                                     , 0 // FL suspension travel
                                                     , 0 // FR suspension travel
                                                     , 0 // RL suspension travel
                                                     , 0 // RR suspension travel
                                                     , 0 // Terrain type (0=others, 1=rumble-strip, 2=asphalt)
                                                     , 0 // Terrain type (0=others, 1=rumble-strip, 2=asphalt) 
                                                     , 0 // Terrain type (0=others, 1=rumble-strip, 2=asphalt)
                                                     , 0); // Terrain type (0=others, 1=rumble-strip, 2=asphalt)
    }
}
