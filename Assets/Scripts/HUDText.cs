using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class HUDText : MonoBehaviour
{
    /*
    This script updates the HUD data.
    */

    public Text HUD; // HUD data

    public Timer Timer; // `Timer` reference
    public FPSCounter FPSCounter; // `FPSCounter` reference
    public VehicleTeleoperation VehicleTeleoperation; // `VehicleTeleoperation` reference
    public WheelEncoder LeftWheelEncoder; // `WheelEncoder` reference for left wheel
    public WheelEncoder RightWheelEncoder; // `WheelEncoder` reference for right wheel
    public IPS IPS; // `IPS` reference
    public IMU IMU; // `IMU` reference
    public LIDAR LIDAR; // `LIDAR` reference

    void Start()
    {
        HUD.text =
        "Simulation Time:\t\t" + "00:00:00" + "\n" +
        "Frame Rate:\t\t\t\t" + "00" + " Hz\n\n" +

        "Driving Mode:\t\t" + "Manual" + "\n" +
        "Gear:\t\t\t\t\t\t" + "D" + "\n" +
        "Speed:\t\t\t\t\t" + "0.00" + " m/s\n\n" +

        "Throttle:\t\t" + "0.00" + "%\n" +
        "Steering:\t\t" + "0.00" + " rad\n\n" +

        "Encoder Ticks:\t" + "[0, 0]" + "\n\n" +

        "IPS Data:\t\t" + "[0, 0, 0]" + " m\n\n" +

        "IMU Data:\t" + "[0, 0, 0]" + " rad\n" +
        "\t\t\t\t\t\t" + "[0, 0, 0]" + " rad/s\n" +
        "\t\t\t\t\t\t" + "[0, 0, 0]" + " m/s^2\n\n" +

        "LIDAR Measurement:\t" + "inf" + " m";
    }

    void Update()
    {
        string simulation_time = Timer.SimulationTime;

        string frame_rate = FPSCounter.FPS.ToString();

        string driving_mode = "";
        if(VehicleTeleoperation.DrivingMode==0)
        {
            driving_mode = "Manual";
        }
        else
        {
            driving_mode = "Autonomous";
        }

        string gear = "";
        if(System.Math.Round((VehicleTeleoperation.RearLeftWheelCollider.rpm + VehicleTeleoperation.RearRightWheelCollider.rpm)/2, 1) < 0)
        {
            gear = "R";
        }
        else
        {
            gear = "D";
        }

        string speed = System.Math.Abs(System.Math.Round(VehicleTeleoperation.Vehicle.transform.InverseTransformDirection(VehicleTeleoperation.Vehicle.GetComponent<Rigidbody>().velocity).z,2)).ToString("F2");

        string throttle = (System.Math.Abs(VehicleTeleoperation.CurrentThrottle)*100).ToString("F2");

        string steering_angle = VehicleTeleoperation.CurrentSteeringAngle.ToString("F2");

        string encoder_ticks = "[" + LeftWheelEncoder.Ticks.ToString() + ", " + RightWheelEncoder.Ticks.ToString() + "]";

        string position = "[" + IPS.CurrentPosition[0].ToString("F2") + ", " + IPS.CurrentPosition[1].ToString("F2") + ", " + IPS.CurrentPosition[2].ToString("F2") + "]";

        string orientation = "[" + IMU.CurrentOrientationEulerAngles[0].ToString("F2") + ", " + IMU.CurrentOrientationEulerAngles[1].ToString("F2") + ", " + IMU.CurrentOrientationEulerAngles[2].ToString("F2") + "]";
        string angular_velocity = "[" + IMU.CurrentAngularVelocity[0].ToString("F2") + ", " + IMU.CurrentAngularVelocity[1].ToString("F2") + ", " + IMU.CurrentAngularVelocity[2].ToString("F2") + "]";
        string linear_acceleration = "[" + IMU.CurrentLinearAcceleration[0].ToString("F2") + ", " + IMU.CurrentLinearAcceleration[1].ToString("F2") + ", " + IMU.CurrentLinearAcceleration[2].ToString("F2") + "]";

        string lidar_measurement = LIDAR.CurrentMeasurement;

        HUD.text =
        "Simulation Time:\t\t" + simulation_time + "\n" +
        "Frame Rate:\t\t\t\t" + frame_rate + " Hz\n\n" +

        "Driving Mode:\t\t" + driving_mode + "\n" +
        "Gear:\t\t\t\t\t\t" + gear + "\n" +
        "Speed:\t\t\t\t\t" + speed + " m/s\n\n" +

        "Throttle:\t\t" + throttle + "%\n" +
        "Steering:\t\t" + steering_angle + " rad\n\n" +

        "Encoder Ticks:\t" + encoder_ticks + "\n\n" +

        "IPS Data:\t\t" + position + " m\n\n" +

        "IMU Data:\t" + orientation + " rad\n" +
        "\t\t\t\t\t\t" + angular_velocity + " rad/s\n" +
        "\t\t\t\t\t\t" + linear_acceleration + " m/s^2\n\n" +

        "LIDAR Measurement:\t" + lidar_measurement + " m";
    }
}
