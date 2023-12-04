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
    public DataRecorder DataRecorder;

    public Timer Timer; // `Timer` reference
    public FPSCounter FPSCounter; // `FPSCounter` reference
    public bool SkidSteer = false; // Choose whether to display `Steer` as angle (rad) or percent (%)
    public VehicleController[] VehicleControllers; // `VehicleController` references
    public AutomobileController[] AutomobileControllers; // `AutomobileController` references
    public WheelEncoder LeftWheelEncoder; // `WheelEncoder` reference for left wheel
    public WheelEncoder RightWheelEncoder; // `WheelEncoder` reference for right wheel
    public bool GNSS = false; // Choose whether to display "GPS" or "GNSS"
    public GPS GPS; // `GPS` reference
    public IMU IMU; // `IMU` reference
    public bool EnableLIDAR = true; // Choose whether to display LIDAR data on HUD
    public LIDAR LIDAR; // `LIDAR` reference

    void Start()
    {
        ResetHUDText();
    }

    void Update()
    {
        if (DataRecorder.getSaveStatus()) ResetHUDText();
        else UpdateHUDText();
    }

    void ResetHUDText()
    {
        if (EnableLIDAR)
        {
            if (GNSS)
            {
                if (SkidSteer && VehicleControllers.Length != 0)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + "00:00:00" + "\n" +
                    "Frame Rate:\t\t\t\t" + "00" + " Hz\n\n" +

                    "Driving Mode:\t\t" + "Manual" + "\n" +
                    "Gear:\t\t\t\t\t\t" + "D" + "\n" +
                    "Speed:\t\t\t\t\t" + "0.00" + " m/s\n\n" +

                    "Throttle:\t\t" + "0.00" + "%\n" +
                    "Steering:\t\t" + "0.00" + "%\n\n" +

                    "Encoder Ticks:\t" + "[0, 0]" + "\n\n" +

                    "GNSS Data:\t\t" + "[0, 0, 0]" + " m\n\n" +

                    "IMU Data:\t" + "[0, 0, 0]" + " rad\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " rad/s\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " m/s^2\n\n" +

                    "LIDAR Measurement:\t" + "inf" + " m";
                }
                else if (!SkidSteer && VehicleControllers.Length != 0)
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

                    "GNSS Data:\t\t" + "[0, 0, 0]" + " m\n\n" +

                    "IMU Data:\t" + "[0, 0, 0]" + " rad\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " rad/s\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " m/s^2\n\n" +

                    "LIDAR Measurement:\t" + "inf" + " m";
                }
                else if (AutomobileControllers.Length != 0)
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

                    "GNSS Data:\t" + "[0, 0, 0]" + " m\n\n" +

                    "IMU Data:\t" + "[0, 0, 0]" + " rad\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " rad/s\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " m/s^2\n\n" +

                    "LIDAR Measurement:\t" + "inf" + " m";
                }
            }
            else
            {
                if (SkidSteer)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + "00:00:00" + "\n" +
                    "Frame Rate:\t\t\t\t" + "00" + " Hz\n\n" +

                    "Driving Mode:\t\t" + "Manual" + "\n" +
                    "Gear:\t\t\t\t\t\t" + "D" + "\n" +
                    "Speed:\t\t\t\t\t" + "0.00" + " m/s\n\n" +

                    "Throttle:\t\t" + "0.00" + "%\n" +
                    "Steering:\t\t" + "0.00" + "%\n\n" +

                    "Encoder Ticks:\t" + "[0, 0]" + "\n\n" +

                    "IPS Data:\t\t" + "[0, 0, 0]" + " m\n\n" +

                    "IMU Data:\t" + "[0, 0, 0]" + " rad\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " rad/s\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " m/s^2\n\n" +

                    "LIDAR Measurement:\t" + "inf" + " m";
                }
                else
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
            }
        }
        else
        {
            if (GNSS)
            {
                if (SkidSteer && VehicleControllers.Length != 0)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + "00:00:00" + "\n" +
                    "Frame Rate:\t\t\t\t" + "00" + " Hz\n\n" +

                    "Driving Mode:\t\t" + "Manual" + "\n" +
                    "Gear:\t\t\t\t\t\t" + "D" + "\n" +
                    "Speed:\t\t\t\t\t" + "0.00" + " m/s\n\n" +

                    "Throttle:\t\t" + "0.00" + "%\n" +
                    "Steering:\t\t" + "0.00" + "%\n\n" +

                    "Encoder Ticks:\t" + "[0, 0]" + "\n\n" +

                    "GNSS Data:\t\t" + "[0, 0, 0]" + " m\n\n" +

                    "IMU Data:\t" + "[0, 0, 0]" + " rad\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " rad/s\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " m/s^2\n\n" +

                    "";
                }
                else if (!SkidSteer && VehicleControllers.Length != 0)
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

                    "GNSS Data:\t\t" + "[0, 0, 0]" + " m\n\n" +

                    "IMU Data:\t" + "[0, 0, 0]" + " rad\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " rad/s\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " m/s^2\n\n" +

                    "";
                }
                else if (AutomobileControllers.Length != 0)
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

                    "GNSS Data:\t" + "[0, 0, 0]" + " m\n\n" +

                    "IMU Data:\t" + "[0, 0, 0]" + " rad\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " rad/s\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " m/s^2\n\n" +

                    "";
                }
            }
            else{
                if (SkidSteer)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + "00:00:00" + "\n" +
                    "Frame Rate:\t\t\t\t" + "00" + " Hz\n\n" +

                    "Driving Mode:\t\t" + "Manual" + "\n" +
                    "Gear:\t\t\t\t\t\t" + "D" + "\n" +
                    "Speed:\t\t\t\t\t" + "0.00" + " m/s\n\n" +

                    "Throttle:\t\t" + "0.00" + "%\n" +
                    "Steering:\t\t" + "0.00" + "%\n\n" +

                    "Encoder Ticks:\t" + "[0, 0]" + "\n\n" +

                    "IPS Data:\t\t" + "[0, 0, 0]" + " m\n\n" +

                    "IMU Data:\t" + "[0, 0, 0]" + " rad\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " rad/s\n" +
                    "\t\t\t\t\t\t" + "[0, 0, 0]" + " m/s^2\n\n" +

                    "";
                }
                else
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

                    "";
                }
            }
        }
    }

    void UpdateHUDText()
    {
        string simulation_time = Timer.SimulationTime;

        string frame_rate = FPSCounter.FPS.ToString();

        string driving_mode = "";
        if(VehicleControllers.Length != 0)
        {
            if(VehicleControllers[0].DrivingMode==0)
            {
                driving_mode = "Manual";
            }
            else
            {
                driving_mode = "Autonomous";
            }
        }
        else if(AutomobileControllers.Length != 0)
        {
            if(AutomobileControllers[0].DrivingMode==0)
            {
                driving_mode = "Manual";
            }
            else
            {
                driving_mode = "Autonomous";
            }
        }

        string gear = "";
        if(VehicleControllers.Length != 0)
        {
            if(System.Math.Round((VehicleControllers[0].RearLeftWheelCollider.rpm + VehicleControllers[0].RearRightWheelCollider.rpm)/2, 1) < 0)
            {
                gear = "R";
            }
            else
            {
                 gear = "D";
            }
        }
        else if(AutomobileControllers.Length != 0)
        {
            if (AutomobileControllers[0].currSpeed != 0 && AutomobileControllers[0].gearNum > 0) gear = "D";
            if (AutomobileControllers[0].currSpeed == 0 && !Input.GetKey(KeyCode.Space)) gear = "N";
            if (AutomobileControllers[0].currSpeed == 0 && Input.GetKey(KeyCode.Space)) gear = "P";
            if (AutomobileControllers[0].currSpeed != 0 && AutomobileControllers[0].gearNum < 0) gear = "R";
        }

        string speed = "";
        if(VehicleControllers.Length != 0)
        {
            speed = System.Math.Abs(System.Math.Round(VehicleControllers[0].Vehicle.transform.InverseTransformDirection(VehicleControllers[0].Vehicle.GetComponent<Rigidbody>().velocity).z,2)).ToString("F2");
        }
        else if(AutomobileControllers.Length != 0)
        {
            speed = System.Math.Abs(System.Math.Round(AutomobileControllers[0].currSpeed/AutomobileControllers[0].speedMultiplier,2)).ToString("F2");
        }

        string throttle = "";
        if(VehicleControllers.Length != 0)
        {
            throttle = (System.Math.Abs(VehicleControllers[0].CurrentThrottle)*100).ToString("F2");
        }
        else if(AutomobileControllers.Length != 0)
        {
            throttle = (System.Math.Abs(AutomobileControllers[0].CurrentThrottle)*100).ToString("F2");
        }

        string steering_angle = "";
        if(VehicleControllers.Length != 0)
        {
            steering_angle = VehicleControllers[0].CurrentSteeringAngle.ToString("F2");
        }
        else if(AutomobileControllers.Length != 0)
        {
            steering_angle = AutomobileControllers[0].CurrentSteeringAngle.ToString("F2");
        }

        string encoder_ticks = "[" + LeftWheelEncoder.Ticks.ToString() + ", " + RightWheelEncoder.Ticks.ToString() + "]";

        string position = "[" + GPS.CurrentPosition[0].ToString("F2") + ", " + GPS.CurrentPosition[1].ToString("F2") + ", " + GPS.CurrentPosition[2].ToString("F2") + "]";

        string orientation = "[" + IMU.CurrentOrientationEulerAngles[0].ToString("F2") + ", " + IMU.CurrentOrientationEulerAngles[1].ToString("F2") + ", " + IMU.CurrentOrientationEulerAngles[2].ToString("F2") + "]";
        string angular_velocity = "[" + IMU.CurrentAngularVelocity[0].ToString("F2") + ", " + IMU.CurrentAngularVelocity[1].ToString("F2") + ", " + IMU.CurrentAngularVelocity[2].ToString("F2") + "]";
        string linear_acceleration = "[" + IMU.CurrentLinearAcceleration[0].ToString("F2") + ", " + IMU.CurrentLinearAcceleration[1].ToString("F2") + ", " + IMU.CurrentLinearAcceleration[2].ToString("F2") + "]";

        if (EnableLIDAR)
        {
            string lidar_measurement = LIDAR.CurrentMeasurement;
            if (GNSS)
            {
                if (SkidSteer && VehicleControllers.Length != 0)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + simulation_time + "\n" +
                    "Frame Rate:\t\t\t\t" + frame_rate + " Hz\n\n" +

                    "Driving Mode:\t\t" + driving_mode + "\n" +
                    "Gear:\t\t\t\t\t\t" + gear + "\n" +
                    "Speed:\t\t\t\t\t" + speed + " m/s\n\n" +

                    "Throttle:\t\t" + throttle + "%\n" +
                    "Steering:\t\t" + steering_angle + "%\n\n" +

                    "Encoder Ticks:\t" + encoder_ticks + "\n\n" +

                    "GNSS Data:\t\t" + position + " m\n\n" +

                    "IMU Data:\t" + orientation + " rad\n" +
                    "\t\t\t\t\t\t" + angular_velocity + " rad/s\n" +
                    "\t\t\t\t\t\t" + linear_acceleration + " m/s^2\n\n" +

                    "LIDAR Measurement:\t" + lidar_measurement + " m";
                }
                else if (!SkidSteer && VehicleControllers.Length != 0)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + simulation_time + "\n" +
                    "Frame Rate:\t\t\t\t" + frame_rate + " Hz\n\n" +

                    "Driving Mode:\t\t" + driving_mode + "\n" +
                    "Gear:\t\t\t\t\t\t" + gear + "\n" +
                    "Speed:\t\t\t\t\t" + speed + " m/s\n\n" +

                    "Throttle:\t\t" + throttle + "%\n" +
                    "Steering:\t\t" + steering_angle + " rad\n\n" +

                    "Encoder Ticks:\t" + encoder_ticks + "\n\n" +

                    "GNSS Data:\t\t" + position + " m\n\n" +

                    "IMU Data:\t" + orientation + " rad\n" +
                    "\t\t\t\t\t\t" + angular_velocity + " rad/s\n" +
                    "\t\t\t\t\t\t" + linear_acceleration + " m/s^2\n\n" +

                    "LIDAR Measurement:\t" + lidar_measurement + " m";
                }
                else if (AutomobileControllers.Length != 0)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + simulation_time + "\n" +
                    "Frame Rate:\t\t\t\t" + frame_rate + " Hz\n\n" +

                    "Driving Mode:\t\t" + driving_mode + "\n" +
                    "Gear:\t\t\t\t\t\t" + gear + "\n" +
                    "Speed:\t\t\t\t\t" + speed + " m/s\n\n" +

                    "Throttle:\t\t" + throttle + "%\n" +
                    "Steering:\t\t" + steering_angle + " rad\n\n" +

                    "Encoder Ticks:\t" + encoder_ticks + "\n\n" +

                    "GNSS Data:\t" + position + " m\n\n" +

                    "IMU Data:\t" + orientation + " rad\n" +
                    "\t\t\t\t\t\t" + angular_velocity + " rad/s\n" +
                    "\t\t\t\t\t\t" + linear_acceleration + " m/s^2\n\n" +

                    "LIDAR Measurement:\t" + lidar_measurement + " m";
                }
            }
            else{
                if (SkidSteer)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + simulation_time + "\n" +
                    "Frame Rate:\t\t\t\t" + frame_rate + " Hz\n\n" +

                    "Driving Mode:\t\t" + driving_mode + "\n" +
                    "Gear:\t\t\t\t\t\t" + gear + "\n" +
                    "Speed:\t\t\t\t\t" + speed + " m/s\n\n" +

                    "Throttle:\t\t" + throttle + "%\n" +
                    "Steering:\t\t" + steering_angle + "%\n\n" +

                    "Encoder Ticks:\t" + encoder_ticks + "\n\n" +

                    "IPS Data:\t\t" + position + " m\n\n" +

                    "IMU Data:\t" + orientation + " rad\n" +
                    "\t\t\t\t\t\t" + angular_velocity + " rad/s\n" +
                    "\t\t\t\t\t\t" + linear_acceleration + " m/s^2\n\n" +

                    "LIDAR Measurement:\t" + lidar_measurement + " m";
                }
                else
                {
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
        }
        else
        {
            if (GNSS)
            {
                if (SkidSteer && VehicleControllers.Length != 0)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + simulation_time + "\n" +
                    "Frame Rate:\t\t\t\t" + frame_rate + " Hz\n\n" +

                    "Driving Mode:\t\t" + driving_mode + "\n" +
                    "Gear:\t\t\t\t\t\t" + gear + "\n" +
                    "Speed:\t\t\t\t\t" + speed + " m/s\n\n" +

                    "Throttle:\t\t" + throttle + "%\n" +
                    "Steering:\t\t" + steering_angle + "%\n\n" +

                    "Encoder Ticks:\t" + encoder_ticks + "\n\n" +

                    "GNSS Data:\t\t" + position + " m\n\n" +

                    "IMU Data:\t" + orientation + " rad\n" +
                    "\t\t\t\t\t\t" + angular_velocity + " rad/s\n" +
                    "\t\t\t\t\t\t" + linear_acceleration + " m/s^2\n\n" +

                    "";
                }
                else if (!SkidSteer && VehicleControllers.Length != 0)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + simulation_time + "\n" +
                    "Frame Rate:\t\t\t\t" + frame_rate + " Hz\n\n" +

                    "Driving Mode:\t\t" + driving_mode + "\n" +
                    "Gear:\t\t\t\t\t\t" + gear + "\n" +
                    "Speed:\t\t\t\t\t" + speed + " m/s\n\n" +

                    "Throttle:\t\t" + throttle + "%\n" +
                    "Steering:\t\t" + steering_angle + " rad\n\n" +

                    "Encoder Ticks:\t" + encoder_ticks + "\n\n" +

                    "GNSS Data:\t\t" + position + " m\n\n" +

                    "IMU Data:\t" + orientation + " rad\n" +
                    "\t\t\t\t\t\t" + angular_velocity + " rad/s\n" +
                    "\t\t\t\t\t\t" + linear_acceleration + " m/s^2\n\n" +

                    "";
                }
                else if (AutomobileControllers.Length != 0)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + simulation_time + "\n" +
                    "Frame Rate:\t\t\t\t" + frame_rate + " Hz\n\n" +

                    "Driving Mode:\t\t" + driving_mode + "\n" +
                    "Gear:\t\t\t\t\t\t" + gear + "\n" +
                    "Speed:\t\t\t\t\t" + speed + " m/s\n\n" +

                    "Throttle:\t\t" + throttle + "%\n" +
                    "Steering:\t\t" + steering_angle + " rad\n\n" +

                    "Encoder Ticks:\t" + encoder_ticks + "\n\n" +

                    "GNSS Data:\t" + position + " m\n\n" +

                    "IMU Data:\t" + orientation + " rad\n" +
                    "\t\t\t\t\t\t" + angular_velocity + " rad/s\n" +
                    "\t\t\t\t\t\t" + linear_acceleration + " m/s^2\n\n" +

                    "";
                }
            }
            else{
                if (SkidSteer)
                {
                    HUD.text =
                    "Simulation Time:\t\t" + simulation_time + "\n" +
                    "Frame Rate:\t\t\t\t" + frame_rate + " Hz\n\n" +

                    "Driving Mode:\t\t" + driving_mode + "\n" +
                    "Gear:\t\t\t\t\t\t" + gear + "\n" +
                    "Speed:\t\t\t\t\t" + speed + " m/s\n\n" +

                    "Throttle:\t\t" + throttle + "%\n" +
                    "Steering:\t\t" + steering_angle + "%\n\n" +

                    "Encoder Ticks:\t" + encoder_ticks + "\n\n" +

                    "IPS Data:\t\t" + position + " m\n\n" +

                    "IMU Data:\t" + orientation + " rad\n" +
                    "\t\t\t\t\t\t" + angular_velocity + " rad/s\n" +
                    "\t\t\t\t\t\t" + linear_acceleration + " m/s^2\n\n" +

                    "";
                }
                else
                {
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

                    "";
                }
            }
        }
    }
}
