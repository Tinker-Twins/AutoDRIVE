using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VehicleTeleoperation : MonoBehaviour
{
    /*
    This script actuates the drive motors and steering servo in order to
    teleoperate the vehicle.

    While throttle (for both forward and reverse drive) and steering inputs can
    be controlled manually/autonomously, the brakes are automatically applied
    unless the throttle input is non-zero.
    */

    private float ThrottleInput;
    private float SteeringInput;

    public GameObject Vehicle;
    public WheelCollider FrontLeftWheelCollider, FrontRightWheelCollider;
    public WheelCollider RearLeftWheelCollider, RearRightWheelCollider;
    public Transform FrontLeftWheelTransform, FrontRightWheelTransform;
    public Transform RearLeftWheelTransform, RearRightWheelTransform;
    public float DriveActuationLimit = 130; // RPM
    public float CalibrationRPM = 100; // RPM
    public float RPMCalibrationFactor = 0;
    public float SteeringActuationLimit = 30; // Degrees
    [Range(-1,1)] public float AutonomousThrottle = 0;
    [Range(-1,1)] public float AutonomousSteering = 0;
    public int DrivingMode = 0; // Driving mode: 0 is manual, 1 is autonomous
    private float DriveTorque = 0; // N-m
    private float BrakeTorque = 0; // N-m
    private float SteeringAngle = 0; // Degrees

    public int CurrentDrivingMode
    {
        get { return DrivingMode; }
    }

    public float CurrentThrottle
    {
        get { return DriveTorque/((RPMCalibrationFactor/CalibrationRPM)*DriveActuationLimit); }
        set { AutonomousThrottle = value; }
    }

    public float CurrentSteeringAngle
    {
        get { return SteeringAngle*(Mathf.PI/180); }
        set { AutonomousSteering = value; }
    }

    public void GetInput()
  	{
        ThrottleInput = Input.GetAxis("Vertical");
        SteeringInput = Input.GetAxis("Horizontal");
  	}

  	private void Steer()
  	{
        if(DrivingMode == 0) SteeringAngle = SteeringActuationLimit*SteeringInput; // Manual Driving
        else SteeringAngle = SteeringActuationLimit*AutonomousSteering; // Autonomous Driving

    		FrontLeftWheelCollider.steerAngle = SteeringAngle;
    		FrontRightWheelCollider.steerAngle = SteeringAngle;
  	}

    private void Drive()
  	{
        //Debug.Log("RPM:" + (RearLeftWheelCollider.rpm + RearRightWheelCollider.rpm)/2); // Average wheel speed (RPM)

        if(DrivingMode == 0) DriveTorque = (RPMCalibrationFactor/CalibrationRPM)*DriveActuationLimit*ThrottleInput; // Manual Driving
        else DriveTorque = (RPMCalibrationFactor/CalibrationRPM)*DriveActuationLimit*AutonomousThrottle; // Autonomous Driving

        BrakeTorque = (RPMCalibrationFactor/CalibrationRPM)*DriveActuationLimit; // Compute brake torque

        if(DriveTorque == 0)
        {
            RearLeftWheelCollider.motorTorque = 0;
            RearRightWheelCollider.motorTorque = 0;
            RearLeftWheelCollider.brakeTorque = BrakeTorque;
            RearRightWheelCollider.brakeTorque = BrakeTorque;
            FrontLeftWheelCollider.brakeTorque = BrakeTorque;
            FrontRightWheelCollider.brakeTorque = BrakeTorque;
        }
        else
        {
            FrontLeftWheelCollider.brakeTorque = 0;
            FrontRightWheelCollider.brakeTorque = 0;
            RearLeftWheelCollider.brakeTorque = 0;
            RearRightWheelCollider.brakeTorque = 0;
            RearLeftWheelCollider.motorTorque = DriveTorque;
            RearRightWheelCollider.motorTorque = DriveTorque;
        }
  	}

  	private void UpdateWheelPoses()
  	{
    		UpdateWheelPose(FrontLeftWheelCollider, FrontLeftWheelTransform);
    		UpdateWheelPose(FrontRightWheelCollider, FrontRightWheelTransform);
    		UpdateWheelPose(RearLeftWheelCollider, RearLeftWheelTransform);
    		UpdateWheelPose(RearRightWheelCollider, RearRightWheelTransform);
  	}

  	private void UpdateWheelPose(WheelCollider _collider, Transform _transform)
  	{
    		Vector3 _pos = _transform.position;
    		Quaternion _quat = _transform.rotation;
    		_collider.GetWorldPose(out _pos, out _quat);
    		_transform.position = _pos;
    		_transform.rotation = _quat;
  	}

  	void FixedUpdate()
  	{
    		GetInput();
    		Steer();
    		Drive();
    		UpdateWheelPoses();
  	}
}
