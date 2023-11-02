using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VehicleController : MonoBehaviour
{
    /*
    This script actuates the drive and steering actuators in order to control
    the motion of the vehicle.

    While throttle (for both forward and reverse drive) and steering inputs can
    be controlled manually/autonomously, the brakes are automatically applied
    unless the throttle input is non-zero.

    If steerable, the individual turning angles for left and right wheels are calculated
    using the commanded steering angle based on the Ackermann steering geometry
    defined by the wheelbase and track width parameters.
    */

    private float ThrottleInput;
    private float SteeringInput;

    public GameObject Vehicle;
    public Rigidbody VehicleRigidBody;
    public Vector3 COM;
    public WheelCollider FrontLeftWheelCollider, FrontRightWheelCollider;
    public WheelCollider RearLeftWheelCollider, RearRightWheelCollider;
    public Transform FrontLeftWheelTransform, FrontRightWheelTransform;
    public Transform RearLeftWheelTransform, RearRightWheelTransform;
    public enum DriveType {IRWD, IFWD, IAWD, CRWD, CFWD, CAWD, SkidSteer};
    public DriveType driveType = DriveType.IRWD; // Set drive type
    public float SteeringRate = 315.789f; // deg/s (w.r.t. physics timestep)
    public float Wheelbase = 141.54f; // mm
    public float TrackWidth = 153f; // mm
    public float WheelRadius = 0.0325f;// m
    public float MotorTorque = 2.352f; // N-m
    public float SteeringLimit = 30f; // deg
    public float LinearGain = 6.8f;
    public float AngularGain = 0.008f;
    public float LinearVelocityLimit = 0.26f; // m/s
    public float AngularVelocityLimit = 0.42f; // rad/s
    [Range(-1,1)] public float AutonomousThrottle = 0;
    [Range(-1,1)] public float AutonomousSteering = 0;
    public int DrivingMode = 1; // Driving mode: 0 is manual, 1 is autonomous

    private float DriveTorque = 0; // N-m
    private float BrakeTorque = 0; // N-m
    private float SteeringAngle = 0; // deg
    private bool MouseHold;
	private float MouseStart;

    private float currentSteeringAngle = 0;
    private float lastSteeringAngle = 0;
    private float angularVelocity = 0; // deg
    private float linearVelocity = 0; // deg

    public int CurrentDrivingMode
    {
        get { return DrivingMode; }
    }

    public float CurrentThrottle
    {
        get { return DriveTorque/MotorTorque; }
        set { AutonomousThrottle = value; }
    }

    public float CurrentSteeringAngle
    {
        get { return -SteeringAngle*(Mathf.PI/180); }
        set { AutonomousSteering = value; }
    }

    void Start()
    {
        VehicleRigidBody.centerOfMass = COM; // Set COM
    }

    public void GetInput()
  	{
        // THROTTLE INPUT
        ThrottleInput = Input.GetAxis("Vertical");

        // STEERING INPUT

        // Continuous control using mouse
        if (Input.GetMouseButton(0))
        {
            float MousePosition = Input.mousePosition.x; // Get the mouse position
            // Check if its the first time pressing down on mouse button
            if (!MouseHold)
            {
                MouseHold = true; // The mouse button is held down
                MouseStart = MousePosition;   // Set the reference position for tracking mouse movement
            }
            SteeringInput = -Mathf.Clamp((MousePosition - MouseStart)/(Screen.width/6), -1, 1); // Clamp the steering command in range of [-1, 1]
        }

        // Discrete control using keyboard
        else
        {
            MouseHold = false; // The mouse button is released
            SteeringInput = -Input.GetAxis("Horizontal");
        }
  	}

  	private void Steer()
  	{
        if(DrivingMode == 0) SteeringAngle = -SteeringInput*SteeringLimit; // Manual Driving
        else // Autonomous Driving
        {
            if(AutonomousSteering > (lastSteeringAngle/SteeringLimit))
            {
                SteeringAngle -= SteeringRate*Time.deltaTime; // Decrement steering angle
                if(SteeringAngle <= -AutonomousSteering*SteeringLimit) SteeringAngle = -AutonomousSteering*SteeringLimit; // Limit to setpoint
            }
            else
            {
                SteeringAngle += SteeringRate*Time.deltaTime; // Increment steering angle
                if(SteeringAngle >= -AutonomousSteering*SteeringLimit) SteeringAngle = -AutonomousSteering*SteeringLimit; // Limit to setpoint
            }
            lastSteeringAngle = SteeringAngle; // Update previous steering angle
        }
        // currentSteeringAngle = Mathf.LerpAngle(lastSteeringAngle, SteeringAngle, SteeringRate*Time.deltaTime);
        //Debug.Log("Steering Angle: " + SteeringAngle);
        //Debug.Log("Left Wheel Angle: " + Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))/((2*Wheelbase)+(TrackWidth*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))))));
        //Debug.Log("Right Wheel Angle: " + Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))/((2*Wheelbase)-(TrackWidth*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))))));
    	FrontLeftWheelCollider.steerAngle = Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))/((2*Wheelbase)+(TrackWidth*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle))))));
    	FrontRightWheelCollider.steerAngle = Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))/((2*Wheelbase)-(TrackWidth*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle))))));
  	}

    private void Drive()
  	{
        //Debug.Log("RPM: " + (RearLeftWheelCollider.rpm + RearRightWheelCollider.rpm)/2); // Average wheel speed (RPM)
        //Debug.Log("Speed: " + (Mathf.PI*0.065)*((RearLeftWheelCollider.rpm + RearRightWheelCollider.rpm)/120)); // Average vehicle speed (m/s)

        if(DrivingMode == 0) DriveTorque = ThrottleInput*MotorTorque; // Manual Driving
        else DriveTorque = AutonomousThrottle*MotorTorque; // Autonomous Driving

        BrakeTorque = MotorTorque; // Compute brake torque

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
            if(driveType==DriveType.IRWD)  // RWD with independent actuators
            {
                FrontLeftWheelCollider.brakeTorque = 0;
                FrontRightWheelCollider.brakeTorque = 0;
                RearLeftWheelCollider.brakeTorque = 0;
                RearRightWheelCollider.brakeTorque = 0;
                RearLeftWheelCollider.motorTorque = DriveTorque;
                RearRightWheelCollider.motorTorque = DriveTorque;
            }
            else if(driveType==DriveType.IFWD) // FWD with independent actuators
            {
                FrontLeftWheelCollider.brakeTorque = 0;
                FrontRightWheelCollider.brakeTorque = 0;
                RearLeftWheelCollider.brakeTorque = 0;
                RearRightWheelCollider.brakeTorque = 0;
                FrontLeftWheelCollider.motorTorque = DriveTorque;
                FrontRightWheelCollider.motorTorque = DriveTorque;
            }
            else if(driveType==DriveType.IAWD) // AWD with independent actuators
            {
                FrontLeftWheelCollider.brakeTorque = 0;
                FrontRightWheelCollider.brakeTorque = 0;
                RearLeftWheelCollider.brakeTorque = 0;
                RearRightWheelCollider.brakeTorque = 0;
                FrontLeftWheelCollider.motorTorque = DriveTorque;
                FrontRightWheelCollider.motorTorque = DriveTorque;
                RearLeftWheelCollider.motorTorque = DriveTorque;
                RearRightWheelCollider.motorTorque = DriveTorque;
            }
            else if(driveType==DriveType.CRWD)  // FWD with common actuator
            {
                FrontLeftWheelCollider.brakeTorque = 0;
                FrontRightWheelCollider.brakeTorque = 0;
                RearLeftWheelCollider.brakeTorque = 0;
                RearRightWheelCollider.brakeTorque = 0;
                RearLeftWheelCollider.motorTorque = DriveTorque/2;
                RearRightWheelCollider.motorTorque = DriveTorque/2;
            }
            else if(driveType==DriveType.CFWD) // FWD with common actuator
            {
                FrontLeftWheelCollider.brakeTorque = 0;
                FrontRightWheelCollider.brakeTorque = 0;
                RearLeftWheelCollider.brakeTorque = 0;
                RearRightWheelCollider.brakeTorque = 0;
                FrontLeftWheelCollider.motorTorque = DriveTorque/2;
                FrontRightWheelCollider.motorTorque = DriveTorque/2;
            }
            else if(driveType==DriveType.CAWD) // AWD with common actuator
            {
                FrontLeftWheelCollider.brakeTorque = 0;
                FrontRightWheelCollider.brakeTorque = 0;
                RearLeftWheelCollider.brakeTorque = 0;
                RearRightWheelCollider.brakeTorque = 0;
                FrontLeftWheelCollider.motorTorque = DriveTorque/4;
                FrontRightWheelCollider.motorTorque = DriveTorque/4;
                RearLeftWheelCollider.motorTorque = DriveTorque/4;
                RearRightWheelCollider.motorTorque = DriveTorque/4;
            }
        }
  	}

    private void ExtendedDifferentialDrive()
  	{
        //Debug.Log("RPM: " + (RearLeftWheelCollider.rpm + RearRightWheelCollider.rpm)/2); // Average wheel speed (RPM)
        //Debug.Log("Speed: " + (Mathf.PI*0.065)*((RearLeftWheelCollider.rpm + RearRightWheelCollider.rpm)/120)); // Average vehicle speed (m/s)

        if (DrivingMode == 0) linearVelocity = ThrottleInput; // Manual Driving
        else linearVelocity = AutonomousThrottle; // Autonomous Driving

        if (DrivingMode == 0) angularVelocity = SteeringInput; // Manual Driving
        else angularVelocity = AutonomousSteering; // Autonomous Driving

        if(linearVelocity == 0 && angularVelocity ==0)
        {
            RearLeftWheelCollider.motorTorque = 0;
            RearRightWheelCollider.motorTorque = 0;
            RearLeftWheelCollider.brakeTorque = MotorTorque;
            RearRightWheelCollider.brakeTorque = MotorTorque;
            FrontLeftWheelCollider.motorTorque = 0;
            FrontRightWheelCollider.motorTorque = 0;
            FrontLeftWheelCollider.brakeTorque = MotorTorque;
            FrontRightWheelCollider.brakeTorque = MotorTorque;
        }
        else
        {
            FrontLeftWheelCollider.brakeTorque = 0;
            FrontRightWheelCollider.brakeTorque = 0;
            RearLeftWheelCollider.brakeTorque = 0;
            RearRightWheelCollider.brakeTorque = 0;
            FrontLeftWheelCollider.motorTorque = ((LinearGain*2*linearVelocity) - (AngularGain*angularVelocity*TrackWidth))/(2*WheelRadius);
            FrontRightWheelCollider.motorTorque = ((LinearGain*2*linearVelocity) + (AngularGain*angularVelocity*TrackWidth))/(2*WheelRadius);
            RearLeftWheelCollider.motorTorque = ((LinearGain*2*linearVelocity) - (AngularGain*angularVelocity*TrackWidth))/(2*WheelRadius);
            RearRightWheelCollider.motorTorque = ((LinearGain*2*linearVelocity) + (AngularGain*angularVelocity*TrackWidth))/(2*WheelRadius);
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
            if(driveType!=DriveType.SkidSteer)
            {
                    Steer(); // Do not steer like a car for skid-steer configuration
                    Drive(); // Do not drive like a car for skid-steer configuration
            }
            else ExtendedDifferentialDrive();
    		UpdateWheelPoses();
  	}
}
