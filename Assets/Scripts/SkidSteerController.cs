using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SkidSteerController : MonoBehaviour
{
    /*
    This script actuates the drive actuators in order to control
    the motion of the vehicle in a skid-steer configuration.
    */

    private float ThrottleInput;
    private float SteeringInput;

    public GameObject Vehicle;
    public WheelCollider FrontLeftWheelCollider, FrontRightWheelCollider;
    public WheelCollider RearLeftWheelCollider, RearRightWheelCollider;
    public Transform FrontLeftWheelTransform, FrontRightWheelTransform;
    public Transform RearLeftWheelTransform, RearRightWheelTransform;
    public float trackWidth = 141.54f; // mm
    public float linearGain = 6.8f;
    public float angularGain = 0.008f;
    public float MotorTorque = 2.352f; // N-m
    public float linVelLimit = 0.26f; // m/s
    public float angVelLimit = 0.42f; // rad/s
    [Range(-1,1)] public float AutonomousThrottle = 0;
    [Range(-1,1)] public float AutonomousSteering = 0;
    public int DrivingMode = 1; // Driving mode: 0 is manual, 1 is autonomous
    private float angularVelocity = 0; // deg
    private float linearVelocity = 0; // deg
    private bool MouseHold;
		private float MouseStart;

    public int CurrentDrivingMode
    {
        get { return DrivingMode; }
    }

    public float CurrentThrottle
    {
        get { return linearVelocity; }
        set { AutonomousThrottle = value; }
    }

    public float CurrentSteeringAngle
    {
        get { return angularVelocity; }
        set { AutonomousSteering = value; }
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
            SteeringInput = Mathf.Clamp((MousePosition - MouseStart)/(Screen.width/6), -1, 1); // Clamp the steering command in range of [-1, 1]
        }

        // Discrete control using keyboard
        else
        {
            MouseHold = false; // The mouse button is released
            SteeringInput = Input.GetAxis("Horizontal");
        }
  	}

    /*
  	private void Steer()
  	{
        if(DrivingMode == 0) SteeringAngle = SteeringInput*angVelLimit; // Manual Driving
        else SteeringAngle = AutonomousSteering* angVelLimit; // Autonomous Driving
        //Debug.Log("Steering Angle: " + SteeringAngle);
        //Debug.Log("Left Wheel Angle: " + Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))/((2*Wheelbase)+(KPI*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))))));
        //Debug.Log("Right Wheel Angle: " + Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))/((2*Wheelbase)-(KPI*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))))));
    	//FrontLeftWheelCollider.steerAngle = Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))/((2*Wheelbase)+(KPI*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle))))));
    	//FrontRightWheelCollider.steerAngle = Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle)))/((2*Wheelbase)-(KPI*Mathf.Tan(Mathf.Deg2Rad*(SteeringAngle))))));
  	}
    */

    private void ExtendedDifferentialDrive()
  	{
        //Debug.Log("RPM: " + (RearLeftWheelCollider.rpm + RearRightWheelCollider.rpm)/2); // Average wheel speed (RPM)
        //Debug.Log("Speed: " + (Mathf.PI*0.065)*((RearLeftWheelCollider.rpm + RearRightWheelCollider.rpm)/120)); // Average vehicle speed (m/s)

        if (DrivingMode == 0) linearVelocity = ThrottleInput*linVelLimit; // Manual Driving
        else linearVelocity = AutonomousThrottle*linVelLimit; // Autonomous Driving

        if (DrivingMode == 0) angularVelocity = SteeringInput*angVelLimit; // Manual Driving
        else angularVelocity = AutonomousSteering*angVelLimit; // Autonomous Driving

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
            FrontLeftWheelCollider.motorTorque = linearGain*linearVelocity - (angularGain*angularVelocity*trackWidth);
            FrontRightWheelCollider.motorTorque = linearGain*linearVelocity + (angularGain*angularVelocity*trackWidth);
            RearLeftWheelCollider.motorTorque = linearGain*linearVelocity - (angularGain*angularVelocity*trackWidth);
            RearRightWheelCollider.motorTorque = linearGain*linearVelocity + (angularGain*angularVelocity*trackWidth);
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
            //Steer();
            ExtendedDifferentialDrive();
    		UpdateWheelPoses();
  	}
}
