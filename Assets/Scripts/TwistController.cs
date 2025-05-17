using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TwistController : MonoBehaviour
{
    /*
    This script defines a parallel feedback control loop to track a reference twist
    (linear and angular velocity) by driving and steering the vehicle.
    */

    // Vehicle
    [Header("Vehicle")]
    public VehicleController VehicleController;
    public IMU IMU;

    // Linear velocity controller
    private PIDController vPID; // PID controller
    [Header("Linear Velocity Controller")]
    public bool vEnable = false; // Enable
    public float KPv = 0f; // Proportional gain
    public float KIv = 0f; // Integral gain
    public float KDv = 0f; // Derivative gain
    public int KSv = 100; // Saturation constant
    public float vSetpoint = 0f; // Desired setpoint (target value)
    private float vFeedback = 0f; // Actual measurement/estimate (feedback)
    private float vError = 0f; // Current error (setpoint - feedback)
    private float vControl = 0f; // Control signal

    // Angular velocity controller
    private PIDController wPID; // PID controller
    [Header("Angular Velocity Controller")]
    public bool wEnable = false; // Enable
    public float KPw = 0f; // Proportional gain
    public float KIw = 0f; // Integral gain
    public float KDw = 0f; // Derivative gain
    public int KSw = 100; // Saturation constant
    public float wSetpoint = 0f; // Desired setpoint (target value)
    private float wFeedback = 0f; // Actual measurement/estimate (feedback)
    private float wError = 0f; // Current error (setpoint - feedback)
    private float wControl = 0f; // Control signal

    // Start is called before the first frame update
    void Start()
    {
        if (vEnable) vPID = new PIDController(KPv, KIv, KDv, KSv); // Linear velocity controller
        if (wEnable) wPID = new PIDController(KPw, KIw, KDw, KSw); // Angular velocity controller
    }

    // Update is called once per frame
    void Update()
    {
        // Linear velocity control
        if (vEnable)
        {
            if (vSetpoint != 0.0f)
            {
                vFeedback = IMU.CurrentLinearVelocity[0]; // Feedback
                vError = vSetpoint - vFeedback; // Error
                // Debug.Log("V Error: " + vError);
                vControl = vPID.Control(vError, Time.time); // Control
                // Debug.Log("V Control: " + vControl);
                VehicleController.CurrentThrottle = Mathf.Clamp(vControl, -1.0f, 1.0f); // Throttle
            }
            else VehicleController.CurrentThrottle = 0.0f; // Throttle
        }

        // Angular velocity control
        if (wEnable)
        {
            if (wSetpoint != 0.0f)
            {
                wFeedback = IMU.CurrentAngularVelocity[2]; // Feedback
                wError = wSetpoint - wFeedback; // Error
                // Debug.Log("W Error: " + wError);
                wControl = wPID.Control(wError, Time.time); // Control
                // Debug.Log("W Control: " + wControl);
                VehicleController.CurrentSteeringAngle = Mathf.Clamp(wControl, -1.0f, 1.0f); // Steering
            }
            else VehicleController.CurrentSteeringAngle = 0.0f; // Steering
        }
    }
}
