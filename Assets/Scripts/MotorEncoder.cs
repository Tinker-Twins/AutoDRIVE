using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MotorEncoder : MonoBehaviour
{
    /*
    This script attaches an incrementel encoder to multiple 'WheelCollider' components.
    The individual encoder readings are passed through an averaging filter for estimation.
    The property `PPR` sets the encoder resolution while `GearRatio` sets the multiplier
    due to the motor gearbox. The two parameters are used to compute the encoder ticks and
    the angular displacement.
    */

    public WheelCollider[] Wheels;
    public int PPR;
    public int GearRatio;

    private float MotorRPM = 0f;
    private float MotorRPS = 0f;
    public float TotalRevolutions = 0f;
    private int TotalTicks = 0;
    private float TotalAngle = 0;

    public int Ticks
    {
        get { return TotalTicks; }
    }

    public float Angle
    {
        get { return TotalAngle; }
    }

    public float RPM
    {
        get
        { return MotorRPM; }
    }

    void FixedUpdate()
    {
        // ENCODER TICKS
        MotorRPM = 0;
        foreach (WheelCollider Wheel in Wheels)
        {
            MotorRPM += Wheel.rpm; // Read the current wheel RPM
        }
        MotorRPM = MotorRPM/Wheels.Length; // Differential average
        MotorRPS = MotorRPM/60f; // Convert to RPS
        TotalRevolutions += MotorRPS * Time.deltaTime; // Scale by time since the last frame and add to the total revolutions
        TotalTicks = (int)(TotalRevolutions*PPR*GearRatio); // Compute ticks of the encoder
        // Debug.Log("Encoder Ticks: " + TotalTicks);

        // ANGULAR DISPLACEMENT
        TotalAngle = ((TotalTicks*2*Mathf.PI)/(PPR*GearRatio)); // Angle turned by the wheel (rad)
        // Debug.Log("Angular Displacement: " + TotalAngle);
    }
}
