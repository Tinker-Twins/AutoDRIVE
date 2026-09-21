using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelEncoder : MonoBehaviour
{
    /*
    This script attaches an incrementel encoder to a 'WheelCollider' component.
    The property `PPR` sets the encoder resolution while `GearRatio` sets the
    multiplier due to the motor gearbox. The two parameters are used to compute
    the encoder ticks and the angle turned by the wheel.
    */

    public WheelCollider Wheel;
    public int PPR;
    public float GearRatio;

    private float RPS = 0f;
    public float TotalRevolutions = 0f;
    private int TotalTicks = 0;
    private int PrevTotalTicks = 0;
    private float TotalAngle = 0;

    private float VelocityFromRPM = 0;

    private float VelocityFromTicks = 0;
    private Queue VelocityBuffer = new Queue();
    private int VelocityBufferLimit = 5;
    private float VelocitySum = 0;

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
        get { return Wheel.rpm; }
    }

    public float SpeedFromRPM
    {
        get { return VelocityFromRPM; }
    }

    public float SpeedFromTicks
    {
        get { return VelocityFromTicks; }
    }

    void FixedUpdate()
    {
        // ENCODER TICKS
        RPS = Wheel.rpm/60f; // Read the current wheel RPM and convert to RPS
        TotalRevolutions += RPS * Time.deltaTime; // Scale by time since the last frame and add to the total revolutions
        TotalTicks = (int)(TotalRevolutions*PPR*GearRatio); // Compute ticks of the encoder
        // Debug.Log("Encoder Ticks: " + TotalTicks);

        // WHEEL ANGLE
        TotalAngle = ((TotalTicks*2*Mathf.PI)/(PPR*GearRatio)); // Angle turned by the wheel (rad)
        // Debug.Log("Wheel Angle: " + TotalAngle);

        // VELOCITY FROM WHEEL SPEED (RPM)
        VelocityFromRPM = (2f*Mathf.PI*Wheel.radius*RPS);
        // Debug.Log("Velocity From RPM: " + VelocityFromRPM);

        // VELOCITY FROM ENCODER TICKS
        // Shifting Average Filter
        if (VelocityBuffer.Count >= VelocityBufferLimit)
        {
            VelocityBuffer.Dequeue();
        }
        VelocityBuffer.Enqueue(((TotalTicks - PrevTotalTicks)/Time.deltaTime)*((360f/(PPR*GearRatio))*(Mathf.PI/180f)*(Wheel.radius)));
        PrevTotalTicks = TotalTicks;
        VelocitySum = 0;
        foreach (float Velocity in VelocityBuffer)
        {
            VelocitySum += Velocity;
        }
        VelocityFromTicks = VelocitySum/VelocityBufferLimit;
        // Debug.Log("Velocity From Ticks: " + VelocityFromTicks);
    }
}