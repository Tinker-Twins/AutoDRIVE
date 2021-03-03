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
    public int GearRatio;

    private float TotalRevolutions = 0f;
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

    void FixedUpdate()
    {
        float RPS = Wheel.rpm/60f; // Read the current wheel RPM and convert to RPS
        TotalRevolutions += RPS * Time.deltaTime; // Scale by time since the last frame and add to the total revolutions
        TotalTicks = (int)(TotalRevolutions*PPR*GearRatio); // Compute ticks of the encoder
        TotalAngle = ((TotalTicks*2*Mathf.PI)/(PPR*GearRatio)); // Angle turned by the wheel (rad)
        //Debug.Log("Encoder Ticks: "+TotalTicks);
        //Debug.Log("Wheel Angle: "+TotalAngle);
    }
}
