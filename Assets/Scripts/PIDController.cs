using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PIDController : MonoBehaviour
{
    /*
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    */

    // PID controller properties
    public float kP { get; private set; } // Proportional gain
    public float kI { get; private set; } // Integral gain
    public float kD { get; private set; } // Derivative gain
    public int kS   { get; private set; } // Saturation constant (error history buffer size)

    private float errInt;           // Error integral
    private float errDif;           // Error difference
    private float errPrev;          // Previous error
    private Queue<float> errHist;   // Limited buffer of error history
    private float tPrev;            // Previous time

    public PIDController(float KP, float KI, float KD, int KS)
    {
        kP      = KP;   // Proportional gain
        kI      = KI;   // Integral gain
        kD      = KD;   // Derivative gain
        kS      = KS;   // Saturation constant (error history buffer size)
        errInt  = 0;    // Error integral
        errDif  = 0;    // Error difference
        errPrev = 0;    // Previous error
        tPrev   = 0;    // Previous time
        errHist = new Queue<float>(kS); // Limited error history buffer
    }

    public float Control(float err, float t)
    {
        /*
        Generate PID controller output.
        :param err: Instantaneous error in control variable w.r.t. setpoint
        :param t  : Current timestamp
        :return u : PID controller output
        */
        float dt = t - tPrev; // Timestep
        if (dt > 0.0f)
        {
            errHist.Enqueue(err); // Update error history
            errInt += err; // Integrate error
            if (errHist.Count > kS) // Jacketing logic to prevent integral windup
            {
                errInt -= errHist.Dequeue(); // Rolling FIFO buffer
            }
            errDif = (err - errPrev); // Error difference
            float u = (kP * err) + (kI * errInt * dt) + (kD * errDif / dt); // PID control law
            errPrev = err; // Update previous error term
            tPrev = t; // Update timestamp
            return u; // Control signal
        }
        else
        {
            Debug.Log("Non-positive timestep detected!");
            return 0.0f;
        }
    }

    public void Reset()
    {
        // Reset all PID controller variables to initial state
        errInt = 0;
        errDif = 0;
        errPrev = 0;
        tPrev = 0;
        errHist.Clear();
    }
}
