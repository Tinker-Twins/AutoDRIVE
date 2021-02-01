using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VehicleLighting : MonoBehaviour
{
    /*
    This script controls the vehicle lighting. The designed vehicle has the
    following functional lights and indicators:

    1. Headlights: Switched between 3 states, viz. OFF, Low-Beam and High-Beam.
    2. Taillights: Automatically switched between OFF, Partially ON (when Headlights are switched ON) and ON (when brakes are applied).
    3. Turn Indicators: Left and Right Turn Indicators toggled ON and OFF.
    5. ReverseIndicators: Automatically toggled ON and OFF depending on the velocity of the driven wheels.
    4. Hazard Indicators: Toggled ON and OFF.
    */

    public VehicleTeleoperation VehicleTeleoperation;

    private float timer = 0f;

    // HEADLIGHTS
    private bool HeadlightsLowBeam = false;
    private bool HeadlightsHighBeam = false;
    public Renderer HeadlightLeft1;
    public Renderer HeadlightLeft2;
    public Renderer HeadlightRight1;
    public Renderer HeadlightRight2;
    public Material HeadlightOFF;
    public Material HeadlightON;

    // TAILLIGHTS
    private bool Taillights = false;
    public Renderer TaillightLeft;
    public Renderer TaillightRight;
    public Material TaillightOFF;
    public Material TaillightPartiallyON;
    public Material TaillightON;

    // TURN INDICATORS
    private bool LeftTurnIndicators = false;
    bool RightTurnIndicators = false;
    bool HazardIndicators = false;
    public Renderer TurnIndicatorFrontLeft;
    public Renderer TurnIndicatorFrontRight;
    public Renderer TurnIndicatorRearLeft;
    public Renderer TurnIndicatorRearRight;
    public Material TurnIndicatorOFF;
    public Material TurnIndicatorON;

    // REVERSE INDICATORS
    private bool ReverseIndicators = false;
    public Renderer ReverseIndicatorLeft;
    public Renderer ReverseIndicatorRight;
    public Material ReverseIndicatorOFF;
    public Material ReverseIndicatorON;

    // AUTONOMOUS CONTROL METHODS
    private int HeadlightState = 0; // 0 = Disabled, 1 = Low Beam, 2 = High Beam
    private int IndicatorState = 0; // 0 = Disabled, 1 = Left Turn Indicator, 2 = Right Turn Indicator, 3 = Hazard Indicators

    public int Headlights
    {
        get { return HeadlightState; }
        set { HeadlightState = value; }
    }

    public int Indicators
    {
        get { return IndicatorState; }
        set { IndicatorState = value; }
    }

    void Update()
    {
        // ============================================== //
        // DEFINE LOGIC TO SET LIGHT STATUS TO TRUE/FALSE //
        // ============================================== //

        // Light Control (Manual Mode)
        if(VehicleTeleoperation.CurrentDrivingMode == 0)
        {
            // Headlights - Low Beam
            if(Input.GetKeyDown(KeyCode.G))
            {
                HeadlightsHighBeam = false;
                HeadlightsLowBeam = !HeadlightsLowBeam;
            }

            // Headlights - High Beam
            if(Input.GetKeyDown(KeyCode.H))
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = !HeadlightsHighBeam;
            }

            // Indicators - Left Turn Indicators
            if(Input.GetKeyDown(KeyCode.L))
            {
                RightTurnIndicators = false;
                HazardIndicators = false;
                LeftTurnIndicators = !LeftTurnIndicators;
            }

            // Indicators - Right Turn Indicators
            if(Input.GetKeyDown(KeyCode.R))
            {
                LeftTurnIndicators = false;
                HazardIndicators = false;
                RightTurnIndicators = !RightTurnIndicators;
            }

            // Indicators - Hazard Indicators
            if(Input.GetKeyDown(KeyCode.E))
            {
                LeftTurnIndicators = false;
                RightTurnIndicators = false;
                HazardIndicators = !HazardIndicators;
            }
        }

        // Light Control (Autonomous Mode)
        if(VehicleTeleoperation.CurrentDrivingMode == 1)
        {
            // Headlights - Disabled
            if(HeadlightState == 0)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = false;
            }

            // Headlights - Low Beam
            if(HeadlightState == 1)
            {
                HeadlightsHighBeam = false;
                HeadlightsLowBeam = true;
            }

            // Headlights - High Beam
            if(HeadlightState == 2)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = true;
            }

            // Indicators - Disabled
            if(Indicators == 0)
            {
                RightTurnIndicators = false;
                HazardIndicators = false;
                LeftTurnIndicators = false;
            }

            // Indicators - Left Turn Indicators
            if(Indicators == 1)
            {
                RightTurnIndicators = false;
                HazardIndicators = false;
                LeftTurnIndicators = true;
            }

            // Indicators - Right Turn Indicators
            if(Indicators == 2)
            {
                LeftTurnIndicators = false;
                HazardIndicators = false;
                RightTurnIndicators = true;
            }

            // Indicators - Hazard Indicators
            if(Indicators == 3)
            {
                LeftTurnIndicators = false;
                RightTurnIndicators = false;
                HazardIndicators = true;
            }
        }

        // Automatic Light Control

        // Brake Lights
        if(System.Math.Round((VehicleTeleoperation.RearLeftWheelCollider.rpm + VehicleTeleoperation.RearRightWheelCollider.rpm)/2, 2) == 0)
        {
            Taillights = true;
        }
        else
        {
            Taillights = false;
        }

        // Reverse Indicators
        if(System.Math.Round((VehicleTeleoperation.RearLeftWheelCollider.rpm + VehicleTeleoperation.RearRightWheelCollider.rpm)/2, 1) < 0)
        {
            ReverseIndicators = true;
        }
        else
        {
            ReverseIndicators = false;
        }

        // =================================== //
        // RENDER LIGHTS BASED ON THEIR STATUS //
        // =================================== //

        // Headlights - Low Beam
        if(HeadlightsLowBeam)
        {
            HeadlightLeft1.material = HeadlightON;
            HeadlightLeft1.GetComponent<Light>().enabled = true;
            HeadlightLeft2.material = HeadlightOFF;
            HeadlightLeft2.GetComponent<Light>().enabled = false;
            HeadlightRight1.material = HeadlightON;
            HeadlightRight1.GetComponent<Light>().enabled = true;
            HeadlightRight2.material = HeadlightOFF;
            HeadlightRight2.GetComponent<Light>().enabled = false;
        }

        // Headlights - High Beam
        else if(HeadlightsHighBeam)
        {
            HeadlightLeft1.material = HeadlightON;
            HeadlightLeft1.GetComponent<Light>().enabled = true;
            HeadlightLeft2.material = HeadlightON;
            HeadlightLeft2.GetComponent<Light>().enabled = true;
            HeadlightRight1.material = HeadlightON;
            HeadlightRight1.GetComponent<Light>().enabled = true;
            HeadlightRight2.material = HeadlightON;
            HeadlightRight2.GetComponent<Light>().enabled = true;
        }

        else
        {
            HeadlightLeft1.material = HeadlightOFF;
            HeadlightLeft1.GetComponent<Light>().enabled = false;
            HeadlightLeft2.material = HeadlightOFF;
            HeadlightLeft2.GetComponent<Light>().enabled = false;
            HeadlightRight1.material = HeadlightOFF;
            HeadlightRight1.GetComponent<Light>().enabled = false;
            HeadlightRight2.material = HeadlightOFF;
            HeadlightRight2.GetComponent<Light>().enabled = false;
        }

        // Brake Lights
        if(Taillights)
        {
            TaillightLeft.material = TaillightON;
            TaillightLeft.GetComponent<Light>().intensity = 1f;
            TaillightLeft.GetComponent<Light>().enabled = true;
            TaillightRight.material = TaillightON;
            TaillightRight.GetComponent<Light>().intensity = 1f;
            TaillightRight.GetComponent<Light>().enabled = true;
        }
        else
        {
            if(HeadlightsLowBeam || HeadlightsHighBeam)
            {
                TaillightLeft.material = TaillightPartiallyON;
                TaillightLeft.GetComponent<Light>().intensity = 0.5f;
                TaillightLeft.GetComponent<Light>().enabled = true;
                TaillightRight.material = TaillightPartiallyON;
                TaillightRight.GetComponent<Light>().intensity = 0.5f;
                TaillightRight.GetComponent<Light>().enabled = true;
            }
            else
            {
                TaillightLeft.material = TaillightOFF;
                TaillightLeft.GetComponent<Light>().enabled = false;
                TaillightRight.material = TaillightOFF;
                TaillightRight.GetComponent<Light>().enabled = false;
            }
        }

        // Left Turn Indicators
        if(LeftTurnIndicators)
        {
            timer = timer + Time.deltaTime;
            if(timer >= 0.5)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorON;
                TurnIndicatorFrontLeft.GetComponent<Light>().enabled = true;
                TurnIndicatorRearLeft.material = TurnIndicatorON;
                TurnIndicatorRearLeft.GetComponent<Light>().enabled = true;
                TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                TurnIndicatorFrontRight.GetComponent<Light>().enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorOFF;
                TurnIndicatorRearRight.GetComponent<Light>().enabled = false;
            }
            if(timer >= 1)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                TurnIndicatorFrontLeft.GetComponent<Light>().enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                TurnIndicatorRearLeft.GetComponent<Light>().enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                TurnIndicatorFrontRight.GetComponent<Light>().enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorOFF;
                TurnIndicatorRearRight.GetComponent<Light>().enabled = false;
                timer = 0;
            }
        }

        // Right Turn Indicators
        else if(RightTurnIndicators)
        {
            timer = timer + Time.deltaTime;
            if(timer >= 0.5)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                TurnIndicatorFrontLeft.GetComponent<Light>().enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                TurnIndicatorRearLeft.GetComponent<Light>().enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorON;
                TurnIndicatorFrontRight.GetComponent<Light>().enabled = true;
                TurnIndicatorRearRight.material = TurnIndicatorON;
                TurnIndicatorRearRight.GetComponent<Light>().enabled = true;
            }
            if(timer >= 1)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                TurnIndicatorFrontLeft.GetComponent<Light>().enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                TurnIndicatorRearLeft.GetComponent<Light>().enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                TurnIndicatorFrontRight.GetComponent<Light>().enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorOFF;
                TurnIndicatorRearRight.GetComponent<Light>().enabled = false;
                timer = 0;
            }
        }

        // Hazard Indicators
        else if(HazardIndicators)
        {
            timer = timer + Time.deltaTime;
            if(timer >= 0.5)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorON;
                TurnIndicatorFrontLeft.GetComponent<Light>().enabled = true;
                TurnIndicatorRearLeft.material = TurnIndicatorON;
                TurnIndicatorRearLeft.GetComponent<Light>().enabled = true;
                TurnIndicatorFrontRight.material = TurnIndicatorON;
                TurnIndicatorFrontRight.GetComponent<Light>().enabled = true;
                TurnIndicatorRearRight.material = TurnIndicatorON;
                TurnIndicatorRearRight.GetComponent<Light>().enabled = true;
            }
            if(timer >= 1)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                TurnIndicatorFrontLeft.GetComponent<Light>().enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                TurnIndicatorRearLeft.GetComponent<Light>().enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                TurnIndicatorFrontRight.GetComponent<Light>().enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorOFF;
                TurnIndicatorRearRight.GetComponent<Light>().enabled = false;
                timer = 0;
            }
        }

        else
        {
            TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
            TurnIndicatorFrontLeft.GetComponent<Light>().enabled = false;
            TurnIndicatorRearLeft.material = TurnIndicatorOFF;
            TurnIndicatorRearLeft.GetComponent<Light>().enabled = false;
            TurnIndicatorFrontRight.material = TurnIndicatorOFF;
            TurnIndicatorFrontRight.GetComponent<Light>().enabled = false;
            TurnIndicatorRearRight.material = TurnIndicatorOFF;
            TurnIndicatorRearRight.GetComponent<Light>().enabled = false;
        }

        // Reverse Indicators
        if(ReverseIndicators)
        {
            ReverseIndicatorLeft.material = ReverseIndicatorON;
            ReverseIndicatorLeft.GetComponent<Light>().enabled = true;
            ReverseIndicatorRight.material = ReverseIndicatorON;
            ReverseIndicatorRight.GetComponent<Light>().enabled = true;
        }
        else
        {
          ReverseIndicatorLeft.material = ReverseIndicatorOFF;
          ReverseIndicatorLeft.GetComponent<Light>().enabled = false;
          ReverseIndicatorRight.material = ReverseIndicatorOFF;
          ReverseIndicatorRight.GetComponent<Light>().enabled = false;
        }
    }
}
