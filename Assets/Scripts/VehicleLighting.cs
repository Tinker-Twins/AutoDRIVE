using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VehicleLighting : MonoBehaviour
{
    /*
    This script controls the vehicle lighting. The designed vehicle has the following functional lights and indicators:

    1. Headlights: Switched between 3 states, viz. OFF, Low-Beam and High-Beam.
    2. Taillights: Automatically switched between OFF, Partially ON (when Headlights are switched ON) and ON (when brakes are applied).
    3. Turn Indicators: Left and Right Turn Indicators toggled ON and OFF.
    5. ReverseIndicators: Automatically toggled ON and OFF depending on the velocity of the driven wheels.
    4. Hazard Indicators: Toggled ON and OFF.
    */

    public VehicleController VehicleController;
    public DataRecorder DataRecorder;
    [HideInInspector] public float RecordedVelocity;

    private float timer = 0f;

    // HEADLIGHTS
    private bool HeadlightsLowBeam = false;
    private bool HeadlightsHighBeam = false;
    public Renderer HeadlightLeft1;
    public Renderer HeadlightLeft2;
    public Renderer HeadlightRight1;
    public Renderer HeadlightRight2;
    public Light SpotLightHeadlightLeft1;
    public Light PointLightHeadlightLeft1;
    public Light PointLightHeadlightLeft2;
    public Light SpotLightHeadlightRight1;
    public Light PointLightHeadlightRight1;
    public Light PointLightHeadlightRight2;
    public Material HeadlightOFF;
    public Material HeadlightON;

    // TAILLIGHTS
    private bool Taillights = false;
    public float TailLightIntensity = 0.05f;
    public Renderer TaillightLeft;
    public Renderer TaillightRight;
    public Renderer TaillightLeft2;
    public Renderer TaillightRight2;
    public Renderer TaillightLeft3;
    public Renderer TaillightRight3;
    public Light PointLightTaillightLeft;
    public Light PointLightTaillightRight;
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
    public Renderer TurnIndicatorFrontLeft2;
    public Renderer TurnIndicatorFrontRight2;
    public Renderer TurnIndicatorRearLeft2;
    public Renderer TurnIndicatorRearRight2;
    public Light PointLightTurnIndicatorFrontLeft;
    public Light PointLightTurnIndicatorFrontRight;
    public Light PointLightTurnIndicatorRearLeft;
    public Light PointLightTurnIndicatorRearRight;
    public Material TurnIndicatorOFF;
    public Material TurnIndicatorON;

    // REVERSE INDICATORS
    private bool ReverseIndicators = false;
    public Renderer ReverseIndicatorLeft;
    public Renderer ReverseIndicatorRight;
    public Light PointLightReverseIndicatorLeft;
    public Light PointLightReverseIndicatorRight;
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
        if(VehicleController.CurrentDrivingMode == 0)
        {
            // Headlights - Low Beam
            if(Input.GetKeyDown(KeyCode.K))
            {
                HeadlightsHighBeam = false;
                HeadlightsLowBeam = !HeadlightsLowBeam;
            }

            // Headlights - High Beam
            if(Input.GetKeyDown(KeyCode.I))
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = !HeadlightsHighBeam;
            }

            // Indicators - Left Turn Indicators
            if(Input.GetKeyDown(KeyCode.J))
            {
                RightTurnIndicators = false;
                HazardIndicators = false;
                LeftTurnIndicators = !LeftTurnIndicators;
            }

            // Indicators - Right Turn Indicators
            if(Input.GetKeyDown(KeyCode.L))
            {
                LeftTurnIndicators = false;
                HazardIndicators = false;
                RightTurnIndicators = !RightTurnIndicators;
            }

            // Indicators - Hazard Indicators
            if(Input.GetKeyDown(KeyCode.M))
            {
                LeftTurnIndicators = false;
                RightTurnIndicators = false;
                HazardIndicators = !HazardIndicators;
            }
        }

        // Light Control (Autonomous Mode)
        if(VehicleController.CurrentDrivingMode == 1)
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
        if (DataRecorder.getSaveStatus())
        {
            // Brake Lights
            if(System.Math.Round(RecordedVelocity,2) == 0)
            {
                Taillights = true;
            }
            else
            {
                Taillights = false;
            }

            // Reverse Indicators
            if(System.Math.Round(RecordedVelocity,2) < 0)
            {
                ReverseIndicators = true;
            }
            else
            {
                ReverseIndicators = false;
            }
        }
        else
        {
            // Brake Lights
            if(System.Math.Round(VehicleController.Vehicle.transform.InverseTransformDirection(VehicleController.Vehicle.GetComponent<Rigidbody>().velocity).z,2) == 0)
            {
                Taillights = true;
            }
            else
            {
                Taillights = false;
            }

            // Reverse Indicators
            if(System.Math.Round(VehicleController.Vehicle.transform.InverseTransformDirection(VehicleController.Vehicle.GetComponent<Rigidbody>().velocity).z,2) < 0)
            {
                ReverseIndicators = true;
            }
            else
            {
                ReverseIndicators = false;
            }
        }

        // =================================== //
        // RENDER LIGHTS BASED ON THEIR STATUS //
        // =================================== //

        // Headlights - Low Beam
        if(HeadlightsLowBeam)
        {
            if (HeadlightLeft1 != null) HeadlightLeft1.material = HeadlightON;
            if (SpotLightHeadlightLeft1 != null) SpotLightHeadlightLeft1.enabled = true;
            if (PointLightHeadlightLeft1 != null) PointLightHeadlightLeft1.enabled = true;
            if (HeadlightLeft2 != null)HeadlightLeft2.material = HeadlightOFF;
            if (PointLightHeadlightLeft2 != null) PointLightHeadlightLeft2.enabled = false;
            if (HeadlightRight1 != null) HeadlightRight1.material = HeadlightON;
            if (SpotLightHeadlightRight1 != null) SpotLightHeadlightRight1.enabled = true;
            if (PointLightHeadlightRight1 != null) PointLightHeadlightRight1.enabled = true;
            if (HeadlightRight2 != null) HeadlightRight2.material = HeadlightOFF;
            if (PointLightHeadlightRight2 != null) PointLightHeadlightRight2.enabled = false;
        }

        // Headlights - High Beam
        else if(HeadlightsHighBeam)
        {
            if (HeadlightLeft1 != null) HeadlightLeft1.material = HeadlightON;
            if (SpotLightHeadlightLeft1 != null) SpotLightHeadlightLeft1.enabled = true;
            if (PointLightHeadlightLeft1 != null) PointLightHeadlightLeft1.enabled = true;
            if (HeadlightLeft2 != null) HeadlightLeft2.material = HeadlightON;
            if (PointLightHeadlightLeft2 != null) PointLightHeadlightLeft2.enabled = true;
            if (HeadlightRight1 != null) HeadlightRight1.material = HeadlightON;
            if (SpotLightHeadlightRight1 != null) SpotLightHeadlightRight1.enabled = true;
            if (PointLightHeadlightRight1 != null) PointLightHeadlightRight1.enabled = true;
            if (HeadlightRight2 != null) HeadlightRight2.material = HeadlightON;
            if (PointLightHeadlightRight2 != null) PointLightHeadlightRight2.enabled = true;
        }

        else
        {
            if (HeadlightLeft1 != null) HeadlightLeft1.material = HeadlightOFF;
            if (SpotLightHeadlightLeft1 != null) SpotLightHeadlightLeft1.enabled = false;
            if (PointLightHeadlightLeft1 != null) PointLightHeadlightLeft1.enabled = false;
            if (HeadlightLeft2 != null) HeadlightLeft2.material = HeadlightOFF;
            if (PointLightHeadlightLeft2 != null) PointLightHeadlightLeft2.enabled = false;
            if (HeadlightRight1 != null) HeadlightRight1.material = HeadlightOFF;
            if (SpotLightHeadlightRight1 != null) SpotLightHeadlightRight1.enabled = false;
            if (PointLightHeadlightRight1 != null) PointLightHeadlightRight1.enabled = false;
            if (HeadlightRight2 != null) HeadlightRight2.material = HeadlightOFF;
            if (PointLightHeadlightRight2 != null) PointLightHeadlightRight2.enabled = false;
        }

        // Brake Lights
        if(Taillights)
        {
            if (TaillightLeft != null) TaillightLeft.material = TaillightON;
            if (TaillightLeft2 != null) TaillightLeft2.material = TaillightON;
            if (TaillightLeft3 != null) TaillightLeft3.material = TaillightON;
            PointLightTaillightLeft.intensity = TailLightIntensity;
            PointLightTaillightLeft.enabled = true;
            if (TaillightRight != null) TaillightRight.material = TaillightON;
            if (TaillightRight2 != null) TaillightRight2.material = TaillightON;
            if (TaillightRight3 != null) TaillightRight3.material = TaillightON;
            PointLightTaillightRight.intensity = TailLightIntensity;
            PointLightTaillightRight.enabled = true;
        }
        else
        {
            if(HeadlightsLowBeam || HeadlightsHighBeam)
            {
                if (TaillightLeft != null) TaillightLeft.material = TaillightPartiallyON;
                if (TaillightLeft2 != null) TaillightLeft2.material = TaillightPartiallyON;
                if (TaillightLeft3 != null) TaillightLeft3.material = TaillightPartiallyON;
                PointLightTaillightLeft.intensity = TailLightIntensity/2;
                PointLightTaillightLeft.enabled = true;
                if (TaillightRight != null) TaillightRight.material = TaillightPartiallyON;
                if (TaillightRight2 != null) TaillightRight2.material = TaillightPartiallyON;
                if (TaillightRight3 != null) TaillightRight3.material = TaillightPartiallyON;
                PointLightTaillightRight.intensity = TailLightIntensity/2;
                PointLightTaillightRight.enabled = true;
            }
            else
            {
                if (TaillightLeft != null) TaillightLeft.material = TaillightOFF;
                if (TaillightLeft2 != null) TaillightLeft2.material = TaillightOFF;
                if (TaillightLeft3 != null) TaillightLeft3.material = TaillightOFF;
                PointLightTaillightLeft.enabled = false;
                if (TaillightRight != null) TaillightRight.material = TaillightOFF;
                if (TaillightRight2 != null) TaillightRight2.material = TaillightOFF;
                if (TaillightRight3 != null) TaillightRight3.material = TaillightOFF;
                PointLightTaillightRight.enabled = false;
            }
        }

        // Left Turn Indicators
        if(LeftTurnIndicators)
        {
            timer = timer + Time.deltaTime;
            if(timer >= 0.5)
            {
                if (TurnIndicatorFrontLeft != null) TurnIndicatorFrontLeft.material = TurnIndicatorON;
                if (TurnIndicatorFrontLeft2 != null) TurnIndicatorFrontLeft2.material = TurnIndicatorON;
                PointLightTurnIndicatorFrontLeft.enabled = true;
                if (TurnIndicatorRearLeft != null) TurnIndicatorRearLeft.material = TurnIndicatorON;
                if (TurnIndicatorRearLeft2 != null) TurnIndicatorRearLeft2.material = TurnIndicatorON;
                PointLightTurnIndicatorRearLeft.enabled = true;
                if (TurnIndicatorFrontRight != null) TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontRight2 != null) TurnIndicatorFrontRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                if (TurnIndicatorRearRight != null) TurnIndicatorRearRight.material = TurnIndicatorOFF;
                if (TurnIndicatorRearRight2 != null) TurnIndicatorRearRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearRight.enabled = false;
            }
            if(timer >= 1)
            {
                if (TurnIndicatorFrontLeft != null) TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontLeft2 != null) TurnIndicatorFrontLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                if (TurnIndicatorRearLeft != null) TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorRearLeft2 != null) TurnIndicatorRearLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                if (TurnIndicatorFrontRight != null) TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontRight2 != null) TurnIndicatorFrontRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                if (TurnIndicatorRearRight != null) TurnIndicatorRearRight.material = TurnIndicatorOFF;
                if (TurnIndicatorRearRight2 != null) TurnIndicatorRearRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearRight.enabled = false;
                timer = 0;
            }
        }

        // Right Turn Indicators
        else if(RightTurnIndicators)
        {
            timer = timer + Time.deltaTime;
            if(timer >= 0.5)
            {
                if (TurnIndicatorFrontLeft != null) TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontLeft2 != null) TurnIndicatorFrontLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                if (TurnIndicatorRearLeft != null) TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorRearLeft2 != null) TurnIndicatorRearLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                if (TurnIndicatorFrontRight != null) TurnIndicatorFrontRight.material = TurnIndicatorON;
                if (TurnIndicatorFrontRight2 != null) TurnIndicatorFrontRight2.material = TurnIndicatorON;
                PointLightTurnIndicatorFrontRight.enabled = true;
                if (TurnIndicatorRearRight != null) TurnIndicatorRearRight.material = TurnIndicatorON;
                if (TurnIndicatorRearRight2 != null) TurnIndicatorRearRight2.material = TurnIndicatorON;
                PointLightTurnIndicatorRearRight.enabled = true;
            }
            if(timer >= 1)
            {
                if (TurnIndicatorFrontLeft != null) TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontLeft2 != null) TurnIndicatorFrontLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                if (TurnIndicatorRearLeft != null) TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorRearLeft2 != null) TurnIndicatorRearLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                if (TurnIndicatorFrontRight != null) TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontRight2 != null) TurnIndicatorFrontRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                if (TurnIndicatorRearRight != null) TurnIndicatorRearRight.material = TurnIndicatorOFF;
                if (TurnIndicatorRearRight2 != null) TurnIndicatorRearRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearRight.enabled = false;
                timer = 0;
            }
        }

        // Hazard Indicators
        else if(HazardIndicators)
        {
            timer = timer + Time.deltaTime;
            if(timer >= 0.5)
            {
                if (TurnIndicatorFrontLeft != null) TurnIndicatorFrontLeft.material = TurnIndicatorON;
                if (TurnIndicatorFrontLeft2 != null) TurnIndicatorFrontLeft2.material = TurnIndicatorON;
                PointLightTurnIndicatorFrontLeft.enabled = true;
                if (TurnIndicatorRearLeft != null) TurnIndicatorRearLeft.material = TurnIndicatorON;
                if (TurnIndicatorRearLeft2 != null) TurnIndicatorRearLeft2.material = TurnIndicatorON;
                PointLightTurnIndicatorRearLeft.enabled = true;
                if (TurnIndicatorFrontRight != null) TurnIndicatorFrontRight.material = TurnIndicatorON;
                if (TurnIndicatorFrontRight2 != null) TurnIndicatorFrontRight2.material = TurnIndicatorON;
                PointLightTurnIndicatorFrontRight.enabled = true;
                if (TurnIndicatorRearRight != null) TurnIndicatorRearRight.material = TurnIndicatorON;
                if (TurnIndicatorRearRight2 != null) TurnIndicatorRearRight2.material = TurnIndicatorON;
                PointLightTurnIndicatorRearRight.enabled = true;
            }
            if(timer >= 1)
            {
                if (TurnIndicatorFrontLeft != null) TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontLeft2 != null) TurnIndicatorFrontLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                if (TurnIndicatorRearLeft != null) TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorRearLeft2 != null) TurnIndicatorRearLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                if (TurnIndicatorFrontRight != null) TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontRight2 != null) TurnIndicatorFrontRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                if (TurnIndicatorRearRight != null) TurnIndicatorRearRight.material = TurnIndicatorOFF;
                if (TurnIndicatorRearRight2 != null) TurnIndicatorRearRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearRight.enabled = false;
                timer = 0;
            }
        }

        else
        {
            if (TurnIndicatorFrontLeft != null) TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontLeft2 != null) TurnIndicatorFrontLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                if (TurnIndicatorRearLeft != null) TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                if (TurnIndicatorRearLeft2 != null) TurnIndicatorRearLeft2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                if (TurnIndicatorFrontRight != null) TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                if (TurnIndicatorFrontRight2 != null) TurnIndicatorFrontRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                if (TurnIndicatorRearRight != null) TurnIndicatorRearRight.material = TurnIndicatorOFF;
                if (TurnIndicatorRearRight2 != null) TurnIndicatorRearRight2.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearRight.enabled = false;
        }

        // Reverse Indicators
        if(ReverseIndicators)
        {
            if (ReverseIndicatorLeft != null) ReverseIndicatorLeft.material = ReverseIndicatorON;
            if (PointLightReverseIndicatorLeft != null) PointLightReverseIndicatorLeft.enabled = true;
            if (ReverseIndicatorRight != null) ReverseIndicatorRight.material = ReverseIndicatorON;
            if (PointLightReverseIndicatorRight != null) PointLightReverseIndicatorRight.enabled = true;
        }
        else
        {
          if (ReverseIndicatorLeft != null) ReverseIndicatorLeft.material = ReverseIndicatorOFF;
          if (PointLightReverseIndicatorLeft != null) PointLightReverseIndicatorLeft.enabled = false;
          if (ReverseIndicatorRight != null) ReverseIndicatorRight.material = ReverseIndicatorOFF;
          if (PointLightReverseIndicatorRight != null) PointLightReverseIndicatorRight.enabled = false;
        }
    }
}
