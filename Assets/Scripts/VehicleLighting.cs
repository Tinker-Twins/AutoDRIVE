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

    private float timer = 0f;

    // HEADLIGHTS
    private bool HeadlightsLowBeam = false;
    private bool HeadlightsHighBeam = false;
    public Renderer HeadlightLeft1;
    public Renderer HeadlightLeft2;
    public Renderer HeadlightRight1;
    public Renderer HeadlightRight2;
    public Light SpotLightHeadlightLeft1;
    public Light SpotLightHeadlightLeft2;
    public Light SpotLightHeadlightRight1;
    public Light SpotLightHeadlightRight2;
    public Light PointLightHeadlightLeft1;
    public Light PointLightHeadlightLeft2;
    public Light PointLightHeadlightRight1;
    public Light PointLightHeadlightRight2;
    public Material HeadlightOFF;
    public Material HeadlightON;

    // TAILLIGHTS
    private bool Taillights = false;
    public Renderer TaillightLeft;
    public Renderer TaillightRight;
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
    public Light SpotLightReverseIndicatorLeft;
    public Light SpotLightReverseIndicatorRight;
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
            if(System.Math.Round(DataRecorder.getSavedVelocity(),1) == 0)
            {
                Taillights = true;
            }
            else
            {
                Taillights = false;
            }

            // Reverse Indicators
            if(System.Math.Round(DataRecorder.getSavedVelocity(),1) < 0)
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
            if(System.Math.Round(VehicleController.Vehicle.transform.InverseTransformDirection(VehicleController.Vehicle.GetComponent<Rigidbody>().velocity).z,1) == 0)
            {
                Taillights = true;
            }
            else
            {
                Taillights = false;
            }

            // Reverse Indicators
            if(System.Math.Round(VehicleController.Vehicle.transform.InverseTransformDirection(VehicleController.Vehicle.GetComponent<Rigidbody>().velocity).z,1) < 0)
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
            HeadlightLeft1.material = HeadlightON;
            SpotLightHeadlightLeft1.enabled = true;
            PointLightHeadlightLeft1.enabled = true;
            HeadlightLeft2.material = HeadlightOFF;
            SpotLightHeadlightLeft2.enabled = false;
            PointLightHeadlightLeft2.enabled = false;
            HeadlightRight1.material = HeadlightON;
            SpotLightHeadlightRight1.enabled = true;
            PointLightHeadlightRight1.enabled = true;
            HeadlightRight2.material = HeadlightOFF;
            SpotLightHeadlightRight2.enabled = false;
            PointLightHeadlightRight2.enabled = false;
        }

        // Headlights - High Beam
        else if(HeadlightsHighBeam)
        {
            HeadlightLeft1.material = HeadlightON;
            SpotLightHeadlightLeft1.enabled = true;
            PointLightHeadlightLeft1.enabled = true;
            HeadlightLeft2.material = HeadlightON;
            SpotLightHeadlightLeft2.enabled = true;
            PointLightHeadlightLeft2.enabled = true;
            HeadlightRight1.material = HeadlightON;
            SpotLightHeadlightRight1.enabled = true;
            PointLightHeadlightRight1.enabled = true;
            HeadlightRight2.material = HeadlightON;
            SpotLightHeadlightRight2.enabled = true;
            PointLightHeadlightRight2.enabled = true;
        }

        else
        {
            HeadlightLeft1.material = HeadlightOFF;
            SpotLightHeadlightLeft1.enabled = false;
            PointLightHeadlightLeft1.enabled = false;
            HeadlightLeft2.material = HeadlightOFF;
            SpotLightHeadlightLeft2.enabled = false;
            PointLightHeadlightLeft2.enabled = false;
            HeadlightRight1.material = HeadlightOFF;
            SpotLightHeadlightRight1.enabled = false;
            PointLightHeadlightRight1.enabled = false;
            HeadlightRight2.material = HeadlightOFF;
            SpotLightHeadlightRight2.enabled = false;
            PointLightHeadlightRight2.enabled = false;
        }

        // Brake Lights
        if(Taillights)
        {
            TaillightLeft.material = TaillightON;
            PointLightTaillightLeft.intensity = 0.05f;
            PointLightTaillightLeft.enabled = true;
            TaillightRight.material = TaillightON;
            PointLightTaillightRight.intensity = 0.05f;
            PointLightTaillightRight.enabled = true;
        }
        else
        {
            if(HeadlightsLowBeam || HeadlightsHighBeam)
            {
                TaillightLeft.material = TaillightPartiallyON;
                PointLightTaillightLeft.intensity = 0.025f;
                PointLightTaillightLeft.enabled = true;
                TaillightRight.material = TaillightPartiallyON;
                PointLightTaillightRight.intensity = 0.025f;
                PointLightTaillightRight.enabled = true;
            }
            else
            {
                TaillightLeft.material = TaillightOFF;
                PointLightTaillightLeft.enabled = false;
                TaillightRight.material = TaillightOFF;
                PointLightTaillightRight.enabled = false;
            }
        }

        // Left Turn Indicators
        if(LeftTurnIndicators)
        {
            timer = timer + Time.deltaTime;
            if(timer >= 0.5)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorON;
                PointLightTurnIndicatorFrontLeft.enabled = true;
                TurnIndicatorRearLeft.material = TurnIndicatorON;
                PointLightTurnIndicatorRearLeft.enabled = true;
                TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearRight.enabled = false;
            }
            if(timer >= 1)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorOFF;
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
                TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorON;
                PointLightTurnIndicatorFrontRight.enabled = true;
                TurnIndicatorRearRight.material = TurnIndicatorON;
                PointLightTurnIndicatorRearRight.enabled = true;
            }
            if(timer >= 1)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorOFF;
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
                TurnIndicatorFrontLeft.material = TurnIndicatorON;
                PointLightTurnIndicatorFrontLeft.enabled = true;
                TurnIndicatorRearLeft.material = TurnIndicatorON;
                PointLightTurnIndicatorRearLeft.enabled = true;
                TurnIndicatorFrontRight.material = TurnIndicatorON;
                PointLightTurnIndicatorFrontRight.enabled = true;
                TurnIndicatorRearRight.material = TurnIndicatorON;
                PointLightTurnIndicatorRearRight.enabled = true;
            }
            if(timer >= 1)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorOFF;
                PointLightTurnIndicatorRearRight.enabled = false;
                timer = 0;
            }
        }

        else
        {
            TurnIndicatorFrontLeft.material = TurnIndicatorOFF;
            PointLightTurnIndicatorFrontLeft.enabled = false;
            TurnIndicatorRearLeft.material = TurnIndicatorOFF;
            PointLightTurnIndicatorRearLeft.enabled = false;
            TurnIndicatorFrontRight.material = TurnIndicatorOFF;
            PointLightTurnIndicatorFrontRight.enabled = false;
            TurnIndicatorRearRight.material = TurnIndicatorOFF;
            PointLightTurnIndicatorRearRight.enabled = false;
        }

        // Reverse Indicators
        if(ReverseIndicators)
        {
            ReverseIndicatorLeft.material = ReverseIndicatorON;
            SpotLightReverseIndicatorLeft.enabled = true;
            PointLightReverseIndicatorLeft.enabled = true;
            ReverseIndicatorRight.material = ReverseIndicatorON;
            SpotLightReverseIndicatorRight.enabled = true;
            PointLightReverseIndicatorRight.enabled = true;
        }
        else
        {
          ReverseIndicatorLeft.material = ReverseIndicatorOFF;
          SpotLightReverseIndicatorLeft.enabled = false;
          PointLightReverseIndicatorLeft.enabled = false;
          ReverseIndicatorRight.material = ReverseIndicatorOFF;
          SpotLightReverseIndicatorRight.enabled = false;
          PointLightReverseIndicatorRight.enabled = false;
        }
    }
}
