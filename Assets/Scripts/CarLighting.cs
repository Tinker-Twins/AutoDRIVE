using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarLighting : MonoBehaviour
{
    /*
    This script controls the vehicle lighting. The designed vehicle has the following functional lights and indicators:

    1. Headlights: Switched between 3 states, viz. OFF, Low-Beam and High-Beam.
    2. Taillights: Automatically switched between OFF, Partially ON (when Headlights are switched ON) and ON (when brakes are engaged).
    3. Turn Indicators: Left and Right Turn Indicators toggled ON and OFF.
    4. Reverse Indicators: Automatically toggled ON and OFF depending on the velocity of the driven wheels.
    5. Hazard Indicators: Toggled ON and OFF.
    6. Brake Indicator: Automatically switched between OFF and ON (when brakes are engaged).
    7. Parking Lights: Toggled ON and OFF.
    8. Fog Lights: Toggled ON and OFF.
    */

    public AutomobileController AutomobileController;
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
    public Light SpotLightHeadlightLeft2;
    public Light SpotLightHeadlightRight1;
    public Light SpotLightHeadlightRight2;
    public Material Headlight1OFF;
    public Material Headlight2OFF;
    public Material Headlight1ON;
    public Material Headlight2ON;

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
    public Material TurnIndicatorFrontOFF;
    public Material TurnIndicatorFrontON;
    public Material TurnIndicatorRearOFF;
    public Material TurnIndicatorRearON;

    // REVERSE INDICATORS
    private bool ReverseIndicators = false;
    public Renderer ReverseIndicatorLeft;
    public Renderer ReverseIndicatorRight;
    public Light PointLightReverseIndicatorLeft;
    public Light PointLightReverseIndicatorRight;
    public Material ReverseIndicatorOFF;
    public Material ReverseIndicatorON;

    // BRAKE INDICATOR
    public bool BrakesEngaged = false;
    public Renderer BrakeLight;
    public Material BrakeLightOFF;
    public Material BrakeLightON;

    // PARKING LIGHTS
    private bool ParkingLights = false;
    public Renderer ParkingLightLeft;
    public Renderer ParkingLightRight;
    public Material ParkingLightOFF;
    public Material ParkingLightON;

    // FOG LIGHTS
    private bool FogLights = false;
    public Renderer FogLightLeft;
    public Renderer FogLightRight;
    public Light SpotLightFogLightLeft;
    public Light SpotLightFogLightRight;
    public Material FogLightsOFF;
    public Material FogLightsON;

    // AUTONOMOUS CONTROL METHODS
    private int HeadlightState = 0; // 0 = Disabled, 1 = Low Beam, 2 = High Beam, 3 = Parking Lights, 4 = Fog Lights, 5 = 1+3, 6 = 1+4, 7 = 2+3, 8 = 2+4, 9 = 3+4, 10 = 1+3+4, 11 = 2+3+4
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
        if(AutomobileController.CurrentDrivingMode == 0)
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

            // Parking Lights
            if(Input.GetKeyDown(KeyCode.U))
            {
                ParkingLights = !ParkingLights;
            }

            // Fog Lights
            if(Input.GetKeyDown(KeyCode.O))
            {
                FogLights = !FogLights;
            }
        }

        // Light Control (Autonomous Mode)
        if(AutomobileController.CurrentDrivingMode == 1)
        {
            // Headlights - Disabled
            if(HeadlightState == 0)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = false;
                ParkingLights = false;
                FogLights = false;
            }

            // Headlights - Low Beam
            if(HeadlightState == 1)
            {
                HeadlightsHighBeam = false;
                HeadlightsLowBeam = true;
                ParkingLights = false;
                FogLights = false;
            }

            // Headlights - High Beam
            if(HeadlightState == 2)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = true;
                ParkingLights = false;
                FogLights = false;
            }

            // Parking Lights
            if(HeadlightState == 3)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = false;
                ParkingLights = true;
                FogLights = false;
            }

            // Fog Lights
            if(HeadlightState == 4)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = false;
                ParkingLights = false;
                FogLights = true;
            }

            // Headlights - Low Beam + Parking Lights
            if(HeadlightState == 5)
            {
                HeadlightsLowBeam = true;
                HeadlightsHighBeam = false;
                ParkingLights = true;
                FogLights = false;
            }

            // Headlights - Low Beam + Fog Lights
            if(HeadlightState == 6)
            {
                HeadlightsLowBeam = true;
                HeadlightsHighBeam = false;
                ParkingLights = false;
                FogLights = true;
            }

            // Headlights - High Beam + Parking Lights
            if(HeadlightState == 7)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = true;
                ParkingLights = true;
                FogLights = false;
            }

            // Headlights - High Beam + Fog Lights
            if(HeadlightState == 8)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = true;
                ParkingLights = false;
                FogLights = true;
            }

            // Headlights - Parking Lights + Fog Lights
            if(HeadlightState == 9)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = false;
                ParkingLights = true;
                FogLights = true;
            }

            // Headlights - Low Beam + Parking Lights + Fog Lights
            if(HeadlightState == 10)
            {
                HeadlightsLowBeam = true;
                HeadlightsHighBeam = false;
                ParkingLights = true;
                FogLights = true;
            }

            // Headlights - High Beam + Parking Lights + Fog Lights
            if(HeadlightState == 11)
            {
                HeadlightsLowBeam = false;
                HeadlightsHighBeam = true;
                ParkingLights = true;
                FogLights = true;
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
        if (Input.GetKey(KeyCode.X) || (AutomobileController.CurrentBrake != 0))
        {
            BrakesEngaged = true;
        }
        else
        {
            BrakesEngaged = false;
        }
        if (DataRecorder.getSaveStatus())
        {
            // Brake Lights
            if(BrakesEngaged)
            {
                Taillights = true;
            }
            else
            {
                Taillights = false;
            }

            // Reverse Indicators
            if(System.Math.Round(RecordedVelocity,1) < 0) // TODO: Change this to `gearNum` by updating `DataRecorder.cs`
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
            if(BrakesEngaged)
            {
                Taillights = true;
            }
            else
            {
                Taillights = false;
            }

            // Reverse Indicators
            if(AutomobileController.currSpeed != 0 && AutomobileController.gearNum < 0)
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
            HeadlightLeft1.material = Headlight1ON;
            SpotLightHeadlightLeft1.enabled = true;
            HeadlightLeft2.material = Headlight2OFF;
            SpotLightHeadlightLeft2.enabled = false;
            HeadlightRight1.material = Headlight1ON;
            SpotLightHeadlightRight1.enabled = true;
            HeadlightRight2.material = Headlight2OFF;
            SpotLightHeadlightRight2.enabled = false;
        }

        // Headlights - High Beam
        else if(HeadlightsHighBeam)
        {
            HeadlightLeft1.material = Headlight1ON;
            SpotLightHeadlightLeft1.enabled = true;
            HeadlightLeft2.material = Headlight2ON;
            SpotLightHeadlightLeft2.enabled = true;
            HeadlightRight1.material = Headlight1ON;
            SpotLightHeadlightRight1.enabled = true;
            HeadlightRight2.material = Headlight2ON;
            SpotLightHeadlightRight2.enabled = true;
        }
        else
        {
            HeadlightLeft1.material = Headlight1OFF;
            SpotLightHeadlightLeft1.enabled = false;
            HeadlightLeft2.material = Headlight2OFF;
            SpotLightHeadlightLeft2.enabled = false;
            HeadlightRight1.material = Headlight1OFF;
            SpotLightHeadlightRight1.enabled = false;
            HeadlightRight2.material = Headlight2OFF;
            SpotLightHeadlightRight2.enabled = false;
        }

        // Brake Lights
        if(Taillights)
        {
            TaillightLeft.material = TaillightON;
            PointLightTaillightLeft.intensity = 4000f;
            PointLightTaillightLeft.enabled = true;
            TaillightRight.material = TaillightON;
            PointLightTaillightRight.intensity = 4000f;
            PointLightTaillightRight.enabled = true;
            BrakeLight.material = BrakeLightON;
        }
        else
        {
            BrakeLight.material = BrakeLightOFF;
            if(HeadlightsLowBeam || HeadlightsHighBeam)
            {
                TaillightLeft.material = TaillightPartiallyON;
                PointLightTaillightLeft.intensity = 1000f;
                PointLightTaillightLeft.enabled = true;
                TaillightRight.material = TaillightPartiallyON;
                PointLightTaillightRight.intensity = 1000f;
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
                TurnIndicatorFrontLeft.material = TurnIndicatorFrontON;
                PointLightTurnIndicatorFrontLeft.enabled = true;
                TurnIndicatorRearLeft.material = TurnIndicatorRearON;
                PointLightTurnIndicatorRearLeft.enabled = true;
                TurnIndicatorFrontRight.material = TurnIndicatorFrontOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorRearOFF;
                PointLightTurnIndicatorRearRight.enabled = false;
            }
            if(timer >= 1)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorFrontOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorRearOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorFrontOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorRearOFF;
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
                TurnIndicatorFrontLeft.material = TurnIndicatorFrontOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorRearOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorFrontON;
                PointLightTurnIndicatorFrontRight.enabled = true;
                TurnIndicatorRearRight.material = TurnIndicatorRearON;
                PointLightTurnIndicatorRearRight.enabled = true;
            }
            if(timer >= 1)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorFrontOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorRearOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorFrontOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorRearOFF;
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
                TurnIndicatorFrontLeft.material = TurnIndicatorFrontON;
                PointLightTurnIndicatorFrontLeft.enabled = true;
                TurnIndicatorRearLeft.material = TurnIndicatorRearON;
                PointLightTurnIndicatorRearLeft.enabled = true;
                TurnIndicatorFrontRight.material = TurnIndicatorFrontON;
                PointLightTurnIndicatorFrontRight.enabled = true;
                TurnIndicatorRearRight.material = TurnIndicatorRearON;
                PointLightTurnIndicatorRearRight.enabled = true;
            }
            if(timer >= 1)
            {
                TurnIndicatorFrontLeft.material = TurnIndicatorFrontOFF;
                PointLightTurnIndicatorFrontLeft.enabled = false;
                TurnIndicatorRearLeft.material = TurnIndicatorRearOFF;
                PointLightTurnIndicatorRearLeft.enabled = false;
                TurnIndicatorFrontRight.material = TurnIndicatorFrontOFF;
                PointLightTurnIndicatorFrontRight.enabled = false;
                TurnIndicatorRearRight.material = TurnIndicatorRearOFF;
                PointLightTurnIndicatorRearRight.enabled = false;
                timer = 0;
            }
        }
        else
        {
            TurnIndicatorFrontLeft.material = TurnIndicatorFrontOFF;
            PointLightTurnIndicatorFrontLeft.enabled = false;
            TurnIndicatorRearLeft.material = TurnIndicatorRearOFF;
            PointLightTurnIndicatorRearLeft.enabled = false;
            TurnIndicatorFrontRight.material = TurnIndicatorFrontOFF;
            PointLightTurnIndicatorFrontRight.enabled = false;
            TurnIndicatorRearRight.material = TurnIndicatorRearOFF;
            PointLightTurnIndicatorRearRight.enabled = false;
        }

        // Reverse Indicators
        if(ReverseIndicators)
        {
            ReverseIndicatorLeft.material = ReverseIndicatorON;
            PointLightReverseIndicatorLeft.enabled = true;
            ReverseIndicatorRight.material = ReverseIndicatorON;
            PointLightReverseIndicatorRight.enabled = true;
        }
        else
        {
          ReverseIndicatorLeft.material = ReverseIndicatorOFF;
          PointLightReverseIndicatorLeft.enabled = false;
          ReverseIndicatorRight.material = ReverseIndicatorOFF;
          PointLightReverseIndicatorRight.enabled = false;
        }

        // Parking Lights
        if(ParkingLights)
        {
            ParkingLightLeft.material = ParkingLightON;
            ParkingLightRight.material = ParkingLightON;
        }
        else
        {
            ParkingLightLeft.material = ParkingLightOFF;
            ParkingLightRight.material = ParkingLightOFF;
        }

        // Fog Lights
        if(FogLights)
        {
            FogLightLeft.material = FogLightsON;
            SpotLightFogLightLeft.enabled = true;
            FogLightRight.material = FogLightsON;
            SpotLightFogLightRight.enabled = true;
        }
        else
        {
            FogLightLeft.material = FogLightsOFF;
            SpotLightFogLightLeft.enabled = false;
            FogLightRight.material = FogLightsOFF;
            SpotLightFogLightRight.enabled = false;
        }
    }
}
