using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ROVLighting : MonoBehaviour
{
    /*
    This script controls the vehicle lighting. The designed vehicle has the following functional lights and indicators:

    1. Headlights: Switched between 3 states, viz. OFF, Low-Beam and High-Beam.
    2. Taillights: Automatically switched between OFF, Partially ON (when Headlights are switched ON) and ON (when brakes are engaged).
    3. Signature Lights: Toggled ON and OFF.
    */

    public AutomobileController AutomobileController;
    public DataRecorder DataRecorder;
    [HideInInspector] public float RecordedVelocity;

    // HEADLIGHTS
    private bool HeadlightsFlag = false;
    public Renderer HeadlightLeft;
    public Renderer HeadlightRight;
    public Light SpotLightHeadlightLeft;
    public Light SpotLightHeadlightRight;
    public Material HeadlightOFF;
    public Material HeadlightON;

    // TAILLIGHTS
    private bool TaillightsFlag = false;
    public bool BrakesEngaged = false;
    public Renderer TaillightLeft;
    public Renderer TaillightRight;
    public Light PointLightTaillightLeft;
    public Light PointLightTaillightRight;
    public Material TaillightOFF;
    public Material TaillightPartiallyON;
    public Material TaillightON;

    // SIGNATURE LIGHTS
    private bool SignatureLightsFlag = false;
    public Renderer SignatureLightLeft;
    public Renderer SignatureLightRight;
    public Material SignatureLightOFF;
    public Material SignatureLightON;

    // AUTONOMOUS CONTROL METHODS
    private int HeadlightState = 0; // 0 = Disabled, 1 = Enabled, 2 = Signature Lights, 3 = 1+2

    public int Headlights
    {
        get { return HeadlightState; }
        set { HeadlightState = value; }
    }

    void Update()
    {
        // ============================================== //
        // DEFINE LOGIC TO SET LIGHT STATUS TO TRUE/FALSE //
        // ============================================== //

        // Light Control (Manual Mode)
        if(AutomobileController.CurrentDrivingMode == 0)
        {
            // Headlights
            if(Input.GetKeyDown(KeyCode.K) || Input.GetKeyDown(KeyCode.I))
            {
                HeadlightsFlag = !HeadlightsFlag;
            }

            // Signature Lights
            if(Input.GetKeyDown(KeyCode.U))
            {
                SignatureLightsFlag = !SignatureLightsFlag;
            }
        }

        // Light Control (Autonomous Mode)
        if(AutomobileController.CurrentDrivingMode == 1)
        {
            // Headlights - Disabled
            if(HeadlightState == 0)
            {
                HeadlightsFlag = false;
                SignatureLightsFlag = false;
            }

            // Headlights - Enabled
            if(HeadlightState == 1)
            {
                HeadlightsFlag = true;
                SignatureLightsFlag = false;
            }

            // Signature Lights
            if(HeadlightState == 2)
            {
                HeadlightsFlag = false;
                SignatureLightsFlag = true;
            }

            // Headlights - Headlights Enabled + Signature Lights
            if(HeadlightState == 3)
            {
                HeadlightsFlag = true;
                SignatureLightsFlag = true;
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
                TaillightsFlag = true;
            }
            else
            {
                TaillightsFlag = false;
            }
        }
        else
        {
            // Brake Lights
            if(BrakesEngaged)
            {
                TaillightsFlag = true;
            }
            else
            {
                TaillightsFlag = false;
            }
        }

        // =================================== //
        // RENDER LIGHTS BASED ON THEIR STATUS //
        // =================================== //

        // Headlights - Enabled
        if(HeadlightsFlag)
        {
            HeadlightLeft.material = HeadlightON;
            SpotLightHeadlightLeft.enabled = true;
            HeadlightRight.material = HeadlightON;
            SpotLightHeadlightRight.enabled = true;
        }
        else
        {
            HeadlightLeft.material = HeadlightOFF;
            SpotLightHeadlightLeft.enabled = false;
            HeadlightRight.material = HeadlightOFF;
            SpotLightHeadlightRight.enabled = false;
        }

        // Brake Lights
        if(TaillightsFlag)
        {
            TaillightLeft.material = TaillightON;
            PointLightTaillightLeft.intensity = 5000f;
            PointLightTaillightLeft.enabled = true;
            TaillightRight.material = TaillightON;
            PointLightTaillightRight.intensity = 5000f;
            PointLightTaillightRight.enabled = true;
        }
        else
        {
            if(HeadlightsFlag)
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

        // Signature Lights
        if(SignatureLightsFlag)
        {
            SignatureLightLeft.material = SignatureLightON;
            SignatureLightRight.material = SignatureLightON;
        }
        else
        {
            SignatureLightLeft.material = SignatureLightOFF;
            SignatureLightRight.material = SignatureLightOFF;
        }
    }
}
