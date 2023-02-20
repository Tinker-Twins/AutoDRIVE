using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Headlights : MonoBehaviour
{
    public Light HeadlightLeft;
    public Light HeadlightRight;

    private bool HeadlightsLowBeam = false;
    private bool HeadlightsHighBeam = false;

    public bool HeadlightStatus{get{return HeadlightsLowBeam|| HeadlightsHighBeam;}}

    void Update()
    {
        // ============================================== //
        // DEFINE LOGIC TO SET LIGHT STATUS TO TRUE/FALSE //
        // ============================================== //

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

        // =================================== //
        // TOGGLE LIGHTS BASED ON THEIR STATUS //
        // =================================== //

        // Headlights - Low Beam
        if(HeadlightsLowBeam)
        {
            HeadlightLeft.intensity = HeadlightRight.intensity = 5f;
            HeadlightLeft.enabled = HeadlightRight.enabled = true;
        }

        // Headlights - High Beam
        else if(HeadlightsHighBeam)
        {
            HeadlightLeft.intensity = HeadlightRight.intensity = 10f;
            HeadlightLeft.enabled = HeadlightRight.enabled = true;
        }

        // Headlights - OFF
        else
        {
            HeadlightLeft.enabled = HeadlightRight.enabled = false;
        }
    }
}
