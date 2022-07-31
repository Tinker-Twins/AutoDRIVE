using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TLController : MonoBehaviour
{
    /*
    This script controls traffic light.
    */

    public Renderer Red;
    public Renderer Yellow;
    public Renderer Green;

    public Light RedLight;
    public Light YellowLight;
    public Light GreenLight;

    public Material RedLightOFF;
    public Material RedLightON;
    public Material YellowLightOFF;
    public Material YellowLightON;
    public Material GreenLightOFF;
    public Material GreenLightON;

    public int TLState = 0; // 0 = Red, 1 = Yellow, 2 = Green

    public int CurrentState
    {
        get { return TLState; }
        set { TLState = value; }
    }

    void Update()
    {
        // Red
        if(TLState == 1)
        {
            Red.material = RedLightON;
            RedLight.enabled = true;
            Yellow.material = YellowLightOFF;
            YellowLight.enabled = false;
            Green.material = GreenLightOFF;
            GreenLight.enabled = false;
        }
        // Yellow
        else if(TLState == 2)
        {
            Red.material = RedLightOFF;
            RedLight.enabled = false;
            Yellow.material = YellowLightON;
            YellowLight.enabled = true;
            Green.material = GreenLightOFF;
            GreenLight.enabled = false;
        }
        // Green
        else if(TLState == 3)
        {
            Red.material = RedLightOFF;
            RedLight.enabled = false;
            Yellow.material = YellowLightOFF;
            YellowLight.enabled = false;
            Green.material = GreenLightON;
            GreenLight.enabled = true;
        }
        // Disabled
        else
        {
            Red.material = RedLightOFF;
            RedLight.enabled = false;
            Yellow.material = YellowLightOFF;
            YellowLight.enabled = false;
            Green.material = GreenLightOFF;
            GreenLight.enabled = false;
        }
    }
}
