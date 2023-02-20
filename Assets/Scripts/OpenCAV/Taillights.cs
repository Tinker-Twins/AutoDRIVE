using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Taillights : MonoBehaviour
{
    public Headlights Headlights;
    public Texture LightsOFF;
    public Texture LightsON;

    public Light TaillightLeft;
    public Light TaillightRight;
    public Renderer TaillightRenderer;

    void Update()
    {
        // Brake Lights
        if(Input.GetKey (KeyCode.X))
        {
            TaillightRenderer.materials[8].SetTexture("_MainTex", LightsON);
            TaillightLeft.intensity = TaillightRight.intensity = 3f;
            TaillightLeft.enabled = TaillightRight.enabled = true;
        }
        else
        {
            if(Headlights.HeadlightStatus)
            {
                TaillightRenderer.materials[8].SetTexture("_MainTex", LightsON);
                TaillightLeft.intensity = TaillightRight.intensity = 1.5f;
                TaillightLeft.enabled = TaillightRight.enabled = true;
            }
            else
            {
                TaillightRenderer.materials[8].SetTexture("_MainTex", LightsOFF);
                TaillightLeft.enabled = TaillightRight.enabled = false;
            }
        }
    }
}
