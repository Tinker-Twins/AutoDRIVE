using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RenderingQuality : MonoBehaviour
{
    /*
    This script toggles the rendering quality of the simulator between High Quality
    (i.e. HDRP and post-processing enabled), Medium Quality (i.e. only HDRP enabled)
    and Low Quality (i.e. HDRP and post-processing disabled). The rendering quality
    also affects the terrain details abd the shadows being generated in real-time.
    */

    //public GameObject PostProcessVolume;
    public GameObject HQ_HDRPSettings;
    public GameObject UQ_HDRPSettings;
    public GameObject ReflectionProbe;
    public Light SceneLight;
    public Terrain[] Terrains;
    public GameObject[] LightingObjects;
    public Text Label;

    private int Mode = 0;

    private void Awake()
    {
        //PostProcessVolume.SetActive(false);
        HQ_HDRPSettings.SetActive(false);
        UQ_HDRPSettings.SetActive(false);
        ReflectionProbe.SetActive(false);
        SceneLight.shadows = LightShadows.None;
        for(int i=0;i<LightingObjects.Length;i++)
            {
                LightingObjects[i].SetActive(false);
            }
        for(int i=0;i<Terrains.Length;i++)
            {
                Terrains[i].detailObjectDensity = 0.0f;
                Terrains[i].heightmapPixelError = 15;
                Terrains[i].Flush();
            }
        Application.targetFrameRate = 30;
        Label.text = "Low Quality";
    }

    public void ToggleRenderingQuality()
    {
        Mode += 1;
        if(Mode > 2)
        {
            Mode = 0;
        }
        if(Mode == 0)
        {
            //PostProcessVolume.SetActive(false);
            HQ_HDRPSettings.SetActive(false);
            UQ_HDRPSettings.SetActive(false);
            ReflectionProbe.SetActive(false);
            SceneLight.shadows = LightShadows.None;
            for(int i=0;i<LightingObjects.Length;i++)
            {
                LightingObjects[i].SetActive(false);
            }
            for(int i=0;i<Terrains.Length;i++)
            {
                Terrains[i].detailObjectDensity = 0.0f;
                Terrains[i].heightmapPixelError = 15;
                Terrains[i].Flush();
            }
            Application.targetFrameRate = 30;
            Label.text = "Low Quality";
        }
        else if(Mode == 1)
        {
            //PostProcessVolume.SetActive(false);
            HQ_HDRPSettings.SetActive(true);
            UQ_HDRPSettings.SetActive(false);
            ReflectionProbe.SetActive(false);
            SceneLight.shadows = LightShadows.Hard;
            for(int i=0;i<LightingObjects.Length;i++)
            {
                LightingObjects[i].SetActive(false);
            }
            for(int i=0;i<Terrains.Length;i++)
            {
                Terrains[i].detailObjectDensity = 0.5f;
                Terrains[i].heightmapPixelError = 10;
                Terrains[i].Flush();
            }
            Application.targetFrameRate = 60;
            Label.text = "High Quality";
        }
        else
        {
            //PostProcessVolume.SetActive(true);
            HQ_HDRPSettings.SetActive(false);
            UQ_HDRPSettings.SetActive(true);
            ReflectionProbe.SetActive(true);
            SceneLight.shadows = LightShadows.Soft;
            for(int i=0;i<LightingObjects.Length;i++)
            {
                LightingObjects[i].SetActive(true);
            }
            for(int i=0;i<Terrains.Length;i++)
            {
                Terrains[i].detailObjectDensity = 1.0f;
                Terrains[i].heightmapPixelError = 5;
                Terrains[i].Flush();
            }
            Application.targetFrameRate = -1;
            Label.text = "Ultra Quality";
        }
    }
}