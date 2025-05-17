using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TimeScale : MonoBehaviour
{
    /*
    Changes the simulation time scale between 0 (paused) and 100 (faster than real-time)
    based on a slider input. The value of 1 corresponds to real-time operation.
    */
    public Slider timeScaleSlider;
    public Text timeScaleText;
    public bool scalePhysicsTime = false;
    private float fixedDeltaTime;

    void Awake()
    {
        timeScaleSlider.value = Time.timeScale; // Read initial timescale
        timeScaleText.text = timeScaleSlider.value.ToString("F2") + "X"; // Update text
        this.fixedDeltaTime = Time.fixedDeltaTime; // Make a copy of the fixedDeltaTime
    }

    void Update()
    {
        Time.timeScale = timeScaleSlider.value; // Change timescale
        if (scalePhysicsTime) Time.fixedDeltaTime = this.fixedDeltaTime * Time.timeScale; // Adjust fixed delta time according to timescale
        timeScaleText.text = timeScaleSlider.value.ToString("F2") + "X"; // Update text
    }
}
