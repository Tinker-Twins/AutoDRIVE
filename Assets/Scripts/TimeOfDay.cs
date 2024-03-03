using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TimeOfDay : MonoBehaviour
{
    /*
    This script moves the scene light (sun) based on time of day.
    */

    public GameObject sceneLight; // Sun
    public Timer simulationTime; // Timer (gives simulation time in seconds)
    public float startTime = 9f; // Initial time of day (in hours)
    public bool automaticUpdate = true; // Switch automatic update on/off
    public float timeScale = 60.0f; // Scale to speed up/down the time of day vs. simulation time
    public float timeOfDay; // Time of the day
    public Vector3 currentAngle; // Store current angle of sun
    public float targetAngle; // Store target angle of sun

    void Start()
    {
        timeOfDay = (startTime*60) + (simulationTime.timer*timeScale/60); // Compute time of day (in minutes)
        currentAngle = sceneLight.transform.eulerAngles; // Store current angle of sun
        targetAngle = (startTime*15f)-90f; // Compute initial sun angle
        currentAngle = new Vector3(targetAngle, currentAngle.y, currentAngle.z); // Compute the sun orientation
        sceneLight.transform.eulerAngles = currentAngle; // Update the sun angle
    }

    void Update()
    {
        if(automaticUpdate)
        {
            timeOfDay = (startTime*60) + (simulationTime.timer*timeScale/60); // Update time of day (speed up/down vs. simulation time)
            targetAngle = (0.25f*timeOfDay)-90f; // Compute target angle of sun
            currentAngle = new Vector3(Mathf.LerpAngle(currentAngle.x, targetAngle, Time.deltaTime), currentAngle.y, currentAngle.z); // Lerp the sun angle smoothly
            sceneLight.transform.eulerAngles = currentAngle; // Update the sun angle smoothly
        }
        else
        {
            targetAngle = (0.25f*timeOfDay)-90f; // Compute target angle of sun
            sceneLight.transform.eulerAngles = new Vector3(targetAngle, currentAngle.y, currentAngle.z); // Update the sun angle
        }
    }
}
