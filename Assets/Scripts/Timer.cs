using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Timer : MonoBehaviour
{
    /*
    This script implements a timer of the format [HH:MM:SS].
    */

    public string SimulationTime;
    public float timer;

    void Update()
    {
        timer += Time.deltaTime;
        string hours = Mathf.Floor(timer/3600).ToString("00");
        string minutes = Mathf.Floor((timer/60)%60).ToString("00");
        string seconds = Mathf.Floor(timer%60).ToString("00");

         SimulationTime = hours + ":" +  minutes + ":" + seconds;
         //Debug.Log(SimulationTime);
    }
}
