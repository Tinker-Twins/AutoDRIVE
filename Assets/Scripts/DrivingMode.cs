using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DrivingMode : MonoBehaviour
{
    /*
    This script toggles the driving mode of the ego vehicle between Manual and
    Autonomous.
    */

    public VehicleController[] VehicleControllers;
    public AutomobileController[] AutomobileControllers;
    public Text Label;
    private bool Mode;

    private void Start()
    {
        for(int i=0; i<VehicleControllers.Length; i++)
        {
            VehicleControllers[i].DrivingMode = 0;
        }
        for(int i=0; i<AutomobileControllers.Length; i++)
        {
            AutomobileControllers[i].DrivingMode = 0;
        }
        Label.text = "Manual";
    }

    public void ToggleDrivingMode()
    {
        Mode = !Mode;
        if(Mode)
        {
            for(int i=0; i<VehicleControllers.Length; i++)
            {
                VehicleControllers[i].DrivingMode = 1;
            }
            for(int i=0; i<AutomobileControllers.Length; i++)
            {
                AutomobileControllers[i].DrivingMode = 1;
            }
            Label.text = "Autonomous";
        }
        else
        {
            for(int i=0; i<VehicleControllers.Length; i++)
            {
                VehicleControllers[i].DrivingMode = 0;
            }
            for(int i=0; i<AutomobileControllers.Length; i++)
            {
                AutomobileControllers[i].DrivingMode = 0;
            }
            Label.text = "Manual";
        }
    }
}