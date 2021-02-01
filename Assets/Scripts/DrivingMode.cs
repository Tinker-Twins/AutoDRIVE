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

    public VehicleTeleoperation VehicleTeleoperation;
    public Text Label;
    private bool Mode;

    private void OnEnable()
    {
        Label.text = "Manual";
    }

    public void ToggleDrivingMode()
    {
        Mode = !Mode;
        if(Mode)
        {
            VehicleTeleoperation.DrivingMode = 1;
            Label.text = "Autonomous";
        }
        else
        {
            VehicleTeleoperation.DrivingMode = 0;
            Label.text = "Manual";
        }
    }
}
