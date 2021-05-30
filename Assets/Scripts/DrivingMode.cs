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

    public VehicleController VehicleController;
    public Text Label;
    private bool Mode;

    private void Start()
    {
        VehicleController.DrivingMode = 1;
        Label.text = "Autonomous";
    }

    public void ToggleDrivingMode()
    {
        Mode = !Mode;
        if(Mode)
        {
            VehicleController.DrivingMode = 0;
            Label.text = "Manual";
        }
        else
        {
            VehicleController.DrivingMode = 1;
            Label.text = "Autonomous";
        }
    }
}
