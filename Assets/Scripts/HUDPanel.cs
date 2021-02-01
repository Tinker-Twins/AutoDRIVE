using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HUDPanel : MonoBehaviour
{
    /*
    This script toggles the HUD panel.
    */

    public GameObject HUD;
    private bool HUDStatus = false;

    public void ToggleHUD()
    {
        HUDStatus = !HUDStatus;
        HUD.SetActive(HUDStatus);
    }
}
