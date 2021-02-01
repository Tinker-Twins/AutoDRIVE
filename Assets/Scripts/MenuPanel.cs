using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MenuPanel : MonoBehaviour
{
    /*
    This script toggles the Menu panel.
    */

    public GameObject Menu;
    private bool MenuStatus = false;

    public void ToggleMenu()
    {
        MenuStatus = !MenuStatus;
        Menu.SetActive(MenuStatus);
    }
}
