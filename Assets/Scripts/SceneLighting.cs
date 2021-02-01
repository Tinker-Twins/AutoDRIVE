using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SceneLighting : MonoBehaviour
{
    /*
    This script toggles the scene light ON and OFF.
    */

    public Light SceneLight;

    public void ToggleSceneLighting()
    {
        SceneLight.enabled = !SceneLight.enabled;
    }
}
