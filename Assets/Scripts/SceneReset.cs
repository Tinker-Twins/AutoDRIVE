using System;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;

public class SceneReset :MonoBehaviour , IPointerClickHandler
{
    public void OnPointerClick(PointerEventData data)
    {
        /*
        This script reloads the scene afresh.
        */
        SceneManager.LoadScene(SceneManager.GetSceneAt(0).name);
    }
}
