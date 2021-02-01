using System;
using UnityEngine;
using UnityEngine.UI;

public class CameraSwitch : MonoBehaviour
{
    /*
    This script switches the cameras sequentially.
    */

    public GameObject[] Cameras;
    public Text Label;

    private int m_CurrentActiveCamera;


    private void OnEnable()
    {
        Label.text = Cameras[m_CurrentActiveCamera].name;
    }


    public void NextCamera()
    {
        int nextactiveobject = m_CurrentActiveCamera + 1 >= Cameras.Length ? 0 : m_CurrentActiveCamera + 1;

        for (int i = 0; i < Cameras.Length; i++)
        {
            Cameras[i].SetActive(i == nextactiveobject);
        }

        m_CurrentActiveCamera = nextactiveobject;
        Label.text = Cameras[m_CurrentActiveCamera].name;
    }
}
