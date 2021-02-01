using System;
using UnityEngine;
using UnityEngine.UI;

public class FPSCounter : MonoBehaviour
{
    /*
    This script measures the average FPS of the simulator application.
    */

    public int FPS;
    const float fpsMeasurePeriod = 0.5f;
    private int m_FpsAccumulator = 0;
    private float m_FpsNextPeriod = 0;

    private void Start()
    {
        m_FpsNextPeriod = Time.realtimeSinceStartup + fpsMeasurePeriod;
    }

    private void Update()
    {
        // Measure average frames per second
        m_FpsAccumulator++;
        if (Time.realtimeSinceStartup > m_FpsNextPeriod)
        {
            FPS = (int) (m_FpsAccumulator/fpsMeasurePeriod);
            m_FpsAccumulator = 0;
            m_FpsNextPeriod += fpsMeasurePeriod;
        }
        //Debug.Log(FPS);
    }
}
