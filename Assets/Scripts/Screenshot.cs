using UnityEngine;

public class Screenshot : MonoBehaviour
{
    // Generate a screenshot and save it to disk with the name `AutoDRIVE Simulator.png`.

    void Update()
    {
        ScreenCapture.CaptureScreenshot("AutoDRIVE Simulator.png");
    }
}