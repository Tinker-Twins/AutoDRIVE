using UnityEngine;

public static class FrameGrabber
{
    /*
    This script grabs camera frames from its respective `RenderTexture` objects
    and encodes them into JPG format.
    */
    public static byte[] CaptureFrame(Camera camera)
    {
        // A camera with a fisheye lens renders a wide view; its picture is the lens's
        FisheyeLens lens = camera.GetComponent<FisheyeLens>();
        RenderTexture targetTexture = lens != null ? lens.Image() : camera.targetTexture;
        RenderTexture.active = targetTexture;
        Texture2D texture2D = new Texture2D(targetTexture.width, targetTexture.height, TextureFormat.RGB24, false);
        texture2D.ReadPixels(new Rect(0, 0, targetTexture.width, targetTexture.height), 0, 0);
        texture2D.Apply();
        byte[] image = texture2D.EncodeToJPG();
        Object.DestroyImmediate(texture2D); // Required to prevent memory leak
        return image;
    }
}
