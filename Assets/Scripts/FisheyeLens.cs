using UnityEngine;

public class FisheyeLens : MonoBehaviour
{
    /*
    This script gives a camera a fisheye lens: equidistant (a ray's distance from the
    picture's centre is proportional to its angle off the axis), `DiagonalFOV` degrees
    across the picture's diagonal. The camera renders an ordinary view just wide enough
    for the fisheye's corners into a texture of the lens's own; the shader `FisheyeShader`
    remaps it into the texture the camera was given (the picture), every frame and
    whenever a frame is grabbed (`FrameGrabber`). A camera given a new texture (the socket
    gives each car after the first one its own) is refitted to it.
    */

    public float DiagonalFOV = 130f; // Degrees across the picture's diagonal
    public int Oversample = 2; // The wide view's size, times the picture's
    public Shader FisheyeShader;

    private Camera lensCamera;
    private RenderTexture picture;
    private RenderTexture wideView;
    private Material fisheyeMaterial;

    void Awake()
    {
        lensCamera = GetComponent<Camera>();
        fisheyeMaterial = new Material(FisheyeShader);
        Fit();
    }

    void LateUpdate()
    {
        Image(); // The picture is also shown on the HUD
    }

    // The fisheye picture of the camera's last frame
    public RenderTexture Image()
    {
        if (lensCamera.targetTexture != wideView) Fit();
        Graphics.Blit(wideView, picture, fisheyeMaterial);
        return picture;
    }

    // Takes the camera's texture as the picture, and widens the camera onto a texture of the lens's own
    void Fit()
    {
        picture = lensCamera.targetTexture;
        float w = picture.width, h = picture.height, d = Mathf.Sqrt(w * w + h * h);
        float halfDiagonal = 0.5f * DiagonalFOV * Mathf.Deg2Rad;
        // The wide view's corners are the fisheye's: rays halfDiagonal off the axis, along the diagonals
        float tanX = Mathf.Tan(halfDiagonal) * w / d, tanY = Mathf.Tan(halfDiagonal) * h / d;
        lensCamera.usePhysicalProperties = false;
        lensCamera.fieldOfView = 2f * Mathf.Atan(tanY) * Mathf.Rad2Deg;
        lensCamera.aspect = w / h;
        if (wideView != null) wideView.Release();
        wideView = new RenderTexture(Oversample * picture.width, Oversample * picture.height, 24, picture.format)
        {
            name = "Fisheye Wide View"
        };
        lensCamera.targetTexture = wideView;
        fisheyeMaterial.SetVector("_Lens", new Vector4(0.5f * d / halfDiagonal, w, h, 0f));
        fisheyeMaterial.SetVector("_Wide", new Vector4(tanX, tanY, 0f, 0f));
    }
}
