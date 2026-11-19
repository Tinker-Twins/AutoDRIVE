using UnityEditor;
using UnityEngine;

// Gives the NeoRacer prefab's onboard camera (Preview, the one AutoDRIVE streams as the
// car's camera) the real car's lens. Upstream modelled it as a Raspberry Pi camera:
// 3.04 mm on a 3.68 x 2.76 mm sensor, 62 x 49 degrees. The NeoRacer's is a 130 degree
// fisheye (2.7 mm, the camera's spec). Its place and lean are the CAD's (NeoRacerModelSwap).
// Menu, or batchmode:
//   -executeMethod NeoRacerCamera.Apply
public static class NeoRacerCamera
{
    const string Prefab = "Assets/Prefabs/NeoRacer/NeoRacer.prefab";
    const string ShaderFile = "Assets/Shaders/Fisheye Shader.shader";
    const string CameraName = "Preview";
    const string Texture = "NeoRacer Camera"; // the texture the streamed camera renders into
    const float DiagonalFOV = 130f;

    [MenuItem("AutoDRIVE/NeoRacer: Real Camera Lens")]
    static void Menu()
    {
        Debug.Log(Make() ? "CAMERA fitted: " + Prefab : "CAMERA FAILED: see the errors above");
    }

    public static void Apply()
    {
        EditorApplication.Exit(Make() ? 0 : 1);
    }

    static bool Make()
    {
        GameObject root = PrefabUtility.LoadPrefabContents(Prefab);
        Camera found = null;
        foreach (Camera cam in root.GetComponentsInChildren<Camera>(true))
        {
            if (cam.name != CameraName) continue;
            if (cam.targetTexture == null || cam.targetTexture.name != Texture || found != null)
            {
                Debug.LogError("NeoRacerCamera: expected one " + CameraName + " camera rendering into " + Texture);
                PrefabUtility.UnloadPrefabContents(root);
                return false;
            }
            found = cam;
        }
        if (found == null)
        {
            Debug.LogError("NeoRacerCamera: the NeoRacer has no " + CameraName + " camera");
            PrefabUtility.UnloadPrefabContents(root);
            return false;
        }
        FisheyeLens lens = found.GetComponent<FisheyeLens>();
        if (lens == null) lens = found.gameObject.AddComponent<FisheyeLens>();
        lens.DiagonalFOV = DiagonalFOV;
        lens.FisheyeShader = AssetDatabase.LoadAssetAtPath<Shader>(ShaderFile);
        PrefabUtility.SaveAsPrefabAsset(root, Prefab);
        PrefabUtility.UnloadPrefabContents(root);
        Debug.Log("CAMERA " + CameraName + ": " + DiagonalFOV + " degree fisheye");
        return true;
    }
}
