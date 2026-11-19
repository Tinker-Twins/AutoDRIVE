using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

// Renders close stills of the NeoRacer where it stands in a scene (the AuE-X930 scene, or the
// one -neoracerScene names), for comparing one model with another: every camera is placed from
// the car itself (its middle, its wheels, its LED matrix), so two models give the same views.
// Batchmode, with graphics (not -nographics):
//   -executeMethod NeoRacerLookShots.Run -neoracerShotDir <folder> [-neoracerScene <scene>] [-neoracerUltra]
public static class NeoRacerLookShots
{
    const string DefaultScene = "Assets/Scenes/NeoRacer - AuE-X930.unity";
    const int Frames = 12; // renders per still, for exposure and anti-aliasing to settle
    const int Width = 1920, Height = 1080;

    public static void Run()
    {
        string[] args = Environment.GetCommandLineArgs();
        string dir = args[Array.IndexOf(args, "-neoracerShotDir") + 1];
        Directory.CreateDirectory(dir);
        int named = Array.IndexOf(args, "-neoracerScene");
        EditorSceneManager.OpenScene(named >= 0 ? args[named + 1] : DefaultScene, OpenSceneMode.Single);
        if (Array.IndexOf(args, "-neoracerUltra") >= 0)
        {
            // the simulator's Ultra Quality: its quality button pressed twice from Low, where a scene starts
            RenderingQuality quality = UnityEngine.Object.FindObjectOfType<RenderingQuality>(true);
            quality.ToggleRenderingQuality();
            quality.ToggleRenderingQuality();
            Debug.Log("SHOT quality " + quality.Label.text);
        }
        VehicleController car = UnityEngine.Object.FindObjectOfType<VehicleController>(true);
        Transform body = car.VehicleRigidBody.transform;
        Vector3 fwd = body.forward, right = body.right, up = body.up;
        Vector3 middle = body.position + fwd * 0.144f + up * 0.07f; // mid-wheelbase, 7 cm up

        var go = new GameObject("Shot Camera");
        Camera cam = go.AddComponent<Camera>();
        HDAdditionalCameraData hd = go.AddComponent<HDAdditionalCameraData>();
        hd.antialiasing = HDAdditionalCameraData.AntialiasingMode.SubpixelMorphologicalAntiAliasing;
        cam.nearClipPlane = 0.01f;
        cam.farClipPlane = 100f;

        void Around(string name, float fov, Vector3 offset, Vector3 at)
        {
            cam.fieldOfView = fov;
            Shot(cam, dir, name, middle + offset, Quaternion.LookRotation(at - (middle + offset), up));
        }
        Around("front_three_quarter_left", 30f, fwd * 0.62f - right * 0.52f + up * 0.36f, middle);
        Around("front_three_quarter_right", 30f, fwd * 0.62f + right * 0.52f + up * 0.36f, middle);
        Around("rear_three_quarter_left", 30f, -fwd * 0.62f - right * 0.52f + up * 0.36f, middle);
        Around("side_left", 30f, -right * 0.95f + up * 0.10f, middle);
        Around("front", 30f, fwd * 0.95f + up * 0.16f, middle);
        Around("front_low_close", 40f, fwd * 0.42f - up * 0.03f, middle + fwd * 0.2f);
        cam.fieldOfView = 30f;
        Shot(cam, dir, "top_down", middle + up * 1.25f, Quaternion.LookRotation(-up, fwd));

        Vector3 sensors = body.position + fwd * 0.2f + up * 0.11f; // between the camera and the LiDAR
        Vector3 eye = sensors + fwd * 0.22f - right * 0.18f + up * 0.12f;
        cam.fieldOfView = 35f;
        Shot(cam, dir, "sensors_close", eye, Quaternion.LookRotation(sensors - eye, up));

        // each sidepod's NEORACER, the left one further back; the scene's light falls on the right,
        // so the car turns half round for the left one (the scene is not saved)
        Vector3 word = body.position + fwd * 0.156f + up * 0.012f + right * 0.066f;
        eye = word + right * 0.3f + up * 0.04f;
        Shot(cam, dir, "sidepod_right_close", eye, Quaternion.LookRotation(word - eye, up));
        Transform root = body.root;
        root.RotateAround(middle, up, 180f);
        word = body.position + body.forward * 0.124f + body.up * 0.012f - body.right * 0.066f;
        eye = word - body.right * 0.3f + body.up * 0.04f;
        Shot(cam, dir, "sidepod_left_close", eye, Quaternion.LookRotation(word - eye, body.up));
        root.RotateAround(middle, up, 180f);

        Vector3 wheel = car.FrontLeftWheelTransform.position;
        eye = wheel - right * 0.25f + fwd * 0.08f + up * 0.06f;
        Shot(cam, dir, "wheel_front_left_close", eye, Quaternion.LookRotation(wheel - eye, up));

        (Vector3 led, Vector3 facing) = Led(body);
        eye = led + facing * 0.18f + up * 0.03f;
        Shot(cam, dir, "led_matrix_close", eye, Quaternion.LookRotation(led - eye, up));
        EditorApplication.Exit(0);
    }

    // The LED matrix's middle and the way its dots face: the area-weighted normal of the
    // triangles drawn with the led_dot material.
    static (Vector3, Vector3) Led(Transform car)
    {
        foreach (MeshRenderer r in car.GetComponentsInChildren<MeshRenderer>(true))
        {
            Material[] mats = r.sharedMaterials;
            int sub = Array.FindIndex(mats, m => m != null && m.name == "led_dot");
            if (sub < 0) continue;
            Mesh mesh = r.GetComponent<MeshFilter>().sharedMesh;
            Vector3[] v = mesh.vertices;
            int[] t = mesh.GetTriangles(sub);
            Vector3 n = Vector3.zero;
            for (int i = 0; i < t.Length; i += 3)
                n += Vector3.Cross(v[t[i + 1]] - v[t[i]], v[t[i + 2]] - v[t[i]]);
            Vector3 facing = r.transform.TransformDirection(n).normalized;
            Debug.Log("SHOT LED matrix at " + r.bounds.center.ToString("F4") + " facing " + facing.ToString("F3"));
            return (r.bounds.center, facing);
        }
        throw new Exception("NeoRacerLookShots: no part of the car is drawn with the led_dot material");
    }

    static void Shot(Camera cam, string dir, string name, Vector3 at, Quaternion look)
    {
        cam.transform.SetPositionAndRotation(at, look);
        var rt = new RenderTexture(Width, Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        cam.targetTexture = rt;
        for (int i = 0; i < Frames; i++)
        {
            cam.Render();
            // HDRP reads the sky's ambient light back from the GPU asynchronously, and outside the
            // player loop nothing completes that; without it the car had only sunlight, its shaded side black
            AsyncGPUReadback.WaitAllRequests();
        }
        RenderTexture.active = rt;
        var tex = new Texture2D(Width, Height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, Width, Height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        cam.targetTexture = null;
        File.WriteAllBytes(Path.Combine(dir, name + ".png"), tex.EncodeToPNG());
        UnityEngine.Object.DestroyImmediate(tex);
        rt.Release();
        Debug.Log("SHOT " + name);
    }
}
