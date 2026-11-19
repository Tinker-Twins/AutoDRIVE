using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Rendering.HighDefinition;
using UnityEngine;

// Puts the NeoRacer's finished CAD model (neoracer-model's parametric car, as
// Models/Vehicle/NeoRacer/CAD/NeoRacer CAD.fbx, made by Tools/neoracer_cad_mesh.py
// and Tools/neoracer_cad_fbx.py) on the NeoRacer prefab, with what the CAD
// measures:
// - the drawn model, in place of the earlier one;
// - the wheels' places: each wheel collider rests where the model's wheel is
//   (the CAD's track, wheelbase and stance: its front axle sits 5.7 mm lower,
//   so the car rests 1.1 degrees nose up);
// - the body's collider, the convex hull of the CAD's body, in place of the
//   earlier model's nine; and the tires' colliders, the tires' size;
// - the camera's place and lean, and the LiDAR's place.
// Every other sensor, script and reference stays. NeoRacerPhysics.Apply runs
// after it: the car's physics stands on the wheels' places. Batchmode:
//   -executeMethod NeoRacerModelSwap.Report   measurements only, no change
//   -executeMethod NeoRacerModelSwap.Apply    import, materials, prefab, audit
//   -executeMethod NeoRacerModelSwap.AddCoSimWheels
//                                             wheels that roll and steer while
//                                             co-simulation poses the car
public static class NeoRacerModelSwap
{
    const string Dir = "Assets/Models/Vehicle/NeoRacer/CAD";
    const string Fbx = Dir + "/NeoRacer CAD.fbx";
    const string Json = Dir + "/NeoRacer CAD.json";
    // The body's hull as a mesh of its own: a mesh collider cooks its mesh when the
    // player loads (Project Settings do not prebake them), so it must be readable,
    // and the model's meshes are not (they would take twice the memory)
    const string HullAsset = Dir + "/NeoRacer CAD Collider.asset";
    const string MaterialDir = Dir + "/Materials";
    const string TextureDir = Dir + "/Textures";
    const string OldFbx = "Assets/Models/Vehicle/NeoRacer/NeoRacer.fbx";
    const string Prefab = "Assets/Prefabs/NeoRacer/NeoRacer.prefab";
    const string Templates = "Assets/Models/Vehicle/NeoRacer/Materials/";
    const string ModelName = "NeoRacer Model";
    const string StreamedCamera = "Preview", DashCamera = "Dashcam";
    // AutoDRIVE's NeoRacer LED shines at 300 nits; the model's lights have
    // emission strengths 1 to 4, so 100 nits per unit of strength.
    const float NitsPerStrength = 100f;
    static readonly string[] Corners = { "FL", "FR", "RL", "RR" };

    [Serializable] class Spec
    {
        public string source; public int triangles; public string[] parts; public ColliderSpec collider; public WheelSpec[] wheels;
        public float[] camera; public float camera_lean_degrees; public float[] lidar; public MatSpec[] materials;
    }
    [Serializable] class ColliderSpec { public float[] vertices; public int[] triangles; public float body_outside_max; }
    [Serializable] class WheelSpec { public string name; public float[] centre; public float radius; public float width; }
    [Serializable] class MatSpec
    {
        public string name; public float[] baseColor; public float metallic; public float roughness;
        public float[] emission; public float emissionStrength; public float alpha; public bool doubleSided; public string texture;
    }

    static Spec LoadSpec() => JsonUtility.FromJson<Spec>(File.ReadAllText(Json));

    // A point in the model's own frame: the spec is Blender's frame (x right,
    // y forward, z up), the model Unity's (x right, y up, z forward).
    static Vector3 ModelPoint(float[] p) => new Vector3(p[0], p[2], p[1]);
    static Vector3 ModelCentre(WheelSpec w) => ModelPoint(w.centre);

    static WheelCollider[] Colliders(VehicleController vc) => new[] {
        vc.FrontLeftWheelCollider, vc.FrontRightWheelCollider, vc.RearLeftWheelCollider, vc.RearRightWheelCollider };

    // Where a wheel's centre sits at rest, in the vehicle's frame: the
    // collider's centre lowered by the suspension travel left at its target
    // position (0 fully out, 1 fully in).
    static Vector3 RestCentre(Transform vehicle, WheelCollider c) =>
        vehicle.InverseTransformPoint(c.transform.TransformPoint(c.center) -
            c.transform.up * c.suspensionDistance * (1f - c.suspensionSpring.targetPosition));

    static string V(Vector3 v) => string.Format("({0:F4}, {1:F4}, {2:F4})", v.x, v.y, v.z);

    public static void Report()
    {
        Spec spec = LoadSpec();
        GameObject root = PrefabUtility.LoadPrefabContents(Prefab);
        VehicleController vc = root.GetComponentInChildren<VehicleController>(true);
        Transform vehicle = vc.VehicleRigidBody.transform;
        Debug.Log("SWAP prefab root " + root.name + ", vehicle " + vehicle.name + " lossyScale " + V(vehicle.lossyScale) +
                  " Wheelbase " + vc.Wheelbase + " mm TrackWidth " + vc.TrackWidth + " mm WheelRadius " + vc.WheelRadius);
        Transform[] old = { vc.FrontLeftWheelTransform, vc.FrontRightWheelTransform, vc.RearLeftWheelTransform, vc.RearRightWheelTransform };
        WheelCollider[] cols = Colliders(vc);
        for (int i = 0; i < 4; i++)
        {
            WheelCollider c = cols[i];
            Debug.Log("SWAP " + Corners[i] + " collider " + c.name + " rest centre " + V(RestCentre(vehicle, c)) +
                      " radius " + c.radius + " suspension " + c.suspensionDistance + " target " + c.suspensionSpring.targetPosition +
                      " rot " + V((Quaternion.Inverse(vehicle.rotation) * c.transform.rotation).eulerAngles) +
                      " | visual " + old[i].name + " at " + V(vehicle.InverseTransformPoint(old[i].position)) +
                      " rot " + V((Quaternion.Inverse(vehicle.rotation) * old[i].rotation).eulerAngles));
        }
        foreach (WheelSpec w in spec.wheels)
            Debug.Log("SWAP model " + w.name + " centre " + V(ModelCentre(w)) + " radius " + w.radius + " width " + w.width);
        int oldRenderers = root.GetComponentsInChildren<MeshFilter>(true).Count(f => f.sharedMesh != null && AssetDatabase.GetAssetPath(f.sharedMesh) == OldFbx);
        int oldColliders = root.GetComponentsInChildren<MeshCollider>(true).Count(m => m.sharedMesh != null && AssetDatabase.GetAssetPath(m.sharedMesh) == OldFbx);
        Debug.Log("SWAP earlier model: " + oldRenderers + " drawn meshes, " + oldColliders + " mesh colliders");
        PrefabUtility.UnloadPrefabContents(root);
        EditorApplication.Exit(0);
    }

    public static void Apply()
    {
        Spec spec = LoadSpec();
        ImportModel(spec);
        SwapPrefab(spec);
        int missing = Audit();
        EditorApplication.Exit(missing == 0 ? 0 : 1);
    }

    public static void AddCoSimWheels()
    {
        GameObject root = PrefabUtility.LoadPrefabContents(Prefab);
        CoSimManager cosim = root.GetComponentInChildren<CoSimManager>(true);
        // Run again, it rewires the one CoSimWheels: two would both write the wheels
        CoSimWheels wheels = cosim.GetComponent<CoSimWheels>();
        if (wheels == null) wheels = cosim.gameObject.AddComponent<CoSimWheels>();
        wheels.CoSim = cosim;
        wheels.Vehicle = root.GetComponentInChildren<VehicleController>(true);
        Debug.Log("SWAP CoSimWheels on " + cosim.name + ", vehicle " + wheels.Vehicle.name);
        PrefabUtility.SaveAsPrefabAsset(root, Prefab);
        PrefabUtility.UnloadPrefabContents(root);
        int missing = Audit();
        EditorApplication.Exit(missing == 0 ? 0 : 1);
    }

    static Color Gamma(float[] c, float alpha) =>
        new Color(Mathf.LinearToGammaSpace(c[0]), Mathf.LinearToGammaSpace(c[1]), Mathf.LinearToGammaSpace(c[2]), alpha);

    static void ImportModel(Spec spec)
    {
        AssetDatabase.ImportAsset(Fbx);
        ModelImporter mi = (ModelImporter)AssetImporter.GetAtPath(Fbx);
        mi.globalScale = 1f;
        mi.useFileScale = true;
        mi.bakeAxisConversion = false;
        mi.importAnimation = false;
        mi.importCameras = false;
        mi.importLights = false;
        mi.isReadable = false;
        mi.meshCompression = ModelImporterMeshCompression.Off;
        mi.materialImportMode = ModelImporterMaterialImportMode.ImportViaMaterialDescription;
        mi.materialLocation = ModelImporterMaterialLocation.InPrefab;

        // A fresh folder each run: a material an earlier build made and this one does not stays out
        AssetDatabase.DeleteAsset(MaterialDir);
        AssetDatabase.CreateFolder(Dir, "Materials");
        Material opaque = AssetDatabase.LoadAssetAtPath<Material>(Templates + "Low Gloss Plastic - Black.mat");
        Material clear = AssetDatabase.LoadAssetAtPath<Material>(Templates + "Camera Lens.mat");
        Material light = AssetDatabase.LoadAssetAtPath<Material>(Templates + "LED - Red.mat");
        foreach (MatSpec s in spec.materials)
        {
            bool emissive = s.emissionStrength > 0f && s.emission.Max() > 0f;
            bool transparent = s.alpha < 0.999f;
            Material m = new Material(transparent ? clear : emissive ? light : opaque) { name = s.name };
            m.SetColor("_BaseColor", Gamma(s.baseColor, transparent ? s.alpha : 1f));
            m.SetFloat("_Metallic", s.metallic);
            m.SetFloat("_Smoothness", 1f - s.roughness);
            m.SetFloat("_DoubleSidedEnable", s.doubleSided ? 1f : 0f);
            Texture2D tex = s.texture != "" ? AssetDatabase.LoadAssetAtPath<Texture2D>(TextureDir + "/" + s.texture) : null;
            if (s.texture != "" && tex == null)
                throw new Exception("texture " + s.texture + " for " + s.name + " is not in " + TextureDir);
            m.SetTexture("_BaseColorMap", tex);
            if (emissive)
            {
                float nits = NitsPerStrength * s.emissionStrength;
                m.SetFloat("_UseEmissiveIntensity", 1f);
                m.SetFloat("_EmissiveIntensity", nits);
                m.SetColor("_EmissiveColorLDR", Gamma(s.emission, 1f));
                m.SetColor("_EmissiveColor", new Color(s.emission[0] * nits, s.emission[1] * nits, s.emission[2] * nits, nits));
                m.SetTexture("_EmissiveColorMap", tex);
            }
            HDShaderUtils.ResetMaterialKeywords(m);
            AssetDatabase.CreateAsset(m, MaterialDir + "/" + s.name + ".mat");
            mi.AddRemap(new AssetImporter.SourceAssetIdentifier(typeof(Material), s.name), m);
        }
        mi.SaveAndReimport();

        GameObject model = AssetDatabase.LoadAssetAtPath<GameObject>(Fbx);
        int tris = 0;
        foreach (MeshFilter f in model.GetComponentsInChildren<MeshFilter>(true))
            tris += f.sharedMesh.triangles.Length / 3;
        Debug.Log("SWAP imported " + Fbx + ": " + model.transform.childCount + " parts, " + tris + " triangles (spec " + spec.triangles + ")");
        foreach (string corner in Corners)
        {
            Transform w = model.transform.Find("Wheel " + corner);
            Debug.Log("SWAP model Wheel " + corner + " at " + V(w.localPosition) + " rot " + V(w.localRotation.eulerAngles) +
                      " bounds centre " + V(w.GetComponent<MeshFilter>().sharedMesh.bounds.center) + " size " + V(w.GetComponent<MeshFilter>().sharedMesh.bounds.size));
        }
        List<string> unmapped = model.GetComponentsInChildren<MeshRenderer>(true).SelectMany(r => r.sharedMaterials)
            .Where(m => m == null || AssetDatabase.GetAssetPath(m) == Fbx).Select(m => m == null ? "null" : m.name).Distinct().ToList();
        if (unmapped.Count > 0)
            throw new Exception("materials not remapped to " + MaterialDir + ": " + string.Join(", ", unmapped));

        // The hull, in the model's frame; Blender's frame is right handed and Unity's left,
        // so each triangle turns the other way round to face out
        float[] v = spec.collider.vertices;
        int[] t = spec.collider.triangles;
        Mesh hull = new Mesh { name = "NeoRacer CAD Collider" };
        hull.vertices = Enumerable.Range(0, v.Length / 3).Select(i => ModelPoint(new[] { v[3 * i], v[3 * i + 1], v[3 * i + 2] })).ToArray();
        hull.triangles = Enumerable.Range(0, t.Length).Select(i => t[i - i % 3 + (3 - i % 3) % 3]).ToArray();
        hull.RecalculateNormals();
        hull.RecalculateBounds();
        AssetDatabase.DeleteAsset(HullAsset);
        AssetDatabase.CreateAsset(hull, HullAsset);
        Debug.Log("SWAP body collider " + HullAsset + ": " + t.Length / 3 + " triangles, the body " +
                  (1000f * spec.collider.body_outside_max).ToString("F1") + " mm outside it at most");
    }

    static void SwapPrefab(Spec spec)
    {
        GameObject root = PrefabUtility.LoadPrefabContents(Prefab);
        VehicleController vc = root.GetComponentInChildren<VehicleController>(true);
        Transform vehicle = vc.VehicleRigidBody.transform;

        // The tires' colliders hang on the drawn wheels (NeoRacerPhysics): they leave the
        // earlier model's wheels before it goes
        MeshCollider[] tires = root.GetComponentsInChildren<MeshCollider>(true).Where(c => c.name == "Collider").ToArray();
        if (tires.Length != 4)
            throw new Exception("expected 4 tire colliders named Collider, found " + tires.Length);
        foreach (MeshCollider tire in tires)
            tire.transform.SetParent(vehicle, true);
        Transform earlier = vehicle.Find(ModelName);
        if (earlier != null)
            UnityEngine.Object.DestroyImmediate(earlier.gameObject);

        // Upstream's model stops being drawn and colliding; its objects stay, as other
        // components and the scenes point at them.
        int removed = 0, uncollided = 0;
        foreach (MeshFilter f in root.GetComponentsInChildren<MeshFilter>(true))
        {
            if (f.sharedMesh == null || AssetDatabase.GetAssetPath(f.sharedMesh) != OldFbx)
                continue;
            MeshRenderer r = f.GetComponent<MeshRenderer>();
            if (r != null)
                UnityEngine.Object.DestroyImmediate(r);
            UnityEngine.Object.DestroyImmediate(f);
            removed++;
        }
        foreach (MeshCollider c in root.GetComponentsInChildren<MeshCollider>(true))
        {
            if (c.sharedMesh == null || AssetDatabase.GetAssetPath(c.sharedMesh) != OldFbx)
                continue;
            UnityEngine.Object.DestroyImmediate(c);
            uncollided++;
        }

        // The new model under the vehicle, its rear axle where the vehicle's rests.
        GameObject model = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(Fbx), vehicle);
        PrefabUtility.UnpackPrefabInstance(model, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
        model.name = ModelName;
        WheelCollider[] cols = Colliders(vc);
        Vector3 rearAxle = (RestCentre(vehicle, cols[2]) + RestCentre(vehicle, cols[3])) / 2f;
        WheelSpec rl = spec.wheels.First(w => w.name == "Wheel RL"), rr = spec.wheels.First(w => w.name == "Wheel RR");
        Vector3 modelRearAxle = (ModelCentre(rl) + ModelCentre(rr)) / 2f;
        model.transform.localRotation = Quaternion.identity;
        model.transform.localScale = new Vector3(1f / vehicle.lossyScale.x, 1f / vehicle.lossyScale.y, 1f / vehicle.lossyScale.z);
        model.transform.localPosition = rearAxle - Vector3.Scale(modelRearAxle, model.transform.localScale);
        Func<float[], Vector3> world = p => model.transform.TransformPoint(ModelPoint(p));

        // Each wheel collider rests at its wheel's centre
        for (int i = 0; i < 4; i++)
        {
            Vector3 was = RestCentre(vehicle, cols[i]);
            Transform wheel = model.transform.Find("Wheel " + Corners[i]);
            cols[i].transform.position += wheel.position - vehicle.TransformPoint(was);
            Debug.Log("SWAP " + Corners[i] + " wheel collider rests at " + V(RestCentre(vehicle, cols[i])) + ", was " + V(was));
        }

        // The tires' colliders on the new wheels, each the tire's size (upstream's cylinder:
        // 1 across, 2 long, on its y axis, which lies along the axle)
        foreach (MeshCollider tire in tires)
        {
            WheelSpec w = spec.wheels.OrderBy(s => (world(s.centre) - tire.transform.position).sqrMagnitude).First();
            tire.transform.SetParent(model.transform.Find(w.name), false);
            tire.transform.localPosition = Vector3.zero;
            tire.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
            tire.transform.localScale = new Vector3(2f * w.radius, w.width / 2f, 2f * w.radius);
        }

        // The body collides as the CAD body's hull (not named Collider: those are the tires')
        GameObject hull = new GameObject("Body Collider");
        hull.transform.SetParent(model.transform, false);
        MeshCollider body = hull.AddComponent<MeshCollider>();
        body.sharedMesh = AssetDatabase.LoadAssetAtPath<Mesh>(HullAsset);
        body.convex = true;

        // The camera (the one streamed, and the dashcam beside it) and the LiDAR where the CAD has them
        Camera[] cams = root.GetComponentsInChildren<Camera>(true);
        Camera streamed = cams.Single(c => c.name == StreamedCamera), dash = cams.Single(c => c.name == DashCamera);
        streamed.transform.SetPositionAndRotation(world(spec.camera), vehicle.rotation * Quaternion.Euler(-spec.camera_lean_degrees, 0f, 0f));
        dash.transform.position = world(spec.camera);
        LIDAR lidar = root.GetComponentInChildren<LIDAR>(true);
        lidar.transform.position = world(spec.lidar);

        // The wheels the vehicle drives: each frame it sets their pose from
        // the wheel colliders, so they spin, steer and ride the suspension.
        SerializedObject so = new SerializedObject(vc);
        string[] fields = { "FrontLeftWheelTransform", "FrontRightWheelTransform", "RearLeftWheelTransform", "RearRightWheelTransform" };
        for (int i = 0; i < 4; i++)
            so.FindProperty(fields[i]).objectReferenceValue = model.transform.Find("Wheel " + Corners[i]);
        so.ApplyModifiedPropertiesWithoutUndo();

        Debug.Log("SWAP prefab: upstream's drawn meshes removed " + removed + ", its colliders " + uncollided + "; " + ModelName + " at " +
                  V(model.transform.localPosition) + " (rear axle " + V(rearAxle) + "), wheels now " +
                  string.Join(", ", fields.Select(f => so.FindProperty(f).objectReferenceValue.name)) +
                  "; camera at " + V(vehicle.InverseTransformPoint(streamed.transform.position)) + " leaning " + spec.camera_lean_degrees +
                  " degrees up, LiDAR at " + V(vehicle.InverseTransformPoint(lidar.transform.position)));
        PrefabUtility.SaveAsPrefabAsset(root, Prefab);
        PrefabUtility.UnloadPrefabContents(root);
    }

    // Object references that point at something gone, in the prefab and the
    // NeoRacer scene.
    static int Audit()
    {
        int missing = 0;
        GameObject root = PrefabUtility.LoadPrefabContents(Prefab);
        missing += CountMissing(root.GetComponentsInChildren<Component>(true), "prefab");
        PrefabUtility.UnloadPrefabContents(root);
        UnityEditor.SceneManagement.EditorSceneManager.OpenScene("Assets/Scenes/NeoRacer - AuE-X930.unity");
        missing += CountMissing(UnityEngine.Object.FindObjectsOfType<Component>(true), "scene");
        Debug.Log("SWAP audit: " + missing + " missing references");
        return missing;
    }

    static int CountMissing(Component[] components, string where)
    {
        int n = 0;
        foreach (Component c in components)
        {
            if (c == null)
            {
                Debug.LogWarning("SWAP " + where + ": a missing script");
                n++;
                continue;
            }
            SerializedProperty p = new SerializedObject(c).GetIterator();
            while (p.NextVisible(true))
            {
                if (p.propertyType == SerializedPropertyType.ObjectReference && p.objectReferenceValue == null && p.objectReferenceInstanceIDValue != 0)
                {
                    Debug.LogWarning("SWAP " + where + ": " + c.gameObject.name + "/" + c.GetType().Name + "." + p.propertyPath + " is missing");
                    n++;
                }
            }
        }
        return n;
    }
}
