using System;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEditor;

// Builds a first-pass NeoRacer vehicle prefab by transplanting the NeoRacer
// game-ready mesh onto a copy of the F1TENTH rig (WheelColliders + sensors +
// VehicleController + SocketIO bridge), then retargeting geometry + LiDAR to
// NeoRacer's real spec. Run via menu or:
//   Unity -batchmode -quit -projectPath . -executeMethod NeoRacerBuilder.Build
public static class NeoRacerBuilder
{
    const string NEO_DIR   = "Assets/Models/Vehicle/NeoRacer";
    const string NEO_FBX   = NEO_DIR + "/NeoRacer.fbx";
    // Source defaults to the FBX committed in this repository, so the rebuild
    // is fully self-contained. Set NEORACER_FBX_SRC to import a fresh CAD
    // export instead.
    static string SrcFbx =>
        Environment.GetEnvironmentVariable("NEORACER_FBX_SRC") ?? NEO_FBX;
    const string F1_PREFAB = "Assets/Prefabs/F1TENTH/F1TENTH.prefab";
    const string OUT_DIR   = "Assets/Prefabs/NeoRacer";
    const string OUT_PREFAB = OUT_DIR + "/NeoRacer.prefab";

    // NeoRacer spec
    const float MASS = 2.8f;           // kg (< 3)
    const float WHEEL_RADIUS = 0.045f; // m (90 mm dia)
    // VehicleController takes FULL wheelbase/track in mm (F1TENTH prefab: 324/236)
    const float WHEELBASE_MM = 288f;
    const float TRACK_MM = 264f;
    // Sensor mounts in vehicle space (origin = rear axle on ground, +Z forward).
    // Camera from CAD part "000_1" at nose center (lens proud of the front face);
    // LiDAR from the dome window center. IMU/IPS still estimated.
    static readonly Vector3 LIDAR_POS = new Vector3(0f, 0.165f, 0.100f);
    static readonly Vector3 IMU_POS   = new Vector3(0f, 0.050f, 0.100f);
    static readonly Vector3 CAM_POS   = new Vector3(0f, 0.092f, 0.310f);
    const string RUBBER_MAT = NEO_DIR + "/NeoRacer_Rubber.mat";
    // LakiBeam1
    const float LIDAR_RATE = 30f, LIDAR_MIN_R = 0.06f, LIDAR_MAX_R = 25f;
    const float LIDAR_MIN_A = -135f, LIDAR_MAX_A = 135f, LIDAR_RES = 0.25f;

    [MenuItem("AutoDRIVE/Build NeoRacer Prefab")]
    public static void Build()
    {
        Debug.Log("[NeoRacer] Build start");

        ImportFbx();

        GameObject root = PrefabUtility.LoadPrefabContents(F1_PREFAB);
        if (root == null) { Debug.LogError("[NeoRacer] F1TENTH prefab not found at " + F1_PREFAB); return; }

        // The F1TENTH asset ships with a baked root pose (-3.0, 0.01, 0.74). Zero it so
        // world == root-local for every placement below; scene spawns override it anyway.
        root.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
        root.name = "NeoRacer";

        var vc = root.GetComponentInChildren<VehicleController>(true);
        var rb = root.GetComponentInChildren<Rigidbody>(true);
        var lidar = root.GetComponentInChildren<LIDAR>(true);
        if (vc == null || rb == null) { Debug.LogError("[NeoRacer] Missing VehicleController/Rigidbody on F1TENTH rig"); return; }

        // First-class rebuild: DELETE all F1TENTH geometry, keep only the functional rig.
        // A top-level node survives if it (or its subtree) carries a functional component,
        // or is one of the named frame markers.
        var keepNames = new System.Collections.Generic.HashSet<string> { "Rear Axle Center", "Front Axle Center" };
        bool Functional(GameObject g) => g.GetComponentsInChildren<Component>(true).Any(c =>
            c is WheelCollider || c is Camera || c is LIDAR || c is IMU || c is GPS ||
            c is WheelEncoder || c is ReflectionProbe);
        var doomed = root.transform.Cast<Transform>()
            .Where(t => !keepNames.Contains(t.name) && !Functional(t.gameObject))
            .Select(t => t.gameObject).ToList();
        foreach (var go in doomed) { Debug.Log($"[NeoRacer] delete F1TENTH node: {go.name}"); Object.DestroyImmediate(go); }
        int stripped = 0;
        foreach (var mr in root.GetComponentsInChildren<MeshRenderer>(true))
        { var mf = mr.GetComponent<MeshFilter>(); Object.DestroyImmediate(mr); if (mf != null) Object.DestroyImmediate(mf); stripped++; }
        Debug.Log($"[NeoRacer] deleted {doomed.Count} F1TENTH nodes, stripped {stripped} leftover mesh renderers");

        // Bring in NeoRacer visual (FBX origin = rear-axle center = prefab origin)
        GameObject fbxAsset = AssetDatabase.LoadAssetAtPath<GameObject>(NEO_FBX);
        if (fbxAsset == null) { Debug.LogError("[NeoRacer] NeoRacer FBX not imported at " + NEO_FBX); return; }
        GameObject neo = (GameObject)PrefabUtility.InstantiatePrefab(fbxAsset);
        neo.name = "NeoRacer_Visual";
        PrefabUtility.UnpackPrefabInstance(neo, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction); // allow reparenting wheels out
        neo.transform.SetParent(root.transform, false);
        neo.transform.localPosition = Vector3.zero;
        neo.transform.localRotation = Quaternion.identity;
        neo.transform.localScale = Vector3.one;

        Transform wFL = FindDeep(neo.transform, "Wheel_FL");
        Transform wFR = FindDeep(neo.transform, "Wheel_FR");
        Transform wRL = FindDeep(neo.transform, "Wheel_RL");
        Transform wRR = FindDeep(neo.transform, "Wheel_RR");
        Transform body = FindDeep(neo.transform, "Body");
        if (wFL == null || wFR == null || wRL == null || wRR == null)
        { Debug.LogError("[NeoRacer] Could not find all 4 Wheel_* nodes in FBX"); return; }

        // Report imported size for scale sanity
        if (body != null)
        {
            var r = body.GetComponent<Renderer>();
            if (r != null) Debug.Log($"[NeoRacer] Body world bounds size = {r.bounds.size} (expect ~0.27 x 0.22 x 0.45)");
        }

        // Normalize placement from the actual wheel geometry (Unity's FBX import landed
        // the visual ~3 m off-origin and rotated; don't trust it). Rigid-transform the
        // whole visual so the rear-axle center sits at the rig origin, the length axis
        // faces +Z (VehicleController drives along +Z), and up stays +Y.
        Debug.Log($"[NeoRacer] wheel pos (raw import) FL={wFL.position} FR={wFR.position} RL={wRL.position} RR={wRR.position}");
        Vector3 rearMid  = (wRL.position + wRR.position) * 0.5f;
        Vector3 frontMid = (wFL.position + wFR.position) * 0.5f;
        Vector3 fwd = frontMid - rearMid; fwd.y = 0f; fwd.Normalize();
        Quaternion R = Quaternion.Inverse(Quaternion.LookRotation(fwd, Vector3.up)); // fwd->+Z, up->+Y
        neo.transform.rotation = R * neo.transform.rotation;
        neo.transform.position = R * (neo.transform.position - rearMid);
        // F1TENTH rig convention: root = ground contact plane, so hubs sit one wheel
        // radius above root. Without this lift the wheels spawn buried in the track
        // and PhysX ejects the car on Play.
        neo.transform.position += Vector3.up * WHEEL_RADIUS;
        Debug.Log($"[NeoRacer] reframed (rearMid->origin+r). wheel pos now FL={wFL.position} FR={wFR.position} RL={wRL.position} RR={wRR.position}");

        // UpdateWheelPose() stomps the bound transform's world rotation with the collider
        // pose every frame, which erases the FBX import correction the CAD meshes need.
        // Drive a neutral pivot instead and keep the mesh as a corrected child.
        Transform pFL = MakeWheelPivot(root.transform, wFL, "WheelPivot_FL");
        Transform pFR = MakeWheelPivot(root.transform, wFR, "WheelPivot_FR");
        Transform pRL = MakeWheelPivot(root.transform, wRL, "WheelPivot_RL");
        Transform pRR = MakeWheelPivot(root.transform, wRR, "WheelPivot_RR");

        Bind(vc.FrontLeftWheelCollider,  pFL, t => vc.FrontLeftWheelTransform  = t);
        Bind(vc.FrontRightWheelCollider, pFR, t => vc.FrontRightWheelTransform = t);
        Bind(vc.RearLeftWheelCollider,   pRL, t => vc.RearLeftWheelTransform   = t);
        Bind(vc.RearRightWheelCollider,  pRR, t => vc.RearRightWheelTransform  = t);

        // The CAD export ships tires/rims/dot-matrix with a literal CAD appearance color
        // "Opaque(249,69,24)" (orange-red, never assigned a real appearance upstream).
        // Swap every slot of it for matte near-black rubber.
        var rubber = AssetDatabase.LoadAssetAtPath<Material>(RUBBER_MAT);
        if (rubber == null)
        {
            rubber = new Material(Shader.Find("HDRP/Lit"));
            rubber.SetColor("_BaseColor", new Color(0.05f, 0.05f, 0.06f));
            rubber.SetFloat("_Smoothness", 0.10f);
            AssetDatabase.CreateAsset(rubber, RUBBER_MAT);
        }
        var matNames = root.GetComponentsInChildren<Renderer>(true)
            .SelectMany(r => r.sharedMaterials).Where(m => m != null)
            .Select(m => m.name).Distinct().ToList();
        Debug.Log("[NeoRacer] visual materials: " + string.Join(" | ", matNames));
        int swapped = 0;
        foreach (var rend in root.GetComponentsInChildren<Renderer>(true))
        {
            var mats = rend.sharedMaterials;
            bool changed = false;
            for (int i = 0; i < mats.Length; i++)
                if (mats[i] != null && (mats[i].name.StartsWith("Opaque(249") || mats[i].name.StartsWith("NeoRacer_Matrix") || mats[i].name.StartsWith("NeoRacer_SideCover"))) { mats[i] = rubber; changed = true; swapped++; }
            if (changed) rend.sharedMaterials = mats;
        }
        // Tires ship as CAD 'Plastic - Glossy (Red)'; real tires are black rubber.
        // Swap on the wheel meshes ONLY -- the body legitimately uses red accents.
        foreach (var pivotName in new[] { "WheelPivot_FL", "WheelPivot_FR", "WheelPivot_RL", "WheelPivot_RR" })
        {
            var pivot = root.transform.Find(pivotName);
            if (pivot == null) continue;
            foreach (var rend in pivot.GetComponentsInChildren<Renderer>(true))
            {
                var mats = rend.sharedMaterials;
                bool ch = false;
                for (int i = 0; i < mats.Length; i++)
                    if (mats[i] != null && mats[i].name.Contains("Glossy (Red)")) { mats[i] = rubber; ch = true; swapped++; }
                if (ch) rend.sharedMaterials = mats;
            }
        }
        Debug.Log($"[NeoRacer] swapped {swapped} red slots (Opaque dot-matrix + tire Glossy Red) -> matte black rubber");

        // Physics + steering geometry; every other tuning value inherited from the F1TENTH rig
        rb.mass = MASS;
        vc.Wheelbase = WHEELBASE_MM;
        vc.TrackWidth = TRACK_MM;
        vc.WheelRadius = WHEEL_RADIUS;
        vc.COM = new Vector3(0f, 0.04f, 0.13f);
        Debug.Log($"[NeoRacer] mass={rb.mass} WB={vc.Wheelbase}mm track={vc.TrackWidth}mm r={vc.WheelRadius} COM={vc.COM}");

        // Body collision shell from NeoRacer body bounds (replaces F1TENTH's deleted geometry)
        var bodyR = body != null ? body.GetComponent<Renderer>() : null;
        if (bodyR == null) { Debug.LogError("[NeoRacer] no Body renderer found; vehicle has no body collider"); }
        else if (bodyR.bounds.size.y > 0.30f || bodyR.bounds.min.y < -0.01f)
        {
            // A sane NeoRacer body is ~0.22 m tall and sits on/above the ground plane.
            // Bad bounds mean a broken FBX conversion; a collider built from them beaches
            // the car on its own shell. Fail loudly instead.
            Debug.LogError($"[NeoRacer] Body bounds insane (center={bodyR.bounds.center} size={bodyR.bounds.size}) -- skipping body collider, FIX THE FBX CONVERSION");
        }
        else
        {
            var box = root.AddComponent<BoxCollider>();
            box.center = bodyR.bounds.center; // root is at identity during build: world == local
            box.size = bodyR.bounds.size;
            Debug.Log($"[NeoRacer] body BoxCollider center={box.center} size={box.size}");
        }

        // Sensor mounts -> NeoRacer positions (same nodes + scripts as every other vehicle)
        MoveNode(root.transform, "Sensors/LIDAR", LIDAR_POS, Quaternion.identity);
        MoveNode(root.transform, "Sensors/IMU", IMU_POS, Quaternion.identity);
        MoveNode(root.transform, "Sensors/IPS", IMU_POS, Quaternion.identity);
        // Low nose camera (9 cm above ground): level horizon, not the F1TENTH mast-cam 10deg tilt
        MoveNode(root.transform, "Sensors/Cameras/Preview", CAM_POS, Quaternion.identity);
        MoveNode(root.transform, "Sensors/Cameras/Dashcam", CAM_POS, Quaternion.identity);
        MoveNode(root.transform, "Sensors/Cameras/Rearcam", new Vector3(0f, 0.12f, -0.03f), Quaternion.Euler(0f, 180f, 0f));

        // GPS reports the vehicle frame (rear-axle center = root), replacing the
        // marker that lived in the deleted Transforms node.
        var gps = root.GetComponentInChildren<GPS>(true);
        if (gps != null) gps.VehicleTransform = root.transform;
        else Debug.LogWarning("[NeoRacer] no GPS component found");

        // RoboRacer scenes drive resets/co-sim through a CoSimManager on the vehicle root
        if (root.GetComponent<CoSimManager>() == null)
        {
            var cosim = root.AddComponent<CoSimManager>();
            cosim.enabled = false;
            Debug.Log("[NeoRacer] added CoSimManager (disabled) to match RoboRacer vehicle rig");
        }

        // NeoRacer has ONE motor encoder (motor shaft, upstream of the diffs), not per-wheel
        // encoders. Both encoder slots read the same rear-axle average, so the two wire
        // fields carry the identical motor-encoder value -- same as the real car's driver.
        var encs = root.GetComponentsInChildren<WheelEncoder>(true);
        foreach (var e in encs)
        {
            e.Wheel = vc.RearLeftWheelCollider;
            e.Wheel2 = vc.RearRightWheelCollider;
        }
        Debug.Log($"[NeoRacer] {encs.Length} encoder slots configured as ONE motor encoder (rear-axle average; PPR={encs.FirstOrDefault()?.PPR} gear={encs.FirstOrDefault()?.GearRatio} = F1TENTH values, NEEDS NeoRacer spec)");

        // LiDAR -> LakiBeam1
        if (lidar != null)
        {
            lidar.ScanRate = LIDAR_RATE;
            lidar.MinimumLinearRange = LIDAR_MIN_R;
            lidar.MaximumLinearRange = LIDAR_MAX_R;
            lidar.MinimumAngularRange = LIDAR_MIN_A;
            lidar.MaximumAngularRange = LIDAR_MAX_A;
            lidar.Resolution = LIDAR_RES;
            int beams = (int)((LIDAR_MAX_A - LIDAR_MIN_A) / LIDAR_RES + 1);
            Debug.Log($"[NeoRacer] LiDAR rate={lidar.ScanRate}Hz range={lidar.MaximumLinearRange}m fov={LIDAR_MAX_A-LIDAR_MIN_A}deg beams={beams}");
        }
        else Debug.LogWarning("[NeoRacer] No LIDAR component found to configure");

        // Receipts: surface anything the teardown disconnected + final rig inventory
        int broken = 0;
        foreach (var comp in root.GetComponentsInChildren<Component>(true))
        {
            if (comp == null) continue;
            var so = new SerializedObject(comp);
            var sp = so.GetIterator();
            while (sp.Next(true))
                if (sp.propertyType == SerializedPropertyType.ObjectReference
                    && sp.objectReferenceValue == null && sp.objectReferenceInstanceIDValue != 0)
                { Debug.LogWarning($"[NeoRacer] broken ref: {comp.gameObject.name}.{comp.GetType().Name}.{sp.propertyPath}"); broken++; }
        }
        Debug.Log($"[NeoRacer] broken-reference audit: {broken}");
        Debug.Log("[NeoRacer] final rig inventory:");
        DumpTree(root.transform, "", 2);

        Directory.CreateDirectory(OUT_DIR);
        bool ok;
        PrefabUtility.SaveAsPrefabAsset(root, OUT_PREFAB, out ok);
        PrefabUtility.UnloadPrefabContents(root);
        AssetDatabase.SaveAssets();
        Debug.Log($"[NeoRacer] Saved prefab -> {OUT_PREFAB} (ok={ok})");
        Debug.Log("[NeoRacer] Build done");
    }

    static Transform MakeWheelPivot(Transform vehicleRoot, Transform wheelMesh, string name)
    {
        var pivot = new GameObject(name).transform;
        pivot.SetParent(vehicleRoot, false);
        pivot.position = wheelMesh.position;
        pivot.rotation = Quaternion.identity;
        wheelMesh.SetParent(pivot, true);
        return pivot;
    }

    static void Bind(WheelCollider col, Transform wheel, System.Action<Transform> setVisual)
    {
        if (col == null) { Debug.LogError("[NeoRacer] null WheelCollider ref on VehicleController"); return; }
        col.transform.position = wheel.position;
        col.radius = WHEEL_RADIUS;
        setVisual(wheel);
    }

    [MenuItem("AutoDRIVE/Rebuild NeoRacer (All)")]
    public static void BuildAll()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        { Debug.LogError("[NeoRacer] Exit Play mode first, then run Rebuild NeoRacer (All)."); return; }
        Build();
        BuildTestScene();
    }

    static void ImportFbx()
    {
        Directory.CreateDirectory(NEO_DIR);
        if (!File.Exists(SrcFbx)) { Debug.LogError("[NeoRacer] source FBX missing: " + SrcFbx); return; }
        if (Path.GetFullPath(SrcFbx) != Path.GetFullPath(NEO_FBX))
            File.Copy(SrcFbx, NEO_FBX, true);
        AssetDatabase.ImportAsset(NEO_FBX, ImportAssetOptions.ForceUpdate);
        var imp = AssetImporter.GetAtPath(NEO_FBX) as ModelImporter;
        if (imp != null)
        {
            imp.useFileScale = true;
            imp.materialImportMode = ModelImporterMaterialImportMode.ImportStandard;
            imp.SaveAndReimport();
        }
        Debug.Log("[NeoRacer] imported FBX -> " + NEO_FBX);
    }

    static Transform FindDeep(Transform t, string name)
    {
        if (t.name == name) return t;
        foreach (Transform c in t)
        {
            var r = FindDeep(c, name);
            if (r != null) return r;
        }
        return null;
    }

    // RoboRacer - Sim Racing: audits clean (F1TENTH.unity ships with a missing
    // Berlin Track prefab upstream) and matches the league environment.
    const string SRC_SCENE = "Assets/Scenes/RoboRacer - Sim Racing.unity";
    const string OUT_SCENE = "Assets/Scenes/NeoRacer - Test.unity";

    [MenuItem("AutoDRIVE/Build NeoRacer Test Scene")]
    public static void BuildTestScene()
    {
        Debug.Log("[NeoRacer] BuildTestScene start");
        var scene = UnityEditor.SceneManagement.EditorSceneManager.OpenScene(SRC_SCENE);

        var oldVc = Object.FindObjectsOfType<VehicleController>(true);
        if (oldVc.Length != 1) { Debug.LogError($"[NeoRacer] expected 1 VehicleController in scene, found {oldVc.Length}"); return; }
        GameObject oldRoot = PrefabUtility.GetOutermostPrefabInstanceRoot(oldVc[0].gameObject);
        if (oldRoot == null) oldRoot = oldVc[0].transform.root.gameObject;
        Debug.Log($"[NeoRacer] old vehicle root: {oldRoot.name} at {oldRoot.transform.position}");

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(OUT_PREFAB);
        var neo = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        neo.name = "NeoRacer";
        neo.transform.SetPositionAndRotation(oldRoot.transform.position, oldRoot.transform.rotation);

        // Vehicle-side components that bind to SCENE objects (HUD lap texts, checkpoint
        // transforms, racetrack name) carry those as instance overrides on the old vehicle;
        // a fresh spawn loses them. Copy the lap timer's full serialized state across.
        var oldLap = oldRoot.GetComponentInChildren<LapTimer>(true);
        var newLap = neo.GetComponentInChildren<LapTimer>(true);
        if (oldLap != null && newLap != null)
        {
            EditorUtility.CopySerialized(oldLap, newLap);
            Debug.Log("[NeoRacer] copied LapTimer scene bindings (HUD texts, checkpoints, racetrack)");
        }
        else Debug.LogWarning($"[NeoRacer] LapTimer pair not found (old={oldLap != null}, new={newLap != null})");

        // Same disease, camera edition: the HUD preview RawImages display the OLD
        // vehicle's RenderTexture assets. Point our cameras at those same RTs.
        foreach (var camPath in new[] { "Sensors/Cameras/Preview", "Sensors/Cameras/Dashcam", "Sensors/Cameras/Rearcam" })
        {
            var ot = oldRoot.transform.Find(camPath);
            var nt = neo.transform.Find(camPath);
            if (ot == null || nt == null) { Debug.LogWarning($"[NeoRacer] camera path missing: {camPath}"); continue; }
            var oc = ot.GetComponent<Camera>();
            var nc = nt.GetComponent<Camera>();
            if (oc != null && nc != null && oc.targetTexture != null)
            {
                nc.targetTexture = oc.targetTexture;
                Debug.Log($"[NeoRacer] camera RT rebind: {camPath} -> {oc.targetTexture.name}");
            }
        }

        int remapped = 0, missing = 0;
        foreach (var go in Object.FindObjectsOfType<GameObject>(true))
        {
            if (go.transform.IsChildOf(oldRoot.transform) || go.transform.IsChildOf(neo.transform)) continue;
            foreach (var comp in go.GetComponents<Component>())
            {
                if (comp == null) continue;
                var so = new SerializedObject(comp);
                var prop = so.GetIterator();
                bool dirty = false;
                while (prop.Next(true))
                {
                    if (prop.propertyType != SerializedPropertyType.ObjectReference || prop.objectReferenceValue == null) continue;
                    Transform t = prop.objectReferenceValue is GameObject g ? g.transform :
                                  prop.objectReferenceValue is Component c ? c.transform : null;
                    if (t == null || !t.IsChildOf(oldRoot.transform)) continue;

                    string path = GetPath(t, oldRoot.transform);
                    Transform nt = path == "" ? neo.transform : neo.transform.Find(path);
                    if (nt == null) { Debug.LogWarning($"[NeoRacer] no match for path '{path}' ({go.name}.{comp.GetType().Name}.{prop.propertyPath})"); missing++; continue; }

                    Object newRef = prop.objectReferenceValue is GameObject ? (Object)nt.gameObject : (Object)nt.GetComponent(prop.objectReferenceValue.GetType());
                    if (newRef == null) { Debug.LogWarning($"[NeoRacer] '{path}' lacks {prop.objectReferenceValue.GetType().Name}"); missing++; continue; }

                    prop.objectReferenceValue = newRef;
                    Debug.Log($"[NeoRacer] remap {go.name}.{comp.GetType().Name}.{prop.propertyPath} -> NeoRacer/{path}");
                    remapped++; dirty = true;
                }
                if (dirty) so.ApplyModifiedPropertiesWithoutUndo();
            }
        }
        Debug.Log($"[NeoRacer] remapped {remapped} references ({missing} unmatched)");

        Object.DestroyImmediate(oldRoot);
        UnityEditor.SceneManagement.EditorSceneManager.SaveScene(scene, OUT_SCENE);
        AssetDatabase.SaveAssets();
        Debug.Log($"[NeoRacer] saved test scene -> {OUT_SCENE}");
        Debug.Log("[NeoRacer] BuildTestScene done");
    }

    static void MoveNode(Transform root, string path, Vector3 localPos, Quaternion localRot)
    {
        var t = root.Find(path);
        if (t == null) { Debug.LogWarning($"[NeoRacer] sensor node missing: {path}"); return; }
        t.localPosition = localPos;
        t.localRotation = localRot;
        Debug.Log($"[NeoRacer] mount {path} -> {localPos}");
    }

    static void DumpTree(Transform t, string indent, int depth)
    {
        Debug.Log($"[NeoRacer]   {indent}{t.name} @ {t.position}");
        if (depth == 0) return;
        foreach (Transform c in t) DumpTree(c, indent + "  ", depth - 1);
    }

    static string GetPath(Transform t, Transform root)
    {
        if (t == root) return "";
        string p = t.name;
        while (t.parent != null && t.parent != root) { t = t.parent; p = t.name + "/" + p; }
        return p;
    }

    [MenuItem("AutoDRIVE/Render NeoRacer Preview")]
    public static void RenderPreview()
    {
        UnityEditor.SceneManagement.EditorSceneManager.NewScene(
            UnityEditor.SceneManagement.NewSceneSetup.EmptyScene,
            UnityEditor.SceneManagement.NewSceneMode.Single);

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(OUT_PREFAB);
        if (prefab == null) { Debug.LogError("[NeoRacer] prefab not found for render"); return; }
        var car = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        car.transform.position = Vector3.zero;

        var lgo = new GameObject("Sun");
        var light = lgo.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 2.0f;
        lgo.transform.rotation = Quaternion.Euler(50f, -40f, 0f);

        var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
        ground.transform.localScale = Vector3.one * 2f;
        ground.transform.position = new Vector3(0f, -0.045f, 0.14f);

        var cgo = new GameObject("Cam");
        var cam = cgo.AddComponent<Camera>();
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.10f, 0.10f, 0.12f);
        cam.fieldOfView = 38f;
        Vector3 target = new Vector3(0f, 0.06f, 0.14f);
        cgo.transform.position = new Vector3(0.5f, 0.38f, -0.42f);
        cgo.transform.LookAt(target);

        int W = 1280, H = 720;
        var rt = new RenderTexture(W, H, 24);
        cam.targetTexture = rt;
        cam.Render();
        RenderTexture.active = rt;
        var tex = new Texture2D(W, H, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, W, H), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        cam.targetTexture = null;
        File.WriteAllBytes(@"C:\tmp\neoracer_unity_preview.png", tex.EncodeToPNG());
        Debug.Log("[NeoRacer] RenderPreview wrote C:\\tmp\\neoracer_unity_preview.png");
    }
}
