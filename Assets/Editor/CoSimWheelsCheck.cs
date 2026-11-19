using System.Reflection;
using UnityEditor;
using UnityEngine;

// Checks CoSimWheels on the NeoRacer prefab. The radius each wheel rolls by is held
// against the mesh itself (its farthest vertex from the axle, which must pass through
// the mesh's origin). The car is then posed along straights and arcs, forward and in
// reverse: a wheel that rolls the right way leaves its tyre's contact point where it
// was (under MaxSlip of the hub's step; a reversed spin moves it twice the step), and
// each front wheel's axle line passes through the turn's centre (Ackermann), or the
// wheel sits at the steering limit when the turn is tighter than the limit allows.
// Parked with a localized pose's jitter the front wheels hold their steer, and a
// co-simulation that stops and starts again begins straight. Batchmode:
//   -executeMethod CoSimWheelsCheck.Run
public static class CoSimWheelsCheck
{
    const string Prefab = "Assets/Prefabs/NeoRacer/NeoRacer.prefab";
    const float Step = 0.002f; // m of rear axle travel per frame
    const int Steps = 80;
    const float MaxSlip = 0.04f; // of the hub's step; one frame's roll leaves 0.02 to 0.03
    const float MaxAxleMiss = 0.001f; // m from the turn's centre
    const float MaxRadiusError = 0.0005f; // m
    const float Jitter = 0.003f; // m, and
    const float JitterYaw = 0.5f; // degrees: a localized pose's noise at rest
    const float MaxParkedSteer = 1f; // degrees the front wheels may move while parked
    static readonly string[] Corners = { "FL", "FR", "RL", "RR" };

    static CoSimWheels wheels;
    static MethodInfo late;
    static Transform body;
    static Transform[] w;
    static float[] radii;
    static Vector3 axle;
    static float limit;

    public static void Run()
    {
        GameObject root = PrefabUtility.LoadPrefabContents(Prefab);
        wheels = root.GetComponentInChildren<CoSimWheels>(true);
        VehicleController vc = wheels.Vehicle;
        limit = vc.SteeringLimit;
        body = vc.VehicleRigidBody.transform;
        w = new[] { vc.FrontLeftWheelTransform, vc.FrontRightWheelTransform, vc.RearLeftWheelTransform, vc.RearRightWheelTransform };
        typeof(CoSimWheels).GetMethod("Start", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(wheels, null);
        late = typeof(CoSimWheels).GetMethod("LateUpdate", BindingFlags.NonPublic | BindingFlags.Instance);
        radii = (float[])typeof(CoSimWheels).GetField("radii", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(wheels);
        axle = (body.InverseTransformPoint(w[2].position) + body.InverseTransformPoint(w[3].position)) / 2;

        bool ok = Radii();
        wheels.CoSim.enabled = true;
        late.Invoke(wheels, null);
        ok &= Drive("straight forward", 0f, 1);
        ok &= Parked();
        ok &= Drive("straight reverse", 0f, -1);
        ok &= Drive("left 1 m forward", -1f, 1);
        ok &= Drive("right 1 m forward", 1f, 1);
        ok &= Drive("right 0.7 m forward", 1f / 0.7f, 1);
        ok &= Drive("left 1 m reverse", -1f, -1);
        ok &= Drive("left 0.5 m forward, past the limit", -1f / 0.5f, 1);
        ok &= Restart();
        Debug.Log("WHEELS " + (ok ? "PASS" : "FAIL"));
        PrefabUtility.UnloadPrefabContents(root);
        EditorApplication.Exit(ok ? 0 : 1);
    }

    static bool Radii()
    {
        bool ok = true;
        string line = "WHEELS radius:";
        for (int i = 0; i < 4; i++)
        {
            Mesh mesh = w[i].GetComponent<MeshFilter>().sharedMesh;
            Vector3 a = (Quaternion.Inverse(w[i].rotation) * body.right).normalized; // the axle, in the mesh's frame
            Vector3 scale = w[i].lossyScale;
            float far = 0f;
            foreach (Vector3 v in mesh.vertices)
            {
                Vector3 p = Vector3.Scale(v, scale);
                far = Mathf.Max(far, (p - Vector3.Dot(p, a) * a).magnitude);
            }
            Vector3 c = Vector3.Scale(mesh.bounds.center, scale);
            float hub = (c - Vector3.Dot(c, a) * a).magnitude;
            line += " " + Corners[i] + " mesh " + far.ToString("F4") + " rolled " + radii[i].ToString("F4") + " hub off " + hub.ToString("F4");
            if (Mathf.Abs(far - radii[i]) > MaxRadiusError || hub > MaxRadiusError) ok = false;
        }
        Debug.Log(line + (ok ? "" : " FAIL"));
        return ok;
    }

    static float Steer(int i) => Vector3.SignedAngle(body.right, w[i].right, body.up);

    // Curvature 1/m of the rear axle's path, right turns positive; direction 1 forward, -1 reverse.
    static bool Drive(string name, float curvature, int direction)
    {
        // A turn is measured over a window of travel. After a change of turn the first window
        // straddles it, and after a change of direction the car first travels back through the
        // open window: the steer has settled within three windows (6 cm) of travel.
        int warmup = 3 * Mathf.CeilToInt(CoSimWheels.TurnWindow / Step) + 1;
        float[] slip = new float[4];
        bool[] atLock = new bool[4]; // a turn tighter than the lock allows: that wheel slides, as the real one would
        float miss = 0f, atLimit = 0f;
        bool past = false;
        for (int s = 0; s < Steps; s++)
        {
            Vector3[] hubs = new Vector3[4], contacts = new Vector3[4], marks = new Vector3[4];
            for (int i = 0; i < 4; i++)
            {
                hubs[i] = w[i].position;
                contacts[i] = hubs[i] - body.up * radii[i];
                marks[i] = w[i].InverseTransformPoint(contacts[i]); // the tyre's material point now on the ground
            }
            float ds = direction * Step;
            if (curvature == 0f)
                body.position += body.forward * ds;
            else
                body.RotateAround(body.TransformPoint(axle) + body.right / curvature, Vector3.up, Mathf.Rad2Deg * curvature * ds);
            late.Invoke(wheels, null);
            if (s < warmup) continue;
            for (int i = 0; i < 4; i++)
                slip[i] = Mathf.Max(slip[i], (w[i].TransformPoint(marks[i]) - contacts[i]).magnitude / (w[i].position - hubs[i]).magnitude);
            if (curvature == 0f) continue;
            Vector3 centre = body.TransformPoint(axle) + body.right / curvature;
            for (int i = 0; i < 2; i++)
            {
                Vector3 rel = body.InverseTransformPoint(w[i].position) - axle;
                float ideal = Mathf.Rad2Deg * Mathf.Atan2(rel.z * curvature, 1 - rel.x * curvature);
                if (Mathf.Abs(ideal) > limit)
                {
                    past = true;
                    atLock[i] = true;
                    atLimit = Mathf.Max(atLimit, Mathf.Abs(Mathf.Abs(Steer(i)) - limit));
                    continue;
                }
                Vector3 d = Vector3.ProjectOnPlane(centre - w[i].position, body.up);
                Vector3 axis = Vector3.ProjectOnPlane(w[i].right, body.up).normalized;
                miss = Mathf.Max(miss, (d - Vector3.Dot(d, axis) * axis).magnitude);
            }
        }
        bool ok = miss <= MaxAxleMiss && atLimit <= 0.01f;
        string line = "WHEELS " + name + ":";
        for (int i = 0; i < 4; i++)
        {
            line += " " + Corners[i] + " slip " + slip[i].ToString("F3") + (atLock[i] ? " (at the lock)" : "") + " steer " + Steer(i).ToString("F1");
            if (slip[i] > MaxSlip && !atLock[i]) ok = false;
        }
        line += curvature == 0f ? "" : "; front axles miss the turn's centre by " + (miss * 1000).ToString("F2") + " mm";
        line += past ? "; inner wheel " + atLimit.ToString("F3") + " deg off the " + limit + " deg limit" : "";
        Debug.Log(line + (ok ? "" : " FAIL"));
        return ok;
    }

    static bool Parked()
    {
        Vector3 position = body.position;
        Quaternion rotation = body.rotation;
        float[] start = { Steer(0), Steer(1) };
        float moved = 0f;
        System.Random random = new System.Random(1);
        float U() => (float)(random.NextDouble() * 2 - 1);
        for (int s = 0; s < 120; s++)
        {
            body.SetPositionAndRotation(position + new Vector3(U() * Jitter, 0f, U() * Jitter),
                                        rotation * Quaternion.Euler(0f, U() * JitterYaw, 0f));
            late.Invoke(wheels, null);
            for (int i = 0; i < 2; i++) moved = Mathf.Max(moved, Mathf.Abs(Mathf.DeltaAngle(start[i], Steer(i))));
        }
        body.SetPositionAndRotation(position, rotation);
        late.Invoke(wheels, null);
        bool ok = moved <= MaxParkedSteer;
        Debug.Log("WHEELS parked, " + (Jitter * 1000) + " mm and " + JitterYaw + " deg jitter: front steer moved " + moved.ToString("F2") + " deg" + (ok ? "" : " FAIL"));
        return ok;
    }

    static bool Restart()
    {
        wheels.CoSim.enabled = false;
        late.Invoke(wheels, null);
        wheels.CoSim.enabled = true;
        late.Invoke(wheels, null);
        float steer = Mathf.Max(Mathf.Abs(Steer(0)), Mathf.Abs(Steer(1)));
        bool ok = steer < 0.01f;
        Debug.Log("WHEELS co-simulation restarted: front steer " + steer.ToString("F2") + " deg" + (ok ? "" : " FAIL"));
        return ok;
    }
}
