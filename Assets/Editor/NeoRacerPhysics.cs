using System.IO;
using UnityEditor;
using UnityEngine;

// Gives the NeoRacer prefab the physics of neoracer-mujoco's NeoRacer, which is computed from
// one description of the car (its spec, neoracer_spec). Tools/neoracer_spec_export.py writes
// what a Unity car needs from it to Assets/Prefabs/NeoRacer/neoracer_spec.json: the settled
// car's mass, center of mass and inertia, the wheels, the springs, the motor, the steering
// servo, the firmware's speed loop and the IMU's place. This puts them on the prefab:
// - its VehicleController becomes a NeoRacerController (the same component, so every scene's
//   references to it hold), which runs the speed loop, the motor and the servo;
// - the rigid body is the whole car: its mass, no drag (the model has none), the center of
//   mass over the middle of the wheelbase at its height above the axle line, the inertia;
// - the wheels' places are the CAD's (NeoRacerModelSwap puts the wheel colliders there): the
//   axle line runs through them, and the steering's wheelbase and track are theirs;
// - each wheel collider: the tire's radius and mass, the drivetrain's losses, the springs'
//   stiffness and damping, their travel (droop and jounce) about the wheel's place in the
//   model, which is where it rests;
// - the IMU where the spec has it, over the axle line as the center of mass is;
// - the tires' grip: the model's friction is a coefficient and no curve, so the curves peak at
//   it and hold it, rising over the slips set here (FORWARD_SLIP, SIDEWAYS_SLIP), chosen so the
//   car's turns match the model's (see the commit that set them).
// Menu, or batchmode:
//   -executeMethod NeoRacerPhysics.Apply
public static class NeoRacerPhysics
{
    const string Prefab = "Assets/Prefabs/NeoRacer/NeoRacer.prefab";
    const string SpecFile = "Assets/Prefabs/NeoRacer/neoracer_spec.json";
    const string Controller = "Assets/Scripts/NeoRacerController.cs";
    // Slip where the grip peaks, and where it has reached its plateau (Unity's slip measures)
    const float FORWARD_SLIP = 0.1f, FORWARD_PLATEAU = 0.4f;
    const float SIDEWAYS_SLIP = 0.1f, SIDEWAYS_PLATEAU = 0.4f;
    const int TIRE_SUBSTEPS = 10; // NeoRacerController.TireSubsteps

    [MenuItem("AutoDRIVE/NeoRacer: Physics from the Spec")]
    static void Menu()
    {
        Debug.Log(Make() ? "PHYSICS applied: " + Prefab : "PHYSICS FAILED: see the errors above");
    }

    public static void Apply()
    {
        EditorApplication.Exit(Make() ? 0 : 1);
    }

    static float F(JSONObject o, string key) => o.GetField(key).f;

    static bool Make()
    {
        var spec = new JSONObject(File.ReadAllText(SpecFile));
        JSONObject body = spec.GetField("body"), wheels = spec.GetField("wheels"), susp = spec.GetField("suspension");
        JSONObject motor = spec.GetField("motor"), servo = spec.GetField("servo"), loop = spec.GetField("speed_loop");

        GameObject root = PrefabUtility.LoadPrefabContents(Prefab);
        VehicleController vehicle = root.GetComponentInChildren<VehicleController>(true);
        if (vehicle == null)
        {
            Debug.LogError("NeoRacerPhysics: the NeoRacer has no VehicleController");
            PrefabUtility.UnloadPrefabContents(root);
            return false;
        }
        if (!(vehicle is NeoRacerController))
        {
            var so = new SerializedObject(vehicle);
            so.FindProperty("m_Script").objectReferenceValue = AssetDatabase.LoadAssetAtPath<MonoScript>(Controller);
            so.ApplyModifiedPropertiesWithoutUndo();
            vehicle = root.GetComponentInChildren<VehicleController>(true);
        }
        var car = (NeoRacerController)vehicle;
        Rigidbody rb = car.VehicleRigidBody;
        WheelCollider[] all = { car.FrontLeftWheelCollider, car.FrontRightWheelCollider,
                                car.RearLeftWheelCollider, car.RearRightWheelCollider };

        // The axle line, in the rigid body's frame: the wheels' places, where they rest. The car
        // stands on it: its up is square to it (the CAD's front axle sits lower than the rear)
        Transform t = rb.transform;
        Vector3[] at = { t.InverseTransformPoint(all[0].transform.position), t.InverseTransformPoint(all[1].transform.position),
                         t.InverseTransformPoint(all[2].transform.position), t.InverseTransformPoint(all[3].transform.position) };
        Vector3 front = (at[0] + at[1]) / 2, rear = (at[2] + at[3]) / 2;
        Vector3 forward = (front - rear).normalized;
        Vector3 up = Vector3.ProjectOnPlane(Vector3.up, forward).normalized;
        var com = body.GetField("center_of_mass").list;
        car.COM = (front + rear) / 2 + forward * com[0].f + up * com[2].f; // the model's +Y left is 0
        var imuAt = spec.GetField("imu").GetField("position").list;
        IMU imu = root.GetComponentInChildren<IMU>(true);
        imu.transform.position = t.TransformPoint((front + rear) / 2 + forward * imuAt[0].f + up * imuAt[2].f);
        var inertia = body.GetField("inertia").list;
        // The model's axes (x forward, y left, z up) are Unity's z, -x and y
        car.Inertia = new Vector3(inertia[1].list[1].f, inertia[2].list[2].f, inertia[0].list[0].f);
        rb.mass = F(body, "mass");
        rb.drag = 0f;
        rb.angularDrag = 0f;

        float droop = F(susp, "droop"), jounce = F(susp, "jounce");
        float mu = F(wheels, "friction");
        foreach (WheelCollider w in all)
        {
            w.radius = F(wheels, "radius");
            w.mass = F(wheels, "mass");
            w.wheelDampingRate = F(wheels, "drive_damping");
            w.suspensionDistance = droop + jounce;
            // The spring rests at its place: jounce under the top of its travel. Unity measures
            // targetPosition from fully extended (measured: at jounce / travel it rested droop under the top)
            w.center = new Vector3(0f, jounce, 0f);
            w.suspensionSpring = new JointSpring
            {
                spring = F(susp, "stiffness"),
                damper = F(susp, "damping"),
                targetPosition = droop / (droop + jounce),
            };
            w.forwardFriction = new WheelFrictionCurve
            {
                extremumSlip = FORWARD_SLIP, extremumValue = mu,
                asymptoteSlip = FORWARD_PLATEAU, asymptoteValue = mu, stiffness = 1f,
            };
            w.sidewaysFriction = new WheelFrictionCurve
            {
                extremumSlip = SIDEWAYS_SLIP, extremumValue = mu,
                asymptoteSlip = SIDEWAYS_PLATEAU, asymptoteValue = mu, stiffness = 1f,
            };
        }

        // LapTimer respawns the car at its last checkpoint, which a track puts just over its floor
        // (AuE-X930: 10 mm). The car's root there put its lowest wheel 35 mm into the floor, past the
        // springs' travel, and threw the car into the air: it respawns standing on that wheel, 1 mm up
        float lift = 0f;
        foreach (WheelCollider w in all)
            lift = Mathf.Max(lift, w.radius - t.InverseTransformPoint(w.transform.position).y + 0.001f);
        rb.GetComponent<LapTimer>().RespawnLift = lift;
        Debug.Log("PHYSICS respawn " + (1000f * lift).ToString("F1") + " mm over a checkpoint");

        // The tires' solid colliders, upstream's cylinders named Collider: upstream hung them on
        // its drawn wheels, so they turned, steered and rode the springs with them. Since the
        // model swap (NeoRacerModelSwap) the drawn wheels are the new model's, and the cylinders
        // stayed on the old wheels' empty places: fixed to the chassis, 2.5 mm over the floor,
        // they rubbed it whenever the body dipped. Each goes back on the drawn wheel nearest it,
        // and does not touch the ground the wheel colliders stand on (NeoRacerController.TireColliders)
        var tires = new System.Collections.Generic.List<Collider>();
        Transform[] drawnWheels = { car.FrontLeftWheelTransform, car.FrontRightWheelTransform,
                                    car.RearLeftWheelTransform, car.RearRightWheelTransform };
        foreach (MeshCollider c in root.GetComponentsInChildren<MeshCollider>(true))
        {
            if (c.name != "Collider") continue;
            Transform nearest = drawnWheels[0];
            foreach (Transform drawn in drawnWheels)
                if ((drawn.position - c.transform.position).sqrMagnitude < (nearest.position - c.transform.position).sqrMagnitude)
                    nearest = drawn;
            if (c.transform.parent != nearest) c.transform.SetParent(nearest, true);
            tires.Add(c);
        }
        if (tires.Count != 4)
        {
            Debug.LogError("NeoRacerPhysics: expected 4 tire colliders named Collider, found " + tires.Count);
            PrefabUtility.UnloadPrefabContents(root);
            return false;
        }
        car.TireColliders = tires.ToArray();
        car.TireSubsteps = TIRE_SUBSTEPS;

        // The rest of the body's colliders must clear the ground with the car at rest on its
        // wheels (the wheels' centers at their places, a radius above the ground, which lies
        // along the axle line)
        float radius = F(wheels, "radius");
        bool clear = true;
        foreach (Collider c in root.GetComponentsInChildren<Collider>(true))
        {
            if (c is WheelCollider || tires.Contains(c) || !c.enabled || c.isTrigger) continue;
            float lowest = float.MaxValue;
            Bounds b = c is MeshCollider m && m.sharedMesh != null ? m.sharedMesh.bounds : new Bounds(Vector3.zero, Vector3.zero);
            if (c is BoxCollider box) b = new Bounds(box.center, box.size);
            else if (c is SphereCollider sphere) b = new Bounds(sphere.center, Vector3.one * 2f * sphere.radius);
            else if (c is CapsuleCollider capsule) b = new Bounds(capsule.center, Vector3.one * 2f * Mathf.Max(capsule.radius, capsule.height / 2f));
            if (c is MeshCollider hull && hull.convex)
                foreach (Vector3 v in hull.sharedMesh.vertices)
                    lowest = Mathf.Min(lowest, Vector3.Dot(t.InverseTransformPoint(c.transform.TransformPoint(v)) - rear, up) + radius);
            else
                for (int i = 0; i < 8; i++)
                {
                    Vector3 corner = b.center + Vector3.Scale(b.extents, new Vector3(i & 1, (i >> 1) & 1, (i >> 2) & 1) * 2f - Vector3.one);
                    lowest = Mathf.Min(lowest, Vector3.Dot(t.InverseTransformPoint(c.transform.TransformPoint(corner)) - rear, up) + radius);
                }
            Debug.Log("PHYSICS collider " + c.GetType().Name + " on " + c.name + ": lowest " + (lowest * 1000f).ToString("F1") + " mm over the ground");
            if (lowest <= 0f) clear = false;
        }
        if (!clear)
        {
            Debug.LogError("NeoRacerPhysics: a collider of the body reaches the ground with the car at rest on its wheels");
            PrefabUtility.UnloadPrefabContents(root);
            return false;
        }

        car.Wheelbase = 1000f * (front - rear).magnitude; // mm, as VehicleController's Ackermann takes them
        car.TrackWidth = 1000f * (at[1] - at[0]).magnitude; // the steered axle's
        car.WheelRadius = radius;
        car.driveType = VehicleController.DriveType.CAWD; // one motor, open differentials: a quarter each
        car.MotorTorque = 1f; // the telemetry's throttle is then the ESC's share of the battery
        car.SteeringLimit = F(servo, "lock") * Mathf.Rad2Deg;

        car.MaxSpeed = F(loop, "max_speed");
        car.LoopPeriod = F(loop, "period");
        car.SpeedFilter = F(loop, "filter");
        car.LoopKp = F(loop, "kp");
        car.LoopKi = F(loop, "ki");
        car.LoopKd = F(loop, "kd");
        car.IntegralLimit = F(loop, "integral_limit");
        car.ErrorDeadband = F(loop, "error_deadband");
        car.StopSpeed = F(loop, "stop_speed");
        car.FeedForward = F(loop, "feed_forward");
        car.FeedForwardDeadband = F(loop, "deadband");
        car.FeedForwardLimit = F(loop, "feed_forward_limit");
        car.PulseLimit = F(loop, "pulse_limit");
        car.FullThrottlePulse = F(loop, "full_throttle_pulse");

        car.BatteryVoltage = F(motor, "battery_voltage");
        car.MotorRatedVoltage = F(motor, "rated_voltage");
        car.MotorStallTorque = F(motor, "stall_torque");
        car.MotorFreeSpeed = F(motor, "free_speed");
        car.GearRatio = F(motor, "gear");
        car.DriveDamping = F(wheels, "drive_damping");

        car.ServoVoltage = F(servo, "voltage_limit");
        car.ServoStallTorque = F(servo, "stall_torque");
        car.ServoFreeSpeed = F(servo, "free_speed");
        car.ServoKp = F(servo, "kp");
        car.ServoKd = F(servo, "kd");
        car.ServoLatency = F(servo, "latency");
        car.SteerLock = F(servo, "lock");
        car.SteerInertia = F(servo, "inertia");
        car.SteerDamping = F(servo, "damping");
        car.SteerFriction = F(servo, "frictionloss");

        string report = "PHYSICS mass " + rb.mass + " kg, COM " + car.COM.ToString("F4") + ", inertia " + car.Inertia.ToString("F5") +
                  "; IMU at " + t.InverseTransformPoint(imu.transform.position).ToString("F4") +
                  "; wheelbase " + car.Wheelbase.ToString("F1") + " mm (spec " + (2000f * F(wheels, "half_wheelbase")).ToString("F1") +
                  "), front track " + car.TrackWidth.ToString("F1") + " mm (spec " + (2000f * F(wheels, "half_track")).ToString("F1") +
                  "), on level ground the body rests " + (Mathf.Asin(-forward.y) * Mathf.Rad2Deg).ToString("F2") + " degrees nose up" +
                  "; wheels r " + F(wheels, "radius") + " m, springs " + F(susp, "stiffness") + " N/m " + F(susp, "damping") +
                  " N-s/m over " + (droop + jounce) + " m; grip " + mu + "; " + tires.Count + " tire colliders; servo lock " + F(servo, "lock") + " rad, latency " +
                  F(servo, "latency") + " s; motor stall " + F(motor, "stall_torque") + " N-m x " + F(motor, "gear") +
                  " (spec from " + spec.GetField("source").str + ")";
        PrefabUtility.SaveAsPrefabAsset(root, Prefab);
        PrefabUtility.UnloadPrefabContents(root);
        Debug.Log(report);
        return true;
    }
}
