using UnityEngine;

public class CoSimWheels : MonoBehaviour
{
    /*
    This script turns and steers the wheels of a vehicle posed by co-simulation.
    A posed vehicle's rigid body is kinematic, so its wheel colliders roll nothing:
    here each wheel rolls the distance its centre moves along its heading, and the
    front wheels steer to the turn the vehicle makes (Ackermann about the rear axle),
    no further than the vehicle's steering limit.
    The turn is measured over every TurnWindow m the rear axle moves, not frame by
    frame: a localized pose jitters by millimetres, and that jitter over a frame's
    fraction of a millimetre of travel reads as a sharp turn.
    */

    public const float TurnWindow = 0.02f; // m of rear axle travel a turn is measured over

    public CoSimManager CoSim;
    public VehicleController Vehicle;

    private Transform body;
    private Transform[] wheels; // FL, FR, RL, RR
    private Vector3[] rest; // Wheel centres in the vehicle's frame, as built
    private Quaternion[] restRotations;
    private float[] radii; // Drawn tyre radii
    private float[] spins; // Degrees
    private Vector3[] lastCentres;
    private Vector3 axle; // Rear axle centre in the vehicle's frame
    private Vector3 windowAxle; // Rear axle centre where the turn's measurement began
    private float windowYaw;
    private float curvature; // Of the rear axle centre's path, 1/m, right turns positive
    private bool posed;

    void Start()
    {
        body = Vehicle.VehicleRigidBody.transform;
        wheels = new Transform[] { Vehicle.FrontLeftWheelTransform, Vehicle.FrontRightWheelTransform,
                                   Vehicle.RearLeftWheelTransform, Vehicle.RearRightWheelTransform };
        rest = new Vector3[4];
        restRotations = new Quaternion[4];
        radii = new float[4];
        spins = new float[4];
        lastCentres = new Vector3[4];
        for (int i = 0; i < 4; i++)
        {
            rest[i] = body.InverseTransformPoint(wheels[i].position);
            restRotations[i] = Quaternion.Inverse(body.rotation) * wheels[i].rotation;
            radii[i] = wheels[i].GetComponent<MeshFilter>().sharedMesh.bounds.extents.y * wheels[i].lossyScale.y;
        }
        axle = (rest[2] + rest[3]) / 2;
    }

    void LateUpdate()
    {
        if (!CoSim.enabled)
        {
            posed = false;
            curvature = 0f;
            return;
        }
        Vector3 axleNow = body.TransformPoint(axle);
        float yaw = body.eulerAngles.y;
        if (!posed)
        {
            windowAxle = axleNow;
            windowYaw = yaw;
        }
        else
        {
            float ds = Vector3.Dot(axleNow - windowAxle, body.forward);
            // Standing still or crawling, the wheels keep their steer until the car has moved a window
            if (Mathf.Abs(ds) >= TurnWindow)
            {
                curvature = Mathf.DeltaAngle(windowYaw, yaw) * Mathf.Deg2Rad / ds;
                windowAxle = axleNow;
                windowYaw = yaw;
            }
        }
        for (int i = 0; i < 4; i++)
        {
            Vector3 offset = rest[i] - axle;
            float steer = i < 2 ? Mathf.Clamp(Mathf.Rad2Deg * Mathf.Atan2(offset.z * curvature, 1 - offset.x * curvature),
                                              -Vehicle.SteeringLimit, Vehicle.SteeringLimit) : 0f;
            Quaternion heading = body.rotation * Quaternion.Euler(0, steer, 0);
            Vector3 centre = body.TransformPoint(rest[i]);
            if (posed)
            {
                float roll = Vector3.Dot(centre - lastCentres[i], heading * Vector3.forward) / radii[i];
                spins[i] = Mathf.Repeat(spins[i] + roll * Mathf.Rad2Deg, 360f);
            }
            wheels[i].SetPositionAndRotation(centre, heading * Quaternion.Euler(spins[i], 0, 0) * restRotations[i]);
            lastCentres[i] = centre;
        }
        posed = true;
    }
}
