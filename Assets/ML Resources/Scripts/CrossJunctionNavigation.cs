using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class CrossJunctionNavigation : Agent
{

    [Header("Ego Vehicle")]
    // SENSOR DATA
    public IPS V0_IPS;
    public IMU V0_IMU;
    public WheelEncoder V0_LeftEncoder;
    public WheelEncoder V0_RightEncoder;
    // ACTUATOR CONTROLLER
    public VehicleController V0_ActuatorController;
    // SPAWN AND GOAL LOCATIONS
    public Transform V0_SpawnLocation;
    public Transform V0_Left_GoalLocation;
    public Transform V0_Right_GoalLocation;
    public float V0_GoalTolerance = 0f; // Tolerance to reach goal (m)

    [Header("Peer Vehicle 1")]
    public GameObject PeerVehicle1;
    // SENSOR DATA
    public IPS V1_IPS;
    public IMU V1_IMU;
    public WheelEncoder V1_LeftEncoder;
    public WheelEncoder V1_RightEncoder;
    // ACTUATOR CONTROLLER
    public VehicleController V1_ActuatorController;
    // SPAWN AND GOAL LOCATIONS
    public Transform V1_SpawnLocation;
    public Transform V1_GoalLocation;
    public float V1_GoalTolerance = 0f; // Tolerance to reach goal (m)

    [Header("Peer Vehicle 2")]
    public GameObject PeerVehicle2;
    // SENSOR DATA
    public IPS V2_IPS;
    public IMU V2_IMU;
    public WheelEncoder V2_LeftEncoder;
    public WheelEncoder V2_RightEncoder;
    // ACTUATOR CONTROLLER
    public VehicleController V2_ActuatorController;
    // SPAWN AND GOAL LOCATIONS
    public Transform V2_SpawnLocation;
    public Transform V2_GoalLocation;
    public float V2_GoalTolerance = 0f; // Tolerance to reach goal (m)


    [Header("Peer Vehicle 3")]
    public GameObject PeerVehicle3;
    // SENSOR DATA
    public IPS V3_IPS;
    public IMU V3_IMU;
    public WheelEncoder V3_LeftEncoder;
    public WheelEncoder V3_RightEncoder;
    // ACTUATOR CONTROLLER
    public VehicleController V3_ActuatorController;
    // SPAWN AND GOAL LOCATIONS
    public Transform V3_SpawnLocation;
    public Transform V3_GoalLocation;
    public float V3_GoalTolerance = 0f; // Tolerance to reach goal (m)


    private Rigidbody V0_Rigidbody;
    private Rigidbody V1_Rigidbody;
    private Rigidbody V2_Rigidbody;
    private Rigidbody V3_Rigidbody;

    private bool CollisionFlag = false;

    public override void Initialize()
    {
        V0_Rigidbody = gameObject.GetComponent<Rigidbody>();
        V1_Rigidbody = PeerVehicle1.GetComponent<Rigidbody>();
        V2_Rigidbody = PeerVehicle2.GetComponent<Rigidbody>();
        V3_Rigidbody = PeerVehicle3.GetComponent<Rigidbody>();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // EGO VEHICLE STATES
        /*
        sensor.AddObservation((float)System.Math.Round(V0_IPS.CurrentPosition[0], 3)); // Position x-coordinate
        sensor.AddObservation((float)System.Math.Round(V0_IPS.CurrentPosition[1], 3)); // Position y-coordinate
        sensor.AddObservation((float)System.Math.Round(V0_IMU.CurrentOrientationEulerAngles[2], 3)); // Yaw
        sensor.AddObservation((float)System.Math.Round((V0_LeftEncoder.SpeedFromTicks+V0_RightEncoder.SpeedFromTicks)/2f, 3)); // Velocity estimate
        */

        // RELATIVE GOAL LOCATION
        sensor.AddObservation((float)System.Math.Round((V0_Left_GoalLocation.position.z+V0_Right_GoalLocation.position.z)/2-V0_IPS.CurrentPosition[0], 3)); // Goal location x-coordinate
        sensor.AddObservation((float)System.Math.Round((-V0_Left_GoalLocation.position.x-V0_Right_GoalLocation.position.x)/2-V0_IPS.CurrentPosition[1], 3)); // Goal location y-coordinate

        // RELATIVE PEER VEHICLE STATES
        // Peer Vehicle 1
        sensor.AddObservation((float)System.Math.Round(V1_IPS.CurrentPosition[0]-V0_IPS.CurrentPosition[0], 3)); // Position x-coordinate
        sensor.AddObservation((float)System.Math.Round(V1_IPS.CurrentPosition[1]-V0_IPS.CurrentPosition[1], 3)); // Position y-coordinate
        sensor.AddObservation((float)System.Math.Round(V1_IMU.CurrentOrientationEulerAngles[2]-V0_IMU.CurrentOrientationEulerAngles[2], 3)); // Yaw
        sensor.AddObservation((float)System.Math.Round((V1_LeftEncoder.SpeedFromTicks+V1_RightEncoder.SpeedFromTicks)/2f, 3)); // Velocity estimate
        // Peer Vehicle 2
        sensor.AddObservation((float)System.Math.Round(V2_IPS.CurrentPosition[0]-V0_IPS.CurrentPosition[0], 3)); // Position x-coordinate
        sensor.AddObservation((float)System.Math.Round(V2_IPS.CurrentPosition[1]-V0_IPS.CurrentPosition[1], 3)); // Position y-coordinate
        sensor.AddObservation((float)System.Math.Round(V2_IMU.CurrentOrientationEulerAngles[2]-V0_IMU.CurrentOrientationEulerAngles[2], 3)); // Yaw
        sensor.AddObservation((float)System.Math.Round((V2_LeftEncoder.SpeedFromTicks+V2_RightEncoder.SpeedFromTicks)/2f, 3)); // Velocity estimate
        // Peer Vehicle 3
        sensor.AddObservation((float)System.Math.Round(V3_IPS.CurrentPosition[0]-V0_IPS.CurrentPosition[0], 3)); // Position x-coordinate
        sensor.AddObservation((float)System.Math.Round(V3_IPS.CurrentPosition[1]-V0_IPS.CurrentPosition[1], 3)); // Position y-coordinate
        sensor.AddObservation((float)System.Math.Round(V3_IMU.CurrentOrientationEulerAngles[2]-V0_IMU.CurrentOrientationEulerAngles[2], 3)); // Yaw
        sensor.AddObservation((float)System.Math.Round((V3_LeftEncoder.SpeedFromTicks+V3_RightEncoder.SpeedFromTicks)/2f, 3)); // Velocity estimate
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // DISCRETE ACTION SPACE
        //int DriveAction = Mathf.FloorToInt(actions.DiscreteActions[0]);
        int SteerAction = Mathf.FloorToInt(actions.DiscreteActions[0]);
        /*
        switch (DriveAction)
        {
            case 0:
                V0_ActuatorController.CurrentThrottle = 0.5f;
                break;
            case 1:
                V0_ActuatorController.CurrentThrottle = 0.75f;
                break;
            case 2:
                V0_ActuatorController.CurrentThrottle = 1.0f;
                break;
        }
        */
        switch (SteerAction)
        {
            case 0:
                V0_ActuatorController.CurrentSteeringAngle = -1f;
                break;
            case 1:
                V0_ActuatorController.CurrentSteeringAngle = 0f;
                break;
            case 2:
                V0_ActuatorController.CurrentSteeringAngle = 1f;
                break;
        }
        V0_ActuatorController.CurrentThrottle = 1.0f;

        // CONTINUOUS ACTION SPACE
        //V0_ActuatorController.CurrentThrottle = Mathf.Clamp(actions.ContinuousActions[0], 0.4f, 1f); // Drive
        //V0_ActuatorController.CurrentSteeringAngle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f); // Steer

        // REWARD FUNCTION
        if (CollisionFlag)
        {
            Debug.Log("Ego Vehicle Collided!");
            SetReward((-0.425f)*GetDistance(V0_IPS.CurrentPosition[0], V0_IPS.CurrentPosition[1], (V0_Left_GoalLocation.position.z+V0_Right_GoalLocation.position.z)/2, (-V0_Left_GoalLocation.position.x-V0_Right_GoalLocation.position.x)/2));
            EndEpisode();
        }
        if (GetDistance(V0_IPS.CurrentPosition[0], V0_IPS.CurrentPosition[1], V0_Left_GoalLocation.position.z, -V0_Left_GoalLocation.position.x) <= V0_GoalTolerance | GetDistance(V0_IPS.CurrentPosition[0], V0_IPS.CurrentPosition[1], V0_Right_GoalLocation.position.z, -V0_Right_GoalLocation.position.x) <= V0_GoalTolerance)
        {
            Debug.Log("Ego Vehicle Reached Goal!");
            SetReward(1f);
            EndEpisode();
        }

        // HEURISTIC CONTROL OF PEER VEHICLES
        // Peer Vehicle 1
        if (GetDistance(V1_IPS.CurrentPosition[0], V1_IPS.CurrentPosition[1], V1_GoalLocation.position.z, -V1_GoalLocation.position.x) <= V1_GoalTolerance) V1_ActuatorController.CurrentThrottle = 0f;
        else if (GetDistance(V1_IPS.CurrentPosition[0], V1_IPS.CurrentPosition[1], V1_GoalLocation.position.z, -V1_GoalLocation.position.x) <= 2*V1_GoalTolerance) V1_ActuatorController.CurrentThrottle = 0.4f;
        else V1_ActuatorController.CurrentThrottle = 1f;
        V1_ActuatorController.CurrentSteeringAngle = 0f;
        // Peer Vehicle 2
        if (GetDistance(V2_IPS.CurrentPosition[0], V2_IPS.CurrentPosition[1], V2_GoalLocation.position.z, -V2_GoalLocation.position.x) <= V2_GoalTolerance) V2_ActuatorController.CurrentThrottle = 0f;
        else if (GetDistance(V2_IPS.CurrentPosition[0], V2_IPS.CurrentPosition[1], V2_GoalLocation.position.z, -V2_GoalLocation.position.x) <= 2*V2_GoalTolerance) V2_ActuatorController.CurrentThrottle = 0.4f;
        else if (GetDistance(V1_IPS.CurrentPosition[0], V1_IPS.CurrentPosition[1], V1_GoalLocation.position.z, -V1_GoalLocation.position.x) <= V1_GoalTolerance) V2_ActuatorController.CurrentThrottle = 1f;
        else V2_ActuatorController.CurrentThrottle = 0.4f;
        V2_ActuatorController.CurrentSteeringAngle = 0f;
        // Peer Vehicle 3
        if (GetDistance(V3_IPS.CurrentPosition[0], V3_IPS.CurrentPosition[1], V3_GoalLocation.position.z, -V3_GoalLocation.position.x) <= V3_GoalTolerance) V3_ActuatorController.CurrentThrottle = 0f;
        else if (GetDistance(V3_IPS.CurrentPosition[0], V3_IPS.CurrentPosition[1], V3_GoalLocation.position.z, -V3_GoalLocation.position.x) <= 2*V3_GoalTolerance) V3_ActuatorController.CurrentThrottle = 0.4f;
        else if (GetDistance(V1_IPS.CurrentPosition[0], V1_IPS.CurrentPosition[1], V1_GoalLocation.position.z, -V1_GoalLocation.position.x) <= V1_GoalTolerance) V3_ActuatorController.CurrentThrottle = 1f;
        else V3_ActuatorController.CurrentThrottle = 0.8f;
        V3_ActuatorController.CurrentSteeringAngle = 0f;
    }

    public override void OnEpisodeBegin()
    {
        // RESET EGO VEHICLE
        // Reset momentum
        V0_Rigidbody.velocity = Vector3.zero;
        V0_Rigidbody.angularVelocity = Vector3.zero;
        // Reset pose
        gameObject.transform.position = V0_SpawnLocation.position;
        gameObject.transform.rotation = V0_SpawnLocation.rotation;

        // RESET PEER VEHICLE 1
        // Reset momentum
        V1_Rigidbody.velocity = Vector3.zero;
        V1_Rigidbody.angularVelocity = Vector3.zero;
        // Reset pose
        PeerVehicle1.transform.position = V1_SpawnLocation.position;
        PeerVehicle1.transform.rotation = V1_SpawnLocation.rotation;

        // RESET PEER VEHICLE 2
        // Reset momentum
        V2_Rigidbody.velocity = Vector3.zero;
        V2_Rigidbody.angularVelocity = Vector3.zero;
        // Reset pose
        PeerVehicle2.transform.position = V2_SpawnLocation.position;
        PeerVehicle2.transform.rotation = V2_SpawnLocation.rotation;

        // RESET PEER VEHICLE 3
        // Reset momentum
        V3_Rigidbody.velocity = Vector3.zero;
        V3_Rigidbody.angularVelocity = Vector3.zero;
        // Reset pose
        PeerVehicle3.transform.position = V3_SpawnLocation.position;
        PeerVehicle3.transform.rotation = V3_SpawnLocation.rotation;

        // RESET COLLISION FLAG
        CollisionFlag = false;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // DISCRETE ACTION SPACE
        var discreteActionsOut = actionsOut.DiscreteActions;
        // Drive
        //if (Input.GetKey(KeyCode.W)) discreteActionsOut[0] = 2;
        //else discreteActionsOut[0] = 0;
        // Steer
        if (Input.GetKey(KeyCode.A)) discreteActionsOut[0] = 0;
        else if (Input.GetKey(KeyCode.D)) discreteActionsOut[0] = 2;
        else discreteActionsOut[0] = 1;


        // CONTINUOUS ACTION SPACE
        var continuousActionsOut = actionsOut.ContinuousActions;
        //actionsOut.ContinuousActions[0] = 0.4f+0.6f*Input.GetAxis("Vertical"); // Drive
        //actionsOut.ContinuousActions[1] = Input.GetAxis("Horizontal"); // Steer
    }

    void OnCollisionEnter(Collision collision)
    {
        CollisionFlag = true;
    }

    float GetDistance(float x1, float y1, float x2, float y2)
    {
        float distance = Mathf.Sqrt(Mathf.Pow(x2-x1, 2) + Mathf.Pow(y2-y1, 2));
        return distance;
    }
}
