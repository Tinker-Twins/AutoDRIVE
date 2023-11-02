using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class F1TenthRacing : Agent
{

    [Header("HUD")]
    public Text txtLapTime;
    public Text txtLastLap;
    public Text txtBestLap;
    public Text txtLapCount;

    [Header("Agent")]
    public int AgentID = 1;
    public KeyCode ThrottleKey = KeyCode.W;
    public KeyCode LeftSteerKey = KeyCode.A;
    public KeyCode RightSteerKey = KeyCode.D;
    public VehicleController EV_ActuatorController;
    public Transform EV_SpawnLocation;
    private float EV_Speed = 0;

    // [Header("Opponent")]
    // public GameObject OpponentVehicle;
    // public Transform OV_SpawnLocation;

    private Rigidbody EV_Rigidbody;
    private Rigidbody OV_Rigidbody;

    private int LapCount = 0; // Measure lap count
    private float LapTime = 0; // Measure lap time
    private float BestLapTime = 1e+6f; // Holds best lap time
    private bool FinishLineFlag = false; // Finish line flag
    private bool CheckpointFlag = false; // Checkpoint flag
    private bool CollisionFlag = false; // Collision flag
    private bool LapCompletionFlag = false; // Lap completion flag
    private bool CheckpointPassingFlag = false; // Checkpoint passing flag
    private bool LapTimeReducedFlag = false; // Best lap time flag

    void OnCollisionEnter(Collision collision)
    {
        CollisionFlag = true; // Collision detected
    }

    // Reset lap time and update lap count when crossing start line
    private void OnTriggerEnter(Collider collider)
    {
        if (((AgentID == 1 && collider.tag == "Finish Line A") || (AgentID == 2 && collider.tag == "Finish Line B")) && !FinishLineFlag)
        {
            // Update only on positive edge of trigger
            LapCompletionFlag = true;
            LapCount += 1;
            if (LapCount < 10) txtLapCount.text = "0" + LapCount.ToString();
            else txtLapCount.text = LapCount.ToString();
            if (LapTime < 10) txtLastLap.text = "0" + LapTime.ToString("f1");
            else txtLastLap.text = LapTime.ToString("f1");
            if (LapTime < BestLapTime)
            {
                LapTimeReducedFlag = true;
                BestLapTime = LapTime;
                if (BestLapTime < 10) txtBestLap.text = "0" + BestLapTime.ToString("f1");
                else txtBestLap.text = BestLapTime.ToString("f1");
            }
            LapTime = 0;
            FinishLineFlag = true;
        }
        else if (collider.tag == "Checkpoint" && !CheckpointFlag)
        {
            CheckpointFlag = true;
            CheckpointPassingFlag = true;
        }
    }

    private void OnTriggerExit(Collider collider)
    {
        FinishLineFlag = false;
        CheckpointFlag = false;
    }

    public override void Initialize()
    {
        EV_Rigidbody = gameObject.GetComponent<Rigidbody>();
        // OV_Rigidbody = OpponentVehicle.GetComponent<Rigidbody>();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        EV_Speed = (float)System.Math.Abs(System.Math.Round(EV_ActuatorController.Vehicle.transform.InverseTransformDirection(EV_ActuatorController.Vehicle.GetComponent<Rigidbody>().velocity).z,2));
        sensor.AddObservation((float)System.Math.Round(EV_Speed, 2)); // Speed of ego-vehicle (m/s)
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // DISCRETE ACTION SPACE
        int DriveAction = Mathf.FloorToInt(actions.DiscreteActions[0]);
        int SteerAction = Mathf.FloorToInt(actions.DiscreteActions[1]);

        switch (DriveAction)
        {
            case 0:
                EV_ActuatorController.CurrentThrottle = 0.1f; // Default to zero while demo recording
                break;
            case 1:
                EV_ActuatorController.CurrentThrottle = 0.5f; // Not used while demo recording (discrete input 0/1)
                break;
            case 2:
                EV_ActuatorController.CurrentThrottle = 1.0f; // Mostly used while demo recording
                break;
        }

        switch (SteerAction)
        {
            case 0:
                EV_ActuatorController.CurrentSteeringAngle = 1f; // Left turn
                break;
            case 1:
                EV_ActuatorController.CurrentSteeringAngle = 0f; // Straight
                break;
            case 2:
                EV_ActuatorController.CurrentSteeringAngle = -1f; // Right turn
                break;
        }

        // CONTINUOUS ACTION SPACE
        //EV_ActuatorController.CurrentThrottle = Mathf.Clamp(actions.ContinuousActions[0], 0f, 1f); // Drive
        //EV_ActuatorController.CurrentSteeringAngle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f); // Steer

        // REWARD FUNCTION
        if (CollisionFlag)
        {
            Debug.Log("Agent Collided!");
            SetReward(-1f);
            EndEpisode();
        }
        if (CheckpointPassingFlag)
        {
            Debug.Log("Agent Passed Checkpoint!");
            SetReward(+0.01f);
            CheckpointPassingFlag = false;
        }
        if (LapCompletionFlag)
        {
            Debug.Log("Agent Completed Lap!");
            SetReward(+0.1f);
            if (LapTimeReducedFlag)
            {
                Debug.Log("Agent's Lap Time Reduced!");
                SetReward(+0.7f);
            }
            LapTimeReducedFlag = false;
            LapCompletionFlag = false;
            EndEpisode();
        }
        else
        {
            SetReward(EV_Speed*0.01f);
        }
    }

    public override void OnEpisodeBegin()
    {
        // RESET EGO VEHICLE
        // Reset momentum
        EV_Rigidbody.velocity = Vector3.zero;
        EV_Rigidbody.angularVelocity = Vector3.zero;
        // Reset pose
        gameObject.transform.position = EV_SpawnLocation.position;
        gameObject.transform.rotation = EV_SpawnLocation.rotation;

        // // RESET OPPONENT VEHICLE
        // // Reset momentum
        // OV_Rigidbody.velocity = Vector3.zero;
        // OV_Rigidbody.angularVelocity = Vector3.zero;
        // // Reset pose
        // OpponentVehicle.transform.position = OV_SpawnLocation.position;
        // OpponentVehicle.transform.rotation = OV_SpawnLocation.rotation;

        // RESET FLAGS
        CollisionFlag = false; // Reset collision flag
        LapTime = 0; // Reset lap time
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // DISCRETE ACTION SPACE
        var discreteActionsOut = actionsOut.DiscreteActions;
        // Drive
        if (Input.GetKey(ThrottleKey)) discreteActionsOut[0] = 2;
        else discreteActionsOut[0] = 0;
        // Steer
        if (Input.GetKey(LeftSteerKey)) discreteActionsOut[1] = 0;
        else if (Input.GetKey(RightSteerKey)) discreteActionsOut[1] = 2;
        else discreteActionsOut[1] = 1;


        // CONTINUOUS ACTION SPACE
        // var continuousActionsOut = actionsOut.ContinuousActions;
        //actionsOut.ContinuousActions[0] = Input.GetAxis("Vertical"); // Drive
        //actionsOut.ContinuousActions[1] = Input.GetAxis("Horizontal"); // Steer
    }

    private void Update()
    {
        // Update lap time on GUI
        if (LapTime < 10) txtLapTime.text = "0" + LapTime.ToString("f1");
        else txtLapTime.text = LapTime.ToString("f1");
    }

    public void FixedUpdate()
    {
        LapTime += Time.fixedDeltaTime; // Update lap time
    }
}