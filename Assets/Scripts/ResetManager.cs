using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ResetManager : MonoBehaviour
{
    /*
    This script emulates a sof-reset of the simulator, by setting the states of the referenced
    entities back to their initial states.
    */

    // Reset flag
    public bool ResetFlag = false;

    // Referenced entities
    public Transform[] Vehicles; // Vehicle transforms
    public Rigidbody[] VehicleRigidBodies; // Vehicle rigid bodies
    public CoSimManager[] CoSimManagers; // `CoSimManager` references
    public WheelEncoder[] LeftWheelEncoders; // `WheelEncoder` references for left wheel
    public WheelEncoder[] RightWheelEncoders; // `WheelEncoder` references for right wheel
    public LapTimer[] LapTimers; // Lap timer references

    // Arrays to store initial states
    // Vehicles
    private Vector3[] init_VehiclePositions;
    private Quaternion[] init_VehicleRotations;
    private Vector3[] init_VehicleVelocities;
    private Vector3[] init_VehicleAngularVelocities;
    // Encoders
    private float[] init_LeftWheelRevolutions;
    private float[] init_RightWheelRevolutions;
    //Lap data
    private int[] init_LapCount;
    private float[] init_LapTime;
    private float[] init_LastLapTime;
    private float[] init_BestLapTime;
    private int[] init_CurrentCheckpoint;
    private int[] init_PreviousCheckpoint;
    private int[] init_CheckpointCount;
    private int[] init_CollisionCount;


    void Start()
    {
        // Initialize arrays with fixed-size to store and retrieve initial values
        int VehicleCount = Vehicles.Length;
        // Vehicles
        init_VehiclePositions = new Vector3[VehicleCount];
        init_VehicleRotations = new Quaternion[VehicleCount];
        // Encoders
        init_LeftWheelRevolutions = new float[VehicleCount];
        init_RightWheelRevolutions = new float[VehicleCount];
        // Lap data
        init_LapCount = new int[VehicleCount];
        init_LapTime = new float[VehicleCount];
        init_LastLapTime = new float[VehicleCount];
        init_BestLapTime = new float[VehicleCount];
        init_CurrentCheckpoint = new int[VehicleCount];
        init_PreviousCheckpoint = new int[VehicleCount];
        init_CheckpointCount = new int[VehicleCount];
        init_CollisionCount = new int[VehicleCount];

        // Store initial states
        for (int i = 0; i < VehicleCount; i++)
        {
            // Vehicles
            init_VehiclePositions[i] = Vehicles[i].position;
            init_VehicleRotations[i] = Vehicles[i].rotation;
            // Encoders
            init_LeftWheelRevolutions[i] = LeftWheelEncoders[i].TotalRevolutions;
            init_RightWheelRevolutions[i] = RightWheelEncoders[i].TotalRevolutions;
            // Lap data
            if(LapTimers.Length != 0)
            {
                init_LapCount[i] = LapTimers[i].LapCount;
                init_LapTime[i] = LapTimers[i].LapTime;
                init_LastLapTime[i] = LapTimers[i].LastLapTime;
                init_BestLapTime[i] = LapTimers[i].BestLapTime;
                init_CurrentCheckpoint[i] = LapTimers[i].CurrentCheckpoint;
                init_PreviousCheckpoint[i] = LapTimers[i].PreviousCheckpoint;
                init_CheckpointCount[i] = LapTimers[i].CheckpointCount;
                init_CollisionCount[i] = LapTimers[i].CollisionCount;
            }
        }
    }

    void Update()
    {
        if (ResetFlag)
        {
            // Disable CoSimManagers and reset vehicles to their initial states
            for (int i = 0; i < Vehicles.Length; i++)
            {
                // Reset vehicle
                CoSimManagers[i].enabled = false;
                VehicleRigidBodies[i].isKinematic = true;
                Vehicles[i].position = init_VehiclePositions[i];
                Vehicles[i].rotation = init_VehicleRotations[i];
                // Reset wheel encoders
                LeftWheelEncoders[i].TotalRevolutions = init_LeftWheelRevolutions[i];
                RightWheelEncoders[i].TotalRevolutions = init_RightWheelRevolutions[i];
                // Reset lap data
                if(LapTimers.Length != 0)
                {
                    LapTimers[i].LapCount = init_LapCount[i];
                    LapTimers[i].LapTime = init_LapTime[i];
                    LapTimers[i].LastLapTime = init_LastLapTime[i];
                    LapTimers[i].BestLapTime = init_BestLapTime[i];
                    LapTimers[i].CurrentCheckpoint = init_CurrentCheckpoint[i];
                    LapTimers[i].PreviousCheckpoint = init_PreviousCheckpoint[i];
                    LapTimers[i].CheckpointCount = init_CheckpointCount[i];
                    LapTimers[i].CollisionCount = init_CollisionCount[i];
                }

            }
            // Reset the flag after reset operation
            ResetFlag = false;
        }
        else
        {
            // Re-enable vehicle dynamics
            for (int i = 0; i < Vehicles.Length; i++)
            {
                VehicleRigidBodies[i].isKinematic = false;
            }
        }
    }
}
