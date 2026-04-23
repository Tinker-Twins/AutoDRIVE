using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LapTimer : MonoBehaviour
{
    public Text txtLapTime;
    public Text txtLastLap;
    public Text txtBestLap;
    public Text txtLapCount;
    public Text txtCollisionCount;

    public string RacetrackName; // Exact name of the racetrack gameobject
    public Transform[] Checkpoints; // Array of transforms of all checkpoints
    public int CurrentCheckpoint = 0;
    public int PreviousCheckpoint = 0;
    public int CollisionCount = 0; // Collision count
    public int CheckpointCount = 0; // Checkpoint count

    public int LapCount = 0; // Measure lap count
    public float LapTime = 0; // Measure lap time
    public float LastLapTime = Mathf.Infinity; // Holds last lap time
    public float BestLapTime = Mathf.Infinity; // Holds best lap time

    private Rigidbody VehicleRigidbody; // Vehicle rigid body component
    private Transform SavedCheckpoint; // Transform of latest saved checkpoint
    private bool FinishLineFlag = false; // Finish line flag
    private bool CheckpointFlag = false; // Checkpoint flag

    void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.name == RacetrackName) Respawn(); // Collision detected with racetrack
    }

    // Reset lap time and update lap count when crossing start line
    private void OnTriggerEnter(Collider collider)
    {   
        // Count a completed lap when the vehicle crosses either finish line after
        // traversing the full checkpoint sequence assigned to this LapTimer.
        if (IsFinishLine(collider) && !FinishLineFlag && (CheckpointCount >= (Checkpoints.Length-1)))
        {
            // Update only on positive edge of trigger
            LapCount += 1;
            LastLapTime = LapTime;
            if (LapTime < BestLapTime) BestLapTime = LapTime;
            LapTime = 0;
            FinishLineFlag = true;
            CheckpointCount = 0;
        }
        else if (collider.tag == "Checkpoint" && !CheckpointFlag)
        {
            CurrentCheckpoint = GetCheckpointIndex(collider.transform);
            if (CurrentCheckpoint < 0) return;
            CheckpointFlag = true;
            if (CurrentCheckpoint == PreviousCheckpoint+1) CheckpointCount = CurrentCheckpoint;
        }
    }

    private void OnTriggerExit(Collider collider)
    {
        FinishLineFlag = false;
        CheckpointFlag = false;
        PreviousCheckpoint = CurrentCheckpoint;
    }

    void Respawn()
    {
        // Reset momentum
        VehicleRigidbody.velocity = Vector3.zero;
        VehicleRigidbody.angularVelocity = Vector3.zero;

        // Get latest passed checkpoint
        SavedCheckpoint = Checkpoints[CheckpointCount%Checkpoints.Length];

        // Reset pose
        gameObject.transform.position = SavedCheckpoint.position;
        gameObject.transform.rotation = SavedCheckpoint.rotation;

        // Update clooision flag and count
        CollisionCount = CollisionCount + 1; // Update collision count
    }

    public void Start()
    {
        VehicleRigidbody = gameObject.GetComponent<Rigidbody>();
    }

    private int GetCheckpointIndex(Transform checkpoint)
    {
        for (int i = 0; i < Checkpoints.Length; i++)
        {
            if (Checkpoints[i] == checkpoint) return i;
        }

        return -1;
    }

    private bool IsFinishLine(Collider collider)
    {
        return collider.tag == "Finish Line A" || collider.tag == "Finish Line B";
    }

    private void Update()
    {
        // Update current lap time on GUI
        if (LapTime < 10) txtLapTime.text = "0" + LapTime.ToString("f1");
        else txtLapTime.text = LapTime.ToString("f1");
        // Update lap count on GUI
        if (LapCount < 10) txtLapCount.text = "0" + LapCount.ToString();
        else txtLapCount.text = LapCount.ToString();
        // Update last lap time on GUI
        if (LastLapTime == Mathf.Infinity) txtLastLap.text = "--";
        else if (LastLapTime < 10) txtLastLap.text = "0" + LastLapTime.ToString("f1");
        else txtLastLap.text = LastLapTime.ToString("f1");
        // Update best lap time on GUI
        if (BestLapTime == Mathf.Infinity) txtBestLap.text = "--";
        else if (BestLapTime < 10) txtBestLap.text = "0" + BestLapTime.ToString("f1");
        else txtBestLap.text = BestLapTime.ToString("f1");
        // Update collision count on GUI
        txtCollisionCount.text = CollisionCount.ToString();
    }

    public void FixedUpdate()
    {
        LapTime += Time.fixedDeltaTime; // Update lap time
    }
}
