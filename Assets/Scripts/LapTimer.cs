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
    public int checkpointCount = 19;

    private Rigidbody VehicleRigidbody;
    private int CheckpointCount = 0; // Number of checkpoints
    private int LapCount = 0; // Measure lap count
    private float LapTime = 0; // Measure lap time
    private float BestLapTime = 1e+6f; // Holds best lap time
    private bool FinishLineFlag = false; // Finish line flag
    private bool CheckpointFlag = false; // Checkpoint flag
    private bool CollisionFlag = false; // Collision flag
    private bool LapCompletionFlag = false; // Lap completion flag
    private bool CheckpointPassingFlag = false; // Checkpoint passing flag

    void OnCollisionEnter(Collision collision)
    {
        CollisionFlag = true; // Collision detected
    }

    // Reset lap time and update lap count when crossing start line
    private void OnTriggerEnter(Collider collider)
    {   
        // Each checkpoint will be triggered approximately 4 times the vehicle passes through it
        if ((collider.tag == "Finish Line A") && !FinishLineFlag && (CheckpointCount>=checkpointCount*3.33f))
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
                BestLapTime = LapTime;
                if (BestLapTime < 10) txtBestLap.text = "0" + BestLapTime.ToString("f1");
                else txtBestLap.text = BestLapTime.ToString("f1");
            }
            LapTime = 0;
            FinishLineFlag = true;
            CheckpointCount = 0;
        }
        else if (collider.tag == "Checkpoint" && !CheckpointFlag)
        {
            CheckpointFlag = true;
            CheckpointPassingFlag = true;
            CheckpointCount += 1;
        }
    }

    private void OnTriggerExit(Collider collider)
    {
        FinishLineFlag = false;
        CheckpointFlag = false;
    }

    public void Start()
    {
        VehicleRigidbody = gameObject.GetComponent<Rigidbody>();
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