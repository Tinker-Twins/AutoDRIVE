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
    private int IgnoreRacetrackRespawnUntilFrame = -1;
    private bool IgnoreVehicleCollisionAfterRespawn = false;
    private Vector3 VehicleCollisionRespawnPosition;
    private long VehicleCollisionGracePairKey = -1;
    private const float VehicleCollisionRespawnOffsetMultiplier = 1.0f;
    private const float VehicleCollisionGraceExitDistanceMultiplier = 4.0f;
    private static Dictionary<long, int> vehicleCollisionFrames = new Dictionary<long, int>();

    void OnCollisionEnter(Collision collision)
    {
        LapTimer otherVehicle = collision.collider.GetComponentInParent<LapTimer>();
        if (otherVehicle != null && otherVehicle != this)
        {
            UpdateVehicleCollisionGrace();
            otherVehicle.UpdateVehicleCollisionGrace();
            long collisionPairKey = GetCollisionPairKey(this, otherVehicle);
            if (IsIgnoringVehicleCollisionWith(collisionPairKey) && otherVehicle.IsIgnoringVehicleCollisionWith(collisionPairKey))
            {
                return;
            }

            int lastCollisionFrame;
            if (vehicleCollisionFrames.TryGetValue(collisionPairKey, out lastCollisionFrame) && lastCollisionFrame == Time.frameCount)
            {
                return;
            }

            vehicleCollisionFrames[collisionPairKey] = Time.frameCount;
            RespawnCollisionPairSideBySide(this, otherVehicle);
            return;
        }

        if (collision.collider.name == RacetrackName && Time.frameCount > IgnoreRacetrackRespawnUntilFrame) Respawn(); // Collision detected with racetrack
    }

    private static long GetCollisionPairKey(LapTimer first, LapTimer second)
    {
        int firstID = first.GetInstanceID();
        int secondID = second.GetInstanceID();
        int minID = Mathf.Min(firstID, secondID);
        int maxID = Mathf.Max(firstID, secondID);

        return ((long)minID << 32) ^ (uint)maxID;
    }

    private static void RespawnCollisionPairSideBySide(LapTimer first, LapTimer second)
    {
        bool firstOnLeft = first.GetInstanceID() < second.GetInstanceID();
        int firstRespawnCheckpointCount = first.GetEffectiveRespawnCheckpointCount();
        int secondRespawnCheckpointCount = second.GetEffectiveRespawnCheckpointCount();
        int firstProgress = first.GetEffectiveRespawnProgress(firstRespawnCheckpointCount);
        int secondProgress = second.GetEffectiveRespawnProgress(secondRespawnCheckpointCount);
        LapTimer leadingVehicle = firstProgress >= secondProgress ? first : second;
        int respawnCheckpointCount = leadingVehicle == first ? firstRespawnCheckpointCount : secondRespawnCheckpointCount;
        Transform respawnCheckpoint = leadingVehicle.Checkpoints[respawnCheckpointCount%leadingVehicle.Checkpoints.Length];
        float lateralOffset = Mathf.Max(
            first.GetVehicleWidth(respawnCheckpoint.right),
            second.GetVehicleWidth(respawnCheckpoint.right)
        ) * VehicleCollisionRespawnOffsetMultiplier;

        long collisionPairKey = GetCollisionPairKey(first, second);
        first.RespawnWithLateralOffset(respawnCheckpointCount, respawnCheckpoint, firstOnLeft ? -lateralOffset : lateralOffset, collisionPairKey);
        second.RespawnWithLateralOffset(respawnCheckpointCount, respawnCheckpoint, firstOnLeft ? lateralOffset : -lateralOffset, collisionPairKey);
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

    public void Respawn()
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

    private void RespawnWithLateralOffset(int checkpointCount, Transform respawnCheckpoint, float lateralOffset, long collisionPairKey)
    {
        IgnoreRacetrackRespawnUntilFrame = Time.frameCount + 2;
        VehicleRigidbody.velocity = Vector3.zero;
        VehicleRigidbody.angularVelocity = Vector3.zero;

        CheckpointCount = checkpointCount;
        CurrentCheckpoint = checkpointCount%Checkpoints.Length;
        PreviousCheckpoint = CurrentCheckpoint;
        SavedCheckpoint = respawnCheckpoint;
        gameObject.transform.position = SavedCheckpoint.position + SavedCheckpoint.right * lateralOffset;
        gameObject.transform.rotation = SavedCheckpoint.rotation;
        IgnoreVehicleCollisionAfterRespawn = true;
        VehicleCollisionRespawnPosition = gameObject.transform.position;
        VehicleCollisionGracePairKey = collisionPairKey;

        CollisionCount = CollisionCount + 1;
    }

    private int GetEffectiveRespawnCheckpointCount()
    {
        if (CheckpointCount == 0 && PreviousCheckpoint > 0) return PreviousCheckpoint;
        return CheckpointCount;
    }

    private int GetEffectiveRespawnProgress(int respawnCheckpointCount)
    {
        return LapCount * Checkpoints.Length + respawnCheckpointCount;
    }

    private float GetVehicleWidth(Vector3 lateralDirection)
    {
        float controllerWidth = GetControllerVehicleWidth();
        if (controllerWidth > 0.0f) return controllerWidth;

        return GetColliderVehicleWidth(lateralDirection);
    }

    private float GetControllerVehicleWidth()
    {
        VehicleController vehicleController = GetComponent<VehicleController>();
        if (vehicleController != null) return vehicleController.TrackWidth / 1000.0f;

        AutomobileController automobileController = GetComponent<AutomobileController>();
        if (automobileController != null) return automobileController.TrackWidth;

        return 0.0f;
    }

    private float GetColliderVehicleWidth(Vector3 lateralDirection)
    {
        Collider[] colliders = GetComponentsInChildren<Collider>();
        if (colliders.Length == 0) return 0.0f;

        lateralDirection.Normalize();
        bool hasBounds = false;
        float minProjection = 0.0f;
        float maxProjection = 0.0f;

        for (int i = 0; i < colliders.Length; i++)
        {
            if (colliders[i].isTrigger) continue;

            Bounds bounds = colliders[i].bounds;
            Vector3 center = bounds.center;
            Vector3 extents = bounds.extents;
            float centerProjection = Vector3.Dot(center, lateralDirection);
            float projectedExtent = Mathf.Abs(lateralDirection.x) * extents.x
                + Mathf.Abs(lateralDirection.y) * extents.y
                + Mathf.Abs(lateralDirection.z) * extents.z;

            float colliderMinProjection = centerProjection - projectedExtent;
            float colliderMaxProjection = centerProjection + projectedExtent;
            if (!hasBounds)
            {
                minProjection = colliderMinProjection;
                maxProjection = colliderMaxProjection;
                hasBounds = true;
            }
            else
            {
                minProjection = Mathf.Min(minProjection, colliderMinProjection);
                maxProjection = Mathf.Max(maxProjection, colliderMaxProjection);
            }
        }

        return hasBounds ? maxProjection - minProjection : 0.0f;
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
        UpdateVehicleCollisionGrace();

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

    private void UpdateVehicleCollisionGrace()
    {
        if (!IgnoreVehicleCollisionAfterRespawn) return;

        float graceExitDistance = GetVehicleWidth(transform.right) * VehicleCollisionGraceExitDistanceMultiplier;
        if (Vector3.Distance(transform.position, VehicleCollisionRespawnPosition) > graceExitDistance)
        {
            IgnoreVehicleCollisionAfterRespawn = false;
            VehicleCollisionGracePairKey = -1;
        }
    }

    private bool IsIgnoringVehicleCollisionWith(long collisionPairKey)
    {
        return IgnoreVehicleCollisionAfterRespawn && VehicleCollisionGracePairKey == collisionPairKey;
    }

    public void FixedUpdate()
    {
        LapTime += Time.fixedDeltaTime; // Update lap time
    }
}
