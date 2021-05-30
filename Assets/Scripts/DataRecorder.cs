using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class DataRecorder : MonoBehaviour
{
    /*
    This script logs sensory data of the ego vehicle and saves it to the specified directory.
    */

    public const string CSVFileName = "Driving Log.csv";
    public const string CameraFramesDirectory = "Camera Frames";
    private string saveLocation = "";
    public Text RecordStatus;

    public GameObject Vehicle; // Vehicle gameobject
    public Rigidbody VehicleRigidBody; // Rigidbody component of the vehicle
    public VehicleController VehicleController; // `VehicleController` reference
    public WheelEncoder LeftWheelEncoder; // `WheelEncoder` reference for left wheel
    public WheelEncoder RightWheelEncoder; // `WheelEncoder` reference for right wheel
    public IPS IPS; // `IPS` reference
    public IMU IMU; // `IMU` reference
    public LIDAR LIDAR; // `LIDAR` reference
    public Camera FrontCamera; // Vehicle front camera
    public Camera RearCamera; // Vehicle rear camera

    private string LIDARRangeArray;

    private Queue<DataSample> dataSamples;
    private int totalSamples;
    private bool recording = false;
    private bool saveRecording = false;
    private bool isRecording = false;
    private bool isSaving = false;
    private Vector3 saved_position;
    private Quaternion saved_rotation;
    private float saved_velocity;


    public bool RecordingStatus
    {
        get {return isRecording;}
        set
        {
            isRecording = value;
            if(value == true)
            {
          			//Debug.Log("Starting data recording...");
          			dataSamples = new Queue<DataSample>();
          			StartCoroutine(Sample());
            }
            else
            {
                //Debug.Log("Stopping data recording...");
                StopCoroutine(Sample());
                //Debug.Log("Writing data to disk...");
          			// Save the vehicle coordinate parameters so as to reset after capturing data
          			saved_position = Vehicle.transform.position;
          			saved_rotation = Vehicle.transform.rotation;
          			// Count samples captured to compute save percentage
          			totalSamples = dataSamples.Count;
          			isSaving = true;
          			StartCoroutine(WriteSamplesToDisk());
            }
        }
    }

    public float getSavePercent()
    {
        return (float)(totalSamples-dataSamples.Count)/totalSamples;
    }

    public bool getSaveStatus()
    {
        return isSaving;
    }

    public bool checkSaveLocation()
    {
      	if (saveLocation != "") return true;
      	else SimpleFileBrowser.ShowSaveDialog(OpenFolder, null, true, null, "Select Output Folder", "Select");
      	return false;
    }

    public float getSavedVelocity()
    {
        return saved_velocity;
    }

    void Start() {
        recording = false;
        saveRecording = false;
        isRecording = false;
        isSaving = false;
        RecordStatus.text = "Record Data";
    }

    void Update () {
        if (Input.GetKeyDown(KeyCode.R)) ToggleRecording(); // Toggle data recording using `R` key
        if (getSaveStatus()) {
            RecordStatus.text = "Saving Data: " + (int)(100*getSavePercent()) + "%";
        }
        else if(saveRecording)
        {
            recording = false;
            saveRecording = false;
            RecordStatus.text = "Record Data";
        }
    }

    public void ToggleRecording()
    {
        if (!recording) {
            if (checkSaveLocation()) {
                RecordStatus.text = "Recording Data";
                recording = true;
                RecordingStatus = true;
            }
        }
        else
        {
            saveRecording = true;
            RecordingStatus = false;
        }
    }

    public IEnumerator Sample()
    {
        // Start the co-routine to capture data every second.
        yield return new WaitForSeconds(0.0666666666666667f);

        if (saveLocation != "")
        {
            DataSample sample = new DataSample();
            sample.timeStamp = System.DateTime.Now.ToString("yyyy_MM_dd_HH_mm_ss_fff");
            sample.position = Vehicle.transform.position;
            sample.rotation = Vehicle.transform.rotation;
            sample.throttle = VehicleController.CurrentThrottle;
            sample.steeringAngle = VehicleController.CurrentSteeringAngle;
            sample.leftEncoderTicks = LeftWheelEncoder.Ticks;
            sample.rightEncoderTicks = RightWheelEncoder.Ticks;;
            sample.positionX = IPS.CurrentPosition[0];
            sample.positionY = IPS.CurrentPosition[1];
            sample.yaw = IMU.CurrentOrientationEulerAngles[2];
            sample.velocity = (float)System.Math.Round(VehicleController.Vehicle.transform.InverseTransformDirection(VehicleRigidBody.velocity).z,2);
            dataSamples.Enqueue(sample);
            sample = null; // Nullify the `sample` variable to avoid recording same data in next loop (may or may not be needed)
        }

        // Only reschedule if the data recording button hasn't toggled
        if (RecordingStatus)
        {
            StartCoroutine(Sample());
        }
    }

    public IEnumerator WriteSamplesToDisk()
    {
      	yield return new WaitForSeconds(0.000f); // Retrieve as fast as possible, while still allowing communication of main thread with screen
      	if (dataSamples.Count > 0) {
        		// Pull off a data sample from the queue
        		DataSample sample = dataSamples.Dequeue();

        		// Pysically move the vehicle to get the correct camera frame(s)
        		Vehicle.transform.position = sample.position;
        		Vehicle.transform.rotation = sample.rotation;

            saved_velocity = sample.velocity; // Update `saved_velocity` variable

        		// Capture and store the camera frame(s)
        		string FrontCameraPath = WriteImage(FrontCamera, "Front_Camera_Frame", sample.timeStamp);
        		string RearCameraPath = WriteImage(RearCamera, "Rear_Camera_Frame", sample.timeStamp);
            if(LIDAR.CurrentRangeArray[359] != null)
            {
                for(int i=0;i<359;i++) LIDARRangeArray += LIDAR.CurrentRangeArray[i] + " ";
                LIDARRangeArray += LIDAR.CurrentRangeArray[359];
            }
            // Log data
        		string row = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}\n", sample.throttle, sample.steeringAngle, sample.leftEncoderTicks, sample.rightEncoderTicks, sample.positionX, sample.positionY, sample.yaw, sample.velocity, FrontCameraPath, RearCameraPath, LIDARRangeArray);
        		File.AppendAllText(Path.Combine(saveLocation, CSVFileName), row);
            LIDARRangeArray = ""; // Nullify the `LIDARRangeArray` variable to avoid concatinating new data with the old one
    	}
    	if (dataSamples.Count > 0) {
      		// Request if there are more data samples to pull
      		StartCoroutine(WriteSamplesToDisk());
    	}
    	else
    	{
      		// All data samples have been pulled, stop the recording
      		StopCoroutine(WriteSamplesToDisk());
      		isSaving = false;

      		// Reset the vehicle to its saved coordinate parameters
      		Vehicle.transform.position = saved_position;
      		Vehicle.transform.rotation = saved_rotation;
      		VehicleRigidBody.velocity = Vector3.zero;
    	}
    }

    private string WriteImage(Camera camera, string prepend, string timestamp)
    {
        camera.Render(); // Force camera update
        RenderTexture targetTexture = camera.targetTexture;
        RenderTexture.active = targetTexture;
        Texture2D texture2D = new Texture2D (targetTexture.width, targetTexture.height, TextureFormat.RGB24, false);
        texture2D.ReadPixels(new Rect (0, 0, targetTexture.width, targetTexture.height), 0, 0);
        texture2D.Apply();
        byte[] image = texture2D.EncodeToJPG();
        UnityEngine.Object.DestroyImmediate(texture2D);
        string directory = Path.Combine(saveLocation, CameraFramesDirectory);
        string path = Path.Combine(directory, prepend + "_" + timestamp + ".jpg");
        File.WriteAllBytes(path, image);
        image = null; // Nullify the `image` variable to avoid recording same frame in next loop (may or may not be needed)
        return path;
    }

    private void OpenFolder(string location)
    {
        saveLocation = location;
        Directory.CreateDirectory(Path.Combine(saveLocation, CameraFramesDirectory));
    }
}

internal class DataSample
{
    public string timeStamp;
    public Vector3 position;
    public Quaternion rotation;
    // Sensory data
    public float throttle;
    public float steeringAngle;
    public float leftEncoderTicks;
    public float rightEncoderTicks;
    public float positionX;
    public float positionY;
    public float yaw;
    public float velocity;
}
