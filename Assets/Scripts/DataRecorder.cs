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
    This script records sensory data of the all vehicles and states of all
    traffic lights, and logs it to the specified directory.
    */

    public GameObject eventSystem; // Default event system

    public bool HotKey = true; // Enable hot-key "R"
    public float RecordRate = 10.0f; // Data recording rate (Hz)
    public Text RecordStatus; // Data recording status
    private string saveLocation = ""; // Data saving location

    [HideInInspector] public string[] VehicleDataFileNames; // CSV file names for storing data of vehicle(s)
    [HideInInspector] public string[] VehicleCameraDirectories; // Directory names for storing camera frames from vehicle(s)
    [HideInInspector] public string[] TrafficLightDataFileNames; // CSV file names for storing data of traffic light(s)

    public GameObject[] Vehicles; // Vehicle gameobject references
    public Rigidbody[] VehicleRigidBodies; // Rigidbody component of the vehicles
    public VehicleController[] VehicleControllers; // `VehicleController` references
    public bool LogAutomobileController = false; // Flag to enable logging information from `AutomobileController`
    public AutomobileController[] AutomobileControllers; // `VehicleController` references
    public VehicleLighting[] VehicleLightings; // `VehicleLighting` references
    public WheelEncoder[] LeftWheelEncoders; // `WheelEncoder` references for left wheel
    public WheelEncoder[] RightWheelEncoders; // `WheelEncoder` references for right wheel
    public GPS[] PositioningSystems; // `GPS` references
    public IMU[] InertialMeasurementUnits; // `IMU` references
    public LIDAR[] LIDARUnits; // `LIDAR` references
    public TF[] Transforms; // `TF` references
    private string LIDARRangeArray;
    public Camera[] FrontCameras; // Vehicle front camera references
    public Camera[] RearCameras; // Vehicle rear camera references
    public GameObject[] NonFixedCameras; // Non-fixed cameras gameobject references
    private string FrontCameraPath;
    private string RearCameraPath;

    public TLController[] TrafficLightControllers; // Traffic light controller references

    private int totalSamples;
    private int recordedSamples;
    private bool recording = false;
    private bool saveRecording = false;
    private bool isRecording = false;
    private bool isSaving = false;

    private List<Queue<VehicleDataSample>> VehicleDataSamples = new List<Queue<VehicleDataSample>>();
    private List<Queue<TrafficLightDataSample>> TrafficLightDataSamples = new List<Queue<TrafficLightDataSample>>();
    private List<Vector3> saved_vehicle_positions = new List<Vector3>();
    private List<Quaternion> saved_vehicle_rotations = new List<Quaternion>();

    private List<Vector3> saved_camera_positions = new List<Vector3>();
    private List<Quaternion> saved_camera_rotations = new List<Quaternion>();

    public bool RecordingStatus
    {
        get {return isRecording;}
        set
        {
            isRecording = value;
            if(value == true)
            {
          		//Debug.Log("Starting data recording...");
                // Create data sample queues for storing data of all vehicles
                for(int i=0;i<VehicleControllers.Length+AutomobileControllers.Length; i++)
                {
                    VehicleDataSamples.Add(new Queue<VehicleDataSample>());
                }
                // Create data sample queues for storing data of all traffic lights
                for(int i=0;i<TrafficLightControllers.Length;i++)
                {
                    TrafficLightDataSamples.Add(new Queue<TrafficLightDataSample>());
                }
          			StartCoroutine(Sample());
            }
            else
            {
                //Debug.Log("Stopping data recording...");
                StopCoroutine(Sample());
                //Debug.Log("Writing data to disk...");
                // Save the vehicle(s) pose parameters so as to reset after capturing data
                saved_vehicle_positions = new List<Vector3>(); // Reset for next iteration
                saved_vehicle_rotations = new List<Quaternion>(); // Reset for next iteration
                saved_camera_positions = new List<Vector3>(); // Reset for next iteration
                saved_camera_rotations = new List<Quaternion>(); // Reset for next iteration
                for (int i=0;i<Vehicles.Length;i++)
                {
                    saved_vehicle_positions.Add(Vehicles[i].transform.position);
                    saved_vehicle_rotations.Add(Vehicles[i].transform.rotation);
                }
                for (int i = 0; i < NonFixedCameras.Length; i++)
                {
                    saved_camera_positions.Add(NonFixedCameras[i].transform.position);
                    saved_camera_rotations.Add(NonFixedCameras[i].transform.rotation);
                }
                // Count data samples captured to compute save percentage
                totalSamples = 0; // Reset for next iteration
                // Count data samples captured from all vehicles
                for(int i=0;i<VehicleControllers.Length+AutomobileControllers.Length; i++)
                {
                    totalSamples += VehicleDataSamples[i].Count;
                }
                // Count data samples captured from all traffic lights
                for(int i=0;i<TrafficLightControllers.Length;i++)
                {
                    totalSamples += TrafficLightDataSamples[i].Count;
                }
          			isSaving = true;
          			StartCoroutine(WriteSamplesToDisk());
            }
        }
    }

    public float getSavePercent()
    {
        recordedSamples = 0; // Reset for next iteration
        // Count data samples written from all vehicles
        for(int i=0;i<VehicleControllers.Length+AutomobileControllers.Length; i++)
        {
            recordedSamples += VehicleDataSamples[i].Count;
        }
        // Count data samples written from all traffic lights
        for(int i=0;i<TrafficLightControllers.Length;i++)
        {
            recordedSamples += TrafficLightDataSamples[i].Count;
        }
        return (float)(totalSamples-recordedSamples)/totalSamples;
    }

    public bool getSaveStatus()
    {
        return isSaving;
    }

    public bool checkSaveLocation()
    {
      	if (saveLocation != "") return true;
      	else
        {
            eventSystem.SetActive(false);
            SimpleFileBrowser.ShowSaveDialog(OpenFolder, null, true, null, "Select Output Folder", "Select");
        }
      	return false;
    }

    void Start() {
        recording = false;
        saveRecording = false;
        isRecording = false;
        isSaving = false;
        RecordStatus.text = "Record Data";
        // Temporary render textures for all vehicles except the first one (first vehicle will render to GUI)
        if(FrontCameras.Length != 0)
        {
            for(int i=1;i<FrontCameras.Length;i++)
            {
                FrontCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
        if(RearCameras.Length != 0)
        {
            for(int i=1;i<RearCameras.Length;i++)
            {
                RearCameras[i].targetTexture = new RenderTexture(1280, 720, 16, RenderTextureFormat.ARGB32);
            }
        }
        // Create CSV files and directories for storing data and camera frames of all vehicles
        VehicleDataFileNames = new string[VehicleControllers.Length + AutomobileControllers.Length];
        VehicleCameraDirectories = new string[VehicleControllers.Length + AutomobileControllers.Length];
        for(int i=0;i<VehicleControllers.Length+AutomobileControllers.Length; i++)
        {
            VehicleDataFileNames[i] = "V"+(i+1).ToString()+" Log.csv";
            VehicleCameraDirectories[i] = "V"+(i+1).ToString()+" Camera Frames";
        }
        // Create CSV files for storing data of all traffic lights
        TrafficLightDataFileNames = new string[TrafficLightControllers.Length];
        for(int i=0;i<TrafficLightControllers.Length;i++)
        {
            TrafficLightDataFileNames[i] = "TL"+(i+1).ToString()+" Log.csv";
        }
    }

    void Update () {
        if (HotKey && Input.GetKeyDown(KeyCode.R)) ToggleRecording(); // Toggle data recording using `R` key
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
        // Start the co-routine to capture data at approximately every second.
        yield return new WaitForSeconds(1/RecordRate);

        if (saveLocation != "")
        {
            // Sample data from all vehicles
            for(int i=0;i<VehicleControllers.Length+AutomobileControllers.Length; i++)
            {
                VehicleDataSample sample = new VehicleDataSample();
                sample.timeStamp = System.DateTime.Now.ToString("yyyy_MM_dd_HH_mm_ss_fff");
                sample.VehiclePosition = Vehicles[i].transform.position;
                sample.VehicleRotation = Vehicles[i].transform.rotation;
                if (NonFixedCameras.Length != 0)
                {
                    sample.CameraPosition = NonFixedCameras[i].transform.position;
                    sample.CameraRotation = NonFixedCameras[i].transform.rotation;
                }
                if (LogAutomobileController) sample.throttle = AutomobileControllers[i].CurrentThrottle;
                else sample.throttle = VehicleControllers[i].CurrentThrottle;
                if (LogAutomobileController) sample.brake = AutomobileControllers[i].CurrentBrake;
                else sample.brake = (VehicleControllers[i].CurrentThrottle == 0) ? 1 : 0;
                if (LogAutomobileController) sample.handBrake = AutomobileControllers[i].CurrentHandbrake;
                else sample.handBrake = 0;
                if (LogAutomobileController) sample.steeringAngle = AutomobileControllers[i].CurrentSteeringAngle;
                else sample.steeringAngle = VehicleControllers[i].CurrentSteeringAngle;
                sample.leftEncoderTicks = LeftWheelEncoders[i].Ticks;
                sample.rightEncoderTicks = RightWheelEncoders[i].Ticks;;
                sample.positionX = PositioningSystems[i].CurrentPosition[0];
                sample.positionY = PositioningSystems[i].CurrentPosition[1];
                sample.positionZ = PositioningSystems[i].CurrentPosition[2];
                sample.roll = InertialMeasurementUnits[i].CurrentOrientationEulerAngles[0];
                sample.pitch = InertialMeasurementUnits[i].CurrentOrientationEulerAngles[1];
                sample.yaw = InertialMeasurementUnits[i].CurrentOrientationEulerAngles[2];
                if (LogAutomobileController) sample.velocity = (float)System.Math.Round(AutomobileControllers[0].currSpeed/AutomobileControllers[0].speedMultiplier, 2);
                else sample.velocity = (float)System.Math.Round(VehicleControllers[i].Vehicle.transform.InverseTransformDirection(VehicleRigidBodies[i].velocity).z, 2);
                sample.angularX = InertialMeasurementUnits[i].CurrentAngularVelocity[0];
                sample.angularY = InertialMeasurementUnits[i].CurrentAngularVelocity[1];
                sample.angularZ = InertialMeasurementUnits[i].CurrentAngularVelocity[2];
                sample.accelX = InertialMeasurementUnits[i].CurrentLinearAcceleration[0];
                sample.accelY = InertialMeasurementUnits[i].CurrentLinearAcceleration[1];
                sample.accelZ = InertialMeasurementUnits[i].CurrentLinearAcceleration[2];
                if (Transforms.Length != 0)
                {
                    sample.Cam0PosX = Transforms[0].CurrentPosition[0];
                    sample.Cam0PosY = Transforms[0].CurrentPosition[1];
                    sample.Cam0PosZ = Transforms[0].CurrentPosition[2];
                    sample.Cam0RotX = Transforms[0].CurrentOrientation[0];
                    sample.Cam0RotY = Transforms[0].CurrentOrientation[1];
                    sample.Cam0RotZ = Transforms[0].CurrentOrientation[2];
                    sample.Cam1PosX = Transforms[1].CurrentPosition[0];
                    sample.Cam1PosY = Transforms[1].CurrentPosition[1];
                    sample.Cam1PosZ = Transforms[1].CurrentPosition[2];
                    sample.Cam1RotX = Transforms[1].CurrentOrientation[0];
                    sample.Cam1RotY = Transforms[1].CurrentOrientation[1];
                    sample.Cam1RotZ = Transforms[1].CurrentOrientation[2];
                }

                VehicleDataSamples[i].Enqueue(sample);
                sample = null; // Nullify the `sample` variable to avoid recording same data in next loop (may or may not be needed)
            }
            // Sample data from all traffic lights
            for(int i=0;i<TrafficLightControllers.Length;i++)
            {
                TrafficLightDataSample sample = new TrafficLightDataSample();
                sample.timeStamp = System.DateTime.Now.ToString("yyyy_MM_dd_HH_mm_ss_fff");
                sample.state = TrafficLightControllers[i].CurrentState;
                TrafficLightDataSamples[i].Enqueue(sample);
                sample = null; // Nullify the `sample` variable to avoid recording same data in next loop (may or may not be needed)
            }
        }
        // Only reschedule if the data recording button hasn't been toggled
        if (RecordingStatus)
        {
            StartCoroutine(Sample());
        }
    }

    public IEnumerator WriteSamplesToDisk()
    {
      	yield return new WaitForSeconds(0.000f); // Retrieve as fast as possible, while still allowing communication of main thread with screen
        // Write data from all vehicles to disk
        for(int i=0;i<VehicleControllers.Length+AutomobileControllers.Length; i++)
        {
            while(VehicleDataSamples[i].Count > 0)
            {
                // Pull off a data sample from the queue
                VehicleDataSample sample = VehicleDataSamples[i].Dequeue();
                // Pysically move the vehicle(s) to get the correct camera frame(s)
                Vehicles[i].transform.position = sample.VehiclePosition;
                Vehicles[i].transform.rotation = sample.VehicleRotation;
                if (NonFixedCameras.Length != 0)
                {
                    NonFixedCameras[i].transform.position = sample.CameraPosition;
                    NonFixedCameras[i].transform.rotation = sample.CameraRotation;
                }
                // Update recorded velocity variable for i-th vehicle
                if (VehicleLightings.Length != 0) VehicleLightings[i].RecordedVelocity = sample.velocity;
                // Capture and store the camera frame(s)
                if(FrontCameras.Length != 0)
                {
                    FrontCameraPath = WriteImage(FrontCameras[i], VehicleCameraDirectories[i], "Camera0_Frame", sample.timeStamp);
                    for(int j=1;j<FrontCameras.Length;j++)
                    {
                        string _FrontCameraPath = WriteImage(FrontCameras[j], VehicleCameraDirectories[i], "Camera"+(j).ToString()+"_Frame", sample.timeStamp);
                    }
                }
                else
                {
                    FrontCameraPath = "";
                }
                if(RearCameras.Length != 0)
                {
                    RearCameraPath = WriteImage(RearCameras[i], VehicleCameraDirectories[i], "Camera"+(FrontCameras.Length).ToString()+"_Frame", sample.timeStamp);
                    for(int j=1;j<RearCameras.Length;j++)
                    {
                        string _RearCameraPath = WriteImage(RearCameras[j], VehicleCameraDirectories[i], "Camera"+(FrontCameras.Length+j).ToString()+"_Frame", sample.timeStamp);
                    }
                }
                else
                {
                    RearCameraPath = "";
                }
                if(LIDARUnits.Length != 0)
                {
                    if(LIDARUnits[i].CurrentRangeArray[LIDARUnits[i].CurrentRangeArray.Length-1] != null)
                    {
                        for(int j=0;j<LIDARUnits[i].CurrentRangeArray.Length-1;j++) LIDARRangeArray += LIDARUnits[i].CurrentRangeArray[j] + " ";
                        LIDARRangeArray += LIDARUnits[i].CurrentRangeArray[LIDARUnits[i].CurrentRangeArray.Length-1];
                    }
                }
                else
                {
                    LIDARRangeArray = "";
                }
                // Log data
                string row = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17}\n",
                    sample.timeStamp, sample.throttle, sample.steeringAngle, sample.leftEncoderTicks, sample.rightEncoderTicks,
                    sample.positionX, sample.positionY, sample.positionZ, sample.roll, sample.pitch, sample.yaw, sample.velocity,
                    sample.angularX, sample.angularY, sample.angularZ, sample.accelX, sample.accelY, sample.accelZ);
                /*string row = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19}\n",
                    sample.timeStamp, sample.throttle, sample.steeringAngle, sample.leftEncoderTicks, sample.rightEncoderTicks,
                    sample.positionX, sample.positionY, sample.positionZ, sample.roll, sample.pitch, sample.yaw, sample.velocity,
                    sample.angularX, sample.angularY, sample.angularZ, sample.accelX, sample.accelY, sample.accelZ,
                    FrontCameraPath, LIDARRangeArray);*/
                /*string row = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19},{20},{21},{22},{23},{24},{25},{26},{27},{28},{29},{30},{31},{32},{33},{34}\n",
                        sample.timeStamp, sample.throttle, sample.brake, sample.handBrake, sample.steeringAngle, sample.leftEncoderTicks, sample.rightEncoderTicks,
                        sample.positionX, sample.positionY, sample.positionZ, sample.roll, sample.pitch, sample.yaw, sample.velocity,
                        sample.angularX, sample.angularY, sample.angularZ, sample.accelX, sample.accelY, sample.accelZ,
                        sample.Cam0PosX, sample.Cam0PosY, sample.Cam0PosZ, sample.Cam0RotX, sample.Cam0RotY, sample.Cam0RotZ, FrontCameraPath,
                        sample.Cam1PosX, sample.Cam1PosY, sample.Cam1PosZ, sample.Cam1RotX, sample.Cam1RotY, sample.Cam1RotZ, RearCameraPath, LIDARRangeArray);*/
                File.AppendAllText(Path.Combine(saveLocation, VehicleDataFileNames[i]), row);
                LIDARRangeArray = ""; // Nullify the `LIDARRangeArray` variable to avoid concatinating new data with the old one
                // Yield after each pass to avoid freezing the simulator upon entering the while loop
                yield return new WaitForSeconds(0.000f);
            }
            // Reset the vehicle(s) to corresponding saved pose parameters [sanity check]
            Vehicles[i].transform.position = saved_vehicle_positions[i];
            Vehicles[i].transform.rotation = saved_vehicle_rotations[i];
            if (NonFixedCameras.Length != 0)
            {
                NonFixedCameras[i].transform.position = saved_camera_positions[i];
                NonFixedCameras[i].transform.rotation = saved_camera_rotations[i];
            }
            VehicleRigidBodies[i].velocity = Vector3.zero;
        }
        // Write data from all traffic lights to disk
        for(int i=0;i<TrafficLightControllers.Length;i++)
        {
            while(TrafficLightDataSamples[i].Count > 0)
            {
            		// Pull off a data sample from the queue
            		TrafficLightDataSample sample = TrafficLightDataSamples[i].Dequeue();
                // Log data
            		string row = string.Format("{0},{1}\n", sample.timeStamp, sample.state);
            		File.AppendAllText(Path.Combine(saveLocation, TrafficLightDataFileNames[i]), row);
                // Yield after each pass to avoid freezing the simulator upon entering the while loop
                yield return new WaitForSeconds(0.000f);
            }
        }
    		// All data samples have been pulled, stop the recording
    		StopCoroutine(WriteSamplesToDisk());
    		isSaving = false;
    		// Reset the vehicle(s) to corresponding saved pose parameters
        for(int i=0;i<Vehicles.Length;i++)
        {
            Vehicles[i].transform.position = saved_vehicle_positions[i];
            Vehicles[i].transform.rotation = saved_vehicle_rotations[i];
            if (NonFixedCameras.Length != 0)
            {
                NonFixedCameras[i].transform.position = saved_camera_positions[i];
                NonFixedCameras[i].transform.rotation = saved_camera_rotations[i];
            }
            VehicleRigidBodies[i].velocity = Vector3.zero;
        }
    }

    private string WriteImage(Camera camera, string folder, string prepend, string timestamp)
    {
        // Create data sample queues for storing data of all vehicles
        camera.Render(); // Force camera update
        RenderTexture targetTexture = camera.targetTexture;
        RenderTexture.active = targetTexture;
        Texture2D texture2D = new Texture2D (targetTexture.width, targetTexture.height, TextureFormat.RGB24, false);
        texture2D.ReadPixels(new Rect (0, 0, targetTexture.width, targetTexture.height), 0, 0);
        texture2D.Apply();
        byte[] image = texture2D.EncodeToJPG();
        UnityEngine.Object.DestroyImmediate(texture2D);
        string directory = Path.Combine(saveLocation, folder);
        string path = Path.Combine(directory, prepend + "_" + timestamp + ".jpg");
        File.WriteAllBytes(path, image);
        image = null; // Nullify the `image` variable to avoid recording same frame in next loop (may or may not be needed)
        return path;
    }

    private void OpenFolder(string location)
    {
        saveLocation = location;
        // Create directories
        for(int i=0;i<VehicleCameraDirectories.Length;i++)
        {
            Directory.CreateDirectory(Path.Combine(saveLocation, VehicleCameraDirectories[i]));
        }
        eventSystem.SetActive(true);
    }
}

internal class VehicleDataSample
{
    public string timeStamp;
    public Vector3 VehiclePosition;
    public Quaternion VehicleRotation;
    public Vector3 CameraPosition;
    public Quaternion CameraRotation;
    // Sensory data
    public float throttle;
    public float brake;
    public float handBrake;
    public float steeringAngle;
    public float leftEncoderTicks;
    public float rightEncoderTicks;
    public float positionX;
    public float positionY;
    public float positionZ;
    public float roll;
    public float pitch;
    public float yaw;
    public float angularX;
    public float angularY;
    public float angularZ;
    public float accelX;
    public float accelY;
    public float accelZ;
    public float velocity;
    // Camera TF
    public float Cam0PosX;
    public float Cam0PosY;
    public float Cam0PosZ;
    public float Cam0RotX;
    public float Cam0RotY;
    public float Cam0RotZ;
    public float Cam1PosX;
    public float Cam1PosY;
    public float Cam1PosZ;
    public float Cam1RotX;
    public float Cam1RotY;
    public float Cam1RotZ;
}

internal class TrafficLightDataSample
{
    public string timeStamp;
    // State data
    public int state;
}
