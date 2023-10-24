using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LIDAR : MonoBehaviour
{
		/*
		This script simulates a 2D LIDAR using raycasting technique. The raycasts measure
		ranges of occluding objects with a linear range (m) between `MinimumLinearRange` and
		`MaximumLinearRange`, and an angular range (deg) between `MinimumAngularRange` and
		`MaximumAngularRange`. The `ScanRate` property determines the scanning frequency
		(Hz) while the 'Resolution' property determines the scanning resolution (deg).
		*/

		public bool AnimateScanner = true; // Animate scanning head of LIDAR
		public bool ShowRaycasts = false; // Visualize raycasts
		public GameObject Scanner; // Reference `Scanner` gameobject
		public GameObject Head; // Reference `Head` gameobject

		[Range(0,100)] public float ScanRate = 7; // LIDAR scanning rate (Hz)
		public float MinimumLinearRange = 0.15f; // LIDAR minimum linear range (m)
		public float MaximumLinearRange = 12f; // LIDAR maximum linear range (m)
		public float MinimumAngularRange = 0; // LIDAR minimum angular range (deg)
		public float MaximumAngularRange = 359; // LIDAR maximum angular range (deg)
		public float Resolution = 1; // Angular resolution (deg)
		public float Intensity = 47.0f; // Intensity of the laser ray

		private int MeasurementsPerScan; // Measurements per scan
		private string[] RangeArray; // Array storing range values of a scan
		private string[] IntensityArray; // Array storing range values of a scan
		private float timer = 0f; // Timer to synchronize laser scan updates

		public string CurrentMeasurement;

		public float CurrentScanRate{get{return ScanRate;}}
		public string[] CurrentRangeArray{get{return RangeArray;}}
		public string[] CurrentIntensityArray{get{return IntensityArray;}}

		private int layer_mask = 1 << 0; // Mask the `Default` layer to allow raycasting only against it

		private void Start()
		{
			MeasurementsPerScan = (int) ((MaximumAngularRange-MinimumAngularRange)/Resolution + 1); // Compute number of measurements per scan
			// Debug.Log(MeasurementsPerScan);
			RangeArray = new string[MeasurementsPerScan]; // Array storing range values of a scan
			IntensityArray = new string[MeasurementsPerScan]; // Array storing range values of a scan
		}

		void FixedUpdate()
		{
			// LASER SCAN
			if(AnimateScanner)
			{
					Scanner.transform.Rotate(Vector3.up, Time.deltaTime*360*ScanRate); // Spin the scanner
					Vector3 LaserVector = Scanner.transform.TransformDirection(Vector3.forward) * 12; // 12 m long vector pointing in the forward direction (i.e. local Z-axis)
					Ray LaserRay = new Ray(Scanner.transform.position, LaserVector); // Initialize a ray w.r.t. the vector at the origin of the `Scanner` transform
					//Debug.DrawRay(Scanner.transform.position, LaserVector, Color.red); // Visually draw the raycast in scene for debugging purposes
					RaycastHit MeasurementRayHit; // Initialize a raycast hit object
					if(Physics.Raycast(LaserRay, out MeasurementRayHit, layer_mask)) CurrentMeasurement = (MeasurementRayHit.distance+MinimumLinearRange).ToString("F2"); // Update the `CurrentMeasurement` value to the `hit distance` if the ray is colliding
					else CurrentMeasurement = "inf"; // Update the `CurrentMeasurement` value to `inf`, otherwise
					// Debug.Log(CurrentMeasurement); // Log the `CurrentMeasurement` to Unity Console
			}
			else CurrentMeasurement = RangeArray[(int)(MeasurementsPerScan/2)];

			timer = timer + Time.deltaTime; // Update timer
			if(timer < 1/ScanRate)
			{
					// Scan
			}
			if(timer >= 1/ScanRate)
			{
					LaserScan(); // Report the scan
					timer = 0; // Reset timer
			}
		}

		void LaserScan()
		{
				float angle = (Head.transform.eulerAngles.y-MinimumAngularRange)*(Mathf.PI/180); // Reset angle to align w.r.t. starting point

				// LASER SCAN
				for(int i=0; i<MeasurementsPerScan; i++)
				{
						float x = Mathf.Sin(angle); // sin(angle in radians)
						float z = Mathf.Cos(angle); // cos(angle in radians)
						if(MinimumAngularRange<=0) angle -= Resolution*(Mathf.PI/180); // Update angle (anti-clockwise)
						else angle += Resolution*(Mathf.PI/180); // Update angle (clockwise)
						
						Vector3 LaserRayDirection = new Vector3(x*MaximumLinearRange, 0, z*MaximumLinearRange); // Compute direction of the raycast
						RaycastHit hit; // Instantiate a raycast hit object
						if(Physics.Raycast(Head.transform.position, LaserRayDirection, out hit, MaximumLinearRange, layer_mask) && hit.distance>MinimumLinearRange) RangeArray[i] = (hit.distance).ToString(); // Update the range measurement to the `hit distance` if the ray is colliding
						else RangeArray[i] = "inf"; // Update the range measurement to `inf`, otherwise
						
						// /*
						if(ShowRaycasts)
						{
								float range = 0; // Initialize current range value
								if (hit.distance <= MaximumLinearRange) range = hit.distance; // Update current range value
								else range = MaximumLinearRange; // Clip current range value to `MaximumLinearRange`
								Vector3 DebugRayDirection = new Vector3(x*range, 0, z*range); // Compute direction of the raycast
								if(i==0)
								{
										Debug.DrawRay(Head.transform.position, DebugRayDirection, Color.green, 1/ScanRate); // Visually draw the raycast in scene for debugging purposes (green color to indicate 1st ray)
								}
								else if(i==(int)(MeasurementsPerScan/2))
								{
										Debug.DrawRay(Head.transform.position, DebugRayDirection, Color.blue, 1/ScanRate); // Visually draw the raycast in scene for debugging purposes (blue color to indicate 90th ray)
								}
								else
								{
										Debug.DrawRay(Head.transform.position, DebugRayDirection, Color.red, 1/ScanRate); // Visually draw the raycast in scene for debugging purposes (red color to indicate other rays)
								}
						}
						// */

						IntensityArray[i] = Intensity.ToString(); // Update the intensity
				}

				// LOG LASER SCAN
				/*
				// Range Array
				string RangeArrayString = "Range Array: "; // Initialize `RangeArrayString`
				foreach(var item in RangeArray)
				{
						RangeArrayString += item + " "; // Concatenate the `RangeArrayString` with all the elements in `RangeArray`
				}
				Debug.Log(RangeArrayString); // Log the `RangeArrayString` to Unity Console

				// Intensity Array
				string IntensityArrayString = "Intensity Array: "; // Initialize `RangeArrayString`
				foreach(var item in IntensityArray)
				{
						IntensityArrayString += item + " "; // Concatenate the `RangeArrayString` with all the elements in `RangeArray`
				}
				Debug.Log(IntensityArrayString); // Log the `RangeArrayString` to Unity Console
				*/
		}
}