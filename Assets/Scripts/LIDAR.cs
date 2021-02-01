using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LIDAR : MonoBehaviour
{
		/*
		This script simulates a 2D LIDAR using raycasting technique. The raycasts
		measure ranges of occluding objects at respective angles. The `ScanRate`
		property determines the scanning frequency (Hz) while the 'MeasurementsPerScan'
		property determines the scanning resolution.
		*/

		public GameObject Scanner; // Reference `Scanner` gameobject
		public GameObject Head; // Reference `Head` gameobject
		[Range(0,10)] public float ScanRate; // LIDAR scanning rate (Hz)

		public float MinimumRange = 0.15f; // LIDAR minimum range (m)
		public float MaximumRange = 12f; // LIDAR maximum range (m)
		public float Intensity = 47.0f; // Intensity of the laser ray

		private int MeasurementsPerScan = 360; // Number of measurements per scan
		private string[] RangeArray = new string[360]; // Array storing range values of a scan
		private string[] IntensityArray = new string[360]; // Array storing range values of a scan
		private float timer = 0f; // Timer to synchronize laser scan updates

		public string CurrentMeasurement;

		public float CurrentScanRate{get{return ScanRate;}}
		public string[] CurrentRangeArray{get{return RangeArray;}}
		public string[] CurrentIntensityArray{get{return IntensityArray;}}

		void FixedUpdate()
		{
			// LASER SCAN
			Scanner.transform.Rotate(Vector3.up, Time.deltaTime*360*ScanRate); // Spin the scanner

			Vector3 LaserVector = Scanner.transform.TransformDirection(Vector3.forward) * 12; // 12 m long vector pointing in the forward direction (i.e. local Z-axis)
			Ray LaserRay = new Ray(Scanner.transform.position, LaserVector); // Initialize a ray w.r.t. the vector at the origin of the `Scanner` transform
			//Debug.DrawRay(Scanner.transform.position, LaserVector, Color.red); // Visually draw the raycast in scene for debugging purposes
			RaycastHit MeasurementRayHit; // Initialize a raycast hit object
			if(Physics.Raycast(LaserRay, out MeasurementRayHit)) CurrentMeasurement = (MeasurementRayHit.distance+MinimumRange).ToString("F2"); // Update the `CurrentMeasurement` value to the `hit distance` if the ray is colliding
			else CurrentMeasurement = "inf"; // Update the `CurrentMeasurement` value to `inf`, otherwise

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
				float angle = (Head.transform.eulerAngles.y)*(Mathf.PI/180); // Reset angle to align w.r.t. `Head`

				// LASER SCAN
				for(int i=0; i<MeasurementsPerScan; i++)
				{
						float x = Mathf.Sin(angle); // sin(angle in radians)
						float z = Mathf.Cos(angle); // cos(angle in radians)
						angle -= (2*Mathf.PI)/MeasurementsPerScan; // Update angle (anti-clockwise)
						Vector3 LaserRayDirection = new Vector3(x*MaximumRange, 0, z*MaximumRange); // Compute direction of the raycast
						RaycastHit hit; // Instantiate a raycast hit object
						/*
						if(i==0)
						{
								Debug.DrawRay(Head.transform.position, LaserRayDirection, Color.green); // Visually draw the raycast in scene for debugging purposes (green color to indicate 1st ray)
						}
						else if(i==89)
						{
								Debug.DrawRay(Head.transform.position, LaserRayDirection, Color.blue); // Visually draw the raycast in scene for debugging purposes (blue color to indicate 90th ray)
						}
						else
						{
								Debug.DrawRay(Head.transform.position, LaserRayDirection, Color.red); // Visually draw the raycast in scene for debugging purposes (red color to indicate other rays)
						}
						*/
						if(Physics.Raycast(Head.transform.position, LaserRayDirection, out hit, MaximumRange) && hit.distance>MinimumRange) RangeArray[i] = (hit.distance).ToString(); // Update the range measurement to the `hit distance` if the ray is colliding
						else RangeArray[i] = "inf"; // Update the range measurement to `inf`, otherwise

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
