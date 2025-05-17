using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.Jobs;
using Unity.Collections;

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
	public bool ShowLaserScan = false; // Visualize laser scan
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

	private Mesh LaserScanMesh; // Mesh for laser scan visualization
	private static readonly int visualizationLayerID = 10; // `Sensor Visualization` layer
	public GameObject HUD; // Use HUD to enable laser scan visualization
	public Material LaserScanMaterial; // Material for laser scan visualization
	public float LaserScanSize = 0.01f; // Size for laser scan visualization
	public Color LaserScanColor; // Color for laser scan visualization

	private void Start()
	{
		MeasurementsPerScan = (int) ((MaximumAngularRange-MinimumAngularRange)/Resolution + 1); // Compute number of measurements per scan
		// Debug.Log(MeasurementsPerScan);
		RangeArray = new string[MeasurementsPerScan]; // Array storing range values of a scan
		IntensityArray = new string[MeasurementsPerScan]; // Array storing range values of a scan

		if (ShowLaserScan)
		{
			LaserScanMesh = new Mesh();
			LaserScanMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
		}
	}

	void FixedUpdate()
	{
		// LASER SCAN
		if(AnimateScanner)
		{
			Scanner.transform.Rotate(Vector3.up, Time.deltaTime*360*ScanRate); // Spin the scanner
			Vector3 LaserVector = Scanner.transform.TransformDirection(Vector3.forward) * MaximumLinearRange; // Vector pointing in the forward direction (i.e. local Z-axis)
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
		float angleIncrement = Resolution * Mathf.Deg2Rad; // Convert resolution to radians
		float initialAngle = (Head.transform.eulerAngles.y - MinimumAngularRange) * Mathf.Deg2Rad; // Initial angle

		// Precompute directions and store in an array
		Vector3[] laserRayDirections = new Vector3[MeasurementsPerScan];
		for (int i = 0; i < MeasurementsPerScan; i++)
		{
			float angle = initialAngle + (i * angleIncrement) * (MinimumAngularRange <= 0 ? -1 : 1);
			laserRayDirections[i] = new Vector3(Mathf.Sin(angle), 0, Mathf.Cos(angle)) * MaximumLinearRange;
		}

		// Perform raycasts
		RaycastHit[] hits = new RaycastHit[MeasurementsPerScan];
		for (int i = 0; i < MeasurementsPerScan; i++)
		{
			if (Physics.Raycast(Head.transform.position, laserRayDirections[i], out hits[i], MaximumLinearRange, layer_mask))
			{
				float hitDistance = hits[i].distance;
				RangeArray[i] = (hitDistance > MinimumLinearRange) ? hitDistance.ToString() : "inf"; // Store range result
			}
			else
			{
				RangeArray[i] = "inf"; // No hit
			}

			IntensityArray[i] = Intensity.ToString(); // Update intensity

			// Draw raycasts (if enabled) -- Only rendered in editor mode
			if (ShowRaycasts)
			{
				float range = Mathf.Min(hits[i].distance, MaximumLinearRange);
				Vector3 debugRayDirection = laserRayDirections[i] * (range / MaximumLinearRange);
				Color debugColor = (i == 0) ? Color.green : (i == (int)(MeasurementsPerScan / 2)) ? Color.blue : Color.red;
				Debug.DrawRay(Head.transform.position, debugRayDirection, debugColor, 1 / ScanRate);
			}
		}

		// Visualize laser scan (if enabled) -- Rendered in editor and standalone mode
		if (ShowLaserScan && HUD.activeSelf)
		{
			int[] indices = new int[hits.Length]; // Reset indices
			Vector3[] points = new Vector3[hits.Length]; // Reset points

			// Update indices and points based on raycast hits
			int idx = 0;
			for (int h = 0; h < hits.Length; h++)
			{
				if (hits[h].point != Vector3.zero)
				{
					indices[idx] = idx; // Ensure sequential indices
					points[idx] = hits[h].point; // Append valid hit points
					idx++; // Increment counter only when a valid point is found
				}
			}

			LaserScanMesh.Clear(); // Clear the previous mesh data
			LaserScanMesh.vertices = points; // Set new vertices
			LaserScanMesh.SetIndices(indices, MeshTopology.Points, 0); // Set indices using `Points` topology
			LaserScanMaterial.SetFloat("_PointSize", LaserScanSize); // Set point size for material shader
			LaserScanMaterial.SetColor("_PointColor", LaserScanColor); // Set point color for material shader

			Graphics.DrawMesh(LaserScanMesh, Vector3.zero, Quaternion.identity, LaserScanMaterial, visualizationLayerID);
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