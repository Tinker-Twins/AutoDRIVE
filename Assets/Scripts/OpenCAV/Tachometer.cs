using UnityEngine;
using System.Collections;

public class Tachometer : MonoBehaviour
{

	public CarController cc;
	public float maxInput, minInput;
	public float correction;
	public float multiplier;
	private float input;
	public float rotationSpeed;

	// Use this for initialization
	void Start ()
	{

	}

	// Update is called once per frame
	void Update ()
	{
		input = Mathf.Abs (cc.engineRPM);
		input = Mathf.Clamp (input, minInput, cc.maxGearChangeRPM);

		float angle = 0;
		angle = Mathf.SmoothStep (angle, Mathf.Asin (input / maxInput) * multiplier, rotationSpeed);


		float crr = (cc.gearNum == 1 || cc.gearNum == -1) ? 25 : correction;
		angle += crr;

		Quaternion newRot = Quaternion.Euler (0, 0, angle);
		transform.rotation = Quaternion.RotateTowards (transform.rotation, newRot, rotationSpeed);
	}
}
