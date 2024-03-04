using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AntiRollBar : MonoBehaviour {

	public WheelCollider WheelL;
	public WheelCollider WheelR;
	public float AntiRoll = 5000.0f;

	private Rigidbody car;

	void Start(){
		car = GetComponent<Rigidbody> ();
	}

	void FixedUpdate ()
	{
		WheelHit hit;
		float travelL = 1.0f;
		float travelR = 1.0f;


		bool groundedL = WheelL.GetGroundHit (out hit);
		if (groundedL) {
			travelL = (-WheelL.transform.InverseTransformPoint (hit.point).y - WheelL.radius) / WheelL.suspensionDistance;
		}

		bool groundedR = WheelR.GetGroundHit (out hit);
		if (groundedR) {
			travelR = (-WheelR.transform.InverseTransformPoint (hit.point).y - WheelR.radius) / WheelR.suspensionDistance;
		}

		float antiRollForce = (travelL - travelR) * AntiRoll;

		if (groundedL)
			car.AddForceAtPosition (WheelL.transform.up * -antiRollForce, WheelL.transform.position);

		if (groundedR)
			car.AddForceAtPosition (WheelR.transform.up * antiRollForce, WheelR.transform.position);
	}
}
